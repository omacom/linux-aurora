// SPDX-License-Identifier: GPL-2.0-only

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/list.h>
#include <linux/pm_domain.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>
#include "centauri_t6050.h"

struct cen_buffer {
	struct list_head list;
	struct device *owner;
	void *cpu;
	dma_addr_t dma;
	size_t size;
};

struct cen_context {
	struct cen_buffer *buffer, *messages;
	u32 index[4], mtr, mcr;
	u16 tr_count, cr_count, message_head;
};

struct centauri_t6050_ipc {
	struct list_head buffers;
	struct pci_dev *function[3];
	bool enabled[3], owned[3], domain_lease[3];
	bool alpha_pm_domain;
	void __iomem *bar[3];
	struct cen_context control, alpha;
	struct cen_buffer *secondary, *window, *cchi_rx, *cchi_tx, *debug_rx;
	u16 cchi_head, cchi_tail;
};

static struct cen_buffer *buffer_alloc(struct centauri_t6050 *c, unsigned int function,
				      size_t size)
{
	struct cen_buffer *buffer = kzalloc(sizeof(*buffer), GFP_KERNEL);

	if (!buffer)
		return NULL;
	buffer->owner = get_device(&c->ipc->function[function]->dev);
	buffer->size = PAGE_ALIGN(size);
	buffer->cpu = dma_alloc_coherent(buffer->owner, buffer->size, &buffer->dma, GFP_KERNEL);
	if (!buffer->cpu) {
		put_device(buffer->owner);
		kfree(buffer);
		return NULL;
	}
	memset(buffer->cpu, 0, buffer->size);
	list_add_tail(&buffer->list, &c->ipc->buffers);
	return buffer;
}

static int ftab_entry(const void *image, size_t size, const char tag[4],
		     u32 *offset, u32 *length, bool sparse)
{
	u32 count, i, off, len;
	const u8 *table = image;

	if (size < 0x30 || memcmp(table + 0x24, "ftab", 4))
		return -EINVAL;
	count = get_unaligned_le32(table + 0x28);
	if (!count || count > 256 || 0x30 + count * 16 > size)
		return -EINVAL;
	for (i = 0; i < count; i++) {
		const u8 *entry = table + 0x30 + 16 * i;

		if (memcmp(entry, tag, 4))
			continue;
		off = get_unaligned_le32(entry + 4);
		len = get_unaligned_le32(entry + 8);
		if (off < 0x30 + count * 16 || !len || (u64)off + len > (sparse ? SZ_64M : size))
			return -EINVAL;
		*offset = off;
		*length = len;
		return 0;
	}
	return -ENOENT;
}

static int secondary_init(struct centauri_t6050 *c)
{
	u32 offset, length, count, i, required;
	const u8 *seed;
	int ret;

	ret = ftab_entry(c->firmware, c->firmware_size, "2ftb", &offset, &length, false);
	if (ret)
		return ret;
	seed = c->firmware + offset;
	if (length < 0x30 || memcmp(seed + 0x24, "ftab", 4))
		return -EINVAL;
	count = get_unaligned_le32(seed + 0x28);
	if (!count || count > 256 || 0x30 + 16 * count != length)
		return -EINVAL;
	required = length;
	for (i = 0; i < count; i++) {
		u64 end = (u64)get_unaligned_le32(seed + 0x34 + i * 16) +
			  get_unaligned_le32(seed + 0x38 + i * 16);

		if (end > SZ_64M)
			return -EINVAL;
		required = max_t(u32, required, end);
	}
	c->ipc->secondary = buffer_alloc(c, 0, required);
	if (!c->ipc->secondary)
		return -ENOMEM;
	memcpy(c->ipc->secondary->cpu, seed, length);
	dma_wmb();
	ret = pci_write_config_dword(c->pdev, 0xf88, lower_32_bits(c->ipc->secondary->dma));
	if (!ret)
		ret = pci_write_config_dword(c->pdev, 0xf8c, upper_32_bits(c->ipc->secondary->dma));
	if (!ret)
		ret = pci_write_config_dword(c->pdev, 0xf90, required);
	if (ret)
		return pcibios_err_to_errno(ret);

	return 0;
}

static int context_init(struct centauri_t6050 *c, struct cen_context *ctx,
			unsigned int function, u16 tr, u16 cr, dma_addr_t scratch, size_t scratch_size)
{
	u8 *header;
	dma_addr_t dma;
	int i;

	ctx->buffer = buffer_alloc(c, function, SZ_32K);
	ctx->messages = buffer_alloc(c, function, SZ_16K);
	if (!ctx->buffer || !ctx->messages)
		return -ENOMEM;
	ctx->tr_count = tr;
	ctx->cr_count = cr;
	ctx->index[0] = 0x78;
	ctx->index[1] = ctx->index[0] + cr * 2;
	ctx->index[2] = ctx->index[1] + tr * 2;
	ctx->index[3] = ctx->index[2] + cr * 2;
	ctx->mtr = ALIGN(ctx->index[3] + tr * 2, tr == 192 ? 16 : 2);
	ctx->mcr = ALIGN(ctx->mtr + 256, 4);
	header = ctx->buffer->cpu;
	dma = ctx->buffer->dma;
	put_unaligned_le16(768, header);
	put_unaligned_le16(0x68, header + 2);
	put_unaligned_le64(dma + 0x68, header + 8);
	for (i = 0; i < 4; i++)
		put_unaligned_le64(dma + ctx->index[i], header + 0x10 + 8 * i);
	put_unaligned_le16(cr, header + 0x30);
	put_unaligned_le16(tr, header + 0x32);
	put_unaligned_le64(dma + ctx->mcr, header + 0x34);
	put_unaligned_le64(dma + ctx->mtr, header + 0x3c);
	put_unaligned_le16(16, header + 0x44);
	put_unaligned_le16(16, header + 0x46);
	put_unaligned_le64(scratch ?: dma + ALIGN(ctx->mcr + 256, 8), header + 0x58);
	put_unaligned_le64(scratch_size, header + 0x60);
	dma_wmb();
	return 0;
}

static int handshake(void __iomem *bar, struct cen_context *ctx,
		     u32 status, u32 address, u32 control)
{
	u32 value;
	int ret;

	writel(1, bar + control);
	ret = readl_poll_timeout(bar + status, value, value == 1, 100, 2000000);
	if (ret) {
		pr_err("centauri_t6050: IPC init timeout status@%#x=%#x control@%#x=%#x context=%pad\n",
		       status, value, control, readl(bar + control), &ctx->buffer->dma);
		return ret;
	}
	writel(lower_32_bits(ctx->buffer->dma), bar + address);
	writel(upper_32_bits(ctx->buffer->dma), bar + address + 4);
	dma_wmb();
	writel(2, bar + control);
	ret = readl_poll_timeout(bar + status, value, value == 2, 100, 2000000);
	if (ret)
		pr_err("centauri_t6050: IPC run timeout status@%#x=%#x control@%#x=%#x context=%pad\n",
		       status, value, control, readl(bar + control), &ctx->buffer->dma);
	return ret;
}

static int message(struct cen_context *ctx, void __iomem *doorbell,
		   const void *payload, size_t length)
{
	u16 head = ctx->message_head;
	u8 *descriptor;
	u16 tail;
	int ret;

	if (head >= 15 || !length || length > 512)
		return -ENOSPC;
	memcpy(ctx->messages->cpu + 512 * (head + 1), payload, length);
	descriptor = ctx->buffer->cpu + ctx->mtr + head * 16;
	put_unaligned_le32(1 | (length << 8), descriptor);
	put_unaligned_le64(ctx->messages->dma + 512 * (head + 1), descriptor + 4);
	put_unaligned_le32(head + 1, descriptor + 12);
	dma_wmb();
	writel(head + 1, doorbell);
	ret = read_poll_timeout(READ_ONCE, tail, le16_to_cpu(tail) == head + 1,
		100, 1200000, false, *(__le16 *)(ctx->buffer->cpu + ctx->index[1]));
	dma_rmb();
	ctx->message_head++;
	return ret;
}

static int open_transfer(struct centauri_t6050 *c, u16 id, struct cen_buffer *buffer,
			 u16 footer)
{
	u8 msg[0x34] = { 1 };
	u32 words = footer >> 2, shift = min_t(u32, __ffs(words), 4);

	msg[2] = words >> shift;
	msg[3] = shift << 4;
	put_unaligned_le16(id, msg + 4);
	put_unaligned_le64(buffer->dma, msg + 8);
	put_unaligned_le64(~0ULL, msg + 0x10);
	put_unaligned_le16(16, msg + 0x18);
	put_unaligned_le16(0xffff, msg + 0x1a);
	put_unaligned_le16(id, msg + 0x1c);
	put_unaligned_le16(0x40, msg + 0x1e);
	return message(&c->ipc->control, c->bar + 0x9000, msg, sizeof(msg));
}

static int cchi_init(struct centauri_t6050 *c)
{
	struct centauri_t6050_ipc *ipc = c->ipc;
	u8 config[4] = {9, 0, 0xff, 0xff}, *entry;
	u16 tail;
	int i, ret;

	ipc->cchi_rx = buffer_alloc(c, 0, SZ_32K);
	ipc->cchi_tx = buffer_alloc(c, 0, SZ_32K);
	ipc->debug_rx = buffer_alloc(c, 0, SZ_16K);
	if (!ipc->cchi_rx || !ipc->cchi_tx || !ipc->debug_rx)
		return -ENOMEM;
	ret = message(&ipc->control, c->bar + 0x9000, config, sizeof(config));
	if (ret)
		return ret;
	ret = open_transfer(c, 4, ipc->cchi_rx, 1024);
	if (!ret)
		ret = open_transfer(c, 3, ipc->cchi_tx, 1024);
	if (!ret)
		ret = open_transfer(c, 5, ipc->debug_rx, 256);
	if (ret)
		return ret;
	for (i = 0; i < 15; i++) {
		entry = ipc->cchi_rx->cpu + i * 1040;
		put_unaligned_le32(1 | (1024 << 8), entry);
		put_unaligned_le32(i + 1, entry + 12);
		entry = ipc->debug_rx->cpu + i * 272;
		put_unaligned_le32(1 | (256 << 8), entry);
		put_unaligned_le32(i + 1, entry + 12);
	}
	dma_wmb();
	writel(15, c->bar + 0x9010);
	writel(15, c->bar + 0x9014);
	entry = ipc->cchi_tx->cpu;
	put_unaligned_le32(1 | (64 << 8), entry);
	put_unaligned_le32(1, entry + 12);
	put_unaligned_le16(64, entry + 16);
	dma_wmb();
	writel(1, c->bar + 0x900c);
	ret = read_poll_timeout(READ_ONCE, tail, le16_to_cpu(tail) != 0,
		100, 2000000, false,
		*(__le16 *)(ipc->control.buffer->cpu + ipc->control.index[1] + 4 * 2));
	dma_rmb();
	if (ret)
		return ret;
	entry = ipc->cchi_rx->cpu;
	if ((get_unaligned_le32(entry) >> 8) < 8 || entry[18] || entry[19])
		return -EPROTO;
	ipc->cchi_head = ipc->cchi_tail = 1;
	return 0;
}

static int memswap_handoff(struct centauri_t6050 *c, unsigned int function, const char tag[4])
{
	struct cen_buffer *secondary = c->ipc->secondary, *working;
	u32 offset, size;
	int ret;

	ret = ftab_entry(secondary->cpu, secondary->size, tag, &offset, &size, false);
	if (ret)
		return ret;
	working = buffer_alloc(c, function, size);
	if (!working)
		return -ENOMEM;
	dma_rmb();
	memcpy(working->cpu, secondary->cpu + offset, size);
	dma_wmb();
	ret = pci_write_config_dword(c->ipc->function[function], 0xf88, lower_32_bits(working->dma));
	if (!ret)
		ret = pci_write_config_dword(c->ipc->function[function], 0xf8c, upper_32_bits(working->dma));
	if (!ret)
		ret = pci_write_config_dword(c->ipc->function[function], 0xf90, size);
	if (ret)
		return pcibios_err_to_errno(ret);
	return 0;
}

static struct dev_pm_domain centauri_t6050_alpha_pm_domain;

static int enumerate_functions(struct centauri_t6050 *c)
{
	struct centauri_t6050_ipc *ipc = c->ipc;
	int i, ret = 0;

	c->pdev->multifunction = 1;
	pci_rescan_bus(c->pdev->bus);
	for (i = 0; i < 3; i++) {
		ipc->function[i] = pci_get_domain_bus_and_slot(pci_domain_nr(c->pdev->bus),
				c->pdev->bus->number, PCI_DEVFN(PCI_SLOT(c->pdev->devfn), i));
		if (!ipc->function[i] || ipc->function[i]->vendor != PCI_VENDOR_ID_APPLE ||
		    (i && ipc->function[i]->device != 0x1901 + i))
			return -ENODEV;
		if (ipc->function[i]->cfg_size < 0xf94 ||
		    !(pci_resource_flags(ipc->function[i], 0) & IORESOURCE_MEM) ||
		    pci_resource_len(ipc->function[i], 0) < (i ? 0x2000 : 0x9080))
			return -ENODEV;
		if (i) {
			/*
			 * The driver core sets dev.driver before calling probe.
			 * Beta can be probing only to defer until c->ready.
			 * Wait for that probe before deciding it is owned.
			 */
			device_lock(&ipc->function[i]->dev);
			if (ipc->function[i]->dev.driver) {
				ret = -EBUSY;
				goto unlock;
			}
		}
		/* F0 already has the independent control owner's domain lease. */
		if (i) {
			ret = iommu_device_use_default_domain(&ipc->function[i]->dev);
			if (ret)
				goto unlock;
			ipc->domain_lease[i] = true;
		}
		ipc->owned[i] = true;
		ipc->bar[i] = pci_iomap(ipc->function[i], 0, 0);
		if (!ipc->bar[i]) {
			ret = -ENOMEM;
			goto unlock;
		}
		if (i) {
			ret = pci_enable_device_mem(ipc->function[i]);
			if (ret)
				goto unlock;
			ipc->enabled[i] = true;
		}
		if (i == 1) {
			dev_pm_domain_set(&ipc->function[i]->dev, &centauri_t6050_alpha_pm_domain);
			ipc->alpha_pm_domain = true;
		}
		ret = dma_set_mask_and_coherent(&ipc->function[i]->dev, DMA_BIT_MASK(40));
		if (ret)
			goto unlock;
		if (i == 1) {
			ret = centauri_t6050_request_irqs(&c->alpha_irqs, ipc->function[i],
						    16, centauri_t6050_alpha_irq, c);
			if (ret)
				goto unlock;
		}
		pci_set_master(ipc->function[i]);
unlock:
		if (i)
			device_unlock(&ipc->function[i]->dev);
		if (ret)
			return ret;
	}
	return 0;
}

static int centauri_t6050_start_ipc_locked(struct centauri_t6050 *c)
{
	struct centauri_t6050_ipc *ipc;
	u32 stage;
	int ret;

	if (!c->firmware || !c->link_ready || c->ipc || readl(c->bar + 0x8000) != 1)
		return -EBUSY;
	ipc = kzalloc(sizeof(*ipc), GFP_KERNEL);
	if (!ipc)
		return -ENOMEM;
	INIT_LIST_HEAD(&ipc->buffers);
	c->ipc = ipc;
	ret = enumerate_functions(c);
	if (ret)
		goto failed;
	ret = secondary_init(c);
	if (ret)
		goto failed;
	ret = context_init(c, &ipc->control, 0, 13, 1, 0, 0);
	if (ret)
		goto failed;
	writel(lower_32_bits(c->descriptor_dma), c->bar + 0x8044);
	writel(upper_32_bits(c->descriptor_dma), c->bar + 0x8048);
	writel(0x20, c->bar + 0x804c);
	writel(0x62746632, c->bar + 0x8074);
	dma_wmb();
	writel(1, c->bar + 0x9070);
	ret = readl_poll_timeout(c->bar + 0x8000, stage, stage == 2 || stage == 3, 100, 3000000);
	if (ret || stage != 2) {
		ret = ret ?: -EIO;
		goto failed;
	}
	ret = handshake(c->bar, &ipc->control, 0x8050, 0x8054, 0x907c);
	if (ret)
		goto failed;
	ret = memswap_handoff(c, 0, "mswc");
	if (!ret)
		ret = memswap_handoff(c, 2, "mswb");
	if (!ret)
		ret = memswap_handoff(c, 1, "msww");
	if (ret)
		goto failed;
	ret = cchi_init(c);
	if (ret)
		goto failed;
	ret = readl_poll_timeout(ipc->bar[1] + 8, stage, stage == 2 || stage == 3, 100, 2000000);
	if (ret || stage != 2) {
		ret = ret ?: -EIO;
		goto failed;
	}
	ipc->window = buffer_alloc(c, 1, CEN_ALPHA_WINDOW_SIZE);
	if (!ipc->window) {
		ret = -ENOMEM;
		goto failed;
	}
	ret = context_init(c, &ipc->alpha, 1, 192, 234,
		ipc->window->dma + CEN_ALPHA_SCRATCH_OFFSET, CEN_ALPHA_SCRATCH_SIZE);
	if (ret)
		goto failed;
	writel(lower_32_bits(ipc->window->dma), ipc->bar[1] + 0x18);
	writel(upper_32_bits(ipc->window->dma), ipc->bar[1] + 0x1c);
	writel(CEN_ALPHA_WINDOW_SIZE, ipc->bar[1] + 0x20);
	ret = handshake(ipc->bar[1], &ipc->alpha, 0xc, 0x10, 0x1028);
	if (ret)
		goto failed;
	ret = centauri_t6050_wifi_register(c);
	if (ret)
		return ret;

	/* Publish usable shared firmware before the Beta consumer probes. */
	smp_store_release(&c->ready, true);
	ret = device_attach(&ipc->function[2]->dev);
	if (ret < 0)
		dev_warn(&c->pdev->dev, "Beta driver attachment failed: %d\n", ret);
	return 0;
failed:
	dev_err(&c->pdev->dev, "IPC bootstrap failed: %d (Control stage=%#x)\n",
		ret, readl(c->bar + 0x8000));
	if (ipc->bar[1])
		dev_err(&c->pdev->dev, "Alpha stage=%#x IPC=%#x window=%#x/%#x/%#x\n",
			readl(ipc->bar[1] + 8), readl(ipc->bar[1] + 0xc),
			readl(ipc->bar[1] + 0x18), readl(ipc->bar[1] + 0x1c), readl(ipc->bar[1] + 0x20));
	if (ipc->bar[2])
		dev_err(&c->pdev->dev, "Beta stage=%#x IPC=%#x\n",
			readl(ipc->bar[2] + 0x18), readl(ipc->bar[2] + 0x1c));

	return ret;
}

int centauri_t6050_start_ipc(struct centauri_t6050 *c)
{
	int ret;

	if (!pci_trylock_rescan_remove())
		return -EAGAIN;
	ret = READ_ONCE(c->removing) ? -ECANCELED : centauri_t6050_start_ipc_locked(c);
	pci_unlock_rescan_remove();
	return ret;
}

void centauri_t6050_free_ipc(struct centauri_t6050 *c)
{
	struct centauri_t6050_ipc *ipc = c->ipc;

	if (!ipc)
		return;
	centauri_t6050_wifi_unregister(c);
	centauri_t6050_free_irqs(&c->alpha_irqs);
	if (!centauri_t6050_ipc_fenced(ipc)) {
		dev_warn(&c->pdev->dev, "retaining IPC DMA after pending transaction timeout\n");
		return;
	}
	centauri_t6050_release_ipc(ipc);
	c->ipc = NULL;
}

bool centauri_t6050_ipc_fenced(struct centauri_t6050_ipc *ipc)
{
	bool stopped = true;
	int i;

	if (!ipc)
		return true;
	for (i = 0; i < 3; i++)
		if (ipc->owned[i])
			pci_clear_master(ipc->function[i]);
	for (i = 0; i < 3; i++)
		if (ipc->owned[i] && !centauri_t6050_stop_dma(ipc->function[i]))
			stopped = false;
	return stopped;
}

/* Discard every pointer into the unbinding control driver's executable lifetime. */
void centauri_t6050_detach_ipc(struct centauri_t6050_ipc *ipc)
{
	int i;

	if (!ipc)
		return;
	if (ipc->alpha_pm_domain) {
		dev_pm_domain_set(&ipc->function[1]->dev, NULL);
		ipc->alpha_pm_domain = false;
	}
	for (i = 0; i < 3; i++) {
		if (ipc->bar[i])
			pci_iounmap(ipc->function[i], ipc->bar[i]);
		ipc->bar[i] = NULL;
		if (ipc->enabled[i])
			pci_disable_device(ipc->function[i]);
		ipc->enabled[i] = false;
	}
}

/* IRQ users must be drained and every function fenced before this release. */
void centauri_t6050_release_ipc(struct centauri_t6050_ipc *ipc)
{
	struct cen_buffer *buffer, *next;
	int i;

	if (!ipc)
		return;
	list_for_each_entry_safe(buffer, next, &ipc->buffers, list) {
		memzero_explicit(buffer->cpu, buffer->size);
		dma_free_coherent(buffer->owner, buffer->size, buffer->cpu, buffer->dma);
		put_device(buffer->owner);
		list_del(&buffer->list);
		kfree(buffer);
	}
	centauri_t6050_detach_ipc(ipc);
	for (i = 0; i < 3; i++) {
		if (ipc->domain_lease[i]) {
			device_lock(&ipc->function[i]->dev);
			iommu_device_unuse_default_domain(&ipc->function[i]->dev);
			device_unlock(&ipc->function[i]->dev);
		}
		pci_dev_put(ipc->function[i]);
	}
	kfree(ipc);
}

int centauri_t6050_alpha_resources(struct centauri_t6050 *c, struct cen_alpha_resources *resources)
{
	struct centauri_t6050_ipc *ipc = c->ipc;

	if (!ipc || !ipc->window || !ipc->alpha.buffer || !ipc->alpha.messages)
		return -ENODEV;
	resources->pdev = ipc->function[1];
	resources->context = ipc->alpha.buffer->cpu;
	resources->context_dma = ipc->alpha.buffer->dma;
	resources->messages = ipc->alpha.messages->cpu;
	resources->messages_dma = ipc->alpha.messages->dma;
	resources->window = ipc->window->cpu;
	resources->window_dma = ipc->window->dma;
	resources->bar = ipc->bar[1];
	memcpy(resources->index, ipc->alpha.index, sizeof(resources->index));
	resources->mtr = ipc->alpha.mtr;
	resources->mcr = ipc->alpha.mcr;
	return 0;
}

struct pci_dev *centauri_t6050_ipc_alpha_dev(struct centauri_t6050 *c)
{
	return c->ipc ? c->ipc->function[1] : NULL;
}

static u32 control_sleep_status(struct centauri_t6050 *c)
{
	return le32_to_cpu(READ_ONCE(*(__le32 *)(c->ipc->control.buffer->cpu + 0x70)));
}

int centauri_t6050_ipc_control_sleep(struct centauri_t6050 *c, bool asleep)
{
	u32 status;
	int ret;

	if (!c->ipc || !c->ipc->control.buffer ||
	    readl(c->bar + 0x8000) != 2 || readl(c->bar + 0x8050) != 2)
		return -EIO;
	dma_wmb();
	writel(asleep ? 3 : 0, c->bar + 0x9078);
	ret = read_poll_timeout(control_sleep_status, status,
				(asleep ? status == 3 : status <= 1) ||
				readl(c->bar + 0x8000) != 2,
				100, 2000000, false, c);
	dma_rmb();
	if (readl(c->bar + 0x8000) != 2)
		ret = -EIO;
	dev_dbg(&c->pdev->dev,
		"Control ACIPC host %s: ret=%d status=%u exec=%u ipc=%u\n",
		asleep ? "sleep" : "wake", ret, status,
		readl(c->bar + 0x8000), readl(c->bar + 0x8050));
	return ret;
}
