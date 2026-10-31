// SPDX-License-Identifier: GPL-2.0
/*
 * Apple ANS NVM Express device driver
 * Copyright The Asahi Linux Contributors
 *
 * Based on the pci.c NVM Express device driver
 * Copyright (c) 2011-2014, Intel Corporation.
 * and on the rdma.c NVMe over Fabrics RDMA host code.
 * Copyright (c) 2015-2016 HGST, a Western Digital Company.
 */

#include <linux/async.h>
#include <linux/blk-crypto.h>
#include <linux/blk-crypto-profile.h>
#include <linux/blkdev.h>
#include <linux/blk-mq.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmapool.h>
#include <linux/interrupt.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/kref.h>
#include <linux/mempool.h>
#include <linux/mm.h>
#include <linux/overflow.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm_runtime.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/once.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/soc/apple/rtkit.h>
#include <linux/soc/apple/sart.h>
#include <linux/reset.h>
#include <linux/time64.h>

#include "nvme.h"

#define APPLE_ANS_BOOT_TIMEOUT	  USEC_PER_SEC

#define APPLE_ANS_COPROC_CPU_CONTROL	 0x44
#define APPLE_ANS_COPROC_CPU_CONTROL_RUN BIT(4)

#define APPLE_ANS_ACQ_DB  0x1004
#define APPLE_ANS_IOCQ_DB 0x100c

#define APPLE_ANS_IOSQ_REGISTER      0x1200
#define APPLE_ANS_IOCQ_REGISTER      0x1208
#define APPLE_ANS_MAX_PEND_CMDS_CTRL 0x1210

#define APPLE_ANS_BOOT_STATUS	 0x1300
#define APPLE_ANS_BOOT_STATUS_OK 0xde71ce55

#define APPLE_ANS_LINEAR_SQ_CTRL 0x24908
#define APPLE_ANS_LINEAR_SQ_EN	 BIT(0)

#define APPLE_ANS_LINEAR_ASQ_DB	 0x2490c
#define APPLE_ANS_LINEAR_IOSQ_DB 0x24910

#define APPLE_NVMMU_NUM_TCBS	  0x28100
#define APPLE_NVMMU_ASQ_TCB_BASE  0x28108
#define APPLE_NVMMU_IOSQ_TCB_BASE 0x28110
#define APPLE_NVMMU_TCB_INVAL	  0x28118
#define APPLE_NVMMU_TCB_STAT	  0x28120

/*
 * This controller is a bit weird in the way command tags works: Both the
 * admin and the IO queue share the same tag space. Additionally, tags
 * cannot be higher than 0x40 which effectively limits the combined
 * queue depth to 0x40. Instead of wasting half of that on the admin queue
 * which gets much less traffic we instead reduce its size here.
 * The controller also doesn't support async event such that no space must
 * be reserved for NVME_NR_AEN_COMMANDS.
 */
#define APPLE_NVME_AQ_DEPTH	   2
#define APPLE_NVME_AQ_MQ_TAG_DEPTH (APPLE_NVME_AQ_DEPTH - 1)

#define APPLE_NVME_IOSQES	7

/*
 * These can be higher, but we need to ensure that any command doesn't
 * require an sg allocation that needs more than a page of data.
 */
#define NVME_MAX_KB_SZ 4096
#define NVME_MAX_SEGS  127

/*
 * This controller comes with an embedded IOMMU known as NVMMU.
 * The NVMMU is pointed to an array of TCBs indexed by the command tag.
 * Each command must be configured inside this structure before it's allowed
 * to execute, including commands that don't require DMA transfers.
 *
 * An exception to this are Apple's vendor-specific commands (opcode 0xD8 on the
 * admin queue): Those commands must still be added to the NVMMU but the DMA
 * buffers cannot be represented as PRPs and must instead be allowed using SART.
 *
 * Programming the PRPs to the same values as those in the submission queue
 * looks rather silly at first. This hardware is however designed for a kernel
 * that runs the NVMMU code in a higher exception level than the NVMe driver.
 * In that setting the NVMe driver first programs the submission queue entry
 * and then executes a hypercall to the code that is allowed to program the
 * NVMMU. The NVMMU driver then creates a shadow copy of the PRPs while
 * verifying that they don't point to kernel text, data, pagetables, or similar
 * protected areas before programming the TCB to point to this shadow copy.
 * Since Linux doesn't do any of that we may as well just point both the queue
 * and the TCB PRP pointer to the same memory.
 */
struct apple_nvmmu_tcb {
	u8 opcode;

#define APPLE_ANS_TCB_DMA_FROM_DEVICE BIT(0)
#define APPLE_ANS_TCB_DMA_TO_DEVICE   BIT(1)
#define APPLE_ANS_TCB_AES             BIT(2)
	u8 dma_flags;

	u8 command_id;
	u8 _unk0;
	__le16 length;
	u8 _unk1[18];
	__le64 prp1;
	__le64 prp2;
	u8 _unk2[16];
	u8 aes_iv[8];
	u8 _aes_unk[64];
};

/*
 * The Apple NVMe controller only supports a single admin and a single IO queue
 * which are both limited to 64 entries and share a single interrupt.
 *
 * The completion queue works as usual. The submission "queue" instead is
 * an array indexed by the command tag on this hardware. Commands must also be
 * present in the NVMMU's tcb array. They are triggered by writing their tag to
 * a MMIO register.
 */
struct apple_nvme_queue {
	struct nvme_command *sqes;
	struct nvme_completion *cqes;
	struct apple_nvmmu_tcb *tcbs;

	dma_addr_t sq_dma_addr;
	dma_addr_t cq_dma_addr;
	dma_addr_t tcb_dma_addr;

	u32 __iomem *sq_db;
	u32 __iomem *cq_db;

	u16 sq_tail;
	u16 cq_head;
	u8 cq_phase;

	bool is_adminq;
	bool enabled;
};

static inline bool apple_nvme_queue_enabled(struct apple_nvme_queue *q)
{
	/* Pair with apple_nvme_enable_queue(). */
	return smp_load_acquire(&q->enabled);
}

static inline void apple_nvme_enable_queue(struct apple_nvme_queue *q)
{
	/* Publish queue initialization before setting q->enabled. */
	smp_store_release(&q->enabled, true);
}

static inline void apple_nvme_disable_queue(struct apple_nvme_queue *q)
{
	WRITE_ONCE(q->enabled, false);
}

/*
 * The apple_nvme_iod describes the data in an I/O.
 *
 * The sg pointer contains the list of PRP chunk allocations in addition
 * to the actual struct scatterlist.
 */
struct apple_nvme_iod {
	struct nvme_request req;
	struct nvme_command cmd;
	struct apple_nvme_queue *q;
	int npages; /* In the PRP list. 0 means small pool in use */
	int nents; /* Used in scatterlist */
	dma_addr_t first_dma;
	unsigned int dma_len; /* length of single DMA segment mapping */
	struct scatterlist *sg;
};

struct apple_nvme_hw {
	bool has_lsq_nvmmu;
	bool has_queue_count;
	bool has_separate_nvmmu;
	bool needs_ioq_registers;
	/* T8152: LSQ_CTRL and IOQA accept writes but read back 0. */
	bool no_ioq_readback;
	/* T8152: the queue setup registers live at different offsets. */
	bool t8152_reg_layout;
	/* T8152: the ANS is a virtual PMGR device with no PS register, so there is no reset line. */
	bool no_reset_control;
	u32 max_queue_depth;
	unsigned long quirks;
};

struct apple_nvme {
	struct device *dev;
	struct kref ref;
	/* Serializes the post-M4 controller disable with reset and teardown. */
	struct mutex disable_lock;
	/* Protects reset_ready against RTKit crash callbacks. */
	spinlock_t event_lock;
	bool reset_ready;
	struct list_head quarantine_node;
	resource_size_t controller_start;
	bool quarantined;
	bool detached;
	bool removing;
	bool abort_reset;
	bool recover_to_live;
	bool suspended;
	unsigned long recovery_flags;

	void __iomem *mmio_coproc;
	void __iomem *mmio_nvme;
	void __iomem *mmio_nvmmu;
	const struct apple_nvme_hw *hw;

	struct device **pd_dev;
	struct device_link **pd_link;
	int pd_count;

	struct apple_sart *sart;
	struct apple_rtkit *rtk;
	struct reset_control *reset;
	bool owns_rtkit;
	bool inherited_rtkit;

	struct dma_pool *prp_page_pool;
	struct dma_pool *prp_small_pool;
	mempool_t *iod_mempool;

	struct nvme_ctrl ctrl;
	struct work_struct remove_work;
	struct work_struct recovery_work;

	struct apple_nvme_queue adminq;
	struct apple_nvme_queue ioq;

	struct blk_mq_tag_set admin_tagset;
	struct blk_mq_tag_set tagset;

	int irq;
	spinlock_t lock;

	/*
	 * Delayed cache flush handling state
	 */
	struct nvme_ns *flush_ns;
	unsigned long flush_interval;
	unsigned long last_flush;
	struct delayed_work flush_dwork;
	struct blk_crypto_profile crypto_profile;
};

/* Map a legacy ANS register offset to the T8152 layout: admin queue +0x4000, IOQ registers +0x4000,
 * LSQ_CTRL 0x24908 -> 0x2813c. */
static u32 apple_nvme_reg(const struct apple_nvme *anv, u32 off)
{
	if (!anv->hw->t8152_reg_layout)
		return off;
	switch (off) {
	case NVME_REG_AQA:
	case NVME_REG_ASQ:
	case NVME_REG_ACQ:
	case APPLE_ANS_IOSQ_REGISTER:
	case APPLE_ANS_IOCQ_REGISTER:
	case APPLE_ANS_MAX_PEND_CMDS_CTRL:
		return off + 0x4000;
	case APPLE_ANS_LINEAR_SQ_CTRL:
		return 0x2813c;
	default:
		return off;
	}
}

/* Bit in recovery_flags: recovery_work owns the next controller stop. */
#define APPLE_NVME_RECOVERY_PENDING 0

/*
 * Controllers that failed to stop. Unknown DMA ownership survives driver
 * unbind; never admit the same controller again until the platform has
 * reset. Entries retain their owner.
 */
static DEFINE_MUTEX(apple_nvme_quarantine_lock);
static LIST_HEAD(apple_nvme_quarantines);

static inline void apple_nvme_writeq(struct apple_nvme *anv, u64 value,
				     void __iomem *addr)
{
	/*
	 * Post-M4 ANS consumes the queue and NVMMU TCB base addresses as paired
	 * 32-bit registers. Match the access sequence used by Apple firmware,
	 * m1n1, and U-Boot; older ANS generations retain their native 64-bit
	 * access.
	 */
	if (anv->hw->needs_ioq_registers)
		lo_hi_writeq(value, addr);
	else
		writeq(value, addr);
}

unsigned int flush_interval = 1000;
module_param(flush_interval, uint, 0644);
MODULE_PARM_DESC(flush_interval,
		 "Grace period in msecs between flushes (ignored on post-M4 ANS)");

static_assert(sizeof(struct nvme_command) == 64);
static_assert(sizeof(struct apple_nvmmu_tcb) == 128);

#define APPLE_NVME_CRYPTO_KEY_SIZE 64
#define APPLE_NVME_CRYPTO_DATA_UNIT_SIZE 4096

static inline struct apple_nvme *ctrl_to_apple_nvme(struct nvme_ctrl *ctrl)
{
	return container_of(ctrl, struct apple_nvme, ctrl);
}

/*
 * The controller's character device can outlive platform-driver unbind.
 * Keep the container until both devres and the last NVMe reference are gone.
 */
static void apple_nvme_release(struct kref *ref)
{
	struct apple_nvme *anv = container_of(ref, struct apple_nvme, ref);

	put_device(anv->dev);
	kfree(anv);
}

static void apple_nvme_put_resource_owner(void *data)
{
	struct apple_nvme *anv = data;

	WRITE_ONCE(anv->detached, true);
	if (!READ_ONCE(anv->quarantined))
		kref_put(&anv->ref, apple_nvme_release);
}

static bool apple_nvme_can_adopt_rtkit(struct apple_nvme *anv)
{
	u32 cc, csts;

	if (!anv->hw->needs_ioq_registers ||
	    !(readl(anv->mmio_coproc + APPLE_ANS_COPROC_CPU_CONTROL) &
	      APPLE_ANS_COPROC_CPU_CONTROL_RUN) ||
	    readl(anv->mmio_nvme + APPLE_ANS_BOOT_STATUS) != APPLE_ANS_BOOT_STATUS_OK)
		return false;
	cc = readl(anv->mmio_nvme + NVME_REG_CC);
	csts = readl(anv->mmio_nvme + NVME_REG_CSTS);

	/*
	 * CPU_RUN and BOOT_STATUS alone survive a session that has gone
	 * inactive. Only a controller that the previous owner left enabled
	 * and ready identifies a live session. Borrowed buffers still require
	 * reserved RAM and protected SART ranges.
	 */
	return (cc & NVME_CC_ENABLE) && (csts & NVME_CSTS_RDY) &&
	       !(csts & NVME_CSTS_CFS);
}

static inline struct apple_nvme *queue_to_apple_nvme(struct apple_nvme_queue *q)
{
	if (q->is_adminq)
		return container_of(q, struct apple_nvme, adminq);

	return container_of(q, struct apple_nvme, ioq);
}

static unsigned int apple_nvme_queue_depth(struct apple_nvme_queue *q)
{
	struct apple_nvme *anv = queue_to_apple_nvme(q);

	if (q->is_adminq)
		return APPLE_NVME_AQ_DEPTH;

	return anv->hw->max_queue_depth;
}

static void apple_nvme_set_reset_ready(struct apple_nvme *anv, bool ready)
{
	unsigned long flags;

	spin_lock_irqsave(&anv->event_lock, flags);
	anv->reset_ready = ready;
	spin_unlock_irqrestore(&anv->event_lock, flags);
}

static void apple_nvme_rtkit_crashed(void *cookie, const void *crashlog, size_t crashlog_size)
{
	struct apple_nvme *anv = cookie;
	unsigned long flags;

	if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->removing))
		return;
	dev_warn(anv->dev, "RTKit crashed; unable to recover without a reboot");
	/*
	 * RX may run inside RTKit initialization, before ctrl/admin_q exist.
	 * Probe's initial reset observes RTKit's latched crash state. Serialize
	 * callback scheduling with closing this gate before work cancellation.
	 */
	spin_lock_irqsave(&anv->event_lock, flags);
	if (anv->reset_ready && !READ_ONCE(anv->quarantined) &&
	    !READ_ONCE(anv->removing))
		nvme_reset_ctrl(&anv->ctrl);
	spin_unlock_irqrestore(&anv->event_lock, flags);
}

static bool apple_nvme_in_inherited_rtkit_range(struct apple_nvme *anv,
						struct apple_rtkit_shmem *bfr)
{
	phys_addr_t last;
	unsigned long pfn, last_pfn;

	if (!anv->inherited_rtkit || !bfr->size ||
	    check_add_overflow((phys_addr_t)bfr->iova,
			       (phys_addr_t)bfr->size - 1, &last) ||
	    !IS_ALIGNED(bfr->iova, dma_get_cache_alignment()) ||
	    !IS_ALIGNED(bfr->size, dma_get_cache_alignment()) ||
	    !apple_sart_is_inherited_region(anv->sart, bfr->iova, bfr->size))
		return false;

	/*
	 * A range property or a partial System RAM intersection does not prove
	 * ownership. Every page must be reserved and present in the linear map,
	 * as the bootloader leaves the buffers it hands over.
	 */
	if (region_intersects(bfr->iova, bfr->size, IORESOURCE_SYSTEM_RAM,
			      IORES_DESC_NONE) != REGION_INTERSECTS)
		return false;
	last_pfn = PHYS_PFN(last);
	for (pfn = PHYS_PFN(bfr->iova); pfn <= last_pfn; pfn++) {
		struct page *page;

		if (!pfn_valid(pfn))
			return false;
		page = pfn_to_page(pfn);
		if (!PageReserved(page) || PageHighMem(page) ||
		    !virt_addr_valid(page_to_virt(page)))
			return false;
	}
	return true;
}

/* A retained DMA mapping must also retain its device context. */
struct apple_nvme_rtkit_mapping {
	struct device *dev;
};

static int apple_nvme_sart_dma_setup(void *cookie,
				     struct apple_rtkit_shmem *bfr)
{
	struct apple_nvme *anv = cookie;
	int ret;

	if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->removing))
		return -ESHUTDOWN;
	if (!bfr->size)
		return -EINVAL;
	if (bfr->iova) {
		struct apple_nvme_rtkit_mapping *mapping;
		dma_addr_t dma;
		void *buffer;

		if (!apple_nvme_in_inherited_rtkit_range(anv, bfr))
			return -EINVAL;
		mapping = kzalloc_obj(*mapping);
		if (!mapping)
			return -ENOMEM;
		buffer = memremap(bfr->iova, bfr->size, MEMREMAP_WB);
		if (!buffer) {
			kfree(mapping);
			return -ENOMEM;
		}

		/*
		 * Firmware keeps the old identity IOVA. A bounce/remapped address
		 * cannot replace it. Skip synchronization until that is verified,
		 * and never copy a rejected bounce buffer over the live data.
		 */
		dma = dma_map_single_attrs(anv->dev, buffer, bfr->size,
					   DMA_FROM_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
		if (dma_mapping_error(anv->dev, dma)) {
			ret = -ENOMEM;
			goto unmap_buffer;
		}
		if (dma != bfr->iova) {
			dma_unmap_single_attrs(anv->dev, dma, bfr->size,
					       DMA_FROM_DEVICE, DMA_ATTR_SKIP_CPU_SYNC);
			ret = -EINVAL;
			goto unmap_buffer;
		}
		dma_sync_single_for_device(anv->dev, dma, bfr->size, DMA_FROM_DEVICE);
		mapping->dev = get_device(anv->dev);
		bfr->buffer = buffer;
		bfr->is_mapped = true;
		bfr->needs_dma_sync = true;
		bfr->private = mapping;
		return 0;

unmap_buffer:
		memunmap(buffer);
		kfree(mapping);
		return ret;
	}

	bfr->buffer =
		dma_alloc_coherent(anv->dev, bfr->size, &bfr->iova, GFP_KERNEL);
	if (!bfr->buffer)
		return -ENOMEM;

	ret = apple_sart_add_allowed_region(anv->sart, bfr->iova, bfr->size);
	if (ret) {
		dma_free_coherent(anv->dev, bfr->size, bfr->buffer, bfr->iova);
		bfr->buffer = NULL;
		return -ENOMEM;
	}

	return 0;
}

static void apple_nvme_sart_dma_destroy(void *cookie,
					struct apple_rtkit_shmem *bfr)
{
	struct apple_nvme *anv = cookie;
	struct apple_nvme_rtkit_mapping *mapping = bfr->private;

	if (mapping) {
		dma_unmap_single(mapping->dev, bfr->iova, bfr->size, DMA_FROM_DEVICE);
		memunmap(bfr->buffer);
		put_device(mapping->dev);
		kfree(mapping);
		return;
	}
	apple_sart_remove_allowed_region(anv->sart, bfr->iova, bfr->size);
	dma_free_coherent(anv->dev, bfr->size, bfr->buffer, bfr->iova);
}

static const struct apple_rtkit_ops apple_nvme_rtkit_ops = {
	.crashed = apple_nvme_rtkit_crashed,
	.shmem_setup = apple_nvme_sart_dma_setup,
	.shmem_destroy = apple_nvme_sart_dma_destroy,
};

static void apple_nvmmu_inval(struct apple_nvme_queue *q, unsigned int tag)
{
	struct apple_nvme *anv = queue_to_apple_nvme(q);

	writel(tag, anv->mmio_nvmmu + APPLE_NVMMU_TCB_INVAL);
	if (readl(anv->mmio_nvmmu + APPLE_NVMMU_TCB_STAT))
		dev_warn_ratelimited(anv->dev,
				     "NVMMU TCB invalidation failed\n");
}

static void apple_nvme_submit_cmd_t8015(struct apple_nvme_queue *q,
				  struct nvme_command *cmd)
{
	struct apple_nvme *anv = queue_to_apple_nvme(q);

	spin_lock_irq(&anv->lock);

	if (q->is_adminq)
		memcpy(&q->sqes[q->sq_tail], cmd, sizeof(*cmd));
	else
		memcpy((void *)q->sqes + (q->sq_tail << APPLE_NVME_IOSQES),
			cmd, sizeof(*cmd));

	if (++q->sq_tail == apple_nvme_queue_depth(q))
		q->sq_tail = 0;

	writel(q->sq_tail, q->sq_db);
	spin_unlock_irq(&anv->lock);
}


static void apple_nvme_submit_cmd_t8103(struct apple_nvme_queue *q,
				  struct nvme_command *cmd,
				  struct request *req)
{
	struct apple_nvme *anv = queue_to_apple_nvme(q);
	u32 tag = nvme_tag_from_cid(cmd->common.command_id);
	struct apple_nvmmu_tcb *tcb = &q->tcbs[tag];

	tcb->opcode = 0;
	tcb->prp1 = cmd->common.dptr.prp1;
	tcb->prp2 = cmd->common.dptr.prp2;
	tcb->length = cmd->rw.length;
	tcb->command_id = tag;

	if (!cmd->common.dptr.prp1)
		tcb->dma_flags = 0;
	else if (nvme_is_write(cmd))
		tcb->dma_flags = APPLE_ANS_TCB_DMA_TO_DEVICE;
	else
		tcb->dma_flags = APPLE_ANS_TCB_DMA_FROM_DEVICE;

	if (unlikely(req->crypt_ctx)) {
		const u8 *key = req->crypt_ctx->bc_key->bytes;
		__le64 dun_le = cpu_to_le64(req->crypt_ctx->bc_dun[0]);

		tcb->dma_flags |= APPLE_ANS_TCB_AES;
		memcpy(tcb->aes_iv, &dun_le, sizeof(dun_le));
		memset(tcb->_aes_unk, 0, sizeof(tcb->_aes_unk));
		memcpy(tcb->_aes_unk, key, 4);
		memcpy(tcb->_aes_unk + 16, key + 16, 48);
	}
	memcpy(&q->sqes[tag], cmd, sizeof(*cmd));

	/*
	 * This lock here doesn't make much sense at a first glance but
	 * removing it will result in occasional missed completion
	 * interrupts even though the commands still appear on the CQ.
	 * It's unclear why this happens but our best guess is that
	 * there is a bug in the firmware triggered when a new command
	 * is issued while we're inside the irq handler between the
	 * NVMMU invalidation (and making the tag available again)
	 * and the final CQ update.
	 */
	spin_lock_irq(&anv->lock);
	writel(tag, q->sq_db);
	spin_unlock_irq(&anv->lock);
}

/*
 * From pci.c:
 * Will slightly overestimate the number of pages needed.  This is OK
 * as it only leads to a small amount of wasted memory for the lifetime of
 * the I/O.
 */
static inline size_t apple_nvme_iod_alloc_size(void)
{
	const unsigned int nprps = DIV_ROUND_UP(
		NVME_MAX_KB_SZ + NVME_CTRL_PAGE_SIZE, NVME_CTRL_PAGE_SIZE);
	const int npages = DIV_ROUND_UP(8 * nprps, PAGE_SIZE - 8);
	const size_t alloc_size = sizeof(__le64 *) * npages +
				  sizeof(struct scatterlist) * NVME_MAX_SEGS;

	return alloc_size;
}

static void **apple_nvme_iod_list(struct request *req)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);

	return (void **)(iod->sg + blk_rq_nr_phys_segments(req));
}

static void apple_nvme_free_prps(struct apple_nvme *anv, struct request *req)
{
	const int last_prp = NVME_CTRL_PAGE_SIZE / sizeof(__le64) - 1;
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	dma_addr_t dma_addr = iod->first_dma;
	int i;

	for (i = 0; i < iod->npages; i++) {
		__le64 *prp_list = apple_nvme_iod_list(req)[i];
		dma_addr_t next_dma_addr = le64_to_cpu(prp_list[last_prp]);

		dma_pool_free(anv->prp_page_pool, prp_list, dma_addr);
		dma_addr = next_dma_addr;
	}
}

static void apple_nvme_unmap_data(struct apple_nvme *anv, struct request *req)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);

	if (iod->dma_len) {
		dma_unmap_page(anv->dev, iod->first_dma, iod->dma_len,
			       rq_dma_dir(req));
		return;
	}

	WARN_ON_ONCE(!iod->nents);

	dma_unmap_sg(anv->dev, iod->sg, iod->nents, rq_dma_dir(req));
	if (iod->npages == 0)
		dma_pool_free(anv->prp_small_pool, apple_nvme_iod_list(req)[0],
			      iod->first_dma);
	else
		apple_nvme_free_prps(anv, req);
	mempool_free(iod->sg, anv->iod_mempool);
}

static void apple_nvme_print_sgl(struct scatterlist *sgl, int nents)
{
	int i;
	struct scatterlist *sg;

	for_each_sg(sgl, sg, nents, i) {
		dma_addr_t phys = sg_phys(sg);

		pr_warn("sg[%d] phys_addr:%pad offset:%d length:%d dma_address:%pad dma_length:%d\n",
			i, &phys, sg->offset, sg->length, &sg_dma_address(sg),
			sg_dma_len(sg));
	}
}

static blk_status_t apple_nvme_setup_prps(struct apple_nvme *anv,
					  struct request *req,
					  struct nvme_rw_command *cmnd)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct dma_pool *pool;
	int length = blk_rq_payload_bytes(req);
	struct scatterlist *sg = iod->sg;
	int dma_len = sg_dma_len(sg);
	u64 dma_addr = sg_dma_address(sg);
	int offset = dma_addr & (NVME_CTRL_PAGE_SIZE - 1);
	__le64 *prp_list;
	void **list = apple_nvme_iod_list(req);
	dma_addr_t prp_dma;
	int nprps, i;

	length -= (NVME_CTRL_PAGE_SIZE - offset);
	if (length <= 0) {
		iod->first_dma = 0;
		goto done;
	}

	dma_len -= (NVME_CTRL_PAGE_SIZE - offset);
	if (dma_len) {
		dma_addr += (NVME_CTRL_PAGE_SIZE - offset);
	} else {
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
	}

	if (length <= NVME_CTRL_PAGE_SIZE) {
		iod->first_dma = dma_addr;
		goto done;
	}

	nprps = DIV_ROUND_UP(length, NVME_CTRL_PAGE_SIZE);
	if (nprps <= (256 / 8)) {
		pool = anv->prp_small_pool;
		iod->npages = 0;
	} else {
		pool = anv->prp_page_pool;
		iod->npages = 1;
	}

	prp_list = dma_pool_alloc(pool, GFP_ATOMIC, &prp_dma);
	if (!prp_list) {
		iod->first_dma = dma_addr;
		iod->npages = -1;
		return BLK_STS_RESOURCE;
	}
	list[0] = prp_list;
	iod->first_dma = prp_dma;
	i = 0;
	for (;;) {
		if (i == NVME_CTRL_PAGE_SIZE >> 3) {
			__le64 *old_prp_list = prp_list;

			prp_list = dma_pool_alloc(pool, GFP_ATOMIC, &prp_dma);
			if (!prp_list)
				goto free_prps;
			list[iod->npages++] = prp_list;
			prp_list[0] = old_prp_list[i - 1];
			old_prp_list[i - 1] = cpu_to_le64(prp_dma);
			i = 1;
		}
		prp_list[i++] = cpu_to_le64(dma_addr);
		dma_len -= NVME_CTRL_PAGE_SIZE;
		dma_addr += NVME_CTRL_PAGE_SIZE;
		length -= NVME_CTRL_PAGE_SIZE;
		if (length <= 0)
			break;
		if (dma_len > 0)
			continue;
		if (unlikely(dma_len < 0))
			goto bad_sgl;
		sg = sg_next(sg);
		dma_addr = sg_dma_address(sg);
		dma_len = sg_dma_len(sg);
	}
done:
	cmnd->dptr.prp1 = cpu_to_le64(sg_dma_address(iod->sg));
	cmnd->dptr.prp2 = cpu_to_le64(iod->first_dma);
	return BLK_STS_OK;
free_prps:
	apple_nvme_free_prps(anv, req);
	return BLK_STS_RESOURCE;
bad_sgl:
	WARN(DO_ONCE(apple_nvme_print_sgl, iod->sg, iod->nents),
	     "Invalid SGL for payload:%d nents:%d\n", blk_rq_payload_bytes(req),
	     iod->nents);
	return BLK_STS_IOERR;
}

static blk_status_t apple_nvme_setup_prp_simple(struct apple_nvme *anv,
						struct request *req,
						struct nvme_rw_command *cmnd,
						struct bio_vec *bv)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	unsigned int offset = bv->bv_offset & (NVME_CTRL_PAGE_SIZE - 1);
	unsigned int first_prp_len = NVME_CTRL_PAGE_SIZE - offset;

	iod->first_dma = dma_map_bvec(anv->dev, bv, rq_dma_dir(req), 0);
	if (dma_mapping_error(anv->dev, iod->first_dma))
		return BLK_STS_RESOURCE;
	iod->dma_len = bv->bv_len;

	cmnd->dptr.prp1 = cpu_to_le64(iod->first_dma);
	if (bv->bv_len > first_prp_len)
		cmnd->dptr.prp2 = cpu_to_le64(iod->first_dma + first_prp_len);
	return BLK_STS_OK;
}

static blk_status_t apple_nvme_map_data(struct apple_nvme *anv,
					struct request *req,
					struct nvme_command *cmnd)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	blk_status_t ret = BLK_STS_RESOURCE;
	int nr_mapped;

	if (blk_rq_nr_phys_segments(req) == 1) {
		struct bio_vec bv = req_bvec(req);

		if (bv.bv_offset + bv.bv_len <= NVME_CTRL_PAGE_SIZE * 2)
			return apple_nvme_setup_prp_simple(anv, req, &cmnd->rw,
							   &bv);
	}

	iod->dma_len = 0;
	iod->sg = mempool_alloc(anv->iod_mempool, GFP_ATOMIC);
	if (!iod->sg)
		return BLK_STS_RESOURCE;
	sg_init_table(iod->sg, blk_rq_nr_phys_segments(req));
	iod->nents = blk_rq_map_sg(req, iod->sg);
	if (!iod->nents)
		goto out_free_sg;

	nr_mapped = dma_map_sg_attrs(anv->dev, iod->sg, iod->nents,
				     rq_dma_dir(req), DMA_ATTR_NO_WARN);
	if (!nr_mapped)
		goto out_free_sg;

	ret = apple_nvme_setup_prps(anv, req, &cmnd->rw);
	if (ret != BLK_STS_OK)
		goto out_unmap_sg;
	return BLK_STS_OK;

out_unmap_sg:
	dma_unmap_sg(anv->dev, iod->sg, iod->nents, rq_dma_dir(req));
out_free_sg:
	mempool_free(iod->sg, anv->iod_mempool);
	return ret;
}

static __always_inline void apple_nvme_unmap_rq(struct request *req)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct apple_nvme *anv = queue_to_apple_nvme(iod->q);
	struct apple_nvmmu_tcb *tcb;
	u32 tag;

	if (blk_rq_nr_phys_segments(req))
		apple_nvme_unmap_data(anv, req);
	if (!anv->hw->has_lsq_nvmmu)
		return;

	tag = nvme_tag_from_cid(iod->cmd.common.command_id);
	tcb = &iod->q->tcbs[tag];
	if (unlikely(tcb->dma_flags & APPLE_ANS_TCB_AES)) {
		memzero_explicit(tcb->aes_iv, sizeof(tcb->aes_iv));
		memzero_explicit(tcb->_aes_unk, sizeof(tcb->_aes_unk));
		tcb->dma_flags &= ~APPLE_ANS_TCB_AES;
	}
}

static void apple_nvme_complete_rq(struct request *req)
{
	apple_nvme_unmap_rq(req);
	nvme_complete_rq(req);
}

static void apple_nvme_complete_batch(struct io_comp_batch *iob)
{
	nvme_complete_batch(iob, apple_nvme_unmap_rq);
}

static inline bool apple_nvme_cqe_pending(struct apple_nvme_queue *q)
{
	struct nvme_completion *hcqe = &q->cqes[q->cq_head];

	return (le16_to_cpu(READ_ONCE(hcqe->status)) & 1) == q->cq_phase;
}

static inline struct blk_mq_tags *
apple_nvme_queue_tagset(struct apple_nvme *anv, struct apple_nvme_queue *q)
{
	if (q->is_adminq)
		return anv->admin_tagset.tags[0];
	else
		return anv->tagset.tags[0];
}

static inline void apple_nvme_handle_cqe(struct apple_nvme_queue *q,
					 struct io_comp_batch *iob, u16 idx)
{
	struct apple_nvme *anv = queue_to_apple_nvme(q);
	struct nvme_completion *cqe = &q->cqes[idx];
	__u16 command_id = READ_ONCE(cqe->command_id);
	struct request *req;

	if (anv->hw->has_lsq_nvmmu)
		apple_nvmmu_inval(q, command_id);

	req = nvme_find_rq(apple_nvme_queue_tagset(anv, q), command_id);
	if (unlikely(!req)) {
		dev_warn(anv->dev, "invalid id %d completed", command_id);
		return;
	}

	if (!nvme_try_complete_req(req, cqe->status, cqe->result) &&
	    !blk_mq_add_to_batch(req, iob,
				 nvme_req(req)->status != NVME_SC_SUCCESS,
				 apple_nvme_complete_batch))
		apple_nvme_complete_rq(req);
}

static inline void apple_nvme_update_cq_head(struct apple_nvme_queue *q)
{
	u32 tmp = q->cq_head + 1;

	if (tmp == apple_nvme_queue_depth(q)) {
		q->cq_head = 0;
		q->cq_phase ^= 1;
	} else {
		q->cq_head = tmp;
	}
}

static bool apple_nvme_poll_cq(struct apple_nvme_queue *q,
			       struct io_comp_batch *iob)
{
	bool found = false;

	while (apple_nvme_cqe_pending(q)) {
		found = true;

		/*
		 * load-load control dependency between phase and the rest of
		 * the cqe requires a full read memory barrier
		 */
		dma_rmb();
		apple_nvme_handle_cqe(q, iob, q->cq_head);
		apple_nvme_update_cq_head(q);
	}

	if (found)
		writel(q->cq_head, q->cq_db);

	return found;
}

static bool apple_nvme_handle_cq(struct apple_nvme_queue *q, bool force)
{
	bool found;
	DEFINE_IO_COMP_BATCH(iob);

	if (!apple_nvme_queue_enabled(q) && !force)
		return false;

	found = apple_nvme_poll_cq(q, &iob);

	if (!rq_list_empty(&iob.req_list))
		apple_nvme_complete_batch(&iob);

	return found;
}

static irqreturn_t apple_nvme_irq(int irq, void *data)
{
	struct apple_nvme *anv = data;
	bool handled = false;
	unsigned long flags;

	spin_lock_irqsave(&anv->lock, flags);
	if (apple_nvme_handle_cq(&anv->ioq, false))
		handled = true;
	if (apple_nvme_handle_cq(&anv->adminq, false))
		handled = true;
	spin_unlock_irqrestore(&anv->lock, flags);

	if (handled)
		return IRQ_HANDLED;
	return IRQ_NONE;
}

static int apple_nvme_create_cq(struct apple_nvme *anv)
{
	struct nvme_command c = {};

	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
	c.create_cq.opcode = nvme_admin_create_cq;
	c.create_cq.prp1 = cpu_to_le64(anv->ioq.cq_dma_addr);
	c.create_cq.cqid = cpu_to_le16(1);
	c.create_cq.qsize = cpu_to_le16(anv->hw->max_queue_depth - 1);
	c.create_cq.cq_flags = cpu_to_le16(NVME_QUEUE_PHYS_CONTIG | NVME_CQ_IRQ_ENABLED);
	c.create_cq.irq_vector = cpu_to_le16(0);

	return nvme_submit_sync_cmd(anv->ctrl.admin_q, &c, NULL, 0);
}

static int apple_nvme_remove_cq(struct apple_nvme *anv)
{
	struct nvme_command c = {};

	c.delete_queue.opcode = nvme_admin_delete_cq;
	c.delete_queue.qid = cpu_to_le16(1);

	return nvme_submit_sync_cmd(anv->ctrl.admin_q, &c, NULL, 0);
}

static int apple_nvme_create_sq(struct apple_nvme *anv)
{
	struct nvme_command c = {};

	/*
	 * Note: we (ab)use the fact that the prp fields survive if no data
	 * is attached to the request.
	 */
	c.create_sq.opcode = nvme_admin_create_sq;
	c.create_sq.prp1 = cpu_to_le64(anv->ioq.sq_dma_addr);
	c.create_sq.sqid = cpu_to_le16(1);
	c.create_sq.qsize = cpu_to_le16(anv->hw->max_queue_depth - 1);
	c.create_sq.sq_flags = cpu_to_le16(NVME_QUEUE_PHYS_CONTIG);
	c.create_sq.cqid = cpu_to_le16(1);

	return nvme_submit_sync_cmd(anv->ctrl.admin_q, &c, NULL, 0);
}

static int apple_nvme_remove_sq(struct apple_nvme *anv)
{
	struct nvme_command c = {};

	c.delete_queue.opcode = nvme_admin_delete_sq;
	c.delete_queue.qid = cpu_to_le16(1);

	return nvme_submit_sync_cmd(anv->ctrl.admin_q, &c, NULL, 0);
}

static bool apple_nvme_delayed_flush(struct apple_nvme *anv, struct nvme_ns *ns,
				     struct request *req)
{
	/*
	 * A flush completion is a persistence boundary for the filesystem.
	 * Post-M4 internal-root storage must submit it to the controller and
	 * wait for its completion, even when the legacy grace period is set.
	 */
	if (anv->hw->needs_ioq_registers ||
	    !anv->flush_interval || req_op(req) != REQ_OP_FLUSH)
		return false;
	if (delayed_work_pending(&anv->flush_dwork))
		return true;
	if (time_before(jiffies, anv->last_flush + anv->flush_interval)) {
		kblockd_mod_delayed_work_on(WORK_CPU_UNBOUND, &anv->flush_dwork,
						anv->flush_interval);
		if (WARN_ON_ONCE(anv->flush_ns && anv->flush_ns != ns))
			goto out;
		anv->flush_ns = ns;
		return true;
	}
out:
	anv->last_flush = jiffies;
	return false;
}

static blk_status_t apple_nvme_queue_rq(struct blk_mq_hw_ctx *hctx,
					const struct blk_mq_queue_data *bd)
{
	struct nvme_ns *ns = hctx->queue->queuedata;
	struct apple_nvme_queue *q = hctx->driver_data;
	struct apple_nvme *anv = queue_to_apple_nvme(q);
	struct request *req = bd->rq;
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_command *cmnd = &iod->cmd;
	blk_status_t ret;

	iod->npages = -1;
	iod->nents = 0;

	/*
	 * We should not need to do this, but we're still using this to
	 * ensure we can drain requests on a dying queue.
	 */
	if (unlikely(!apple_nvme_queue_enabled(q)))
		return BLK_STS_IOERR;

	if (!nvme_check_ready(&anv->ctrl, req, true))
		return nvme_fail_nonready_command(&anv->ctrl, req);

	ret = nvme_setup_cmd(ns, req);
	if (ret)
		return ret;
	if (unlikely(req->crypt_ctx)) {
		const struct blk_crypto_key *key = req->crypt_ctx->bc_key;

		if (WARN_ON_ONCE(key->size != APPLE_NVME_CRYPTO_KEY_SIZE ||
				 key->crypto_cfg.crypto_mode !=
					 BLK_ENCRYPTION_MODE_AES_256_XTS ||
				 key->crypto_cfg.key_type !=
					 BLK_CRYPTO_KEY_TYPE_HW_WRAPPED ||
				 key->crypto_cfg.data_unit_size !=
					 APPLE_NVME_CRYPTO_DATA_UNIT_SIZE)) {
			ret = BLK_STS_NOTSUPP;
			goto out_free_cmd;
		}
	}

	if (blk_rq_nr_phys_segments(req)) {
		ret = apple_nvme_map_data(anv, req, cmnd);
		if (ret)
			goto out_free_cmd;
	}

	nvme_start_request(req);

	if (apple_nvme_delayed_flush(anv, ns, req)) {
                blk_mq_complete_request(req);
                return BLK_STS_OK;
        }

	if (anv->hw->has_lsq_nvmmu)
		apple_nvme_submit_cmd_t8103(q, cmnd, req);
	else
		apple_nvme_submit_cmd_t8015(q, cmnd);

	return BLK_STS_OK;

out_free_cmd:
	nvme_cleanup_cmd(req);
	return ret;
}

static int apple_nvme_init_hctx(struct blk_mq_hw_ctx *hctx, void *data,
				unsigned int hctx_idx)
{
	hctx->driver_data = data;
	return 0;
}

static int apple_nvme_init_request(struct blk_mq_tag_set *set,
				   struct request *req, unsigned int hctx_idx,
				   int numa_node)
{
	struct apple_nvme_queue *q = set->driver_data;
	struct apple_nvme *anv = queue_to_apple_nvme(q);
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct nvme_request *nreq = nvme_req(req);

	iod->q = q;
	nreq->ctrl = &anv->ctrl;
	nreq->cmd = &iod->cmd;

	return 0;
}

static void apple_nvme_disable_legacy(struct apple_nvme *anv, bool shutdown)
{
	enum nvme_ctrl_state state = nvme_ctrl_state(&anv->ctrl);
	u32 csts = readl(anv->mmio_nvme + NVME_REG_CSTS);
	bool dead = false, freeze = false;
	unsigned long flags;

	if (apple_rtkit_is_crashed(anv->rtk))
		dead = true;
	if (!(csts & NVME_CSTS_RDY))
		dead = true;
	if (csts & NVME_CSTS_CFS)
		dead = true;

	if (state == NVME_CTRL_LIVE ||
	    state == NVME_CTRL_RESETTING) {
		freeze = true;
		nvme_start_freeze(&anv->ctrl);
	}

	/*
	 * Give the controller a chance to complete all entered requests if
	 * doing a safe shutdown.
	 */
	if (!dead && shutdown && freeze)
		nvme_wait_freeze_timeout(&anv->ctrl);

	nvme_quiesce_io_queues(&anv->ctrl);

	if (!dead) {
		if (apple_nvme_queue_enabled(&anv->ioq)) {
			apple_nvme_remove_sq(anv);
			apple_nvme_remove_cq(anv);
		}

		/*
		 * Always disable the NVMe controller after shutdown.
		 * We need to do this to bring it back up later anyway, and we
		 * can't do it while the firmware is not running (e.g. in the
		 * resume reset path before RTKit is initialized), so for Apple
		 * controllers it makes sense to unconditionally do it here.
		 * Additionally, this sequence of events is reliable, while
		 * others (like disabling after bringing back the firmware on
		 * resume) seem to run into trouble under some circumstances.
		 *
		 * Both U-Boot and m1n1 also use this convention (i.e. an ANS
		 * NVMe controller is handed off with firmware shut down, in an
		 * NVMe disabled state, after a clean shutdown).
		 */
		if (shutdown)
			nvme_disable_ctrl(&anv->ctrl, shutdown);
		nvme_disable_ctrl(&anv->ctrl, false);
	}

	apple_nvme_disable_queue(&anv->ioq);
	apple_nvme_disable_queue(&anv->adminq);
	mb(); /* ensure that nvme_queue_rq() sees that enabled is cleared */
	nvme_quiesce_admin_queue(&anv->ctrl);

	/* last chance to complete any requests before nvme_cancel_request */
	spin_lock_irqsave(&anv->lock, flags);
	apple_nvme_handle_cq(&anv->ioq, true);
	apple_nvme_handle_cq(&anv->adminq, true);
	spin_unlock_irqrestore(&anv->lock, flags);

	nvme_cancel_tagset(&anv->ctrl);
	nvme_cancel_admin_tagset(&anv->ctrl);

	/*
	 * The driver will not be starting up queues again if shutting down so
	 * must flush all entered requests to their failed completion to avoid
	 * deadlocking blk-mq hot-cpu notifier.
	 */
	if (shutdown) {
		nvme_unquiesce_io_queues(&anv->ctrl);
		nvme_unquiesce_admin_queue(&anv->ctrl);
	}
}

static void apple_nvme_quarantine(struct apple_nvme *anv, int error)
{
	int i;

	/* Called under disable_lock after both dispatch queues are quiesced. */
	WRITE_ONCE(anv->quarantined, true);
	/*
	 * The retained requests keep blk-mq timeouts and completions pointing
	 * into this module until reboot.
	 */
	__module_get(THIS_MODULE);
	apple_nvme_set_reset_ready(anv, false);
	disable_irq(anv->irq);
	if (anv->pd_count > 1) {
		for (i = 0; i < anv->pd_count; i++)
			pm_runtime_get_noresume(anv->pd_dev[i]);
	}
	mutex_lock(&apple_nvme_quarantine_lock);
	list_add_tail(&anv->quarantine_node, &apple_nvme_quarantines);
	mutex_unlock(&apple_nvme_quarantine_lock);
	nvme_change_ctrl_state(&anv->ctrl, NVME_CTRL_DELETING);
	nvme_mark_namespaces_dead(&anv->ctrl);
	dev_crit(anv->dev,
		 "controller did not stop (%d); retaining DMA and requests until platform reset\n",
		 error);
}

static int apple_nvme_disable_locked(struct apple_nvme *anv, bool shutdown)
{
	enum nvme_ctrl_state state;
	unsigned long flags;
	int ret;

	lockdep_assert_held(&anv->disable_lock);
	if (READ_ONCE(anv->quarantined))
		return -EIO;

	state = nvme_ctrl_state(&anv->ctrl);
	if ((state == NVME_CTRL_LIVE || state == NVME_CTRL_RESETTING) &&
	    !test_bit(NVME_CTRL_FROZEN, &anv->ctrl.flags))
		nvme_start_freeze(&anv->ctrl);
	if (shutdown && test_bit(NVME_CTRL_FROZEN, &anv->ctrl.flags) &&
	    !apple_rtkit_is_crashed(anv->rtk))
		nvme_wait_freeze_timeout(&anv->ctrl);

	/* Quiescing dispatch does not stop already submitted DMA. */
	nvme_quiesce_io_queues(&anv->ctrl);
	nvme_quiesce_admin_queue(&anv->ctrl);
	apple_nvme_disable_queue(&anv->ioq);
	apple_nvme_disable_queue(&anv->adminq);

	/*
	 * CFS and firmware crashes are not DMA fences. Post-M4 does not use
	 * Delete SQ/CQ, SHN or RTKit sleep; only a completed CC.EN clear makes
	 * the queue and request mappings eligible for release.
	 */
	ret = nvme_disable_ctrl(&anv->ctrl, false);
	if (ret) {
		apple_nvme_quarantine(anv, ret);
		return ret;
	}

	spin_lock_irqsave(&anv->lock, flags);
	apple_nvme_handle_cq(&anv->ioq, true);
	apple_nvme_handle_cq(&anv->adminq, true);
	spin_unlock_irqrestore(&anv->lock, flags);
	nvme_cancel_tagset(&anv->ctrl);
	nvme_cancel_admin_tagset(&anv->ctrl);
	if (shutdown) {
		nvme_unquiesce_io_queues(&anv->ctrl);
		nvme_unquiesce_admin_queue(&anv->ctrl);
	}
	return 0;
}

static int apple_nvme_disable(struct apple_nvme *anv, bool shutdown)
{
	if (!anv->hw->needs_ioq_registers) {
		apple_nvme_disable_legacy(anv, shutdown);
		return 0;
	}
	guard(mutex)(&anv->disable_lock);
	return apple_nvme_disable_locked(anv, shutdown);
}

static void apple_nvme_recovery_work(struct work_struct *work)
{
	struct apple_nvme *anv = container_of(work, struct apple_nvme, recovery_work);
	bool restart = anv->recover_to_live;
	int ret;

	/*
	 * The timeout callback must return before cancellation waits for the
	 * block layer's reference to that request. This worker owns the stop.
	 */
	if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->removing))
		goto done;
	mutex_lock(&anv->disable_lock);
	ret = apple_nvme_disable_locked(anv, false);
	/*
	 * Without restart, this stop aborts an initialization, and no reset
	 * follows. The requests it cancelled have been requeued; let them
	 * reach the stopped queues and fail, so that reset_work does not wait
	 * for them forever.
	 */
	if (!ret && !restart) {
		nvme_unquiesce_io_queues(&anv->ctrl);
		nvme_unquiesce_admin_queue(&anv->ctrl);
	}
	mutex_unlock(&anv->disable_lock);
	if (ret)
		goto done;
	clear_bit(APPLE_NVME_RECOVERY_PENDING, &anv->recovery_flags);
	if (restart && !READ_ONCE(anv->removing) && !READ_ONCE(anv->suspended))
		nvme_try_sched_reset(&anv->ctrl);
	return;
done:
	clear_bit(APPLE_NVME_RECOVERY_PENDING, &anv->recovery_flags);
}

static enum blk_eh_timer_return apple_nvme_timeout(struct request *req)
{
	struct apple_nvme_iod *iod = blk_mq_rq_to_pdu(req);
	struct apple_nvme_queue *q = iod->q;
	struct apple_nvme *anv = queue_to_apple_nvme(q);
	struct nvme_completion *cqe = &q->cqes[q->cq_head];
	enum nvme_ctrl_state state = nvme_ctrl_state(&anv->ctrl);
	unsigned long flags;
	u32 cc, csts;

	if (blk_mq_request_completed(req))
		return BLK_EH_DONE;
	if (READ_ONCE(anv->quarantined))
		return BLK_EH_RESET_TIMER;
	cc = readl(anv->mmio_nvme + NVME_REG_CC);
	csts = readl(anv->mmio_nvme + NVME_REG_CSTS);

	/*
	 * The first Identify runs while the controller is CONNECTING.  Poll the
	 * completion queue before applying the non-live fast-fail rule so a lost
	 * interrupt cannot turn a completed initialization command into a reset.
	 */
	if (!apple_rtkit_is_crashed(anv->rtk) && !(csts & NVME_CSTS_CFS)) {
		spin_lock_irqsave(&anv->lock, flags);
		apple_nvme_handle_cq(q, false);
		spin_unlock_irqrestore(&anv->lock, flags);
		if (blk_mq_request_completed(req)) {
			dev_warn(anv->dev,
				 "I/O %d(aq:%d) timeout in state %d: completion polled\n",
				 req->tag, q->is_adminq, state);
			return BLK_EH_DONE;
		}
	}

	if (anv->hw->needs_ioq_registers) {
		/*
		 * remove() drains the namespaces through the live controller and
		 * stops it afterwards. A request that times out meanwhile stops
		 * the controller here, as pci.c does for a deleting controller.
		 */
		if ((state == NVME_CTRL_DELETING ||
		     state == NVME_CTRL_DELETING_NOIO) &&
		    READ_ONCE(anv->removing)) {
			nvme_req(req)->flags |= NVME_REQ_CANCELLED;
			if (apple_nvme_disable(anv, true))
				return BLK_EH_RESET_TIMER;
			return BLK_EH_DONE;
		}
		/*
		 * An existing reset/removal owns the stop of its old requests.
		 * CONNECTING is different: reset_work may be waiting for this
		 * very admin command and needs the separate recovery worker.
		 */
		if (state != NVME_CTRL_LIVE && state != NVME_CTRL_CONNECTING)
			return BLK_EH_RESET_TIMER;
		if (!READ_ONCE(anv->removing) &&
		    !test_and_set_bit(APPLE_NVME_RECOVERY_PENDING, &anv->recovery_flags)) {
			/*
			 * Reserve RESETTING before scheduling recovery. An existing
			 * initialization instead receives an abort indication; its
			 * synchronous command is cancelled only after DMA stops.
			 */
			anv->recover_to_live =
				nvme_change_ctrl_state(&anv->ctrl, NVME_CTRL_RESETTING);
			if (!anv->recover_to_live) {
				if (nvme_ctrl_state(&anv->ctrl) != NVME_CTRL_CONNECTING) {
					clear_bit(APPLE_NVME_RECOVERY_PENDING,
						  &anv->recovery_flags);
					return BLK_EH_RESET_TIMER;
				}
				WRITE_ONCE(anv->abort_reset, true);
			}
			queue_work(system_dfl_wq, &anv->recovery_work);
		}
		return BLK_EH_RESET_TIMER;
	}

	if (state != NVME_CTRL_LIVE) {
		/*
		 * From rdma.c:
		 * If we are resetting, connecting or deleting we should
		 * complete immediately because we may block controller
		 * teardown or setup sequence
		 * - ctrl disable/shutdown fabrics requests
		 * - connect requests
		 * - initialization admin requests
		 * - I/O requests that entered after unquiescing and
		 *   the controller stopped responding
		 *
		 * All other requests should be cancelled by the error
		 * recovery work, so it's fine that we fail it here.
		 */
		dev_warn(anv->dev,
			 "I/O %d(aq:%d) timeout in state %d CC=0x%08x CSTS=0x%08x CQ=%u/%u status=0x%04x cid=%u\n",
			 req->tag, q->is_adminq, state, cc, csts, q->cq_head,
			 q->cq_phase, le16_to_cpu(READ_ONCE(cqe->status)),
			 READ_ONCE(cqe->command_id));
		if (blk_mq_request_started(req) &&
		    !blk_mq_request_completed(req)) {
			nvme_req(req)->status = NVME_SC_HOST_ABORTED_CMD;
			nvme_req(req)->flags |= NVME_REQ_CANCELLED;
			blk_mq_complete_request(req);
		}
		return BLK_EH_DONE;
	}

	/*
	 * aborting commands isn't supported which leaves a full reset as our
	 * only option here
	 */
	dev_warn(anv->dev,
		 "I/O %d(aq:%d) timeout: resetting controller CC=0x%08x CSTS=0x%08x CQ=%u/%u status=0x%04x cid=%u\n",
		 req->tag, q->is_adminq, cc, csts, q->cq_head, q->cq_phase,
		 le16_to_cpu(READ_ONCE(cqe->status)),
		 READ_ONCE(cqe->command_id));
	nvme_req(req)->flags |= NVME_REQ_CANCELLED;
	apple_nvme_disable(anv, false);
	nvme_reset_ctrl(&anv->ctrl);
	return BLK_EH_DONE;
}

static int apple_nvme_poll(struct blk_mq_hw_ctx *hctx,
			   struct io_comp_batch *iob)
{
	struct apple_nvme_queue *q = hctx->driver_data;
	struct apple_nvme *anv = queue_to_apple_nvme(q);
	bool found;
	unsigned long flags;

	if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->detached))
		return 0;
	spin_lock_irqsave(&anv->lock, flags);
	found = apple_nvme_poll_cq(q, iob);
	spin_unlock_irqrestore(&anv->lock, flags);

	return found;
}

static const struct blk_mq_ops apple_nvme_mq_admin_ops = {
	.queue_rq = apple_nvme_queue_rq,
	.complete = apple_nvme_complete_rq,
	.init_hctx = apple_nvme_init_hctx,
	.init_request = apple_nvme_init_request,
	.timeout = apple_nvme_timeout,
};

static const struct blk_mq_ops apple_nvme_mq_ops = {
	.queue_rq = apple_nvme_queue_rq,
	.complete = apple_nvme_complete_rq,
	.init_hctx = apple_nvme_init_hctx,
	.init_request = apple_nvme_init_request,
	.timeout = apple_nvme_timeout,
	.poll = apple_nvme_poll,
};

static void apple_nvme_init_queue(struct apple_nvme_queue *q)
{
	unsigned int depth = apple_nvme_queue_depth(q);
	struct apple_nvme *anv = queue_to_apple_nvme(q);

	q->sq_tail = 0;
	q->cq_head = 0;
	q->cq_phase = 1;
	if (anv->hw->has_lsq_nvmmu)
		memset(q->tcbs, 0, anv->hw->max_queue_depth
			* sizeof(struct apple_nvmmu_tcb));
	memset(q->cqes, 0, depth * sizeof(struct nvme_completion));
	apple_nvme_enable_queue(q);
}

/*
 * Wait for the requests that entered before a post-M4 reset. Give up once
 * the reset has been aborted, whose failure path stops the controller, or
 * the controller has been quarantined, which keeps its requests.
 */
static void apple_nvme_wait_freeze(struct apple_nvme *anv)
{
	while (!nvme_wait_freeze_timeout(&anv->ctrl)) {
		if (READ_ONCE(anv->abort_reset) || READ_ONCE(anv->quarantined))
			return;
	}
}

static void apple_nvme_reset_work(struct work_struct *work)
{
	unsigned int nr_io_queues = 1;
	bool phase_locked = false;
	int ret;
	u32 boot_status, aqa, ioqa, ioqa_depth;
	struct apple_nvme *anv =
		container_of(work, struct apple_nvme, ctrl.reset_work);
	enum nvme_ctrl_state state = nvme_ctrl_state(&anv->ctrl);

	if (anv->hw->needs_ioq_registers) {
		mutex_lock(&anv->disable_lock);
		phase_locked = true;
	}
	if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->removing)) {
		if (phase_locked)
			mutex_unlock(&anv->disable_lock);
		return;
	}
	WRITE_ONCE(anv->abort_reset, false);
	if (state != NVME_CTRL_RESETTING) {
		dev_warn(anv->dev, "ctrl state %d is not RESETTING\n", state);
		ret = -ENODEV;
		goto out;
	}

	/* there's unfortunately no known way to recover if RTKit crashed :( */
	if (apple_rtkit_is_crashed(anv->rtk)) {
		dev_err(anv->dev,
			"RTKit has crashed without any way to recover.");
		ret = -EIO;
		goto out;
	}

	/*
	 * Post-M4 firmware can be handed over live, but cannot renegotiate HELLO
	 * and EPMAP across boot stages. Adopt the first healthy inherited session;
	 * subsequent post-M4 controller resets retain the owned RTKit session.
	 */
	if (!anv->owns_rtkit && apple_nvme_can_adopt_rtkit(anv)) {
		ret = apple_rtkit_adopt_running(anv->rtk);
		if (ret)
			goto out;
		dev_dbg(anv->dev, "adopted inherited post-M4 RTKit session\n");

		/*
		 * U-Boot keeps the controller enabled until the mailbox session has
		 * an owner. Disable it only after adoption, so that the admin queue
		 * and NVMMU pointers below are replaced while CC.EN is clear. The
		 * I/O queue registers are written after Create SQ, as on a cold
		 * start.
		 */
		anv->ctrl.cap = readq(anv->mmio_nvme + NVME_REG_CAP);
		anv->ctrl.ctrl_config =
			readl(anv->mmio_nvme + NVME_REG_CC);
		if (anv->ctrl.ctrl_config & NVME_CC_ENABLE) {
			ret = nvme_disable_ctrl(&anv->ctrl, false);
			if (ret)
				goto out;
			dev_dbg(anv->dev,
				"disabled inherited post-M4 controller after RTKit adoption\n");
		}
		goto rtkit_ready;
	}

	/*
	 * RTKit must be shut down cleanly for the (soft)-reset to work, except
	 * on post-M4: that firmware stays live and only its queue DMA is
	 * stopped and drained, under the register-programming lock.
	 */
	if (apple_rtkit_is_running(anv->rtk)) {
		if (anv->hw->needs_ioq_registers) {
			if (!anv->owns_rtkit) {
				ret = -EIO;
				goto out;
			}
			ret = apple_nvme_disable_locked(anv, false);
			if (ret)
				goto out;
			goto rtkit_ready;
		}
		/* reset the controller if it is enabled */
		if (anv->ctrl.ctrl_config & NVME_CC_ENABLE)
			apple_nvme_disable_legacy(anv, false);
		dev_dbg(anv->dev, "Trying to shut down RTKit before reset.");
		ret = apple_rtkit_shutdown(anv->rtk);
		if (ret)
			goto out;

		writel(0, anv->mmio_coproc + APPLE_ANS_COPROC_CPU_CONTROL);
	}

	/*
	 * Only do the soft-reset if the CPU is not running, which means either we
	 * or the previous stage shut it down cleanly.
	 */
	if (!(readl(anv->mmio_coproc + APPLE_ANS_COPROC_CPU_CONTROL) &
		APPLE_ANS_COPROC_CPU_CONTROL_RUN)) {

		if (anv->hw->needs_ioq_registers) {
			/*
			 * Post-M4 ANS cannot be reset through PMGR: the reset
			 * assert raises an SError. A previous boot stage that
			 * stopped the coprocessor cleanly leaves its firmware
			 * resident, so set RUN and run a fresh RTKit boot
			 * handshake.
			 */
			dev_dbg(anv->dev,
				"post-M4 firmware handoff: restarting stopped ANS without reset\n");
			writel(APPLE_ANS_COPROC_CPU_CONTROL_RUN,
			       anv->mmio_coproc + APPLE_ANS_COPROC_CPU_CONTROL);
			ret = apple_rtkit_boot(anv->rtk);
			goto booted;
		}

		ret = reset_control_assert(anv->reset);
		if (ret)
			goto out;

		ret = apple_rtkit_reinit(anv->rtk);
		if (ret)
			goto out;

		ret = reset_control_deassert(anv->reset);
		if (ret)
			goto out;

		writel(APPLE_ANS_COPROC_CPU_CONTROL_RUN,
		       anv->mmio_coproc + APPLE_ANS_COPROC_CPU_CONTROL);

		ret = apple_rtkit_boot(anv->rtk);
	} else {
		ret = apple_rtkit_wake(anv->rtk);
	}

booted:
	if (ret) {
		dev_err(anv->dev, "ANS did not boot");
		goto out;
	}

rtkit_ready:
	if (anv->hw->needs_ioq_registers &&
	    (READ_ONCE(anv->removing) || READ_ONCE(anv->abort_reset))) {
		ret = -ECANCELED;
		goto out;
	}
	anv->owns_rtkit = true;

	ret = readl_poll_timeout(anv->mmio_nvme + APPLE_ANS_BOOT_STATUS,
				 boot_status,
				 boot_status == APPLE_ANS_BOOT_STATUS_OK,
				 USEC_PER_MSEC, APPLE_ANS_BOOT_TIMEOUT);
	if (ret) {
		dev_err(anv->dev, "ANS did not initialize");
		goto out;
	}

	dev_dbg(anv->dev, "ANS booted successfully.");

	/*
	 * Limit the max command size to prevent iod->sg allocations going
	 * over a single page.
	 */
	anv->ctrl.max_hw_sectors = min_t(u32, NVME_MAX_KB_SZ << 1,
					 dma_max_mapping_size(anv->dev) >> 9);
	anv->ctrl.max_segments = NVME_MAX_SEGS;

	dma_set_max_seg_size(anv->dev, 0xffffffff);

	if (anv->hw->has_lsq_nvmmu) {
		/*
		 * Enable NVMMU and linear submission queues which is required
		 * since T6000.
		 */
		writel(APPLE_ANS_LINEAR_SQ_EN,
			anv->mmio_nvme + apple_nvme_reg(anv, APPLE_ANS_LINEAR_SQ_CTRL));
		if (anv->hw->needs_ioq_registers) {
			u32 lsq_ctrl = readl(anv->mmio_nvme +
					     apple_nvme_reg(anv, APPLE_ANS_LINEAR_SQ_CTRL));

			if (!(lsq_ctrl & APPLE_ANS_LINEAR_SQ_EN) && !anv->hw->no_ioq_readback) {
				dev_err(anv->dev,
					"failed to enable linear submission queues: LSQ_CTRL=0x%08x\n",
					lsq_ctrl);
				ret = -EIO;
				goto out;
			}
		}

		/*
		 * The legacy register takes queue entry counts. On post-M4
		 * controllers the same offset is IOQA and both fields contain the
		 * zero-based queue size, like NVMe AQA. Programming 64 for a
		 * 64-entry queue makes ANS treat it as 65 entries and reject CQ head
		 * 63 once the host wraps through the queue.
		 */
		ioqa_depth = anv->hw->max_queue_depth;
		if (anv->hw->needs_ioq_registers)
			ioqa_depth--;
		ioqa = ioqa_depth | (ioqa_depth << 16);
		writel(ioqa, anv->mmio_nvme + apple_nvme_reg(anv, APPLE_ANS_MAX_PEND_CMDS_CTRL));
		if (anv->hw->needs_ioq_registers && !anv->hw->no_ioq_readback &&
		    readl(anv->mmio_nvme + apple_nvme_reg(anv, APPLE_ANS_MAX_PEND_CMDS_CTRL)) != ioqa) {
			dev_err(anv->dev, "failed to program I/O queue aperture\n");
			ret = -EIO;
			goto out;
		}

		/* Setup the NVMMU for the maximum admin and IO queue depth */
		writel(anv->hw->max_queue_depth - 1,
			anv->mmio_nvmmu + APPLE_NVMMU_NUM_TCBS);
	}

	/* Setup the admin queue */
	aqa = APPLE_NVME_AQ_DEPTH - 1;
	aqa |= aqa << 16;
	writel(aqa, anv->mmio_nvme + apple_nvme_reg(anv, NVME_REG_AQA));
	apple_nvme_writeq(anv, anv->adminq.sq_dma_addr,
			  anv->mmio_nvme + apple_nvme_reg(anv, NVME_REG_ASQ));
	apple_nvme_writeq(anv, anv->adminq.cq_dma_addr,
			  anv->mmio_nvme + apple_nvme_reg(anv, NVME_REG_ACQ));

	if (anv->hw->has_lsq_nvmmu) {
		/* Setup NVMMU for both queues */
		apple_nvme_writeq(anv, anv->adminq.tcb_dma_addr,
				  anv->mmio_nvmmu + APPLE_NVMMU_ASQ_TCB_BASE);
		apple_nvme_writeq(anv, anv->ioq.tcb_dma_addr,
				  anv->mmio_nvmmu + APPLE_NVMMU_IOSQ_TCB_BASE);
	}

	anv->ctrl.sqsize =
		anv->hw->max_queue_depth - 1; /* 0's based queue depth */
	anv->ctrl.cap = readq(anv->mmio_nvme + NVME_REG_CAP);

	dev_dbg(anv->dev, "Enabling controller now");
	ret = nvme_enable_ctrl(&anv->ctrl);
	if (ret)
		goto out;

	dev_dbg(anv->dev, "Starting admin queue");
	apple_nvme_init_queue(&anv->adminq);
	nvme_unquiesce_admin_queue(&anv->ctrl);

	if (!nvme_change_ctrl_state(&anv->ctrl, NVME_CTRL_CONNECTING)) {
		dev_warn(anv->ctrl.device,
			 "failed to mark controller CONNECTING\n");
		ret = -ENODEV;
		goto out;
	}

	/*
	 * Admin commands may need the recovery worker to cancel them. Never
	 * hold disable_lock while waiting for their completions.
	 */
	if (phase_locked) {
		mutex_unlock(&anv->disable_lock);
		phase_locked = false;
	}
	ret = nvme_init_ctrl_finish(&anv->ctrl, false);
	if (ret)
		goto out;

	dev_dbg(anv->dev, "Creating IOCQ");
	ret = apple_nvme_create_cq(anv);
	if (ret)
		goto out;
	dev_dbg(anv->dev, "Creating IOSQ");
	ret = apple_nvme_create_sq(anv);
	if (ret)
		goto out_remove_cq;

	if (anv->hw->needs_ioq_registers) {
		mutex_lock(&anv->disable_lock);
		phase_locked = true;
		if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->removing) ||
		    READ_ONCE(anv->abort_reset)) {
			ret = -ECANCELED;
			goto out;
		}
	}
	apple_nvme_init_queue(&anv->ioq);
	if (anv->hw->needs_ioq_registers) {
		/*
		 * Post-M4 firmware also takes the I/O queue addresses through
		 * dedicated registers. Without them it crashes and its crash log
		 * reports null I/O SQ and CQ addresses.
		 */
		apple_nvme_writeq(anv, anv->ioq.cq_dma_addr,
				  anv->mmio_nvme + apple_nvme_reg(anv, APPLE_ANS_IOCQ_REGISTER));
		apple_nvme_writeq(anv, anv->ioq.sq_dma_addr,
				  anv->mmio_nvme + apple_nvme_reg(anv, APPLE_ANS_IOSQ_REGISTER));
	}
	nr_io_queues = 1;
	if (anv->hw->has_queue_count) {
		ret = nvme_set_queue_count(&anv->ctrl, &nr_io_queues);
		if (ret)
			goto out_remove_sq;
		if (nr_io_queues != 1) {
			ret = -ENXIO;
			goto out_remove_sq;
		}
	}

	anv->ctrl.queue_count = nr_io_queues + 1;

	nvme_unquiesce_io_queues(&anv->ctrl);
	if (phase_locked) {
		/*
		 * Do not hold disable_lock while waiting for the requests that
		 * entered before the reset: if one of them times out, stopping
		 * the controller needs the lock. That stop fails the requests,
		 * unless it quarantines the controller, which
		 * apple_nvme_wait_freeze() gives up on.
		 */
		mutex_unlock(&anv->disable_lock);
		apple_nvme_wait_freeze(anv);
		mutex_lock(&anv->disable_lock);
		if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->removing) ||
		    READ_ONCE(anv->abort_reset)) {
			nvme_unfreeze(&anv->ctrl);
			ret = -ECANCELED;
			goto out;
		}
	} else {
		nvme_wait_freeze(&anv->ctrl);
	}
	blk_mq_update_nr_hw_queues(&anv->tagset, 1);
	nvme_unfreeze(&anv->ctrl);

	if (!nvme_change_ctrl_state(&anv->ctrl, NVME_CTRL_LIVE)) {
		dev_warn(anv->ctrl.device,
			 "failed to mark controller live state\n");
		ret = -ENODEV;
		goto out_remove_sq;
	}

	/*
	 * nvme_start_ctrl() sends Set Features if the controller reports
	 * asynchronous events. If that times out, the controller is live and
	 * recovery_work needs disable_lock to stop it. As in pci.c, a stop
	 * that lands before nvme_start_ctrl() unquiesces the I/O queues loses
	 * its quiesce, and requests then fail on the stopped queues until the
	 * next reset instead of waiting for it.
	 */
	if (phase_locked)
		mutex_unlock(&anv->disable_lock);
	nvme_start_ctrl(&anv->ctrl);

	dev_dbg(anv->dev, "ANS boot and NVMe init completed.");
	return;

out_remove_sq:
	if (!anv->hw->needs_ioq_registers)
		apple_nvme_remove_sq(anv);
out_remove_cq:
	if (!anv->hw->needs_ioq_registers)
		apple_nvme_remove_cq(anv);
out:
	if (phase_locked)
		mutex_unlock(&anv->disable_lock);
	dev_warn(anv->ctrl.device, "Reset failure status: %d\n", ret);
	nvme_change_ctrl_state(&anv->ctrl, NVME_CTRL_DELETING);
	if (apple_nvme_disable(anv, false) || READ_ONCE(anv->removing))
		return;
	nvme_get_ctrl(&anv->ctrl);
	nvme_mark_namespaces_dead(&anv->ctrl);
	if (!queue_work(nvme_wq, &anv->remove_work))
		nvme_put_ctrl(&anv->ctrl);
}

static void apple_nvme_remove_dead_ctrl_work(struct work_struct *work)
{
	struct apple_nvme *anv =
		container_of(work, struct apple_nvme, remove_work);

	nvme_put_ctrl(&anv->ctrl);
	device_release_driver(anv->dev);
}

static int apple_nvme_reg_read32(struct nvme_ctrl *ctrl, u32 off, u32 *val)
{
	struct apple_nvme *anv = ctrl_to_apple_nvme(ctrl);

	if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->detached))
		return -ENODEV;
	*val = readl(anv->mmio_nvme + off);
	return 0;
}

static int apple_nvme_reg_write32(struct nvme_ctrl *ctrl, u32 off, u32 val)
{
	struct apple_nvme *anv = ctrl_to_apple_nvme(ctrl);

	if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->detached))
		return -ENODEV;
	writel(val, anv->mmio_nvme + off);
	return 0;
}

static int apple_nvme_reg_read64(struct nvme_ctrl *ctrl, u32 off, u64 *val)
{
	struct apple_nvme *anv = ctrl_to_apple_nvme(ctrl);

	if (READ_ONCE(anv->quarantined) || READ_ONCE(anv->detached))
		return -ENODEV;
	*val = readq(anv->mmio_nvme + off);
	return 0;
}

static int apple_nvme_get_address(struct nvme_ctrl *ctrl, char *buf, int size)
{
	struct device *dev = ctrl_to_apple_nvme(ctrl)->dev;

	return snprintf(buf, size, "%s\n", dev_name(dev));
}

static void apple_nvme_free_ctrl(struct nvme_ctrl *ctrl)
{
	struct apple_nvme *anv = ctrl_to_apple_nvme(ctrl);

	kref_put(&anv->ref, apple_nvme_release);
}

static const struct nvme_ctrl_ops nvme_ctrl_ops = {
	.name = "apple-nvme",
	.module = THIS_MODULE,
	.flags = 0,
	.reg_read32 = apple_nvme_reg_read32,
	.reg_write32 = apple_nvme_reg_write32,
	.reg_read64 = apple_nvme_reg_read64,
	.free_ctrl = apple_nvme_free_ctrl,
	.get_address = apple_nvme_get_address,
	.get_virt_boundary = nvme_get_virt_boundary,
};

static void apple_nvme_async_probe(void *data, async_cookie_t cookie)
{
	struct apple_nvme *anv = data;

	flush_work(&anv->ctrl.reset_work);
	flush_work(&anv->ctrl.scan_work);
	nvme_put_ctrl(&anv->ctrl);
}

static void devm_apple_nvme_put_tag_set(void *data)
{
	struct blk_mq_tag_set *set = data;
	struct apple_nvme *anv = queue_to_apple_nvme(set->driver_data);

	if (!READ_ONCE(anv->quarantined))
		blk_mq_free_tag_set(set);
}

static int apple_nvme_alloc_tagsets(struct apple_nvme *anv)
{
	int ret;

	anv->admin_tagset.ops = &apple_nvme_mq_admin_ops;
	anv->admin_tagset.nr_hw_queues = 1;
	anv->admin_tagset.queue_depth = APPLE_NVME_AQ_MQ_TAG_DEPTH;
	anv->admin_tagset.timeout = NVME_ADMIN_TIMEOUT;
	anv->admin_tagset.numa_node = NUMA_NO_NODE;
	anv->admin_tagset.cmd_size = sizeof(struct apple_nvme_iod);
	anv->admin_tagset.driver_data = &anv->adminq;

	ret = blk_mq_alloc_tag_set(&anv->admin_tagset);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(anv->dev, devm_apple_nvme_put_tag_set,
				       &anv->admin_tagset);
	if (ret)
		return ret;

	anv->tagset.ops = &apple_nvme_mq_ops;
	anv->tagset.nr_hw_queues = 1;
	anv->tagset.nr_maps = 1;
	/*
	 * Tags are used as an index to the NVMMU and must be unique across
	 * both queues. The admin queue gets the first APPLE_NVME_AQ_DEPTH which
	 * must be marked as reserved in the IO queue.
	 */
	anv->tagset.reserved_tags = APPLE_NVME_AQ_DEPTH;
	anv->tagset.queue_depth = anv->hw->max_queue_depth - 1;
	anv->tagset.timeout = NVME_IO_TIMEOUT;
	anv->tagset.numa_node = NUMA_NO_NODE;
	anv->tagset.cmd_size = sizeof(struct apple_nvme_iod);
	anv->tagset.driver_data = &anv->ioq;

	ret = blk_mq_alloc_tag_set(&anv->tagset);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(anv->dev, devm_apple_nvme_put_tag_set,
					&anv->tagset);
	if (ret)
		return ret;

	anv->ctrl.admin_tagset = &anv->admin_tagset;
	anv->ctrl.tagset = &anv->tagset;

	return 0;
}

static void apple_nvme_free_queue(void *data)
{
	struct apple_nvme_queue *q = data;
	struct apple_nvme *anv = queue_to_apple_nvme(q);
	unsigned int depth = apple_nvme_queue_depth(q);
	size_t sq_size = anv->hw->has_lsq_nvmmu ?
		depth * sizeof(struct nvme_command) : depth << APPLE_NVME_IOSQES;

	if (READ_ONCE(anv->quarantined))
		return;
	if (q->tcbs)
		dma_free_coherent(anv->dev,
				  anv->hw->max_queue_depth * sizeof(struct apple_nvmmu_tcb),
				  q->tcbs, q->tcb_dma_addr);
	if (q->sqes)
		dma_free_coherent(anv->dev, sq_size, q->sqes, q->sq_dma_addr);
	if (q->cqes)
		dma_free_coherent(anv->dev, depth * sizeof(struct nvme_completion),
				  q->cqes, q->cq_dma_addr);
}

static int apple_nvme_queue_alloc(struct apple_nvme *anv,
				  struct apple_nvme_queue *q)
{
	unsigned int depth = apple_nvme_queue_depth(q);
	size_t iosq_size;
	int ret;

	ret = devm_add_action_or_reset(anv->dev, apple_nvme_free_queue, q);
	if (ret)
		return ret;
	q->cqes = dma_alloc_coherent(anv->dev,
				     depth * sizeof(struct nvme_completion),
				     &q->cq_dma_addr, GFP_KERNEL);
	if (!q->cqes)
		return -ENOMEM;

	if (anv->hw->has_lsq_nvmmu)
		iosq_size = depth * sizeof(struct nvme_command);
	else
		iosq_size = depth << APPLE_NVME_IOSQES;

	q->sqes = dma_alloc_coherent(anv->dev, iosq_size,
				     &q->sq_dma_addr, GFP_KERNEL);
	if (!q->sqes)
		return -ENOMEM;

	if (anv->hw->has_lsq_nvmmu) {
		/*
		 * We need the maximum queue depth here because the NVMMU only
		 * has a single depth configuration shared between both queues.
		 */
		q->tcbs = dma_alloc_coherent(anv->dev,
					     anv->hw->max_queue_depth *
					     sizeof(struct apple_nvmmu_tcb),
					     &q->tcb_dma_addr, GFP_KERNEL);
		if (!q->tcbs)
			return -ENOMEM;
	}

	/*
	 * initialize phase to make sure the allocated and empty memory
	 * doesn't look like a full cq already.
	 */
	q->cq_phase = 1;
	return 0;
}

static void apple_nvme_detach_genpd(struct apple_nvme *anv)
{
	int i;

	if (anv->pd_count <= 1 || READ_ONCE(anv->quarantined))
		return;

	for (i = anv->pd_count - 1; i >= 0; i--) {
		if (anv->pd_link[i])
			device_link_del(anv->pd_link[i]);
		if (!IS_ERR_OR_NULL(anv->pd_dev[i]))
			dev_pm_domain_detach(anv->pd_dev[i], true);
	}
}

static int apple_nvme_attach_genpd(struct apple_nvme *anv)
{
	struct device *dev = anv->dev;
	int i;

	anv->pd_count = of_count_phandle_with_args(
		dev->of_node, "power-domains", "#power-domain-cells");
	if (anv->pd_count <= 1)
		return 0;

	anv->pd_dev = devm_kcalloc(dev, anv->pd_count, sizeof(*anv->pd_dev),
				   GFP_KERNEL);
	if (!anv->pd_dev)
		return -ENOMEM;

	anv->pd_link = devm_kcalloc(dev, anv->pd_count, sizeof(*anv->pd_link),
				    GFP_KERNEL);
	if (!anv->pd_link)
		return -ENOMEM;

	for (i = 0; i < anv->pd_count; i++) {
		anv->pd_dev[i] = dev_pm_domain_attach_by_id(dev, i);
		if (IS_ERR(anv->pd_dev[i])) {
			apple_nvme_detach_genpd(anv);
			return PTR_ERR(anv->pd_dev[i]);
		}

		anv->pd_link[i] = device_link_add(dev, anv->pd_dev[i],
						  DL_FLAG_STATELESS |
						  DL_FLAG_PM_RUNTIME |
						  DL_FLAG_RPM_ACTIVE);
		if (!anv->pd_link[i]) {
			apple_nvme_detach_genpd(anv);
			return -EINVAL;
		}
	}

	return 0;
}

static void apple_nvme_free_pools(void *data)
{
	struct apple_nvme *anv = data;

	if (READ_ONCE(anv->quarantined))
		return;
	mempool_destroy(anv->iod_mempool);
	dma_pool_destroy(anv->prp_small_pool);
	dma_pool_destroy(anv->prp_page_pool);
}

static void apple_nvme_free_rtkit(void *data)
{
	struct apple_nvme *anv = data;

	apple_nvme_set_reset_ready(anv, false);
	guard(mutex)(&anv->disable_lock);
	if (anv->hw->needs_ioq_registers)
		apple_rtkit_free_retaining_buffers(anv->rtk);
	else
		apple_rtkit_free(anv->rtk);
	anv->rtk = NULL;
}

static void apple_nvme_flush_work(struct work_struct *work)
{
	struct nvme_command c = { };
	struct apple_nvme *anv;
	struct nvme_ns *ns;
	int err;

	anv = container_of(work, struct apple_nvme, flush_dwork.work);
	ns = anv->flush_ns;
	if (WARN_ON_ONCE(!ns))
		return;

	c.common.opcode = nvme_cmd_flush;
	c.common.nsid = cpu_to_le32(anv->flush_ns->head->ns_id);
	err = nvme_submit_sync_cmd(ns->queue, &c, NULL, 0);
	if (err) {
		dev_err(anv->dev, "Deferred flush failed: %d\n", err);
	} else {
		anv->last_flush = jiffies;
	}
}

static struct apple_nvme *apple_nvme_alloc(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct resource *resource;
	struct apple_nvme *anv, *held;
	bool inherited_rtkit;
	int ret;

	resource = platform_get_resource_byname(pdev, IORESOURCE_MEM, "nvme");
	if (!resource)
		return ERR_PTR(-EINVAL);
	mutex_lock(&apple_nvme_quarantine_lock);
	list_for_each_entry(held, &apple_nvme_quarantines, quarantine_node) {
		if (held->controller_start == resource->start) {
			mutex_unlock(&apple_nvme_quarantine_lock);
			return ERR_PTR(dev_err_probe(dev, -EBUSY,
				"DMA ownership is uncertain until platform reset\n"));
		}
	}
	mutex_unlock(&apple_nvme_quarantine_lock);
	anv = kzalloc_obj(*anv);
	if (!anv)
		return ERR_PTR(-ENOMEM);

	anv->dev = get_device(dev);
	kref_init(&anv->ref);
	mutex_init(&anv->disable_lock);
	spin_lock_init(&anv->event_lock);
	INIT_LIST_HEAD(&anv->quarantine_node);
	anv->controller_start = resource->start;
	ret = devm_add_action_or_reset(dev, apple_nvme_put_resource_owner, anv);
	if (ret)
		return ERR_PTR(ret);
	anv->adminq.is_adminq = true;
	platform_set_drvdata(pdev, anv);

	anv->hw = of_device_get_match_data(&pdev->dev);
	if (!anv->hw) {
		ret = -ENODEV;
		goto out_detach_genpd;
	}

	ret = apple_nvme_attach_genpd(anv);
	if (ret < 0) {
		dev_err_probe(dev, ret, "Failed to attach power domains");
		goto out_detach_genpd;
	}
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64))) {
		ret = -ENXIO;
		goto out_detach_genpd;
	}

	anv->irq = platform_get_irq(pdev, 0);
	if (anv->irq < 0) {
		ret = anv->irq;
		goto out_detach_genpd;
	}
	if (!anv->irq) {
		ret = -ENXIO;
		goto out_detach_genpd;
	}

	anv->mmio_coproc = devm_platform_ioremap_resource_byname(pdev, "ans");
	if (IS_ERR(anv->mmio_coproc)) {
		ret = PTR_ERR(anv->mmio_coproc);
		goto out_detach_genpd;
	}
	anv->mmio_nvme = devm_platform_ioremap_resource_byname(pdev, "nvme");
	if (IS_ERR(anv->mmio_nvme)) {
		ret = PTR_ERR(anv->mmio_nvme);
		goto out_detach_genpd;
	}
	if (anv->hw->has_separate_nvmmu) {
		anv->mmio_nvmmu =
			devm_platform_ioremap_resource_byname(pdev, "nvmmu");
		if (IS_ERR(anv->mmio_nvmmu)) {
			ret = PTR_ERR(anv->mmio_nvmmu);
			goto out_detach_genpd;
		}
	} else {
		anv->mmio_nvmmu = anv->mmio_nvme;
	}

	if (anv->hw->has_lsq_nvmmu) {
		anv->adminq.sq_db = anv->mmio_nvme + APPLE_ANS_LINEAR_ASQ_DB;
		anv->adminq.cq_db = anv->mmio_nvme + APPLE_ANS_ACQ_DB;
		anv->ioq.sq_db = anv->mmio_nvme + APPLE_ANS_LINEAR_IOSQ_DB;
		anv->ioq.cq_db = anv->mmio_nvme + APPLE_ANS_IOCQ_DB;
	} else {
		anv->adminq.sq_db = anv->mmio_nvme + NVME_REG_DBS;
		anv->adminq.cq_db = anv->mmio_nvme + APPLE_ANS_ACQ_DB;
		anv->ioq.sq_db = anv->mmio_nvme + NVME_REG_DBS + 8;
		anv->ioq.cq_db = anv->mmio_nvme + APPLE_ANS_IOCQ_DB;
	}

	anv->sart = devm_apple_sart_get(dev);
	if (IS_ERR(anv->sart)) {
		ret = dev_err_probe(dev, PTR_ERR(anv->sart),
				    "Failed to initialize SART");
		goto out_detach_genpd;
	}

	/* T8152's ANS is a virtual PMGR device with no PS register, so the DT has no reset to give. */
	if (anv->hw->no_reset_control)
		anv->reset = NULL;
	else
		anv->reset = devm_reset_control_array_get_exclusive(anv->dev);
	if (IS_ERR(anv->reset)) {
		ret = dev_err_probe(dev, PTR_ERR(anv->reset),
				    "Failed to get reset control");
		goto out_detach_genpd;
	}

	INIT_WORK(&anv->ctrl.reset_work, apple_nvme_reset_work);
	INIT_WORK(&anv->remove_work, apple_nvme_remove_dead_ctrl_work);
	INIT_WORK(&anv->recovery_work, apple_nvme_recovery_work);
	spin_lock_init(&anv->lock);

	ret = apple_nvme_queue_alloc(anv, &anv->adminq);
	if (ret)
		goto out_detach_genpd;
	ret = apple_nvme_queue_alloc(anv, &anv->ioq);
	if (ret)
		goto out_detach_genpd;

	ret = devm_add_action_or_reset(dev, apple_nvme_free_pools, anv);
	if (ret)
		goto out_detach_genpd;
	anv->prp_page_pool = dma_pool_create("prp list page", anv->dev,
					     NVME_CTRL_PAGE_SIZE,
					     NVME_CTRL_PAGE_SIZE, 0);
	if (!anv->prp_page_pool) {
		ret = -ENOMEM;
		goto out_detach_genpd;
	}

	anv->prp_small_pool =
		dma_pool_create("prp list 256", anv->dev, 256, 256, 0);
	if (!anv->prp_small_pool) {
		ret = -ENOMEM;
		goto out_detach_genpd;
	}

	WARN_ON_ONCE(apple_nvme_iod_alloc_size() > PAGE_SIZE);
	anv->iod_mempool =
		mempool_create_kmalloc_pool(1, apple_nvme_iod_alloc_size());
	if (!anv->iod_mempool) {
		ret = -ENOMEM;
		goto out_detach_genpd;
	}
	ret = apple_nvme_alloc_tagsets(anv);
	if (ret)
		goto out_detach_genpd;

	ret = devm_request_irq(anv->dev, anv->irq, apple_nvme_irq, 0,
			       "nvme-apple", anv);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to request IRQ");
		goto out_detach_genpd;
	}

	/*
	 * A post-M4 boot stage may leave RTKit fully running.  Prime the known
	 * system endpoints before mailbox RX starts, otherwise inherited syslog
	 * traffic can race the later reset work and be dropped as undiscovered.
	 */
	inherited_rtkit = apple_nvme_can_adopt_rtkit(anv);
	/* Publish admission before the initializer enables mailbox RX. */
	anv->inherited_rtkit = inherited_rtkit;
	if (anv->hw->needs_ioq_registers)
		dev_dbg(dev, "post-M4 firmware handoff: CC=%#x CSTS=%#x adopt=%d\n",
			readl(anv->mmio_nvme + NVME_REG_CC),
			readl(anv->mmio_nvme + NVME_REG_CSTS), inherited_rtkit);
	if (inherited_rtkit)
		anv->rtk = apple_rtkit_init_adopted(dev, anv, NULL, 0,
						    &apple_nvme_rtkit_ops);
	else
		anv->rtk = apple_rtkit_init(dev, anv, NULL, 0,
					    &apple_nvme_rtkit_ops);
	if (IS_ERR(anv->rtk)) {
		ret = dev_err_probe(dev, PTR_ERR(anv->rtk),
				    "Failed to initialize RTKit");
		goto out_detach_genpd;
	}

	ret = devm_add_action_or_reset(dev, apple_nvme_free_rtkit, anv);
	if (ret)
		goto out_detach_genpd;

	if (anv->hw->has_lsq_nvmmu) {
		ret = devm_blk_crypto_profile_init(dev, &anv->crypto_profile, 0);
		if (ret)
			goto out_detach_genpd;
		anv->crypto_profile.max_dun_bytes_supported = sizeof(u64);
		anv->crypto_profile.key_types_supported =
			BLK_CRYPTO_KEY_TYPE_HW_WRAPPED;
		anv->crypto_profile.modes_supported[BLK_ENCRYPTION_MODE_AES_256_XTS] =
			BIT(ilog2(APPLE_NVME_CRYPTO_DATA_UNIT_SIZE));
		anv->crypto_profile.dev = dev;
	}

	ret = nvme_init_ctrl(&anv->ctrl, anv->dev, &nvme_ctrl_ops,
			     NVME_QUIRK_SKIP_CID_GEN | NVME_QUIRK_IDENTIFY_CNS |
			     NVME_QUIRK_ADMIN_PAGE_ALIGN | anv->hw->quirks);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to initialize nvme_ctrl");
		goto out_detach_genpd;
	}

	/* Released by apple_nvme_free_ctrl(), including asynchronous users. */
	kref_get(&anv->ref);

	if (anv->hw->has_lsq_nvmmu)
		anv->ctrl.crypto_profile = &anv->crypto_profile;

	return anv;
out_detach_genpd:
	apple_nvme_detach_genpd(anv);
	return ERR_PTR(ret);
}

static int apple_nvme_probe(struct platform_device *pdev)
{
	struct apple_nvme *anv;
	int ret;

	anv = apple_nvme_alloc(pdev);
	if (IS_ERR(anv))
		return PTR_ERR(anv);

	ret = nvme_add_ctrl(&anv->ctrl);
	if (ret)
		goto out_put_ctrl;

	anv->ctrl.admin_q = blk_mq_alloc_queue(&anv->admin_tagset, NULL, NULL);
	if (IS_ERR(anv->ctrl.admin_q)) {
		ret = -ENOMEM;
		anv->ctrl.admin_q = NULL;
		goto out_uninit_ctrl;
	}

	if (flush_interval) {
		anv->flush_interval = msecs_to_jiffies(flush_interval);
		anv->flush_ns = NULL;
		anv->last_flush = jiffies - anv->flush_interval;
	}

	INIT_DELAYED_WORK(&anv->flush_dwork, apple_nvme_flush_work);

	apple_nvme_set_reset_ready(anv, true);
	nvme_reset_ctrl(&anv->ctrl);
	async_schedule(apple_nvme_async_probe, anv);

	return 0;

out_uninit_ctrl:
	nvme_uninit_ctrl(&anv->ctrl);
out_put_ctrl:
	nvme_put_ctrl(&anv->ctrl);
	apple_nvme_detach_genpd(anv);
	return ret;
}

static void apple_nvme_remove(struct platform_device *pdev)
{
	struct apple_nvme *anv = platform_get_drvdata(pdev);

	WRITE_ONCE(anv->removing, true);
	apple_nvme_set_reset_ready(anv, false);
	nvme_change_ctrl_state(&anv->ctrl, NVME_CTRL_DELETING);
	cancel_work_sync(&anv->recovery_work);
	cancel_work_sync(&anv->ctrl.reset_work);
	nvme_stop_ctrl(&anv->ctrl);
	cancel_delayed_work_sync(&anv->flush_dwork);
	nvme_remove_namespaces(&anv->ctrl);
	/* A quarantined controller keeps everything it may still DMA into. */
	if (apple_nvme_disable(anv, true))
		return;
	if (anv->ctrl.admin_q && !blk_queue_dying(anv->ctrl.admin_q)) {
		/*
		 * If the controller was reset during removal, it's possible
		 * user requests may be waiting on a stopped queue. Start the
		 * queue to flush these to completion.
		 */
		nvme_unquiesce_admin_queue(&anv->ctrl);
		blk_mq_destroy_queue(anv->ctrl.admin_q);
	}
	nvme_uninit_ctrl(&anv->ctrl);

	/* Post-M4 firmware stays live; see apple_nvme_shutdown(). */
	if (!anv->hw->needs_ioq_registers && apple_rtkit_is_running(anv->rtk)) {
		apple_rtkit_shutdown(anv->rtk);

		writel(0, anv->mmio_coproc + APPLE_ANS_COPROC_CPU_CONTROL);
	}

	WRITE_ONCE(anv->detached, true);
	apple_nvme_detach_genpd(anv);
}

static void apple_nvme_shutdown(struct platform_device *pdev)
{
	struct apple_nvme *anv = platform_get_drvdata(pdev);

	WRITE_ONCE(anv->removing, true);
	apple_nvme_set_reset_ready(anv, false);
	flush_delayed_work(&anv->flush_dwork);
	if (apple_nvme_disable(anv, true))
		return;
	cancel_work_sync(&anv->recovery_work);
	cancel_work_sync(&anv->ctrl.reset_work);
	/*
	 * Post-M4 ANS firmware crashes when asked to perform the RTKit shutdown
	 * handshake during final system shutdown, even after all Linux queues are
	 * quiesced and the controller is left enabled. Leave the co-processor
	 * running until the imminent platform reset. The controller queues are
	 * stopped separately; firmware-owned shared buffers remain retained.
	 */
	if (anv->hw->needs_ioq_registers) {
		dev_dbg(anv->dev,
			"post-M4 shutdown: leaving RTKit running for system reset\n");
		return;
	}

	if (apple_rtkit_is_running(anv->rtk)) {
		apple_rtkit_shutdown(anv->rtk);

		writel(0, anv->mmio_coproc + APPLE_ANS_COPROC_CPU_CONTROL);
	}
}

static int apple_nvme_resume(struct device *dev)
{
	struct apple_nvme *anv = dev_get_drvdata(dev);

	if (anv->hw->needs_ioq_registers) {
		if (READ_ONCE(anv->quarantined))
			return -EIO;
		if (!anv->suspended)
			return 0;
		WRITE_ONCE(anv->suspended, false);
		return nvme_try_sched_reset(&anv->ctrl);
	}
	return nvme_reset_ctrl(&anv->ctrl);
}

/*
 * The ANS stays up across system sleep on post-M4 hardware, and it keeps
 * using its PCIe link while it runs. genpd, however, switches a domain off
 * in the noirq phase once every device attached to it has suspended, no
 * matter what runtime PM says. Put the devices that hold the controller's
 * domains on the awake path so that genpd leaves those domains powered
 * until resume.
 */
static void apple_nvme_keep_link_powered(struct apple_nvme *anv)
{
	int i;

	/* A single domain is attached to the controller device itself. */
	if (anv->pd_count <= 1) {
		device_set_awake_path(anv->dev);
		return;
	}

	for (i = 0; i < anv->pd_count; i++)
		device_set_awake_path(anv->pd_dev[i]);
}

static int apple_nvme_suspend(struct device *dev)
{
	struct apple_nvme *anv = dev_get_drvdata(dev);
	int ret;

	if (anv->hw->needs_ioq_registers) {
		if (!nvme_change_ctrl_state(&anv->ctrl, NVME_CTRL_RESETTING))
			return -EBUSY;
		WRITE_ONCE(anv->suspended, true);
	}
	ret = apple_nvme_disable(anv, true);
	if (ret)
		return ret;
	/*
	 * Only the NVMe controller was disabled. Its owned RTKit session stays
	 * live, so resume rebuilds queues without clearing CPU_CONTROL.RUN.
	 */
	if (anv->hw->needs_ioq_registers) {
		cancel_work_sync(&anv->recovery_work);
		apple_nvme_keep_link_powered(anv);
		return 0;
	}

	if (apple_rtkit_is_running(anv->rtk)) {
		ret = apple_rtkit_shutdown(anv->rtk);

		writel(0, anv->mmio_coproc + APPLE_ANS_COPROC_CPU_CONTROL);
	}

	return ret;
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_nvme_pm_ops, apple_nvme_suspend,
				apple_nvme_resume);

static const struct apple_nvme_hw apple_nvme_t8015_hw = {
	.has_lsq_nvmmu = false,
	.has_queue_count = true,
	.max_queue_depth = 16,
};

static const struct apple_nvme_hw apple_nvme_t8103_hw = {
	.has_lsq_nvmmu = true,
	.has_queue_count = true,
	.max_queue_depth = 64,
};

/*
 * The post-M4 (T8132 and later) I/O queue setup follows Yureka Lilian's T8132
 * support in m1n1 (commit 53f8ee9b54ba, "nvme: support T8132").
 */
static const struct apple_nvme_hw apple_nvme_t8132_hw = {
	.has_lsq_nvmmu = true,
	.has_separate_nvmmu = true,
	.needs_ioq_registers = true,
	.max_queue_depth = 64,
};

static const struct apple_nvme_hw apple_nvme_t8140_hw = {
	.has_lsq_nvmmu = true,
	.has_separate_nvmmu = true,
	.needs_ioq_registers = true,
	.max_queue_depth = 64,
	/*
	 * An fsynced overwrite completed and later reverted after an abrupt
	 * reset; explicit flushes persisted the preceding data. Let the block
	 * layer follow each FUA write with a flush.
	 */
	.quirks = NVME_QUIRK_BROKEN_FUA,
};

/* T8152: NVMMU registers sit in the 0x60000 NVMe window (no separate region; the T8140
 * alias window does not exist and faults). */
static const struct apple_nvme_hw apple_nvme_t8152_hw = {
	/* T8152 register layout, linear submission queue, 64 queue entries. */
	.has_lsq_nvmmu = true,
	.has_queue_count = true,
	.has_separate_nvmmu = false,
	.no_ioq_readback = true,
	.t8152_reg_layout = true,
	.no_reset_control = true,
	.needs_ioq_registers = true,
	.max_queue_depth = 64,
	.quirks = NVME_QUIRK_BROKEN_FUA,
};

static const struct of_device_id apple_nvme_of_match[] = {
	{ .compatible = "apple,t8015-nvme-ans2", .data = &apple_nvme_t8015_hw },
	{ .compatible = "apple,t8103-nvme-ans2", .data = &apple_nvme_t8103_hw },
	{ .compatible = "apple,t8132-nvme-ans2", .data = &apple_nvme_t8132_hw },
	{ .compatible = "apple,t8140-nvme-ans2", .data = &apple_nvme_t8140_hw },
	{ .compatible = "apple,t8152-nvme-ans2", .data = &apple_nvme_t8152_hw },
	{ .compatible = "apple,nvme-ans2", .data = &apple_nvme_t8103_hw },
	{},
};
MODULE_DEVICE_TABLE(of, apple_nvme_of_match);

static struct platform_driver apple_nvme_driver = {
	.driver = {
		.name = "nvme-apple",
		.of_match_table = apple_nvme_of_match,
		.pm = pm_sleep_ptr(&apple_nvme_pm_ops),
	},
	.probe = apple_nvme_probe,
	.remove = apple_nvme_remove,
	.shutdown = apple_nvme_shutdown,
};
module_platform_driver(apple_nvme_driver);

MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple ANS NVM Express device driver");
MODULE_LICENSE("GPL");
