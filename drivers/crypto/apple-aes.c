// SPDX-License-Identifier: GPL-2.0-only
/*
 * Apple SIO AES engine skcipher driver (T6000, T8112, T6040 and T8140).
 *
 * The engine takes commands through a word FIFO: load a key into one of
 * two key slots, load an IV into an IV slot, then run a block operation
 * between two DMA addresses behind the SIO DART. A barrier command with
 * its interrupt bit set raises the completion flag.
 *
 * Only software-supplied keys are used. The hardware key selectors and
 * the key-disable/DKEY apertures are never written by this driver.
 *
 * Data is staged through one DMA bounce buffer so that arbitrary
 * scatterlists, in-place requests and CTR tails need no special casing.
 */

#include <crypto/aes.h>
#include <crypto/engine.h>
#include <crypto/internal/skcipher.h>
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/property.h>
#include <linux/scatterlist.h>
#include <linux/seq_file.h>
#include <linux/unaligned.h>

#define AES_REG_CONTROL		0x08
#define   AES_CONTROL_START	BIT(0)
#define   AES_CONTROL_STOP	BIT(1)
#define   AES_CONTROL_CLEAR_FIFO BIT(2)
#define   AES_CONTROL_RESET	BIT(3)
#define AES_REG_READINESS	0x0c
#define AES_REG_IRQ_STATUS	0x18
#define AES_REG_IRQ_ENABLE	0x1c
#define   AES_IRQ_FIFO_OVERFLOW	BIT(1)
#define   AES_IRQ_UNKNOWN_CMD	BIT(2)
#define   AES_IRQ_FLAG		BIT(5)
#define   AES_IRQ_KEY0_INVALID	BIT(7)
#define   AES_IRQ_KEY0_EMPTY	BIT(11)
#define   AES_IRQ_KEY1_INVALID	BIT(13)
#define   AES_IRQ_KEY1_EMPTY	BIT(17)
#define AES_REG_FIFO_STATUS	0x24
#define   AES_FIFO_LEVEL	GENMASK(15, 8)
#define   AES_FIFO_FULL		BIT(2)
#define   AES_FIFO_EMPTY	BIT(1)
#define AES_REG_CMD_FIFO	0x200

#define AES_CMD_OPCODE		GENMASK(31, 28)
#define   AES_OP_SET_KEY	1
#define   AES_OP_SET_IV		2
#define   AES_OP_CRYPT		5
#define   AES_OP_BARRIER	8

/* SET_KEY */
#define AES_KEY_SLOT		BIT(27)
#define AES_KEY_SELECT		GENMASK(26, 24)
#define   AES_KEY_SELECT_SOFTWARE 0
#define AES_KEY_LEN		GENMASK(23, 22)
#define   AES_KEY_LEN_128	0
#define   AES_KEY_LEN_192	1
#define   AES_KEY_LEN_256	2
#define AES_KEY_ENCRYPT		BIT(20)
#define AES_KEY_KEYGEN		GENMASK(19, 18)
#define AES_KEY_MODE		GENMASK(17, 16)
#define   AES_MODE_ECB		0
#define   AES_MODE_CBC		1
#define   AES_MODE_CTR		2

/* SET_IV */
#define AES_IV_SLOT		GENMASK(27, 26)

/*
 * CRYPT: followed by the upper address word, then the low 32 bits of the
 * source and destination addresses. The upper address word carries
 * address bits 32 and up of the source in its upper half and of the
 * destination in its lower half.
 */
#define AES_CRYPT_KEY_SLOT	BIT(27)
#define AES_CRYPT_IV_SLOT	GENMASK(26, 25)
#define AES_CRYPT_LEN		GENMASK(23, 0)
#define AES_ADDR_HI_SRC_SHIFT	16

/* BARRIER */
#define AES_BARRIER_IRQ		BIT(27)

#define AES_IRQ_ERRORS		(AES_IRQ_FIFO_OVERFLOW | AES_IRQ_UNKNOWN_CMD)
#define AES_IRQ_KEY_ERRORS	(AES_IRQ_KEY0_INVALID | AES_IRQ_KEY1_INVALID)

#define AES_BOUNCE_SIZE		SZ_512K
#define AES_TIMEOUT_MS		1000
#define AES_PRIORITY		150
#define AES_AUTOSUSPEND_MS	50

struct apple_sio_aes_data {
	u32 revision;
	/* DMA address width the upper address word can carry. */
	u8 dma_bits;
};

struct apple_sio_aes {
	struct device *dev;
	void __iomem *base;
	int irq;
	struct crypto_engine *engine;
	struct completion done;
	u32 irq_status;
	bool running;

	const struct apple_sio_aes_data *data;
	void *bounce;
	dma_addr_t bounce_dma;
	u32 bounce_hi;

	u64 requests;
	u64 bytes;
	u64 errors;
	u32 readiness;
	struct dentry *debugfs;
};

struct apple_sio_aes_ctx {
	u8 key[AES_MAX_KEY_SIZE];
	unsigned int keylen;
};

struct apple_sio_aes_reqctx {
	u32 mode;
	bool encrypt;
};

/* The algorithms are global; the first engine to probe serves them. */
static DEFINE_MUTEX(apple_sio_aes_lock);
static struct apple_sio_aes *apple_sio_aes_dev;

static irqreturn_t apple_sio_aes_irq(int irq, void *data)
{
	struct apple_sio_aes *aes = data;
	u32 status;

	if (!READ_ONCE(aes->running))
		return IRQ_NONE;

	status = readl(aes->base + AES_REG_IRQ_STATUS);
	if (!(status & (AES_IRQ_FLAG | AES_IRQ_ERRORS)))
		return IRQ_NONE;

	writel(status, aes->base + AES_REG_IRQ_STATUS);
	aes->irq_status |= status;
	writel(0, aes->base + AES_REG_IRQ_ENABLE);
	complete(&aes->done);

	return IRQ_HANDLED;
}

static void apple_sio_aes_control(struct apple_sio_aes *aes, u32 bits)
{
	u32 val = readl(aes->base + AES_REG_CONTROL);

	writel(val | bits, aes->base + AES_REG_CONTROL);
}

static int apple_sio_aes_push(struct apple_sio_aes *aes, u32 word)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout_atomic(aes->base + AES_REG_FIFO_STATUS, status,
					!(status & AES_FIFO_FULL), 1, 1000);
	if (ret)
		return ret;

	writel(word, aes->base + AES_REG_CMD_FIFO);
	return 0;
}

/* Key and IV bytes go into the FIFO as little-endian 32-bit words. */
static int apple_sio_aes_push_bytes(struct apple_sio_aes *aes, const u8 *buf,
				    unsigned int len)
{
	unsigned int i;
	int ret;

	for (i = 0; i < len; i += 4) {
		ret = apple_sio_aes_push(aes, get_unaligned_le32(buf + i));
		if (ret)
			return ret;
	}

	return 0;
}

static void apple_sio_aes_start(struct apple_sio_aes *aes)
{
	writel(~0U, aes->base + AES_REG_IRQ_STATUS);
	apple_sio_aes_control(aes, AES_CONTROL_CLEAR_FIFO);
	apple_sio_aes_control(aes, AES_CONTROL_RESET);
	apple_sio_aes_control(aes, AES_CONTROL_START);
}

static void apple_sio_aes_stop(struct apple_sio_aes *aes)
{
	writel(0, aes->base + AES_REG_IRQ_ENABLE);
	apple_sio_aes_control(aes, AES_CONTROL_STOP);
	writel(~0U, aes->base + AES_REG_IRQ_STATUS);
}

/* Run one block operation in place on the first @len bytes of the bounce buffer. */
static int apple_sio_aes_run(struct apple_sio_aes *aes,
			     const struct apple_sio_aes_ctx *ctx,
			     const struct apple_sio_aes_reqctx *rctx,
			     const u8 *iv, unsigned int len)
{
	u32 keylen, status, cmd;
	unsigned long left;
	int ret;

	switch (ctx->keylen) {
	case AES_KEYSIZE_128:
		keylen = AES_KEY_LEN_128;
		break;
	case AES_KEYSIZE_192:
		keylen = AES_KEY_LEN_192;
		break;
	default:
		keylen = AES_KEY_LEN_256;
		break;
	}

	reinit_completion(&aes->done);
	aes->irq_status = 0;
	apple_sio_aes_start(aes);
	writel(AES_IRQ_FLAG | AES_IRQ_ERRORS, aes->base + AES_REG_IRQ_ENABLE);

	cmd = FIELD_PREP(AES_CMD_OPCODE, AES_OP_SET_KEY) |
	      FIELD_PREP(AES_KEY_SLOT, 0) |
	      FIELD_PREP(AES_KEY_SELECT, AES_KEY_SELECT_SOFTWARE) |
	      FIELD_PREP(AES_KEY_LEN, keylen) |
	      FIELD_PREP(AES_KEY_ENCRYPT, rctx->encrypt) |
	      FIELD_PREP(AES_KEY_KEYGEN, 0) |
	      FIELD_PREP(AES_KEY_MODE, rctx->mode);
	ret = apple_sio_aes_push(aes, cmd);
	if (!ret)
		ret = apple_sio_aes_push_bytes(aes, ctx->key, ctx->keylen);

	if (!ret && rctx->mode != AES_MODE_ECB) {
		cmd = FIELD_PREP(AES_CMD_OPCODE, AES_OP_SET_IV) |
		      FIELD_PREP(AES_IV_SLOT, 0);
		ret = apple_sio_aes_push(aes, cmd);
		if (!ret)
			ret = apple_sio_aes_push_bytes(aes, iv, AES_BLOCK_SIZE);
	}

	if (!ret) {
		cmd = FIELD_PREP(AES_CMD_OPCODE, AES_OP_CRYPT) |
		      FIELD_PREP(AES_CRYPT_KEY_SLOT, 0) |
		      FIELD_PREP(AES_CRYPT_IV_SLOT, 0) |
		      FIELD_PREP(AES_CRYPT_LEN, len);
		dma_sync_single_for_device(aes->dev, aes->bounce_dma, len,
					   DMA_BIDIRECTIONAL);
		ret = apple_sio_aes_push(aes, cmd);
		if (!ret)
			ret = apple_sio_aes_push(aes, aes->bounce_hi);
		if (!ret)
			ret = apple_sio_aes_push(aes, lower_32_bits(aes->bounce_dma));
		if (!ret)
			ret = apple_sio_aes_push(aes, lower_32_bits(aes->bounce_dma));
	}

	if (!ret) {
		WRITE_ONCE(aes->running, true);
		ret = apple_sio_aes_push(aes, FIELD_PREP(AES_CMD_OPCODE, AES_OP_BARRIER) |
					      AES_BARRIER_IRQ);
	}

	if (ret) {
		dev_err(aes->dev, "command FIFO stuck (fifo %#x)\n",
			readl(aes->base + AES_REG_FIFO_STATUS));
	} else {
		left = wait_for_completion_timeout(&aes->done,
						   msecs_to_jiffies(AES_TIMEOUT_MS));
		status = aes->irq_status | readl(aes->base + AES_REG_IRQ_STATUS);
		if (!left) {
			dev_err(aes->dev, "timeout: irq %#x fifo %#x\n", status,
				readl(aes->base + AES_REG_FIFO_STATUS));
			ret = -ETIMEDOUT;
		} else if (status & (AES_IRQ_ERRORS | AES_IRQ_KEY_ERRORS)) {
			dev_err(aes->dev, "engine error: irq %#x\n", status);
			ret = -EIO;
		}
	}

	WRITE_ONCE(aes->running, false);
	synchronize_irq(aes->irq);
	apple_sio_aes_stop(aes);
	dma_sync_single_for_cpu(aes->dev, aes->bounce_dma, len, DMA_BIDIRECTIONAL);

	return ret;
}

/* Add @n to a 128-bit big-endian counter. */
static void apple_sio_aes_ctr_add(u8 *ctr, u32 n)
{
	int i;
	u32 c;

	for (i = AES_BLOCK_SIZE - 4; i >= 0 && n; i -= 4) {
		c = get_unaligned_be32(ctr + i);
		put_unaligned_be32(c + n, ctr + i);
		n = (c + n < c) ? 1 : 0;
	}
}

static int apple_sio_aes_do_one(struct crypto_engine *engine, void *areq)
{
	struct skcipher_request *req = container_of(areq, struct skcipher_request, base);
	struct crypto_skcipher *tfm = crypto_skcipher_reqtfm(req);
	struct apple_sio_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	struct apple_sio_aes_reqctx *rctx = skcipher_request_ctx(req);
	struct apple_sio_aes *aes = apple_sio_aes_dev;
	unsigned int total = req->cryptlen, off = 0, n, run;
	int src_nents, dst_nents;
	u8 iv[AES_BLOCK_SIZE] = { }, next[AES_BLOCK_SIZE];
	int ret;

	if (rctx->mode != AES_MODE_ECB)
		memcpy(iv, req->iv, AES_BLOCK_SIZE);

	src_nents = sg_nents_for_len(req->src, total);
	dst_nents = sg_nents_for_len(req->dst, total);
	if (src_nents < 0 || dst_nents < 0) {
		ret = -EINVAL;
		goto out;
	}

	ret = pm_runtime_resume_and_get(aes->dev);
	if (ret < 0)
		goto out;

	while (off < total) {
		n = min(total - off, (unsigned int)AES_BOUNCE_SIZE);
		if (rctx->mode == AES_MODE_CTR) {
			/* Never let the low counter word wrap inside one command. */
			u32 low = get_unaligned_be32(iv + AES_BLOCK_SIZE - 4);
			u64 room = ((u64)U32_MAX - low + 1) * AES_BLOCK_SIZE;

			if (n > room)
				n = room;
		}
		run = round_up(n, AES_BLOCK_SIZE);

		sg_pcopy_to_buffer(req->src, src_nents, aes->bounce, n, off);
		if (run > n)
			memset(aes->bounce + n, 0, run - n);
		if (rctx->mode == AES_MODE_CBC && !rctx->encrypt)
			memcpy(next, aes->bounce + run - AES_BLOCK_SIZE, AES_BLOCK_SIZE);

		ret = apple_sio_aes_run(aes, ctx, rctx, iv, run);
		if (ret)
			break;

		if (rctx->mode == AES_MODE_CBC && rctx->encrypt)
			memcpy(next, aes->bounce + run - AES_BLOCK_SIZE, AES_BLOCK_SIZE);
		sg_pcopy_from_buffer(req->dst, dst_nents, aes->bounce, n, off);

		if (rctx->mode == AES_MODE_CBC)
			memcpy(iv, next, AES_BLOCK_SIZE);
		else if (rctx->mode == AES_MODE_CTR)
			apple_sio_aes_ctr_add(iv, run / AES_BLOCK_SIZE);

		off += n;
		aes->bytes += n;
	}

	memzero_explicit(aes->bounce, min_t(unsigned int, total, AES_BOUNCE_SIZE));
	pm_runtime_put_autosuspend(aes->dev);

	if (!ret && rctx->mode != AES_MODE_ECB)
		memcpy(req->iv, iv, AES_BLOCK_SIZE);

out:
	aes->requests++;
	if (ret)
		aes->errors++;
	memzero_explicit(next, sizeof(next));

	/*
	 * The request is done in the engine kthread, but crypto_finalize_*()
	 * runs the completion callback, which expects softirq context.
	 */
	local_bh_disable();
	crypto_finalize_skcipher_request(engine, req, ret);
	local_bh_enable();
	return 0;
}

static int apple_sio_aes_setkey(struct crypto_skcipher *tfm, const u8 *key,
				unsigned int keylen)
{
	struct apple_sio_aes_ctx *ctx = crypto_skcipher_ctx(tfm);
	int ret;

	ret = aes_check_keylen(keylen);
	if (ret)
		return ret;

	memcpy(ctx->key, key, keylen);
	ctx->keylen = keylen;
	return 0;
}

static void apple_sio_aes_exit_tfm(struct crypto_skcipher *tfm)
{
	struct apple_sio_aes_ctx *ctx = crypto_skcipher_ctx(tfm);

	memzero_explicit(ctx, sizeof(*ctx));
}

static int apple_sio_aes_queue(struct skcipher_request *req, u32 mode, bool encrypt)
{
	struct apple_sio_aes_reqctx *rctx = skcipher_request_ctx(req);
	struct apple_sio_aes *aes = apple_sio_aes_dev;

	if (mode != AES_MODE_CTR && !IS_ALIGNED(req->cryptlen, AES_BLOCK_SIZE))
		return -EINVAL;
	if (!aes)
		return -ENODEV;

	rctx->mode = mode;
	rctx->encrypt = encrypt;
	return crypto_transfer_skcipher_request_to_engine(aes->engine, req);
}

static int apple_sio_aes_ecb_encrypt(struct skcipher_request *req)
{
	return apple_sio_aes_queue(req, AES_MODE_ECB, true);
}

static int apple_sio_aes_ecb_decrypt(struct skcipher_request *req)
{
	return apple_sio_aes_queue(req, AES_MODE_ECB, false);
}

static int apple_sio_aes_cbc_encrypt(struct skcipher_request *req)
{
	return apple_sio_aes_queue(req, AES_MODE_CBC, true);
}

static int apple_sio_aes_cbc_decrypt(struct skcipher_request *req)
{
	return apple_sio_aes_queue(req, AES_MODE_CBC, false);
}

static int apple_sio_aes_ctr_crypt(struct skcipher_request *req)
{
	return apple_sio_aes_queue(req, AES_MODE_CTR, true);
}

static int apple_sio_aes_init_tfm(struct crypto_skcipher *tfm)
{
	crypto_skcipher_set_reqsize(tfm, sizeof(struct apple_sio_aes_reqctx));
	return 0;
}

#define APPLE_SIO_AES_ALG(_name, _drv, _bs, _ivsize, _enc, _dec)		\
	{								\
		.base = {						\
			.base.cra_name = _name,				\
			.base.cra_driver_name = _drv,			\
			.base.cra_priority = AES_PRIORITY,		\
			.base.cra_flags = CRYPTO_ALG_ASYNC |		\
					  CRYPTO_ALG_KERN_DRIVER_ONLY,	\
			.base.cra_blocksize = _bs,			\
			.base.cra_ctxsize = sizeof(struct apple_sio_aes_ctx), \
			.base.cra_module = THIS_MODULE,			\
			.min_keysize = AES_MIN_KEY_SIZE,		\
			.max_keysize = AES_MAX_KEY_SIZE,		\
			.ivsize = _ivsize,				\
			.chunksize = AES_BLOCK_SIZE,			\
			.setkey = apple_sio_aes_setkey,			\
			.encrypt = _enc,				\
			.decrypt = _dec,				\
			.init = apple_sio_aes_init_tfm,			\
			.exit = apple_sio_aes_exit_tfm,			\
		},							\
		.op.do_one_request = apple_sio_aes_do_one,		\
	}

static struct skcipher_engine_alg apple_sio_aes_algs[] = {
	APPLE_SIO_AES_ALG("ecb(aes)", "ecb-aes-apple-sio", AES_BLOCK_SIZE, 0,
			  apple_sio_aes_ecb_encrypt, apple_sio_aes_ecb_decrypt),
	APPLE_SIO_AES_ALG("cbc(aes)", "cbc-aes-apple-sio", AES_BLOCK_SIZE,
			  AES_BLOCK_SIZE, apple_sio_aes_cbc_encrypt,
			  apple_sio_aes_cbc_decrypt),
	APPLE_SIO_AES_ALG("ctr(aes)", "ctr-aes-apple-sio", 1, AES_BLOCK_SIZE,
			  apple_sio_aes_ctr_crypt, apple_sio_aes_ctr_crypt),
};

static int apple_sio_aes_stats_show(struct seq_file *s, void *unused)
{
	struct apple_sio_aes *aes = s->private;

	seq_printf(s, "revision: %u\n", aes->data->revision);
	seq_printf(s, "bounce_dma: %pad\n", &aes->bounce_dma);
	seq_printf(s, "readiness_at_probe: %#010x\n", aes->readiness);
	seq_printf(s, "requests: %llu\n", aes->requests);
	seq_printf(s, "bytes: %llu\n", aes->bytes);
	seq_printf(s, "errors: %llu\n", aes->errors);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(apple_sio_aes_stats);

static void apple_sio_aes_engine_exit(void *engine)
{
	crypto_engine_exit(engine);
}

static void apple_sio_aes_free_bounce(void *data)
{
	struct apple_sio_aes *aes = data;

	dma_free_noncoherent(aes->dev, AES_BOUNCE_SIZE, aes->bounce,
			     aes->bounce_dma, DMA_BIDIRECTIONAL);
}

/*
 * Build the upper address word for an in-place operation on @addr. The
 * engine drops address bits above its DMA width, so refuse such an
 * address instead of letting it alias a lower one.
 */
static int apple_sio_aes_addr_hi(const struct apple_sio_aes_data *data,
				 dma_addr_t addr, u32 *hi)
{
	u32 upper = upper_32_bits(addr);

	if (data->dma_bits < 64 && (u64)addr >> data->dma_bits)
		return -ERANGE;

	*hi = (upper << AES_ADDR_HI_SRC_SHIFT) | upper;
	return 0;
}

static int apple_sio_aes_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct apple_sio_aes *aes;
	int ret;

	if (!dev->pm_domain)
		return dev_err_probe(dev, -EINVAL, "missing AES power domain\n");
	if (!iommu_get_domain_for_dev(dev))
		return dev_err_probe(dev, -ENODEV, "missing SIO DART domain\n");

	aes = devm_kzalloc(dev, sizeof(*aes), GFP_KERNEL);
	if (!aes)
		return -ENOMEM;
	aes->dev = dev;
	aes->data = device_get_match_data(dev);
	if (!aes->data)
		return -ENODEV;
	init_completion(&aes->done);
	platform_set_drvdata(pdev, aes);

	/* Bank 0 is the engine. The key-disable and DKEY banks are not mapped. */
	aes->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(aes->base))
		return PTR_ERR(aes->base);

	aes->irq = platform_get_irq(pdev, 0);
	if (aes->irq < 0)
		return aes->irq;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(aes->data->dma_bits));
	if (ret)
		return dev_err_probe(dev, ret, "no %u-bit DMA\n", aes->data->dma_bits);

	aes->bounce = dma_alloc_noncoherent(dev, AES_BOUNCE_SIZE, &aes->bounce_dma,
					    DMA_BIDIRECTIONAL, GFP_KERNEL);
	if (!aes->bounce)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, apple_sio_aes_free_bounce, aes);
	if (ret)
		return ret;
	ret = apple_sio_aes_addr_hi(aes->data, aes->bounce_dma, &aes->bounce_hi);
	if (ret)
		return dev_err_probe(dev, ret, "bounce buffer at %pad is out of reach\n",
				     &aes->bounce_dma);

	ret = devm_request_irq(dev, aes->irq, apple_sio_aes_irq, 0,
			       dev_name(dev), aes);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, AES_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret < 0)
		return ret;
	aes->readiness = readl(aes->base + AES_REG_READINESS);
	writel(0, aes->base + AES_REG_IRQ_ENABLE);
	writel(~0U, aes->base + AES_REG_IRQ_STATUS);
	pm_runtime_put_autosuspend(dev);

	aes->engine = crypto_engine_alloc_init(dev, true);
	if (!aes->engine)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, apple_sio_aes_engine_exit, aes->engine);
	if (ret)
		return ret;
	ret = crypto_engine_start(aes->engine);
	if (ret)
		return ret;

	mutex_lock(&apple_sio_aes_lock);
	if (apple_sio_aes_dev) {
		mutex_unlock(&apple_sio_aes_lock);
		return dev_err_probe(dev, -EBUSY, "another AES engine is registered\n");
	}
	apple_sio_aes_dev = aes;
	ret = crypto_engine_register_skciphers(apple_sio_aes_algs,
					       ARRAY_SIZE(apple_sio_aes_algs));
	if (ret)
		apple_sio_aes_dev = NULL;
	mutex_unlock(&apple_sio_aes_lock);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register algorithms\n");

	aes->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("stats", 0400, aes->debugfs, aes, &apple_sio_aes_stats_fops);

	dev_info(dev, "AES engine rev %u ready (readiness %#010x, %u-bit DMA, ECB/CBC/CTR, priority %d)\n",
		 aes->data->revision, aes->readiness, aes->data->dma_bits,
		 AES_PRIORITY);
	return 0;
}

static void apple_sio_aes_remove(struct platform_device *pdev)
{
	struct apple_sio_aes *aes = platform_get_drvdata(pdev);

	debugfs_remove_recursive(aes->debugfs);
	mutex_lock(&apple_sio_aes_lock);
	crypto_engine_unregister_skciphers(apple_sio_aes_algs,
					   ARRAY_SIZE(apple_sio_aes_algs));
	apple_sio_aes_dev = NULL;
	mutex_unlock(&apple_sio_aes_lock);
}

static int apple_sio_aes_suspend(struct device *dev)
{
	struct apple_sio_aes *aes = dev_get_drvdata(dev);
	int ret;

	ret = crypto_engine_stop(aes->engine);
	if (ret)
		return ret;
	return pm_runtime_force_suspend(dev);
}

static int apple_sio_aes_resume(struct device *dev)
{
	struct apple_sio_aes *aes = dev_get_drvdata(dev);
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;
	return crypto_engine_start(aes->engine);
}

static DEFINE_SIMPLE_DEV_PM_OPS(apple_sio_aes_pm, apple_sio_aes_suspend,
				apple_sio_aes_resume);

/*
 * Revision 3 decodes 36 address bits; this driver keeps it on a 32-bit
 * mask, so its upper address word stays zero. Revision 5 decodes 42
 * address bits, ten in each half of the upper address word, and its SIO
 * DART window starts above 4 GiB, so that word is always in use there.
 */
static const struct apple_sio_aes_data apple_sio_aes_v3 = {
	.revision = 3,
	.dma_bits = 32,
};

static const struct apple_sio_aes_data apple_sio_aes_v5 = {
	.revision = 5,
	.dma_bits = 42,
};

static const struct of_device_id apple_sio_aes_of_match[] = {
	{ .compatible = "apple,t6000-aes", .data = &apple_sio_aes_v3 },
	{ .compatible = "apple,t8112-aes", .data = &apple_sio_aes_v3 },
	{ .compatible = "apple,t6041-aes", .data = &apple_sio_aes_v5 },
	{ .compatible = "apple,t8140-aes", .data = &apple_sio_aes_v5 },
	{ .compatible = "apple,t8142-aes", .data = &apple_sio_aes_v5 },
	{ }
};
MODULE_DEVICE_TABLE(of, apple_sio_aes_of_match);

static struct platform_driver apple_sio_aes_driver = {
	.probe = apple_sio_aes_probe,
	.remove = apple_sio_aes_remove,
	.driver = {
		.name = "apple-sio-aes-crypto",
		.of_match_table = apple_sio_aes_of_match,
		.pm = pm_sleep_ptr(&apple_sio_aes_pm),
	},
};
module_platform_driver(apple_sio_aes_driver);

MODULE_DESCRIPTION("Apple SIO AES engine skcipher driver");
MODULE_AUTHOR("Ryan Murray <ryan@aurorasilicon.org>");
MODULE_LICENSE("GPL");
