// SPDX-License-Identifier: GPL-2.0
//
// Apple SoC SPI device driver
//
// Copyright The Asahi Linux Contributors
//
// Based on spi-sifive.c, Copyright 2018 SiFive, Inc.

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/spi/spi.h>
#include <linux/spinlock.h>

#define APPLE_SPI_CTRL			0x000
#define APPLE_SPI_CTRL_RUN		BIT(0)
#define APPLE_SPI_CTRL_TX_RESET		BIT(2)
#define APPLE_SPI_CTRL_RX_RESET		BIT(3)

#define APPLE_SPI_CFG			0x004
#define APPLE_SPI_CFG_CPHA		BIT(1)
#define APPLE_SPI_CFG_CPOL		BIT(2)
#define APPLE_SPI_CFG_MODE		GENMASK(6, 5)
#define APPLE_SPI_CFG_MODE_POLLED	0
#define APPLE_SPI_CFG_MODE_IRQ		1
#define APPLE_SPI_CFG_MODE_DMA		2
#define APPLE_SPI_CFG_IE_RXCOMPLETE	BIT(7)
#define APPLE_SPI_CFG_IE_TXRXTHRESH	BIT(8)
#define APPLE_SPI_CFG_LSB_FIRST		BIT(13)
#define APPLE_SPI_CFG_WORD_SIZE		GENMASK(16, 15)
#define APPLE_SPI_CFG_WORD_SIZE_8B	0
#define APPLE_SPI_CFG_WORD_SIZE_16B	1
#define APPLE_SPI_CFG_WORD_SIZE_32B	2
#define APPLE_SPI_CFG_FIFO_THRESH	GENMASK(18, 17)
#define APPLE_SPI_CFG_FIFO_THRESH_8B	0
#define APPLE_SPI_CFG_FIFO_THRESH_4B	1
#define APPLE_SPI_CFG_FIFO_THRESH_1B	2
#define APPLE_SPI_CFG_IE_TXCOMPLETE	BIT(21)

#define APPLE_SPI_STATUS		0x008
#define APPLE_SPI_STATUS_RXCOMPLETE	BIT(0)
#define APPLE_SPI_STATUS_TXRXTHRESH	BIT(1)
#define APPLE_SPI_STATUS_TXCOMPLETE	BIT(2)

#define APPLE_SPI_PIN			0x00c
#define APPLE_SPI_PIN_KEEP_MOSI		BIT(0)
#define APPLE_SPI_PIN_CS		BIT(1)

#define APPLE_SPI_TXDATA		0x010
#define APPLE_SPI_RXDATA		0x020
#define APPLE_SPI_CLKDIV		0x030
#define APPLE_SPI_CLKDIV_MAX		0x7ff
#define APPLE_SPI_RXCNT			0x034
#define APPLE_SPI_WORD_DELAY		0x038
#define APPLE_SPI_TXCNT			0x04c

#define APPLE_SPI_FIFOSTAT		0x10c
#define APPLE_SPI_FIFOSTAT_TXFULL	BIT(4)
#define APPLE_SPI_FIFOSTAT_LEVEL_TX	GENMASK(15, 8)
#define APPLE_SPI_FIFOSTAT_RXEMPTY	BIT(20)
#define APPLE_SPI_FIFOSTAT_LEVEL_RX	GENMASK(31, 24)

#define APPLE_SPI_IE_XFER		0x130
#define APPLE_SPI_IF_XFER		0x134
#define APPLE_SPI_XFER_RXCOMPLETE	BIT(0)
#define APPLE_SPI_XFER_TXCOMPLETE	BIT(1)

#define APPLE_SPI_IE_FIFO		0x138
#define APPLE_SPI_IF_FIFO		0x13c
#define APPLE_SPI_FIFO_RXTHRESH		BIT(4)
#define APPLE_SPI_FIFO_TXTHRESH		BIT(5)
#define APPLE_SPI_FIFO_RXFULL		BIT(8)
#define APPLE_SPI_FIFO_TXEMPTY		BIT(9)
#define APPLE_SPI_FIFO_RXUNDERRUN	BIT(16)
#define APPLE_SPI_FIFO_TXOVERFLOW	BIT(17)

#define APPLE_SPI_SHIFTCFG		0x150
#define APPLE_SPI_SHIFTCFG_CLK_ENABLE	BIT(0)
#define APPLE_SPI_SHIFTCFG_CS_ENABLE	BIT(1)
#define APPLE_SPI_SHIFTCFG_AND_CLK_DATA	BIT(8)
#define APPLE_SPI_SHIFTCFG_CS_AS_DATA	BIT(9)
#define APPLE_SPI_SHIFTCFG_TX_ENABLE	BIT(10)
#define APPLE_SPI_SHIFTCFG_RX_ENABLE	BIT(11)
#define APPLE_SPI_SHIFTCFG_BITS		GENMASK(21, 16)
#define APPLE_SPI_SHIFTCFG_OVERRIDE_CS	BIT(24)

#define APPLE_SPI_PINCFG		0x154
#define APPLE_SPI_PINCFG_KEEP_CLK	BIT(0)
#define APPLE_SPI_PINCFG_KEEP_CS	BIT(1)
#define APPLE_SPI_PINCFG_KEEP_MOSI	BIT(2)
#define APPLE_SPI_PINCFG_CLK_IDLE_VAL	BIT(8)
#define APPLE_SPI_PINCFG_CS_IDLE_VAL	BIT(9)
#define APPLE_SPI_PINCFG_MOSI_IDLE_VAL	BIT(10)

#define APPLE_SPI_DELAY_PRE		0x160
#define APPLE_SPI_DELAY_POST		0x168
#define APPLE_SPI_DELAY_POST_CYCLES	0x194
#define APPLE_SPI_DELAY_ENABLE		BIT(0)
#define APPLE_SPI_DELAY_NO_INTERBYTE	BIT(1)
#define APPLE_SPI_DELAY_SET_SCK		BIT(4)
#define APPLE_SPI_DELAY_SET_CS		BIT(5)
#define APPLE_SPI_DELAY_SET_MOSI	BIT(6)
#define APPLE_SPI_DELAY_SCK_VAL		BIT(8)
#define APPLE_SPI_DELAY_CS_VAL		BIT(10)
#define APPLE_SPI_DELAY_PRE_ARM		BIT(11)
#define APPLE_SPI_DELAY_MOSI_VAL	BIT(12)
#define APPLE_SPI_DELAY_CYCLES		GENMASK(31, 16)

/*
 * Hardware CS delays: the pre-delay asserts the active-low CS and the
 * post-delay releases it.  Bit 11 is undocumented and named for its observed
 * role only.
 */
#define APPLE_SPI_DELAY_PRE_FLAGS	(APPLE_SPI_DELAY_PRE_ARM | \
					 APPLE_SPI_DELAY_SET_CS | \
					 APPLE_SPI_DELAY_NO_INTERBYTE | \
					 APPLE_SPI_DELAY_ENABLE)
#define APPLE_SPI_DELAY_POST_FLAGS	(APPLE_SPI_DELAY_CS_VAL | \
					 APPLE_SPI_DELAY_SET_CS | \
					 APPLE_SPI_DELAY_NO_INTERBYTE | \
					 APPLE_SPI_DELAY_ENABLE)

#define APPLE_SPI_FIFO_DEPTH		16

/*
 * The slowest refclock available is 24MHz, the highest divider is 0x7ff,
 * the largest word size is 32 bits, the FIFO depth is 16, the maximum
 * intra-word delay is 0xffff refclocks. So the maximum time a transfer
 * cycle can take is:
 *
 * (0x7ff * 32 + 0xffff) * 16 / 24e6 Hz ~= 87ms
 *
 * Double it and round it up to 200ms for good measure.
 */
#define APPLE_SPI_TIMEOUT_MS		200

struct apple_spi {
	void __iomem      *regs;        /* MMIO register address */
	struct clk        *clk;         /* bus clock */
	struct completion done;         /* wake-up from interrupt */
	int               irq;

	/*
	 * An interrupt-driven transfer in flight. The interrupt handler moves
	 * the data, so a refill does not wait for this thread to be scheduled.
	 */
	spinlock_t        lock;         /* protects the state below and IE */
	bool              active;
	bool              threaded_tx; /* transmit-only fallback */
	const void        *tx_ptr;
	void              *rx_ptr;
	u32               remaining_tx;
	u32               remaining_rx;
	u32               xfer_flags;   /* completions still awaited */
	unsigned int      bytes_per_word;
};

static inline void reg_write(struct apple_spi *spi, int offset, u32 value)
{
	writel_relaxed(value, spi->regs + offset);
}

static inline u32 reg_read(struct apple_spi *spi, int offset)
{
	return readl_relaxed(spi->regs + offset);
}

static inline void reg_mask(struct apple_spi *spi, int offset, u32 clear, u32 set)
{
	u32 val = reg_read(spi, offset);

	val &= ~clear;
	val |= set;
	reg_write(spi, offset, val);
}

static void apple_spi_init(struct apple_spi *spi)
{
	/* Set CS high (inactive) and disable override and auto-CS */
	reg_write(spi, APPLE_SPI_PIN, APPLE_SPI_PIN_CS);
	reg_mask(spi, APPLE_SPI_SHIFTCFG, APPLE_SPI_SHIFTCFG_OVERRIDE_CS, 0);
	reg_mask(spi, APPLE_SPI_PINCFG, APPLE_SPI_PINCFG_CS_IDLE_VAL, APPLE_SPI_PINCFG_KEEP_CS);

	/* Reset FIFOs */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RX_RESET | APPLE_SPI_CTRL_TX_RESET);

	/* Configure defaults */
	reg_write(spi, APPLE_SPI_CFG,
		  FIELD_PREP(APPLE_SPI_CFG_FIFO_THRESH, APPLE_SPI_CFG_FIFO_THRESH_8B) |
		  FIELD_PREP(APPLE_SPI_CFG_MODE, APPLE_SPI_CFG_MODE_IRQ) |
		  FIELD_PREP(APPLE_SPI_CFG_WORD_SIZE, APPLE_SPI_CFG_WORD_SIZE_8B));

	/* Disable IRQs */
	reg_write(spi, APPLE_SPI_IE_FIFO, 0);
	reg_write(spi, APPLE_SPI_IE_XFER, 0);

	/* Disable delays */
	reg_write(spi, APPLE_SPI_DELAY_PRE, 0);
	reg_write(spi, APPLE_SPI_DELAY_POST, 0);
	reg_write(spi, APPLE_SPI_DELAY_POST_CYCLES, 0);
}

static void apple_spi_set_cs(struct spi_device *device, bool is_high)
{
	struct apple_spi *spi = spi_controller_get_devdata(device->controller);

	reg_mask(spi, APPLE_SPI_PIN, APPLE_SPI_PIN_CS, is_high ? APPLE_SPI_PIN_CS : 0);
}

static int apple_spi_delay_cycles(struct apple_spi *spi, struct spi_delay *delay,
				  u32 *cycles)
{
	u64 value;
	int ns;

	ns = spi_delay_to_ns(delay, NULL);
	if (ns < 0)
		return ns;

	value = DIV_ROUND_UP_ULL((u64)clk_get_rate(spi->clk) * ns,
				  1000000000ULL);
	if (value > FIELD_MAX(APPLE_SPI_DELAY_CYCLES))
		return -ERANGE;

	*cycles = value;
	return 0;
}

static int apple_spi_cs_timing_cycles(struct apple_spi *spi,
				      struct spi_device *device,
				      u32 *setup_cycles, u32 *hold_cycles)
{
	int ret;

	/* This controller path has no hardware representation for inactive time. */
	if (device->cs_inactive.value)
		return -EOPNOTSUPP;

	ret = apple_spi_delay_cycles(spi, &device->cs_setup, setup_cycles);
	if (ret)
		return ret;
	return apple_spi_delay_cycles(spi, &device->cs_hold, hold_cycles);
}

static void apple_spi_write_cs_timing(struct apple_spi *spi, u32 setup_cycles,
				      u32 hold_cycles)
{
	if (!setup_cycles && !hold_cycles) {
		reg_mask(spi, APPLE_SPI_SHIFTCFG,
			 APPLE_SPI_SHIFTCFG_OVERRIDE_CS, 0);
		reg_write(spi, APPLE_SPI_DELAY_PRE, 0);
		reg_write(spi, APPLE_SPI_DELAY_POST, 0);
		reg_write(spi, APPLE_SPI_DELAY_POST_CYCLES, 0);
		return;
	}

	reg_write(spi, APPLE_SPI_DELAY_PRE,
		  FIELD_PREP(APPLE_SPI_DELAY_CYCLES, setup_cycles) |
		  APPLE_SPI_DELAY_PRE_FLAGS);
	reg_write(spi, APPLE_SPI_DELAY_POST_CYCLES, hold_cycles);
	reg_write(spi, APPLE_SPI_DELAY_POST,
		  FIELD_PREP(APPLE_SPI_DELAY_CYCLES, hold_cycles) |
		  APPLE_SPI_DELAY_POST_FLAGS);
	reg_mask(spi, APPLE_SPI_SHIFTCFG, 0,
		 APPLE_SPI_SHIFTCFG_OVERRIDE_CS);
}

/*
 * Only validate the delays here. The registers are programmed for each
 * message in apple_spi_prepare_message(): their contents are lost when
 * runtime PM gates the controller's power domain between messages, and
 * they are shared by all devices on the bus.
 */
static int apple_spi_set_cs_timing(struct spi_device *device)
{
	struct apple_spi *spi = spi_controller_get_devdata(device->controller);
	u32 setup_cycles, hold_cycles;
	int ret;

	ret = apple_spi_cs_timing_cycles(spi, device, &setup_cycles,
					 &hold_cycles);
	if (ret)
		return ret;

	dev_dbg(&device->dev, "hardware CS setup=%u hold=%u cycles\n",
		setup_cycles, hold_cycles);
	return 0;
}

static int apple_spi_prepare_message(struct spi_controller *ctlr, struct spi_message *msg)
{
	struct apple_spi *spi = spi_controller_get_devdata(ctlr);
	struct spi_device *device = msg->spi;

	u32 cfg = ((device->mode & SPI_CPHA ? APPLE_SPI_CFG_CPHA : 0) |
		   (device->mode & SPI_CPOL ? APPLE_SPI_CFG_CPOL : 0) |
		   (device->mode & SPI_LSB_FIRST ? APPLE_SPI_CFG_LSB_FIRST : 0));

	/* Update core config */
	reg_mask(spi, APPLE_SPI_CFG,
		 APPLE_SPI_CFG_CPHA | APPLE_SPI_CFG_CPOL | APPLE_SPI_CFG_LSB_FIRST, cfg);

	if (ctlr->set_cs_timing) {
		u32 setup_cycles, hold_cycles;
		int ret;

		ret = apple_spi_cs_timing_cycles(spi, device, &setup_cycles,
						 &hold_cycles);
		if (ret)
			return ret;
		apple_spi_write_cs_timing(spi, setup_cycles, hold_cycles);
	}

	return 0;
}

static bool apple_spi_prep_transfer(struct apple_spi *spi, struct spi_transfer *t)
{
	u32 cr, fifo_threshold;

	/* Calculate and program the clock rate */
	cr = DIV_ROUND_UP(clk_get_rate(spi->clk), t->speed_hz);
	cr = min_t(u32, cr, APPLE_SPI_CLKDIV_MAX);
	cr = max(cr, 1U);
	reg_write(spi, APPLE_SPI_CLKDIV, cr);
	t->effective_speed_hz = clk_get_rate(spi->clk) / cr;

	/* Update bits per word */
	reg_mask(spi, APPLE_SPI_SHIFTCFG, APPLE_SPI_SHIFTCFG_BITS,
		 FIELD_PREP(APPLE_SPI_SHIFTCFG_BITS, t->bits_per_word));

	/* We will want to poll if the time we need to wait is
	 * less than the context switching time.
	 * Let's call that threshold 5us. The operation will take:
	 *    bits_per_word * fifo_threshold / hz <= 5 * 10^-6
	 *    200000 * bits_per_word * fifo_threshold <= hz
	 */
	fifo_threshold = APPLE_SPI_FIFO_DEPTH / 2;
	return (200000 * t->bits_per_word * fifo_threshold) <= t->effective_speed_hz;
}

static int apple_spi_poll(struct apple_spi *spi, u32 fifo_bit, u32 xfer_bit)
{
	unsigned long timeout = jiffies + APPLE_SPI_TIMEOUT_MS * HZ / 1000;
	u32 fifo, xfer;

	do {
		fifo = reg_read(spi, APPLE_SPI_IF_FIFO);
		xfer = reg_read(spi, APPLE_SPI_IF_XFER);
		if (time_after(jiffies, timeout))
			return -ETIMEDOUT;
	} while (!((fifo & fifo_bit) || (xfer & xfer_bit)));

	return 0;
}

static void apple_spi_tx(struct apple_spi *spi, const void **tx_ptr, u32 *left,
			 unsigned int bytes_per_word)
{
	u32 inuse, words, wrote;

	if (!*tx_ptr)
		return;

	inuse = FIELD_GET(APPLE_SPI_FIFOSTAT_LEVEL_TX, reg_read(spi, APPLE_SPI_FIFOSTAT));
	words = wrote = min_t(u32, *left, APPLE_SPI_FIFO_DEPTH - inuse);

	if (!words)
		return;

	*left -= words;

	switch (bytes_per_word) {
	case 1: {
		const u8 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	case 2: {
		const u16 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	case 4: {
		const u32 *p = *tx_ptr;

		while (words--)
			reg_write(spi, APPLE_SPI_TXDATA, *p++);
		break;
	}
	default:
		WARN_ON(1);
	}

	*tx_ptr = ((u8 *)*tx_ptr) + bytes_per_word * wrote;
}

static void apple_spi_rx(struct apple_spi *spi, void **rx_ptr, u32 *left,
			 unsigned int bytes_per_word)
{
	u32 words, read;

	if (!*rx_ptr)
		return;

	words = read = FIELD_GET(APPLE_SPI_FIFOSTAT_LEVEL_RX, reg_read(spi, APPLE_SPI_FIFOSTAT));
	WARN_ON(words > *left);

	if (!words)
		return;

	*left -= min_t(u32, *left, words);

	switch (bytes_per_word) {
	case 1: {
		u8 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	case 2: {
		u16 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	case 4: {
		u32 *p = *rx_ptr;

		while (words--)
			*p++ = reg_read(spi, APPLE_SPI_RXDATA);
		break;
	}
	default:
		WARN_ON(1);
	}

	*rx_ptr = ((u8 *)*rx_ptr) + bytes_per_word * read;
}

static u32 apple_spi_fifo_flags(struct apple_spi *spi)
{
	u32 flags = 0;

	if (spi->remaining_tx)
		flags |= APPLE_SPI_FIFO_TXTHRESH;
	if (spi->remaining_rx)
		flags |= APPLE_SPI_FIFO_RXTHRESH;
	return flags;
}

/* Moves what the FIFOs allow and notes the halves that completed. */
static void apple_spi_service(struct apple_spi *spi)
{
	spi->xfer_flags &= ~reg_read(spi, APPLE_SPI_IF_XFER);
	apple_spi_tx(spi, &spi->tx_ptr, &spi->remaining_tx, spi->bytes_per_word);
	apple_spi_rx(spi, &spi->rx_ptr, &spi->remaining_rx, spi->bytes_per_word);
}

/*
 * The FIFOs are refilled and drained here, in the interrupt handler, not by
 * the waiting thread. The FIFO is 16 words deep and the threshold interrupt
 * fires with 8 left, so at 8 MHz the refill has about 8 us before the
 * transmit FIFO runs dry; the controller then stops the clock with chip
 * select still asserted until it is refilled. A thread woken to refill
 * waits for the scheduler, which under load takes hundreds of
 * microseconds, and a Mesa fingerprint sensor drops or misreads a frame
 * that stalls that long.
 */
static irqreturn_t apple_spi_irq(int irq, void *dev_id)
{
	struct apple_spi *spi = dev_id;
	u32 fifo, xfer;

	spin_lock(&spi->lock);
	fifo = reg_read(spi, APPLE_SPI_IF_FIFO) & reg_read(spi, APPLE_SPI_IE_FIFO);
	xfer = reg_read(spi, APPLE_SPI_IF_XFER) & reg_read(spi, APPLE_SPI_IE_XFER);
	if (!fifo && !xfer) {
		spin_unlock(&spi->lock);
		return IRQ_NONE;
	}

	/* Keep the working thread-driven handshake for transmit-only uploads. */
	if (spi->active && spi->threaded_tx) {
		reg_write(spi, APPLE_SPI_IE_XFER, 0);
		reg_write(spi, APPLE_SPI_IE_FIFO, 0);
		complete(&spi->done);
		spin_unlock(&spi->lock);
		return IRQ_HANDLED;
	}

	if (spi->active)
		apple_spi_service(spi);
	reg_write(spi, APPLE_SPI_IF_FIFO,
		  fifo & (APPLE_SPI_FIFO_TXTHRESH | APPLE_SPI_FIFO_RXTHRESH));

	if (spi->active && spi->xfer_flags) {
		reg_write(spi, APPLE_SPI_IE_XFER, spi->xfer_flags);
		reg_write(spi, APPLE_SPI_IE_FIFO, apple_spi_fifo_flags(spi));
	} else {
		reg_write(spi, APPLE_SPI_IE_XFER, 0);
		reg_write(spi, APPLE_SPI_IE_FIFO, 0);
		if (spi->active) {
			spi->active = false;
			complete(&spi->done);
		}
	}
	spin_unlock(&spi->lock);

	return IRQ_HANDLED;
}

static int apple_spi_transfer_one(struct spi_controller *ctlr, struct spi_device *device,
				  struct spi_transfer *t)
{
	struct apple_spi *spi = spi_controller_get_devdata(ctlr);
	bool poll = apple_spi_prep_transfer(spi, t);
	unsigned int bytes_per_word;
	unsigned long flags;
	u32 words, fifo_flags;
	int retries = 100;
	int ret = 0;

	if (t->bits_per_word > 16)
		bytes_per_word = 4;
	else if (t->bits_per_word > 8)
		bytes_per_word = 2;
	else
		bytes_per_word = 1;

	words = t->len / bytes_per_word;
	spi->tx_ptr = t->tx_buf;
	spi->rx_ptr = t->rx_buf;
	spi->remaining_tx = t->tx_buf ? words : 0;
	spi->remaining_rx = t->rx_buf ? words : 0;
	spi->bytes_per_word = bytes_per_word;
	spi->threaded_tx = t->tx_buf && !t->rx_buf;

	/* Reset FIFOs */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RX_RESET | APPLE_SPI_CTRL_TX_RESET);

	/* Clear IRQ flags */
	reg_write(spi, APPLE_SPI_IF_XFER, ~0);
	reg_write(spi, APPLE_SPI_IF_FIFO, ~0);

	/* Determine transfer completion flags we wait for */
	spi->xfer_flags = 0;
	if (t->tx_buf)
		spi->xfer_flags |= APPLE_SPI_XFER_TXCOMPLETE;
	if (t->rx_buf)
		spi->xfer_flags |= APPLE_SPI_XFER_RXCOMPLETE;

	/* Set transfer length */
	reg_write(spi, APPLE_SPI_TXCNT, spi->remaining_tx);
	reg_write(spi, APPLE_SPI_RXCNT, spi->remaining_rx);

	/* Prime transmit FIFO */
	apple_spi_tx(spi, &spi->tx_ptr, &spi->remaining_tx, bytes_per_word);

	/* Start transfer */
	reg_write(spi, APPLE_SPI_CTRL, APPLE_SPI_CTRL_RUN);

	/* TX again since a few words get popped off immediately */
	apple_spi_tx(spi, &spi->tx_ptr, &spi->remaining_tx, bytes_per_word);

	if (poll) {
		while (spi->xfer_flags) {
			/* Wait for anything to happen */
			ret = apple_spi_poll(spi, apple_spi_fifo_flags(spi),
					     spi->xfer_flags);
			if (ret)
				break;

			/* Transmit and receive everything we can */
			apple_spi_service(spi);
		}
	} else if (spi->threaded_tx) {
		/*
		 * Transmit-only uploads can lose FIFO progress with the IRQ
		 * engine. Use the pre-IRQ-engine wake/refill/rearm sequence;
		 * retain in-handler service for receive and duplex transfers.
		 */
		while (spi->xfer_flags) {
			reinit_completion(&spi->done);
			spin_lock_irqsave(&spi->lock, flags);
			spi->active = true;
			reg_write(spi, APPLE_SPI_IE_XFER, spi->xfer_flags);
			reg_write(spi, APPLE_SPI_IE_FIFO, apple_spi_fifo_flags(spi));
			spin_unlock_irqrestore(&spi->lock, flags);

			if (!wait_for_completion_timeout(&spi->done,
							 msecs_to_jiffies(APPLE_SPI_TIMEOUT_MS)))
				ret = -ETIMEDOUT;

			spin_lock_irqsave(&spi->lock, flags);
			spi->active = false;
			reg_write(spi, APPLE_SPI_IE_XFER, 0);
			reg_write(spi, APPLE_SPI_IE_FIFO, 0);
			spin_unlock_irqrestore(&spi->lock, flags);
			synchronize_irq(spi->irq);
			if (ret)
				break;
			apple_spi_service(spi);
		}
	} else {
		/* Allow for the transfer itself on top of the usual timeout. */
		unsigned long ms = APPLE_SPI_TIMEOUT_MS +
			DIV_ROUND_UP_ULL((u64)words * t->bits_per_word * 1000,
					 max(t->effective_speed_hz, 1U));

		reinit_completion(&spi->done);
		spin_lock_irqsave(&spi->lock, flags);
		spi->active = true;
		reg_write(spi, APPLE_SPI_IE_XFER, spi->xfer_flags);
		reg_write(spi, APPLE_SPI_IE_FIFO, apple_spi_fifo_flags(spi));
		spin_unlock_irqrestore(&spi->lock, flags);

		if (!wait_for_completion_timeout(&spi->done, msecs_to_jiffies(ms))) {
			spin_lock_irqsave(&spi->lock, flags);
			spi->active = false;
			reg_write(spi, APPLE_SPI_IE_XFER, 0);
			reg_write(spi, APPLE_SPI_IE_FIFO, 0);
			spin_unlock_irqrestore(&spi->lock, flags);
			synchronize_irq(spi->irq);
			ret = -ETIMEDOUT;
		}
	}
	if (ret) {
		dev_err(&ctlr->dev, "transfer timed out (remaining %d tx, %d rx)\n",
			spi->remaining_tx, spi->remaining_rx);
		goto err;
	}

	/*
	 * Sometimes the transfer completes before the last word is in the RX FIFO.
	 * Normally one retry is all it takes to get the last word out.
	 */
	while (spi->remaining_rx && retries--)
		apple_spi_rx(spi, &spi->rx_ptr, &spi->remaining_rx, bytes_per_word);

	if (spi->remaining_tx)
		dev_err(&ctlr->dev, "transfer completed with %d words left to transmit\n",
			spi->remaining_tx);
	if (spi->remaining_rx)
		dev_err(&ctlr->dev, "transfer completed with %d words left to receive\n",
			spi->remaining_rx);

err:
	fifo_flags = reg_read(spi, APPLE_SPI_IF_FIFO);
	WARN_ON(fifo_flags & APPLE_SPI_FIFO_TXOVERFLOW);
	WARN_ON(fifo_flags & APPLE_SPI_FIFO_RXUNDERRUN);

	/* Stop transfer */
	reg_write(spi, APPLE_SPI_CTRL, 0);

	return ret;
}

static int apple_spi_probe(struct platform_device *pdev)
{
	struct apple_spi *spi;
	int ret, irq;
	struct spi_controller *ctlr;
	struct device_node *np;

	ctlr = devm_spi_alloc_host(&pdev->dev, sizeof(struct apple_spi));
	if (!ctlr)
		return -ENOMEM;

	spi = spi_controller_get_devdata(ctlr);
	init_completion(&spi->done);
	spin_lock_init(&spi->lock);

	spi->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(spi->regs))
		return PTR_ERR(spi->regs);

	spi->clk = devm_clk_get_enabled(&pdev->dev, NULL);
	if (IS_ERR(spi->clk))
		return dev_err_probe(&pdev->dev, PTR_ERR(spi->clk),
				     "Unable to find or enable bus clock\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	spi->irq = irq;

	ret = devm_request_irq(&pdev->dev, irq, apple_spi_irq, 0,
			       dev_name(&pdev->dev), spi);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Unable to bind to interrupt\n");

	ctlr->bus_num = pdev->id;
	ctlr->num_chipselect = 1;
	ctlr->mode_bits = SPI_CPHA | SPI_CPOL | SPI_LSB_FIRST;
	ctlr->bits_per_word_mask = SPI_BPW_RANGE_MASK(1, 32);
	ctlr->prepare_message = apple_spi_prepare_message;
	ctlr->set_cs = apple_spi_set_cs;
	/*
	 * Only the Mesa fingerprint sensor's bus uses the hardware CS delays.
	 * The SPI core stops running software CS delays for every device on a
	 * controller that provides set_cs_timing, so installing it everywhere
	 * would move the keyboard, NOR flash and others off their working
	 * timing, and the keyboard's inactive delay has no hardware form.
	 */
	np = of_get_compatible_child(pdev->dev.of_node, "apple,mesa-fingerprint");
	if (np) {
		ctlr->set_cs_timing = apple_spi_set_cs_timing;
		of_node_put(np);
	}
	ctlr->transfer_one = apple_spi_transfer_one;
	ctlr->use_gpio_descriptors = true;
	ctlr->auto_runtime_pm = true;

	pm_runtime_set_active(&pdev->dev);
	ret = devm_pm_runtime_enable(&pdev->dev);
	if (ret < 0)
		return ret;

	apple_spi_init(spi);

	ret = devm_spi_register_controller(&pdev->dev, ctlr);
	if (ret < 0)
		return dev_err_probe(&pdev->dev, ret, "devm_spi_register_controller failed\n");

	return 0;
}

static const struct of_device_id apple_spi_of_match[] = {
	{ .compatible = "apple,t8103-spi", },
	{ .compatible = "apple,spi", },
	{}
};
MODULE_DEVICE_TABLE(of, apple_spi_of_match);

static struct platform_driver apple_spi_driver = {
	.probe = apple_spi_probe,
	.driver = {
		.name = "apple-spi",
		.of_match_table = apple_spi_of_match,
	},
};
module_platform_driver(apple_spi_driver);

#if IS_ENABLED(CONFIG_SPI_APPLE_KUNIT_TEST)
#include "spi-apple-test.c"
#endif

MODULE_AUTHOR("Hector Martin <marcan@marcan.st>");
MODULE_DESCRIPTION("Apple SoC SPI driver");
MODULE_LICENSE("GPL");
