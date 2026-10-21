// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Apple mailbox driver
 *
 * Copyright The Asahi Linux Contributors
 *
 * This driver adds support for two mailbox variants (called ASC and M3 by
 * Apple) found in Apple SoCs such as the M1. It consists of two FIFOs used to
 * exchange 64+32 bit messages between the main CPU and a co-processor.
 * Various coprocessors implement different IPC protocols based on these simple
 * messages and shared memory buffers.
 *
 * Both the main CPU and the co-processor see the same set of registers but
 * the first FIFO (A2I) is always used to transfer messages from the application
 * processor (us) to the I/O processor and the second one (I2A) for the
 * other direction.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/soc/apple/mailbox.h>
#include <linux/spinlock.h>
#include <linux/types.h>

#define APPLE_ASC_MBOX_CONTROL_FULL BIT(16)
#define APPLE_ASC_MBOX_CONTROL_EMPTY BIT(17)

#define APPLE_ASC_MBOX_A2I_CONTROL 0x110
#define APPLE_ASC_MBOX_A2I_SEND0 0x800
#define APPLE_ASC_MBOX_A2I_SEND1 0x808
#define APPLE_ASC_MBOX_A2I_RECV0 0x810
#define APPLE_ASC_MBOX_A2I_RECV1 0x818

#define APPLE_ASC_MBOX_I2A_CONTROL 0x114
#define APPLE_ASC_MBOX_I2A_SEND0 0x820
#define APPLE_ASC_MBOX_I2A_SEND1 0x828
#define APPLE_ASC_MBOX_I2A_RECV0 0x830
#define APPLE_ASC_MBOX_I2A_RECV1 0x838

/* ASCWrap v6 embeds the unchanged ASC mailbox block at this offset. */
#define APPLE_ASCWRAP_V6_MBOX_OFFSET 0x8000
#define APPLE_ASCWRAP_V6_WRAPPER_SIZE 0x88000
#define APPLE_ASCWRAP_V6_IOP_VBAR_SIZE 0x8
#define APPLE_ASCWRAP_V6_CPU_CONTROL 0x44
#define APPLE_ASCWRAP_V6_CPU_RUN BIT(4)
#define APPLE_ASCWRAP_V6_CPU_STOP_SECOND_CLEAR BIT(5)

#define APPLE_T8140_GFX_WRAPPER_BASE 0x482600000ULL
#define APPLE_T8140_GFX_IOP_VBAR_BASE 0x482050000ULL
#define APPLE_T8140_GFX1_WRAPPER_BASE 0x482e00000ULL
#define APPLE_T8140_GFX1_IOP_VBAR_BASE 0x482850000ULL

static_assert(APPLE_ASCWRAP_V6_MBOX_OFFSET + APPLE_ASC_MBOX_A2I_CONTROL ==
	      0x8110);
static_assert(APPLE_ASCWRAP_V6_MBOX_OFFSET + APPLE_ASC_MBOX_I2A_CONTROL ==
	      0x8114);
static_assert(APPLE_ASCWRAP_V6_MBOX_OFFSET + APPLE_ASC_MBOX_A2I_SEND0 ==
	      0x8800);
static_assert(APPLE_ASCWRAP_V6_MBOX_OFFSET + APPLE_ASC_MBOX_I2A_RECV0 ==
	      0x8830);

/*
 * The T8140 AOP "setup port": a second, smaller mailbox the AOP firmware
 * uses during its boot.
 */
#define APPLE_T8140_AOP_SETUP_A2I_CONTROL 0x000
#define APPLE_T8140_AOP_SETUP_I2A_CONTROL 0x004
#define APPLE_T8140_AOP_SETUP_A2I_SEND0   0x180
#define APPLE_T8140_AOP_SETUP_A2I_SEND1   0x188
#define APPLE_T8140_AOP_SETUP_I2A_RECV0   0x1b0
#define APPLE_T8140_AOP_SETUP_I2A_RECV1   0x1b8

#define APPLE_T8015_MBOX_A2I_CONTROL	0x108
#define APPLE_T8015_MBOX_I2A_CONTROL	0x10c

#define APPLE_M3_MBOX_CONTROL_FULL BIT(16)
#define APPLE_M3_MBOX_CONTROL_EMPTY BIT(17)

#define APPLE_M3_MBOX_A2I_CONTROL 0x50
#define APPLE_M3_MBOX_A2I_SEND0 0x60
#define APPLE_M3_MBOX_A2I_SEND1 0x68
#define APPLE_M3_MBOX_A2I_RECV0 0x70
#define APPLE_M3_MBOX_A2I_RECV1 0x78

#define APPLE_M3_MBOX_I2A_CONTROL 0x80
#define APPLE_M3_MBOX_I2A_SEND0 0x90
#define APPLE_M3_MBOX_I2A_SEND1 0x98
#define APPLE_M3_MBOX_I2A_RECV0 0xa0
#define APPLE_M3_MBOX_I2A_RECV1 0xa8

#define APPLE_M3_MBOX_IRQ_ENABLE 0x48
#define APPLE_M3_MBOX_IRQ_ACK 0x4c
#define APPLE_M3_MBOX_IRQ_A2I_EMPTY BIT(0)
#define APPLE_M3_MBOX_IRQ_A2I_NOT_EMPTY BIT(1)
#define APPLE_M3_MBOX_IRQ_I2A_EMPTY BIT(2)
#define APPLE_M3_MBOX_IRQ_I2A_NOT_EMPTY BIT(3)

#define APPLE_MBOX_MSG1_OUTCNT GENMASK(56, 52)
#define APPLE_MBOX_MSG1_INCNT GENMASK(51, 48)
#define APPLE_MBOX_MSG1_OUTPTR GENMASK(47, 44)
#define APPLE_MBOX_MSG1_INPTR GENMASK(43, 40)
#define APPLE_MBOX_MSG1_MSG GENMASK(31, 0)

#define APPLE_MBOX_TX_TIMEOUT 500
#define APPLE_G17_MBOX_RX_BUDGET 64
#define APPLE_G17_MBOX_ATOMIC_TX_US 1000

struct apple_mbox_hw {
	unsigned int reg_offset;
	bool is_ascwrap_v6;
	bool ap_initializes_mailboxes;

	unsigned int control_full;
	unsigned int control_empty;

	unsigned int a2i_control;
	unsigned int a2i_send0;
	unsigned int a2i_send1;

	unsigned int i2a_control;
	unsigned int i2a_recv0;
	unsigned int i2a_recv1;

	bool has_irq_controls;
	unsigned int irq_enable;
	unsigned int irq_ack;
	unsigned int irq_bit_recv_not_empty;
	unsigned int irq_bit_send_empty;
};

int apple_mbox_ascwrap_v6_get_lifecycle(
	struct apple_mbox *mbox,
	struct apple_mbox_ascwrap_v6_lifecycle *lifecycle)
{
	if (!mbox || !lifecycle)
		return -EINVAL;
	if (READ_ONCE(mbox->removing))
		return -ENODEV;
	if (!mbox->hw->is_ascwrap_v6)
		return -EOPNOTSUPP;

	*lifecycle = mbox->ascwrap_v6;
	return 0;
}
EXPORT_SYMBOL_GPL(apple_mbox_ascwrap_v6_get_lifecycle);

int apple_mbox_ascwrap_v6_require_safe_cpu_lifecycle(struct apple_mbox *mbox)
{
	if (!mbox || !mbox->hw->is_ascwrap_v6)
		return -EOPNOTSUPP;
	if (READ_ONCE(mbox->removing))
		return -ENODEV;

	/* Future provider revisions may report additional lifecycle work. */
	if (mbox->ascwrap_v6.missing)
		return -EOPNOTSUPP;

	return 0;
}
EXPORT_SYMBOL_GPL(apple_mbox_ascwrap_v6_require_safe_cpu_lifecycle);

static void apple_mbox_ascwrap_v6_stop_cpu_locked(struct apple_mbox *mbox)
{
	void __iomem *cpu_control;
	u32 val;

	if (!mbox->ascwrap_v6_cpu_running)
		return;

	cpu_control = (u8 __iomem *)mbox->wrapper_regs +
		      APPLE_ASCWRAP_V6_CPU_CONTROL;

	/* Stop: clear RUN, reread, clear bit 5. */
	val = readl(cpu_control);
	writel(val & ~APPLE_ASCWRAP_V6_CPU_RUN, cpu_control);
	val = readl(cpu_control);
	writel(val & ~APPLE_ASCWRAP_V6_CPU_STOP_SECOND_CLEAR, cpu_control);

	mbox->ascwrap_v6_cpu_running = false;
	pm_runtime_mark_last_busy(mbox->dev);
	pm_runtime_put_autosuspend(mbox->dev);
}

/*
 * A failed resume leaves the runtime PM error set, and runtime PM then fails
 * every later resume with -EINVAL without retrying the power domain.  The
 * provider keeps no state across a failed resume, so mark it suspended again
 * and let the next start retry the domain.
 */
static void apple_mbox_clear_resume_error(struct apple_mbox *mbox)
{
	if (mbox->dev->power.runtime_error)
		pm_runtime_set_suspended(mbox->dev);
}

int apple_mbox_ascwrap_v6_start_cpu(struct apple_mbox *mbox)
{
	void __iomem *cpu_control;
	u32 val;
	int ret = 0;

	if (!mbox || !mbox->hw->is_ascwrap_v6 || !mbox->wrapper_regs)
		return -EOPNOTSUPP;

	mutex_lock(&mbox->lifecycle_lock);
	if (mbox->removing) {
		ret = -ENODEV;
		goto out_unlock;
	}
	if (mbox->ascwrap_v6.missing) {
		ret = -EOPNOTSUPP;
		goto out_unlock;
	}
	if (mbox->ascwrap_v6_cpu_running) {
		ret = -EBUSY;
		goto out_unlock;
	}

	ret = pm_runtime_resume_and_get(mbox->dev);
	if (ret) {
		dev_err(mbox->dev, "ASC CPU start: runtime resume failed: %d\n", ret);
		apple_mbox_clear_resume_error(mbox);
		goto out_unlock;
	}

	cpu_control = (u8 __iomem *)mbox->wrapper_regs +
		      APPLE_ASCWRAP_V6_CPU_CONTROL;

	/* Start: read, set RUN, write. */
	val = readl(cpu_control);
	writel(val | APPLE_ASCWRAP_V6_CPU_RUN, cpu_control);
	mbox->ascwrap_v6_cpu_running = true;

out_unlock:
	mutex_unlock(&mbox->lifecycle_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_mbox_ascwrap_v6_start_cpu);

int apple_mbox_ascwrap_v6_stop_cpu(struct apple_mbox *mbox)
{
	int ret = 0;

	if (!mbox || !mbox->hw->is_ascwrap_v6 || !mbox->wrapper_regs)
		return -EOPNOTSUPP;

	mutex_lock(&mbox->lifecycle_lock);
	if (mbox->removing) {
		ret = -ENODEV;
		goto out_unlock;
	}

	apple_mbox_ascwrap_v6_stop_cpu_locked(mbox);

out_unlock:
	mutex_unlock(&mbox->lifecycle_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(apple_mbox_ascwrap_v6_stop_cpu);

static bool apple_mbox_is_g17(const struct apple_mbox *mbox)
{
	return mbox->ascwrap_v6.role == APPLE_MBOX_ASCWRAP_V6_ROLE_GFX ||
	       mbox->ascwrap_v6.role == APPLE_MBOX_ASCWRAP_V6_ROLE_GFX1;
}

int apple_mbox_send(struct apple_mbox *mbox, const struct apple_mbox_msg msg,
		    bool atomic)
{
	unsigned long flags, remaining, now;
	unsigned long deadline = jiffies + msecs_to_jiffies(APPLE_MBOX_TX_TIMEOUT);
	int ret;
	u32 mbox_ctrl;
	long t;

	spin_lock_irqsave(&mbox->tx_lock, flags);
	if (unlikely(READ_ONCE(mbox->removing))) {
		spin_unlock_irqrestore(&mbox->tx_lock, flags);
		return -ENODEV;
	}
	mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->a2i_control);

	while (mbox_ctrl & mbox->hw->control_full) {
		now = jiffies;
		if (time_after_eq(now, deadline)) {
			spin_unlock_irqrestore(&mbox->tx_lock, flags);
			return -ETIMEDOUT;
		}
		remaining = deadline - now;
		if (apple_mbox_is_g17(mbox) && !atomic && mbox->irq_send_empty < 0) {
			/* The GPU has no send-empty IRQ. Do not busy wait with IRQs
			 * off while dead firmware keeps its FIFO full. Recheck both
			 * the removal fence and the one absolute deadline on retry.
			 */
			spin_unlock_irqrestore(&mbox->tx_lock, flags);
			usleep_range(1000, 2000);
			spin_lock_irqsave(&mbox->tx_lock, flags);
			if (unlikely(READ_ONCE(mbox->removing))) {
				spin_unlock_irqrestore(&mbox->tx_lock, flags);
				return -ENODEV;
			}
			mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->a2i_control);
			continue;
		}
		if (atomic || mbox->irq_send_empty < 0) {
			ret = readl_poll_timeout_atomic(
				mbox->regs + mbox->hw->a2i_control, mbox_ctrl,
				!(mbox_ctrl & mbox->hw->control_full), 100,
				apple_mbox_is_g17(mbox) ? APPLE_G17_MBOX_ATOMIC_TX_US :
				APPLE_MBOX_TX_TIMEOUT * 1000);

			if (ret) {
				spin_unlock_irqrestore(&mbox->tx_lock, flags);
				return ret;
			}

			break;
		}
		/*
		 * The interrupt is level triggered and will keep firing as long as the
		 * FIFO is empty. It will also keep firing if the FIFO was empty
		 * at any point in the past until it has been acknowledged at the
		 * mailbox level. By acknowledging it here we can ensure that we will
		 * only get the interrupt once the FIFO has been cleared again.
		 * If the FIFO is already empty before the ack it will fire again
		 * immediately after the ack.
		 */
		if (mbox->hw->has_irq_controls) {
			writel_relaxed(mbox->hw->irq_bit_send_empty,
				       mbox->regs + mbox->hw->irq_ack);
		}
		reinit_completion(&mbox->tx_empty);
		if (!mbox->tx_irq_unmasked) {
			mbox->tx_irq_unmasked = true;
			enable_irq(mbox->irq_send_empty);
		}
		spin_unlock_irqrestore(&mbox->tx_lock, flags);

		t = wait_for_completion_interruptible_timeout(
			&mbox->tx_empty, remaining);
		if (t <= 0) {
			/* A late IRQ may already have masked this under tx_lock. */
			spin_lock_irqsave(&mbox->tx_lock, flags);
			if (mbox->tx_irq_unmasked) {
				disable_irq_nosync(mbox->irq_send_empty);
				mbox->tx_irq_unmasked = false;
			}
			spin_unlock_irqrestore(&mbox->tx_lock, flags);
			return t < 0 ? t : -ETIMEDOUT;
		}

		spin_lock_irqsave(&mbox->tx_lock, flags);
		if (unlikely(READ_ONCE(mbox->removing))) {
			spin_unlock_irqrestore(&mbox->tx_lock, flags);
			return -ENODEV;
		}
		mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->a2i_control);
	}

	writeq_relaxed(msg.msg0, mbox->regs + mbox->hw->a2i_send0);
	writeq_relaxed(FIELD_PREP(APPLE_MBOX_MSG1_MSG, msg.msg1),
		       mbox->regs + mbox->hw->a2i_send1);

	spin_unlock_irqrestore(&mbox->tx_lock, flags);

	return 0;
}
EXPORT_SYMBOL(apple_mbox_send);

static irqreturn_t apple_mbox_send_empty_irq(int irq, void *data)
{
	struct apple_mbox *mbox = data;

	/*
	 * We don't need to acknowledge the interrupt at the mailbox level
	 * here even if supported by the hardware. It will keep firing but that
	 * doesn't matter since it's disabled at the main interrupt controller.
	 * apple_mbox_send will acknowledge it before enabling
	 * it at the main controller again.
	 */
	spin_lock(&mbox->tx_lock);
	if (mbox->tx_irq_unmasked) {
		disable_irq_nosync(mbox->irq_send_empty);
		mbox->tx_irq_unmasked = false;
	}
	complete(&mbox->tx_empty);
	spin_unlock(&mbox->tx_lock);

	return IRQ_HANDLED;
}

static int apple_mbox_poll_locked(struct apple_mbox *mbox)
{
	struct apple_mbox_msg msg;
	int ret = 0;
	u32 mbox_ctrl;
	u32 budget = apple_mbox_is_g17(mbox) ? APPLE_G17_MBOX_RX_BUDGET : U32_MAX;

	/* A stopped receiver may still have firmware traffic in its FIFO. */
	if (!mbox->rx)
		return 0;
	mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->i2a_control);

	while (!(mbox_ctrl & mbox->hw->control_empty) && budget--) {
		msg.msg0 = readq_relaxed(mbox->regs + mbox->hw->i2a_recv0);
		msg.msg1 = FIELD_GET(
			APPLE_MBOX_MSG1_MSG,
			readq_relaxed(mbox->regs + mbox->hw->i2a_recv1));

		mbox->rx(mbox, msg, mbox->cookie);
		ret++;
		mbox_ctrl = readl_relaxed(mbox->regs + mbox->hw->i2a_control);
	}

	/*
	 * The interrupt will keep firing even if there are no more messages
	 * unless we also acknowledge it at the mailbox level here.
	 * There's no race if a message comes in between the check in the while
	 * loop above and the ack below: If a new messages arrives inbetween
	 * those two the interrupt will just fire again immediately after the
	 * ack since it's level triggered.
	 */
	if (mbox->hw->has_irq_controls) {
		writel_relaxed(mbox->hw->irq_bit_recv_not_empty,
			       mbox->regs + mbox->hw->irq_ack);
	}

	return ret;
}

static irqreturn_t apple_mbox_recv_irq(int irq, void *data)
{
	struct apple_mbox *mbox = data;
	unsigned long flags;
	int consumed = 0;

	spin_lock_irqsave(&mbox->rx_lock, flags);
	if (likely(!READ_ONCE(mbox->removing)))
		consumed = apple_mbox_poll_locked(mbox);
	spin_unlock_irqrestore(&mbox->rx_lock, flags);

	/* G17 uses a oneshot threaded IRQ. Leave it masked during this sleep
	 * so a firmware flood cannot monopolize a CPU at IRQ-thread priority.
	 * The level condition retriggers after return; no message is discarded.
	 */
	if (apple_mbox_is_g17(mbox) && consumed == APPLE_G17_MBOX_RX_BUDGET)
		usleep_range(1000, 2000);

	return IRQ_HANDLED;
}

int apple_mbox_poll(struct apple_mbox *mbox)
{
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&mbox->rx_lock, flags);
	if (unlikely(READ_ONCE(mbox->removing)))
		ret = -ENODEV;
	else
		ret = apple_mbox_poll_locked(mbox);
	spin_unlock_irqrestore(&mbox->rx_lock, flags);

	return ret;
}
EXPORT_SYMBOL(apple_mbox_poll);

int apple_mbox_start(struct apple_mbox *mbox)
{
	u32 control;
	int ret = 0;

	mutex_lock(&mbox->lifecycle_lock);
	if (mbox->removing) {
		ret = -ENODEV;
		goto out_unlock;
	}
	if (mbox->active)
		goto out_unlock;

	ret = pm_runtime_resume_and_get(mbox->dev);
	if (ret) {
		dev_err(mbox->dev, "mailbox start: runtime resume failed: %d\n", ret);
		apple_mbox_clear_resume_error(mbox);
		goto out_unlock;
	}

	if (mbox->hw->ap_initializes_mailboxes) {
		/* The primary T8140 outbox requires its enable bit before use. */
		control = readl(mbox->regs + mbox->hw->i2a_control);
		writel(control | BIT(0), mbox->regs + mbox->hw->i2a_control);
	}

	/*
	 * Only some variants of this mailbox HW provide interrupt control
	 * at the mailbox level. We therefore need to handle enabling/disabling
	 * interrupts at the main interrupt controller anyway for hardware that
	 * doesn't. Just always keep the interrupts we care about enabled at
	 * the mailbox level so that both hardware revisions behave almost
	 * the same.
	 */
	if (mbox->hw->has_irq_controls) {
		writel_relaxed(mbox->hw->irq_bit_recv_not_empty |
				       mbox->hw->irq_bit_send_empty,
			       mbox->regs + mbox->hw->irq_enable);
	}

	enable_irq(mbox->irq_recv_not_empty);
	mbox->active = true;

out_unlock:
	mutex_unlock(&mbox->lifecycle_lock);
	return ret;
}
EXPORT_SYMBOL(apple_mbox_start);

static void apple_mbox_stop_locked(struct apple_mbox *mbox)
{
	if (!mbox->active)
		return;

	mbox->active = false;
	disable_irq(mbox->irq_recv_not_empty);
	pm_runtime_mark_last_busy(mbox->dev);
	pm_runtime_put_autosuspend(mbox->dev);
}

void apple_mbox_stop(struct apple_mbox *mbox)
{
	mutex_lock(&mbox->lifecycle_lock);
	apple_mbox_stop_locked(mbox);
	mutex_unlock(&mbox->lifecycle_lock);
}
EXPORT_SYMBOL(apple_mbox_stop);

void apple_mbox_hold_rx(struct apple_mbox *mbox)
{
	disable_irq(mbox->irq_recv_not_empty);
}
EXPORT_SYMBOL(apple_mbox_hold_rx);

void apple_mbox_release_rx(struct apple_mbox *mbox)
{
	enable_irq(mbox->irq_recv_not_empty);
}
EXPORT_SYMBOL(apple_mbox_release_rx);

/*
 * Stop the provider-owned CPU, then quiesce every transport edge before
 * devres releases IRQs or MMIO mappings. Device links order consumer removal
 * first; the callback clear and waiter wake also cover future consumers that
 * violate that ownership rule.
 */
static void apple_mbox_quiesce(void *data)
{
	struct apple_mbox *mbox = data;
	unsigned long flags;
	bool had_rx;
	bool was_active;

	mutex_lock(&mbox->lifecycle_lock);
	if (mbox->removing) {
		mutex_unlock(&mbox->lifecycle_lock);
		return;
	}
	apple_mbox_ascwrap_v6_stop_cpu_locked(mbox);

	/* Serialize the removal fence with every MMIO send critical section. */
	spin_lock_irqsave(&mbox->tx_lock, flags);
	WRITE_ONCE(mbox->removing, true);
	spin_unlock_irqrestore(&mbox->tx_lock, flags);

	/*
	 * Publish the inactive state under the lifecycle lock, then drop it
	 * before disable_irq() waits for a callback. A callback is allowed to
	 * finish a send or stop request without an rx_lock -> lifecycle_lock
	 * inversion. The removal fence prevents any matching restart.
	 */
	was_active = mbox->active;
	mbox->active = false;
	mutex_unlock(&mbox->lifecycle_lock);
	if (was_active) {
		disable_irq(mbox->irq_recv_not_empty);
		pm_runtime_mark_last_busy(mbox->dev);
		pm_runtime_put_autosuspend(mbox->dev);
	}

	/* Serialize once-only masking with the IRQ and send-error paths. */
	spin_lock_irqsave(&mbox->tx_lock, flags);
	if (mbox->tx_irq_unmasked) {
		disable_irq_nosync(mbox->irq_send_empty);
		mbox->tx_irq_unmasked = false;
	}
	spin_unlock_irqrestore(&mbox->tx_lock, flags);
	/* The handler takes tx_lock, so drain it after releasing the lock. */
	if (mbox->irq_send_empty >= 0)
		synchronize_irq(mbox->irq_send_empty);
	complete_all(&mbox->tx_empty);
	synchronize_irq(mbox->irq_recv_not_empty);

	spin_lock_irqsave(&mbox->rx_lock, flags);
	had_rx = mbox->rx;
	mbox->rx = NULL;
	mbox->cookie = NULL;
	spin_unlock_irqrestore(&mbox->rx_lock, flags);

	if (WARN_ON_ONCE(had_rx))
		dev_warn(mbox->dev,
			 "consumer callback survived provider remove ordering\n");

	pm_runtime_barrier(mbox->dev);
}

struct apple_mbox *apple_mbox_get(struct device *dev, int index)
{
	struct of_phandle_args args;
	struct platform_device *pdev;
	struct apple_mbox *mbox;
	int ret;

	ret = of_parse_phandle_with_args(dev->of_node, "mboxes", "#mbox-cells",
					 index, &args);
	if (ret || !args.np)
		return ERR_PTR(ret);

	pdev = of_find_device_by_node(args.np);
	of_node_put(args.np);

	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);

	mbox = platform_get_drvdata(pdev);
	if (!mbox) {
		mbox = ERR_PTR(-EPROBE_DEFER);
		goto out_put_pdev;
	}

	if (!device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		mbox = ERR_PTR(-ENODEV);
		goto out_put_pdev;
	}

out_put_pdev:
	put_device(&pdev->dev);

	return mbox;
}
EXPORT_SYMBOL(apple_mbox_get);

struct apple_mbox *apple_mbox_get_byname(struct device *dev, const char *name)
{
	int index;

	index = of_property_match_string(dev->of_node, "mbox-names", name);
	if (index < 0)
		return ERR_PTR(index);

	return apple_mbox_get(dev, index);
}
EXPORT_SYMBOL(apple_mbox_get_byname);

static int apple_mbox_probe(struct platform_device *pdev)
{
	int ret;
	char *irqname;
	const char *firmware_role;
	struct apple_mbox *mbox;
	struct device *dev = &pdev->dev;
	struct resource *res, *iop_vbar_res;

	mbox = devm_kzalloc(dev, sizeof(*mbox), GFP_KERNEL);
	if (!mbox)
		return -ENOMEM;

	mbox->dev = &pdev->dev;
	mbox->hw = of_device_get_match_data(dev);
	if (!mbox->hw)
		return -EINVAL;

	if (mbox->hw->is_ascwrap_v6) {
		/*
		 * Validate the entire provider identity before mapping either
		 * resource.  These are fixed T8140 CPU-physical windows and role
		 * assignments, not generic consumer-provided aliases.
		 */
		res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "wrapper");
		iop_vbar_res = platform_get_resource_byname(pdev, IORESOURCE_MEM,
							    "iop-vbar");
		if (!res || !iop_vbar_res ||
		    resource_size(res) != APPLE_ASCWRAP_V6_WRAPPER_SIZE ||
		    resource_size(iop_vbar_res) != APPLE_ASCWRAP_V6_IOP_VBAR_SIZE)
			return dev_err_probe(dev, -EINVAL,
					     "invalid ASCWrap v6 provider resources\n");

		ret = of_property_read_string(dev->of_node, "apple,firmware-role",
					      &firmware_role);
		if (ret)
			return dev_err_probe(dev, ret,
					     "missing ASCWrap v6 firmware role\n");

		if (!strcmp(firmware_role, "GFX")) {
			mbox->ascwrap_v6.role = APPLE_MBOX_ASCWRAP_V6_ROLE_GFX;
			if (res->start != APPLE_T8140_GFX_WRAPPER_BASE ||
			    iop_vbar_res->start != APPLE_T8140_GFX_IOP_VBAR_BASE)
				return dev_err_probe(dev, -EINVAL,
						     "GFX ASCWrap v6 resources do not match T8140\n");
		} else if (!strcmp(firmware_role, "GFX1")) {
			mbox->ascwrap_v6.role = APPLE_MBOX_ASCWRAP_V6_ROLE_GFX1;
			if (res->start != APPLE_T8140_GFX1_WRAPPER_BASE ||
			    iop_vbar_res->start != APPLE_T8140_GFX1_IOP_VBAR_BASE)
				return dev_err_probe(dev, -EINVAL,
						     "GFX1 ASCWrap v6 resources do not match T8140\n");
		} else {
			return dev_err_probe(dev, -EINVAL,
					     "invalid ASCWrap v6 firmware role\n");
		}

		mbox->ascwrap_v6.wrapper_start = res->start;
		mbox->ascwrap_v6.wrapper_size = resource_size(res);
		mbox->ascwrap_v6.iop_vbar_start = iop_vbar_res->start;
		mbox->ascwrap_v6.iop_vbar_size = resource_size(iop_vbar_res);
		/* T8140 implements both the CPU start and the CPU stop. */
		mbox->ascwrap_v6.missing = 0;

		mbox->wrapper_regs =
			devm_platform_ioremap_resource_byname(pdev, "wrapper");
		if (IS_ERR(mbox->wrapper_regs))
			return PTR_ERR(mbox->wrapper_regs);

		mbox->iop_vbar_regs =
			devm_platform_ioremap_resource_byname(pdev, "iop-vbar");
		if (IS_ERR(mbox->iop_vbar_regs))
			return PTR_ERR(mbox->iop_vbar_regs);

		mbox->regs = (u8 __iomem *)mbox->wrapper_regs +
			     mbox->hw->reg_offset;
	} else {
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res || resource_size(res) < mbox->hw->reg_offset +
						mbox->hw->i2a_recv1 + sizeof(u64))
			return dev_err_probe(dev, -EINVAL,
					     "mailbox register window is too small\n");

		mbox->regs = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(mbox->regs))
			return PTR_ERR(mbox->regs);
		mbox->regs = (u8 __iomem *)mbox->regs + mbox->hw->reg_offset;
	}

	mbox->irq_recv_not_empty =
		platform_get_irq_byname(pdev, "recv-not-empty");
	if (mbox->irq_recv_not_empty < 0)
		return mbox->irq_recv_not_empty;

	/*
	 * Some coprocessors (e.g. the T6021 ANE ASC) have no send-empty line.
	 * apple_mbox_send then polls the A2I control register instead.
	 */
	mbox->irq_send_empty =
		platform_get_irq_byname_optional(pdev, "send-empty");
	if (mbox->irq_send_empty < 0 && mbox->irq_send_empty != -ENXIO)
		return mbox->irq_send_empty;

	spin_lock_init(&mbox->rx_lock);
	spin_lock_init(&mbox->tx_lock);
	mutex_init(&mbox->lifecycle_lock);
	init_completion(&mbox->tx_empty);

	irqname = devm_kasprintf(dev, GFP_KERNEL, "%s-recv", dev_name(dev));
	if (!irqname)
		return -ENOMEM;

	if (apple_mbox_is_g17(mbox))
		ret = devm_request_threaded_irq(dev, mbox->irq_recv_not_empty, NULL,
				apple_mbox_recv_irq,
				IRQF_ONESHOT | IRQF_NO_AUTOEN | IRQF_NO_SUSPEND,
				irqname, mbox);
	else
		ret = devm_request_irq(dev, mbox->irq_recv_not_empty,
				apple_mbox_recv_irq,
				IRQF_NO_AUTOEN | IRQF_NO_SUSPEND, irqname, mbox);
	if (ret)
		return ret;

	if (mbox->irq_send_empty >= 0) {
		irqname = devm_kasprintf(dev, GFP_KERNEL, "%s-send",
					 dev_name(dev));
		if (!irqname)
			return -ENOMEM;

		ret = devm_request_irq(dev, mbox->irq_send_empty,
				       apple_mbox_send_empty_irq,
				       IRQF_NO_AUTOEN | IRQF_NO_SUSPEND,
				       irqname, mbox);
		if (ret)
			return ret;
	}

	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, apple_mbox_quiesce, mbox);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, mbox);
	return 0;
}

static void apple_mbox_remove(struct platform_device *pdev)
{
	struct apple_mbox *mbox = platform_get_drvdata(pdev);

	/* Block new phandle lookups before quiescing the provider. */
	platform_set_drvdata(pdev, NULL);
	apple_mbox_quiesce(mbox);
}

static const struct apple_mbox_hw apple_mbox_t8015_hw = {
	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_T8015_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_ASC_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_ASC_MBOX_A2I_SEND1,

	.i2a_control = APPLE_T8015_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_ASC_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_ASC_MBOX_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_asc_hw = {
	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_ASC_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_ASC_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_ASC_MBOX_A2I_SEND1,

	.i2a_control = APPLE_ASC_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_ASC_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_ASC_MBOX_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_t8140_asc_hw = {
	.ap_initializes_mailboxes = true,

	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_ASC_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_ASC_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_ASC_MBOX_A2I_SEND1,

	.i2a_control = APPLE_ASC_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_ASC_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_ASC_MBOX_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_ascwrap_v6_hw = {
	.reg_offset = APPLE_ASCWRAP_V6_MBOX_OFFSET,
	.is_ascwrap_v6 = true,

	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_ASC_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_ASC_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_ASC_MBOX_A2I_SEND1,

	.i2a_control = APPLE_ASC_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_ASC_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_ASC_MBOX_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_t8140_aop_setup_hw = {
	.control_full = APPLE_ASC_MBOX_CONTROL_FULL,
	.control_empty = APPLE_ASC_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_T8140_AOP_SETUP_A2I_CONTROL,
	.a2i_send0 = APPLE_T8140_AOP_SETUP_A2I_SEND0,
	.a2i_send1 = APPLE_T8140_AOP_SETUP_A2I_SEND1,

	.i2a_control = APPLE_T8140_AOP_SETUP_I2A_CONTROL,
	.i2a_recv0 = APPLE_T8140_AOP_SETUP_I2A_RECV0,
	.i2a_recv1 = APPLE_T8140_AOP_SETUP_I2A_RECV1,

	.has_irq_controls = false,
};

static const struct apple_mbox_hw apple_mbox_m3_hw = {
	.control_full = APPLE_M3_MBOX_CONTROL_FULL,
	.control_empty = APPLE_M3_MBOX_CONTROL_EMPTY,

	.a2i_control = APPLE_M3_MBOX_A2I_CONTROL,
	.a2i_send0 = APPLE_M3_MBOX_A2I_SEND0,
	.a2i_send1 = APPLE_M3_MBOX_A2I_SEND1,

	.i2a_control = APPLE_M3_MBOX_I2A_CONTROL,
	.i2a_recv0 = APPLE_M3_MBOX_I2A_RECV0,
	.i2a_recv1 = APPLE_M3_MBOX_I2A_RECV1,

	.has_irq_controls = true,
	.irq_enable = APPLE_M3_MBOX_IRQ_ENABLE,
	.irq_ack = APPLE_M3_MBOX_IRQ_ACK,
	.irq_bit_recv_not_empty = APPLE_M3_MBOX_IRQ_I2A_NOT_EMPTY,
	.irq_bit_send_empty = APPLE_M3_MBOX_IRQ_A2I_EMPTY,
};

static const struct of_device_id apple_mbox_of_match[] = {
	{ .compatible = "apple,t6031-agx-asc-mailbox", .data = &apple_mbox_asc_hw },
	{ .compatible = "apple,t8122-agx-asc-mailbox", .data = &apple_mbox_asc_hw },
	{ .compatible = "apple,t8140-ascwrap-v6", .data = &apple_mbox_ascwrap_v6_hw },
	{ .compatible = "apple,t8140-asc-mailbox", .data = &apple_mbox_t8140_asc_hw },
	{ .compatible = "apple,t8140-aop-setup-mailbox", .data = &apple_mbox_t8140_aop_setup_hw },
	/*
	 * t6030 gfx-asc mailbox in the boot.bin DTB: it carries no asc-mailbox-v4 fallback so that
	 * kernels without M3 GPU support leave it unbound.
	 */
	{ .compatible = "apple,t6030-agx-asc-mailbox", .data = &apple_mbox_asc_hw },
	{ .compatible = "apple,asc-mailbox-v4", .data = &apple_mbox_asc_hw },
	{ .compatible = "apple,t8015-asc-mailbox", .data = &apple_mbox_t8015_hw },
	{ .compatible = "apple,m3-mailbox-v2", .data = &apple_mbox_m3_hw },
	{}
};
MODULE_DEVICE_TABLE(of, apple_mbox_of_match);

static struct platform_driver apple_mbox_driver = {
	.driver = {
		.name = "apple-mailbox",
		.of_match_table = apple_mbox_of_match,
	},
	.probe = apple_mbox_probe,
	.remove = apple_mbox_remove,
};
module_platform_driver(apple_mbox_driver);

MODULE_LICENSE("Dual MIT/GPL");
MODULE_AUTHOR("Sven Peter <sven@svenpeter.dev>");
MODULE_DESCRIPTION("Apple Mailbox driver");

#if IS_ENABLED(CONFIG_KUNIT)
#include "mailbox-g17-test.h"
#endif
