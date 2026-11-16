/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _LINUX_DMA_APPLE_SIO_H
#define _LINUX_DMA_APPLE_SIO_H

#include <linux/types.h>

/**
 * struct apple_sio_idle - idle transfer of an Apple SIO channel
 * @addr: DMA address of the buffer
 * @len: buffer length; 0 removes the channel's idle transfer
 * @period_len: length of each of its descriptors, not zero and at most @len
 *
 * Passed as &dma_slave_config.peripheral_config in a dmaengine_slave_config()
 * call of its own, which sets or removes the idle transfer and nothing else:
 * the channel's other settings take a separate call.
 *
 * While a channel has an idle transfer, it plays that buffer cyclically
 * whenever no other transfer runs, and goes from one to the other in
 * descriptor order: terminating a transfer hands over to the idle one instead
 * of stopping the channel, and a transfer issued later follows the idle one's
 * issued descriptors. The peripheral sees no gap in between.
 *
 * dmaengine_synchronize() returns once the descriptors of a terminated
 * transfer have completed. If they do not in time, it stops the channel and
 * waits for them first; the idle transfer then starts again, after a gap.
 * Only an acknowledged TERMINATE counts as stopped. If the channel cannot be
 * confirmed stopped, dmaengine_synchronize() warns, as it cannot fail.
 * Removing the idle transfer stops the channel if nothing else runs, and
 * returns 0 once its descriptors completed. If the channel cannot be
 * confirmed stopped, it returns an error and the buffer is still in use.
 *
 * Setting an idle transfer fails with -EBUSY while the channel has one, or
 * while it cannot take over the channel's issued descriptors. Setting and
 * removing it, terminating and synchronizing must not run concurrently on a
 * channel: its client serializes them. A channel that never had an idle
 * transfer works as before.
 */
struct apple_sio_idle {
	dma_addr_t addr;
	size_t len;
	size_t period_len;
};

#endif
