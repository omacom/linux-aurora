// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/workqueue.h>

#include "shim.h"

void sep_cancel_work_sync(void *work)
{
	cancel_work_sync(work);
}

void sep_flush_work(void *work)
{
	flush_work(work);
}

void sep_cancel_delayed_work_sync(void *work)
{
	cancel_delayed_work_sync(work);
}
