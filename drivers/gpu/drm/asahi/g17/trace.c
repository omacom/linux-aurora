// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/dma-fence.h>
#include <asm/arch_timer.h>

#include "trace_api.h"

#define CREATE_TRACE_POINTS
#include "trace.h"

bool asahi_g17_trace_publish_enabled(void)
{
	return trace_asahi_g17_publish_enabled();
}

void asahi_g17_trace_publish(u8 engine, u8 qid, u32 ordinal, u32 ctx, u64 seq,
			     struct dma_fence *fence)
{
	trace_asahi_g17_publish(engine, qid, ordinal, ctx, seq,
				fence ? fence->context : 0, fence ? fence->seqno : 0,
				__arch_counter_get_cntvct());
}

void asahi_g17_trace_complete(u8 engine, u8 qid, u32 ordinal, u32 ctx, u64 seq,
			      u64 gpu_start, u64 gpu_end)
{
	if (trace_asahi_g17_complete_enabled())
		trace_asahi_g17_complete(engine, qid, ordinal, ctx, seq, gpu_start,
					 gpu_end, __arch_counter_get_cntvct());
}

void asahi_g17_trace_bind(u8 engine, u8 qid, u32 ctx, u8 kind)
{
	trace_asahi_g17_bind(engine, qid, ctx, kind);
}

bool asahi_g17_trace_doorbell_enabled(void)
{
	return trace_asahi_g17_doorbell_enabled();
}

void asahi_g17_trace_doorbell(u8 engine, u32 power)
{
	trace_asahi_g17_doorbell(engine, power, __arch_counter_get_cntvct());
}

void asahi_g17_trace_message(u8 kind, u64 value)
{
	if (trace_asahi_g17_message_enabled())
		trace_asahi_g17_message(kind, value, __arch_counter_get_cntvct());
}

void asahi_g17_trace_event(u8 phase)
{
	if (trace_asahi_g17_event_enabled())
		trace_asahi_g17_event(phase, __arch_counter_get_cntvct());
}

void asahi_g17_trace_teardown(u8 kind, u64 a, u64 b, u64 c)
{
	if (trace_asahi_g17_teardown_enabled())
		trace_asahi_g17_teardown(kind, a, b, c);
}
