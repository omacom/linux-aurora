/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Entry points of the G17 command lifecycle tracepoints, called from Rust
 * (g17/trace.rs).
 */
#ifndef _ASAHI_G17_TRACE_API_H
#define _ASAHI_G17_TRACE_API_H

#include <linux/types.h>

struct dma_fence;

bool asahi_g17_trace_publish_enabled(void);
void asahi_g17_trace_publish(u8 engine, u8 qid, u32 ordinal, u32 ctx, u64 seq,
			     struct dma_fence *fence);
void asahi_g17_trace_complete(u8 engine, u8 qid, u32 ordinal, u32 ctx, u64 seq,
			      u64 gpu_start, u64 gpu_end);
void asahi_g17_trace_bind(u8 engine, u8 qid, u32 ctx, u8 kind);
bool asahi_g17_trace_doorbell_enabled(void);
void asahi_g17_trace_doorbell(u8 engine, u32 power);
void asahi_g17_trace_message(u8 kind, u64 value);
void asahi_g17_trace_event(u8 phase);
void asahi_g17_trace_teardown(u8 kind, u64 a, u64 b, u64 c);

#endif /* _ASAHI_G17_TRACE_API_H */
