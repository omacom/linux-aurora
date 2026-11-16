/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
/*
 * Command lifecycle tracepoints of the G17 runtime. Times are joined by
 * (ctx, seq); `counter` is the CPU's virtual counter and `gpu_start`/`gpu_end`
 * come from the firmware's completion record, so a reader can place firmware
 * dispatch and completion on the host timeline.
 */
#undef TRACE_SYSTEM
#define TRACE_SYSTEM asahi_g17

#if !defined(ASAHI_G17_TRACE_H) || defined(TRACE_HEADER_MULTI_READ)
#define ASAHI_G17_TRACE_H

#include <linux/tracepoint.h>

TRACE_EVENT(asahi_g17_publish,
	TP_PROTO(u8 engine, u8 qid, u32 ordinal, u32 ctx, u64 seq,
		 u64 fence_context, u64 fence_seqno, u64 counter),
	TP_ARGS(engine, qid, ordinal, ctx, seq, fence_context, fence_seqno, counter),
	TP_STRUCT__entry(
		__field(u8, engine)
		__field(u8, qid)
		__field(u32, ordinal)
		__field(u32, ctx)
		__field(u64, seq)
		__field(u64, fence_context)
		__field(u64, fence_seqno)
		__field(u64, counter)
	),
	TP_fast_assign(
		__entry->engine = engine;
		__entry->qid = qid;
		__entry->ordinal = ordinal;
		__entry->ctx = ctx;
		__entry->seq = seq;
		__entry->fence_context = fence_context;
		__entry->fence_seqno = fence_seqno;
		__entry->counter = counter;
	),
	TP_printk("engine=%u qid=%u ordinal=%u ctx=%u seq=%llu fence=%llu:%llu counter=%llu",
		  __entry->engine, __entry->qid, __entry->ordinal, __entry->ctx,
		  __entry->seq, __entry->fence_context, __entry->fence_seqno,
		  __entry->counter)
);

TRACE_EVENT(asahi_g17_complete,
	TP_PROTO(u8 engine, u8 qid, u32 ordinal, u32 ctx, u64 seq,
		 u64 gpu_start, u64 gpu_end, u64 counter),
	TP_ARGS(engine, qid, ordinal, ctx, seq, gpu_start, gpu_end, counter),
	TP_STRUCT__entry(
		__field(u8, engine)
		__field(u8, qid)
		__field(u32, ordinal)
		__field(u32, ctx)
		__field(u64, seq)
		__field(u64, gpu_start)
		__field(u64, gpu_end)
		__field(u64, counter)
	),
	TP_fast_assign(
		__entry->engine = engine;
		__entry->qid = qid;
		__entry->ordinal = ordinal;
		__entry->ctx = ctx;
		__entry->seq = seq;
		__entry->gpu_start = gpu_start;
		__entry->gpu_end = gpu_end;
		__entry->counter = counter;
	),
	TP_printk("engine=%u qid=%u ordinal=%u ctx=%u seq=%llu gpu=%llu..%llu counter=%llu",
		  __entry->engine, __entry->qid, __entry->ordinal, __entry->ctx,
		  __entry->seq, __entry->gpu_start, __entry->gpu_end, __entry->counter)
);

TRACE_EVENT(asahi_g17_bind,
	TP_PROTO(u8 engine, u8 qid, u32 ctx, u8 kind),
	TP_ARGS(engine, qid, ctx, kind),
	TP_STRUCT__entry(
		__field(u8, engine)
		__field(u8, qid)
		__field(u32, ctx)
		__field(u8, kind)
	),
	TP_fast_assign(
		__entry->engine = engine;
		__entry->qid = qid;
		__entry->ctx = ctx;
		__entry->kind = kind;
	),
	TP_printk("engine=%u qid=%u ctx=%u kind=%u",
		  __entry->engine, __entry->qid, __entry->ctx, __entry->kind)
);

TRACE_EVENT(asahi_g17_doorbell,
	TP_PROTO(u8 engine, u32 power, u64 counter),
	TP_ARGS(engine, power, counter),
	TP_STRUCT__entry(
		__field(u8, engine)
		__field(u32, power)
		__field(u64, counter)
	),
	TP_fast_assign(
		__entry->engine = engine;
		__entry->power = power;
		__entry->counter = counter;
	),
	TP_printk("engine=%u power=%#x counter=%llu", __entry->engine, __entry->power,
		  __entry->counter)
);

TRACE_EVENT(asahi_g17_message,
	TP_PROTO(u8 kind, u64 value, u64 counter),
	TP_ARGS(kind, value, counter),
	TP_STRUCT__entry(
		__field(u8, kind)
		__field(u64, value)
		__field(u64, counter)
	),
	TP_fast_assign(
		__entry->kind = kind;
		__entry->value = value;
		__entry->counter = counter;
	),
	TP_printk("kind=%u value=%#llx counter=%llu", __entry->kind, __entry->value,
		  __entry->counter)
);

/* Failed-context teardown and render slot reuse (kind: see g17/trace.rs teardown()). */
TRACE_EVENT(asahi_g17_teardown,
	TP_PROTO(u8 kind, u64 a, u64 b, u64 c),
	TP_ARGS(kind, a, b, c),
	TP_STRUCT__entry(
		__field(u8, kind)
		__field(u64, a)
		__field(u64, b)
		__field(u64, c)
	),
	TP_fast_assign(
		__entry->kind = kind;
		__entry->a = a;
		__entry->b = b;
		__entry->c = c;
	),
	TP_printk("kind=%u a=%#llx b=%#llx c=%#llx", __entry->kind, __entry->a, __entry->b, __entry->c)
);

TRACE_EVENT(asahi_g17_event,
	TP_PROTO(u8 phase, u64 counter),
	TP_ARGS(phase, counter),
	TP_STRUCT__entry(
		__field(u8, phase)
		__field(u64, counter)
	),
	TP_fast_assign(
		__entry->phase = phase;
		__entry->counter = counter;
	),
	TP_printk("phase=%u counter=%llu", __entry->phase, __entry->counter)
);

#endif

#undef TRACE_INCLUDE_PATH
#define TRACE_INCLUDE_PATH .
#undef TRACE_INCLUDE_FILE
#define TRACE_INCLUDE_FILE trace
#include <trace/define_trace.h>
