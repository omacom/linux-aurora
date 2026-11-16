// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Command lifecycle tracepoints (tracefs `events/asahi_g17`). Each call is a
//! static-key test in C while its event is disabled.

use kernel::{bindings, dma_fence::RawDmaFence, sync::Arc};

unsafe extern "C" {
    fn asahi_g17_trace_publish_enabled() -> bool;
    fn asahi_g17_trace_publish(engine: u8, qid: u8, ordinal: u32, ctx: u32, seq: u64,
        fence: *mut bindings::dma_fence);
    fn asahi_g17_trace_complete(engine: u8, qid: u8, ordinal: u32, ctx: u32, seq: u64,
        gpu_start: u64, gpu_end: u64);
    fn asahi_g17_trace_bind(engine: u8, qid: u8, ctx: u32, kind: u8);
    fn asahi_g17_trace_message(kind: u8, value: u64);
    fn asahi_g17_trace_doorbell_enabled() -> bool;
    fn asahi_g17_trace_doorbell(engine: u8, power: u32);
    fn asahi_g17_trace_event(phase: u8);
    fn asahi_g17_trace_teardown(kind: u8, a: u64, b: u64, c: u64);
}

pub(super) const RENDER: u8 = 0;
pub(super) const COMPUTE: u8 = 1;

/// Called once the command's doorbell has been rung.
pub(super) fn publish(engine: u8, qid: u8, ordinal: u32, packet: &Arc<super::job::Packet>) {
    // SAFETY: Pure static-key query.
    if !unsafe { asahi_g17_trace_publish_enabled() } {
        return;
    }
    let fence = packet.completion.fence();
    // SAFETY: `fence` holds a reference for the duration of the call.
    unsafe {
        asahi_g17_trace_publish(engine, qid, ordinal, packet.context.id(), packet.order.sequence,
            fence.raw())
    };
}

/// Called when exact completion with GPU timestamps is retired.
pub(super) fn complete(engine: u8, qid: u8, ordinal: u32, packet: &super::job::Packet,
    gpu: [u64; 2])
{
    // SAFETY: Plain values only.
    unsafe {
        asahi_g17_trace_complete(engine, qid, ordinal, packet.context.id(), packet.order.sequence,
            gpu[0], gpu[1])
    };
}

/// A physical queue was bound to a logical owner: 0 fresh, 1 rebound, 2 same VM.
pub(super) fn bind(engine: u8, qid: u8, ctx: u32, kind: u8) {
    // SAFETY: Plain values only.
    unsafe { asahi_g17_trace_bind(engine, qid, ctx, kind) };
}

/// A work doorbell is about to ring; `power` samples the core power state
/// only while the event is enabled.
pub(super) fn doorbell(engine: u8, power: impl FnOnce() -> u32) {
    // SAFETY: Pure static-key query.
    if !unsafe { asahi_g17_trace_doorbell_enabled() } {
        return;
    }
    // SAFETY: Plain values only.
    unsafe { asahi_g17_trace_doorbell(engine, power()) };
}

pub(super) const MAILBOX: u8 = 0;
pub(super) const CONTROL: u8 = 1;
pub(super) const EFFORT: u8 = 2;

/// A host-to-firmware message: a mailbox message, a device-control record by
/// opcode, or a new performance effort value.
pub(super) fn message(kind: u8, value: u64) {
    // SAFETY: Plain values only.
    unsafe { asahi_g17_trace_message(kind, value) };
}

/// Failed-context teardown: 1 compute fault (qid, stamp), 2 unstamped render retire (slot,
/// ordinal), 3 kill published (cookie, context, held slots | 1 if it skipped the wait for other
/// contexts' compute under memory pressure; slot 0 is never held), 4 kill acknowledged (cookie,
/// qid, or 0x100 for a render-only kill), 5 detach (context, slots, linked), 6 render slot to a
/// new owner (slot, old, new | killed << 63), 7 render-only kill published (as 3).
pub(crate) fn teardown(kind: u8, a: u64, b: u64, c: u64) {
    // SAFETY: Plain values only.
    unsafe { asahi_g17_trace_teardown(kind, a, b, c) };
}

/// Event worker pass boundaries: 0 begin, 1 fences signalled, 2 end.
pub(super) fn event(phase: u8) {
    // SAFETY: Plain values only.
    unsafe { asahi_g17_trace_event(phase) };
}
