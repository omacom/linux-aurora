Queued open-firmware regressions
===============================

Run compilation in a controller buildbox slot. These tests use the current
production C function bodies; allocation, buffer addresses, V4L2 scheduling and
MMIO are explicit host fixtures. Linux UAPI headers and an ASan/UBSan-capable C
compiler are required::

  sh tests/run-host.sh /tmp/avd-queue-host
  python3 tests/program-test.py /tmp/avd-program
  python3 tests/dispatch-test.py /tmp/avd-dispatch
  python3 tests/recovery-test.py /tmp/avd-recovery

The program test runs the H.264 header/slice emitters, open-program builder,
t8112_configure_stream(), queued descriptor submit and SRAM streaming code.
It generates a 1920x1088 IDR program with two I slices and fixture controls/IOVAs, plus
ABI 2 descriptors for FIFO indices 0 through 14. It verifies busy-mailbox
backpressure, the doorbell address, LSR FIFO addressing, decoder IRQ masks,
ring wrap and free-running u32 counter wrap. It checks mailbox draining,
full-tag rejection and fatal completion ownership until quiesced recovery. It also checks that legacy VP
EXEC words retain their existing values. Input SHA256 manifests and generated
C sources accompany the binaries.

For T8112 (host revision 4, firmware v3), VP EXEC start is 0x2b000200 plus
FIFO_IDX, and END is 0x2b000400 plus FIFO_IDX. VP programs retain the legacy
zero low nibble. PP start is 0x2b00020f plus FIFO_IDX: its low nibble carries
the hardware FIFO mask, 15. Do not replace the VP's low nibble with the PP mask
or confuse host revision IDs with firmware image versions.

The dispatch test compiles the production submit/retire functions and V4L2
core detach/held-buffer helpers. It checks early handoff without completing
DMA-owned buffers, held slices, two contexts, deferred capacity, stale work,
prepare/PM/abort failures and exactly-once scheduler release. The matching
V4L2 m2m core source must be present in the kernel checkout.

Routed CM3 test
--------------

Use avd-fw PR #4's linear ABI 2 implementation (baseline commit
7f2f41369d14e0deccbac9b7d2b7c9070e48e66d). Build its images and install the
optional Unicorn Python dependency. The firmware checkout supplies the existing
transport-test.py harness::

  python3 tests/unicorn-queue-test.py /path/to/avd-fw \
      /path/to/avd-fw/build/avd-fw-v3-t1.bin /tmp/avd-program --broken

The baseline must reproduce both missing CM3 decoder interrupt routing and,
when those routes are repaired in the fixture, the incorrect mailbox status
EMPTY test. A firmware fix must program the queue's CM3 IRQ router and use
FIFO status bit 17, rather than IRQ-source bit 2, for outbox EMPTY. Both fixes
are in ``aurora-silicon/avd-fw`` from commit ``50d7f869``. With that firmware
built::

  python3 tests/unicorn-queue-test.py /path/to/fixed-avd-fw \
      /path/to/fixed-avd-fw/build/avd-fw-v3-t1.bin /tmp/avd-program

The test boots the actual reset vector, checks NVIC and CM3 router masks before
calling IRQ wrappers, consumes the production Linux descriptor/program, and
raises VP/PP DONE only after the modeled program/kick. It requires descriptor
state COMPLETE and exactly the expected tagged mailbox-1 notification at FIFO
0, 1 and 14. The firmware's original all-image transport tests remain useful
for refill, ordering, backpressure and error behavior.

Unicorn models decoder events and memory; it does not decode pixels or prove
silicon register behavior, DMA safety, throughput or decoded output correctness.
A controller-approved, single-request hardware decode and software framemd5
comparison remain necessary. The accompanying firmware patch changes queue
mode only; apply it to the pinned open firmware source and retain its MIT
license. The unmodified firmware image cannot produce tagged completions with
the corrected mailbox-status model.

The FIFO status and IRQ-router geometry follow the public MIT emulator at
https://github.com/eiln/avd/blob/main/avd_emu.py. The pinned avd-fw source's
Tansy routing independently identifies the same banks and bit positions.

Concurrent pictures and DMA addresses
------------------------------------

``program-test.py`` also generates per-image subdirectories for v3-t0, v3-t1,
v5-t0 and v5-t2. It runs the production H.264 bank allocator and bank selection,
uses both banks of eight fixture contexts, and emits IDR, P and B pictures.
Every scratch, output, compressed-reference and MV address passed to an address
emitter must be nonzero. The emitted address locations/encodings are exported
alongside the actual descriptor and configure_stream MMIO snapshot for each
slot. Coded-data operands are checked separately by the routed test. Seven
fault-injection runs zero each bank-1 buffer address in turn and must fail.
A missing saved FIFO is rejected at every valid slot before the doorbell,
including when the context contains a different nonzero fallback address.

Build the firmware with the busy-VP scheduling fix, then run::

  for image in v3-t0 v3-t1 v5-t0 v5-t2; do
    python3 tests/unicorn-inflight-test.py /path/to/avd-fw \
        /path/to/avd-fw/build/avd-fw-$image.bin /tmp/avd-program
  done

This test boots each image's reset vector, checks router and NVIC masks, replays
production host MMIO, and verifies every emitted DMA operand at its recorded
program location. It holds VP DONE, PP DONE and the completion mailbox to test
ownership and backpressure. Workloads range from 2 to 15 pictures. On the
15-FIFO images all requested pictures are posted before any VP DONE. v5-t0
advertises only seven physical FIFOs: workloads above seven use a seven-slot
window with slot reuse and a new full completion tag. This is not evidence of
8..15 simultaneous hardware FIFOs on that image. H.264 retains two banks per
context; larger windows represent multiple contexts.

The decoder model asserts that a VP cannot receive the next picture's START
before its current picture's DONE. With the old firmware this reproduces at
slot 1; ``--before`` requires that failure instead of treating it as a pass.
The fixed scheduler waits at that boundary while retaining VP/PP overlap and
strict global SRAM program order. The native firmware test also covers
independent VPs and ordered PP completion.

The live-VP assertion is a conservative decoder ownership model. It proves the
software feed ordering and its correction, not the undocumented silicon cause
of a DART fault. A hardware retest remains necessary to connect this scheduling
defect to a reported zero-address DMA and to verify pixels and performance.

VP9 probability dependencies and picture banks
---------------------------------------------

``vp9-test.py`` compiles the production VP9 codec and V4L2 probability helpers.
It exercises all combinations of the three dependency flags, real backward
adaptation with synthetic counts, independent forward updates, per-bank count
pointers, bank exhaustion, PP retirement while another bank is current, and
old-firmware fallback. It also emits two VP9 programs and their production queue
descriptors/configure_stream snapshots for ``unicorn-vp9-test.py``::

  python3 tests/vp9-test.py /tmp/avd-vp9
  python3 tests/unicorn-vp9-test.py /path/to/avd-fw \
      /path/to/avd-fw/build/avd-fw-v3-t1.bin /tmp/avd-vp9

For the independent legacy comparison, run in a kernel Git checkout containing
the public pre-change revision::

  AVD_VP9_BASELINE_REF=<pre-change-ref> python3 tests/vp9-test.py /tmp/avd-vp9

This builds separate before/after executables and compares the complete legacy
programs and saved probability contexts, including key, inter, frame-parallel,
non-refreshing and error-resilient frames. No firmware capture is used.

The flags below describe whether a picture *produces* a backward-adaptation
dependency for the next prepare. A non-refreshing successor still consumes the
saved context of an earlier refreshing picture. No CPU can read that context
before that producer's counts have been adapted.

================= ============= =============== ============================
refresh_frame_ctx parallel_mode error_resilient  Release next prepare
================= ============= =============== ============================
0                 any           any             During prepare
1                 1             any             During prepare (forward only)
1                 0             1               During prepare (no refresh)
1                 0             0               After tagged VP DONE/adaptation
================= ============= =============== ============================

Preparation, VP callbacks and PP retirement run under queue_mutex. Readiness
checks only observe the published dependency state under queue_lock. Counts
come from coherent allocations and the VP callback uses dma_rmb() before reading
them. Each bank has its own count-pointer table, forward probabilities, header
snapshot, previous-frame flags, FIFO and all DMA scratch. Only PP retirement
releases a bank, and it selects the bank saved with CAPTURE rather than the
context's current builder. A duplicate VP event cannot adapt twice. Failure and
reset retirement never adapt partial counts.

The temporal-MV input color address follows the preceding picture across banks;
the current color output belongs to the new bank. The public MIT VP9 builder at
https://github.com/eiln/avd/blob/main/avid/vp9/halv3.py alternates these operands
(``pps2_tile_addrs``); the single-bank legacy driver used one address for both.
A stream using old queue firmware retains one bank and adapts after full
completion. The ordinary legacy transport and its instruction builder are
unchanged.

The Unicorn test boots the real reset vector and routes VP8/PP IRQs. It checks
that VP(1) is issued while PP(0) owns its FIFO, and that PP(1) waits for PP(0).
It repeats with a blocked VP notification and holds the outbox across PP DONE.
The original H.264 routed tests exercise descriptors without the optional flag
and must retain their single completion per picture.

Optional queue ABI 2 VP notifications
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The feature is in ``aurora-silicon/avd-fw`` from commit
``bf49a31b1036c72be5fdc37eb8232f6c953e0626`` (queue feature bit 2).
This is a backwards-compatible optional ABI 2 feature:

* Info feature bit 2 advertises tagged VP DONE. Without it the host keeps VP9
  at one picture per stream. Features 0/1 and the ordinary completion stay as
  before.
* Host descriptor flag bit 1 requests one VP event. An old host leaves it zero
  and receives no additional tokens. Reserved words must still start at zero.
* After VP DONE, firmware writes the full submitted tag at descriptor +0x38,
  DMB, writes 1 at +0x3c, DMB, then sends
  ``0x54540000 | (tag & 0xfff0) | index``. These records live independently of
  the PP completion_tag. The host validates the prefix, marker, short sequence,
  full saved tag and submitted ownership before recording the event.
* Outbox backpressure retains the VP event. A successful opted-in picture's PP
  completion cannot be notified before its VP event. Errors may finish without
  a VP event and require the existing reset/ownership handling.
* Neither the VP token nor its CPU callback permits buffer, descriptor, FIFO or
  bank reuse. Only the original tagged PP completion does that. A missing VP
  event remains bounded by the original per-picture watchdog.

AV1 picture overlap and codec VP pools
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``av1-test.py`` compiles the production AV1 codec, entropy defaults, open
program builder and queued descriptor writer with ASan/UBSan. It is also
called by the registered ``avd-codec-tail-tests.py`` host check. Run::

  AVD_AV1_BASELINE_REF=d62b4894 python3 tests/av1-test.py /tmp/avd-av1
  python3 tests/unicorn-av1-test.py /path/to/avd-fw \
      /path/to/avd-fw/build/avd-fw-v5-t2.bin /tmp/avd-av1

The independent legacy comparison builds before/after executables and compares
complete emitted programs plus initialized CDF/map tails for twelve header cases.
The flag table covers primary-reference/default CDF cases, both producer and
consumer disable_frame_end_update_cdf, segmentation enabled/update_map on both
pictures, and all 256 refresh masks. Pending request p_req values are used for
readiness; p_cur still describes an earlier picture and must not be used.

Coherent CDF output and completion boundary
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The public AV1 specification section 7.7 updates frame CDFs from the saved
context-update tile; sections 7.19 and 7.20 save motion fields and segmentation:
https://aomediacodec.github.io/av1-spec/ . CDF adaptation depends on entropy
parsing, independently of pixel reconstruction. The driver supplies probs_out
to the VP program, while the open firmware starts ordered PP reconstruction
after VP DONE. These are the implementation grounds for using VP completion
for CDF output; they are not hardware measurements of memory visibility.

Queued AV1 now directs probs_out to a separate dma_alloc_coherent allocation
in each bank. Input probabilities and output probabilities remain distinct.
Preparation seeds output with input, preserving frames with frame-end update
disabled. A named live producer blocks until its full-tag VP callback, invoked
by the queue after dma_rmb(). The consumer copies the coherent output into its
own input and seeded output. It never samples a PP-owned CAPTURE CDF tail.
This follows the existing coherent VP9 count-output completion boundary.
The firmware descriptor DMB alone is not a substitute for DMA coherence or
proof of the silicon's completion semantics. Physical CDF completion timing,
pixel hashes and performance must still be qualified on hardware.

Only successful PP retirement copies coherent CDF output into the producer's
CAPTURE tail, retaining the saved-reference ABI for subsequent pictures after
bank reuse. Error/reset retirement discards output. Banks, FIFOs, CAPTURE and
requests remain owned through PP; VP completion only publishes readiness.
The existing CPU-mappable CAPTURE requirement is retained. No CPU DMA sync is
attempted on a CAPTURE allocation still owned by PP.

Queue firmware advertising VP-DONE feature bit 2 enables two scratch banks;
with a validated codec pool (feature bit 3), AV1 uses one bank per advertised
AV1 VP plus one retained by PP, bounded by physical FIFO slots. Legacy
transport and queue firmware without tagged VP DONE retain one bank and
identical programs and initialized tails. No firmware change is required.

Each pending request uses exact ref_frame_idx/primary_ref_frame timestamps,
independently of refresh order. The consumer's disable_frame_end_update_cdf
does not remove its input dependency. refresh_frame_flags only changes which
slots userspace subsequently names, not the dependency of this picture.

=============================== ==============================================
Consumer / producer condition   Prepare dependency while producer is in flight
=============================== ==============================================
Named live reference            Wait for its tagged VP DONE on any engine
PRIMARY_REF_NONE                No primary CDF or map dependency
Intra or error resilient        Default CDF; map dependency still checked
Primary slot names retired ref  Read saved CAPTURE CDF
Primary CDF, producer end update Enabled: coherent output after tagged VP DONE
Primary CDF, producer end update Disabled: seeded coherent output after VP DONE
Consumer end update disabled    Still consumes selected primary CDF
Consumer segmentation disabled  No previous-map read
Consumer segmentation enabled   Updated producer map: wait for PP (same Mi size)
Producer map not updated         Copy immutable saved map into consumer bank
Different reference Mi size      Zero map; no map dependency
USE_REF_FRAME_MVS                Wait for PP of any named in-flight reference
refresh_frame_flags 0..255       No additional dependency; named slot decides
=============================== ==============================================

Segmentation and motion-field outputs remain in CAPTURE and retain their PP
barriers. Both consumer update_map values retain previous-map input when
segmentation is enabled. Saved maps and CDFs are copied into consumer-owned
storage, so producer bank reuse cannot invalidate a live consumer's input.
Busy banks publish fully prepared metadata with release/acquire; allocation
failures and PP error/reset retirement select the saved CAPTURE bank.

For unchanged geometry, a free queued bank reuses scratch allocations and
zeroes their contents, preserving fresh-allocation semantics without repeated
DMA mapping and deferred unmapping in prepare. Size changes reallocate; the
single-bank legacy path retains its original allocation behavior. The zero
previous-segment map retains its existing size-based reuse.

The Unicorn run boots the v5-t2 CM3 reset vector, routes AV1 and PP IRQs,
and checks production AV1 program words/descriptors/configuration. It shows
VP(1) issued during PP(0), concurrent VPs 9/10/11, a fourth picture waiting for
VP9 DONE, reverse engine completion, full-tag events, ordered PP and blocked
mailboxes. Routed errors on each AV1 engine drain every owner and reset.
Host tests cover all producer VP/map/MV barriers, all refresh masks, adaptive
CDF output distinct from input, poisoned CAPTURE tails, PP CDF persistence,
error discard, private consumer copies after producer bank reuse, duplicate
callbacks, allocation failures and scratch reuse/zeroing/resize. These tests
model DMA completion; they do not decode pixels or prove physical visibility.

H.264 conformance fixes
----------------------

The H.264 cases live in ``program-test.py`` and ``dispatch-test.py``. The
program test also compiles the emitters from the pinned merged rewrite,
``757fc07a6930ee12b04fe034138dc9880dc4ff3f``, with the same synthetic controls
and addresses. It retains ``before.json``, ``after.json`` and generated C.
Set ``AVD_H264_BASELINE_ROOT`` to an exported baseline driver directory when
running outside a Git checkout. ``AVD_H264_BASELINE_REF`` can select another
Git revision explicitly.

The complete-program matrix covers zero through sixteen dense DPB entries,
a sparse DPB, I/P/B slices, CAVLC/CABAC, explicit/implicit weights,
first/continuation slices and both address encodings. Dense programs remain
byte-identical. Sparse programs may change only the low four-bit packed DPB
slot operand of reference-list commands. All 65536 DPB occupancy maps are
checked, including VALID entries without ACTIVE. The reference-header count
now counts all VALID entries, matching the headers actually emitted; an
inactive VALID entry can therefore change the count field in each header.

The production prepare function is exercised with two banks: picture 0 stays
submitted while picture 1 holds slices, rejects syntax or malformed data,
and is restarted. Rejection drops only picture 1's unsubmitted slices and
job table. The saved CAPTURE bank selects PP retirement even when the other
bank is the current builder. SP/SI switching slices, FMO, non-increasing
macroblock positions and unequal luma/chroma bit depths are rejected. A repeated first slice before the previous picture
finishes is rejected before bank selection can discard its held slices. Both H.264 and HEVC applied SPS validation is tested.

The dispatch test compiles the production queue initializer, probes
CREATE_BUFS capabilities before S_FMT, and checks format switches remove
stale HOLD support. Pictures with 1, 3, 4, 9, 10, 12 and 20 slices each reach
the transport exactly once. Intermediate slices use the core HOLD helper:
only OUTPUT completes, CAPTURE and its bank remain owned by the builder.
The final slice uses ``v4l2_m2m_job_finish_detach()``: both buffers leave the
scheduler atomically, neither completes early, and the bank remains owned
until tagged PP retirement. Advertising HOLD does not prevent that final
handoff. The legacy transport follows the same slice collection path with
one bank; the queued transport permits two banks.

These are host control/program/ownership regressions. The lead's conformance
run must still establish decoded pixels and absence of hardware timeouts.

A non-IDR I picture with no VALID DPB entries now uses the existing
reference-free intra header. The intended changes are the header INTRA bit,
clearing the H.264 NOT_IDR flag, and omitting the empty reference-table
preamble. Across all four revision/address variants, weight modes and
first/continuation slices, its complete program must equal the baseline IDR
program exactly. Pictures with references retain their original header.
The baseline comparison retains all 32 before/after empty-DPB cases.

Film grain controls, frame flags and the legacy syntax/program emitters are
unchanged. The independent baseline comparison includes film grain control
values and SHOW_FRAME/SHOWABLE_FRAME combinations at 8 and 10 bits. It checks
program/tail parity, not grain synthesis. The stateless AV1 frame UAPI has no
show_existing_frame decode control: userspace presents the existing CAPTURE
buffer without a new tile decode. No VP event retires that CAPTURE buffer or
its media request early. Hardware qualification must include those presentation
paths as well as pixel hashes; this change makes no new grain-support claim.
