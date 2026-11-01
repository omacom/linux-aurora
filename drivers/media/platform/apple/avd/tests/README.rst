Queued open-firmware regressions
===============================

Run compilation in a controller buildbox slot. These tests use the current
production C function bodies; allocation, buffer addresses, V4L2 scheduling and
MMIO are explicit host fixtures. Linux UAPI headers and an ASan/UBSan-capable C
compiler are required::

  sh tests/run-host.sh /tmp/avd-queue-host
  python3 tests/program-test.py /tmp/avd-program
  python3 tests/dispatch-test.py /tmp/avd-dispatch

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

Use public MIT ``aurora-silicon/avd-fw`` branch
``avd/queue-abi2-sram-ring`` from commit
``bf49a31b1036c72be5fdc37eb8232f6c953e0626`` (queue feature bit 2).
Rebuild with ``make test && sh tests/build-all.sh``. The former local
VP-DONE patch is superseded by that commit and is no longer shipped.
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

AV1 conservative picture overlap
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

``av1-test.py`` compiles the production AV1 codec, entropy defaults, open
program builder and queued descriptor writer with ASan/UBSan. It is also
called by the registered ``avd-codec-tail-tests.py`` host check. Run::

  AVD_AV1_BASELINE_REF=a1b76856 python3 tests/av1-test.py /tmp/avd-av1
  python3 tests/unicorn-av1-test.py /path/to/avd-fw \
      /path/to/avd-fw/build/avd-fw-v5-t2.bin /tmp/avd-av1

The independent legacy comparison builds before/after executables and compares
complete emitted programs plus initialized CDF/map tails for eight header cases.
The flag table covers primary-reference/default CDF cases, both producer and
consumer disable_frame_end_update_cdf, segmentation enabled/update_map on both
pictures, and all 256 refresh masks. Pending request p_req values are used for
readiness; p_cur still describes an earlier picture and must not be used.

Visibility evidence and limit
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The public AV1 specification sections 7.7, 7.19 and 7.20 define frame CDF,
motion-field and segmentation storage semantics:
https://aomediacodec.github.io/av1-spec/ . They do not define this decoder's
VP/PP DMA boundary. In our driver ``set_header()`` supplies probs_out, seg and
w_color output addresses to the VP program; ``avd_av1_set_prob()`` seeds CDF
output and ``avd_av1_set_seg()`` initializes non-updated maps. The open firmware
``avdq_vp_done()`` changes VP to PP_READY and ``notify_vp()`` publishes a tag
with barriers. Neither reads the AV1 output memory nor supplies an AV1-specific
DMA visibility guarantee. A CM3 DMB orders its descriptor stores; it does not
prove the decoder's prior external tail writes are CPU-visible.

There is therefore **no evidence here proving that a hardware-written AV1 CDF,
segmentation map or motion field is visible at VP DONE**. There is also no
proof that they are written only at PP DONE. Unicorn injects DONE and cannot
establish either assertion. The implementation keeps those dependencies until
PP retirement instead of inferring a visibility contract from their operands.
No CPU DMA sync is attempted on a CAPTURE allocation still owned by PP.

Safe subset
^^^^^^^^^^^

Queue firmware advertising VP-DONE feature bit 2 enables two scratch banks;
legacy transport and older queue firmware retain one bank and identical
programs. The CDF output, segmentation output and motion-field/color output
already live in each picture's distinct CAPTURE tail. Each scratch bank owns
its FIFO, CDF input, pipe/filter/tile scratch, a saved initialized map and a
separate map input snapshot. Only PP retirement releases its bank. VP DONE
never releases storage or publishes hardware-written tails for AV1.

For a pending consumer, each busy producer is tested by the exact timestamp
named through ref_frame_idx/primary_ref_frame, rather than the preceding frame
or a global refresh-context flag. refresh_frame_flags controls which slots
userspace subsequently names; it does not create a dependency by itself.

=============================== ==============================================
Consumer / producer condition   Prepare dependency while producer is in flight
=============================== ==============================================
PRIMARY_REF_NONE                No primary CDF or map dependency
Intra or error resilient        Default CDF; map dependency still checked
Primary slot names retired ref  None on an in-flight picture
Primary CDF, producer end update Enabled: wait for producer PP retirement
Primary CDF, producer end update Disabled: use immutable prepared CDF input copy
Consumer end update disabled    Does not remove its CDF input dependency
Consumer segmentation disabled  No previous-map read
Consumer segmentation enabled   Updated producer map: wait for PP (same Mi size)
Producer map not updated         Copy immutable saved map into consumer bank
Different reference Mi size      Zero map; no map dependency
USE_REF_FRAME_MVS                Wait for PP of any named in-flight reference
refresh_frame_flags 0..255       No additional dependency; named slot decides
=============================== ==============================================

Both consumer update_map values retain the previous-map input when segmentation
is enabled, as the existing program supplies it in either case. A disabled
producer segmentation map is saved as zeroes. Producer map snapshots are copied
into consumer-owned coherent storage, so producer PP retirement and bank reuse
cannot invalidate a live successor's VP input. CDF snapshots are similarly
copied into the consumer's CDF input. No hardware-written tail is sampled by
these copies. Busy banks publish fully prepared metadata with release/acquire;
allocation failures and PP error/reset retirement release the saved CAPTURE's
bank, independent of the current builder.

This subset needs no extra VP token: firmware already waits for VP(N) DONE
before feeding VP(N+1). AV1 descriptors retain their ordinary completion-only
flags. The Unicorn run boots the v5-t2 CM3 reset vector, routes AV1 and PP IRQs,
and checks production AV1 program words/descriptors/configuration. It shows
VP(1) issued during PP(0), PP serialization and blocked completion mailboxes.
Host tests poison in-flight CAPTURE tails to ensure snapshot consumers never
read them. Pixels, physical DMA visibility and throughput remain hardware
qualification work for the lead. Full CDF/MV dependency release at VP DONE is
still a separate evidence requirement.
