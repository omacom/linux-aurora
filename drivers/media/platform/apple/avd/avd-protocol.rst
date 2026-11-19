AVD queued transport implementation status
==========================================

The pool owns sixteen independently tagged slots.  A source/destination pair
and a runtime-PM reference remain owned by the slot until in-order retirement
within its context.  A duplicate or obsolete tag cannot complete a reused
slot.  Inbox-full leaves a prepared slot reserved and retries without losing
its buffers.  Timeout or streamoff resets the whole engine before failing all
slots affected by that reset.  Slot deadlines survive inbox retries.

The first commit adding avd-queue.h is the shared interface.  Later additions
only extend it.  Codec callbacks own codec-private state and must release that
state from retire(), including when prepare() failed part way through.

Open firmware compatibility
--------------------------

The deployed, untagged open-firmware protocol has a one-frame limit.  Existing
codec run() callbacks build a DMA-visible per-slot program; only the backend
writes those instructions to the VP FIFO.  That compatibility backend retains
the old host-side PP kick.  A native frame_ops program must use the queued
avd-fw transport before it can replace the compatibility path; this backend
returns EOPNOTSUPP for native programs rather than guessing their wire format.

Partial HOLD_CAPTURE slice requests use buf_done_and_job_finish(), returning
the copied OUTPUT slice while CAPTURE remains held on the ready list. Final
slices use job_finish_detach(): it atomically transfers both ready buffers to
the frame slot and releases m2m dispatch without completing either buffer.
The next picture can prepare while the decoder still owns the earlier frame.
The slot returns those buffers only after its tagged completion or quiesced
recovery. The queue capability flags are never toggled around job_finish().
This requires the matching V4L2 m2m detach API; an external-module deployment
must install the matching m2m module as well as apple-avd.

Apple Borage transport
----------------------

Only T6040 has Borage match data.  The Apple backend contains the 64-byte
little-endian connection configuration, session command builders, decode
prefix, eight-entry inbox publishing, a 64-entry completion shadow and bounded
20-word result parsing.  Type 7 is VP/parsing completion and never releases a
buffer.  The notification address rule has one definition,
AVD_APPLE_NOTIFY_BASE, so a capture can correct it.

Boot checks a nonzero ITCM span and a nonzero/non-all-ones version word before
accepting a preloaded image.  A separately installed firmware image can restore
ITCM after power loss.  The last image word is zero padded without reading
past the firmware object.  No Apple image is embedded in this source tree.

Activation remains disabled in match data: the supplied clean specification
has no ITCM register offsets, link programming sequence, result-area address,
result field layout or final-completion value.  It would be unsafe to invent
these or reuse the different open-firmware mailbox.  The closest executable
path is the open compatibility backend.  The Apple register description and
reply layout must be completed together from clean facts; setting verified
alone is insufficient.  The DT must then describe the complete Apple aperture
as an owned named resource before probe can map it.

Unresolved field ledger
-----------------------

This is the single ledger for fields marked [C] in the common transport.  No
semantic names have been assigned to the unknown bits.  Codec-specific fields
remain in their codec packers' ledgers.

==============================  ================================================
Field                           Source and required evidence
==============================  ================================================
Config +00/+08/+10/+18/+20/      SPEC 3.2: exact values, meanings unproved;
28/+2c/+30/+38                   capture connection construction and LinkUp
ITCM start/end/clear/enable      SPEC 3.1: operations, offsets/values absent;
                                clean per-SoC register ledger required
IMEM version-word offset        SPEC 3.1: display format proved, offset absent
Mailbox link register writes    SPEC 3.2: aperture 1, state 4, LinkUp proved;
                                offsets and bit operations absent
Mailbox ready register          SPEC 3.1: status 1, poll 10 us; offset absent
Mailbox notification word       SPEC 4.2; expected inbox-entry address from MIT
                                aura trace_avd.py V3 precedent; capture Borage
Inbox consumption/full rule     SPEC 4.2: host positions modulo eight;
                                hardware consumer register/doorbell absent
Result-area base/indexing       SPEC 4.3: 64 entries, 20 words; base and mailbox
                                notification interpretation absent
Reply type/client/frame/status  SPEC 4.3: correlation proved, word layout absent;
                                synthetic KUnit layout is not a hardware fact
Final reply type/status values  SPEC 4.3: only VP done = 7 proved; capture final
                                completion and error replies before activation
Decode +00 bit 9                SPEC 5.2: boolean argument, meaning unproved;
                                left zero until the producer is established
Decode +50..+5f                 SPEC 5: Borage extension, nonzero use unproved;
                                initialized to zero before codec packing
Commands 3/4 and other values   SPEC 5.1: unproved; never sent
PIODMA mapping stream binding   SPEC 7: three mapper roles proved, Linux binding
                                and per-domain fp/program DVA ownership pending
Retention and clock floors      SPEC 8: per-SoC capture required; no guessed knobs
==============================  ================================================

Licensed implementation references: eiln/avd (MIT), avd_emu.py, and
https://github.com/aurora-silicon/aura/blob/main/proxyclient/hv/trace_avd.py
(MIT).  The Borage facts above come from the supplied clean SPEC sections 3-5;
no binary or captured Apple command images are used.

Firmware-advertised multi-VP pools
---------------------------------

The queued open transport uses the optional ABI 2 feature bit 3 and the AVM2
pool record at SRAM +0x7f80. Its eight u32 words are magic 0x41564d32, version 1,
four codec VP masks (HEVC, H.264, VP9, AV1), an inferred-count codec mask, and a
zero reserved word. Masks must be disjoint, within the firmware VP register
limit and consistent with its codec mask. An absent or malformed extension
keeps the k4m single-VP selector and two H.264/HEVC banks. Reload clears all
cached masks and rotation state before negotiation, so an older image cannot
inherit a prior multi-VP capability.

For a valid pool the host rotates successive submissions through its hardware
VP indices. The prepared program names its private FIFO, not a VP; retrying a
busy mailbox cannot consume a rotation turn or alter the saved program/FIFO.
Separate streams share that rotation, so they also use distinct engines.
Firmware retains exclusive VP picture ownership through DONE and feeds the
SRAM programs in order. PP starts strictly in descriptor submission order,
even when VP completion order differs. Within each context, the host preserves
submission and retirement order.

H.264 and HEVC allocate one scratch bank per advertised VP plus a bank that PP
may still own, bounded by the physical FIFO count. The VP work of the next
picture reads its own bitstream and slice state; reference reconstruction is
consumed by the ordered PP stage. Buffers, instruction FIFOs and scratch banks
remain owned through PP completion, not merely VP DONE. No reference wait or
probability dependency is removed by choosing another engine.

VP9 retains its two banks and producer-side backward-adaptation barrier from
the VP-DONE implementation. Its tagged VP callback remains separate from full
PP retirement. AV1 publishes per-bank coherent CDF output after its tagged VP callback,
while updated segmentation maps and reference motion fields retain their PP
barriers. Only PP retirement releases banks and saves CDFs into CAPTURE tails.
See tests/README.rst for the dependency matrix and hardware qualification limit.

Pool counts live in the firmware image, not an additional host SoC table.
The avd-fw v5-t2 T6040 counts (HEVC 4, H.264 4, VP9 1, AV1 3) are inferred from
T8132 and marked as inferred by the image. The v2/v3/v4/v5-t0/v5-t1 pools use
the public codec variant counts. The T6020 v3-t2 image reuses v3-t1 tunables;
it and all multi-engine silicon behavior still require device qualification.

The host program fixture checks negotiation, invalid/overlapping/absent pool
fallback, reset, rotation, bank limits, saved FIFO retry and unchanged legacy
programs. It emits both ordinary and multi-VP programs. Run the routed test
with ``--multi`` to verify the production H.264 submissions use all advertised
engines, retain each bank through PP, handle reversed VP DONE, and keep PP and
retirement in order. The firmware transport test covers every advertised codec
and engine and tagged VP notifications on every image. These event models do
not decode pixels or establish throughput.
