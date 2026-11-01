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
