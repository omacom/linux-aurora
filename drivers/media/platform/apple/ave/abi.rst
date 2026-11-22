.. SPDX-License-Identifier: GPL-2.0-only

Apple AVE firmware ABIs
======================

The firmware TEXT CRC chooses the wire layout. SoC match data supplies register
addresses, boot arguments and device identity. These are separate from streaming
qualification: identifying a firmware does not qualify its encoder payloads.

9003.78.0 keeps its existing boot request, host clock synchronization, command
IDs, contiguous surface descriptors and reply envelope. The KUnit oracle in
``tests/ave-9003-reference.h`` freezes the driver builders and layout from Linux
commit 23e431a98962. Tests compare every byte of AVC and HEVC Start and Process
at 1920x1080 with B-frames enabled/disabled, and 3840x2160 with B-frames enabled.
They compare both deterministic synthetic DART addresses and masked images.
The constant image tables in ave-image.c are unchanged by this refactor.

6070.11.1 is selected only for T6000 TEXT CRC 460c7a7c or T8112 TEXT CRC 25bfd039,
with the corresponding SoC identity. Boot, seven-channel discovery, Config,
Open, Close and scratch-confirmed Halt are implemented from the clean derived
SPEC-6070-lifecycle.md ledger. Start and ordinary Process writers implement the
clean revision-2/P1 field ledgers: fixed-capacity address banks, split output
arrays, four interleaved entropy sets, rational frame rate, one initial-QP word,
retained codec-state partitions and per-slot HEVC slice-header tables. The
session allocator provides distinct backing for every live bank and rejects
non-16-KiB-aligned IOVAs.

Streaming is enabled for one quality-first NV12 constant-quality session of
8-bit H.264 (Baseline, Main or High) with low delay: I and P pictures, two
reconstruction slots that alternate, frame widths in whole macroblocks. B
pictures, HEVC, Main 10, bit-rate modes, speed priority, standalone LRME and
Reset remain unavailable; a session asking for B pictures is encoded without
them, the others are refused. No Priority command is sent. Complete is the
normal drain barrier; Flush is reserved for cancellation before Stop and Close.
KUnit checks the 1080p and 4K fields directly from the clean ledgers and embeds
no Apple command image.

Lifecycle-only validation
-------------------------

Build with CONFIG_DEBUG_FS. On a supported 6070 device, write ``1`` (AVC) or ``2``
(HEVC) to ``/sys/kernel/debug/apple-ave-<dev>/lifecycle_test``. This synchronous operation
runs boot, Config, Open, Close and Halt, then requests runtime suspend. It does
not allocate encoding surfaces or issue Start, Process or uninit for an encoder
that was never initialized. A second write exercises a cold restart and DATA
restore. An existing session makes the operation return EBUSY.

The platform must supply the extra-0 register resource as ``extra0`` or register
resource index 3, with at least 0x24 bytes on T6000 or 0x4000 bytes on T8112.
The driver uses the translated platform resource address, maps its containing
16 KiB page through the device DMA API and puts that dynamically allocated IOVA
in Config. Platform DT/binding changes are separate from this driver change.
No fixed firmware or register IOVA is used as an ordinary DMA allocation.

Before every boot the driver checks the firmware's IOMMU translations. DATA must
match the loader-exported pre-start CRC at probe; subsequent cold boots restore
that pristine image and verify its CRC. Firmware reservations and translations
remain persistent. IPC and extra-0 mappings are per-boot resources.

Halt clears scratch0, sends ID 14 without expecting a ring reply, and waits up to
five seconds for scratch0 0x08042006. Only then does the driver stop the IOP and
check control readback. Runtime PM performs the power transition; a second boot
is refused until suspend/resume has intervened. A missing Halt confirmation or
failed lifecycle leaves the firmware broken and retains DMA mappings, runtime
power and the module reference. A forced unload or in-place restart is not a
recovery procedure.

Hardware acceptance remains independent of KUnit: check both SoC VCBAR readbacks,
exact hello, ring dimensions, Config/Open/Close completion IDs and status, Halt
scratch confirmation, actual power gating and a second cold boot. Encoding on
9003 requires the existing hardware regression lane; these software tests do not
establish hardware equivalence.

The 9003 runtime recovery path retains its upstream quarantine and reset-witness
checks. It is deliberately unavailable for 6070: a broken lifecycle must keep
its mappings and power until scratch-confirmed Halt can establish quiescence.
