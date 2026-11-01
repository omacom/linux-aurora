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
FIFO status bit 17, rather than IRQ-source bit 2, for outbox EMPTY. With that
firmware and its status-bit-corrected transport harness::

  git -C /path/to/fixed-avd-fw apply --unidiff-zero \
      /path/to/linux/drivers/media/platform/apple/avd/tests/avd-fw-queue-fixes.patch
  # Rebuild the firmware images, then:
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
