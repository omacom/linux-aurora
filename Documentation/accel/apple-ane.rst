.. SPDX-License-Identifier: GPL-2.0-only

Apple ANE firmware preflight
===========================

The shared ANE firmware module recognizes the H16 (``apple,t6041-ane``,
physical T6040/M4 Pro) and H17 (``apple,t8140-ane``) generations. Recognition
is not an inference support claim: both remain unavailable for execution.
They never enter the H13 task-manager or H14 disk-firmware boot paths.

H17 ownership
-------------

Linux has no ANE exclave ownership handoff. The module refuses registration
with ``-EOPNOTSUPP`` and one informational message when an H17 node is
present, including a disabled node. This check occurs before platform core
DMA configuration and automatic power-domain attachment. A probe-time
check alone would be too late. Removing ``apple,exclave-assigned`` does not
establish ownership and does not bypass refusal.

H16 preload handoff
------------------

The bootloader must reserve the personalized TEXT and DATA preload segments
as separate, non-reusable ``no-map`` children of ``/reserved-memory``, attach them in
that order through ``memory-region`` and name them ``firmware-text`` and
``firmware-data``. It must also supply ``apple,firmware-iovas`` as two 64-bit
values: ``0x10000000000`` and ``0x100000c0000``. Their sizes must be
``0xc0000`` and ``0x2ac000``. Physical placement is boot-dependent; board DT
sources must not contain captured physical addresses. The reservations must
remain intact until reboot, including after driver failure. The driver
reads each range from its node and refuses it when any part is Linux
System RAM, so a loader may either reserve the range at boot or leave it
outside the memory map and describe it with a node.

Metadata preflight checks resource geometry and these ranges without MMIO
or power access. Board DTs carry disabled reservation placeholders; while
the loader has not filled and enabled them the device is refused with
``-ENODEV`` and one informational message. Missing reservations fail with
``-ENODATA`` and malformed reservations with ``-EINVAL``. A device that
passes is brought up through the program stage described below once per
boot, from a work item after probe. No accel node or program-loading
interface exists before the program stage.

The J616s enables the ANE and its three DARTs; other H16 boards keep them
disabled until proven. The five real PMGR providers already encode
the system/CPU/MPM/task/base dependency graph. The PMGR binding does not
represent the virtual no-state gates, so no fictitious register-backed
domains are created for them. SID 0 and SID 11 on all three DARTs use the
shared DMA domain; no untested stream remapping is added.

Implemented SPEC-r2 components
------------------------------

The shared transport has an eight-descriptor dynamic channel parser with
unique names, bounded counts, owned-surface bounds and overlap checks. The
runtime uses copied descriptor geometry, verifies the descriptor snapshot,
and checks the complete IO reply envelope. Zero acknowledgment is success;
a nonzero acknowledgment is a firmware refusal, never a retry or success.
Malformed replies, guard changes and timeouts quarantine the instance. A
quarantined queue cannot allocate another BO, and no reset is attempted.

The version-12 RTKit management state machine validates HELLO, endpoint maps,
system endpoint start, IOP/AP power acknowledgments and application endpoint
start ordering. It does not invent responses for unknown message types or
unimplemented system endpoint buffer requests. Its caller must provide one
absolute session deadline and a bounded 64-bit mailbox transport.

H16 task streams begin with little-endian ``u32 1`` and twelve reserved zero
bytes. This marker precedes the first aligned task; it is not a task count.
The initial H16 gate rejects a zero marker, other values and nonzero reserved
bytes. H17 streams open with 0 or 1; the gate refuses every other value.
The public H16 encoder fixes this prologue at commit
``927013f5480768b07c8e62a866e1514c41aba778`` of
``joshuaswarren/mil-hwx-compiler``, in
``plugins/H16G/Encoding/H16GALUEncoder.mm``.

H16 validation helpers walk nine fixed task-header words, the documented
conditional extra word, dense/scatter records and the nine allowed register
blocks. A separate helper checks the eight-byte Mach-O relocation entries,
unique aligned offsets, the actual kernel-section ordinal and addend bounds.
Original-container, tensor, program and aperture validation is provided
separately as described below.

The device-free selftest in ``tools/testing/selftests/drivers/ane`` generates
the SPEC-r2 fp16 add-128 inputs and CPU oracle and verifies all three pinned
buffer hashes. It can also check a supplied HWX against the pinned size/hash.
It never opens an accel device or claims a Linux inference result.

Remaining implementation gates
------------------------------

The preload validator can authenticate owned snapshots of the normalized
resident image with 42 ordered patchbay records. Live snapshot acquisition,
the original-container userspace load ABI, resolved program bindings and
physical boot backend still need implementation. Off-list preload mutations
remain refused.

The 2026-10-31 H16 runtime observation did not capture cold boot, scratch
negotiation, RTKit management, CONFIG_GET or RESOURCE_INFO_GET. The version-12
and 24 MHz predicates remain conservative proposed startup gates; runtime
traffic does not prove them. Resource and allocation ceilings stay unchanged.
H17 additionally requires a Linux ownership backend before direct access.
There are no module parameters to bypass these gates.

Resident image identity
-----------------------

The preload checker requires exactly ``0xc0000`` TEXT bytes and ``0x2ac000``
DATA bytes. It validates the raw four-byte tag and little-endian width of
each of the 42 packed patchbay records, including records at odd offsets.
It then hashes TEXT followed by DATA, substituting zero only for the 20
reviewed mutable value cells and the DATA-relative tunables interval
``[0x17c80, 0x18320)``. The expected SHA-256 is compiled into the validator:
``1f865ed04c4c38f6fdcc4543a8814f818d3cea583de64d22ffe96e3a93ca4697``.
Tags, widths, unlisted values, all TEXT bytes and the DATA zerofill tail
remain authenticated. An off-list change is never accepted merely because
a zero word became nonzero or a value resembles an address.

Validation never changes its input snapshots. A future boot backend must
prove ownership and snapshot stability before calling it, preserving the
resident pages, their personalized mutable values and the reservations.
The checker is not connected to preflight yet: a generic CPU resource claim
cannot distinguish an ordinary busy ``no-map`` reservation from another
owner. No live pages are read by this preparatory path. A successful identity
check does not establish RTKit state, ownership of a running firmware
instance or permission to publish new DMA mappings.

Original-container import and load planning
------------------------------------------

The internal original-HWX import helper bounds the complete 64-bit byte
count before allocation, copies userspace memory once, then validates and
digests that private snapshot. It accepts no caller-provided digest or
section metadata. A failed import leaves the destination untouched, and an
occupied snapshot cannot be overwritten. No ioctl exposes this helper yet.

Validated metadata retains whole-segment geometry and file extents, so one
TEXT allocation covers both task and constant sections. Scratch and external
surfaces have zero file bytes. The load planner reparses the authenticated
original instead of trusting cached metadata, rounds allocations to 16 KiB
and applies a 512 MiB aggregate host ceiling before any DMA allocation.

The resource geometry helper requires one backend-owned mapping per planned
segment and one binding per external surface, resolved by the backend from
the validated program contract. It checks exact sizes, permissions,
alignment, full-width DVA overflow and overlap against every other mapping
and the resident firmware aperture. Binding VMs, classes and unique indices
must match the validated surface requirements.
Failure publishes no partial resource map. The helper checks geometry;
the backend must separately prove mapping ownership and lifetime.

There is no H16 LOAD_PROGRAM or CREATE_PROCESS body serializer or backend
that resolves resources and installs a process here. CREATE_PROCESS reply
IDs do not supply resource bindings. The backend must resolve them from the
validated program contract and verify mapping ownership. No user ID or
surface ordinal substitutes for that authority, and no load plan installs
an executable process. The original-HWX userspace ABI remains deferred until
the backend can return a program capability backed by validated resources.
Channel-5 allocation replies, boot publication and H17 direct ownership
remain refused.

H16 macOS 27.0 runtime contract
------------------------------

J616s has one ANE interrupt, 1054, used as the shared-memory channel
doorbell/completion line for all firmware-to-host replies and events.
It has no ANE exclave assignment. This identifies the interrupt role, not
its cold-start masking/acknowledgment sequence. H17's two IRQ roles remain
unknown. ANE runtime uses its own channel transport; it does not establish
an A7IOP/RTBuddy boot sequence.

Host command bytes 0..3 are zero, the little-endian u16 ID is at +4 and
the u16 acknowledgment at +6. All observed successful replies had zero
acknowledgment. Nonzero meanings remain unknown and are refused.

==================  ======  =======  ==================
Command             ID      Channel  Bytes
==================  ======  =======  ==================
LOAD_PROGRAM        0x0200  1        0x1c0
UNLOAD_PROGRAM      0x0201  1        0xc
CREATE_PROCESS      0x0202  1        0x258
TERMINATE_PROCESS   0x0203  1        0x10
EXECUTE             0x020c  7        0xca8/0xcc0/0xcd8
POWER_GATE          0x002d  7        0xc
EVENT_REPORT        0x0300  6        0x60 (firmware to host)
==================  ======  =======  ==================

These sizes bound the observed version, not future firmware. POWER_GATE has
a u32 Boolean at +8. The execute-size increment of 0x18 counts custom BAR
records. The channel numbers are observed table indices;
the clean cold-boot facts identify their channel names. The parser retains each
descriptor's original index rather than assuming name order is wire order.

The transport prepares the H16 envelope and rejects unrecognized runtime
IDs, wrong sizes, channels and power-gate values before publication.
H14's 0x0204 procedure packet and short CREATE_PROCESS body remain H14-only;
H16's serializer and UUID completion table are connected to the shared
submission and event paths, subject to validated process resources.
The H16 runtime command table does not authorize startup commands.

H16 execute and event fields
---------------------------

The macOS 27.0 field contract has a fixed 0xca8-byte execute body followed
by zero, one or two observed 0x18-byte custom BAR records. The 64-slot
ordinary buffer array starts at +0x60 with a 0x30-byte stride; the valid
count is at +0x28. A descriptor carries flags at +0, resource index at +4,
numeric class at +8, the fixed value 3 at +0x10, a u64 DVA at +0x18 and a
u64 byte span at +0x20. Other descriptor bytes are zero. The UUID at
execute +0x20 is independent of program/process/procedure IDs at +8/+0xc/+0x10.
The observed priority is 5 at +0x18; request flags and reserved bytes are zero.

The custom BAR count is at +0xca0 and records start at +0xca8. Each carries
u32 configuration/value-type/BAR selectors, u8 bit offset/width, u16 flags
and a u64 value. ``ane_h16_execute_encode()`` accepts kernel-owned resource
metadata, checks nonempty counts up to 64 and overflow-free mapping spans,
and zeroes unused slots and reserved space. The caller must first match
indices, numeric classes, permissions, selectors and values to immutable
validated process metadata and keep mappings pinned. The helper does not
establish that authority. Its two-BAR ceiling, 64-bit field bound and
same-configuration/BAR overlap refusal are conservative draft restrictions,
not established firmware-wide limits. Unknown class and selector semantics
are not guessed. There is no userspace interface for these structures.

Channel 7 acknowledges submission only. Channel 6 carries 0x60-byte 0x0300
events, with UUID at +8, program/process/procedure at +0x10/+0x14/+0x18 and
state at +0x1c. State 0 is start, 3 hardware-start, 1 successful terminal
completion and 2 terminal failure. Failure was established from field use;
it was not present in the runtime sample. The first word selects report
slots or rolls independently; 0xfe00..0xfe03 is not a universal prefix or a
completion key. CPU-load and timestamp fields are diagnostic.

``ane_h16_event_decode()`` validates the channel, exact size, command and
zero u16 acknowledgment before returning fields. The result helper requires
all four identity fields to match an outstanding request. Destination 1 is
routed away from the general scheduler; unknown states/destinations refuse.
A caller must register a unique pending UUID before publication, consume it
atomically once on a terminal report, synchronize DMA, and apply quarantine
rules before considering any mapping release. A classifier result alone
cannot prove safe release or reject duplicate events without pending state.

The shared H16 submission path allocates a non-reused 64-bit UUID, installs
it in a bounded pending table before publishing channel 7, and waits for
both the transport reply and its own terminal event. Events may arrive
before the acknowledgment. States 0 and 3 leave the UUID pending; states
1 and 2 retire it once after matching every identifier. Unknown, duplicate
and late UUIDs cannot retire another request. An identifier mismatch for a pending UUID or a malformed report
quarantines the queue. ISP-destination reports and short
CPU-load notifications cannot complete a general-scheduler request.

One absolute deadline covers acknowledgment and terminal completion; a
report at or beyond that deadline cannot succeed. Failure state 2,
transport refusal, timeout or corruption prevents output copy-back and
quarantines the queue without retry or reset. Firmware-visible process
mappings remain resident, including after file close. Closing a file
revokes its process capabilities before freeing its staging BO handles.

Only private coherent DMA mappings may appear in H16 execute descriptors.
Userspace BOs serve as staging buffers: inputs are snapshotted before
publication and outputs are copied back after validated terminal success.
Write barriers order input and command stores before publication; DMA read
barriers order output reads after completion. Streaming DMA sync APIs must
not be applied to these coherent allocations. There is no userspace alias
to code, scratch or live firmware buffers.

The backend must resolve the resource map from the original HWX and the
reviewed program contract, rather than decode it from CREATE_PROCESS reply
IDs. A userspace index, numeric class or program/process ID cannot
create that map or confer ownership. Only established input/output classes
are admitted; custom BAR requests require a validated selector map and
remain unavailable. The legacy 0x28-byte completion predicate remains
H14-only. Channel-5 allocation/control/free operations require a validated
H16 reply provider and remain unavailable; H14 allocation replies are never
reused for H16.

No original-container load ABI or H16 process installer is supplied
by this change. Consequently no H16 process can yet be installed, inference
remains refused and no accel node is registered. The submission integration
is preparatory and does not open the boot, preload, ownership, firmware
version or resource gates.

KUnit covers byte-exact synthetic execute fields, all 1..64 buffer counts,
zero/one/two custom BAR lengths, wide DVAs/spans, reserved-zero output,
boundary and overflow refusal, all event states, interleaved identities,
ISP routing and malformed reports. No test packet is copied from a capture.

Shared H16/H17 firmware transport
--------------------------------

``ane_fw_transport.h`` supplies device-independent ring operations over owned
coherent surfaces and caller-provided memory, DMA-barrier, MMIO, clock and
allocation accessors. Engine-relative doorbell and pending/acknowledge offsets
come from ``ane_fw_platform`` match data. The caller must establish ownership,
firmware identity, power and the private DMA domain before using it. This
helper does not change probe or enable either generation.

Initialization copies exactly eight 0x100-byte descriptors from the IPC
surface and validates their names, types, bits, slot counts, alignment and
non-overlapping ranges, including exclusion of the table itself. IO and
DATA_CHAIN_H2T require type 0; SHAREDMALLOC and IO_T2H require type 1. Each
operation checks the immutable table snapshot. Only type-0 slots are armed,
in word-1, word-2, word-0 order, with values 0, 0, 1 and a DMA write barrier.
Descriptor geometry and doorbell bits remain authoritative. Logical channel
names are independent of their descriptor order.

Publication copies a command into a 64-byte-aligned arena inside the pool,
after the caller's reserved boot prefix. It publishes both length words,
a DMA write barrier, the command DVA, another barrier and the descriptor's
doorbell bit. There is one outstanding transaction per sending channel;
heads advance modulo the descriptor slot count only after a successful
reply. A reply requires the exact DVA with bit 0 set, unchanged length,
zero result word, zero command prefix, unchanged u16 ID and zero u16 ack.
Bit 1 is corruption even before ownership returns. Nonzero ack is returned
as ``-EREMOTEIO`` and retained for diagnosis.

IO_T2H is present when word 0 has bit 0 clear and either word 0 or word 1 is
nonzero. Reports must be four-byte aligned and wholly inside one owned
surface. The transport copies the report before setting word 0 bit 0 and
ringing the descriptor doorbell, without changing the other slot words.
Report dispatch and pending UUID lookup belong to the caller. In particular,
consuming a stale UUID report cannot complete an unrelated request, and the
DATA_CHAIN reply acknowledges submission rather than inference completion.

The SHAREDMALLOC provider accepts allocation tuples only. Requests are
rounded to 16 KiB, with a 1 MiB request ceiling, the exact 2 MiB allocation
with tag 0x45585050 exception, 12 MiB in rounded allocations and 64 handles.
The allocation callback supplies a zeroed, pinned private-domain mapping;
the transport checks its size, DVA alignment, overflow and exclusion from
all other owned surfaces before publishing it. Handles are nonzero ordinals.
By default, FREE and distinguishable control tuples return ``-EOPNOTSUPP``
through the mandatory refusal logger. The pinned H16 build enables the
proved native FREE reply described below. An all-zero slot is empty and
cannot distinguish a control request; its lookup request layout remains
unproved. No control reply is invented.

IRQ service reads pending once, rejects bits above 7 and writes the exact
nonzero snapshot to acknowledge. Only a SHAREDMALLOC notification authorizes
allocation service: a fresh slot and an allocation request both have a zero
header, so the body becoming visible does not establish publication. After
acknowledging, the transport captures the number of queued entries before
answering any of them. This handles coalesced notifications without admitting
a later request using an earlier notification. The wait helper services one allocation at
a time and checks the original absolute monotonic deadline, including time
spent inside the allocation callback. Servicing never starts a new timeout.

The caller serializes all entry points. Initialize a fresh transport only
before firmware starts or after a proved reset. The pool arena is monotonic
and returns ``-ENOSPC`` when exhausted; command storage is not recycled.
Successful reply bodies remain in the pool transaction span. A timeout,
malformed firmware tuple, allocation failure or firmware refusal latches an
error, retains the transaction and mappings and stops further publication.
No surface is freed by these helpers, including a rejected allocation result.
The backend must retain mappings, power and module references until reset.

The field contract follows the clean H16/H17 cold-boot facts, sections 1,
7 through 11, and the SPEC-r2 channel and deadline rules. Allocation limits
are conservative host policy observed sufficient on H17, rather than
firmware-advertised authority. The KUnit suite uses synthetic memory and
accessor traces, with no MMIO or hardware access. Its configuration fragment
is ``drivers/accel/ane/transport.kunitconfig`` and runs on x86_64 and arm64.

Original HWX validation
-----------------------

The validator consumes one kernel-owned immutable snapshot of the complete
HWX. It requires subtype 7 and bounded, exactly consumed load commands;
rejects unknown command kinds, duplicate descriptors, overlapping segments,
sections and file regions; and validates symbols against the complete
section-ordinal table. Tensor names, directions, normalized fp16 dimensions,
overflow-checked strides and full batched spans must agree with their
surfaces and buffer references. There must be at least one input and one
output, with the single output last.

The linear and scratch program descriptors have exact fixed fields and
zero reserved bytes. All five 16-byte aperture records must equal whole
surface segment bases in order, followed by the optional kernel base;
reserved halves and unused records are zero. Interior, duplicate, reordered
and scratch/text aperture addresses are refused. Task counts and word counts
must agree with the independent stream walk. Relocations use the actual
kernel section ordinal, unique bounded offsets and valid addends, and may
patch only packed-record payload words. Headers, register selectors and
padding remain immutable.

SHA-256 covers the entire original snapshot, including load commands and
padding. The snapshot digest is rechecked before execution. Validation
provides metadata, not allocation authority or permission to load a program;
the original-container ABI and firmware resource map remain required.

The hardware-free KUnit suites cover pending UUID lifecycle, acknowledgment
versus completion, interleaving, progress, failures, malformed and mismatched
reports, routing, duplicates, timeouts, capacity and UUID exhaustion. HWX
tests generate synthetic linear/scratch forms with optional kernel data,
then exercise command, geometry, tensor, aperture, symbol, relocation and
digest refusal paths. No hardware inference result is implied.

H13 timeout and engine recovery
=======================

The host task-manager path (``ane.ko``) gives each submission one second,
including DART fault-drain retries. An expired submission returns
``-ETIMEDOUT``; recovery never silently retries a program whose output may be
partial. The task's GEM lookup references are released when the ioctl returns.

Recovery holds the engine and PM-entry locks, drops the current consumer
runtime-PM reference, synchronously suspends its power-domain suppliers, and
checks both the PMGR ACTUAL fields and every attached genpd for power-off.
The SET window omits the shared SYS_CPU parent, whose DART consumers can
suspend asynchronously; it must also turn off before the consumer resumes.
Recovery then restores the consumer
reference, checks power-on before engine MMIO, and runs the probe task-manager
enable sequence. BO handles and mappings remain valid for resubmission.
The system-sleep force helpers are unsuitable here: a forced, in-use supplier
can retain its active runtime status and keep its parent domains powered.

A power-off acknowledgment is required before reinitializing the engine.
Only after power-on and task-manager readiness are also verified does recovery
release the exceptional module pin and reclaim preserved BOs. If any step
fails, mappings, backing pages and the pin remain held; new submissions fail.
This also covers a partially resumed engine whose retained queue state is
uncertain. The ``reset`` sysfs write retries recovery. Module reload cannot
recover a device while that pin is held. Direct PMGR SET writes and speculative
writes to undocumented task-error registers are not part of this reset path.

The ``ane-pm-recovery`` KUnit suite exercises the shared PM transaction against
real runtime-PM device links, including repeated cycles, single-domain devices,
consumer/supplier suspend errors, a missing power-off acknowledgment and a
failed resume. It checks that every exit restores the caller's PM reference.
The minimal config is
``tools/testing/selftests/drivers/ane/pm.kunitconfig``.

Destructive recovery selftest
----------------------------

``tools/testing/selftests/drivers/ane/recovery.py`` is an explicitly invoked
hardware test, excluded from the default device-free selftests. It requires
exclusive ownership of an M1-family ANE, Python 3, the packaged H13 add fixture,
and a deliberately invalid convolution program. Run it through the hardware
controller with a watchdog and kernel-log capture::

  python3 recovery.py /usr/share/omarchy-ane/fixtures/h13-anec/add/program-0.anec hang.anec

The test keeps both clients open, submits the invalid program three times,
requires a bounded ``ETIMEDOUT`` and ``wedged=0`` each time, and runs twenty
bit-exact add jobs on the surviving client's original fd and mappings after
each reset. Every call changes its inputs and poisons the valid output lanes,
so stale output cannot pass. Its oracle uses exact fp16 quarter values and checks the complete
output, including padding. Closing both clients must return the module
reference count to its initial value; the boot identity must remain unchanged.
An operator should also verify three power-off acknowledgments in the kernel
log and a normal module unload/reload after all clients close. A failed reset
is a test failure, not permission to force unload the module.

Generate ``hang.anec`` on a build host with ``make-hang-fixture.py`` and a clean
checkout of the MIT-licensed public compiler at commit
``ef2dbceefedff38ecce4f3dc593e148c5e7a6d54`` from
https://github.com/aurora-silicon/mil-hwx-compiler/pull/2::

  python3 make-hang-fixture.py /path/to/mil-hwx-compiler hang.anec --cxx clang++

This first checks that the normal encoder refuses the shape. It bypasses only
that refusal in a temporary translation unit to generate a deterministic
stride-2 1x1 128-to-256 convolution at 56-to-28 spatial size, the refused family
reported to hang the M1 Pro. The compiler checkout is unchanged. No Apple
binary or private program capture is used. The generator writes a hash receipt;
the hardware test must fail if this particular instance does not actually
produce a timeout. The loader pins both fixture hashes and uses ABI 1 ioctls directly, so no
shared libane library is needed. ``--check-fixtures`` validates the two containers without
opening a device.

H14 program lifetime
--------------------

Byte-identical sections share one program and one process. Every file holds
a reference on each program it loaded, and ``PROC_CREATE`` and ``EXEC``
refuse a program the file holds no reference on with ``-ENOENT``. A program's
kernel IDs stay valid for as long as a file holds it; whether it is loaded in
the firmware right now (resident) is managed by the driver:

- When the last file holding a program closes, the driver sends
  TERMINATE_PROCESS (0x203) for its process and then DESTROY_PROGRAM (0x201),
  and only after both are acknowledged gives the section buffers back.
- The firmware's L2 spill pool holds eight 2 MiB buffers; every
  CREATE_PROCESS grows it while it holds fewer than eight. At most eight
  programs are resident. Before each LOAD the least recently used program a
  file still holds is unloaded for room; its next ``PROC_CREATE`` or ``EXEC``
  loads it again and creates a new process, with the same IDs. The firmware's
  program and process tables (240 and 496 advertised) are not the limit.
- Only when every resident program is held (a load or unload whose outcome is
  unknown) does the driver restart the firmware at that idle point (see
  below). Programs that files still hold become non-resident and load again
  on use.

A program whose TD stream opens or closes on the wrong task is refused with
``-EINVAL`` before the firmware sees it. Header word 2 of each TD block must
be 0x08 on the first task, 0 between and 0x22 on the last (0x2a for a single
task), as in every decoded Apple H14 program. A program with 0x2a on every
task of several returned from its call on the M2 and then left the firmware
unable to complete any further command until reboot.

A failed load or unload keeps the program and its buffers until a verified
firmware restart.

The firmware asks for memory on the SHAREDMALLOC ring and gives it back with
a FREE request, which carries the returned IOVA in the slot header and an
empty body. It waits for the answer with its shared-memory lock held, and a
FREE caused by a command can arrive after that command's acknowledgment. The
driver serves the ring before each command, while a command or call is
pending, and for 10 ms of quiet after each unload command. A FREE releases the
buffer and its table slot, so the table and its byte count follow the live
allocations rather than every allocation since boot.

``fw_memory`` in the device's accel debugfs directory reports the live
firmware allocations, their bytes, the table high-water mark, the known,
held, resident and cached programs, the resident limit, the restarts for
room and the firmware generation. A command timeout also logs the allocation ring and the
target-to-host rings at the host cursors.

H14 recovery lifecycle
----------------------

The firmware path (``ane_t6021.ko``) gives the command acknowledgment and
finish event one bounded deadline. A failed exchange quarantines submissions
and attempts one reset under the firmware lock. Operator retry uses the same
``wedged`` and ``reset`` attributes as H13.

The supported reset profile is the Linux-owned 13.5 legacy firmware path:
the complete SoC power-domain binding, an owned staged image, and the fenced
boot sequence. Borrowed firmware, RTKit mode, later generations and incomplete
initial boots remain fenced. A second H14 binding is refused because the
firmware cache and BO pool have one owner.

A restart for room (planned) runs with the firmware idle under the firmware
lock. It answers any pending SHAREDMALLOC request and posts POWER_DOWN
(0x0b, the 8-byte header only). That command never completes: the firmware
stops its timers and waits in WFI for good. The driver instead waits for the
dispatcher to clear the completion field it marked, then for the ASC to report
WFI on twenty reads 100 us apart, then clears CPU_CONTROL RUN (bit 4) and
requires CPU_STATUS STOPPED (bit 1) with RUN clear. On T8112 an idle core reads 0x2c
and a halted one 0x28, neither of which is stopped. After a command timeout
the command worker is blocked and nothing can overtake the stuck command, so
only the RUN clear is tried. If
STOPPED is not seen, nothing is powered off: the device stays quarantined
with its mappings, power claim and pin. Only after STOPPED does the restart
suspend the consumer and suppliers through the shared PM transaction. This
prevents firmware power-state writes from racing provider shutdown. It then
verifies every domain and the CPU ACTUAL field off, resumes, repeats the probe
power/PWGATE gates, and requires the CPU to be stopped before changing its
image or boot state. It restores a cached, validated pristine image into the
existing mappings; recovery does not fetch a replacement file or rebuild the
entry aliases. The existing boot helper performs the READY/DONE handshake,
then recovery validates ChMan, writes the host acknowledgment and requires a
valid CONFIG_GET response. An enabled firmware performance property is
restored through its existing command.

Old boot pools, firmware MALLOC buffers and closed firmware-visible BOs keep
their metadata and backing until the new boot is validated. Failed resets
retain those records and any CPU lifetime pin needed by a running or uncertain
CPU. Retired MALLOC arenas remain charged against the existing byte and record
limits; at most two failed-boot arenas can accumulate. A successful restart
reclaims retired arenas and closed BOs. Live old BO handles and mappings remain
client-owned memory and can be reused for loading a program again.

Program and process IDs are opaque kernel identifiers. The allocator survives
firmware restarts and refuses exhaustion instead of wrapping. A restart drops
every firmware wire ID; programs that files hold load again with new wire IDs
on their next use, so an old wire ID never aliases a new one and clients keep
their kernel IDs. The ``ane-fw-ids`` KUnit suite checks non-reuse and
exhaustion boundaries, and ``ane-t6021-lifetime`` checks the task count, the
never-terminate rule, the resident budget, eviction order, reload after a
restart and the restart stop sequence.

H14 keeps its existing domain-link lifetime reference and a reference for the
currently running firmware CPU. Recovery releases the old CPU reference only
after verified quiescence and acquires a new one before the first boot write;
it does not accumulate one reference per failure. This change does not provide
normal H14 module unload while firmware is active. H14 restart still requires
silicon qualification; the H13 hang fixture must not be submitted through ABI 2.

H14 system-sleep adapter
=======================

``ane_t6021_pm.h`` provides the system-sleep PM operations and submission
fencing used by ane_t6021. Stop, firmware reload, channels, allocation ownership
and program replay remain in the shared H14 lifetime implementation from
https://github.com/aurora-silicon/linux/pull/737. System sleep and timeout
recovery use one stop helper, one boot function and one transaction commit.
The shutdown protocol is not implemented a second time by the PM adapter.

The adapter contract is:

* Embed ``struct ane_t6021_pm`` in the client, initialized once at probe with
  its existing firmware mutex, lifetime callback table, client context and
  ``ANE_T6021_PM_ACTIVE`` state.
* Use ``DEFINE_ANE_T6021_SYSTEM_PM_OPS(name, getter)`` to define the device PM
  operations. The getter returns the embedded adapter without replacing the
  driver's own drvdata. The driver binds ``pm_sleep_ptr(&name)`` to
  ``driver.pm`` and supplies the shared lifetime callbacks below.
* Every external firmware-facing submission checks
  ``ane_t6021_pm_submit_error()`` under that same firmware mutex. Suspend acquires the mutex, draining the current
  synchronous command and its finish event, and fences queued submitters
  before calling the shared stop callback.
* The stop callback performs the validated firmware shutdown handshake, drains
  RX and proves ASC STOPPED with RUN clear before returning success. Clearing
  RUN or observing ordinary WFI/idle is insufficient. Failure must retain
  power claims, DMA mappings, firmware backing and the module pin.
* The restart callback is the same cold-start/handshake/program-replay path
  used by timeout recovery. Supplier and DART system callbacks must restore
  access first. Return success only after channels, CONFIG_GET and desired
  firmware properties are restored and the lifetime owner has made public IDs
  usable under its normal lazy-reload policy. RX and all firmware-owned
  resources stay with this one lifetime implementation.

System PM holds an additional consumer reference and disables runtime PM
during the late/noirq phases. Suspend stops at ``ane_t6021_restart_stop()`` and
keeps the shared restart transaction on the client across the system power
boundary. Genpd and
DART system callbacks perform that power transition. Resume checks provider
access and the stopped-core predicate, then invokes the same boot and commit
functions as runtime recovery. Runtime recovery still uses the shared runtime-PM
cycle between stop and boot. No runtime-PM force/cycle helper is called by the
system-sleep callbacks.

Failure leaves submissions fenced with EHOSTDOWN and retains firmware-visible
backing and pins. A verified explicit recovery can reopen the PM gate. The
adapter never sends firmware power commands or duplicates the program/allocation
cache. Internal shutdown/restart commands run through the shared lifetime owner
while external work is fenced.

Device-free tests
-----------------

The ``ane-h14-sleep`` KUnit suite invokes the generated device callbacks with a
fake shared-lifetime backend. It checks three successive cycles, that callbacks
hold the firmware mutex with submissions fenced, failed stop and restart
propagation, unwind before suspend, and suspend/freeze/poweroff versus
resume/thaw/restore callback mapping. Firmware stop predicates, shutdown command
layouts and program replay are tested by the lifetime owner, not reimplemented
here. Run the PM suites without hardware::

  ./tools/testing/kunit/kunit.py run \
    --kunitconfig=tools/testing/selftests/drivers/ane/pm.kunitconfig \
    --jobs=8

M2 validation plan
------------------

Runtime testing requires the shared lifetime implementation and this PM
integration together. Do not infer a hardware pass from the adapter's KUnit
results.
Run on a T8112 with the candidate kernel and its matching ane_t6021 module,
through the hardware controller with exclusive device ownership, bounded
console capture and a tested boot fallback. Record the kernel/module revision,
firmware identity, boot ID, initial kernel log and the smoke command. Use the
same known-good H14 smoke and model/input fixtures for the baseline and every
post-resume check. Verify the complete output, including its expected surface
size, and vary inputs or poison outputs so retained output cannot pass.

1. Run the smoke before suspend. Keep a client with a loaded program, process,
   BO handles and mappings open for the duration of the test.
2. Save the current pm_test setting. Repeat the following three times, capturing
   the new kernel log and running the smoke after *each* cycle::

     echo platform > /sys/power/pm_test
     echo mem > /sys/power/state
     # Run the recorded H14 smoke, checking its exit status and full output.

   Require the shared stop proof and successful restart/handshake/replay receipts
   each time. Exercise the surviving client's original handles with new inputs,
   and a newly opened client, after each cycle. Require an unchanged boot ID,
   correct nonempty outputs, and no LEGACY timeout, failed exchange, DART fault,
   crash or PM failure. Confirm identical repeated cycles do not accumulate
   module references or retained allocations.
3. Set pm_test to ``none``. If ``/sys/power/mem_sleep`` advertises ``s2idle``,
   select it, arrange a supported timed or external wake through the controller,
   and run a real ``echo mem > /sys/power/state`` cycle. Run both smokes after
   wake and apply the same checks. Record unsupported s2idle or wake support
   explicitly; a platform test does not establish real-suspend behavior.
4. Restore pm_test and mem_sleep selections. Preserve failure evidence before
   using the approved known-good boot fallback. Do not force-unload retained
   firmware memory or treat an unproven stop as permission to remove power.

Staged H16 cold boot
-------------------

The boot bring-up runs every stage in order. ``<debugfs>/ane-h16/<device>/stage``
(root only) reports the last completed stage and the run, refusal and
quarantine state. Before the boot bring-up has started, writing
``preflight``, ``map``, ``boot``, ``startup`` or ``program`` runs every
stage that has not run yet, up to the one written, for diagnosis.
A stage at or below the completed one is refused with ``-EALREADY``. There
is no module parameter and no H14 boot parameter changes the H16 path. H17 still refuses ownership and registration.
All build-dependent boot values and ordered masked updates belong to the
H16 27.0 (26A428) match-data build shared by the supported H16 profiles
(27.0 26A428, 27.2 26B123, 26.6.2 25G83, and 26.5.2 25F84). The preload
validator must accept the resident image before a profile can run.
The T8132 (M4) has its own builds: 26.6.2 25G83 and 27.2 26B123. Each firmware
build of an SoC may hand over its own TEXT and DATA sizes (27.2 on the T8132:
0xbc000 and 0x2b0000); the driver picks the build whose sizes and IOVAs the
loader handed over, and each preload rule checks the sizes it was made for. Live mutable
patchbay and firmware tunable values are preserved, with no disk firmware owner.

``preflight`` raises the five real domains through genpd, in DT order, and
checks every requested/ACTUAL low byte twice before reading the engine.
CPU control must be zero and RVBAR must be ``0x10000000001``. It reads all
three DART fault status/address words, version/protect/lock headers and four
DAPF records per instance against ``apple,dapf-ranges``. The ANE backend
writes no engine, DART or DAPF register at this rung. Genpd owns power writes.

``map`` logs all three DART protection words (enforcing bit 0 clear on the
27.2 26B123 profile) before claiming an exclusive, singleton IOMMU group and an unmanaged domain
covering the three SID-0 DART instances. Private surfaces currently require
a DMA-coherent client; noncoherent clients refuse pool allocation before RUN. No
unexported architecture cache helper is called from the module. It checks
the finalized aperture
rather than widening geometry in the client. TEXT is read-only, DATA is
read/write, and the four range-containing 16 KiB pages use identity addresses
and MMIO attributes. Every DAPF lock is checked before any record changes;
a locked mismatch refuses. Unlocked changes preserve unrelated flag bits
and are read back. The backend does not change protect or lock registers.

The DT must describe only SID 0 on these three instances for this backend.
The generic IOMMU attachment API attaches every stream in ``iommus``; it
cannot select a subset from the six-stream production description. Until
the test DT supplies only SID 0, map and later stages explicitly refuse
extra streams. The inherited test DTS currently enables the six-stream
node and therefore needs this integration adjustment before rung 2. The
production node stays disabled. The driver does not rewrite the live DT
or access the DART driver's private stream configuration.

``boot`` allocates the zeroed command pool at ``0x10000384000`` and completes
the H16 CE request/ack protocol, bounded to 5001 delays of 10 us. Ordered
ANEAXI2AF, ANEXLAXI2AF and selected C1PPT updates follow before any scratch
write. A pre-existing C1PPT lock requires matching values. There is no H17
PWGATE sequence. The host clears scratch 0 through 7 in order, prepares the
polled management client, writes scratch 6 = 1 and 7 = 0, rechecks RVBAR,
and writes CPU control 0 then 0x10. RVBAR is never rewritten.

The host polls READY ``0x08042006`` every 1 ms, with at most 1000 polls and
one absolute deadline. Scratch 0 is a channel count below 0x21. IPC bytes
are scratch 1 plus one; heap bytes are scratch 3; scratch 4 is a Boolean.
IPC and heap allocations are rounded to 16 KiB and bounded before use.
The IPC allocation at ``0x100003d0000`` must fit below the fixed heap base
``0x100003e4000``. The heap has a 16 MiB host-policy limit and must also fit
below shared allocations. These limits are not firmware capability claims.

The 0x174-byte init record at pool + 0x180 has a zero first qword, IPC DVA,
0x380000 configured span, the corresponding 0xfc80000 remainder, requested
heap DVA/bytes, unrounded IPC bytes and pool DVA/bytes. Reserved bytes and
the cold-start retained value are zero. Template count is 64 and record
+0x12c is 4. A DMA write barrier precedes publication to scratch 0 low,
scratch 1 high, then scratch 7 ``0xf7fbdff9``. The reply must contain both
READY and the exact IPC base; unchanged init-record addresses time out.
The shared transport validates the table before arming any type-0 ring.
Only then does the host write READY to scratch 3 and wait for zero.

``startup`` sends only PRINT_ENABLE (0x4, 12 bytes), START (0x0, 12),
CONFIG_GET (0x3, 16) and RESOURCE_INFO_GET (0x22, 100) on IO. SHAREDMALLOC
allocation requests are serviced under the shared transport's bounded host
policy while waiting. The pinned H16 profile also acknowledges native FREE
for its own live allocations with a zero tuple. Backing storage and quota
stay pinned until reboot; retired allocations cannot authorize reports or
a second FREE. Foreign frees and control forms without a proved request
layout quarantine without an invented reply. Other firmware profiles retain
the default refusal of FREE/control. Each command keeps one absolute
deadline and requires a zero acknowledgment. The CONFIG_GET tick must be nonzero and is logged;
24 MHz is a comparison value, not a refusal predicate. All 100 resource-info
reply bytes are logged and do not grant allocation authority.

Mailbox service uses 64-bit pop-on-read accesses and bounded, nonblocking
receive/transmit batches. Offered HELLO, EPMAP and power messages use the
shared version-12 parser. Unknown or incompatible messages are logged and
receive no invented response. A missing or stalled RTKit session never
blocks the scratch protocol. IRQ 1054 remains disabled through all stages.

Before RUN, errors unwind mappings and genpd references with a rollback log.
Immediately before the first RUN write the backend pins the module. Every
later failure latches quarantine, dumps scratch 0 through 7 and DART fault
words, and retains mappings, pages and power. Probe stays bound on such an
error so automatic devres cleanup cannot reclaim firmware DMA. A successful
boot also stays pinned because no safe in-place shutdown/reset is proved.
Further attempts are refused until reboot. There is no writable stage knob.

Hardware test plan
------------------

Use a matching kernel/module set and a DT that enables the ANE and its three
DARTs with SID 0 only, the two preload ranges and an aperture covering the
identity pages (``t6040-j616s-ane-test.dts``). Keep shipping DT nodes
disabled. Each write below is one stage; stop and preserve full dmesg at the
first unexpected value::

  echo preflight > /sys/kernel/debug/ane-h16/<device>/stage
  echo map > /sys/kernel/debug/ane-h16/<device>/stage

Before RUN a failed stage rolls back and the module can be reloaded.
Expected preflight messages include ``H16 preload identity H16 27.0 26A428
accepted``, ``H16 power ACTUAL`` twice for each of the five words with low
byte ff, ``H16 CPU control/RVBAR 0 10000000001``, DART status/address and
DAPF comparisons, then ``H16 stage preflight complete``. Map adds TEXT/DATA
mapping messages and four ``H16 identity MMIO`` messages, clean fault words
and ``H16 stage map complete``. A six-stream tree instead logs ``H16 map
requires only SID 0 on three DARTs; refusing extra streams``.

The boot rung releases the firmware CPU::

  echo boot > /sys/kernel/debug/ane-h16/<device>/stage

Expect CE state/ack and ``H16 tunables applied`` (or locked-match) logs,
then ``H16 READY 8042006 0``, request logs (the comparator values are
channels 8, IPC-minus-one c440, heap f00000, mode 0), then
``H16 table validated 100003d0000 8``, ``H16 scratch 3 cleared 0 0`` and
``H16 stage boot complete; held until reboot``. Comparator sizing values are
not additional admission checks. From here the module stays loaded.

In the same boot::

  echo startup > /sys/kernel/debug/ane-h16/<device>/stage

Expect SHAREDMALLOC allocation messages as offered, ``H16 startup ack`` for
IDs 4, 0, 3 and 22 with zero status, ``H16 CONFIG_GET tick`` with a nonzero
value, exactly 100 bytes under ``H16 RESOURCE_INFO`` and ``H16 stage startup
complete; held until reboot``. A failure after RUN must instead show the
scratch/fault dump and ``H16 quarantine``. Do not retry; collect logs and
reboot. No inference result is claimed by these diagnostic rungs.

The ``ane-h16-boot`` KUnit suite uses a simulated engine, scratch cells,
mailbox polling and rings. It covers startup including allocation service,
missing READY, wrong/stale table publication, oversized IPC/heap requests,
channel bounds, locked DAPF differences, nonzero acks, scratch-3 timeout,
zero tick, default refusal and quarantine after RUN. It also covers DART
protection refusal, matching locked DAPF, heap/shared overlap, native FREE,
foreign FREE and control refusal.

H16 program stage
-----------------

The ``program`` stage runs every startup step, then registers an accel node
with ABI version 3 (``DRM_ANE_CHIP_H16``). GET_CAPS, H16_LOAD and H16_EXEC
are its only ioctls. The node is absent at every other stage, so the default
refusal and the H17 refusal are unchanged.

H16_LOAD takes the original HWX and four host-built sections: generic,
operation, procedure and text properties. The kernel imports the HWX once,
validates it with the HWX and task-stream checks, and plans whole-segment
mappings. Each region and each host-built section gets its own private
mapping in a program window above the SHAREDMALLOC range. Only the output
surface and scratch are device-writable. Userspace never supplies a DVA,
firmware ID, resource index or command. The LOAD_PROGRAM slots come from one
table:

====  ==========================  ========  =====
Slot  Source                      Index     Class
====  ==========================  ========  =====
0     generic (host-built)        1002      4
1     kernel section, else const  1007      3
2     HWX task stream             1006      4
3     operation (host-built)      1003      4
4     procedure (host-built)      1004      4
5     empty
6     text properties             1005      4
7, 8  empty
====  ==========================  ========  =====

Every populated slot has +0x10 = 1 and a full 64-bit DVA and size. The
request carries 0xffffffff at +0x1b8. The reply must echo every slot, keep a
zero acknowledgment and replace that word with the program ID. Zero is a
valid ID. CREATE_PROCESS (0x258 bytes) sends the program ID with a zero body.
Its reply must keep the body zero and returns the process ID, where zero is
also valid. A program with a kernel section must tag its operation record
with the kernel slot index 1007. The uapi header names these indices.

Slot 1 carries the ``__KERN_0`` kernel section when the program has one and
otherwise the ``__TEXT,__const`` constants, which macOS loads in every
program.

EXECUTE resources come from a kernel-owned map. The class follows the
resource usage mapping (inputs 0, the output 1). Each descriptor names its
buffer by the ID of the generic section that LOAD installs: generic entry
k describes surface k in HWX surface order, and the kernel refuses a generic
section whose count, direction, type or size disagrees with the program,
that repeats an ID or that uses the reserved IDs 2 and 3. The mcache tuple in the linear program
descriptor (BAR ID, bit offset and width) adds one custom-BAR record with
selector 1, value type 0 and the first pool DSID, 36. The scratch form sends
none.

Each H16_EXEC copies inputs into the private input mappings and poisons the
output mapping. It then sends 0x002d with 0, keeps the request UUID pending
before publishing EXECUTE on channel 7, and treats the channel-7 reply as
transport acknowledgment only. Completion is a state-1 0x0300 report whose
UUID, program, process and procedure IDs all match. Then 0x002d with 1 and
a DMA read barrier precede the output copy. A state-2 report, a mismatched
ID, a malformed report or a timeout quarantines the session. Reports with
an unknown UUID never complete a request. Closing the file sends TERMINATE
then UNLOAD. Mappings stay until reboot.

One program lifetime per firmware boot is mandatory. On this firmware
family the first EXECUTE of a newly loaded program can write into the
previous program's output buffer when the buffer counts match, and a larger
output overruns that mapping. A second H16_LOAD therefore fails with -EBUSY
and the log line ``H16 one program per firmware boot; reboot required``,
also after the first program closed. The transport never publishes the
secure-mode commands 0x27, 0x28 and 0x34 or the exclave commands 0x30 and
0x31.

The ``ane-h16-program`` KUnit suite covers add-128 and synthetic slot
packing, LOAD/CREATE/TERMINATE/UNLOAD bodies and replies, the deny list,
the custom-BAR record, a fake-firmware session with two executions and
teardown, the second-LOAD refusal, bad LOAD/CREATE replies, mismatched
UUIDs and IDs, and state-2 quarantine.
``tools/testing/selftests/drivers/ane/h16-run.py`` loads the pinned add-128
HWX, runs it twice with poisoned output and compares with ``expected.bin``.
=======
For the experimental T8140 cold AP-direct stages and their ownership and
loader requirements, see :doc:`apple-ane-cold`.
