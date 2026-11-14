.. SPDX-License-Identifier: GPL-2.0-only

J314s secondary CPU context-loss diagnostic
=========================================

This is a lab experiment, not a suspend or idle implementation. It can leave a
CPU stopped and the kernel blocked until an external reset. It must never be
enabled in a distributed kernel configuration or invoked on the controller Mac.

Build APPLE_CPU_CONTEXT_LAB=y and use apple_cpu_context=1 only with the matching
AURA v0.1.0-smp-79d1c20c loader on the J314s M1 Pro. The diagnostic creates
root-only debugfs trigger and result files. Probe does not execute the test.
The trigger accepts exactly ``cpu1-context-loss`` followed by a newline, once
per boot, after the external harness has verified the immutable kernel, loader,
device tree, boot identity, no active VMs, serial identity and reset recovery.

All ten online CPUs enter stop_machine callbacks with interrupts disabled.
CPU0 remains active in CPU1's E cluster; the other eight CPUs cannot initiate
waited IPIs or TLB shootdowns. CPU1 enters the CPU-PM notifier path, then generic
cpu_suspend. This stopped-machine test preserves the local timer registers
instead of entering timer broadcast: no CPU can service broadcast events while
all callbacks mask IRQ/FIQ, and CPU0 explicitly restarts CPU1. CPU-local hrtimer
broadcast can otherwise veto its own owner before the test reaches poweroff.
Expired local events are handled after the callbacks release the CPUs.
A CPU-PM veto has already been unwound by
CPU_PM_ENTER_FAILED and does not receive an unmatched CPU_PM_EXIT.

The result's target_step distinguishes local-state capture (1), CPU-PM entry
(2), CPU-PM admission (3), finisher entry (4), committed stop (5), cpu_suspend
return (6) and completed restore checks (7). cpu_pm_result records the notifier
return separately. These fields identify admission failures without reporting
an unexecuted power-off or saved-context return as a hardware failure.

The diagnostic additionally writes short ``^x`` lines directly to the existing
J314s serial0 console. Its live DT address and Apple UART compatibility must
match, FIFO and TX must already be configured, and each character polls at most
8192 times. It never changes UART configuration. MMIO32 and a non-posted device
mapping match the Apple early console. Capture the UART externally: a reset
can lose the kernel result and even recently written filesystem receipts.
Missing markers do not prove a stage was not reached because a full FIFO can
drop a bounded marker.

``G`` precedes stop_machine; ``O`` is the observer's GO; ``a`` is local-state
capture; ``b``/``c`` bracket successful CPU-PM entry; ``d`` is the finisher;
``e`` verifies the published resume address; ``f`` commits the stop; ``l``
follows the final L1 sweep; ``w`` follows the stop write before WFI; ``F`` is
observed OFF; ``R``/``S`` bracket the startup writes; ``r`` is the physical
resume trampoline; ``u`` follows cpu_suspend return; ``C`` completes restoration
checks; and ``D`` is the observer's completed return wait. The assembly markers
use scratch registers and MMIO only, preserving the no-stack/no-C contract
after the final L1 sweep. The physical entry marker itself is experimental
instrumentation; do not infer a successful cpu_resume from seeing it alone.

CPU_IMPL status accesses use the same 64-bit width as AURA's smp_stop_cpu.
Observer markers ``B``/``V=<hex>`` bracket the initial status read, and
``P``/``T=<hex>`` bracket the first read after stop commitment. An OFF timeout
prints the final value as ``X=<hex>`` and elapsed counter ticks as ``E=<hex>``,
then ``A`` at observer abort. The status loop also has a finite poll count if
its counter stops advancing. A hardware MMIO transaction can itself stall;
the external controller remains the actual recovery bound. Concurrent markers
from the two cores can interleave, so retain the raw stream as well as parsed
labels and never treat a missing or malformed label as proof of absence.

The finisher publishes an idmap trampoline through CPU1's existing AURA release
field. It saves the generic suspend context and local state, programs only the
local timer/CYC registers, sweeps only local L1, requests one CPU stop and waits
without returning. CPU0 requires a nonzero status before the attempt and an
observed OFF transition within 50 ms before issuing the two CPU-start writes.
There is no RVBAR, ACC_OVRD, DCP, PMP, shared-L2 or DRAM power write.

AURA's existing per-MPIDR reset path calls the physical trampoline, which enters
cpu_resume. Only the generic saved-context return counts as a successful
cpu_suspend. The delivery field, CYC and timer snapshots must be restored before
the other CPUs are released. The observer can cancel an uncommitted finisher,
but leaves the delivery field to CPU1 once a stop is committed and never retries
a start. Observer errors do not overwrite CPU1's result. Fast-IPI pending state
is acknowledged after a real saved-context return; a pre-existing pending IPI
aborts before poweroff and is left for the normal interrupt handler.
An unrecovered CPU prevents stop_machine
from completing: that is an expected terminal failure requiring the separately
reviewed external reset, not a bounded software recovery or successful return.

M1 EHID4 disables DC maintenance by VA and set/way operations on L2. The final
leaf uses L1 set/way only, with shared L2 kept active by CPU0. The explicit PoC
requests for the context, stash and release field do not establish hardware
visibility by themselves. Whether this return contract works on the M1 is what
the single reviewed hardware experiment must determine.

A successful result proves one secondary's context-loss return only. Whole-system
entry/wake, boot-CPU handoff, device restoration, repeated cycles and measured
sleep power remain separate acceptance requirements.
