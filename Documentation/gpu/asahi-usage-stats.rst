.. SPDX-License-Identifier: GPL-2.0-only OR MIT

Asahi client engine time
========================

Asahi reports the following optional keys in each DRM file's fdinfo, following
:doc:`drm-usage-stats`::

    drm-engine-vertex:   123456 ns
    drm-engine-fragment: 234567 ns
    drm-engine-compute:  345678 ns

``vertex`` is the tiling/vertex stage of render commands, ``fragment`` is their
fragment stage, and ``compute`` is compute dispatch. These are separate timelines:
vertex and fragment can overlap. There is no separately submitted blit engine in
the driver's command ABI. Copies executed by compute or render are included in
the engine that executes them.

The counters belong to the open DRM file, rather than to a PID or a queue handle.
All queues and VMs of that file contribute to the same ledger. Duplicated or
inherited file descriptors share counters and must be deduplicated using
``drm-client-id``. Closing a queue or VM does not reset its file's counters, and
completions retain the ledger until their firmware work retires. A new DRM file
starts at zero, even when another file in the same process has submitted work.

Time source and limits
----------------------

Only completed stage intervals are reported. A long-running command contributes
after retirement, so polling during execution can temporarily show no change.
Counters accumulate monotonically, saturating at the largest unsigned 64-bit
value. Conversion rounds down by less than one nanosecond per retired stage.

G13 and G14 use the firmware's internal start/end timestamp cells. The existing
timestamp microsequence operations run even when the application requests no
timestamp output. Internal buffers were already allocated for each command;
fdinfo never reads an application's timestamp buffer.

The G15 and native G16 backends reuse the internal timestamp cells already read
by their successful retirement statistics path. The shared G17 execution backend,
including T6040/G16S and T8140/G17P, charges the timestamp pairs delivered by its
once-only completion path. These backends use the firmware's 24 MHz command timer;
the classic backend uses its hardware configuration's timer frequency.

These are firmware stage envelopes, not hardware shader occupancy measurements.
They can include firmware scheduling, preemption and stalls between the stage's
start and end operations. Intervals are summed without attempting to merge
overlapping commands on the same engine. Missing, zero-start or reversed intervals
are ignored. Failed or cancelled commands do not contribute incomplete intervals;
successful stages that retired before a later failure remain counted. An
unexecuted G17 replay contributes only when it eventually retires successfully.
There is no submit-to-completion wall-time fallback in the current backends.

``drm-cycles-*`` and ``drm-maxfreq-*`` are intentionally absent. The timestamp
clock is independent of the variable GPU execution clock. Treating command timer
ticks as GPU busy cycles, or multiplying time by a sampled/maximum GPU frequency,
would give incorrect frequency-weighted utilization. Per-client execution-cycle
counters would require a separate hardware measurement source.

Accounting cost
---------------

Each file allocates one three-counter ledger. Queues and pending completions
retain references to it. Retirement adds one relaxed, saturating atomic update
per valid stage, with no new locks or allocations. The native backends reuse
existing timestamp reads. Classic completion callbacks read two internal cells
per stage; commands without application timestamps also execute two existing
timestamp microsequence operations per stage. Reading fdinfo loads three counters
and does not traverse queues, wait for fences or access hardware.

Host arithmetic tests
---------------------

The ledger uses only ``core`` and can be tested directly from a source checkout::

    rustc --edition 2021 --test drivers/gpu/drm/asahi/usage.rs -o /tmp/asahi-usage-test
    /tmp/asahi-usage-test

Tests cover conversion, missing/reset timestamps, saturation, overlapping stages,
client isolation, out-of-order completions and simultaneous queues sharing a file.
