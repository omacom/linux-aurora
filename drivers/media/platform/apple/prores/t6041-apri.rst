.. SPDX-License-Identifier: GPL-2.0-only

===============================
T6041 ProRes initialization data
===============================

The driver can carry ProRes initialization rows built in, and on T6041 a
machine may override them with ``apple/prores-t6041-init.bin``, an APRI
container generated for that Mac from its own macOS and not distributed with
Linux.  Without the file, T6041 uses its built-in rows: the 82, 80 and 62
values macOS writes to the three windows before its first job.  T6000 and
T8140 use their built-in rows and do not request this file.

When the file is present, the driver reads and validates the whole of it
before mapping device registers.  An invalid file makes the device fail to
probe without partially initializing the encoder; it never falls back to the
built-in rows.

APRI version 1
==============

APRI is a little-endian, length-delimited format.  Every integer is an
unsigned 32-bit little-endian value::

    byte[4] magic = "APRI"
    u32 version = 1
    u32 soc = 0x6041
    u32 block_count = 3

    repeated block_count times:
        u32 block_id
        u32 entry_count
        repeated entry_count times:
            u32 register_offset
            u32 mask
            u32 value

The blocks are fixed and ordered: SCODEC-system (ID 1, 82 entries), ProRes
core (ID 2, 80 entries), and SCODEC AXI-to-fabric wrapper (ID 3, 62 entries).
The only valid version 1 size is 2,728 bytes; padding and trailing data are
rejected.

Offsets must be unique within a block and four-byte aligned.  System and core
offsets are below ``0x4000``; wrapper offsets are below ``0x1000000``.  Masks
must be nonzero, and values may not set bits outside their masks.  For each
entry the driver performs::

    new = (old & ~mask) | (value & mask)

After putting the core in MODE-off, the driver applies every row in container
order.  It then installs the descriptor ring and statistics buffers and
continues with the common T6000/T8140 initialization sequence.

The container deliberately has no MMIO addresses, selectors, power policy,
or arbitrary write commands.  File integrity is the responsibility of the
package that installs the per-Mac file.

T8132
=====

``apple/prores-t8132-init.bin`` is the T6041 container with ``soc = 0x8132``
and a wrapper block of 48 entries instead of 62: 2,560 bytes in all. macOS
brings up the T8132 engine with read-modify-write tables of 82 (SCODEC
system), 80 (core) and 48 (AXI wrapper) rows, below ``+0x4040`` in the
wrapper window. The driver has no built-in T8132 rows, so that engine probes
only with the file.

T6050
=====

``apple/prores-t6050-init.bin`` is the same container with ``soc = 0x6050``,
``block_count = 4`` and a fourth block, SCODEC control (ID 4, 12 entries,
offsets below ``0x4000``): 2,880 bytes in all.  Without the file, T6050 uses
its built-in rows: the 82, 12, 80 and 62 values macOS writes to the system,
control, core and wrapper windows before its first job.  The control rows are
applied after the system rows and before the core rows.

T8142
=====

``apple/prores-t8142-init.bin`` has T6050's four blocks with ``soc = 0x8142``
and a 38-entry wrapper block: 2,592 bytes in all.  Like T6050, T8142 has
built-in rows, the 82 system, 12 control, 80 core and 38 wrapper values macOS
27.2.1 writes before its first job on a J815, so the file is optional and,
when present, replaces them.
