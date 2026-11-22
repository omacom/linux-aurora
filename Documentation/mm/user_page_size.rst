.. SPDX-License-Identifier: GPL-2.0-only

==============================
Per-process user mapping sizes
==============================

The experimental mixed-page implementation separates the user mapping
size of an address space from the kernel's native allocation size.
``PAGE_SIZE`` still describes native physical pages. ``mm_page_size(mm)``
describes the minimum user mapping and protection unit of that address
space. A process retains one mapping size for the lifetime of its ``mm``.
Exec may create a new ``mm`` with a different size; changing a launch
preference never changes an existing mapping.

The generic interfaces described here are internal development interfaces.
The experimental launch commands still use private prctl numbers; these
are not assigned upstream UAPI. The arm64 backend's supported modes and
current restrictions are documented in
:doc:`../arch/arm64/user4k`.

Exec policy and inheritance
===========================

``ARCH_HAS_USER_PAGE_SIZE`` enables two per-thread policy fields in
``task_struct``:

``exec_page_shift``
    A one-shot request for the next exec. Zero means no one-shot request.

``default_exec_page_shift``
    An inherited preference used when no one-shot request overrides it.
    Zero means the architecture's ordinary executable policy.

Fork and thread creation copy both fields. Subsequent changes affect only
the calling thread, including when it shares an ``mm`` with other threads.
An exec by a non-leader uses the execing thread's policy. The fields do not
belong in ``mm_struct``: threads sharing one address space may launch new
images with different preferences.

Generic prctl handling validates reserved arguments before calling the
architecture validator. A rejected request leaves both fields unchanged.
Setting a native one-shot request while an inherited alternative is active
stores ``PAGE_SHIFT`` explicitly, so the next exec can override that
alternative. Clearing the inherited preference does not consume a pending
one-shot request.

Exec stages arguments in a private ``mm``. Its initial allocation receives
the one-shot shift, if any; the final executable loader resolves the
inherited preference and any executable-specific policy. An ELF interpreter
must be loaded using the same final mapping size as its executable.
Failure before committing the new image retains both preferences.
``setup_new_exec()`` consumes the one-shot field after the new image is
committed, while retaining the inherited preference.

Architecture responsibilities
=============================

An architecture selecting ``ARCH_HAS_USER_PAGE_SIZE`` must implement:

* ``arch_exec_page_size_shift(size)``: return zero for a native request, a
  validated alternative shift, or a negative errno. Validation includes
  the supported hardware, the kernel configuration and any boot-time
  switch, not just whether the requested byte size is a power of two.
* ``arch_mm_init_exec(mm, shift)``: initialize the private address space's
  geometry before allocating its page-table root. Zero denotes native
  geometry. The hook receives the request explicitly and does not read
  a separate architecture thread-policy field.
* ``arch_mm_alt_granule_enabled()``, optionally: report whether this boot
  may create alternative-granule address spaces. It must be fixed before
  memory management initialization. When it is false the ``MM_SUBPAGE``
  page_ext record is not reserved. arm64 returns true unless booted
  with ``user4k=off``.
* Binary-loader integration that resolves the final executable policy
  before exposing the address space. ``bprm_set_page_shift()`` transfers
  staged argument bytes when the final choice changes the initial granule.
* Page-table allocation, walking, invalidation, context switching and MM
  operations for every size the architecture exposes.

The architecture retains control over executable compatibility policy.
For example, the arm64 AArch32 loader prefers 4K where the backend supports
it and retains the native ABI when that capability is absent. This is
separate from running an AArch32 kernel: ``arch/arm`` needs its own backend
integration before it can advertise alternative mapping sizes. The generic
exec policy alone does not provide that backend or expose new sizes there.

Architectures without this capability retain native mappings and reject
the experimental launch-policy commands. ``ARCH_HAS_MM_PGTABLE`` describes
per-mm page-table accessors; it does not by itself promise an executable
userspace ABI.
