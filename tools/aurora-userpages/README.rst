.. SPDX-License-Identifier: GPL-2.0-only

Per-process userspace page-size launcher
=======================================

Build with ``make`` in this directory. The running arm64 kernel must provide
Aurora's experimental per-process granule API. The feature is on by default;
a kernel booted with ``user4k=off`` refuses every non-native size with
``EINVAL``, and ``native`` still works.

Examples::

    userpages 4k FEXInterpreter ./x86-program
    userpages 4k python3 script.py
    userpages native program
    userpages --status

The choice becomes the calling thread's inherited exec preference: it applies
to the launched program and subsequent execs in its descendants. Fork retains
the address space and both exec preferences. Other threads and processes are
unchanged. ``native`` clears the inherited alternative preference. Ordinary
processes that never opt in continue to use the kernel's native default.

``--once`` changes only the next successful exec; an existing inherited
preference remains available for later execs. For example,
``userpages --once native program`` runs one program with native pages and
retains any inherited 4K preference for its subsequent execs. Failed exec does
not consume the pending request. No selection changes a live address space.
AArch32 programs always use 4K when supported, otherwise native pages,
regardless of either preference.

The kernel checks supported sizes and CPU capabilities. The current branch
supports 4K userspace on 16K/64K kernels and 16K userspace on 64K kernels.
64K userspace on a 16K kernel is not enabled. This launcher uses the private
PR_AURORA_{SET,GET}_{EXEC,DEFAULT}_PAGE_SIZE API in this tree;
it is not an upstream Linux ABI.
