.. SPDX-License-Identifier: GPL-2.0-only

============================================
Experimental per-process page sizes on arm64
============================================

The mixed-page implementation lets an address space use a different user
translation granule from the kernel's native physical page size. The kernel
page size remains fixed for the boot. An address space's user granule remains
fixed for its lifetime; exec can create a new address space with another
supported granule. See :doc:`../../mm/user_page_size` for the generic launch
policy and architecture integration contract.

This is an experimental feature and ABI. It is built into the kernel and
on unless the kernel is booted with ``user4k=off``. The launch
commands use private prctl numbers, not assigned upstream UAPI. The
implementation and tests do not establish complete application, device or
hardware qualification.

Configuration and supported combinations
========================================

``CONFIG_ARM64_USER4K_EXPERIMENTAL`` requires 48-bit VA and PA and a native
16K or 64K page size. The option is on by default and selects the generic
``MM_SUBPAGE`` backing and ``ARCH_HAS_USER_PAGE_SIZE`` exec policy support.
LPA2 and ``NO_PAGE_MAPCOUNT`` configurations are excluded by Kconfig.

The exposed combinations are:

=================== ================== ============================
Native kernel size  AArch64 default    AArch64 alternatives
=================== ================== ============================
4K                  4K                 None in this implementation
16K                 16K                4K, when supported by the CPUs
64K                 64K                4K and 16K, subject to support
=================== ================== ============================

Capability detection uses the sanitized system-wide granule intersection,
and does not run with ``user4k=off``. AArch32 executables select 4K when that
capability is available and retain the native granule otherwise, regardless
of launch preferences. The latter case still enforces native
mapping/protection alignment. AArch32 mmap2 file offsets retain their 4K
ABI unit in either case.

An AArch32 executable running under an arm64 kernel is distinct from an
``arch/arm`` kernel. The latter retains its existing native ABI; this series
does not yet provide its alternative-granule backend. Exploiting extra
translation capabilities available only within an asymmetric AArch32 CPU
domain also remains unimplemented.

Larger-than-native process granules are not exposed. Internal 64K-on-16K
fixtures exercise allocation, mappings and argument staging, but those
fixtures do not enable a public 64K process ABI on a 16K kernel.

Turning it off at boot
======================

Building the option in enables it. Add ``user4k=off`` to the kernel
command line to turn it off for that boot; ``user4k=on`` or no parameter
leaves it on. The value is parsed with ``kstrtobool()``, so ``1``/``0``
and ``y``/``n`` also work, and it cannot be changed after boot.

While it is off:

* ``PR_AURORA_SET_EXEC_PAGE_SIZE`` and ``PR_AURORA_SET_DEFAULT_PAGE_SIZE``
  fail with ``EINVAL`` for every non-native size; a native or zero size
  still succeeds. The GET commands report the native default (zero).
* Every exec, including AArch32 executables, gets the native granule.
* The CPUs report no alternative user granule: the boot log has no
  ``4K userspace translation granule`` capability line.
* The 4K ownership record is not reserved in ``page_ext``. With it on,
  the record takes 32 bytes for every native page of RAM (about 0.2% with
  16K pages). ``page_ext`` itself is still allocated if another user, such
  as page_owner, needs it.

The shared MM code paths that serve every process are compiled in either
way and do not change with the parameter.

While it is on, the CPU capability line appears on CPUs that support
the 4K translation granule, and the launcher below can select 4K pages.

Selecting an executable's page size
===================================

The kernel must not be booted with ``user4k=off``. Build the development
launcher with::

    make -C tools/aurora-userpages

For example::

    tools/aurora-userpages/userpages 4k COMMAND [ARGUMENTS...]
    tools/aurora-userpages/userpages --once native COMMAND [ARGUMENTS...]
    tools/aurora-userpages/userpages --status

The ordinary selection establishes an inherited per-thread exec preference.
``--once`` overrides it for one successful exec. A failed exec retains the
pending request; changing a preference never changes existing mappings.
Fork copies the preferences and mappings. Threads sharing an mm have the
same current granule but may establish different preferences for later exec.
``native`` clears an inherited alternative preference; a one-shot native
request can override that preference without clearing it.

The final ELF loader resolves the granule before committing the new image.
If arguments were staged with another granule, their bytes are transferred
into a new private mm. Temporary argument mappings are bounded by both the
user leaf and native page size. Logical argument-size limits continue to
use the process granule. The interpreter and executable share one mm and
therefore one final granule. ELF segment alignment alone does not opt a
program into an alternative ABI.

``AT_PAGESZ`` reports the process granule. User virtual address alignment,
protection boundaries and mapping lengths use that granule. Native physical
allocation, PFN arithmetic and file-cache indices continue to use the kernel
page size. Application execution does not require any of the KUnit options
or the ``arm64.user4k_test=1`` diagnostic boot parameter.

Memory and table ownership
==========================

Keep the following units separate:

* ``PAGE_SIZE`` and physical PFNs describe native physical pages.
* ``mm_page_size(mm)`` describes the user mapping/protection unit.
* A folio owns one or more native pages and carries their physical charge.

Smaller anonymous mappings retain explicit subpage ownership. A reference to
the containing folio alone does not retain an individual reusable slot.
The slot's byte offset must survive fault, COW, reverse mapping, migration,
swap and pinning operations. Newly exposed bytes are zeroed, and changing
one mapping's protection must not widen that change to adjacent slots.

File offsets use a native cache index plus an explicit byte offset. VMA
split/merge, truncation and reverse mapping must preserve both components.
Physical allocation charges count the actual backing once, independently
of how many user mappings expose it.

Lower page tables can share a native allocation. Table tokens preserve the
fragment address, while locking and lifetime use the native table owner.
Unissued cached fragments retain owner references. Issued fragments are not
reused within a live owner. Deferred reclamation must preserve the fragment
token through TLB invalidation and the required RCU grace period, including
allocation-failure fallback paths.

Translation context and invalidation
====================================

The arm64 backend supplies per-mm page-table geometry. TTBR1 retains native
kernel tables, while TTBR0 uses the selected user geometry. A granule change
installs and synchronizes a reserved TTBR0 before changing TG0, then
synchronizes the new geometry before installing the user root. Software PAN
also selects the user TG0 even when restoration of TTBR0 is deferred until
uaccess or return to userspace.

The existing mm ASID is retained. Selecting another granule does not add
an unconditional context-switch TLB flush. User invalidation operands and
range strides use the target mm's geometry; kernel and stage-2 operations
retain their own native geometry. Transition/idmap paths restore native
TG0 and then the active mm's user geometry when returning.

The current configuration retains the existing 48-bit user address size.
Independent TG0/TG1 fields do not make all other translation controls
independent; the LPA2 exclusion must not be removed merely by changing TG0.

Sharing, pinning and interface limits
=====================================

Mixed-granule sharing is permitted. Shared mappings refer to one coherent
backing object, with permissions belonging to each virtual mapping. Shared
futex identity includes the object and byte offset. Private copying is not
an implementation of shared-memory semantics.

Some interfaces have additional layout or ownership requirements. For
example, a perf ring's first mapper fixes its userspace granule; a mapper
using another granule is rejected. This does not prohibit mixed-granule
memfd or System V shared-memory aliases.

A legacy page-array pin cannot express every smaller-granule byte range.
Converted consumers use typed fragment references carrying backing, offset,
length and ownership. Unconverted paths must not silently round a pin or DMA
mapping into neighboring slots. Mock IOMMU/VFIO tests validate the recorded
translations and ownership paths, not physical-device DMA on arbitrary
hardware.

Alternative-granule THP/KSM and hugetlb support, remaining legacy pin
consumers, larger-than-native process activation and broader device
integration still need work. Userfaultfd feature bits and per-range ioctl
masks must be consulted; success for a shmem operation does not imply
support for the corresponding hugetlb operation. Sparse individual fills
can retain an entire native folio; packing behavior is not a throughput
or memory-efficiency guarantee for every workload.

Test coverage
=============

The arm64 selftests are in ``tools/testing/selftests/arm64/user4k``. The
fixtures that select another page size need a guest booted without
``user4k=off``. The ``page-contract`` executable checks the exposed
userspace contract. Extended
fixtures cover exec policy, AArch32, shared mappings, accounting, reclaim,
I/O and device-facing interfaces. Several are disposable-guest PID1 programs
with companion executables or storage requirements; they are not ordinary
host test commands.

``CONFIG_ARM64_USER4K_KUNIT_TEST`` and the data/TLB suites exercise internal
geometry and ownership. Boot them without ``user4k=off``. The
``arm64-user4k-policy`` suite also runs with ``user4k=off`` (for example
``kunit.py run --kernel_args user4k=off``) and checks that the feature stays
closed while off.
Some fixtures additionally require ``arm64.user4k_test=1`` and
some hardware translation tests are disabled under KASAN. Check individual
KTAP results and skips rather than treating boot success as a suite pass.
Private IOMMUFD/VFIO test-device options are separate and should remain off
in application kernels.

The NUMA COW batching case runs only at boot, before userspace can change
its policy prerequisite. Its results remain visible in KUnit debugfs, but
it has no runtime rerun control. Other suites retain their rerun controls.

Validation must distinguish native/default operation, each exposed
alternative, and AArch32 fallback with 4K support unavailable. Native ARM32
short-descriptor and LPAE controls check that generic changes preserve the
existing ABI; they do not validate a mixed-page ARM32 backend. Internal
larger-leaf tests likewise do not qualify a public process mode.
