/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2012 ARM Ltd.
 */
#ifndef __ASM_MMU_H
#define __ASM_MMU_H

#include <asm/cputype.h>

#define MMCF_AARCH32	0x1	/* mm context flag for AArch32 executables */
#define MMCF_USER_4K	0x2	/* experimental, immutable mm table geometry */
#define MMCF_USER_16K	0x4	/* experimental, immutable mm table geometry */
#define MMCF_USER_64K	0x8	/* internal geometry; larger user ABI not enabled */
#define MMCF_USER_GRANULE (MMCF_USER_4K | MMCF_USER_16K | MMCF_USER_64K)
#define USER_ASID_BIT	48
#define USER_ASID_FLAG	(UL(1) << USER_ASID_BIT)

#ifndef __ASSEMBLER__

#include <linux/errno.h>
#include <linux/refcount.h>
#include <linux/sizes.h>
#include <asm/cpufeature.h>

#ifdef CONFIG_ARM64_USER4K_EXPERIMENTAL
extern bool arm64_user4k_on;

/*
 * Mixed-granule userspace is on unless the kernel is booted with user4k=off.
 * Fixed once early parameters are parsed.
 */
static inline bool user4k_enabled(void)
{
	return arm64_user4k_on;
}
#define arch_mm_alt_granule_enabled user4k_enabled
#else
static inline bool user4k_enabled(void)
{
	return false;
}
#endif

/*
 * Validate a page-size request for a later exec. Zero selects the native
 * granule; an alternative needs user4k enabled and CPU support. Pure for KUnit.
 */
static inline int arm64_exec_page_size_request(unsigned long size, bool enabled,
					       bool has_4k, bool has_16k)
{
	if (!size || size == PAGE_SIZE)
		return 0;
	if (!enabled)
		return -EINVAL;
	if (size == SZ_4K)
		return has_4k ? 12 : -EOPNOTSUPP;
	if (PAGE_SIZE > SZ_16K && size == SZ_16K)
		return has_16k ? 14 : -EOPNOTSUPP;
	return -EINVAL;
}

/* AArch32 falls back to the native ABI when 4K translation is unavailable. */
static inline unsigned int arm64_exec_page_shift(bool compat, bool has_4k,
		unsigned int native_shift, unsigned int requested_shift)
{
	if (compat)
		return has_4k ? 12 : native_shift;
	return requested_shift ?: native_shift;
}

typedef struct {
	atomic64_t	id;
#ifdef CONFIG_COMPAT
	void		*sigpage;
#endif
	refcount_t	pinned;
	void		*vdso;
	unsigned long	flags;
	u8		pkey_allocation_map;
#ifdef CONFIG_ARM64_USER4K_EXPERIMENTAL
	/* PTE/PMD/PUD fragment pools; initialized before atomic publication. */
	void		*user4k_pt_frag[3];
#endif
} mm_context_t;

static inline bool arm64_context_user4k(const mm_context_t *context)
{
	return IS_ENABLED(CONFIG_ARM64_USER4K_EXPERIMENTAL) &&
	       (context->flags & MMCF_USER_4K);
}

static inline unsigned int arm64_context_page_shift(const mm_context_t *context)
{
	if (IS_ENABLED(CONFIG_ARM64_USER4K_EXPERIMENTAL)) {
		if (context->flags & MMCF_USER_4K)
			return 12;
		if (context->flags & MMCF_USER_16K)
			return 14;
		if (context->flags & MMCF_USER_64K)
			return 16;
	}
	return PAGE_SHIFT;
}

#ifdef CONFIG_ARM64_USER4K_EXPERIMENTAL
#define arch_mm_page_shift(mm) arm64_context_page_shift(&(mm)->context)
#endif

/*
 * We use atomic64_read() here because the ASID for an 'mm_struct' can
 * be reallocated when scheduling one of its threads following a
 * rollover event (see new_context() and flush_context()). In this case,
 * a concurrent TLBI (e.g. via try_to_unmap_one() and ptep_clear_flush())
 * may use a stale ASID. This is fine in principle as the new ASID is
 * guaranteed to be clean in the TLB, but the TLBI routines have to take
 * care to handle the following race:
 *
 *    CPU 0                    CPU 1                          CPU 2
 *
 *    // ptep_clear_flush(mm)
 *    xchg_relaxed(pte, 0)
 *    DSB ISHST
 *    old = ASID(mm)
 *         |                                                  <rollover>
 *         |                   new = new_context(mm)
 *         \-----------------> atomic_set(mm->context.id, new)
 *                             cpu_switch_mm(mm)
 *                             // Hardware walk of pte using new ASID
 *    TLBI(old)
 *
 * In this scenario, the barrier on CPU 0 and the dependency on CPU 1
 * ensure that the page-table walker on CPU 1 *must* see the invalid PTE
 * written by CPU 0.
 */
#define ASID(mm)	(atomic64_read(&(mm)->context.id) & 0xffff)

static inline bool arm64_kernel_unmapped_at_el0(void)
{
	return alternative_has_cap_unlikely(ARM64_UNMAP_KERNEL_AT_EL0);
}

extern void arm64_memblock_init(void);
extern void paging_init(void);
extern void bootmem_init(void);
extern void create_mapping_noalloc(phys_addr_t phys, unsigned long virt,
				   phys_addr_t size, pgprot_t prot);
extern void create_pgd_mapping(struct mm_struct *mm, phys_addr_t phys,
			       unsigned long virt, phys_addr_t size,
			       pgprot_t prot, bool page_mappings_only);
extern void *fixmap_remap_fdt(phys_addr_t dt_phys, int *size, pgprot_t prot);
extern void mark_linear_text_alias_ro(void);
extern int split_kernel_leaf_mapping(unsigned long start, unsigned long end);
extern void linear_map_maybe_split_to_ptes(void);

/*
 * This check is triggered during the early boot before the cpufeature
 * is initialised. Checking the status on the local CPU allows the boot
 * CPU to detect the need for non-global mappings and thus avoiding a
 * pagetable re-write after all the CPUs are booted. This check will be
 * anyway run on individual CPUs, allowing us to get the consistent
 * state once the SMP CPUs are up and thus make the switch to non-global
 * mappings if required.
 */
static inline bool kaslr_requires_kpti(void)
{
	/*
	 * E0PD does a similar job to KPTI so can be used instead
	 * where available.
	 */
	if (IS_ENABLED(CONFIG_ARM64_E0PD)) {
		u64 mmfr2 = read_sysreg_s(SYS_ID_AA64MMFR2_EL1);
		if (cpuid_feature_extract_unsigned_field(mmfr2,
						ID_AA64MMFR2_EL1_E0PD_SHIFT))
			return false;
	}

	return true;
}

#ifdef CONFIG_UNMAP_KERNEL_AT_EL0
void kpti_install_ng_mappings(void);
#else
static inline void kpti_install_ng_mappings(void) {}
#endif

extern bool page_alloc_available;

#endif	/* !__ASSEMBLER__ */
#endif
