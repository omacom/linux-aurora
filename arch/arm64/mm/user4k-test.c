// SPDX-License-Identifier: GPL-2.0-only
/*
 * Architectural prerequisite for mixed-granule userspace.
 *
 * Packed architectural tables and isolated Linux mms are separate fixtures.
 * PAGE_SIZE, struct page, all ordinary mms and TTBR1 retain native geometry. Only bounded
 * kernel workers may borrow the isolated Linux mms. Never pass
 * packed architectural tables to generic MM walkers or expose them to a task.
 */
#include <kunit/test.h>
#include <linux/bitfield.h>
#include <linux/bio.h>
#include <linux/anon_inodes.h>
#include <linux/coredump.h>
#include <linux/cgroup.h>
#include <linux/delay.h>
#include <linux/binfmts.h>
#include <linux/hugetlb.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iova_bitmap.h>
#include <linux/of_address.h>
#include <linux/file.h>
#include <linux/falloc.h>
#include <linux/shmem_fs.h>
#include <linux/seq_file.h>
#include <linux/swap_ops.h>
#include <linux/kthread.h>
#include <linux/khugepaged.h>
#include <linux/mm.h>
#include <linux/mman.h>
#include <linux/migrate.h>
#include <linux/mm_subpage.h>
#include <linux/mempolicy.h>
#include <linux/memcontrol.h>
#include <linux/pagemap.h>
#include <linux/pagewalk.h>
#include <linux/personality.h>
#include <linux/prctl.h>
#include <linux/perf_event.h>
#include <linux/preempt.h>
#include <linux/rmap.h>
#include <linux/uprobes.h>
#include <linux/vdso_datastore.h>
#include <vdso/datapage.h>
#include <linux/umh.h>
#include <linux/userfaultfd_k.h>
#include <linux/uio.h>
#include <linux/syscalls.h>

#include <asm/cpufeature.h>
#include <asm/daifflags.h>
#include <asm/elf.h>
#include <asm/vdso.h>
#include <asm/mmu_context.h>
#include <asm/mte.h>
#include <asm/pgalloc.h>
#include <asm/ptrace.h>
#include <asm/pgtable-hwdef.h>
#include <asm/sysreg.h>
#include <asm/tlb.h>
#include <asm/tlbflush.h>
#include <asm/uaccess.h>

#include "../../../drivers/iommu/iommufd/iommufd_private.h"
#include "../../../drivers/iommu/iommufd/io_pagetable.h"
#include "../../../mm/internal.h"
#include "../../../mm/vma.h"
#include "../../../mm/swap.h"
#ifdef CONFIG_FUTEX
#include "../../../kernel/futex/futex.h"
#endif

#define USER4K_SHIFT	12
#define TEST_SLOTS MM_SUBPAGES_PER_PAGE
#define USER4K_SIZE	BIT(USER4K_SHIFT)
#define TEST_VA		(4UL * SZ_1M)
#define TEST_LEVELS	4
#define PROBE_SLOTS	4
#define TEST_ROUNDS	8
#define TEST_PROT	(PTE_TYPE_PAGE | PTE_USER | PTE_AF | PTE_NG | \
			 PTE_SHARED | PTE_PXN | PTE_UXN | PTE_ATTRINDX(MT_NORMAL))

static bool user4k_test_enabled;

static int __init user4k_test_setup(char *str)
{
	bool enabled;
	int ret = kstrtobool(str, &enabled);

	if (ret)
		return ret;
	/* Software MM tests are sanitizable; raw/borrowed TTBR0 tests stay gated. */
	if (enabled && IS_ENABLED(CONFIG_KASAN))
		return -EINVAL;
	user4k_test_enabled = enabled;
	return 0;
}
early_param("arm64.user4k_test", user4k_test_setup);

struct test_tables {
	u64 *level[TEST_LEVELS];
	unsigned int shift;
};

struct test_result {
	u64 native[PROBE_SLOTS];
	u64 read[PROBE_SLOTS];
	u64 write[PROBE_SLOTS];
	u64 replaced[PROBE_SLOTS];
	u64 replaced_read;
	u64 kernel;
	u64 tcr;
	u64 ttbr1;
	u64 restored_tcr;
	u64 restored_ttbr0;
};

static void test_free_page(void *ptr)
{
	free_page((unsigned long)ptr);
}

static void *test_alloc_page(struct kunit *test)
{
	void *ptr = (void *)get_zeroed_page(GFP_KERNEL);

	if (!ptr)
		return NULL;
	if (kunit_add_action_or_reset(test, test_free_page, ptr))
		return NULL;
	return ptr;
}

static void test_put_compound(void *ptr)
{
	put_page(ptr);
}

static void *test_alloc_granule(struct kunit *test, unsigned int shift)
{
	struct page *page;

	if (shift <= PAGE_SHIFT)
		return test_alloc_page(test);
	page = alloc_pages(GFP_KERNEL | __GFP_ZERO | __GFP_COMP, shift - PAGE_SHIFT);
	if (!page)
		return NULL;
	if (kunit_add_action_or_reset(test, test_put_compound, page))
		return NULL;
	return page_address(page);
}

static unsigned int test_index(unsigned long va, unsigned int shift,
			       unsigned int level)
{
	unsigned int bits = shift - PTDESC_ORDER;

	return (va >> ARM64_HW_PGTABLE_LEVEL_SHIFT_FOR(level, shift)) & (BIT(bits) - 1);
}

static int test_alloc_tables(struct kunit *test, struct test_tables *tables,
			     unsigned int shift)
{
	int level;
	void *packed = NULL;

	tables->shift = shift;
	if (shift == USER4K_SHIFT) {
		/* Also test table address bits 13:12, not just leaf addresses. */
		packed = test_alloc_page(test);
		if (!packed)
			return -ENOMEM;
	}
	for (level = 0; level < TEST_LEVELS; level++) {
		tables->level[level] = packed ? packed + level * USER4K_SIZE :
					       test_alloc_page(test);
		if (!tables->level[level])
			return -ENOMEM;
	}
	for (level = 0; level < TEST_LEVELS - 1; level++) {
		unsigned int index = test_index(TEST_VA, shift, level);

		tables->level[level][index] =
			virt_to_phys(tables->level[level + 1]) | PMD_TYPE_TABLE;
	}
	return 0;
}

static void user4k_geometry_test(struct kunit *test)
{
	static const unsigned int shifts4k[] = { 39, 30, 21, 12 };
	static const unsigned int shifts16k[] = { 47, 36, 25, 14 };
	static const unsigned int shifts64k[] = { 55, 42, 29, 16 };
	unsigned int level;

	KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVELS_FOR(48, 12), 4);
	KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVELS_FOR(48, 14), 4);
	KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVELS_FOR(48, 16), 3);
	KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVELS_FOR(42, 16), 2);
	KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVELS_FOR(39, 12), 3);
	KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVELS_FOR(47, 14), 3);
	for (level = 0; level < TEST_LEVELS; level++) {
		KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVEL_SHIFT_FOR(level, 12),
				shifts4k[level]);
		KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVEL_SHIFT_FOR(level, 14),
				shifts16k[level]);
		KUNIT_EXPECT_EQ(test, ARM64_HW_PGTABLE_LEVEL_SHIFT(level),
				PAGE_SHIFT == 16 ? shifts64k[level] : shifts16k[level]);
		KUNIT_EXPECT_EQ(test, test_index(BIT(shifts4k[level]), 12, level), 1U);
		KUNIT_EXPECT_EQ(test, test_index(BIT(shifts4k[level]) - 1, 12, level), 0U);
	}
}

static void test_leaf_encoding(struct kunit *test, unsigned int shift)
{
	struct mm_struct *mm = kunit_kzalloc(test, sizeof(*mm), GFP_KERNEL);
	const phys_addr_t base = GENMASK_ULL(47, PAGE_SHIFT);
	pgprot_t prot = __pgprot(TEST_PROT | PTE_RDONLY);
	unsigned int i;
	pte_t pte;

	KUNIT_ASSERT_NOT_NULL(test, mm);
	pte = phys_pte_mm(mm, base, prot);
	KUNIT_EXPECT_EQ(test, pte_val(pte), pte_val(pfn_pte(base >> PAGE_SHIFT, prot)));
	KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte), base);
	KUNIT_EXPECT_EQ(test, pgprot_val(pte_pgprot_mm(mm, pte)), pgprot_val(prot));
	mm->context.flags |= shift == 12 ? MMCF_USER_4K : MMCF_USER_16K;
	for (i = 0; i < (PAGE_SIZE >> shift); i++) {
		phys_addr_t phys = base + i * (1UL << shift);

		pte = phys_pte_mm(mm, phys, prot);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte), phys);
		KUNIT_EXPECT_EQ(test, pte_pfn(pte), (unsigned long)(base >> PAGE_SHIFT));
		KUNIT_EXPECT_EQ(test, pgprot_val(pte_pgprot_mm(mm, pte)), pgprot_val(prot));
		if (i + 1 < (PAGE_SIZE >> shift))
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte_advance_pfn_mm(mm, pte, 1)),
				phys + (1UL << shift));
		pte = pte_modify(pte, __pgprot(TEST_PROT));
		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte), phys);
		KUNIT_EXPECT_FALSE(test, pte_val(pte) & PTE_RDONLY);
	}
}

static void user4k_leaf_encoding_test(struct kunit *test)
{
	test_leaf_encoding(test, 12);
}

static void user16k_leaf_encoding_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K alternative leaves require larger native backing");
		return;
	}
	test_leaf_encoding(test, 14);
}


/* Caller masks all asynchronous exceptions; this function cannot sleep. */
static void test_switch_ttbr0(u64 ttbr, u64 tcr)
{
	/* The zero root is valid for both tested granules. */
	cpu_set_reserved_ttbr0();
	local_flush_tlb_all();
	write_sysreg(tcr, tcr_el1);
	isb();
	write_sysreg(ttbr, ttbr0_el1);
	isb();
}

static u64 test_translate(unsigned long va, bool write)
{
	if (write)
		asm volatile("at s1e0w, %0" : : "r" (va) : "memory");
	else
		asm volatile("at s1e0r, %0" : : "r" (va) : "memory");
	isb();
	return read_sysreg_par();
}

#define MM_TEST_MAPPINGS 11

struct test_mm {
	struct kunit *test;
	struct mm_struct *mm;
	unsigned long address[MM_TEST_MAPPINGS];
	phys_addr_t phys[MM_TEST_MAPPINGS];
};

static pte_t *test_mm_lookup(struct mm_struct *mm, unsigned long addr,
			     spinlock_t **ptl);

static void test_mm_hardware(struct test_mm *ctx)
{
	struct kunit *test = ctx->test;
	struct vm_area_struct vma = TLB_FLUSH_VMA(ctx->mm, 0);
	struct arch_tlbflush_unmap_batch batch = {};
	pte_t *leaves[4];
	u64 invalidated[3][4];
	u64 result[MM_TEST_MAPPINGS], writes[MM_TEST_MAPPINGS];
	u64 old_tcr, old_ttbr0, old_ttbr1, old_par, tcr, kernel;
	u64 old_id = atomic64_read(&ctx->mm->context.id);
	u64 restored_tcr, restored_ttbr0, retained_ttbr1;
	unsigned long flags;
	unsigned int i, step;

	for (i = 0; i < ARRAY_SIZE(leaves); i++) {
		spinlock_t *ptl;

		leaves[i] = test_mm_lookup(ctx->mm, ctx->address[i], &ptl);
		if (leaves[i])
			pte_unmap_unlock(leaves[i], ptl);
		KUNIT_ASSERT_NOT_NULL(test, leaves[i]);
	}

	preempt_disable();
	flags = local_daif_save();
	old_tcr = read_sysreg(tcr_el1);
	old_ttbr0 = read_sysreg(ttbr0_el1);
	old_ttbr1 = read_sysreg(ttbr1_el1);
	old_par = read_sysreg_par();
	if ((old_tcr & (TCR_EL1_DS | TCR_EL1_TG1_MASK | TCR_EL1_A1)) !=
	    (TCR_TG1_16K | TCR_EL1_A1)) {
		local_daif_restore(flags);
		preempt_enable();
		kunit_skip(test, "requires 16K TTBR1, DS=0, and A1=1");
		return;
	}
	tcr = (old_tcr & ~(TCR_EL1_TG0_MASK | TCR_EL1_T0SZ_MASK | TCR_EL1_EPD0)) |
		TCR_TG0_4K | TCR_T0SZ(48);
	test_switch_ttbr0(virt_to_phys(ctx->mm->pgd), tcr);
	/* A1 selects the saved TTBR1 ASID for these private test translations. */
	atomic64_set(&ctx->mm->context.id, old_ttbr1 >> 48);
	for (i = 0; i < MM_TEST_MAPPINGS; i++) {
		result[i] = test_translate(ctx->address[i], false);
		writes[i] = test_translate(ctx->address[i], true);
	}
	for (step = 0; step < ARRAY_SIZE(invalidated); step++) {
		unsigned int first = step == 2 ? 3 : 1;
		unsigned int last = step == 1 ? 3 : first + 1;

		for (i = first; i < last; i++)
			WRITE_ONCE(*leaves[i], __pte(0));
		switch (step) {
		case 0:
			__flush_tlb_page(&vma, ctx->address[first] + 1,
					 TLBF_NONOTIFY | TLBF_NOBROADCAST);
			break;
		case 1:
			__flush_tlb_range(&vma, ctx->address[first], ctx->address[last],
					  SZ_4K, 3, TLBF_NONOTIFY);
			break;
		case 2:
			arch_tlbbatch_add_pending(&batch, ctx->mm, ctx->address[first],
						 ctx->address[first] + SZ_4K);
			arch_tlbbatch_flush(&batch);
			break;
		}
		isb();
		for (i = 0; i < ARRAY_SIZE(leaves); i++)
			invalidated[step][i] = test_translate(ctx->address[i], false);
		for (i = first; i < last; i++)
			WRITE_ONCE(*leaves[i], __pte(ctx->phys[i] | TEST_PROT));
		dsb(ishst);
		isb();
		/* Repopulate before the next independent invalidation. */
		for (i = 0; i < ARRAY_SIZE(leaves); i++)
			test_translate(ctx->address[i], false);
	}
	asm volatile("at s1e1r, %0" : : "r" (ctx->mm->pgd) : "memory");
	isb();
	kernel = read_sysreg_par();
	retained_ttbr1 = read_sysreg(ttbr1_el1);
	test_switch_ttbr0(old_ttbr0, old_tcr);
	restored_tcr = read_sysreg(tcr_el1);
	restored_ttbr0 = read_sysreg(ttbr0_el1);
	write_sysreg(old_par, par_el1);
	isb();
	atomic64_set(&ctx->mm->context.id, old_id);
	local_daif_restore(flags);
	preempt_enable();

	KUNIT_EXPECT_EQ(test, restored_tcr, old_tcr);
	KUNIT_EXPECT_EQ(test, restored_ttbr0, old_ttbr0);
	KUNIT_EXPECT_EQ(test, retained_ttbr1, old_ttbr1);
	KUNIT_EXPECT_EQ(test, kernel & SYS_PAR_EL1_F, 0ULL);
	KUNIT_EXPECT_EQ(test, kernel & SYS_PAR_EL1_PA,
			virt_to_phys(ctx->mm->pgd) & SYS_PAR_EL1_PA);
	for (step = 0; step < ARRAY_SIZE(invalidated); step++) {
		for (i = 0; i < ARRAY_SIZE(leaves); i++) {
			bool hole = step == 0 ? i == 1 : step == 1 ? i == 1 || i == 2 : i == 3;

			KUNIT_EXPECT_EQ(test, invalidated[step][i] & SYS_PAR_EL1_F, (u64)hole);
			if (!hole)
				KUNIT_EXPECT_EQ(test, invalidated[step][i] & SYS_PAR_EL1_PA,
						ctx->phys[i]);
		}
	}
	for (i = 0; i < MM_TEST_MAPPINGS; i++) {
		KUNIT_EXPECT_EQ(test, result[i] & SYS_PAR_EL1_F, 0ULL);
		KUNIT_EXPECT_EQ(test, result[i] & SYS_PAR_EL1_PA, ctx->phys[i]);
		KUNIT_EXPECT_EQ(test, writes[i] & SYS_PAR_EL1_F, 0ULL);
		KUNIT_EXPECT_EQ(test, writes[i] & SYS_PAR_EL1_PA, ctx->phys[i]);
	}
}

struct test_walk {
	struct test_mm *ctx;
	unsigned long size;
	unsigned long holes;
	unsigned int present;
};

/* Callbacks run under the PTE lock; record errors without invoking KUnit. */
static int test_walk_pte(pte_t *pte, unsigned long addr, unsigned long next,
			 struct mm_walk *walk)
{
	struct test_walk *state = walk->private;
	u64 value = pte_val(ptep_get(pte));
	unsigned int i;

	if (next - addr != state->size)
		return -EINVAL;
	if (!value) {
		state->holes += next - addr;
		return 0;
	}
	for (i = 0; i < MM_TEST_MAPPINGS; i++) {
		if (state->ctx->address[i] != addr)
			continue;
		if (value != (state->ctx->phys[i] | TEST_PROT))
			return -EINVAL;
		state->present++;
		return 0;
	}
	return -EINVAL;
}

static int test_walk_hole(unsigned long addr, unsigned long next, int depth,
			  struct mm_walk *walk)
{
	struct test_walk *state = walk->private;

	state->holes += next - addr;
	return 0;
}

static const struct mm_walk_ops test_walk_ops = {
	.pte_entry = test_walk_pte,
	.pte_hole = test_walk_hole,
};

static pte_t *test_mm_lookup(struct mm_struct *mm, unsigned long addr,
			     spinlock_t **ptl)
{
	pgd_t *pgd = pgd_offset(mm, addr);
	p4d_t *p4d;
	pud_t *pud;
	pmd_t *pmd;

	if (pgd_none(*pgd) || pgd_bad(*pgd))
		return NULL;
	p4d = p4d_offset_mm(mm, pgd, addr);
	if (p4d_none_mm(mm, *p4d) || p4d_bad_mm(mm, *p4d))
		return NULL;
	pud = pud_offset_mm(mm, p4d, addr);
	if (pud_none(*pud) || pud_bad(*pud))
		return NULL;
	pmd = pmd_offset_mm(mm, pud, addr);
	if (pmd_none(*pmd) || pmd_bad(*pmd))
		return NULL;
	return pte_offset_map_lock(mm, pmd, addr, ptl);
}

static void test_mm_destroy(void *ptr)
{
	struct test_mm *ctx = ptr;
	struct mm_struct *mm = ctx->mm;
	struct mmu_gather tlb;
	unsigned int i;

	if (!mm)
		return;
	mmap_write_lock(mm);
	for (i = 0; i < MM_TEST_MAPPINGS; i++) {
		spinlock_t *ptl;
		pte_t *pte = test_mm_lookup(mm, ctx->address[i], &ptl);

		if (pte) {
			/* Test-owned physical pages, not faulted/rmap-owned pages. */
			pte_clear(mm, ctx->address[i], pte);
			pte_unmap_unlock(pte, ptl);
		}
	}
	/* Exercise the real generic gather/teardown path, not a test walker. */
	tlb_gather_mmu_fullmm(&tlb, mm);
	free_pgd_range(&tlb, 0, 1UL << 48, 0, 0);
	tlb_finish_mmu(&tlb);
	mmap_write_unlock(mm);
	KUNIT_EXPECT_EQ(ctx->test, mm_pgtables_bytes(mm), 0UL);
	mmput(mm);
	ctx->mm = NULL;
}

static unsigned int test_mm_backing_pages(struct test_mm *ctx)
{
	struct page *owners[3 * MM_TEST_MAPPINGS];
	unsigned int count = 0, total = 0, i, j, level;
	struct mm_struct *mm = ctx->mm;

	for (i = 0; i < MM_TEST_MAPPINGS; i++) {
		unsigned long addr = ctx->address[i];
		p4d_t *p4d = p4d_offset_mm(mm, pgd_offset(mm, addr), addr);
		pud_t *pud = pud_offset_mm(mm, p4d, addr);
		pmd_t *pmd = pmd_offset_mm(mm, pud, addr);
		pte_t *pte = pte_offset_kernel_mm(mm, pmd, addr);
		struct page *pages[] = { virt_to_page(pud), virt_to_page(pmd), virt_to_page(pte) };

		for (level = mm_pud_folded(mm) ? 1 : 0; level < ARRAY_SIZE(pages); level++) {
			pages[level] = compound_head(pages[level]);
			for (j = 0; j < count; j++)
				if (owners[j] == pages[level])
					break;
			if (j == count) {
				owners[count++] = pages[level];
				total += 1U << compound_order(pages[level]);
			}
		}
	}
	return total;
}

/* Only valid for a private, empty mm with no ASID or hardware references. */
static int test_mm_select_granule(struct mm_struct *mm, unsigned int shift)
{
	pgd_t *native_pgd = mm->pgd, *new_pgd;
	unsigned long flag = shift == 12 ? MMCF_USER_4K :
			     shift == 14 ? MMCF_USER_16K : MMCF_USER_64K;

	mm->context.flags |= flag;
	new_pgd = pgd_alloc(mm);
	mm->context.flags &= ~flag;
	if (!new_pgd)
		return -ENOMEM;
	pgd_free(mm, native_pgd);
	mm->pgd = new_pgd;
	mm->context.flags |= flag;
	return 0;
}

static int test_mm_select4k(struct mm_struct *mm)
{
	return test_mm_select_granule(mm, 12);
}

static void test_mm_tables(struct kunit *test, unsigned int shift, bool hardware)
{
	struct test_mm *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	bool small = shift != PAGE_SHIFT;
	unsigned int pmd_shift = small ? shift + (shift - 3) : PMD_SHIFT;
	unsigned int pud_shift = small ? shift + 2 * (shift - 3) : PUD_SHIFT;
	unsigned int pgd_shift = small ? (shift == 16 ? pud_shift :
					shift + 3 * (shift - 3)) : PGDIR_SHIFT;
	unsigned long size = 1UL << shift;
	struct mm_struct *mm;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	ctx->test = test;
	for (i = 0; i < 4; i++)
		ctx->address[i] = TEST_VA + i * size;
	ctx->address[4] = (1UL << pmd_shift) - size;
	ctx->address[5] = 1UL << pmd_shift;
	ctx->address[6] = (1UL << pud_shift) - size;
	ctx->address[7] = 1UL << pud_shift;
	ctx->address[8] = (1UL << pgd_shift) - size;
	ctx->address[9] = 1UL << pgd_shift;
	ctx->address[10] = (1UL << 48) - size;
	if (pud_shift == pgd_shift) {
		ctx->address[6] -= size;
		ctx->address[7] += size;
	}
	for (i = 0; i < MM_TEST_MAPPINGS; i++) {
		void *data = test_alloc_granule(test, shift);

		KUNIT_ASSERT_NOT_NULL(test, data);
		ctx->phys[i] = virt_to_phys(data) + (shift < PAGE_SHIFT ? (i % (PAGE_SIZE >> shift)) * size : 0);
	}
	mm = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, mm);
	ctx->mm = mm;
	/* Register after the data allocations: detach tables before freeing data. */
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_mm_destroy, ctx), 0);
	KUNIT_EXPECT_EQ(test, mm_page_shift(mm), (unsigned int)PAGE_SHIFT);
	if (small) {
		KUNIT_ASSERT_EQ(test, test_mm_select_granule(mm, shift), 0);
		KUNIT_EXPECT_EQ(test, (unsigned long)mm->pgd & (size - 1), 0UL);
	}
	KUNIT_EXPECT_EQ(test, mm_page_shift(mm), shift);
	if (small)
		KUNIT_EXPECT_EQ(test, mm_pud_folded(mm), shift == 16);
	KUNIT_EXPECT_EQ(test, mm_page_size(mm), size);
	KUNIT_EXPECT_EQ(test, mm_page_mask(mm), ~(size - 1));
	KUNIT_EXPECT_EQ(test, mm_page_shift(&init_mm), (unsigned int)PAGE_SHIFT);
	KUNIT_EXPECT_EQ(test, pmd_addr_end_mm(mm, (1UL << pmd_shift) - 1,
					     (1UL << pmd_shift) + size),
			1UL << pmd_shift);
	KUNIT_EXPECT_EQ(test, pgd_addr_end_mm(mm, ULONG_MAX - size, 0), 0UL);

	for (i = 0; i < MM_TEST_MAPPINGS; i++) {
		unsigned long addr = ctx->address[i];
		pgd_t *pgd = pgd_offset(mm, addr);
		p4d_t *p4d = p4d_alloc(mm, pgd, addr);
		pud_t *pud;
		pmd_t *pmd;
		pte_t *pte;
		spinlock_t *ptl;

		KUNIT_ASSERT_NOT_NULL(test, p4d);
		pud = pud_alloc(mm, p4d, addr);
		KUNIT_ASSERT_NOT_NULL(test, pud);
		pmd = pmd_alloc(mm, pud, addr);
		KUNIT_ASSERT_NOT_NULL(test, pmd);
		pte = pte_alloc_map_lock(mm, pmd, addr, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, pte);
		/* Do not assert while holding the spinlock/RCU section. */
		set_pte(pte, __pte(ctx->phys[i] | TEST_PROT));
		pte_unmap_unlock(pte, ptl);
	}
	/* Installed table bytes and physical backing (including cached slots) differ. */
	{
		unsigned int shifts[] = { pmd_shift, pud_shift, pgd_shift };
		unsigned int tables = 0, owners = 0;

		for (unsigned int level = 0; level < (mm_pud_folded(mm) ? 2 : 3); level++) {
			unsigned int unique = 0;

			for (unsigned int a = 0; a < MM_TEST_MAPPINGS; a++) {
				unsigned int b;

				for (b = 0; b < a; b++)
					if ((ctx->address[a] >> shifts[level]) ==
					    (ctx->address[b] >> shifts[level]))
						break;
				unique += b == a;
			}
			tables += unique;
			if (shift < PAGE_SHIFT)
				owners += DIV_ROUND_UP(unique, PAGE_SIZE >> shift);
			else
				owners += unique << (shift - PAGE_SHIFT);
		}
		KUNIT_EXPECT_EQ(test, mm_pgtables_bytes(mm), tables * size);
		KUNIT_EXPECT_EQ(test, test_mm_backing_pages(ctx), owners);
	}
	for (i = 0; i < MM_TEST_MAPPINGS; i++) {
		spinlock_t *ptl;
		pte_t *pte = test_mm_lookup(mm, ctx->address[i], &ptl);
		u64 value;

		KUNIT_ASSERT_NOT_NULL(test, pte);
		value = pte_val(ptep_get(pte));
		pte_unmap_unlock(pte, ptl);
		KUNIT_EXPECT_EQ(test, value, ctx->phys[i] | TEST_PROT);
	}
	{
		struct vm_area_struct *vma = vm_area_alloc(mm);
		struct test_walk state = { .ctx = ctx, .size = size };
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, vma);
		vma->vm_start = 0;
		vma->vm_end = 1UL << 48;
		mmap_write_lock(mm);
		ret = walk_page_range_vma(vma, 0, 1UL << 48, &test_walk_ops, &state);
		mmap_write_unlock(mm);
		vm_area_free(vma);
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, state.present, MM_TEST_MAPPINGS);
		KUNIT_EXPECT_EQ(test, state.holes, (1UL << 48) - MM_TEST_MAPPINGS * size);
	}
	if (hardware)
		test_mm_hardware(ctx);
	{
		struct mmu_gather tlb;
		unsigned long before = mm_pgtables_bytes(mm);
		unsigned long boundary = 1UL << pmd_shift;

		mmap_write_lock(mm);
		/* An adjacent VMA's unaligned ceiling must keep the shared table. */
		tlb_gather_mmu_fullmm(&tlb, mm);
		free_pgd_range(&tlb, 0, boundary - size, 0, boundary - size);
		tlb_finish_mmu(&tlb);
		KUNIT_EXPECT_EQ(test, mm_pgtables_bytes(mm), before);
		for (i = 0; i < MM_TEST_MAPPINGS; i++) {
			spinlock_t *ptl;
			pte_t *pte;

			if (ctx->address[i] >= boundary)
				continue;
			pte = test_mm_lookup(mm, ctx->address[i], &ptl);
			if (pte) {
				pte_clear(mm, ctx->address[i], pte);
				pte_unmap_unlock(pte, ptl);
			}
		}
		tlb_gather_mmu_fullmm(&tlb, mm);
		free_pgd_range(&tlb, 0, boundary, 0, boundary);
		tlb_finish_mmu(&tlb);
		mmap_write_unlock(mm);
		KUNIT_EXPECT_EQ(test, mm_pgtables_bytes(mm), before - size);
		/* The next table and the higher levels must still be walkable. */
		for (i = 0; i < MM_TEST_MAPPINGS; i++) {
			spinlock_t *ptl;
			pte_t *pte = test_mm_lookup(mm, ctx->address[i], &ptl);
			u64 value;

			if (ctx->address[i] < boundary) {
				if (pte)
					pte_unmap_unlock(pte, ptl);
				KUNIT_EXPECT_PTR_EQ(test, pte, NULL);
				continue;
			}
			KUNIT_ASSERT_NOT_NULL(test, pte);
			value = pte_val(ptep_get(pte));
			pte_unmap_unlock(pte, ptl);
			KUNIT_EXPECT_EQ(test, value, ctx->phys[i] | TEST_PROT);
		}
	}
	test_mm_destroy(ctx);
}

static void user4k_mm_tables_test(struct kunit *test)
{
	test_mm_tables(test, 12, false);
}

static void native_mm_tables_test(struct kunit *test)
{
	test_mm_tables(test, PAGE_SHIFT, false);
}

static void user16k_mm_tables_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K alternative tables require larger native backing");
		return;
	}
	test_mm_tables(test, 14, false);
}

static void user64k_mm_tables_test(struct kunit *test)
{
	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "64K is already the native geometry");
		return;
	}
	test_mm_tables(test, 16, false);
}

static void test_free_vma(void *ptr)
{
	vm_area_free(ptr);
}

static void user4k_anon_read_fault_test(struct kunit *test)
{
	struct test_mm *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	struct vm_area_struct *vma;
	vm_flags_t vm_flags = VM_READ | VM_MAYREAD;
	unsigned int i, j;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	ctx->test = test;
	ctx->mm = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, ctx->mm);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_mm_destroy, ctx), 0);
	KUNIT_ASSERT_EQ(test, test_mm_select4k(ctx->mm), 0);
	for (i = 0; i < 4; i++)
		ctx->address[i] = TEST_VA + i * SZ_4K;
	vma = vm_area_alloc(ctx->mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_free_vma, vma), 0);
	vma->vm_start = TEST_VA;
	vma->vm_end = TEST_VA + PAGE_SIZE;
	vma_set_anonymous(vma);
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	vm_flags |= VM_NOHUGEPAGE;
#endif
	vm_flags_init(vma, vm_flags);
	vma->vm_page_prot = vm_get_page_prot(vm_flags);
	/* This VMA is a private fault fixture, not inserted into the VMA tree. */
	for (i = 0; i < 4; i++) {
		vm_fault_t ret;

		mmap_read_lock(ctx->mm);
		ret = handle_mm_fault(vma, ctx->address[i] + 1, FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(ctx->mm);
		KUNIT_ASSERT_EQ(test, ret, (vm_fault_t)0);
		for (j = 0; j < 4; j++) {
			spinlock_t *ptl;
			pte_t *ptep = test_mm_lookup(ctx->mm, ctx->address[j], &ptl);
			pte_t pte;

			KUNIT_ASSERT_NOT_NULL(test, ptep);
			pte = ptep_get(ptep);
			pte_unmap_unlock(ptep, ptl);
			if (j > i) {
				KUNIT_EXPECT_TRUE(test, pte_none(pte));
				continue;
			}
			KUNIT_EXPECT_TRUE(test, pte_present(pte));
			KUNIT_EXPECT_TRUE(test, pte_special(pte));
			KUNIT_EXPECT_FALSE(test, pte_write(pte));
			KUNIT_EXPECT_TRUE(test, is_zero_pfn(pte_pfn(pte)));
		}
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(ctx->mm, MM_ANONPAGES), 0L);
}

static void user4k_mm_hardware_test(struct kunit *test)
{
	if (!user4k_test_enabled) {
		kunit_skip(test, "requires arm64.user4k_test=1 in a disposable VM");
		return;
	}
	if (!system_supports_4kb_granule() || system_supports_cnp() ||
	    cpus_have_final_cap(ARM64_HAS_S1PIE) ||
	    cpus_have_final_cap(ARM64_HAS_S1POE)) {
		kunit_skip(test, "requires 4K, no CnP, and direct PTE permissions");
		return;
	}
	test_mm_tables(test, 12, true);
}

struct fragment_fixture {
	struct mm_struct *mm;
	void *table[TEST_SLOTS + 2];
	struct page *pinned[3];
};

static void fragment_fixture_destroy(void *data)
{
	struct fragment_fixture *f = data;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(f->table); i++)
		if (f->table[i])
			arm64_pgtable_free(f->table[i]);
	if (f->mm)
		mmput(f->mm);
	rcu_barrier();
	for (i = 0; i < ARRAY_SIZE(f->pinned); i++)
		if (f->pinned[i])
			put_page(f->pinned[i]);
}

static void test_fragment_lifetime(struct kunit *test, unsigned int shift)
{
	struct fragment_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	struct ptdesc *pte_owner;
	unsigned int i, slots = PAGE_SIZE >> shift;
	unsigned long size = 1UL << shift;
	unsigned int read_shift = 0;
	int pending_refs;
	bool live;

	KUNIT_ASSERT_NOT_NULL(test, f);
	f->mm = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, f->mm);
	KUNIT_ASSERT_EQ(test,
		kunit_add_action_or_reset(test, fragment_fixture_destroy, f), 0);
	/* Direct allocator fixture: the mm's native root is never changed or used. */
	for (i = 0; i < slots + 2; i++) {
		enum arm64_user4k_pt_level level = i < slots ? USER4K_PT_PTE : i - slots + 1;

		f->table[i] = arm64_user_pt_alloc_granule(f->mm, level, shift);
		KUNIT_ASSERT_NOT_NULL(test, f->table[i]);
		KUNIT_EXPECT_TRUE(test, IS_ALIGNED((unsigned long)f->table[i], size));
		KUNIT_EXPECT_EQ(test, arm64_ptep_page_shift(f->table[i]), shift);
		KUNIT_EXPECT_EQ(test, arm64_ptep_page_shift(f->table[i] + size - sizeof(pte_t)), shift);
		KUNIT_EXPECT_EQ(test, pte_batch_hint(f->table[i], __pte(TEST_PROT | PTE_CONT)),
				IS_ENABLED(CONFIG_ARM64_CONTPTE) ? arm64_cont_ptes(shift) : 1U);
		if (!i && PAGE_SHIFT == 16) {
			void *cached = f->mm->context.user4k_pt_frag[level];

			KUNIT_EXPECT_PTR_EQ(test, arm64_user_pt_alloc_granule(f->mm, level,
							shift == 12 ? 14 : 12), NULL);
			KUNIT_EXPECT_PTR_EQ(test, f->mm->context.user4k_pt_frag[level], cached);
		}
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(f->table[i], 0, size), NULL);
		if (i == 0 || i >= slots) {
			unsigned int slot = i == 0 ? 0 : i - slots + 1;

			f->pinned[slot] = virt_to_page(f->table[i]);
			get_page(f->pinned[slot]);
		} else {
			KUNIT_EXPECT_PTR_EQ(test, virt_to_page(f->table[i]), f->pinned[0]);
			KUNIT_EXPECT_EQ(test, (unsigned long)f->table[i] -
					(unsigned long)f->table[0], i * (unsigned long)size);
		}
		memset(f->table[i], 0x50 + i, size);
	}
	for (i = 0; i < slots + 2; i++) {
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(f->table[i], 0x50 + i, size), NULL);
		memset(f->table[i], 0, size);
	}
	/* Pointer-only walkers must inspect all 128 entries of a 16K group. */
	if (IS_ENABLED(CONFIG_ARM64_CONTPTE)) {
		pte_t *ptes = f->table[0];
		unsigned int nr = arm64_cont_ptes(shift), j;
		pte_t observed;

		for (j = 0; j < nr; j++)
			ptes[j] = pte_mkold(pte_mkclean(__pte(TEST_PROT | PTE_CONT |
							(SZ_4M + (j << shift)))));
		ptes[nr - 1] = pte_mkyoung(pte_mkdirty(ptes[nr - 1]));
		observed = ptep_get(ptes + 1);
		KUNIT_EXPECT_TRUE(test, pte_young(observed));
		KUNIT_EXPECT_TRUE(test, pte_dirty(observed));
		observed = ptep_get_lockless(ptes + 1);
		KUNIT_EXPECT_TRUE(test, pte_young(observed));
		KUNIT_EXPECT_TRUE(test, pte_dirty(observed));
		KUNIT_EXPECT_EQ(test, pte_val(observed) &
				(ARM64_USER4K_TABLE_ADDR_MASK & ~(size - 1)),
				(u64)(SZ_4M + size));
		memset(ptes, 0, size);
	}
	KUNIT_EXPECT_PTR_NE(test, f->pinned[0], f->pinned[1]);
	KUNIT_EXPECT_PTR_NE(test, f->pinned[1], f->pinned[2]);
	pte_owner = page_ptdesc(f->pinned[0]);
	KUNIT_EXPECT_EQ(test, atomic_read(&pte_owner->pt_frag_refcount), (int)slots);
	/* Refill holes in the same owner, without retaining old table contents. */
	for (i = 1; i < slots; i++) {
		void *old = f->table[i];

		memset(old, 0x6a, size);
		arm64_pgtable_free(old);
		f->table[i] = arm64_user_pt_alloc_granule(f->mm, USER4K_PT_PTE, shift);
		KUNIT_ASSERT_NOT_NULL(test, f->table[i]);
		KUNIT_EXPECT_PTR_EQ(test, f->table[i], old);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(f->table[i], 0, size), NULL);
	}
	/* Begin the reader before queuing a callback, so it cannot finish early. */
	rcu_read_lock();
	arm64_pte_free_defer(f->mm, f->table[0]);
	f->table[0] = NULL;
	pending_refs = atomic_read(&pte_owner->pt_frag_refcount);
	for (i = 1; i < slots - 1; i++) {
		arm64_pgtable_free(f->table[i]);
		f->table[i] = NULL;
	}
	/* Pending slot zero keeps the owner alive through this reader. */
	arm64_pgtable_free(f->table[slots - 1]);
	f->table[slots - 1] = NULL;
	live = folio_test_pgtable(page_folio(f->pinned[0]));
	read_shift = arm64_ptep_page_shift(page_address(f->pinned[0]));
	rcu_read_unlock();
	KUNIT_EXPECT_EQ(test, pending_refs, (int)slots);
	KUNIT_EXPECT_TRUE(test, live);
	KUNIT_EXPECT_EQ(test, read_shift, shift);
	rcu_barrier();
	KUNIT_EXPECT_FALSE(test, folio_test_pgtable(page_folio(f->pinned[0])));
	KUNIT_EXPECT_EQ(test, page_count(f->pinned[0]), 1);
	for (i = slots; i < slots + 2; i++) {
		struct ptdesc *pt = virt_to_ptdesc(f->table[i]);

		arm64_pgtable_free(f->table[i]);
		f->table[i] = NULL;
		/* A wholly empty owner is released without waiting for mm teardown. */
		KUNIT_EXPECT_EQ(test, atomic_read(&pt->pt_frag_refcount), 0);
		KUNIT_EXPECT_FALSE(test, folio_test_pgtable(ptdesc_folio(pt)));
	}
	mmput(f->mm);
	f->mm = NULL;
	for (i = 1; i < 3; i++) {
		KUNIT_EXPECT_FALSE(test, folio_test_pgtable(page_folio(f->pinned[i])));
		KUNIT_EXPECT_EQ(test, page_count(f->pinned[i]), 1);
	}
}

/* Direct table-owner test; no larger-than-native user mm is enabled here. */
static void user64k_table_owner_test(struct kunit *test)
{
	struct fragment_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	unsigned int i, offset, order = 16 - PAGE_SHIFT;
	unsigned int shift;
	bool live;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test,
		kunit_add_action_or_reset(test, fragment_fixture_destroy, f), 0);
	f->mm = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, f->mm);
	for (i = 0; i < 3; i++) {
		struct ptdesc *pt;

		f->table[i] = arm64_user_pt_alloc_granule(f->mm, i, 16);
		KUNIT_ASSERT_NOT_NULL(test, f->table[i]);
		pt = virt_to_ptdesc(f->table[i]);
		f->pinned[i] = ptdesc_page(pt);
		get_page(f->pinned[i]);
		KUNIT_EXPECT_TRUE(test, IS_ALIGNED((unsigned long)f->table[i], SZ_64K));
		KUNIT_EXPECT_EQ(test, compound_order(f->pinned[i]), order);
		KUNIT_EXPECT_EQ(test, atomic_read(&pt->pt_frag_refcount), 1);
		KUNIT_EXPECT_PTR_EQ(test, f->mm->context.user4k_pt_frag[i], NULL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(f->table[i], 0, SZ_64K), NULL);
		/* Include the last entry of every constituent native page. */
		for (offset = PAGE_SIZE - sizeof(pte_t); offset < SZ_64K; offset += PAGE_SIZE) {
			void *entry = f->table[i] + offset;

			KUNIT_EXPECT_PTR_EQ(test, virt_to_ptdesc(entry), pt);
			KUNIT_EXPECT_EQ(test, arm64_ptep_page_shift(entry), 16U);
			KUNIT_EXPECT_EQ(test, pte_batch_hint(entry, __pte(TEST_PROT | PTE_CONT)),
					32U - ((offset / sizeof(pte_t)) % 32));
			if (i == USER4K_PT_PTE)
				KUNIT_EXPECT_PTR_EQ(test, ptep_lockptr(f->mm, entry),
						ptep_lockptr(f->mm, f->table[i]));
			if (i == USER4K_PT_PMD)
				KUNIT_EXPECT_PTR_EQ(test, pmd_lockptr(f->mm, entry),
						pmd_lockptr(f->mm, f->table[i]));
		}
		memset(f->table[i], 0x51 + i, SZ_64K);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(f->table[i], 0x51 + i, SZ_64K), NULL);
		memset(f->table[i], 0, SZ_64K);
	}
	/* Contiguous readers on the last native page must still use 64K geometry. */
	if (IS_ENABLED(CONFIG_ARM64_CONTPTE)) {
		pte_t *ptes = f->table[0] + SZ_64K - 32 * sizeof(pte_t);
		pte_t observed;
		unsigned int j;

		for (j = 0; j < 32; j++)
			ptes[j] = pte_mkold(pte_mkclean(__pte(TEST_PROT | PTE_CONT |
							(SZ_4M + (j << 16)))));
		ptes[31] = pte_mkyoung(pte_mkdirty(ptes[31]));
		observed = ptep_get(ptes + 1);
		KUNIT_EXPECT_TRUE(test, pte_young(observed));
		KUNIT_EXPECT_TRUE(test, pte_dirty(observed));
		observed = ptep_get_lockless(ptes + 1);
		KUNIT_EXPECT_TRUE(test, pte_young(observed));
		KUNIT_EXPECT_TRUE(test, pte_dirty(observed));
		KUNIT_EXPECT_EQ(test, pte_val(observed) &
				(ARM64_USER4K_TABLE_ADDR_MASK & ~(SZ_64K - 1UL)),
				(u64)(SZ_4M + SZ_64K));
		memset(f->table[0], 0, SZ_64K);
	}
	rcu_read_lock();
	arm64_pte_free_defer(f->mm, f->table[0]);
	live = folio_test_pgtable(page_folio(f->pinned[0]));
	shift = arm64_ptep_page_shift(f->table[0] + SZ_64K - sizeof(pte_t));
	f->table[0] = NULL;
	rcu_read_unlock();
	KUNIT_EXPECT_TRUE(test, live);
	KUNIT_EXPECT_EQ(test, shift, 16U);
	rcu_barrier();
	for (i = 0; i < 3; i++) {
		if (f->table[i]) {
			arm64_pgtable_free(f->table[i]);
			f->table[i] = NULL;
		}
		KUNIT_EXPECT_FALSE(test, folio_test_pgtable(page_folio(f->pinned[i])));
		KUNIT_EXPECT_EQ(test, page_count(f->pinned[i]), 1);
		KUNIT_EXPECT_EQ(test, compound_order(f->pinned[i]), order);
	}
}

static void user4k_fragment_lifetime_test(struct kunit *test)
{
	test_fragment_lifetime(test, 12);
}

static void user16k_fragment_lifetime_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K fragments require larger native backing");
		return;
	}
	test_fragment_lifetime(test, 14);
}

#ifdef CONFIG_TRANSPARENT_HUGEPAGE
static void test_put_mm(void *ptr)
{
	mmput(ptr);
}

static void native_pgtable_deposit_test(struct kunit *test)
{
	struct mm_struct *mm = mm_alloc();
	pgtable_t first, second, out1, out2;
	pmd_t *pmd;
	spinlock_t *ptl;

	KUNIT_ASSERT_NOT_NULL(test, mm);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_put_mm, mm), 0);
	pmd = pmd_alloc_one(mm, 0);
	KUNIT_ASSERT_NOT_NULL(test, pmd);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, arm64_pgtable_free, pmd), 0);
	first = pte_alloc_one(mm);
	KUNIT_ASSERT_NOT_NULL(test, first);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, arm64_pgtable_free, first), 0);
	second = pte_alloc_one(mm);
	KUNIT_ASSERT_NOT_NULL(test, second);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, arm64_pgtable_free, second), 0);
	ptl = pmd_lock(mm, pmd);
	pgtable_trans_huge_deposit(mm, pmd, first);
	pgtable_trans_huge_deposit(mm, pmd, second);
	out1 = pgtable_trans_huge_withdraw(mm, pmd);
	out2 = pgtable_trans_huge_withdraw(mm, pmd);
	spin_unlock(ptl);
	KUNIT_EXPECT_PTR_EQ(test, out1, second);
	KUNIT_EXPECT_PTR_EQ(test, out2, first);
}
#endif

static void test_data_slot_put(void *ptr)
{
	mm_subpage_put(ptr);
}

static void test_data_pool_free(void *ptr)
{
	mm_subpage_pool_close(ptr);
	mm_subpage_pool_put(ptr);
}

static bool test_alloc_data_slots(struct kunit *test, struct test_mm *ctx)
{
	struct mm_subpage_pool *pool = mm_subpage_pool_create(GFP_KERNEL);
	struct folio *folio;
	unsigned int i;
	int ret;

	if (!pool)
		return false;
	if (kunit_add_action_or_reset(test, test_data_pool_free, pool))
		return false;
	folio = folio_alloc(GFP_KERNEL, 0);
	if (!folio)
		return false;
	ret = mm_subpage_pool_add_folio(pool, folio, GFP_KERNEL);
	if (ret) {
		folio_put(folio);
		return false;
	}
	for (i = 0; i < 4; i++) {
		struct mm_subpage *slot = mm_subpage_alloc(pool);

		if (IS_ERR(slot))
			return false;
		if (kunit_add_action_or_reset(test, test_data_slot_put, slot))
			return false;
		ctx->phys[i] = mm_subpage_phys(slot);
	}
	return true;
}

/* These private mappings are borrowed only for bounded, fault-disabled uaccess. */
static struct test_mm *test_usercopy_mm(struct kunit *test, bool small)
{
	struct test_mm *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	unsigned long size = small ? SZ_4K : PAGE_SIZE;
	unsigned int i;

	if (!ctx)
		return NULL;
	ctx->test = test;
	if (small && !test_alloc_data_slots(test, ctx))
		return NULL;
	for (i = 0; i < 4; i++) {
		if (!small) {
			void *data = test_alloc_page(test);

			if (!data)
				return NULL;
			ctx->phys[i] = virt_to_phys(data);
		}
		ctx->address[i] = TEST_VA + i * size;
		memset(phys_to_virt(ctx->phys[i]), (small ? 0x40 : 0x80) + i, 64);
	}
	ctx->mm = mm_alloc();
	if (!ctx->mm)
		return NULL;
	if (kunit_add_action_or_reset(test, test_mm_destroy, ctx))
		return NULL;
	if (small && test_mm_select4k(ctx->mm))
		return NULL;
	for (i = 0; i < 4; i++) {
		unsigned long addr = ctx->address[i];
		pgd_t *pgd = pgd_offset(ctx->mm, addr);
		p4d_t *p4d = p4d_alloc(ctx->mm, pgd, addr);
		pud_t *pud;
		pmd_t *pmd;
		pte_t *pte;
		spinlock_t *ptl;

		if (!p4d)
			return NULL;
		pud = pud_alloc(ctx->mm, p4d, addr);
		if (!pud)
			return NULL;
		pmd = pmd_alloc(ctx->mm, pud, addr);
		if (!pmd)
			return NULL;
		pte = pte_alloc_map_lock(ctx->mm, pmd, addr, &ptl);
		if (!pte)
			return NULL;
		if (i != 2)
			set_pte(pte, phys_pte_mm(ctx->mm, ctx->phys[i],
						__pgprot(TEST_PROT | (i ? 0 : PTE_RDONLY))));
		pte_unmap_unlock(pte, ptl);
	}
	return ctx;
}

static void user4k_rmap_walk_test(struct kunit *test)
{
	struct test_mm *ctx = test_usercopy_mm(test, true);
	struct vm_area_struct *vma;
	unsigned int i, j, sync;
	spinlock_t *ptl;
	pte_t *pte;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	vma = vm_area_alloc(ctx->mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_free_vma, vma), 0);
	vma->vm_start = TEST_VA;
	vma->vm_end = TEST_VA + 2 * PAGE_SIZE;
	/* A later alias must not extend a native-owner range walk's bound. */
	ctx->address[4] = TEST_VA + PAGE_SIZE;
	ctx->phys[4] = ctx->phys[0];
	pte = test_mm_lookup(ctx->mm, ctx->address[4], &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	set_pte(pte, phys_pte_mm(ctx->mm, ctx->phys[4], __pgprot(TEST_PROT)));
	pte_unmap_unlock(pte, ptl);
	/* Same physical slot at a different virtual byte offset and native index. */
	ctx->address[5] = TEST_VA + PAGE_SIZE + SZ_4K;
	ctx->phys[5] = ctx->phys[3];
	pte = test_mm_lookup(ctx->mm, ctx->address[5], &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	set_pte(pte, phys_pte_mm(ctx->mm, ctx->phys[5], __pgprot(TEST_PROT)));
	pte_unmap_unlock(pte, ptl);
	for (sync = 0; sync < 2; sync++) {
		struct page_vma_mapped_walk whole = {
			.pfn = ctx->phys[0] >> PAGE_SHIFT,
			.nr_pages = 1,
			.vma = vma,
			.address = TEST_VA,
			.flags = sync ? PVMW_SYNC : 0,
		};
		unsigned int count = 0;
		bool valid = true;

		/* A native-owner walk must visit every mapped 4K occupant. */
		while (page_vma_mapped_walk(&whole)) {
			unsigned int index = (whole.address - TEST_VA) / SZ_4K;

			if (index >= 4 || index == 2 || !whole.pte ||
			    pte_phys_mm(ctx->mm, ptep_get(whole.pte)) != ctx->phys[index])
				valid = false;
			if (++count > 4) {
				page_vma_mapped_walk_done(&whole);
				break;
			}
		}
		KUNIT_EXPECT_TRUE(test, valid);
		KUNIT_EXPECT_EQ(test, count, 3U);
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 6; j++) {
				struct rmap_walk_range range = {
					.address = ctx->address[j],
					.subpage = true,
					.subpage_offset = offset_in_page(ctx->phys[i]),
				};
				DEFINE_FOLIO_RMAP_WALK(one, page_folio(pfn_to_page(PHYS_PFN(ctx->phys[i]))),
					vma, range, sync ? PVMW_SYNC : 0);
				struct mm_subpage *slot = NULL;
				bool found = page_vma_mapped_walk(&one);
				phys_addr_t phys = 0;

				if (found) {
					phys = pte_phys_mm(ctx->mm, ptep_get(one.pte));
					slot = mm_subpage_get_from_phys(phys);
					page_vma_mapped_walk_done(&one);
				}
				KUNIT_EXPECT_EQ(test, found, (i == j && j != 2) ||
						(j == 4 && i == 0) || (j == 5 && i == 3));
				if (found) {
					KUNIT_EXPECT_EQ(test, phys, ctx->phys[i]);
					KUNIT_EXPECT_NOT_NULL(test, slot);
					if (slot) {
						KUNIT_EXPECT_EQ(test, mm_subpage_phys(slot), phys);
						mm_subpage_put(slot);
					}
				}
			}
		}
	}
}

static int test_map_alias(struct test_mm *ctx, unsigned int index,
			  unsigned long addr, phys_addr_t phys)
{
	struct mm_struct *mm = ctx->mm;
	pgd_t *pgd = pgd_offset(mm, addr);
	p4d_t *p4d = p4d_alloc(mm, pgd, addr);
	pud_t *pud;
	pmd_t *pmd;
	pte_t *pte;
	spinlock_t *ptl;

	ctx->address[index] = addr;
	ctx->phys[index] = phys;
	if (!p4d)
		return -ENOMEM;
	pud = pud_alloc(mm, p4d, addr);
	if (!pud)
		return -ENOMEM;
	pmd = pmd_alloc(mm, pud, addr);
	if (!pmd)
		return -ENOMEM;
	pte = pte_alloc_map_lock(mm, pmd, addr, &ptl);
	if (!pte)
		return -ENOMEM;
	set_pte(pte, pte_mkspecial(phys_pte_mm(mm, phys, __pgprot(TEST_PROT))));
	pte_unmap_unlock(pte, ptl);
	return 0;
}

#ifdef CONFIG_PERF_EVENTS
static void test_perf_page_size(struct kunit *test, unsigned int shift)
{
	const unsigned long addresses[] = {
		SZ_1M, SZ_1G + SZ_1M, (1UL << 39) + SZ_1M,
	};
	struct test_mm *ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	void *data = test_alloc_granule(test, PAGE_SHIFT);
	unsigned long flags;
	u64 present, absent;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	KUNIT_ASSERT_NOT_NULL(test, data);
	ctx->test = test;
	ctx->mm = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, ctx->mm);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_mm_destroy, ctx), 0);
	if (shift != PAGE_SHIFT)
		KUNIT_ASSERT_EQ(test, test_mm_select_granule(ctx->mm, shift), 0);
	for (i = 0; i < ARRAY_SIZE(addresses); i++)
		KUNIT_ASSERT_EQ(test, test_map_alias(ctx, i, addresses[i], virt_to_phys(data)), 0);
	for (i = 0; i < ARRAY_SIZE(addresses); i++) {
		local_irq_save(flags);
		present = perf_get_pgtable_size(ctx->mm, addresses[i]);
		absent = perf_get_pgtable_size(ctx->mm, addresses[i] + (1UL << shift));
		local_irq_restore(flags);
		KUNIT_EXPECT_EQ(test, present, 1ULL << shift);
		KUNIT_EXPECT_EQ(test, absent, 0ULL);
	}
}

static void user4k_perf_page_size_test(struct kunit *test)
{
	test_perf_page_size(test, 12);
}

static void user16k_perf_page_size_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "requires native backing larger than 16K");
		return;
	}
	test_perf_page_size(test, 14);
}

static void native_perf_page_size_test(struct kunit *test)
{
	test_perf_page_size(test, PAGE_SHIFT);
}
#endif

/* Physical-range lookup only; coarse anonymous ownership is a separate step. */
static void user64k_rmap_lookup_test(struct kunit *test)
{
	struct test_mm *ctx;
	struct vm_area_struct *vma;
	void *data;
	unsigned long pfn, per_leaf = SZ_64K / PAGE_SIZE;
	unsigned int mode, sync, i;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires userspace leaf larger than native page");
		return;
	}
	ctx = kunit_kzalloc(test, sizeof(*ctx), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ctx);
	ctx->test = test;
	data = test_alloc_granule(test, 17);
	KUNIT_ASSERT_NOT_NULL(test, data);
	pfn = virt_to_phys(data) >> PAGE_SHIFT;
	ctx->mm = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, ctx->mm);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_mm_destroy, ctx), 0);
	KUNIT_ASSERT_EQ(test, test_mm_select_granule(ctx->mm, 16), 0);
	vma = vm_area_alloc(ctx->mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_free_vma, vma), 0);
	vma->vm_start = (1UL << 29) - SZ_64K;
	vma->vm_end = vma->vm_start + 4 * SZ_64K;
	vma->vm_pgoff = 4096;
	page_folio(virt_to_page(data))->index = vma->vm_pgoff;
	for (i = 0; i < 3; i++)
		KUNIT_ASSERT_EQ(test, test_map_alias(ctx, i, vma->vm_start + i * SZ_64K,
				virt_to_phys(data) + (i == 1 ? SZ_64K : 0)), 0);

	for (mode = 0; mode < 2; mode++) {
		if (mode) {
			for (i = 0; i < 3; i++) {
				spinlock_t *ptl;
				pte_t *pte = test_mm_lookup(ctx->mm, ctx->address[i], &ptl);

				KUNIT_ASSERT_NOT_NULL(test, pte);
				set_pte(pte, swp_entry_to_pte(make_readable_migration_entry(
							ctx->phys[i] >> PAGE_SHIFT)));
				pte_unmap_unlock(pte, ptl);
			}
		}
		for (sync = 0; sync < 2; sync++) {
			unsigned int flags = (sync ? PVMW_SYNC : 0) |
				(mode ? PVMW_MIGRATION : 0);

			for (i = 0; i < 2 * per_leaf; i++) {
				struct page_vma_mapped_walk walk = {
					.pfn = pfn + i, .nr_pages = 1,
					/* Deliberately invalid pgoff: single-page callers may omit it. */
					.pgoff = 0, .vma = vma,
					.address = vma->vm_start + i * PAGE_SIZE,
					.flags = flags,
				};
				unsigned int count = 0;
				bool valid = true;

				while (page_vma_mapped_walk(&walk)) {
					valid &= walk.pte && walk.address ==
						vma->vm_start + (i / per_leaf) * SZ_64K;
					if (++count > 1) {
						page_vma_mapped_walk_done(&walk);
						break;
					}
				}
				KUNIT_EXPECT_TRUE(test, valid);
				KUNIT_EXPECT_EQ(test, count, 1U);
#ifdef CONFIG_MEMORY_FAILURE
				if (!mode)
					KUNIT_EXPECT_EQ(test, page_mapped_in_vma(pfn_to_page(pfn + i), vma),
							vma->vm_start + i * PAGE_SIZE);
#endif
			}
			/* Two native pages straddle a coarse-leaf and PTE-table boundary. */
			{
				struct page_vma_mapped_walk walk = {
					.pfn = pfn + per_leaf - 1, .nr_pages = 2,
					.pgoff = vma->vm_pgoff + per_leaf - 1, .vma = vma,
					.address = vma->vm_start + SZ_64K - PAGE_SIZE,
					.flags = flags,
				};
				unsigned int count = 0;
				bool valid = true;

				KUNIT_EXPECT_EQ(test, vma_address_end(&walk), vma->vm_start + 2 * SZ_64K);
				while (page_vma_mapped_walk(&walk)) {
					valid &= walk.address == vma->vm_start + count * SZ_64K;
					if (++count > 2) {
						page_vma_mapped_walk_done(&walk);
						break;
					}
				}
				KUNIT_EXPECT_TRUE(test, valid);
				KUNIT_EXPECT_EQ(test, count, 2U);
				KUNIT_EXPECT_TRUE(test, walk.flags & PVMW_PGTABLE_CROSSED);
			}
			for (i = 0; i < 2; i++) {
				struct page_vma_mapped_walk walk = {
					.pfn = i ? pfn + 2 * per_leaf : pfn - 1,
					.nr_pages = 1, .vma = vma,
					.address = vma->vm_start, .flags = flags,
				};
				bool found = page_vma_mapped_walk(&walk);

				if (found)
					page_vma_mapped_walk_done(&walk);
				KUNIT_EXPECT_FALSE(test, found);
			}
		}
	}
}

static void user4k_rmap_boundary_test(struct kunit *test)
{
	struct test_mm *ctx = test_usercopy_mm(test, true);
	struct vm_area_struct *vma;
	unsigned int i, count = 0;
	struct page_vma_mapped_walk walk;
	bool valid = true;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	vma = vm_area_alloc(ctx->mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_free_vma, vma), 0);
	vma->vm_start = SZ_2M - SZ_4K;
	vma->vm_end = vma->vm_start + PAGE_SIZE;
	vma->vm_pgoff = 37;
	page_folio(pfn_to_page(ctx->phys[0] >> PAGE_SHIFT))->index = vma->vm_pgoff;
	for (i = 0; i < 3; i++)
		KUNIT_ASSERT_EQ(test, test_map_alias(ctx, 4 + i, vma->vm_start + i * SZ_4K,
						    ctx->phys[i]), 0);
	walk = (struct page_vma_mapped_walk) {
		.pfn = ctx->phys[0] >> PAGE_SHIFT,
		.pgoff = vma->vm_pgoff,
		.nr_pages = 1,
		.vma = vma,
		.address = vma->vm_start,
		.flags = PVMW_SYNC,
	};
	while (page_vma_mapped_walk(&walk)) {
		if (count >= 3 || walk.address != vma->vm_start + count * SZ_4K ||
		    pte_phys_mm(ctx->mm, ptep_get(walk.pte)) != ctx->phys[count])
			valid = false;
		if (++count > 3) {
			page_vma_mapped_walk_done(&walk);
			break;
		}
	}
	KUNIT_EXPECT_TRUE(test, valid);
	KUNIT_EXPECT_EQ(test, count, 3U);
	KUNIT_EXPECT_TRUE(test, walk.flags & PVMW_PGTABLE_CROSSED);
#ifdef CONFIG_MEMORY_FAILURE
	{
		spinlock_t *ptl;
		pte_t *pte = test_mm_lookup(ctx->mm, vma->vm_start, &ptl);

		KUNIT_ASSERT_NOT_NULL(test, pte);
		ptep_get_and_clear(ctx->mm, vma->vm_start, pte);
		pte_unmap_unlock(pte, ptl);
		KUNIT_EXPECT_EQ(test,
			page_mapped_in_vma(pfn_to_page(ctx->phys[0] >> PAGE_SHIFT), vma),
			vma->vm_start + SZ_4K);
	}
#endif
}

static void native_rmap_walk_test(struct kunit *test)
{
	struct test_mm *ctx = test_usercopy_mm(test, false);
	struct vm_area_struct *vma;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, ctx);
	vma = vm_area_alloc(ctx->mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_free_vma, vma), 0);
	vma->vm_start = TEST_VA;
	vma->vm_end = TEST_VA + 4 * PAGE_SIZE;
	for (i = 0; i < 4; i++) {
		struct page_vma_mapped_walk walk = {
			.pfn = ctx->phys[i] >> PAGE_SHIFT,
			.nr_pages = 1,
			.vma = vma,
			.address = ctx->address[i],
			.flags = PVMW_SYNC,
		};
		unsigned int count = 0;
		bool valid = true;

		while (page_vma_mapped_walk(&walk)) {
			if (walk.address != ctx->address[i] ||
			    pte_phys_mm(ctx->mm, ptep_get(walk.pte)) != ctx->phys[i])
				valid = false;
			if (++count > 1) {
				page_vma_mapped_walk_done(&walk);
				break;
			}
		}
		KUNIT_EXPECT_TRUE(test, valid);
		KUNIT_EXPECT_EQ(test, count, i != 2 ? 1U : 0U);
#ifdef CONFIG_MEMORY_FAILURE
		page_folio(pfn_to_page(ctx->phys[i] >> PAGE_SHIFT))->index = i;
		KUNIT_EXPECT_EQ(test, page_mapped_in_vma(pfn_to_page(ctx->phys[i] >> PAGE_SHIFT), vma),
				i != 2 ? ctx->address[i] : (unsigned long)-EFAULT);
#endif
	}
}

static void user4k_vma_offsets_test(struct kunit *test)
{
	struct vm_area_struct *vma = kunit_kzalloc(test, sizeof(*vma), GFP_KERNEL);
	unsigned int slot, length, step;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	vma->vm_start = (1UL << 48) - 8 * PAGE_SIZE;
	for (slot = 0; slot < TEST_SLOTS; slot++) {
		for (length = 1; length <= 8; length++) {
			struct vm_page_offset initial = { ULONG_MAX - 3, slot * SZ_4K };
			struct vm_page_offset pos;

			vma->vm_end = vma->vm_start + length * SZ_4K;
			vma_set_page_offset(vma, initial);
			KUNIT_EXPECT_EQ(test, vma_last_pgoff(vma),
					initial.index + (slot + length - 1) / TEST_SLOTS);
			for (step = 0; step < length; step++) {
				unsigned long addr = vma->vm_start + step * SZ_4K;

				pos = vma_page_offset_at(vma, addr);
				KUNIT_EXPECT_EQ(test, pos.index, initial.index + (slot + step) / TEST_SLOTS);
				KUNIT_EXPECT_EQ(test, pos.offset, ((slot + step) % TEST_SLOTS) * (unsigned int)SZ_4K);
				KUNIT_EXPECT_EQ(test, linear_page_index(vma, addr), pos.index);
				KUNIT_EXPECT_EQ(test, vma_address_at_offset(vma, pos), addr);
			}
			pos = vma_page_offset_at(vma, vma->vm_end);
			KUNIT_EXPECT_EQ(test, vma_address_at_offset(vma, pos), (unsigned long)-EFAULT);
			pos = initial;
			pos.index--;
			KUNIT_EXPECT_EQ(test, vma_address_at_offset(vma, pos), (unsigned long)-EFAULT);
			if (slot) {
				pos = initial;
				pos.offset--;
				KUNIT_EXPECT_EQ(test, vma_address_at_offset(vma, pos), (unsigned long)-EFAULT);
			}
		}
	}
}

static void user4k_vma_intervals_test(struct kunit *test)
{
	struct vm_area_struct *vmas = kunit_kcalloc(test, 3, sizeof(*vmas), GFP_KERNEL);
	struct address_space *mapping = kunit_kzalloc(test, sizeof(*mapping), GFP_KERNEL);
	struct vm_area_struct *vma;
	unsigned int i, count = 0;

	KUNIT_ASSERT_NOT_NULL(test, vmas);
	KUNIT_ASSERT_NOT_NULL(test, mapping);
	mapping->i_mmap = RB_ROOT_CACHED;
	for (i = 0; i < 3; i++) {
		vmas[i].vm_start = TEST_VA + i * SZ_8K;
		vmas[i].vm_end = vmas[i].vm_start + SZ_4K;
		vma_set_page_offset(&vmas[i], (struct vm_page_offset) { 77, i * SZ_4K });
		mapping_rmap_tree_insert(&vmas[i], mapping);
	}
	mapping_rmap_tree_foreach(vma, mapping, 77, 77) {
		unsigned long address = vma_address_at_offset(vma,
						(struct vm_page_offset) { 77, SZ_4K });

		KUNIT_EXPECT_EQ(test, address,
				vma == &vmas[1] ? vma->vm_start : (unsigned long)-EFAULT);
		count++;
	}
	KUNIT_EXPECT_EQ(test, count, 3U);
	KUNIT_EXPECT_PTR_EQ(test, mapping_rmap_tree_iter_first(mapping, 76, 76), NULL);
	KUNIT_EXPECT_PTR_EQ(test, mapping_rmap_tree_iter_first(mapping, 78, 78), NULL);
	for (i = 0; i < 3; i++)
		mapping_rmap_tree_remove(&vmas[i], mapping);
	KUNIT_EXPECT_TRUE(test, RB_EMPTY_ROOT(&mapping->i_mmap.rb_root));
}

static void user4k_vma_mm_free(void *arg)
{
	mmput(arg);
}

/* These private mms have no concurrent VMA writers after dropping the lock. */
static struct vm_area_struct *test_vma_lookup(struct mm_struct *mm, unsigned long addr)
{
	struct vm_area_struct *vma;

	mmap_read_lock(mm);
	vma = vma_lookup(mm, addr);
	mmap_read_unlock(mm);
	return vma;
}

static void user4k_vma_split_test(struct kunit *test)
{
	struct mm_struct *mm = mm_alloc();
	struct vm_area_struct *vma, *middle, *part;
	unsigned long start = TEST_VA + SZ_4K;
	vma_flags_t flags;
	int ret;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, mm);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
	KUNIT_ASSERT_EQ(test, test_mm_select4k(mm), 0);
	vma = vm_area_alloc(mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	/* Dummy vm_ops keep insert_vm_struct from replacing this file-like offset. */
	vma->vm_start = start;
	vma->vm_end = start + 3 * SZ_4K;
	vm_flags_init(vma, VM_READ | VM_MAYREAD | VM_MAYWRITE);
	vma_set_page_offset(vma, (struct vm_page_offset) { 77, 3 * SZ_4K });
	mmap_write_lock(mm);
	ret = insert_vm_struct(mm, vma);
	if (ret) {
		mmap_write_unlock(mm);
		vm_area_free(vma);
		KUNIT_FAIL(test, "insert_vm_struct: %d", ret);
		return;
	}
	{
		VMA_ITERATOR(vmi, mm, start);

		vma_iter_load(&vmi);
		flags = vma->flags;
		vma_flags_set(&flags, VMA_WRITE_BIT);
		middle = vma_modify_flags(&vmi, vma, vma, start + SZ_4K,
					  start + 2 * SZ_4K, &flags);
	}
	mmap_write_unlock(mm);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, middle);
	KUNIT_EXPECT_EQ(test, mm->map_count, 3);
	for (i = 0; i < 3; i++) {
		part = test_vma_lookup(mm, start + i * SZ_4K);
		KUNIT_ASSERT_NOT_NULL(test, part);
		KUNIT_EXPECT_EQ(test, part->vm_start, start + i * SZ_4K);
		KUNIT_EXPECT_EQ(test, part->vm_end, start + (i + 1) * SZ_4K);
		KUNIT_EXPECT_EQ(test, part->vm_pgoff, 77UL + (i + 3) / TEST_SLOTS);
		KUNIT_EXPECT_EQ(test, vma_subpage_offset(part), ((i + 3) % TEST_SLOTS) * (unsigned int)SZ_4K);
	}
}

static struct mm_struct *test_vma_mm_granule(struct kunit *test, unsigned int shift)
{
	struct mm_struct *mm = mm_alloc();

	if (!mm)
		return NULL;
	if (kunit_add_action_or_reset(test, user4k_vma_mm_free, mm))
		return NULL;
	if (test_mm_select_granule(mm, shift))
		return NULL;
	return mm;
}

static struct mm_struct *test_vma_mm(struct kunit *test)
{
	return test_vma_mm_granule(test, 12);
}

static struct vm_area_struct *test_vma_add(struct mm_struct *mm, unsigned long start,
					 unsigned long len, struct vm_page_offset pos,
					 vm_flags_t flags)
{
	struct vm_area_struct *vma = vm_area_alloc(mm);
	int ret;

	if (!vma)
		return ERR_PTR(-ENOMEM);
	vma->vm_start = start;
	vma->vm_end = start + len;
	vm_flags_init(vma, flags | VM_MAYREAD | VM_MAYWRITE);
	vma_set_page_offset(vma, pos);
	mmap_write_lock(mm);
	ret = insert_vm_struct(mm, vma);
	mmap_write_unlock(mm);
	if (ret) {
		vm_area_free(vma);
		return ERR_PTR(ret);
	}
	return vma;
}

static void user4k_vma_merge_test(struct kunit *test)
{
	unsigned long start = TEST_VA + SZ_4K;
	unsigned int scenario, i;

	for (scenario = 0; scenario < 4; scenario++) {
		struct mm_struct *mm = test_vma_mm(test);
		struct vm_area_struct *vmas[3], *merged;
		struct vm_page_offset initial = { 77, 3 * SZ_4K };
		struct vm_page_offset expected;
		unsigned long left = scenario >= 2 ? SZ_4K : 0;
		unsigned long right = scenario == 1 || scenario == 3 ? 2 * SZ_4K : 3 * SZ_4K;
		vma_flags_t flags;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		for (i = 0; i < 3; i++) {
			struct vm_page_offset pos = vm_page_offset_add(initial, i * SZ_4K);
			vm_flags_t bits = VM_READ;

			if (i == 1)
				bits |= VM_WRITE;
			if ((!i && scenario >= 2) || (i == 2 && scenario == 1))
				bits |= VM_EXEC;
			/* Same native index, wrong byte offset: this must not merge. */
			if (i == 2 && scenario == 3)
				pos.offset = 0;
			vmas[i] = test_vma_add(mm, start + i * SZ_4K, SZ_4K, pos, bits);
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vmas[i]);
		}
		flags = vmas[1]->flags;
		vma_flags_clear(&flags, VMA_WRITE_BIT);
		mmap_write_lock(mm);
		{
			VMA_ITERATOR(vmi, mm, start + SZ_4K);

			vma_iter_load(&vmi);
			merged = vma_modify_flags(&vmi, vmas[0], vmas[1], start + SZ_4K,
						  start + 2 * SZ_4K, &flags);
			if (!IS_ERR(merged)) {
				vma_start_write(merged);
				merged->flags = flags;
			}
		}
		mmap_write_unlock(mm);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, merged);
		expected = vm_page_offset_add(initial, left);
		KUNIT_EXPECT_EQ(test, mm->map_count, scenario == 0 ? 1 : scenario == 3 ? 3 : 2);
		KUNIT_EXPECT_EQ(test, merged->vm_start, start + left);
		KUNIT_EXPECT_EQ(test, merged->vm_end, start + right);
		KUNIT_EXPECT_EQ(test, merged->vm_pgoff, expected.index);
		KUNIT_EXPECT_EQ(test, vma_subpage_offset(merged), expected.offset);
	}
}

static void user4k_vma_partial_merge_test(struct kunit *test)
{
	unsigned long start = TEST_VA;
	unsigned int side;

	for (side = 0; side < 2; side++) {
		struct mm_struct *mm = test_vma_mm(test);
		struct vm_page_offset initial = { 77, 2 * SZ_4K };
		struct vm_area_struct *first, *second, *merged, *remaining;
		unsigned long first_len = side ? SZ_4K : 2 * SZ_4K;
		vma_flags_t flags;
		struct vm_page_offset expected;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		first = test_vma_add(mm, start, first_len, initial, VM_READ | (side ? 0 : VM_WRITE));
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, first);
		second = test_vma_add(mm, start + first_len, 3 * SZ_4K - first_len,
				      vm_page_offset_add(initial, first_len), VM_READ | (side ? VM_WRITE : 0));
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, second);
		flags = side ? first->flags : second->flags;
		mmap_write_lock(mm);
		{
			VMA_ITERATOR(vmi, mm, start + SZ_4K);

			vma_iter_load(&vmi);
			merged = vma_modify_flags(&vmi, first, side ? second : first,
						  start + SZ_4K, start + 2 * SZ_4K, &flags);
		}
		mmap_write_unlock(mm);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, merged);
		KUNIT_EXPECT_EQ(test, mm->map_count, 2);
		KUNIT_EXPECT_EQ(test, merged->vm_start, start + (side ? 0 : SZ_4K));
		KUNIT_EXPECT_EQ(test, merged->vm_end, start + (side ? 2 : 3) * SZ_4K);
		expected = vm_page_offset_add(initial, side ? 0 : SZ_4K);
		KUNIT_EXPECT_EQ(test, merged->vm_pgoff, expected.index);
		KUNIT_EXPECT_EQ(test, vma_subpage_offset(merged), expected.offset);
		remaining = test_vma_lookup(mm, start + (side ? 2 * SZ_4K : 0));
		KUNIT_ASSERT_NOT_NULL(test, remaining);
		expected = vm_page_offset_add(initial, side ? 2 * SZ_4K : 0);
		KUNIT_EXPECT_EQ(test, remaining->vm_pgoff, expected.index);
		KUNIT_EXPECT_EQ(test, vma_subpage_offset(remaining), expected.offset);
	}
}

static void user4k_vma_copy_shrink_test(struct kunit *test)
{
	struct mm_struct *mm = test_vma_mm(test);
	struct vm_area_struct *source, *dest;
	struct vm_page_offset initial = { 77, 3 * SZ_4K };
	unsigned long start = TEST_VA + SZ_4K;
	unsigned long target = TEST_VA + 16 * PAGE_SIZE + SZ_4K;
	bool need_locks = false;
	unsigned int i;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, mm);
	source = test_vma_add(mm, start, PAGE_SIZE, initial, VM_READ);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, source);
	for (i = 0; i < 3; i++) {
		unsigned long addr = i == 2 ? target - SZ_4K : target + i * SZ_4K;
		struct vm_page_offset pos = vm_page_offset_add(initial, i == 2 ? 0 : (i + 1) * SZ_4K);

		mmap_write_lock(mm);
		pgoff_t anon_pgoff = __linear_anon_page_index(source,
					start + (i == 2 ? 0 : (i + 1) * SZ_4K));

		dest = copy_vma(&source, addr, SZ_4K, pos, anon_pgoff, &need_locks, false);
		mmap_write_unlock(mm);
		KUNIT_ASSERT_NOT_NULL(test, dest);
		KUNIT_EXPECT_EQ(test, mm->map_count, 2);
		KUNIT_EXPECT_EQ(test, dest->vm_start, i == 2 ? target - SZ_4K : target);
		KUNIT_EXPECT_EQ(test, dest->vm_end, i ? target + 2 * SZ_4K : target + SZ_4K);
		KUNIT_EXPECT_EQ(test, dest->vm_pgoff, 77UL + (i == 2 ? 3UL : 4UL) / TEST_SLOTS);
		KUNIT_EXPECT_EQ(test, vma_subpage_offset(dest), ((i == 2 ? 3U : 4U) % TEST_SLOTS) * SZ_4K);
		KUNIT_EXPECT_EQ(test, source->vm_pgoff, initial.index);
		KUNIT_EXPECT_EQ(test, vma_subpage_offset(source), initial.offset);
	}
	mmap_write_lock(mm);
	{
		VMA_ITERATOR(vmi, mm, dest->vm_start);

		/* Since Linux 7.3 vma_shrink() only moves the end of a VMA. */
		vma_iter_load(&vmi);
		ret = vma_shrink(&vmi, dest, target + SZ_4K);
	}
	mmap_write_unlock(mm);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, dest->vm_start, target - SZ_4K);
	KUNIT_EXPECT_EQ(test, dest->vm_end, target + SZ_4K);
	KUNIT_EXPECT_EQ(test, dest->vm_pgoff, 77UL + 3 / TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, vma_subpage_offset(dest), (3 % TEST_SLOTS) * SZ_4K);
}

struct anon_slot_fixture {
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	struct mm_subpage_pool *pool;
	struct mm_subpage *slots[4];
	bool mapped[4];
};

static void anon_slot_fixture_free(void *arg)
{
	struct anon_slot_fixture *f = arg;
	struct mmu_gather tlb;
	unsigned int i;

	if (f->pool)
		mm_subpage_pool_close(f->pool);
	if (f->mm) {
		mmap_write_lock(f->mm);
		for (i = 0; i < 4; i++) {
			unsigned long addr = TEST_VA + i * SZ_4K;
			spinlock_t *ptl;
			pte_t *pte = test_mm_lookup(f->mm, addr, &ptl);

			if (!pte)
				continue;
			ptep_get_and_clear_full(f->mm, addr, pte, 0);
			if (f->mapped[i]) {
				struct vm_area_struct *vma = vma_lookup(f->mm, addr);

				mm_subpage_remove_anon_rmap(f->slots[i], vma);
				dec_mm_counter(f->mm, MM_ANONPAGES);
				f->mapped[i] = false;
			}
			pte_unmap_unlock(pte, ptl);
		}
		tlb_gather_mmu_fullmm(&tlb, f->mm);
		free_pgd_range(&tlb, 0, 1UL << 48, 0, 0);
		tlb_finish_mmu(&tlb);
		mmap_write_unlock(f->mm);
	}
	/* No slot reference is released before the PTEs and TLBs are retired. */
	for (i = 0; i < 4; i++)
		if (f->slots[i])
			mm_subpage_put(f->slots[i]);
	if (f->pool)
		mm_subpage_pool_put(f->pool);
	if (f->mm)
		mmput(f->mm);
	memset(f, 0, sizeof(*f));
}

static struct anon_slot_fixture *anon_slot_fixture_create_from(struct kunit *test,
							     struct anon_slot_fixture *parent)
{
	struct anon_slot_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	struct vm_area_struct *vma;
	struct folio *folio;
	int ret;

	if (!f || kunit_add_action_or_reset(test, anon_slot_fixture_free, f))
		return NULL;
	f->mm = mm_alloc();
	if (!f->mm || test_mm_select4k(f->mm))
		return NULL;
	f->pool = mm_subpage_pool_create(GFP_KERNEL);
	if (!f->pool)
		return NULL;
	folio = folio_alloc(GFP_KERNEL, 0);
	if (!folio)
		return NULL;
	ret = mm_subpage_pool_add_folio(f->pool, folio, GFP_KERNEL);
	if (ret) {
		folio_put(folio);
		return NULL;
	}
	vma = vm_area_alloc(f->mm);
	if (!vma)
		return NULL;
	vma_set_anonymous(vma);
	vma->vm_start = TEST_VA;
	vma->vm_end = TEST_VA + 2 * PAGE_SIZE;
	vm_flags_init(vma, VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE);
	vma_set_page_prot(vma);
	if (parent) {
		mmap_write_lock(parent->mm);
		mmap_write_lock_nested(f->mm, SINGLE_DEPTH_NESTING);
		vma_set_page_offset(vma, vma_page_offset_at(parent->vma, TEST_VA));
		/* As vm_area_init_from(): forked anon folios keep their index. */
		__vma_set_anon_pgoff(vma, vma_start_anon_pgoff(parent->vma));
		ret = anon_vma_fork(vma, parent->vma);
	} else {
		mmap_write_lock(f->mm);
		ret = 0;
	}
	if (!ret && parent) {
		VMA_ITERATOR(vmi, f->mm, TEST_VA);

		vma_start_write(vma);
		ret = vma_iter_store_gfp(&vmi, vma, GFP_KERNEL);
		if (!ret)
			f->mm->map_count++;
	} else if (!ret) {
		ret = insert_vm_struct(f->mm, vma);
	}
	if (!ret) {
		f->vma = vma;
		ret = anon_vma_prepare(vma);
	}
	if (!f->vma)
		unlink_anon_vmas(vma);
	mmap_write_unlock(f->mm);
	if (parent)
		mmap_write_unlock(parent->mm);
	if (!f->vma)
		vm_area_free(vma);
	return ret ? NULL : f;
}

static struct anon_slot_fixture *anon_slot_fixture_create(struct kunit *test)
{
	return anon_slot_fixture_create_from(test, NULL);
}

static void user4k_anon_rmap_test(struct kunit *test)
{
	struct anon_slot_fixture *f = anon_slot_fixture_create(test);
	struct folio *folio = NULL;
	unsigned int i;
	vma_flags_t flags;
	int ret, bad_offset = 0, bad_index = 0;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (i = 0; i < 4; i++) {
		struct mm_subpage *slot = mm_subpage_alloc_at(f->pool, i * SZ_4K);
		unsigned long addr = TEST_VA + i * SZ_4K;
		struct test_mm tables = { .mm = f->mm };
		spinlock_t *ptl;
		pte_t *pte;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
		f->slots[i] = slot;
		folio = mm_subpage_folio(slot);
		/* Allocate the table first; clear the test leaf before rmap attachment. */
		KUNIT_ASSERT_EQ(test, test_map_alias(&tables, 0, addr, mm_subpage_phys(slot)), 0);
		mmap_read_lock(f->mm);
		folio_lock(folio);
		pte = test_mm_lookup(f->mm, addr, &ptl);
		if (!pte) {
			folio_unlock(folio);
			mmap_read_unlock(f->mm);
			KUNIT_FAIL(test, "missing allocated PTE");
			return;
		}
		pte_clear(f->mm, addr, pte);
		if (!i)
			bad_offset = mm_subpage_add_anon_rmap(slot, f->vma, addr + 1);
		ret = mm_subpage_add_anon_rmap(slot, f->vma, addr);
		if (!ret) {
			f->mapped[i] = true;
			inc_mm_counter(f->mm, MM_ANONPAGES);
			set_pte_at(f->mm, addr, pte, phys_pte_mm(f->mm, mm_subpage_phys(slot),
						__pgprot(TEST_PROT | PTE_RDONLY)));
			if (!i)
				bad_index = mm_subpage_add_anon_rmap(slot, f->vma, addr + PAGE_SIZE);
		}
		pte_unmap_unlock(pte, ptl);
		folio_unlock(folio);
		mmap_read_unlock(f->mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 1);
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)i + 1);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)i + 1);
	}
	KUNIT_EXPECT_EQ(test, bad_offset, -EINVAL);
	KUNIT_EXPECT_EQ(test, bad_index, -EINVAL);
	KUNIT_EXPECT_EQ(test, folio->index, TEST_VA >> PAGE_SHIFT);
	KUNIT_EXPECT_FALSE(test, PageAnonExclusive(&folio->page));
	KUNIT_EXPECT_EQ(test, folio_expected_ref_count(folio), folio_ref_count(folio));
	folio_lock(folio);
	ret = folio_referenced(folio, 1, NULL, &flags);
	folio_unlock(folio);
	KUNIT_EXPECT_EQ(test, ret, 1);
	for (i = 0; i < 4; i++) {
		spinlock_t *ptl;
		pte_t *pte = test_mm_lookup(f->mm, TEST_VA + i * SZ_4K, &ptl);
		bool young;

		KUNIT_ASSERT_NOT_NULL(test, pte);
		young = pte_young(ptep_get(pte));
		pte_unmap_unlock(pte, ptl);
		KUNIT_EXPECT_FALSE(test, young);
	}
	folio_lock(folio);
	ret = folio_referenced(folio, 1, NULL, &flags);
	folio_unlock(folio);
	KUNIT_EXPECT_EQ(test, ret, 0);
}

/* The fixture takes one existing slot reference, including on failure. */
static int anon_slot_fixture_map(struct anon_slot_fixture *f,
				 unsigned int index, struct mm_subpage *slot)
{
	unsigned long addr = TEST_VA + index * SZ_4K;
	struct test_mm tables = { .mm = f->mm };
	struct folio *folio = mm_subpage_folio(slot);
	spinlock_t *ptl;
	pte_t *pte;
	int ret;

	f->slots[index] = slot;
	ret = test_map_alias(&tables, 0, addr, mm_subpage_phys(slot));
	if (ret)
		return ret;
	mmap_read_lock(f->mm);
	folio_lock(folio);
	pte = test_mm_lookup(f->mm, addr, &ptl);
	if (!pte) {
		ret = -ENOMEM;
		goto unlock;
	}
	pte_clear(f->mm, addr, pte);
	ret = mm_subpage_add_anon_rmap(slot, f->vma, addr);
	if (!ret) {
		f->mapped[index] = true;
		inc_mm_counter(f->mm, MM_ANONPAGES);
		set_pte_at(f->mm, addr, pte, phys_pte_mm(f->mm, mm_subpage_phys(slot),
					__pgprot(TEST_PROT | PTE_RDONLY)));
	}
	pte_unmap_unlock(pte, ptl);
unlock:
	folio_unlock(folio);
	mmap_read_unlock(f->mm);
	return ret;
}

static void user4k_anon_fork_rmap_test(struct kunit *test)
{
	struct anon_slot_fixture *parent = anon_slot_fixture_create(test);
	struct anon_slot_fixture *child;
	struct mm_subpage *slot;
	struct folio *folio;
	vma_flags_t flags;
	int ret;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, parent);
	child = anon_slot_fixture_create_from(test, parent);
	KUNIT_ASSERT_NOT_NULL(test, child);
	for (i = 0; i < 4; i++) {
		slot = mm_subpage_alloc_at(parent->pool, i * SZ_4K);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(parent, i, slot), 0);
		mm_subpage_get(slot);
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(child, i, slot), 0);
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 2);
	}
	folio = mm_subpage_folio(slot);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 8);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 8);
	folio_lock(folio);
	ret = folio_referenced(folio, 1, NULL, &flags);
	folio_unlock(folio);
	KUNIT_EXPECT_EQ(test, ret, 2);
	anon_slot_fixture_free(parent);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 4);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 4);
	for (i = 0; i < 4; i++)
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(child->slots[i]), 1);
	/* The child mappings and root remain valid after the parent disappears. */
	folio_lock(folio);
	ret = folio_referenced(folio, 1, NULL, &flags);
	folio_unlock(folio);
	KUNIT_EXPECT_EQ(test, ret, 0);
}

static void user4k_nonlinear_rmap_reuse_test(struct kunit *test)
{
	struct anon_slot_fixture *a = anon_slot_fixture_create(test);
	struct anon_slot_fixture *b = anon_slot_fixture_create(test);
	struct anon_slot_fixture *c = anon_slot_fixture_create(test);
	struct mm_subpage *slot;
	struct folio *folio;
	struct anon_vma_chain *avc;
	phys_addr_t phys;
	vma_flags_t flags;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, a);
	KUNIT_ASSERT_NOT_NULL(test, b);
	KUNIT_ASSERT_NOT_NULL(test, c);
	slot = mm_subpage_alloc_at(a->pool, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
	KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(a, 0, slot), 0);
	folio = mm_subpage_folio(slot);
	slot = mm_subpage_alloc_at(a->pool, 3 * SZ_4K);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
	phys = mm_subpage_phys(slot);
#ifdef CONFIG_FAILSLAB
	{
		unsigned int pending;

		mmap_read_lock(b->mm);
		folio_lock(folio);
		WRITE_ONCE(current->fail_nth, 1);
		ret = mm_subpage_add_anon_rmap(slot, b->vma, TEST_VA + SZ_4K);
		pending = READ_ONCE(current->fail_nth);
		WRITE_ONCE(current->fail_nth, 0);
		/* Keep cleanup valid even if allocation unexpectedly succeeds. */
		if (!ret)
			mm_subpage_remove_anon_rmap(slot, b->vma);
		folio_unlock(folio);
		mmap_read_unlock(b->mm);
		KUNIT_EXPECT_EQ(test, pending, 0U);
		KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 0);
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 1);
	}
#endif
	/* Allocation failure left the slot available for the same operation. */
	KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(b, 1, slot), 0);
	anon_slot_fixture_free(b);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 1);
	slot = mm_subpage_alloc_at(a->pool, 3 * SZ_4K);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
	KUNIT_EXPECT_EQ(test, mm_subpage_phys(slot), phys);

	/* Give this empty VMA a different logical native index as well. */
	mmap_write_lock(c->mm);
	anon_vma_lock_write(c->vma->anon_vma);
	list_for_each_entry(avc, &c->vma->anon_vma_chain, same_vma)
		anon_rmap_tree_remove(avc, avc->anon_vma);
	/* Linux 7.3 keys the anon_vma tree by the anonymous page offset. */
	c->vma->vm_pgoff += 9;
	c->vma->__vm_anon_pgoff_lo += 9;
	list_for_each_entry(avc, &c->vma->anon_vma_chain, same_vma)
		anon_rmap_tree_insert(avc, avc->anon_vma);
	anon_vma_unlock_write(c->vma->anon_vma);
	mmap_write_unlock(c->mm);
#ifdef CONFIG_FAILSLAB
	/* Retiring a previous root also needs failure-atomic bookkeeping. */
	mmap_read_lock(c->mm);
	folio_lock(folio);
	WRITE_ONCE(current->fail_nth, 1);
	ret = mm_subpage_add_anon_rmap(slot, c->vma, TEST_VA + 2 * SZ_4K);
	WRITE_ONCE(current->fail_nth, 0);
	if (!ret)
		mm_subpage_remove_anon_rmap(slot, c->vma);
	folio_unlock(folio);
	mmap_read_unlock(c->mm);
	KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
	KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 0);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 1);
#endif
	KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(c, 2, slot), 0);
	mmap_read_lock(c->mm);
	KUNIT_EXPECT_EQ(test, page_address_in_vma(folio, &folio->page, c->vma),
			TEST_VA + 2 * SZ_4K);
	mmap_read_unlock(c->mm);
	folio_lock(folio);
	ret = folio_referenced(folio, 1, NULL, &flags);
	folio_unlock(folio);
	KUNIT_EXPECT_EQ(test, ret, 2);
	/* The original root may disappear before the other slot's last mapping. */
	anon_slot_fixture_free(a);
	folio_lock(folio);
	ret = folio_referenced(folio, 1, NULL, &flags);
	folio_unlock(folio);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(c->slots[2]), 1);
}

static void user4k_dup_mmap_test(struct kunit *test)
{
	struct anon_slot_fixture *parent = anon_slot_fixture_create(test);
	struct mm_struct *child;
	struct folio *folio;
	unsigned int i;
	int ret = 0;

	KUNIT_ASSERT_NOT_NULL(test, parent);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, child);
	for (i = 0; i < 4; i++) {
		struct mm_subpage *slot = mm_subpage_alloc_at(parent->pool, i * SZ_4K);
		unsigned long addr = TEST_VA + i * SZ_4K;
		spinlock_t *ptl;
		pte_t *pte;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(parent, i, slot), 0);
		pte = test_mm_lookup(parent->mm, addr, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, pte);
		set_pte_at(parent->mm, addr, pte,
			   pte_mkwrite_novma(pte_mkdirty(ptep_get(pte))));
		pte_unmap_unlock(pte, ptl);
	}
	/* Split into four 4K VMAs plus a trailing hole, preserving COW flags. */
	mmap_write_lock(parent->mm);
	for (i = 1; i < 4; i += 2) {
		unsigned long addr = TEST_VA + i * SZ_4K;
		VMA_ITERATOR(vmi, parent->mm, addr);
		struct vm_area_struct *vma = vma_iter_load(&vmi);
		vma_flags_t flags = vma->flags;

		vma_flags_set(&flags, VMA_DONTDUMP_BIT);
		vma = vma_modify_flags(&vmi, vma, vma, addr, addr + SZ_4K, &flags);
		if (IS_ERR(vma)) {
			ret = PTR_ERR(vma);
			break;
		}
	}
	mmap_write_unlock(parent->mm);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, parent->mm->map_count, 5);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent->mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, child->map_count, 5);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_ANONPAGES), 4UL);
	folio = mm_subpage_folio(parent->slots[0]);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 8);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 8);
	for (i = 0; i < 4; i++) {
		unsigned long addr = TEST_VA + i * SZ_4K;
		struct vm_area_struct *vma = test_vma_lookup(child, addr);
		unsigned int side;

		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_EXPECT_EQ(test, vma_subpage_offset(vma), i * (unsigned int)SZ_4K);
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(parent->slots[i]), 2);
		for (side = 0; side < 2; side++) {
			struct mm_struct *mm = side ? child : parent->mm;
			spinlock_t *ptl;
			pte_t *pte = test_mm_lookup(mm, addr, &ptl);
			pte_t entry;

			KUNIT_ASSERT_NOT_NULL(test, pte);
			entry = ptep_get(pte);
			pte_unmap_unlock(pte, ptl);
			KUNIT_EXPECT_TRUE(test, pte_present(entry));
			KUNIT_EXPECT_FALSE(test, pte_write(entry));
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, entry),
					mm_subpage_phys(parent->slots[i]));
		}
	}
	anon_slot_fixture_free(parent);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 4);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 4);
	/* KUnit's child-mm cleanup now exercises ordinary exit_mmap(). */
}

static void test_subpage_put(void *arg)
{
	mm_subpage_put(arg);
}

static void user4k_anon_root_lifetime_test(struct kunit *test)
{
	struct anon_slot_fixture *old = anon_slot_fixture_create(test);
	struct anon_slot_fixture *other = anon_slot_fixture_create(test);
	struct mm_subpage *slot;
	struct anon_vma *root;
	struct folio *folio;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, old);
	KUNIT_ASSERT_NOT_NULL(test, other);
	slot = mm_subpage_alloc_at(old->pool, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
	KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(old, 0, slot), 0);
	folio = mm_subpage_folio(slot);
	root = old->vma->anon_vma->root;
	mm_subpage_get(slot);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_subpage_put, slot), 0);
	anon_slot_fixture_free(old);
	/* Only the owner now pins the root; no VMA or PTE survives. */
	KUNIT_EXPECT_EQ(test, atomic_read(&root->refcount), 1);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
	KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 0);
	mm_subpage_get(slot);
	ret = anon_slot_fixture_map(other, 0, slot);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&root->refcount), 1);
}

static void test_folio_put(void *arg)
{
	folio_put(arg);
}

static void user4k_data_tlb_batch_test(struct kunit *test)
{
	struct anon_slot_fixture *f = anon_slot_fixture_create(test);
	struct folio *native = folio_alloc(GFP_KERNEL, 2);
	struct mmu_gather tlb;
	struct folio *backing;
	unsigned int i;
	bool full = false;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, native), 0);
	for (i = 0; i < 4; i++) {
		f->slots[i] = mm_subpage_alloc_at(f->pool, i * SZ_4K);
		if (IS_ERR(f->slots[i])) {
			f->slots[i] = NULL;
			KUNIT_FAIL(test, "slot allocation failed");
			return;
		}
	}
	backing = mm_subpage_folio(f->slots[0]);
	memset(folio_address(backing), 0x5a, SZ_4K);
	tlb_gather_mmu(&tlb, f->mm);
	tlb_change_page_size(&tlb, MM_SUBPAGE_SIZE);
	tlb_flush_pte_range(&tlb, TEST_VA, PAGE_SIZE);
	for (i = 0; i < 64; i++) {
		mm_subpage_get(f->slots[i % 4]);
		if (__tlb_remove_subpage(&tlb, f->slots[i % 4])) {
			full = true;
			tlb_flush_mmu(&tlb);
			tlb_flush_pte_range(&tlb, TEST_VA, PAGE_SIZE);
		}
		if (i == 12) {
			/* Exercise the native count payload (3 << 2 has tag bit 2). */
			folio_ref_add(native, 3);
			if (__tlb_remove_folio_pages(&tlb, &native->page, 3, false)) {
				full = true;
				tlb_flush_mmu(&tlb);
				tlb_flush_pte_range(&tlb, TEST_VA, PAGE_SIZE);
			}
		}
	}
	mm_subpage_put(f->slots[0]);
	f->slots[0] = NULL;
	/* Without memory pressure these fit in the local plus one dynamic batch. */
	if (!full) {
		struct mm_subpage *extra = mm_subpage_alloc_at(f->pool, 0);

		KUNIT_EXPECT_EQ(test, PTR_ERR(extra), -EAGAIN);
		if (!IS_ERR(extra))
			mm_subpage_put(extra);
		KUNIT_EXPECT_EQ(test, folio_ref_count(backing), 67);
		KUNIT_EXPECT_EQ(test, folio_ref_count(native), 4);
		KUNIT_EXPECT_GT(test, tlb.batch_count, 0U);
	}
	tlb_finish_mmu(&tlb);
	KUNIT_EXPECT_EQ(test, folio_ref_count(backing), 3);
	KUNIT_EXPECT_EQ(test, folio_ref_count(native), 1);
	f->slots[0] = mm_subpage_alloc_at(f->pool, 0);
	if (IS_ERR(f->slots[0])) {
		f->slots[0] = NULL;
		KUNIT_FAIL(test, "slot was not reusable after TLB completion");
		return;
	}
	KUNIT_EXPECT_PTR_EQ(test, mm_subpage_folio(f->slots[0]), backing);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(backing), 0, SZ_4K), NULL);
	for (i = 0; i < 4; i++) {
		struct mm_subpage *extra = mm_subpage_alloc_at(f->pool, i * SZ_4K);

		KUNIT_EXPECT_EQ(test, PTR_ERR(extra), -EAGAIN);
		if (!IS_ERR(extra))
			mm_subpage_put(extra);
	}
}

static void user4k_anon_zap_test(struct kunit *test)
{
	struct anon_slot_fixture *f = anon_slot_fixture_create(test);
	struct mm_subpage *slots[4];
	struct folio *folio;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (i = 0; i < 4; i++) {
		slots[i] = mm_subpage_alloc_at(f->pool, i * SZ_4K);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slots[i]);
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(f, i, slots[i]), 0);
		mm_subpage_get(slots[i]);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_subpage_put, slots[i]), 0);
	}
	folio = mm_subpage_folio(slots[0]);
	mmap_read_lock(f->mm);
	zap_vma_range(f->vma, TEST_VA + SZ_4K, SZ_4K);
	f->mapped[1] = false;
	f->slots[1] = NULL;
	mmap_read_unlock(f->mm);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 3);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 7);
	for (i = 0; i < 4; i++) {
		spinlock_t *ptl;
		pte_t *pte = test_mm_lookup(f->mm, TEST_VA + i * SZ_4K, &ptl);
		bool present;

		KUNIT_ASSERT_NOT_NULL(test, pte);
		present = pte_present(ptep_get(pte));
		pte_unmap_unlock(pte, ptl);
		KUNIT_EXPECT_EQ(test, present, i != 1);
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slots[i]), i == 1 ? 0 : 1);
	}
	/* Include both the existing hole and a trailing run of empty entries. */
	mmap_read_lock(f->mm);
	zap_vma(f->vma);
	for (i = 0; i < 4; i++) {
		f->mapped[i] = false;
		f->slots[i] = NULL;
	}
	mmap_read_unlock(f->mm);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 4);
	for (i = 0; i < 4; i++)
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slots[i]), 0);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(f->mm, MM_ANONPAGES), 0UL);
}

static void user4k_anon_writable_slots_test(struct kunit *test)
{
	struct anon_slot_fixture *f = anon_slot_fixture_create(test);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (i = 0; i < 4; i++) {
		struct mm_subpage *slot = mm_subpage_alloc_at(f->pool, i * SZ_4K);
		unsigned long addr = TEST_VA + i * SZ_4K;
		spinlock_t *ptl;
		pte_t *pte;
		bool writable;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(f, i, slot), 0);
		pte = test_mm_lookup(f->mm, addr, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, pte);
		set_pte_at(f->mm, addr, pte,
			   pte_mkwrite_novma(pte_mkdirty(ptep_get(pte))));
		writable = pte_write(ptep_get(pte));
		pte_unmap_unlock(pte, ptl);
		KUNIT_EXPECT_TRUE(test, writable);
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 1);
	}
	KUNIT_EXPECT_EQ(test, folio_mapcount(mm_subpage_folio(f->slots[0])), 4);
}

static struct vm_area_struct *test_fault_vma_in_mm(struct mm_struct *mm,
					    unsigned long start, unsigned long len)
{
	struct vm_area_struct *vma;
	vm_flags_t flags = VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE;
	int ret;

	if (!mm)
		return NULL;
	vma = vm_area_alloc(mm);
	if (!vma)
		return NULL;
	vma_set_anonymous(vma);
	vma->vm_start = start;
	vma->vm_end = start + len;
	vm_flags_init(vma, flags);
	vma->vm_page_prot = vm_get_page_prot(flags);
	mmap_write_lock(mm);
	ret = insert_vm_struct(mm, vma);
	mmap_write_unlock(mm);
	if (ret) {
		vm_area_free(vma);
		return NULL;
	}
	return vma;
}

struct coarse_anon_fixture {
	struct mm_struct *mm[2];
	bool attached[2];
	struct vm_area_struct *vma[2];
	struct folio *folio;
	unsigned long mapped[2];
	unsigned int leaves;
	bool pinned;
};

static void coarse_anon_fixture_free(void *ptr)
{
	struct coarse_anon_fixture *f = ptr;
	unsigned int i, nr = SZ_64K / PAGE_SIZE;

	if (f->pinned) {
		unpin_user_page(folio_page(f->folio, nr - 1));
		f->pinned = false;
	}
	for (i = 0; i < 2; i++) {
		unsigned int leaf;

		if (!f->mm[i])
			continue;
		mmap_write_lock(f->mm[i]);
		for (leaf = 0; leaf < (f->leaves ?: 1); leaf++) {
			unsigned long addr = TEST_VA + leaf * SZ_64K;
			spinlock_t *ptl;
			pte_t *pte = test_mm_lookup(f->mm[i], addr, &ptl);

			if (pte) {
				ptep_get_and_clear_full(f->mm[i], addr, pte, 0);
				if (f->mapped[i] & BIT(leaf)) {
					folio_remove_rmap_pte(f->folio,
						folio_page(f->folio, leaf * nr), f->vma[i]);
					dec_mm_counter(f->mm[i], MM_ANONPAGES);
				}
				pte_unmap_unlock(pte, ptl);
			}
			if (f->mapped[i] & BIT(leaf)) {
				folio_put_refs(f->folio, nr);
				f->mapped[i] &= ~BIT(leaf);
			}
		}
		mmap_write_unlock(f->mm[i]);
		if (f->vma[i] && !f->attached[i]) {
			unlink_anon_vmas(f->vma[i]);
			vm_area_free(f->vma[i]);
		}
		mmput(f->mm[i]);
		f->vma[i] = NULL;
		f->mm[i] = NULL;
	}
	if (f->folio) {
		folio_put(f->folio);
		f->folio = NULL;
	}
}

static void expect_coarse_anon_state(struct kunit *test, struct folio *folio,
				    unsigned int maps, bool exclusive)
{
	unsigned int i, nr = folio_nr_pages(folio);

	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)(maps * nr));
	for (i = 0; i < nr; i++) {
		struct page *page = folio_page(folio, i);

		if (IS_ENABLED(CONFIG_PAGE_MAPCOUNT))
			KUNIT_EXPECT_EQ(test, (atomic_read(&page->_mapcount) + 1), (int)maps);
		KUNIT_EXPECT_EQ(test, PageAnonExclusive(page), exclusive);
	}
}

static void user64k_anon_rmap_span_test(struct kunit *test)
{
	struct coarse_anon_fixture *f;
	unsigned int i, nr = SZ_64K / PAGE_SIZE;
	pte_t entry, *pte;
	spinlock_t *ptl;
	int ret, dup_ret, share_ret;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires coarse anonymous leaf");
		return;
	}
	f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, coarse_anon_fixture_free, f), 0);
	f->folio = folio_alloc(GFP_KERNEL | __GFP_ZERO, 16 - PAGE_SHIFT);
	KUNIT_ASSERT_NOT_NULL(test, f->folio);
	for (i = 0; i < 2; i++) {
		struct test_mm tables = {};

		f->mm[i] = mm_alloc();
		KUNIT_ASSERT_NOT_NULL(test, f->mm[i]);
		KUNIT_ASSERT_EQ(test, test_mm_select_granule(f->mm[i], 16), 0);
		if (!i) {
			f->vma[i] = test_fault_vma_in_mm(f->mm[i], TEST_VA, SZ_64K);
			f->attached[i] = !!f->vma[i];
		} else {
			f->vma[i] = vm_area_alloc(f->mm[i]);
			if (f->vma[i]) {
				vma_set_anonymous(f->vma[i]);
				f->vma[i]->vm_start = TEST_VA;
				f->vma[i]->vm_end = TEST_VA + SZ_64K;
				vm_flags_init(f->vma[i], f->vma[0]->vm_flags);
				f->vma[i]->vm_page_prot = f->vma[0]->vm_page_prot;
				vma_set_page_offset(f->vma[i], vma_page_offset_at(f->vma[0], TEST_VA));
			}
		}
		KUNIT_ASSERT_NOT_NULL(test, f->vma[i]);
		tables.mm = f->mm[i];
		KUNIT_ASSERT_EQ(test, test_map_alias(&tables, 0, TEST_VA, PFN_PHYS(folio_pfn(f->folio))), 0);
		pte = test_mm_lookup(f->mm[i], TEST_VA, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, pte);
		pte_clear(f->mm[i], TEST_VA, pte);
		pte_unmap_unlock(pte, ptl);
	}
	mmap_write_lock(f->mm[0]);
	ret = anon_vma_prepare(f->vma[0]);
	mmap_write_unlock(f->mm[0]);
	KUNIT_ASSERT_EQ(test, ret, 0);
	mmap_write_lock(f->mm[0]);
	mmap_write_lock_nested(f->mm[1], SINGLE_DEPTH_NESTING);
	ret = anon_vma_fork(f->vma[1], f->vma[0]);
	if (!ret) {
		VMA_ITERATOR(vmi, f->mm[1], TEST_VA);

		vma_start_write(f->vma[1]);
		ret = vma_iter_store_gfp(&vmi, f->vma[1], GFP_KERNEL);
		if (!ret) {
			f->mm[1]->map_count++;
			f->attached[1] = true;
		}
	}
	mmap_write_unlock(f->mm[1]);
	mmap_write_unlock(f->mm[0]);
	KUNIT_ASSERT_EQ(test, ret, 0);

	pte = test_mm_lookup(f->mm[0], TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	folio_add_new_anon_rmap(f->folio, f->vma[0], TEST_VA, RMAP_EXCLUSIVE);
	folio_ref_add(f->folio, nr); /* Fixture retains its original allocation ref. */
	inc_mm_counter(f->mm[0], MM_ANONPAGES);
	f->mapped[0] = true;
	entry = pte_mkwrite(pte_mkdirty(phys_pte_mm(f->mm[0], PFN_PHYS(folio_pfn(f->folio)),
						 f->vma[0]->vm_page_prot)), f->vma[0]);
	set_ptes(f->mm[0], TEST_VA, pte, entry, 1);
	pte_unmap_unlock(pte, ptl);
	expect_coarse_anon_state(test, f->folio, 1, true);
	KUNIT_EXPECT_EQ(test, folio_ref_count(f->folio), (int)nr + 1);

	mm_flags_set(MMF_HAS_PINNED, f->mm[0]);
	ret = try_grab_folio(page_folio(folio_page(f->folio, nr - 1)), 1, FOLL_PIN);
	f->pinned = !ret;
	KUNIT_ASSERT_EQ(test, ret, 0);
	pte = test_mm_lookup(f->mm[0], TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	ptep_set_wrprotect(f->mm[0], TEST_VA, pte);
	raw_write_seqcount_begin(&f->mm[0]->write_protect_seq);
	dup_ret = folio_try_dup_anon_rmap_pte(f->folio, &f->folio->page, f->vma[1], f->vma[0]);
	raw_write_seqcount_end(&f->mm[0]->write_protect_seq);
	entry = ptep_get_and_clear(f->mm[0], TEST_VA, pte);
	share_ret = folio_try_share_anon_rmap_pte(f->folio, &f->folio->page, f->vma[0]);
	set_ptes(f->mm[0], TEST_VA, pte, entry, 1);
	pte_unmap_unlock(pte, ptl);
	KUNIT_EXPECT_EQ(test, dup_ret, -EBUSY);
	KUNIT_EXPECT_EQ(test, share_ret, -EBUSY);
	expect_coarse_anon_state(test, f->folio, 1, true);
	unpin_user_page(folio_page(f->folio, nr - 1));
	f->pinned = false;

	pte = test_mm_lookup(f->mm[0], TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	entry = ptep_get_and_clear(f->mm[0], TEST_VA, pte);
	share_ret = folio_try_share_anon_rmap_pte(f->folio, &f->folio->page, f->vma[0]);
	set_ptes(f->mm[0], TEST_VA, pte, pte_wrprotect(entry), 1);
	pte_unmap_unlock(pte, ptl);
	KUNIT_ASSERT_EQ(test, share_ret, 0);
	expect_coarse_anon_state(test, f->folio, 1, false);

	pte = test_mm_lookup(f->mm[0], TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	raw_write_seqcount_begin(&f->mm[0]->write_protect_seq);
	dup_ret = folio_try_dup_anon_rmap_pte(f->folio, &f->folio->page, f->vma[1], f->vma[0]);
	raw_write_seqcount_end(&f->mm[0]->write_protect_seq);
	if (!dup_ret) {
		folio_ref_add(f->folio, nr);
		inc_mm_counter(f->mm[1], MM_ANONPAGES);
		f->mapped[1] = true;
	}
	pte_unmap_unlock(pte, ptl);
	KUNIT_ASSERT_EQ(test, dup_ret, 0);
	pte = test_mm_lookup(f->mm[1], TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	set_ptes(f->mm[1], TEST_VA, pte, pte_wrprotect(entry), 1);
	pte_unmap_unlock(pte, ptl);
	expect_coarse_anon_state(test, f->folio, 2, false);
	KUNIT_EXPECT_EQ(test, folio_ref_count(f->folio), (int)(2 * nr + 1));
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(f->mm[0], MM_ANONPAGES), 1L);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(f->mm[1], MM_ANONPAGES), 1L);

	/* Exercise ordinary unmapping of each shared coarse leaf. */
	for (i = 0; i < 2; i++) {
		mmap_read_lock(f->mm[i]);
		zap_vma_range(f->vma[i], TEST_VA, SZ_64K);
		f->mapped[i] = 0;
		mmap_read_unlock(f->mm[i]);
		expect_coarse_anon_state(test, f->folio, 1 - i, false);
		KUNIT_EXPECT_EQ(test, folio_ref_count(f->folio), (int)((1 - i) * nr + 1));
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(f->mm[i], MM_ANONPAGES), 0L);
	}

	/* Keep an observation ref while cleanup releases the empty address spaces. */
	{
		struct folio *folio = f->folio;

		folio_get(folio);
		coarse_anon_fixture_free(f);
		expect_coarse_anon_state(test, folio, 0, false);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
		folio_put(folio);
	}
}

static void user64k_anon_zap_batch_test(struct kunit *test)
{
	struct coarse_anon_fixture *f;
	struct test_mm tables = {};
	struct vm_area_struct *vma;
	struct mm_struct *mm;
	unsigned int i, nr = SZ_64K / PAGE_SIZE, batch;
	pte_t entry, *pte;
	spinlock_t *ptl;
	int ret;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires coarse anonymous leaves");
		return;
	}
	f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, coarse_anon_fixture_free, f), 0);
	f->leaves = 2;
	f->folio = folio_alloc(GFP_KERNEL | __GFP_ZERO, 17 - PAGE_SHIFT);
	KUNIT_ASSERT_NOT_NULL(test, f->folio);
	mm = f->mm[0] = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, mm);
	/* Partial unmap can enqueue this folio on the memcg split queue. */
	KUNIT_ASSERT_EQ(test, mem_cgroup_charge(f->folio, mm, GFP_KERNEL), 0);
	KUNIT_ASSERT_EQ(test, test_mm_select_granule(mm, 16), 0);
	vma = f->vma[0] = test_fault_vma_in_mm(mm, TEST_VA, 2 * SZ_64K);
	f->attached[0] = !!vma;
	KUNIT_ASSERT_NOT_NULL(test, vma);
	tables.mm = mm;
	for (i = 0; i < 2; i++) {
		unsigned long addr = TEST_VA + i * SZ_64K;

		KUNIT_ASSERT_EQ(test, test_map_alias(&tables, i, addr,
				PFN_PHYS(folio_pfn(f->folio)) + i * SZ_64K), 0);
		pte = test_mm_lookup(mm, addr, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, pte);
		pte_clear(mm, addr, pte);
		pte_unmap_unlock(pte, ptl);
	}
	mmap_write_lock(mm);
	ret = anon_vma_prepare(vma);
	mmap_write_unlock(mm);
	KUNIT_ASSERT_EQ(test, ret, 0);
	pte = test_mm_lookup(mm, TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	folio_add_new_anon_rmap(f->folio, vma, TEST_VA, RMAP_EXCLUSIVE);
	folio_ref_add(f->folio, 2 * nr);
	add_mm_counter(mm, MM_ANONPAGES, 2);
	f->mapped[0] = BIT(0) | BIT(1);
	entry = pte_mkwrite(pte_mkdirty(phys_pte_mm(mm,
			PFN_PHYS(folio_pfn(f->folio)), vma->vm_page_prot)), vma);
	set_ptes(mm, TEST_VA, pte, entry, 2);
	batch = folio_subpage_pte_batch(vma, f->folio, pte, entry, 2, 0);
	pte_unmap_unlock(pte, ptl);
	KUNIT_EXPECT_EQ(test, batch, 2U);
	KUNIT_EXPECT_EQ(test, folio_mapcount(f->folio), (int)(2 * nr));
	KUNIT_EXPECT_EQ(test, folio_ref_count(f->folio), (int)(2 * nr + 1));

	/* The reference walker must consume logical PTEs and native mapcounts. */
	{
		vma_flags_t flags;
		bool young;

		folio_lock(f->folio);
		ret = folio_referenced(f->folio, 1, NULL, &flags);
		folio_unlock(f->folio);
		KUNIT_EXPECT_EQ(test, ret, 1);
		pte = test_mm_lookup(mm, TEST_VA, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, pte);
		young = pte_young(ptep_get(pte)) || pte_young(ptep_get(pte + 1));
		pte_unmap_unlock(pte, ptl);
		KUNIT_EXPECT_FALSE(test, young);
		folio_lock(f->folio);
		ret = folio_referenced(f->folio, 1, NULL, &flags);
		folio_unlock(f->folio);
		KUNIT_EXPECT_EQ(test, ret, 0);
	}

	/* Removing one logical leaf must leave the adjacent leaf's native span. */
	mmap_read_lock(mm);
	zap_vma_range(vma, TEST_VA, SZ_64K);
	f->mapped[0] &= ~BIT(0);
	mmap_read_unlock(mm);
	KUNIT_EXPECT_EQ(test, folio_mapcount(f->folio), (int)nr);
	KUNIT_EXPECT_EQ(test, folio_ref_count(f->folio), (int)nr + 1);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 1L);
	for (i = 0; i < 2 * nr; i++)
		if (IS_ENABLED(CONFIG_PAGE_MAPCOUNT))
			KUNIT_EXPECT_EQ(test, atomic_read(&folio_page(f->folio, i)->_mapcount),
					i < nr ? -1 : 0);
	pte = test_mm_lookup(mm, TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	ret = pte_none(ptep_get(pte)) && pte_present(ptep_get(pte + 1));
	pte_unmap_unlock(pte, ptl);
	KUNIT_EXPECT_TRUE(test, ret);

	/* Restore the first leaf, then remove both through one batched walk. */
	pte = test_mm_lookup(mm, TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, pte);
	folio_add_anon_rmap_pte(f->folio, &f->folio->page, vma,
			      TEST_VA, RMAP_EXCLUSIVE);
	folio_ref_add(f->folio, nr);
	inc_mm_counter(mm, MM_ANONPAGES);
	f->mapped[0] |= BIT(0);
	set_ptes(mm, TEST_VA, pte, entry, 1);
	pte_unmap_unlock(pte, ptl);
	KUNIT_EXPECT_EQ(test, folio_mapcount(f->folio), (int)(2 * nr));
	mmap_read_lock(mm);
	zap_vma_range(vma, TEST_VA, 2 * SZ_64K);
	f->mapped[0] = 0;
	mmap_read_unlock(mm);
	KUNIT_EXPECT_EQ(test, folio_mapcount(f->folio), 0);
	KUNIT_EXPECT_EQ(test, folio_ref_count(f->folio), 1);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 0L);
	for (i = 0; i < 2 * nr; i++)
		if (IS_ENABLED(CONFIG_PAGE_MAPCOUNT))
			KUNIT_EXPECT_EQ(test, atomic_read(&folio_page(f->folio, i)->_mapcount), -1);
}

static void user64k_file_rmap_span_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	struct folio *folio;
	unsigned int i, nr = SZ_64K / PAGE_SIZE;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires coarse nonanonymous leaf");
		return;
	}
	mm = test_vma_mm_granule(test, 16);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	vma = vm_area_alloc(mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_free_vma, vma), 0);
	vma->vm_start = TEST_VA;
	vma->vm_end = TEST_VA + 2 * SZ_64K;
	folio = folio_alloc(GFP_KERNEL | __GFP_ZERO, 16 - PAGE_SHIFT);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	/* Private metadata-only fixture: no file-cache promotion or fault is implied. */
	folio_lock(folio);
	folio_add_file_rmap_pte(folio, &folio->page, vma);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)nr);
	for (i = 0; i < nr; i++)
		if (IS_ENABLED(CONFIG_PAGE_MAPCOUNT))
			KUNIT_EXPECT_EQ(test, atomic_read(&folio_page(folio, i)->_mapcount) + 1, 1);
	folio_dup_file_rmap_pte(folio, &folio->page, vma);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)(2 * nr));
	for (i = 0; i < nr; i++)
		if (IS_ENABLED(CONFIG_PAGE_MAPCOUNT))
			KUNIT_EXPECT_EQ(test, atomic_read(&folio_page(folio, i)->_mapcount) + 1, 2);
	folio_remove_rmap_pte(folio, &folio->page, vma);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)nr);
	folio_remove_rmap_pte(folio, &folio->page, vma);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	for (i = 0; i < nr; i++)
		if (IS_ENABLED(CONFIG_PAGE_MAPCOUNT))
			KUNIT_EXPECT_EQ(test, atomic_read(&folio_page(folio, i)->_mapcount), -1);
	folio_unlock(folio);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
	folio_put(folio);
}

static struct vm_area_struct *test_fault_vma(struct kunit *test,
					    unsigned long start, unsigned long len)
{
	return test_fault_vma_in_mm(test_vma_mm(test), start, len);
}

static struct vm_area_struct *test_fault_vma_granule(struct kunit *test,
		unsigned int shift, unsigned long start, unsigned long len)
{
	return test_fault_vma_in_mm(test_vma_mm_granule(test, shift), start, len);
}

static void test_anon_write_fault_granule(struct kunit *test, unsigned int shift)
{
	unsigned long size = 1UL << shift;
	unsigned int shape;

	/* Swap/reclaim integration is a later gate, before any ABI activation. */
	KUNIT_ASSERT_EQ(test, total_swap_pages, 0UL);
	for (shape = 0; shape < 2; shape++) {
		unsigned long start = TEST_VA + (shape ? size : 0);
		unsigned long len = shape ? size : 2 * PAGE_SIZE;
		struct vm_area_struct *vma = test_fault_vma_granule(test, shift, start, len);
		struct folio *backing = NULL;
		unsigned int i, mapped = 0;
		vm_fault_t ret;

		KUNIT_ASSERT_NOT_NULL(test, vma);
		mmap_read_lock(vma->vm_mm);
		ret = handle_mm_fault(vma, TEST_VA + size + 3,
				      FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(vma->vm_mm);
		KUNIT_ASSERT_EQ(test, ret, (vm_fault_t)0);
		/* Remove transient LRU-add references before checking ownership. */
		lru_add_drain_all();
		for (i = 0; i < (PAGE_SIZE >> shift); i++) {
			unsigned long addr = TEST_VA + i * size;
			spinlock_t *ptl;
			pte_t *pte = test_mm_lookup(vma->vm_mm, addr, &ptl);
			struct mm_subpage *slot = NULL;
			bool expected = !shape || i == 1;
			bool present, writable = false, zero = false, identity = false;

			KUNIT_ASSERT_NOT_NULL(test, pte);
			present = pte_present(ptep_get(pte));
			if (present) {
				slot = mm_subpage_get_from_phys(pte_phys_mm(vma->vm_mm, ptep_get(pte)));
				writable = pte_write(ptep_get(pte));
				if (slot) {
					struct folio *folio = mm_subpage_folio(slot);

					if (!backing)
						backing = folio;
					identity = folio == backing &&
						mm_subpage_phys(slot) ==
						pte_phys_mm(vma->vm_mm, ptep_get(pte)) &&
						(shape || mm_subpage_offset(slot) == i * size);
					zero = !memchr_inv(folio_address(folio) + mm_subpage_offset(slot),
							   0, size);
					mm_subpage_put(slot);
				}
				mapped++;
			}
			pte_unmap_unlock(pte, ptl);
			KUNIT_EXPECT_EQ(test, present, expected);
			if (expected) {
				KUNIT_EXPECT_TRUE(test, writable);
				KUNIT_EXPECT_TRUE(test, identity);
				KUNIT_EXPECT_TRUE(test, zero);
			}
		}
		KUNIT_ASSERT_NOT_NULL(test, backing);
		KUNIT_EXPECT_EQ(test, mapped, shape ? 1U : (PAGE_SIZE >> shift));
		KUNIT_EXPECT_EQ(test, folio_mapcount(backing), (int)mapped);
		KUNIT_EXPECT_EQ(test, folio_ref_count(backing), (int)mapped);
		KUNIT_EXPECT_TRUE(test, folio_test_lru(backing));
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_ANONPAGES),
				(unsigned long)mapped);
	}
}

static void user4k_anon_write_fault_test(struct kunit *test)
{
	test_anon_write_fault_granule(test, 12);
}

static void user16k_anon_write_fault_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K alternative faults require larger native backing");
		return;
	}
	test_anon_write_fault_granule(test, 14);
}

static void user4k_swap_encoding_test(struct kunit *test)
{
	const unsigned long offsets[] = { 1, 2, (1UL << __SWP_OFFSET_BITS) - 2 };
	const unsigned int types[] = { 0, 1, __SWP_TYPE_MASK };
	unsigned int i, j, slot, flags;

	for (i = 0; i < ARRAY_SIZE(offsets); i++) {
		for (j = 0; j < ARRAY_SIZE(types); j++) {
			for (slot = 0; slot < PAGE_SIZE / SZ_4K; slot++) {
				for (flags = 0; flags < 4; flags++) {
					softleaf_t entry = swp_entry(types[j], offsets[i]);
					pte_t pte = softleaf_to_pte(entry), moved;

					pte = pte_swp_set_subpage_offset(pte, slot * SZ_4K);
					if (flags & 1)
						pte = pte_swp_mkexclusive(pte);
					if (flags & 2)
						pte = pte_swp_mkuffd(pte);
					KUNIT_EXPECT_FALSE(test, pte_present(pte));
					KUNIT_EXPECT_EQ(test, softleaf_from_pte(pte).val, entry.val);
					KUNIT_EXPECT_EQ(test, pte_swp_subpage_offset(pte), slot * (unsigned int)SZ_4K);
					moved = pte_move_swp_offset(pte, 1);
					KUNIT_EXPECT_EQ(test, swp_offset(softleaf_from_pte(moved)), offsets[i] + 1);
					KUNIT_EXPECT_EQ(test, pte_swp_subpage_offset(moved), slot * (unsigned int)SZ_4K);
					KUNIT_EXPECT_TRUE(test, pte_same(pte_move_swp_offset(moved, -1), pte));
					moved = pte_swp_set_subpage_offset(pte, ((slot + 1) % (PAGE_SIZE / SZ_4K)) * SZ_4K);
					KUNIT_EXPECT_EQ(test, softleaf_from_pte(moved).val, entry.val);
					KUNIT_EXPECT_EQ(test, pte_swp_subpage_offset(moved), ((slot + 1) % (PAGE_SIZE / SZ_4K)) * (unsigned int)SZ_4K);
				}
			}
		}
	}
}

static pte_t test_fault_entry(struct kunit *test, struct mm_struct *mm, unsigned long addr)
{
	spinlock_t *ptl;
	pte_t *pte = test_mm_lookup(mm, addr, &ptl);
	pte_t entry = __pte(0);

	KUNIT_EXPECT_NOT_NULL(test, pte);
	if (pte) {
		entry = ptep_get(pte);
		pte_unmap_unlock(pte, ptl);
	}
	return entry;
}


static void test_file_put(void *file)
{
	fput(file);
}

static struct file *test_cache_file(struct kunit *test, unsigned long size, bool fill)
{
	struct file *file = shmem_file_setup("user4k-file-test", size, EMPTY_VMA_FLAGS);
	unsigned char *buf;
	loff_t pos = 0;
	unsigned int i;

	if (IS_ERR(file))
		return file;
	if (kunit_add_action_or_reset(test, test_file_put, file))
		return ERR_PTR(-ENOMEM);
	if (!fill)
		return file;
	buf = kunit_kmalloc(test, size, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);
	for (i = 0; i < size; i++)
		buf[i] = 0x31 + (i / SZ_4K);
	if (kernel_write(file, buf, size, &pos) != size)
		return ERR_PTR(-EIO);
	return file;
}

static struct vm_area_struct *test_file_vma_granule(struct kunit *test, struct file *file,
		unsigned int shift, bool shared, unsigned long start, unsigned long len,
		unsigned long offset)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	vm_flags_t flags = VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE;
	int ret;

	mm = test_vma_mm_granule(test, shift);
	if (!mm)
		return NULL;
	vma = vm_area_alloc(mm);
	if (!vma)
		return NULL;
	vma->vm_start = start;
	vma->vm_end = start + len;
	vma_set_page_offset(vma, (struct vm_page_offset) {
		.index = offset >> PAGE_SHIFT, .offset = offset & ~PAGE_MASK,
	});
	if (shared)
		flags |= VM_SHARED | VM_MAYSHARE;
	vm_flags_init(vma, flags);
	vma->vm_file = get_file(file);
	vma->vm_page_prot = vm_get_page_prot(flags);
	mmap_write_lock(mm);
	ret = vfs_mmap(file, vma);
	if (!ret) {
		vma_set_page_prot(vma);
		ret = insert_vm_struct(mm, vma);
	}
	mmap_write_unlock(mm);
	if (ret) {
		fput(vma->vm_file);
		vm_area_free(vma);
		return NULL;
	}
	return vma;
}

static struct vm_area_struct *test_file_vma(struct kunit *test, struct file *file,
		bool native, bool shared, unsigned long start, unsigned long len,
		unsigned long offset)
{
	return test_file_vma_granule(test, file, native ? PAGE_SHIFT : 12,
				   shared, start, len, offset);
}

static void user4k_mmap_descriptor_test(struct kunit *test)
{
	struct mm_struct *mm = test_vma_mm(test);
	struct vm_area_struct *vma;
	struct vm_area_desc desc;

	KUNIT_ASSERT_NOT_NULL(test, mm);
	vma = vm_area_alloc(mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_free_vma, vma), 0);
	vma->vm_start = TEST_VA;
	vma->vm_end = TEST_VA + PAGE_SIZE;
	vma_set_page_offset(vma, (struct vm_page_offset) { 7, SZ_4K });
	compat_set_desc_from_vma(&desc, NULL, vma);
	KUNIT_EXPECT_EQ(test, desc.pgoff, 7UL);
	KUNIT_EXPECT_EQ(test, desc.subpage_offset, (unsigned int)SZ_4K);
	desc.pgoff = 19;
	desc.subpage_offset = 3 * SZ_4K;
	KUNIT_ASSERT_EQ(test, __compat_vma_mmap(&desc, vma), 0);
	KUNIT_EXPECT_TRUE(test, vm_page_offset_equal(vma_page_offset_at(vma, TEST_VA),
						    (struct vm_page_offset) { 19, 3 * SZ_4K }));
}

static void user4k_mmap_region_offsets_test(struct kunit *test)
{
	struct file *file;
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	unsigned long start = TEST_VA + SZ_4K;
	vm_flags_t flags = VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE |
		VM_SHARED | VM_MAYSHARE | VM_NORESERVE;
	unsigned long result[4];
	unsigned int i;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mapping fixture requires hardware opt-in");
		return;
	}
	file = test_cache_file(test, 8 * PAGE_SIZE, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	/* Only the full native-sized lengths are used before accounting conversion. */
	kthread_use_mm(mm);
	mmap_write_lock(mm);
	result[0] = mmap_region(file, start + PAGE_SIZE, PAGE_SIZE, legacy_to_vma_flags(flags),
			       (struct vm_page_offset) { 1, SZ_4K }, NULL);
	result[1] = mmap_region(file, start + 2 * PAGE_SIZE, PAGE_SIZE, legacy_to_vma_flags(flags),
			       (struct vm_page_offset) { 2, SZ_4K }, NULL);
	result[2] = mmap_region(file, start, PAGE_SIZE, legacy_to_vma_flags(flags),
			       (struct vm_page_offset) { 0, SZ_4K }, NULL);
	/* Adjacent virtual bytes with a different file offset must not merge. */
	result[3] = mmap_region(file, start + 3 * PAGE_SIZE, PAGE_SIZE, legacy_to_vma_flags(flags),
			       (struct vm_page_offset) { 3, 2 * SZ_4K }, NULL);
	mmap_write_unlock(mm);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, result[0], start + PAGE_SIZE);
	KUNIT_ASSERT_EQ(test, result[1], start + 2 * PAGE_SIZE);
	KUNIT_ASSERT_EQ(test, result[2], start);
	KUNIT_ASSERT_EQ(test, result[3], start + 3 * PAGE_SIZE);
	KUNIT_EXPECT_EQ(test, mm->map_count, 2);
	vma = test_vma_lookup(mm, start);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_EXPECT_EQ(test, vma->vm_start, start);
	KUNIT_EXPECT_EQ(test, vma->vm_end, start + 3 * PAGE_SIZE);
	for (i = 0; i < 12; i++)
		KUNIT_EXPECT_TRUE(test, vm_page_offset_equal(vma_page_offset_at(vma, start + i * SZ_4K),
					(struct vm_page_offset) { (i + 1) / TEST_SLOTS, ((i + 1) % TEST_SLOTS) * SZ_4K }));
	vma = test_vma_lookup(mm, start + 3 * PAGE_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_EXPECT_EQ(test, vma_subpage_offset(vma), 2U * SZ_4K);
}

#ifdef CONFIG_PROC_FS
/* Exercise the actual status/statm producers without publishing a fake task. */
extern void task_mem(struct seq_file *m, struct mm_struct *mm);
extern unsigned long task_vsize(struct mm_struct *mm);
extern unsigned long task_statm(struct mm_struct *mm, unsigned long *shared,
		unsigned long *text, unsigned long *data, unsigned long *resident);

static void expect_status_kb(struct kunit *test, struct seq_file *seq,
		const char *name, unsigned long expected)
{
	const char *field = strstr(seq->buf, name);
	unsigned long value = ULONG_MAX;

	KUNIT_ASSERT_NOT_NULL(test, field);
	KUNIT_EXPECT_EQ(test, sscanf(field + strlen(name), "%lu", &value), 1);
	KUNIT_EXPECT_EQ_MSG(test, value, expected, "%s", name);
}

static void user4k_accounting_readers_test(struct kunit *test)
{
	struct mm_struct *small = test_vma_mm(test);
	struct mm_struct *native = mm_alloc();
	struct seq_file seq = {};
	unsigned long shared, text, data, resident, size, maxrss = 0;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, native), 0);
	seq.size = 2048;
	seq.buf = kunit_kzalloc(test, seq.size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, seq.buf);
	/* Same logical counts, deliberately not multiples of a native page. */
	for (i = 0; i < 2; i++) {
		struct mm_struct *mm = i ? native : small;
		unsigned long kb = i ? PAGE_SIZE / 1024 : 4;

		mm->total_vm = 11;
		mm->hiwater_vm = 13;
		mm->data_vm = 5;
		mm->stack_vm = 2;
		mm->exec_vm = 3;
		mm->locked_vm = mm_pages_to_account(mm, 1);
		atomic64_set(&mm->pinned_vm, 2);
		mm->hiwater_rss = 10;
		mm->start_code = TEST_VA + mm_page_size(mm);
		mm->end_code = mm->start_code + 2 * mm_page_size(mm);
		percpu_counter_set(&mm->rss_stat[MM_ANONPAGES], 5);
		percpu_counter_set(&mm->rss_stat[MM_FILEPAGES], 2);
		percpu_counter_set(&mm->rss_stat[MM_SHMEMPAGES], 1);
		percpu_counter_set(&mm->rss_stat[MM_SWAPENTS], 3);
		seq.count = 0;
		task_mem(&seq, mm);
		KUNIT_EXPECT_FALSE(test, seq_has_overflowed(&seq));
		seq.buf[min(seq.count, seq.size - 1)] = '\0';
		expect_status_kb(test, &seq, "VmPeak:", 13 * kb);
		expect_status_kb(test, &seq, "VmSize:", 11 * kb);
		expect_status_kb(test, &seq, "VmLck:", kb);
		expect_status_kb(test, &seq, "VmPin:", 2 * PAGE_SIZE / 1024);
		expect_status_kb(test, &seq, "VmHWM:", 10 * kb);
		expect_status_kb(test, &seq, "VmRSS:", 8 * kb);
		expect_status_kb(test, &seq, "RssAnon:", 5 * kb);
		expect_status_kb(test, &seq, "RssFile:", 2 * kb);
		expect_status_kb(test, &seq, "RssShmem:", kb);
		expect_status_kb(test, &seq, "VmData:", 5 * kb);
		expect_status_kb(test, &seq, "VmStk:", 2 * kb);
		expect_status_kb(test, &seq, "VmExe:", 2 * kb);
		expect_status_kb(test, &seq, "VmLib:", kb);
		expect_status_kb(test, &seq, "VmSwap:", 3 * kb);
		KUNIT_EXPECT_EQ(test, task_vsize(mm), 11 * kb * 1024);
		{
			struct vm_area_struct vma = { .vm_mm = mm };

			KUNIT_EXPECT_EQ(test, vma_kernel_pagesize(&vma), kb * 1024);
			KUNIT_EXPECT_EQ(test, vma_mmu_pagesize(&vma), kb * 1024);
		}
	}
	/* Maxima must compare bytes, not incompatible 4K/16K counts. */
	setmax_mm_hiwater_rss(&maxrss, small);
	KUNIT_EXPECT_EQ(test, maxrss, 40UL);
	setmax_mm_hiwater_rss(&maxrss, native);
	KUNIT_EXPECT_EQ(test, maxrss, 10UL * PAGE_SIZE / 1024);
	setmax_mm_hiwater_rss(&maxrss, small);
	KUNIT_EXPECT_EQ(test, maxrss, 10UL * PAGE_SIZE / 1024);
	small->hiwater_rss = 10 * TEST_SLOTS + 1;
	setmax_mm_hiwater_rss(&maxrss, small);
	KUNIT_EXPECT_EQ(test, maxrss, (10UL * TEST_SLOTS + 1) * 4);

	KUNIT_EXPECT_EQ(test, mm_pages_to_native(small, 0), 0UL);
	KUNIT_EXPECT_EQ(test, mm_pages_to_native(small, 1), 1UL);
	KUNIT_EXPECT_EQ(test, mm_pages_to_native(small, 5), DIV_ROUND_UP(5UL, TEST_SLOTS));
	KUNIT_EXPECT_EQ(test, mm_pages_for_reader(native, small, 5), 5UL * TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, mm_pages_for_reader(small, native, 5), DIV_ROUND_UP(5UL, TEST_SLOTS));
	KUNIT_EXPECT_EQ(test, mm_pages_for_reader(small, small, 5), 5UL);
	KUNIT_EXPECT_EQ(test, mm_pages_for_reader(native, native, 5), 5UL);
	KUNIT_EXPECT_EQ(test, mm_pages_to_shift(small, 5, MM_ACCOUNT_SHIFT), 5UL);
	KUNIT_EXPECT_EQ(test, mm_pages_to_shift(native, 5, MM_ACCOUNT_SHIFT), 5UL * TEST_SLOTS);

	/* A kernel reader defaults to native units; partial pages round up. */
	size = task_statm(small, &shared, &text, &data, &resident);
	KUNIT_EXPECT_EQ(test, size, DIV_ROUND_UP(11UL, TEST_SLOTS));
	KUNIT_EXPECT_EQ(test, shared, 1UL);
	KUNIT_EXPECT_EQ(test, text, 1UL);
	KUNIT_EXPECT_EQ(test, data, DIV_ROUND_UP(7UL, TEST_SLOTS));
	KUNIT_EXPECT_EQ(test, resident, DIV_ROUND_UP(8UL, TEST_SLOTS));
	if (user4k_test_enabled || IS_ENABLED(CONFIG_KASAN)) {
		kthread_use_mm(small);
		size = task_statm(native, &shared, &text, &data, &resident);
		kthread_unuse_mm(small);
		KUNIT_EXPECT_EQ(test, size, 11UL * TEST_SLOTS);
		KUNIT_EXPECT_EQ(test, shared, 3UL * TEST_SLOTS);
		KUNIT_EXPECT_EQ(test, text, 2UL * TEST_SLOTS);
		KUNIT_EXPECT_EQ(test, data, 7UL * TEST_SLOTS);
		KUNIT_EXPECT_EQ(test, resident, 8UL * TEST_SLOTS);
	}
	/* Synthetic counters have no page references; restore before mmput. */
	for (i = 0; i < 2; i++) {
		struct mm_struct *mm = i ? native : small;
		unsigned int counter;

		for (counter = 0; counter < NR_MM_COUNTERS; counter++)
			percpu_counter_set(&mm->rss_stat[counter], 0);
		mm->total_vm = mm->data_vm = mm->stack_vm = mm->exec_vm = 0;
		mm->locked_vm = 0;
		atomic64_set(&mm->pinned_vm, 0);
	}
}
#endif

static void mixed_locked_account_units_test(struct kunit *test)
{
	unsigned int shift;
	unsigned long native_units = PAGE_SIZE >> MM_ACCOUNT_SHIFT;

	for (shift = 12; shift <= 16; shift += 2) {
		struct mm_struct *mm = mm_alloc();
		unsigned long size = 1UL << shift;
		unsigned long base, limit;
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
		if (shift != PAGE_SHIFT)
			KUNIT_ASSERT_EQ(test, test_mm_select_granule(mm, shift), 0);
		base = mm_pages_to_account(mm, 1);
		mmap_write_lock(mm);
		/* Model one locked VMA leaf and two independently charged native pins. */
		mm->locked_vm = base;
		ret = __account_locked_vm(mm, 1, true, current, true);
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm->locked_vm, base + native_units);
		KUNIT_EXPECT_EQ(test, mm_locked_vm_bytes(mm), size + PAGE_SIZE);
		KUNIT_EXPECT_EQ(test, mm_locked_vm_native_pages(mm),
				DIV_ROUND_UP(size + PAGE_SIZE, PAGE_SIZE));
		ret = __account_locked_vm(mm, 1, true, current, true);
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm->locked_vm, base + 2 * native_units);
		ret = __account_locked_vm(mm, 1, false, current, true);
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm_locked_vm_bytes(mm), size + PAGE_SIZE);
		ret = __account_locked_vm(mm, 1, false, current, true);
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm->locked_vm, base);
		KUNIT_EXPECT_EQ(test, mm_locked_vm_bytes(mm), size);

		/* A refused charge must leave the exact pre-existing amount intact. */
		limit = task_rlimit(current, RLIMIT_MEMLOCK) >> MM_ACCOUNT_SHIFT;
		mm->locked_vm = limit;
		ret = __account_locked_vm(mm, 1, true, current, false);
		KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
		KUNIT_EXPECT_EQ(test, mm->locked_vm, limit);
		mm->locked_vm = ULONG_MAX - native_units + 1;
		ret = __account_locked_vm(mm, 1, true, current, true);
		KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
		KUNIT_EXPECT_EQ(test, mm->locked_vm, ULONG_MAX - native_units + 1);
		mm->locked_vm = 0;
		if (native_units > 1) {
			ret = __account_locked_vm(mm,
				(ULONG_MAX >> (PAGE_SHIFT - MM_ACCOUNT_SHIFT)) + 1,
				true, current, true);
			KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
			KUNIT_EXPECT_EQ(test, mm->locked_vm, 0UL);
		}
		mmap_write_unlock(mm);
	}
}

/* Prior shmem fixtures can refund global commit from deferred fput work. */
static s64 test_commit_baseline(struct kunit *test)
{
	s64 before = percpu_counter_sum(&vm_committed_as), after;

	/* No mmap, file, or filesystem locks are held at these call sites. */
	flush_delayed_fput();
	after = percpu_counter_sum(&vm_committed_as);
	if (after != before)
		kunit_info(test, "drained deferred file refunds: commit delta %lld", after - before);
	return after;
}

static void user64k_commit_accounting_test(struct kunit *test)
{
	struct mm_struct *mm;
	s64 before;
	unsigned int i;
	unsigned long native_per_leaf;
	int ret;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires userspace granule larger than native");
		return;
	}
	mm = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, mm);
	KUNIT_ASSERT_EQ(test,
		kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
	KUNIT_ASSERT_EQ(test, test_mm_select_granule(mm, 16), 0);
	native_per_leaf = SZ_64K / PAGE_SIZE;
	before = test_commit_baseline(test);
	mmap_write_lock(mm);
	for (i = 0; i < 9; i++) {
		ret = mm_account_memory(mm, mm, 1);
		KUNIT_EXPECT_EQ(test, ret, 0);
		if (ret)
			break;
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, (unsigned long)i + 1);
		KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as) - before,
				(s64)((i + 1) * native_per_leaf));
	}
	/* Multiplication overflow must not mutate either reservation counter. */
	ret = mm_account_memory(mm, mm, LONG_MAX / native_per_leaf + 1);
	KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, (unsigned long)i);
	KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as) - before,
			(s64)(i * native_per_leaf));
	mm_acct_memory(mm, 3);
	i += 3;
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, (unsigned long)i);
	KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as) - before,
			(s64)(i * native_per_leaf));
	while (i) {
		mm_unacct_memory(mm, 1);
		i--;
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, (unsigned long)i);
		KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as) - before,
				(s64)(i * native_per_leaf));
	}
	mmap_write_unlock(mm);
}

#ifdef CONFIG_PAGE_TABLE_CHECK
static void expect_coarse_file_counts(struct kunit *test, void *data,
				     unsigned int phase)
{
	unsigned int offset;

	for (offset = 0; offset < 2 * SZ_64K; offset += SZ_4K) {
		int anon = -1, file = -1, expected;
		int ret = page_table_check_get_counts(virt_to_page(data + offset),
						     offset & ~PAGE_MASK, &anon, &file);

		/* Phase0: both coarse leaves;1:plusalias;2:firstclear;3:bothclear;4:allclear. */
		expected = phase < 2 || (phase == 2 && offset >= SZ_64K);
		if (phase >= 1 && phase <= 3 && offset == 2 * SZ_64K - SZ_4K)
			expected++;
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, anon, 0);
		KUNIT_EXPECT_EQ_MSG(test, file, expected, "phase=%u offset=%u", phase, offset);
	}
}
#endif

static void user64k_page_table_check_test(struct kunit *test)
{
#ifdef CONFIG_PAGE_TABLE_CHECK
	struct mm_struct *coarse, *small;
	void *data;
	pte_t ptes[2] = {}, alias = __pte(0), first, last;
	int anon, file;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires userspace leaf larger than native page");
		return;
	}
	data = test_alloc_granule(test, 17);
	KUNIT_ASSERT_NOT_NULL(test, data);
	if (page_table_check_get_counts(virt_to_page(data), 0, &anon, &file)) {
		kunit_skip(test, "requires page-table-check enabled at boot");
		return;
	}
	coarse = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, coarse);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, coarse), 0);
	KUNIT_ASSERT_EQ(test, test_mm_select_granule(coarse, 16), 0);
	small = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, small), 0);
	KUNIT_ASSERT_EQ(test, test_mm_select_granule(small, 12), 0);
	first = phys_pte_mm(coarse, virt_to_phys(data), __pgprot(TEST_PROT));
	last = phys_pte_mm(small, virt_to_phys(data) + 2 * SZ_64K - SZ_4K,
			   __pgprot(TEST_PROT));

	/* Exercise checker hooks directly; no actual coarse file fault is claimed. */
	page_table_check_ptes_set(coarse, TEST_VA, ptes, first, 2);
	ptes[0] = first;
	ptes[1] = pte_advance_pfn_mm(coarse, first, 1);
	expect_coarse_file_counts(test, data, 0);
	page_table_check_ptes_set(small, TEST_VA, &alias, last, 1);
	alias = last;
	expect_coarse_file_counts(test, data, 1);
	page_table_check_pte_clear(coarse, TEST_VA, ptes[0]);
	ptes[0] = __pte(0);
	expect_coarse_file_counts(test, data, 2);
	page_table_check_pte_clear(coarse, TEST_VA + SZ_64K, ptes[1]);
	ptes[1] = __pte(0);
	expect_coarse_file_counts(test, data, 3);
	page_table_check_pte_clear(small, TEST_VA, alias);
	expect_coarse_file_counts(test, data, 4);
#else
	kunit_skip(test, "requires CONFIG_PAGE_TABLE_CHECK");
#endif
}

static void user4k_commit_accounting_test(struct kunit *test)
{
	struct mm_struct *mm = test_vma_mm(test);
	s64 before;
	int ret;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, mm);
	before = test_commit_baseline(test);
	mmap_write_lock(mm);
	for (i = 0; i < 9; i++) {
		ret = mm_account_memory(mm, mm, 1);
		KUNIT_EXPECT_EQ(test, ret, 0);
		if (ret)
			break;
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, (unsigned long)i + 1);
		KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as) - before,
				(s64)DIV_ROUND_UP(i + 1, TEST_SLOTS));
	}
	if (sysctl_overcommit_memory == OVERCOMMIT_GUESS) {
		unsigned long old = mm->committed_user_pages;
		s64 committed = percpu_counter_sum(&vm_committed_as);

		ret = mm_account_memory(mm, mm, (totalram_pages() + total_swap_pages + 1) * TEST_SLOTS);
		KUNIT_EXPECT_EQ(test, ret, -ENOMEM);
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, old);
		KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as), committed);
	}
	/* Native physical-page callers must charge four user granules per page. */
	ret = __account_locked_vm(mm, 1, true, current, true);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, mm->locked_vm, (unsigned long)TEST_SLOTS);
	ret = __account_locked_vm(mm, 1, false, current, true);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, mm->locked_vm, 0UL);

	/* Refund one granule at a time: splitting must not multiply refunds. */
	while (i) {
		mm_unacct_memory(mm, 1);
		i--;
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, (unsigned long)i);
		KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as) - before,
				(s64)DIV_ROUND_UP(i, TEST_SLOTS));
	}
	mmap_write_unlock(mm);
}

/* Prefaulting must stop at each process-page boundary, not native pages. */
static void user_granule_prefault_test(struct kunit *test)
{
	unsigned int shift;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mapping fixture requires hardware opt-in");
		return;
	}
	for (shift = 12; shift <= PAGE_SHIFT; shift += 2) {
		struct mm_struct *mm = test_vma_mm_granule(test, shift);
		unsigned long size = 1UL << shift, start = TEST_VA;
		unsigned long flags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE;
		unsigned long mapped[3];
		size_t left[3][4] = {};
		int phase, ret = 0;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		kthread_use_mm(mm);
		mapped[0] = vm_mmap(NULL, start, size, PROT_READ | PROT_WRITE, flags, 0);
		mapped[2] = vm_mmap(NULL, start + 2 * size, size,
				    PROT_READ | PROT_WRITE, flags, 0);
		mapped[1] = start + size;
		if (mapped[0] == start && mapped[2] == start + 2 * size) {
			for (phase = 0; phase < 3; phase++) {
				char __user *addr = (char __user *)(start + 17);
				size_t len = 3 * size - 17;

				/* Missing middle, then read-only, then fully writable. */
				if (phase) {
					mapped[1] = vm_mmap(NULL, start + size, size,
						PROT_READ | (phase == 2 ? PROT_WRITE : 0),
						MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, 0);
					if (mapped[1] != start + size)
						break;
				}
				left[phase][0] = fault_in_safe_writeable(addr, len);
				left[phase][1] = fault_in_readable(addr, len);
				left[phase][2] = fault_in_writeable(addr, len);
				left[phase][3] = fault_in_subpage_writeable(addr, len);
			}
			ret = vm_munmap(start, 3 * size);
		}
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, mapped[0], start);
		KUNIT_ASSERT_EQ(test, mapped[1], start + size);
		KUNIT_ASSERT_EQ(test, mapped[2], start + 2 * size);
		KUNIT_EXPECT_EQ(test, ret, 0);
		for (phase = 0; phase < 3; phase++) {
			KUNIT_EXPECT_EQ(test, left[phase][0], phase < 2 ? 2 * size : 0UL);
			KUNIT_EXPECT_EQ(test, left[phase][1], phase == 0 ? 2 * size : 0UL);
			KUNIT_EXPECT_EQ(test, left[phase][2], phase < 2 ? 2 * size : 0UL);
			KUNIT_EXPECT_EQ(test, left[phase][3], phase < 2 ? 2 * size : 0UL);
		}
	}
}

static void user4k_mmap_accounting_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	unsigned long start = TEST_VA + SZ_4K, mapped;
	vm_flags_t flags = VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE;
	s64 before;
	unsigned int i;
	int ret = 0;
	vm_fault_t fault = 0;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mapping fixture requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	before = test_commit_baseline(test);
	kthread_use_mm(mm);
	mmap_write_lock(mm);
	mapped = mmap_region(NULL, start, 3 * SZ_4K, legacy_to_vma_flags(flags),
			     (struct vm_page_offset) { start >> PAGE_SHIFT, SZ_4K }, NULL);
	mmap_write_unlock(mm);
	if (!IS_ERR_VALUE(mapped)) {
		mmap_read_lock(mm);
		vma = vma_lookup(mm, start);
		fault = handle_mm_fault(vma, start, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(mm);
	}
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, mapped, start);
	KUNIT_ASSERT_EQ(test, fault & VM_FAULT_ERROR, 0U);
	KUNIT_EXPECT_EQ(test, mm->total_vm, 3UL);
	KUNIT_EXPECT_EQ(test, mm->data_vm, 3UL);
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 3UL);
	KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as) - before, 1LL);
	/* Remove the middle first, then the two neighbors. */
	for (i = 0; i < 3; i++) {
		unsigned int slot = i == 0 ? 1 : i == 1 ? 0 : 2;
		VMA_ITERATOR(vmi, mm, start + slot * SZ_4K);

		kthread_use_mm(mm);
		mmap_write_lock(mm);
		ret = do_vmi_munmap(&vmi, mm, start + slot * SZ_4K, SZ_4K, NULL, false);
		mmap_write_unlock(mm);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm->total_vm, 2UL - i);
		KUNIT_EXPECT_EQ(test, mm->data_vm, 2UL - i);
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 2UL - i);
		KUNIT_EXPECT_EQ(test, percpu_counter_sum(&vm_committed_as) - before, i == 2 ? 0LL : 1LL);
	}
	KUNIT_EXPECT_EQ(test, mm->map_count, 0);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 0L);
}

static vm_fault_t test_file_fault(struct vm_area_struct *vma, unsigned long addr, bool write)
{
	vm_fault_t ret;

	mmap_read_lock(vma->vm_mm);
	ret = handle_mm_fault(vma, addr, FAULT_FLAG_REMOTE |
			      (write ? FAULT_FLAG_WRITE : 0), NULL);
	mmap_read_unlock(vma->vm_mm);
	return ret;
}

static void user4k_vm_mmap_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct file *file;
	struct vm_area_struct *vma;
	unsigned long start = TEST_VA + SZ_4K;
	unsigned long anon_flags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE;
	unsigned long file_flags = MAP_SHARED | MAP_FIXED_NOREPLACE;
	unsigned long result[7];
	int unmap[4];
	pte_t pte;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mapping fixture requires hardware opt-in");
		return;
	}
	file = test_cache_file(test, 2 * PAGE_SIZE, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	mm->mmap_base = TEST_VA + 2 * PAGE_SIZE + SZ_4K;
	mm_flags_clear(MMF_TOPDOWN, mm);
	kthread_use_mm(mm);
	result[0] = vm_mmap(NULL, start, SZ_4K, PROT_READ | PROT_WRITE, anon_flags, 0);
	result[1] = vm_mmap(file, start + 2 * SZ_4K, SZ_4K, PROT_READ | PROT_WRITE,
			    file_flags, 3 * SZ_4K);
	result[2] = vm_mmap(file, start + 3 * SZ_4K, SZ_4K, PROT_READ | PROT_WRITE,
			    file_flags, 4 * SZ_4K);
	result[3] = vm_mmap(NULL, start, SZ_4K, PROT_READ, anon_flags, 0);
	result[4] = vm_mmap(NULL, start + 17, SZ_4K, PROT_READ, anon_flags, 0);
	result[5] = vm_mmap(file, start + SZ_4K, SZ_4K, PROT_READ, file_flags, SZ_4K + 1);
	result[6] = vm_mmap(NULL, 0, SZ_4K + 1, PROT_READ | PROT_WRITE,
			    MAP_PRIVATE | MAP_ANONYMOUS, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, result[0], start);
	KUNIT_ASSERT_EQ(test, result[1], start + 2 * SZ_4K);
	KUNIT_ASSERT_EQ(test, result[2], start + 3 * SZ_4K);
	KUNIT_EXPECT_EQ(test, result[3], (unsigned long)-EEXIST);
	KUNIT_EXPECT_EQ(test, result[4], (unsigned long)-EINVAL);
	KUNIT_EXPECT_EQ(test, result[5], (unsigned long)-EINVAL);
	KUNIT_ASSERT_EQ(test, result[6], mm->mmap_base);
	KUNIT_EXPECT_EQ(test, mm->total_vm, 5UL);
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 3UL);
	vma = test_vma_lookup(mm, start);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, start, true) & VM_FAULT_ERROR, 0U);
	vma = test_vma_lookup(mm, start + 2 * SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_EXPECT_EQ(test, vma->vm_end - vma->vm_start, 2UL * SZ_4K);
	KUNIT_EXPECT_EQ(test, vma_subpage_offset(vma), 3U * SZ_4K);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, start + 2 * SZ_4K, false) & VM_FAULT_ERROR, 0U);
	pte = test_fault_entry(test, mm, start + 2 * SZ_4K);
	KUNIT_ASSERT_TRUE(test, pte_present(pte));
	KUNIT_EXPECT_EQ(test, *(u8 *)phys_to_virt(pte_phys_mm(mm, pte)), (u8)0x34);
	pte = test_fault_entry(test, mm, start + 3 * SZ_4K);
	KUNIT_ASSERT_TRUE(test, pte_present(pte));
	KUNIT_EXPECT_EQ(test, *(u8 *)phys_to_virt(pte_phys_mm(mm, pte)), (u8)0x35);
	kthread_use_mm(mm);
	unmap[0] = vm_munmap(start + 2 * SZ_4K, 1);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, unmap[0], 0);
	vma = test_vma_lookup(mm, start + 3 * SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_EXPECT_EQ(test, vma->vm_pgoff, 4UL / TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, vma_subpage_offset(vma), (4U % TEST_SLOTS) * SZ_4K);
	KUNIT_EXPECT_EQ(test, mm->total_vm, 4UL);
	kthread_use_mm(mm);
	unmap[1] = vm_munmap(start, SZ_4K);
	unmap[2] = vm_munmap(start + 3 * SZ_4K, SZ_4K);
	unmap[3] = vm_munmap(result[6], 2 * SZ_4K);
	kthread_unuse_mm(mm);
	KUNIT_EXPECT_EQ(test, unmap[1], 0);
	KUNIT_EXPECT_EQ(test, unmap[2], 0);
	KUNIT_EXPECT_EQ(test, unmap[3], 0);
	KUNIT_EXPECT_EQ(test, mm->total_vm, 0UL);
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 0UL);
	KUNIT_EXPECT_EQ(test, mm->map_count, 0);
}

static void test_mlock_drain_all(struct kunit *test, struct folio *folio)
{
	unsigned int before, after;
	bool counted;

	lru_add_drain();
	counted = folio_test_unevictable(folio);
	before = counted ? folio->mlock_count : 0;
	/* A sleeping fixture can enqueue mlock work on more than one CPU. */
	lru_add_drain_all();
	after = folio_test_unevictable(folio) ? folio->mlock_count : 0;
	if (counted && before != after)
		kunit_info(test, "remote mlock drain changed count %u -> %u", before, after);
}

static void user4k_mlock_counts_test(struct kunit *test)
{
	struct mm_struct *mm = test_vma_mm(test);
	struct mm_subpage *slot;
	struct folio *folio;
	unsigned long addr;
	unsigned int i;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mlock requires hardware opt-in");
		return;
	}
	KUNIT_ASSERT_NOT_NULL(test, mm);
	kthread_use_mm(mm);
	addr = vm_mmap(NULL, TEST_VA, PAGE_SIZE, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE | MAP_LOCKED, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA);
	slot = mm_subpage_get_from_phys(pte_phys_mm(mm, test_fault_entry(test, mm, TEST_VA)));
	KUNIT_ASSERT_NOT_NULL(test, slot);
	folio = mm_subpage_folio(slot);
	folio_get(folio);
	mm_subpage_put(slot);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	test_mlock_drain_all(test, folio);
	KUNIT_EXPECT_TRUE(test, folio_test_mlocked(folio));
	KUNIT_EXPECT_EQ(test, folio->mlock_count, TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, mm->locked_vm, (unsigned long)TEST_SLOTS);
	for (i = 0; i < TEST_SLOTS; i++) {
		int ret;

		kthread_use_mm(mm);
		ret = vm_munmap(TEST_VA + i * SZ_4K, SZ_4K);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		test_mlock_drain_all(test, folio);
		/* mlock_count shares storage with the evictable LRU links. */
		if (i != TEST_SLOTS - 1)
			KUNIT_EXPECT_EQ(test, folio->mlock_count, TEST_SLOTS - 1 - i);
		KUNIT_EXPECT_EQ(test, mm->locked_vm, TEST_SLOTS - 1UL - i);
		KUNIT_EXPECT_EQ(test, folio_test_mlocked(folio), i != TEST_SLOTS - 1);
	}
}

static void test_mlock_enable_lru_cache(void *unused)
{
	lru_cache_enable();
}

static void user4k_mlock_shared_test(struct kunit *test)
{
	struct mm_struct *small = test_vma_mm(test), *native = mm_alloc();
	struct file *file = test_cache_file(test, PAGE_SIZE, true);
	struct folio *folio;
	unsigned long addr;
	int ret;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		if (native)
			mmput(native);
		kunit_skip(test, "borrowed-mm mlock requires hardware opt-in");
		return;
	}
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, native), 0);
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	/* Exact counts require synchronous LRU/lock accounting. Normal deferred
	 * batches may legitimately miss locks while the folio is off LRU;
	 * user4k_mlock_repair_test checks that reclaim repairs that case.
	 */
	lru_cache_disable();
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
			  test_mlock_enable_lru_cache, NULL), 0);
	kthread_use_mm(small);
	addr = vm_mmap(file, TEST_VA, PAGE_SIZE, PROT_READ | PROT_WRITE,
		       MAP_SHARED | MAP_LOCKED | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(small);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA);
	kthread_use_mm(native);
	addr = vm_mmap(file, TEST_VA, PAGE_SIZE, PROT_READ | PROT_WRITE,
		       MAP_SHARED | MAP_LOCKED | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(native);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA);
	folio = filemap_get_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	test_mlock_drain_all(test, folio);
	KUNIT_EXPECT_TRUE(test, folio_test_mlocked(folio));
	KUNIT_EXPECT_EQ(test, folio->mlock_count, TEST_SLOTS + 1);
	kthread_use_mm(small);
	ret = vm_munmap(TEST_VA + SZ_4K, SZ_4K);
	kthread_unuse_mm(small);
	KUNIT_ASSERT_EQ(test, ret, 0);
	test_mlock_drain_all(test, folio);
	KUNIT_EXPECT_EQ(test, folio->mlock_count, TEST_SLOTS);
	kthread_use_mm(native);
	ret = vm_munmap(TEST_VA, PAGE_SIZE);
	kthread_unuse_mm(native);
	KUNIT_ASSERT_EQ(test, ret, 0);
	test_mlock_drain_all(test, folio);
	KUNIT_EXPECT_EQ(test, folio->mlock_count, TEST_SLOTS - 1);
	KUNIT_EXPECT_TRUE(test, folio_test_mlocked(folio));
	kthread_use_mm(small);
	ret = vm_munmap(TEST_VA, PAGE_SIZE);
	kthread_unuse_mm(small);
	KUNIT_ASSERT_EQ(test, ret, 0);
	test_mlock_drain_all(test, folio);
	KUNIT_EXPECT_FALSE(test, folio_test_mlocked(folio));
}

static void test_mlock_putback(void *folio)
{
	folio_putback_lru(folio);
}

static void user4k_mlock_repair_test(struct kunit *test)
{
	unsigned int shifts[] = { PAGE_SHIFT, 12, 14 };
	unsigned int mode, walker;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mlock requires hardware opt-in");
		return;
	}
	for (mode = 0; mode < (PAGE_SHIFT > 14 ? 3U : 2U); mode++) {
		unsigned long size = 1UL << shifts[mode];

		for (walker = 0; walker < 2; walker++) {
			struct mm_struct *mm = test_vma_mm_granule(test, shifts[mode]);
			struct file *file = test_cache_file(test, PAGE_SIZE, true);
			struct folio *folio;
			unsigned long addr, offset;
			vma_flags_t flags = EMPTY_VMA_FLAGS;
			int maps, ret;

			KUNIT_ASSERT_NOT_NULL(test, mm);
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
			folio = filemap_get_folio(file->f_mapping, 0);
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
					  test_folio_put, folio), 0);
			lru_add_drain_all();
			KUNIT_ASSERT_TRUE(test, folio_isolate_lru(folio));
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
					  test_mlock_putback, folio), 0);
			/* Two aliases leave a locked mapping even after native unmap.
			 * Force deferred mlocks to run while the folio is isolated.
			 */
			for (offset = 0; offset < 2 * PAGE_SIZE; offset += PAGE_SIZE) {
				kthread_use_mm(mm);
				addr = vm_mmap(file, TEST_VA + offset, PAGE_SIZE,
					       PROT_READ | PROT_WRITE,
					       MAP_SHARED | MAP_LOCKED | MAP_FIXED_NOREPLACE, 0);
				kthread_unuse_mm(mm);
				KUNIT_ASSERT_EQ(test, addr, TEST_VA + offset);
			}
			lru_add_drain_all();
			KUNIT_EXPECT_FALSE(test, folio_test_lru(folio));
			KUNIT_EXPECT_TRUE(test, folio_test_mlocked(folio));
			kunit_release_action(test, test_mlock_putback, folio);
			lru_add_drain_all();
			KUNIT_ASSERT_TRUE(test, folio_test_unevictable(folio));
			KUNIT_EXPECT_EQ(test, folio->mlock_count, 0U);

			kthread_use_mm(mm);
			ret = vm_munmap(TEST_VA, size);
			kthread_unuse_mm(mm);
			KUNIT_ASSERT_EQ(test, ret, 0);
			lru_add_drain_all();
			KUNIT_EXPECT_FALSE(test, folio_test_mlocked(folio));
			maps = folio_mapcount(folio);
			KUNIT_EXPECT_EQ(test, maps, (int)(2 * PAGE_SIZE / size) - 1);
			folio_lock(folio);
			if (walker)
				ret = folio_referenced(folio, 1, NULL, &flags);
			else
				try_to_unmap(folio, TTU_SYNC);
			folio_unlock(folio);
			lru_add_drain_all();
			if (walker) {
				KUNIT_EXPECT_GE(test, ret, 0);
				KUNIT_EXPECT_TRUE(test, vma_flags_test(&flags, VMA_LOCKED_BIT));
			}
			KUNIT_EXPECT_TRUE(test, folio_test_mlocked(folio));
			KUNIT_EXPECT_EQ(test, folio_mapcount(folio), maps);
			for (offset = size; offset < 2 * PAGE_SIZE; offset += size) {
				pte_t pte = test_fault_entry(test, mm, TEST_VA + offset);

				KUNIT_ASSERT_TRUE(test, pte_present(pte));
				KUNIT_EXPECT_PTR_EQ(test,
					page_folio(pfn_to_page(PHYS_PFN(pte_phys_mm(mm, pte)))),
					folio);
			}
			kthread_use_mm(mm);
			ret = vm_munmap(TEST_VA, 2 * PAGE_SIZE);
			kthread_unuse_mm(mm);
			KUNIT_ASSERT_EQ(test, ret, 0);
			lru_add_drain_all();
			KUNIT_EXPECT_FALSE(test, folio_test_mlocked(folio));
			KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
			kunit_info(test, "%uK %s repairs deferred mlock without unmapping",
				   1U << (shifts[mode] - 10), walker ? "referenced" : "unmap");
		}
	}
}

asmlinkage long __arm64_sys_mremap(const struct pt_regs *regs);

static unsigned long test_mremap(unsigned long old, unsigned long old_len,
		unsigned long new_len, unsigned long flags, unsigned long new)
{
	struct pt_regs regs = { .regs = { old, old_len, new_len, flags, new }, .orig_x0 = old };

	return __arm64_sys_mremap(&regs);
}

static int test_iova_bitmap_fill(struct iova_bitmap *bitmap, unsigned long iova,
				size_t length, void *opaque)
{
	unsigned long unit = *(unsigned long *)opaque;

	iova_bitmap_set(bitmap, iova + 3 * unit, length - 10 * unit);
	/* A second write advances backwards into an already populated range. */
	iova_bitmap_set(bitmap, iova, 5 * unit);
	return 0;
}

/* Real IOMMUFD lifecycle with a translation-recording IOMMU, no physical DMA. */
struct test_fragment_domain {
	struct iommu_domain domain;
	struct xarray translations;
	unsigned long fail_after;
	unsigned long largest_map;
	unsigned long mapped;
	unsigned long map_calls;
};

struct test_fragment_iopt {
	struct io_pagetable table;
	struct test_fragment_domain domains[2];
	struct kunit *test;
};

static int test_fragment_map(struct iommu_domain *domain, unsigned long iova,
		phys_addr_t phys, size_t pgsize, size_t pgcount, int prot,
		gfp_t gfp, size_t *mapped)
{
	struct test_fragment_domain *mock = container_of(domain, typeof(*mock), domain);
	unsigned long length = pgsize * pgcount, done;
	int rc;

	mock->map_calls++;
	mock->largest_map = max(mock->largest_map, length);
	for (done = 0; done < length; done += SZ_4K) {
		if (mock->fail_after && mock->mapped >= mock->fail_after)
			return -ENOMEM;
		rc = xa_insert(&mock->translations, (iova + done) >> 12,
			       xa_mk_value((phys + done) >> 12), gfp);
		if (rc)
			return rc;
		*mapped += SZ_4K;
		mock->mapped += SZ_4K;
	}
	return 0;
}

static size_t test_fragment_unmap(struct iommu_domain *domain, unsigned long iova,
		size_t pgsize, size_t pgcount, struct iommu_iotlb_gather *gather)
{
	struct test_fragment_domain *mock = container_of(domain, typeof(*mock), domain);
	unsigned long length = pgsize * pgcount, done;

	for (done = 0; done < length; done += SZ_4K) {
		if (!xa_erase(&mock->translations, (iova + done) >> 12))
			break;
		mock->mapped -= SZ_4K;
	}
	return done;
}

static phys_addr_t test_fragment_translate(struct iommu_domain *domain, dma_addr_t iova)
{
	struct test_fragment_domain *mock = container_of(domain, typeof(*mock), domain);
	void *entry = xa_load(&mock->translations, iova >> 12);

	return entry ? ((phys_addr_t)xa_to_value(entry) << 12) + (iova & (SZ_4K - 1)) : 0;
}

static void test_fragment_sync(struct iommu_domain *domain, struct iommu_iotlb_gather *gather)
{
}

static const struct iommu_domain_ops test_fragment_domain_ops = {
	.map_pages = test_fragment_map,
	.unmap_pages = test_fragment_unmap,
	.iova_to_phys = test_fragment_translate,
	.iotlb_sync = test_fragment_sync,
};

static void test_fragment_iopt_free(void *data)
{
	struct test_fragment_iopt *fixture = data;
	struct iommu_domain *domain;
	unsigned long unmapped, index;
	unsigned int i;

	iopt_unmap_all(&fixture->table, &unmapped);
	while ((domain = xa_find(&fixture->table.domains, &(unsigned long){0},
				ULONG_MAX, XA_PRESENT)))
		iopt_table_remove_domain(&fixture->table, domain);
	iopt_destroy_table(&fixture->table);
	for (i = 0; i < ARRAY_SIZE(fixture->domains); i++) {
		struct test_fragment_domain *mock = &fixture->domains[i];
		void *entry;

		KUNIT_EXPECT_TRUE(fixture->test, xa_empty(&mock->translations));
		xa_for_each(&mock->translations, index, entry)
			xa_erase(&mock->translations, index);
		xa_destroy(&mock->translations);
	}
}

static struct test_fragment_iopt *test_fragment_iopt_alloc(struct kunit *test)
{
	struct test_fragment_iopt *fixture = kunit_kzalloc(test, sizeof(*fixture), GFP_KERNEL);
	unsigned int i;

	if (!fixture)
		return NULL;
	fixture->test = test;
	iopt_init_table(&fixture->table);
	for (i = 0; i < ARRAY_SIZE(fixture->domains); i++) {
		struct test_fragment_domain *mock = &fixture->domains[i];

		xa_init(&mock->translations);
		mock->domain.ops = &test_fragment_domain_ops;
		mock->domain.type = IOMMU_DOMAIN_UNMANAGED;
		mock->domain.pgsize_bitmap = SZ_4K;
		mock->domain.geometry.aperture_end = ULONG_MAX;
	}
	if (kunit_add_action_or_reset(test, test_fragment_iopt_free, fixture))
		return NULL;
	return fixture;
}

static int test_fragment_map_user(struct test_fragment_iopt *fixture,
		struct mm_struct *mm, unsigned long va, unsigned long iova, unsigned long length)
{
	struct iommufd_ctx ctx = {};
	int rc;

	kthread_use_mm(mm);
	rc = iopt_map_user_pages(&ctx, &fixture->table, &iova, (void __user *)va,
				 length, IOMMU_READ | IOMMU_WRITE, 0);
	kthread_unuse_mm(mm);
	return rc;
}

static void user4k_iommufd_lifetime_test(struct kunit *test)
{
	unsigned int shifts[] = { PAGE_SHIFT, 12, 14, 16 }, mode;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm IOMMUFD requires hardware opt-in");
		return;
	}
	for (mode = 0; mode < ARRAY_SIZE(shifts); mode++) {
		unsigned long size = 1UL << shifts[mode];
		unsigned long len = 2 * max(size, PAGE_SIZE);
		unsigned long dest = TEST_VA + 32 * PAGE_SIZE + size;
		unsigned long iova = SZ_1M, alias = SZ_16M, addr, moved, missed, unmapped;
		struct mm_struct *mm;
		struct test_fragment_iopt *fixture;
		struct iopt_pages *pages;
		struct iopt_area *area;
		unsigned char *buffer;
		unsigned long off, pinned;
		LIST_HEAD(copy);
		int rc;

		if (mode && shifts[mode] == PAGE_SHIFT)
			continue;
		mm = test_vma_mm_granule(test, shifts[mode]);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		fixture = test_fragment_iopt_alloc(test);
		KUNIT_ASSERT_NOT_NULL(test, fixture);
		buffer = kunit_kmalloc(test, len, GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, buffer);
		memset(buffer, 0x31, len);
		kthread_use_mm(mm);
		addr = vm_mmap(NULL, TEST_VA, len, PROT_READ | PROT_WRITE,
				MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
		missed = IS_ERR_VALUE(addr) ? len : copy_to_user((void __user *)addr, buffer, len);
		moved = missed ? 0 : test_mremap(addr, len, len, MREMAP_FIXED | MREMAP_MAYMOVE, dest);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, moved, dest);
		KUNIT_ASSERT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[0].domain), 0);
		KUNIT_ASSERT_EQ(test, test_fragment_map_user(fixture, mm, dest, iova, len), 0);
		pinned = atomic64_read(&mm->pinned_vm);
		KUNIT_EXPECT_EQ(test, pinned, len / PAGE_SIZE);
		KUNIT_EXPECT_GE(test, fixture->domains[0].largest_map, PAGE_SIZE);
		for (off = 0; off < len; off += size) {
			pte_t pte = test_fault_entry(test, mm, dest + off);
			phys_addr_t actual = iommu_iova_to_phys(&fixture->domains[0].domain, iova + off);

			KUNIT_EXPECT_EQ(test, actual, pte_phys_mm(mm, pte));
		}
		KUNIT_ASSERT_EQ(test, iopt_get_pages(&fixture->table, iova + size, len - size, &copy), 0);
		rc = iopt_map_pages(&fixture->table, &copy, len - size, &alias, IOMMU_READ | IOMMU_WRITE, 0);
		iopt_free_pages_list(&copy);
		KUNIT_ASSERT_EQ(test, rc, 0);
		KUNIT_EXPECT_EQ(test, (unsigned long)atomic64_read(&mm->pinned_vm), pinned);
		KUNIT_ASSERT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[1].domain), 0);
		KUNIT_EXPECT_EQ(test, (unsigned long)atomic64_read(&mm->pinned_vm), pinned);

		/* Replace the original VA: mappings and copied IOAS must retain old pins. */
		kthread_use_mm(mm);
		addr = vm_mmap(NULL, dest, len, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, 0);
		memset(buffer, 0x5a, len);
		missed = IS_ERR_VALUE(addr) ? len : copy_to_user((void __user *)dest, buffer, len);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, missed, 0UL);
		down_read(&fixture->table.iova_rwsem);
		area = iopt_area_iter_first(&fixture->table, iova, iova);
		pages = area->pages;
		/* Force the domain-backed reader for the native control too. */
		memset(buffer, 0x77, len);
		rc = iopt_pages_rw_access(pages, 0, buffer, len, IOMMUFD_ACCESS_RW_WRITE);
		up_read(&fixture->table.iova_rwsem);
		KUNIT_ASSERT_EQ(test, rc, 0);
		for (off = 0; off < len; off += SZ_4K) {
			phys_addr_t phys = iommu_iova_to_phys(&fixture->domains[1].domain, iova + off);
			void *ptr = kmap_local_page(pfn_to_page(PHYS_PFN(phys)));
			bool correct = !memchr_inv(ptr + offset_in_page(phys), 0x77, SZ_4K);

			kunmap_local(ptr);
			KUNIT_EXPECT_TRUE(test, correct);
		}
		kthread_use_mm(mm);
		missed = copy_from_user(buffer, (void __user *)dest, len);
		/* Local CPU access retains the existing copy_from_user fast path. */
		rc = iopt_pages_rw_access(pages, 0, buffer, len, 0);
		kthread_unuse_mm(mm);
		KUNIT_EXPECT_EQ(test, rc, 0);
		KUNIT_EXPECT_EQ(test, missed, 0UL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer, 0x5a, len), NULL);
		KUNIT_ASSERT_EQ(test, iopt_unmap_iova(&fixture->table, iova, len, &unmapped), 0);
		KUNIT_EXPECT_EQ(test, unmapped, len);
		KUNIT_EXPECT_EQ(test, (unsigned long)atomic64_read(&mm->pinned_vm), (len - (size < PAGE_SIZE ? 0 : size)) / PAGE_SIZE);
		iopt_table_remove_domain(&fixture->table, &fixture->domains[0].domain);
		KUNIT_EXPECT_EQ(test, fixture->domains[1].mapped, len - size);
		KUNIT_ASSERT_EQ(test, iopt_unmap_iova(&fixture->table, alias, len - size, &unmapped), 0);
		KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
		kunit_release_action(test, test_fragment_iopt_free, fixture);
		kunit_release_action(test, user4k_vma_mm_free, mm);
		kunit_info(test, "IOMMUFD %luK: relocated PA, batching, copy, two domains, VA replacement and unpin passed", size / SZ_1K);
	}
}

static void user4k_iommufd_rollback_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct test_fragment_iopt *fixture;
	unsigned long addr, iova = SZ_1M, size = SZ_4K;
	int rc;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm IOMMUFD requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	fixture = test_fragment_iopt_alloc(test);
	KUNIT_ASSERT_NOT_NULL(test, fixture);
	KUNIT_ASSERT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[0].domain), 0);
	KUNIT_ASSERT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[1].domain), 0);
	kthread_use_mm(mm);
	addr = vm_mmap(NULL, TEST_VA, 3 * size, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
	rc = vm_munmap(TEST_VA + size, size);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA);
	KUNIT_ASSERT_EQ(test, rc, 0);
	KUNIT_EXPECT_EQ(test, test_fragment_map_user(fixture, mm, addr, iova, 3 * size), -EFAULT);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	KUNIT_EXPECT_EQ(test, fixture->domains[0].mapped, 0UL);
	KUNIT_EXPECT_EQ(test, fixture->domains[1].mapped, 0UL);
	kthread_use_mm(mm);
	addr = vm_mmap(NULL, TEST_VA + size, size, PROT_READ,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA + size);
	KUNIT_EXPECT_LT(test, test_fragment_map_user(fixture, mm, TEST_VA, iova, 3 * size), 0);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	kthread_use_mm(mm);
	addr = vm_mmap(NULL, TEST_VA + size, size, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA + size);
	fixture->domains[1].fail_after = size;
	KUNIT_EXPECT_EQ(test, test_fragment_map_user(fixture, mm, TEST_VA, iova, 3 * size), -ENOMEM);
	KUNIT_EXPECT_EQ(test, fixture->domains[0].mapped, 0UL);
	KUNIT_EXPECT_EQ(test, fixture->domains[1].mapped, 0UL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	fixture->domains[1].fail_after = 0;
	KUNIT_ASSERT_EQ(test, test_fragment_map_user(fixture, mm, TEST_VA, iova, 3 * size), 0);
	KUNIT_EXPECT_EQ(test, fixture->domains[0].mapped, 3 * size);
	KUNIT_EXPECT_EQ(test, fixture->domains[1].mapped, 3 * size);
	kunit_release_action(test, test_fragment_iopt_free, fixture);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	kunit_release_action(test, user4k_vma_mm_free, mm);
}

static void user4k_iommufd_granule_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct test_fragment_iopt *fixture;
	unsigned long addr, unmapped, cut = SZ_1M + PAGE_SIZE - 1;
	int rc;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm IOMMUFD requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	fixture = test_fragment_iopt_alloc(test);
	KUNIT_ASSERT_NOT_NULL(test, fixture);
	fixture->domains[0].domain.pgsize_bitmap = PAGE_SIZE;
	KUNIT_ASSERT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[0].domain), 0);
	kthread_use_mm(mm);
	addr = vm_mmap(NULL, TEST_VA, 3 * PAGE_SIZE, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA);
	/* The device cannot cover a misaligned fragment by widening the mapping. */
	KUNIT_EXPECT_EQ(test, test_fragment_map_user(fixture, mm, TEST_VA + SZ_4K, SZ_1M, PAGE_SIZE), -EINVAL);
	KUNIT_EXPECT_EQ(test, fixture->domains[0].mapped, 0UL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	KUNIT_EXPECT_EQ(test, test_fragment_map_user(fixture, mm, TEST_VA, SZ_1M, SZ_4K), -EINVAL);
	/* Full contiguous native spans still work, including legacy area splitting. */
	KUNIT_ASSERT_EQ(test, iopt_disable_large_pages(&fixture->table), 0);
	KUNIT_ASSERT_EQ(test, test_fragment_map_user(fixture, mm, TEST_VA, SZ_1M, 2 * PAGE_SIZE), 0);
	KUNIT_EXPECT_EQ(test, fixture->domains[0].largest_map, PAGE_SIZE);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 2LL);
	KUNIT_ASSERT_EQ(test, iopt_cut_iova(&fixture->table, &cut, 1), 0);
	rc = iopt_unmap_iova(&fixture->table, SZ_1M, PAGE_SIZE, &unmapped);
	KUNIT_ASSERT_EQ(test, rc, 0);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 1LL);
	KUNIT_EXPECT_EQ(test, fixture->domains[0].mapped, PAGE_SIZE);
	KUNIT_ASSERT_EQ(test, iopt_unmap_all(&fixture->table, &unmapped), 0);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	/* A closed IOVA interval ending at ULONG_MAX must not wrap at map/unmap. */
	KUNIT_ASSERT_EQ(test, test_fragment_map_user(fixture, mm, TEST_VA,
				ULONG_MAX - 2 * PAGE_SIZE + 1, 2 * PAGE_SIZE), 0);
	KUNIT_ASSERT_EQ(test, iopt_unmap_all(&fixture->table, &unmapped), 0);
	KUNIT_EXPECT_EQ(test, unmapped, 2 * PAGE_SIZE);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	kunit_release_action(test, test_fragment_iopt_free, fixture);
	kunit_release_action(test, user4k_vma_mm_free, mm);
}

static void user4k_iommufd_shared_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct test_fragment_iopt *fixture;
	struct file *file;
	struct folio *folio;
	unsigned long addr, unmapped, iova = SZ_1M;
	unsigned char *buffer;
	loff_t pos = 0;
	phys_addr_t phys;
	void *ptr;
	int rc;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm IOMMUFD requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	file = test_cache_file(test, 2 * PAGE_SIZE, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	fixture = test_fragment_iopt_alloc(test);
	KUNIT_ASSERT_NOT_NULL(test, fixture);
	buffer = kunit_kmalloc(test, PAGE_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, buffer);
	KUNIT_ASSERT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[0].domain), 0);
	kthread_use_mm(mm);
	addr = vm_mmap(file, TEST_VA, 2 * SZ_4K, PROT_READ | PROT_WRITE,
			MAP_SHARED | MAP_FIXED_NOREPLACE, SZ_4K);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA);
	KUNIT_ASSERT_EQ(test, test_fragment_map_user(fixture, mm, TEST_VA, iova, 2 * SZ_4K), 0);
	folio = filemap_get_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	KUNIT_ASSERT_TRUE(test, folio_maybe_dma_pinned(folio));
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 1LL);
	folio_lock(folio);
	folio_clear_dirty_for_io(folio);
	folio_unlock(folio);
	phys = iommu_iova_to_phys(&fixture->domains[0].domain, iova);
	KUNIT_EXPECT_EQ(test, phys, PFN_PHYS(folio_pfn(folio)) + SZ_4K);
	ptr = kmap_local_folio(folio, SZ_4K);
	memset(ptr, 0x93, 2 * SZ_4K);
	kunmap_local(ptr);
	kthread_use_mm(mm);
	rc = vm_munmap(TEST_VA, 2 * SZ_4K);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, rc, 0);
	/* A failed second-domain attachment must retain the first domain's pins. */
	fixture->domains[1].fail_after = SZ_4K;
	KUNIT_EXPECT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[1].domain), -ENOMEM);
	KUNIT_EXPECT_EQ(test, fixture->domains[0].mapped, 2 * SZ_4K);
	KUNIT_EXPECT_EQ(test, fixture->domains[1].mapped, 0UL);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 1LL);
	KUNIT_EXPECT_TRUE(test, folio_maybe_dma_pinned(folio));
	fixture->domains[1].fail_after = 0;
	KUNIT_ASSERT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[1].domain), 0);
	KUNIT_ASSERT_EQ(test, iopt_unmap_all(&fixture->table, &unmapped), 0);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	KUNIT_EXPECT_FALSE(test, folio_maybe_dma_pinned(folio));
	KUNIT_EXPECT_TRUE(test, folio_test_dirty(folio));
	KUNIT_ASSERT_EQ(test, kernel_read(file, buffer, PAGE_SIZE, &pos), (ssize_t)PAGE_SIZE);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer, 0, SZ_4K), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer + SZ_4K, 0x93, 2 * SZ_4K), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer + 3 * SZ_4K, 0, PAGE_SIZE - 3 * SZ_4K), NULL);
	kunit_release_action(test, test_fragment_iopt_free, fixture);
	kunit_release_action(test, user4k_vma_mm_free, mm);
}

static void test_iommufd_exit_account(struct kunit *test, unsigned int shift)
{
	struct mm_struct *mm;
	struct test_fragment_iopt *fixture;
	struct iopt_pages *pages;
	struct iopt_area *area;
	unsigned long addr, unmapped;
	LIST_HEAD(keep);

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm IOMMUFD requires hardware opt-in");
		return;
	}
	mm = test_vma_mm_granule(test, shift);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	fixture = test_fragment_iopt_alloc(test);
	KUNIT_ASSERT_NOT_NULL(test, fixture);
	kthread_use_mm(mm);
	addr = vm_mmap(NULL, TEST_VA, 2 * PAGE_SIZE, PROT_READ | PROT_WRITE,
			MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA);
	KUNIT_ASSERT_EQ(test, test_fragment_map_user(fixture, mm, TEST_VA, SZ_1M, 2 * PAGE_SIZE), 0);
	down_read(&fixture->table.iova_rwsem);
	area = iopt_area_iter_first(&fixture->table, SZ_1M, SZ_1M);
	pages = area->pages;
	mutex_lock(&pages->mutex);
	/* Select VFIO-compatible accounting before the lazy range acquires pins. */
	pages->account_mode = IOPT_PAGES_ACCOUNT_MM;
	mutex_unlock(&pages->mutex);
	up_read(&fixture->table.iova_rwsem);
	KUNIT_ASSERT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[0].domain), 0);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 2LL);
	KUNIT_EXPECT_EQ(test, mm->locked_vm, 2 * (PAGE_SIZE >> MM_ACCOUNT_SHIFT));
	/* Keep the provider, and thus mm_count, after mm_users reaches zero. */
	KUNIT_ASSERT_EQ(test, iopt_get_pages(&fixture->table, SZ_1M, 2 * PAGE_SIZE, &keep), 0);
	kunit_release_action(test, user4k_vma_mm_free, mm);
	KUNIT_EXPECT_EQ(test, atomic_read(&mm->mm_users), 0);
	KUNIT_EXPECT_EQ(test, iopt_table_add_domain(&fixture->table, &fixture->domains[1].domain), 0);
	KUNIT_EXPECT_EQ(test, iopt_unmap_all(&fixture->table, &unmapped), 0);
	KUNIT_EXPECT_EQ(test, atomic64_read(&mm->pinned_vm), 0LL);
	KUNIT_EXPECT_EQ(test, mm->locked_vm, 0UL);
	KUNIT_EXPECT_EQ(test, pages->npinned, 0UL);
	KUNIT_EXPECT_EQ(test, pages->last_npinned, 0UL);
	iopt_free_pages_list(&keep);
	kunit_release_action(test, test_fragment_iopt_free, fixture);
}

static void user4k_iommufd_exit_account_test(struct kunit *test)
{
	test_iommufd_exit_account(test, PAGE_SHIFT);
	test_iommufd_exit_account(test, 12);
}

static void user4k_iova_bitmap_test(struct kunit *test)
{
	unsigned int shifts[] = { PAGE_SHIFT, 12, 14 };
	unsigned int mode;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm bitmap requires hardware opt-in");
		return;
	}
	for (mode = 0; mode < (PAGE_SHIFT > 14 ? 3U : 2U); mode++) {
		struct mm_struct *mm = test_vma_mm_granule(test, shifts[mode]);
		unsigned long size = 1UL << shifts[mode], len = 4 * size;
		unsigned long old = TEST_VA, dest = TEST_VA + 32 * PAGE_SIZE + size;
		unsigned long start = dest + size - sizeof(u64), bytes = 2 * size;
		unsigned long unit = SZ_4K, iova = 1UL << 32, addr, moved;
		unsigned char *actual, *expected;
		struct iova_bitmap *bitmap;
		unsigned long missed;
		int ret = 0;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		actual = kunit_kzalloc(test, len, GFP_KERNEL);
		expected = kunit_kzalloc(test, len, GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, actual);
		KUNIT_ASSERT_NOT_NULL(test, expected);
		kthread_use_mm(mm);
		addr = vm_mmap(NULL, old, len, PROT_READ | PROT_WRITE,
			       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
		missed = IS_ERR_VALUE(addr) ? len : copy_to_user((void __user *)addr, actual, len);
		moved = missed ? 0 : test_mremap(old, len, len,
					MREMAP_FIXED | MREMAP_MAYMOVE, dest);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, addr, old);
		KUNIT_ASSERT_EQ(test, missed, 0UL);
		KUNIT_ASSERT_EQ(test, moved, dest);
		/* VA and PA offsets differ after moving by one alternative leaf. */
		if (size < PAGE_SIZE) {
			pte_t pte = test_fault_entry(test, mm, dest);

			KUNIT_EXPECT_NE(test, pte_phys_mm(mm, pte) & ~PAGE_MASK,
					dest & ~PAGE_MASK);
		}
		kthread_use_mm(mm);
		bitmap = iova_bitmap_alloc(iova, bytes * 8 * unit, unit,
					   (u64 __user *)start);
		if (!IS_ERR(bitmap)) {
			ret = iova_bitmap_for_each(bitmap, &unit, test_iova_bitmap_fill);
			iova_bitmap_free(bitmap);
		} else {
			ret = PTR_ERR(bitmap);
		}
		missed = copy_from_user(actual, (void __user *)dest, len);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_ASSERT_EQ(test, missed, 0UL);
		memset(expected + size - sizeof(u64), 0xff, bytes);
		expected[size - sizeof(u64) + bytes - 1] = 1;
		KUNIT_EXPECT_EQ(test, memcmp(actual, expected, len), 0);
		kunit_release_action(test, user4k_vma_mm_free, mm);
		kunit_info(test, "IOVA bitmap relocated %luK user leaves checked", size / SZ_1K);
	}
}

static void test_iova_free(void *bitmap)
{
	iova_bitmap_free(bitmap);
}

static void test_iova_kvfree(void *buffer)
{
	kvfree(buffer);
}

static void user4k_iova_bitmap_window_test(struct kunit *test)
{
	struct mm_struct *mm;
	unsigned long capacity = PAGE_SIZE / sizeof(struct user_page_fragment);
	unsigned long bytes = (capacity + 3) * SZ_4K, len = bytes + 2 * SZ_4K;
	unsigned long start = TEST_VA + SZ_4K - sizeof(u64), unit = SZ_4K;
	unsigned long addr, missed;
	unsigned char *actual;
	struct iova_bitmap *bitmap;
	int ret;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm bitmap requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	actual = kvzalloc(len, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, actual);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_iova_kvfree, actual), 0);
	kthread_use_mm(mm);
	addr = vm_mmap(NULL, TEST_VA, len, PROT_READ | PROT_WRITE,
		       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
	bitmap = iova_bitmap_alloc(0, bytes * 8 * unit, unit, (u64 __user *)start);
	ret = IS_ERR(bitmap) ? PTR_ERR(bitmap) :
		iova_bitmap_for_each(bitmap, &unit, test_iova_bitmap_fill);
	if (!IS_ERR(bitmap))
		iova_bitmap_free(bitmap);
	missed = IS_ERR_VALUE(addr) ? len : copy_from_user(actual, (void __user *)addr, len);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, addr, TEST_VA);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, missed, 0UL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(actual, 0, SZ_4K - sizeof(u64)), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(actual + SZ_4K - sizeof(u64), 0xff, bytes - 1), NULL);
	KUNIT_EXPECT_EQ(test, actual[SZ_4K - sizeof(u64) + bytes - 1], (unsigned char)1);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(actual + SZ_4K - sizeof(u64) + bytes,
					  0, SZ_4K + sizeof(u64)), NULL);
	kunit_release_action(test, user4k_vma_mm_free, mm);
	kunit_info(test, "IOVA bitmap crossed %lu-record window and revisited first byte", capacity);
}

static void user4k_iova_bitmap_retained_test(struct kunit *test)
{
	unsigned int shifts[] = { PAGE_SHIFT, 12, 14 };
	unsigned int mode;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm bitmap requires hardware opt-in");
		return;
	}
	for (mode = 0; mode < (PAGE_SHIFT > 14 ? 3U : 2U); mode++) {
		struct mm_struct *mm = test_vma_mm_granule(test, shifts[mode]);
		unsigned long size = 1UL << shifts[mode], len = 2 * size;
		struct file *file = test_cache_file(test, 4 * PAGE_SIZE, false);
		unsigned char *buffer = kunit_kzalloc(test, len, GFP_KERNEL);
		unsigned long addr, replacement = 0, missed = 0;
		struct iova_bitmap *bitmap;
		struct folio *folio;
		loff_t pos = size;
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		KUNIT_ASSERT_NOT_NULL(test, buffer);
		kthread_use_mm(mm);
		addr = vm_mmap(file, TEST_VA, len, PROT_READ | PROT_WRITE,
			       MAP_SHARED | MAP_FIXED_NOREPLACE, size);
		bitmap = iova_bitmap_alloc(0, len * 8 * SZ_4K, SZ_4K,
					   (u64 __user *)TEST_VA);
		if (!IS_ERR(bitmap))
			iova_bitmap_set(bitmap, 3 * SZ_4K, SZ_4K);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, addr, TEST_VA);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, bitmap);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_iova_free, bitmap), 0);
		folio = filemap_get_folio(file->f_mapping, size >> PAGE_SHIFT);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
		KUNIT_ASSERT_TRUE(test, folio_maybe_dma_pinned(folio));
		kthread_use_mm(mm);
		ret = vm_munmap(TEST_VA, len);
		if (!ret) {
			replacement = vm_mmap(NULL, TEST_VA, len, PROT_READ | PROT_WRITE,
					     MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
			if (!IS_ERR_VALUE(replacement)) {
				memset(buffer, 0x5a, len);
				missed = copy_to_user((void __user *)replacement, buffer, len);
			}
		}
		/* The cached pins still refer to the old file, not the new VMA. */
		iova_bitmap_set(bitmap, 9 * SZ_4K, 2 * SZ_4K);
		if (!ret && !IS_ERR_VALUE(replacement))
			missed |= copy_from_user(buffer, (void __user *)replacement, len);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_ASSERT_EQ(test, replacement, TEST_VA);
		KUNIT_ASSERT_EQ(test, missed, 0UL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer, 0x5a, len), NULL);
		kunit_release_action(test, test_iova_free, bitmap);
		KUNIT_EXPECT_FALSE(test, folio_maybe_dma_pinned(folio));
		KUNIT_EXPECT_TRUE(test, folio_test_dirty(folio));
		KUNIT_ASSERT_EQ(test, kernel_read(file, buffer, len, &pos), (ssize_t)len);
		KUNIT_EXPECT_EQ(test, buffer[0], (unsigned char)8);
		KUNIT_EXPECT_EQ(test, buffer[1], (unsigned char)6);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer + 2, 0, len - 2), NULL);
		kunit_release_action(test, user4k_vma_mm_free, mm);
		kunit_info(test, "IOVA bitmap %luK shared backing retained across unmap/replacement", size / SZ_1K);
	}
}

static long test_mprotect(unsigned long start, unsigned long len, unsigned long prot);

static void user4k_iova_bitmap_partial_test(struct kunit *test)
{
	unsigned int shifts[] = { PAGE_SHIFT, 12, 14 };
	unsigned int mode;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm bitmap requires hardware opt-in");
		return;
	}
	for (mode = 0; mode < (PAGE_SHIFT > 14 ? 3U : 2U); mode++) {
		struct mm_struct *mm = test_vma_mm_granule(test, shifts[mode]);
		unsigned long size = 1UL << shifts[mode], len = 3 * size;
		unsigned char *buffer = kunit_kzalloc(test, len, GFP_KERNEL);
		unsigned long addr, refill, missed;
		struct iova_bitmap *bitmap;
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		KUNIT_ASSERT_NOT_NULL(test, buffer);
		kthread_use_mm(mm);
		addr = vm_mmap(NULL, TEST_VA, len, PROT_READ | PROT_WRITE,
			       MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
		missed = IS_ERR_VALUE(addr) ? len : copy_to_user((void __user *)addr, buffer, len);
		ret = vm_munmap(TEST_VA + size, size);
		bitmap = iova_bitmap_alloc(0, len * 8 * SZ_4K, SZ_4K, (u64 __user *)TEST_VA);
		if (!IS_ERR(bitmap))
			iova_bitmap_set(bitmap, 0, len * 8 * SZ_4K);
		missed |= copy_from_user(buffer, (void __user *)TEST_VA, size);
		missed |= copy_from_user(buffer + 2 * size, (void __user *)(TEST_VA + 2 * size), size);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, addr, TEST_VA);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_ASSERT_EQ(test, missed, 0UL);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, bitmap);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_iova_free, bitmap), 0);
		/* Preserve the existing void API: pin failure stops at the valid prefix. */
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer, 0xff, size), NULL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer + 2 * size, 0, size), NULL);
		kthread_use_mm(mm);
		refill = vm_mmap(NULL, TEST_VA + size, size, PROT_READ | PROT_WRITE,
				 MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
		iova_bitmap_set(bitmap, 0, len * 8 * SZ_4K);
		missed = copy_from_user(buffer, (void __user *)TEST_VA, len);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, refill, TEST_VA + size);
		KUNIT_ASSERT_EQ(test, missed, 0UL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer, 0xff, len), NULL);
		kunit_release_action(test, test_iova_free, bitmap);
		memset(buffer, 0, len);
		kthread_use_mm(mm);
		missed = copy_to_user((void __user *)TEST_VA, buffer, len);
		ret = test_mprotect(TEST_VA, len, PROT_READ);
		bitmap = iova_bitmap_alloc(0, len * 8 * SZ_4K, SZ_4K, (u64 __user *)TEST_VA);
		if (!IS_ERR(bitmap)) {
			iova_bitmap_set(bitmap, 0, len * 8 * SZ_4K);
			iova_bitmap_free(bitmap);
		}
		missed |= copy_from_user(buffer, (void __user *)TEST_VA, len);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_ASSERT_EQ(test, missed, 0UL);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, bitmap);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer, 0, len), NULL);
		kunit_release_action(test, user4k_vma_mm_free, mm);
		kunit_info(test, "IOVA bitmap %luK partial pin, retry and read-only checks passed", size / SZ_1K);
	}
}

static u64 test_table_descriptor(struct mm_struct *mm, unsigned long addr, bool pud)
{
	pgd_t *pgdp = pgd_offset(mm, addr);
	p4d_t *p4dp = p4d_offset_mm(mm, pgdp, addr);
	pud_t *pudp = pud_offset_mm(mm, p4dp, addr);

	return pud ? pud_val(*pudp) : pmd_val(*pmd_offset_mm(mm, pudp, addr));
}

static void user4k_mremap_move_test(struct kunit *test)
{
	static const struct { unsigned long old, new, len; unsigned int level; } cases[] = {
		{ TEST_VA + SZ_4K, 3 * SZ_2M + 2 * SZ_4K, 3 * SZ_4K, 0 },
		{ SZ_2M - SZ_4K, SZ_1G - 2 * SZ_4K, 3 * SZ_4K, 0 },
		{ SZ_1G - SZ_4K, BIT_ULL(39) - 2 * SZ_4K, 3 * SZ_4K, 0 },
		{ 4 * SZ_2M, 8 * SZ_2M, SZ_2M, 1 },
		{ 2UL * SZ_1G, 4UL * SZ_1G, SZ_1G, 2 },
	};
	unsigned int shape;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mremap fixture requires hardware opt-in");
		return;
	}
	for (shape = 0; shape < ARRAY_SIZE(cases); shape++) {
		struct mm_struct *mm = test_vma_mm(test);
		unsigned long old = cases[shape].old, new = cases[shape].new;
		unsigned long len = cases[shape].len;
		unsigned long offsets[] = { 0, SZ_4K, len - SZ_4K };
		unsigned long result, warm, moved, vm_before, commit_before, rss_before;
		unsigned long flags = MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE;
		unsigned long old_left[3], new_left[3];
		phys_addr_t phys[3];
		u8 readback[3], value = 0x79, byte;
		u64 table = 0;
		unsigned int i;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		if (cases[shape].level)
			flags |= MAP_NORESERVE;
		kthread_use_mm(mm);
		warm = vm_mmap(NULL, SZ_1M, SZ_4K, PROT_READ | PROT_WRITE, flags, 0);
		result = vm_mmap(NULL, old, len, PROT_READ | PROT_WRITE, flags, 0);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, warm, SZ_1M);
		KUNIT_ASSERT_EQ(test, result, old);
		KUNIT_ASSERT_EQ(test, access_remote_vm(mm, warm, &value, 1, FOLL_WRITE), 1);
		for (i = 0; i < 3; i++) {
			value = 0x80 + i;
			KUNIT_ASSERT_EQ(test, access_remote_vm(mm, old + offsets[i], &value, 1, FOLL_WRITE), 1);
			phys[i] = pte_phys_mm(mm, test_fault_entry(test, mm, old + offsets[i]));
		}
		if (cases[shape].level) {
			table = test_table_descriptor(mm, old, cases[shape].level == 2);
			/* Warm-up consumed the first fragment; the moved table is packed. */
			KUNIT_EXPECT_NE(test, table & (3UL * SZ_4K), 0ULL);
		}
		vm_before = mm->total_vm;
		commit_before = mm->committed_user_pages;
		rss_before = get_mm_rss_sum(mm);
		kthread_use_mm(mm);
		pagefault_disable();
		for (i = 0; i < 3; i++)
			old_left[i] = copy_from_user(&byte, (void __user *)(old + offsets[i]), 1);
		pagefault_enable();
		moved = test_mremap(old, len, len, MREMAP_FIXED | MREMAP_MAYMOVE, new);
		pagefault_disable();
		for (i = 0; i < 3; i++) {
			new_left[i] = copy_from_user(&readback[i], (void __user *)(new + offsets[i]), 1);
			old_left[i] += !copy_from_user(&byte, (void __user *)(old + offsets[i]), 1);
		}
		pagefault_enable();
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, moved, new);
		KUNIT_EXPECT_EQ(test, mm->total_vm, vm_before);
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, commit_before);
		KUNIT_EXPECT_EQ(test, get_mm_rss_sum(mm), rss_before);
		for (i = 0; i < 3; i++) {
			KUNIT_EXPECT_EQ(test, old_left[i], 0UL);
			KUNIT_EXPECT_EQ(test, new_left[i], 0UL);
			KUNIT_EXPECT_EQ(test, readback[i], (u8)(0x80 + i));
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, test_fault_entry(test, mm, new + offsets[i])), phys[i]);
		}
		if (cases[shape].level)
			KUNIT_EXPECT_EQ(test, test_table_descriptor(mm, new, cases[shape].level == 2), table);
	}
}

static void user4k_stack_relocate_test(struct kunit *test)
{
	const unsigned long shifts[] = { SZ_4K, SZ_2M + SZ_4K, SZ_1G + 3 * SZ_4K };
	unsigned int shape, i;

	for (shape = 0; shape < ARRAY_SIZE(shifts); shape++) {
		struct mm_struct *mm = test_vma_mm(test);
		struct vm_area_struct *vma;
		struct vm_page_offset pos;
		unsigned long top, old, new;
		phys_addr_t phys[3];
		u8 bytes[3] = { 0x42, 0x73, 0xb5 };
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		KUNIT_ASSERT_EQ(test, create_init_stack_vma(mm, &vma, &top), 0);
		old = vma->vm_end - 3 * SZ_4K;
		new = old - shifts[shape];
		mmap_write_lock(mm);
		ret = expand_stack_locked(vma, old);
		mmap_write_unlock(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm->total_vm, 3UL);
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 3UL);
		for (i = 0; i < 3; i++) {
			KUNIT_ASSERT_EQ(test, access_remote_vm(mm, old + i * SZ_4K,
						&bytes[i], 1, FOLL_WRITE), 1);
			phys[i] = pte_phys_mm(mm, test_fault_entry(test, mm, old + i * SZ_4K));
		}
		pos = vma_page_offset_at(vma, old);
		mmap_write_lock(mm);
		ret = relocate_vma_down(vma, shifts[shape]);
		mmap_write_unlock(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, vma->vm_start, new);
		KUNIT_EXPECT_EQ(test, vma->vm_end, new + 3 * SZ_4K);
		KUNIT_EXPECT_TRUE(test, vm_page_offset_equal(vma_page_offset_at(vma, new), pos));
		KUNIT_EXPECT_EQ(test, mm->total_vm, 3UL);
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 3UL);
		for (i = 0; i < 3; i++) {
			u8 byte = 0;

			KUNIT_EXPECT_EQ(test, access_remote_vm(mm, new + i * SZ_4K, &byte, 1, 0), 1);
			KUNIT_EXPECT_EQ(test, byte, bytes[i]);
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, test_fault_entry(test, mm, new + i * SZ_4K)), phys[i]);
		}
		/* Grow after relocation: keep the file-like anon offset linear. */
		mmap_write_lock(mm);
		vm_flags_clear(vma, VM_STACK_INCOMPLETE_SETUP);
		ret = expand_stack_locked(vma, new - SZ_4K);
		mmap_write_unlock(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm->total_vm, 4UL);
		KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 4UL);
		KUNIT_EXPECT_EQ(test, access_remote_vm(mm, new - SZ_4K, &bytes[0], 1, FOLL_WRITE), 1);
		KUNIT_EXPECT_TRUE(test, vm_page_offset_equal(vma_page_offset_at(vma, new), pos));
	}
}

static void user4k_mremap_cow_test(struct kunit *test)
{
	struct mm_struct *parent, *child;
	unsigned long old = TEST_VA + SZ_4K, new = SZ_2M - 2 * SZ_4K, mapped;
	unsigned long moved;
	u8 value = 0x29, byte;
	int ret;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mremap fixture requires hardware opt-in");
		return;
	}
	parent = test_vma_mm(test);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, parent);
	KUNIT_ASSERT_NOT_NULL(test, child);
	kthread_use_mm(parent);
	mapped = vm_mmap(NULL, old, 3 * SZ_4K, PROT_READ | PROT_WRITE,
			 MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(parent);
	KUNIT_ASSERT_EQ(test, mapped, old);
	KUNIT_ASSERT_EQ(test, access_remote_vm(parent, old + SZ_4K, &value, 1, FOLL_WRITE), 1);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	kthread_use_mm(child);
	/* Move only the middle quarter, leaving both neighbors mapped. */
	moved = test_mremap(old + SZ_4K, SZ_4K, SZ_4K,
			    MREMAP_FIXED | MREMAP_MAYMOVE, new);
	kthread_unuse_mm(child);
	KUNIT_ASSERT_EQ(test, moved, new);
	value = 0xa4;
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, new, &value, 1, FOLL_WRITE), 1);
	KUNIT_EXPECT_EQ(test, access_remote_vm(parent, old + SZ_4K, &byte, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, byte, (u8)0x29);
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, new, &byte, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, byte, value);
	KUNIT_EXPECT_EQ(test, mm_pages_to_bytes(child, child->total_vm), 3UL * SZ_4K);
	KUNIT_EXPECT_EQ(test, child->committed_user_pages, 3UL);
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, old, &value, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, old + 2 * SZ_4K, &value, 1, 0), 1);
}

static void user4k_mremap_resize_test(struct kunit *test)
{
	struct mm_struct *mm;
	unsigned long old = TEST_VA + SZ_4K, new = TEST_VA + 8 * PAGE_SIZE + 2 * SZ_4K;
	unsigned long res[7];
	u8 value = 0x98, byte = 0;
	int unmap[2];

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mremap fixture requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	kthread_use_mm(mm);
	res[0] = vm_mmap(NULL, old, SZ_4K, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
	res[1] = test_mremap(old, SZ_4K, 3 * SZ_4K - 1, 0, 0);
	res[2] = test_mremap(old + 1, SZ_4K, SZ_4K, 0, 0);
	res[3] = test_mremap(old, 3 * SZ_4K, 3 * SZ_4K,
			MREMAP_FIXED | MREMAP_MAYMOVE, new + 1);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, res[0], old);
	KUNIT_ASSERT_EQ(test, res[1], old);
	KUNIT_EXPECT_EQ(test, res[2], (unsigned long)-EINVAL);
	KUNIT_EXPECT_EQ(test, res[3], (unsigned long)-EINVAL);
	KUNIT_EXPECT_EQ(test, mm->total_vm, 3UL);
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 3UL);
	KUNIT_ASSERT_EQ(test, access_remote_vm(mm, old + SZ_4K, &value, 1, FOLL_WRITE), 1);
	kthread_use_mm(mm);
	res[4] = test_mremap(old, 3 * SZ_4K, 2 * SZ_4K, 0, 0);
	res[5] = test_mremap(old, 2 * SZ_4K, 2 * SZ_4K,
		MREMAP_FIXED | MREMAP_MAYMOVE | MREMAP_DONTUNMAP, new);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, res[4], old);
	KUNIT_ASSERT_EQ(test, res[5], new);
	KUNIT_EXPECT_EQ(test, mm->total_vm, 4UL);
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 4UL);
	KUNIT_EXPECT_EQ(test, access_remote_vm(mm, new + SZ_4K, &byte, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, byte, value);
	KUNIT_EXPECT_EQ(test, access_remote_vm(mm, old + SZ_4K, &byte, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, byte, (u8)0);
	kthread_use_mm(mm);
	unmap[0] = vm_munmap(old, 2 * SZ_4K);
	unmap[1] = vm_munmap(new, 2 * SZ_4K);
	kthread_unuse_mm(mm);
	KUNIT_EXPECT_EQ(test, unmap[0], 0);
	KUNIT_EXPECT_EQ(test, unmap[1], 0);
	KUNIT_EXPECT_EQ(test, mm->total_vm, 0UL);
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, 0UL);
}

static int test_reject_mremap(struct vm_area_struct *vma)
{
	return -EIO;
}

static void user4k_cold_batch_test(struct kunit *test)
{
	struct mm_struct *mm, *child;
	struct vm_area_struct *vma;
	unsigned long start = TEST_VA, mapped;
	spinlock_t *ptl;
	pte_t *ptep, entry;
	u8 byte = 0x68;
	unsigned int i;
	int ret;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm cold fixture requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	KUNIT_ASSERT_NOT_NULL(test, child);
	kthread_use_mm(mm);
	mapped = vm_mmap(NULL, start, PAGE_SIZE, PROT_READ | PROT_WRITE,
			 MAP_ANONYMOUS | MAP_PRIVATE | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, mapped, start);
	KUNIT_ASSERT_EQ(test, access_remote_vm(mm, start, &byte, 1, FOLL_WRITE), 1);
	/* Only the last quarter is young: batching must aggregate that bit. */
	mmap_read_lock(mm);
	vma = vma_lookup(mm, start);
	for (i = 0; i < 3; i++) {
		unsigned long addr = start + i * SZ_4K;

		ptep = test_mm_lookup(mm, addr, &ptl);
		if (!ptep) {
			mmap_read_unlock(mm);
			KUNIT_FAIL(test, "missing cold test PTE");
			return;
		}
		entry = ptep_clear_flush(vma, addr, ptep);
		set_pte_at(mm, addr, ptep, pte_mkold(entry));
		pte_unmap_unlock(ptep, ptl);
	}
	mmap_read_unlock(mm);
	kthread_use_mm(mm);
	ret = do_madvise(mm, start + 3 * SZ_4K, SZ_4K, MADV_COLD);
	kthread_unuse_mm(mm);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_TRUE(test, pte_young(test_fault_entry(test, mm, start + 3 * SZ_4K)));
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	kthread_use_mm(mm);
	ret = do_madvise(mm, start, PAGE_SIZE, MADV_COLD);
	kthread_unuse_mm(mm);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_TRUE(test, pte_young(test_fault_entry(test, mm, start + 3 * SZ_4K)));
	kunit_release_action(test, user4k_vma_mm_free, child);
	kthread_use_mm(mm);
	ret = do_madvise(mm, start, PAGE_SIZE, MADV_COLD);
	kthread_unuse_mm(mm);
	KUNIT_EXPECT_EQ(test, ret, 0);
	for (i = 0; i < 4; i++)
		KUNIT_EXPECT_FALSE(test, pte_young(test_fault_entry(test, mm, start + i * SZ_4K)));
	byte = 0;
	KUNIT_EXPECT_EQ(test, access_remote_vm(mm, start, &byte, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, byte, (u8)0x68);
}

static void user4k_prefault_dirty_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct folio *folio;
	unsigned long start = TEST_VA + SZ_4K, mapped;
	spinlock_t *ptl;
	pte_t *ptep, entry;
	u8 byte = 0x7a;
	int locked = 1;
	long ret;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm prefault fixture requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	kthread_use_mm(mm);
	mapped = vm_mmap(NULL, start, SZ_4K, PROT_READ | PROT_WRITE,
			 MAP_ANONYMOUS | MAP_PRIVATE | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, mapped, start);
	KUNIT_ASSERT_EQ(test, access_remote_vm(mm, start, &byte, 1, FOLL_WRITE), 1);
	mmap_read_lock(mm);
	ptep = test_mm_lookup(mm, start, &ptl);
	if (!ptep) {
		mmap_read_unlock(mm);
		KUNIT_FAIL(test, "missing prefault test PTE");
		return;
	}
	entry = ptep_clear_flush(vma_lookup(mm, start), start, ptep);
	folio = page_folio(pte_page(entry));
	folio_get(folio);
	set_pte_at(mm, start, ptep, pte_mkclean(entry));
	folio_clear_dirty(folio);
	pte_unmap_unlock(ptep, ptl);
	mmap_read_unlock(mm);
	kthread_use_mm(mm);
	mmap_read_lock(mm);
	ret = faultin_page_range(mm, start, start + SZ_4K, true, &locked);
	if (locked)
		mmap_read_unlock(mm);
	kthread_unuse_mm(mm);
	KUNIT_EXPECT_EQ(test, ret, 1L);
	KUNIT_EXPECT_TRUE(test, folio_test_dirty(folio));
	folio_put(folio);
}

struct test_fsync_record {
	loff_t start, end;
	unsigned int calls;
	int datasync;
};

static int test_fsync_range(struct file *file, loff_t start, loff_t end, int datasync)
{
	struct test_fsync_record *record = file->private_data;

	*record = (struct test_fsync_record) { start, end, record->calls + 1, datasync };
	return 0;
}

asmlinkage long __arm64_sys_msync(const struct pt_regs *regs);

static void user4k_msync_offsets_test(struct kunit *test)
{
	struct file *file;
	struct vm_area_struct *vma;
	struct file_operations ops;
	const struct file_operations *original_ops;
	void *original_private;
	struct test_fsync_record records[3] = {}, record = {};
	struct pt_regs regs = { .regs = { TEST_VA + SZ_4K, 1, MS_SYNC }, .orig_x0 = TEST_VA + SZ_4K };
	long result[3];
	unsigned int i;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm sync fixture requires hardware opt-in");
		return;
	}
	file = test_cache_file(test, 3 * PAGE_SIZE, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	vma = test_file_vma(test, file, false, true, TEST_VA + SZ_4K,
			    5 * SZ_4K, 3 * SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	original_ops = file->f_op;
	original_private = file->private_data;
	ops = *original_ops;
	ops.fsync = test_fsync_range;
	file->f_op = &ops;
	file->private_data = &record;
	kthread_use_mm(vma->vm_mm);
	for (i = 0; i < ARRAY_SIZE(result); i++) {
		regs.regs[0] = vma->vm_start + i * SZ_4K;
		regs.orig_x0 = regs.regs[0];
		result[i] = __arm64_sys_msync(&regs);
		records[i] = record;
	}
	kthread_unuse_mm(vma->vm_mm);
	file->f_op = original_ops;
	file->private_data = original_private;
	for (i = 0; i < ARRAY_SIZE(result); i++) {
		KUNIT_EXPECT_EQ(test, result[i], 0L);
		KUNIT_EXPECT_EQ(test, records[i].calls, i + 1);
		KUNIT_EXPECT_EQ(test, records[i].start, (loff_t)(3 + i) * SZ_4K);
		KUNIT_EXPECT_EQ(test, records[i].end, (loff_t)(4 + i) * SZ_4K - 1);
		KUNIT_EXPECT_EQ(test, records[i].datasync, 1);
	}
}

static void user4k_mremap_file_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	struct file *file;
	struct vm_operations_struct rejecting_ops;
	const struct vm_operations_struct *original_ops;
	unsigned long old = TEST_VA + SZ_4K, new = SZ_1G - SZ_4K;
	unsigned long mapped, moved;
	u8 bytes[3];
	unsigned int i;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mremap fixture requires hardware opt-in");
		return;
	}
	file = test_cache_file(test, 2 * PAGE_SIZE, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	kthread_use_mm(mm);
	mapped = vm_mmap(file, old, 3 * SZ_4K, PROT_READ | PROT_WRITE,
			 MAP_SHARED | MAP_FIXED_NOREPLACE, 3 * SZ_4K);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, mapped, old);
	for (i = 0; i < 3; i++)
		KUNIT_ASSERT_EQ(test, access_remote_vm(mm, old + i * SZ_4K, &bytes[i], 1, 0), 1);
	vma = test_vma_lookup(mm, old);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	original_ops = vma->vm_ops;
	rejecting_ops = *original_ops;
	rejecting_ops.mremap = test_reject_mremap;
	mmap_write_lock(mm);
	vma->vm_ops = &rejecting_ops;
	mmap_write_unlock(mm);
	kthread_use_mm(mm);
	moved = test_mremap(old, 3 * SZ_4K, 3 * SZ_4K,
			    MREMAP_FIXED | MREMAP_MAYMOVE, new);
	kthread_unuse_mm(mm);
	/* Restore stack-owned callbacks before any assertion can leave this scope. */
	mmap_write_lock(mm);
	{
		VMA_ITERATOR(vmi, mm, 0);
		struct vm_area_struct *part;

		for_each_vma(vmi, part)
			if (part->vm_ops == &rejecting_ops)
				part->vm_ops = original_ops;
	}
	mmap_write_unlock(mm);
	KUNIT_ASSERT_EQ(test, moved, (unsigned long)-EIO);
	for (i = 0; i < 3; i++) {
		u8 byte = 0;

		KUNIT_EXPECT_EQ(test, access_remote_vm(mm, old + i * SZ_4K, &byte, 1, 0), 1);
		KUNIT_EXPECT_EQ(test, byte, bytes[i]);
	}
	KUNIT_EXPECT_EQ(test, mm->map_count, 1);
	KUNIT_EXPECT_EQ(test, mm->total_vm, 3UL);
	kthread_use_mm(mm);
	moved = test_mremap(old, 3 * SZ_4K, 3 * SZ_4K,
			    MREMAP_FIXED | MREMAP_MAYMOVE, new);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, moved, new);
	vma = test_vma_lookup(mm, new);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_EXPECT_EQ(test, vma_subpage_offset(vma), 3U * SZ_4K);
	for (i = 0; i < 3; i++) {
		u8 byte = 0;

		KUNIT_EXPECT_EQ(test, access_remote_vm(mm, new + i * SZ_4K, &byte, 1, 0), 1);
		KUNIT_EXPECT_EQ(test, byte, bytes[i]);
	}
}

static void user4k_exec_arguments_test(struct kunit *test)
{
	unsigned int native;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm stack setup requires hardware opt-in");
		return;
	}
	for (native = 0; native < 2; native++) {
		struct linux_binprm *bprm = kunit_kzalloc(test, sizeof(*bprm), GFP_KERNEL);
		struct mm_struct *mm = native ? mm_alloc() : test_vma_mm(test);
		unsigned long size, mask, initial, copied, new_top;
		unsigned long a_len = 2 * PAGE_SIZE + SZ_4K + 31, b_len = SZ_4K + 43;
		char *a, *b, *buf, *too_long;
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, bprm);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		if (native)
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
		size = mm_page_size(mm);
		mask = mm_page_mask(mm);
		a_len = min(a_len, 16 * size + 31);
		bprm->mm = mm;
		KUNIT_ASSERT_EQ(test, create_init_stack_vma(mm, &bprm->vma, &bprm->p), 0);
		initial = bprm->p;
		bprm->argmin = initial - SZ_2M;
		bprm->rlim_stack.rlim_cur = SZ_8M;
		bprm->rlim_stack.rlim_max = RLIM_INFINITY;
		a = kunit_kmalloc(test, a_len + 1, GFP_KERNEL);
		b = kunit_kmalloc(test, b_len + 1, GFP_KERNEL);
		buf = kunit_kmalloc(test, a_len + b_len + 7, GFP_KERNEL);
		too_long = kunit_kmalloc(test, 32 * size + 1, GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, a);
		KUNIT_ASSERT_NOT_NULL(test, b);
		KUNIT_ASSERT_NOT_NULL(test, buf);
		KUNIT_ASSERT_NOT_NULL(test, too_long);
		memset(a, 'A', a_len);
		a[a_len] = 0;
		memset(b, 'b', b_len);
		b[b_len] = 0;
		memset(too_long, 'x', 32 * size);
		too_long[32 * size] = 0;
		KUNIT_EXPECT_EQ(test, copy_string_kernel(too_long, bprm), -E2BIG);
		KUNIT_EXPECT_EQ(test, bprm->p, initial);
		KUNIT_ASSERT_EQ(test, copy_string_kernel("tail", bprm), 0);
		bprm->exec = bprm->p;
		KUNIT_ASSERT_EQ(test, copy_string_kernel(a, bprm), 0);
		KUNIT_ASSERT_EQ(test, copy_string_kernel(b, bprm), 0);
		bprm->argc = 3;
		copied = initial - bprm->p;
		KUNIT_EXPECT_EQ(test, copied, a_len + b_len + 7);
		KUNIT_ASSERT_EQ(test, access_remote_vm(mm, bprm->p, buf, copied, 0), (int)copied);
		KUNIT_EXPECT_MEMEQ(test, buf, b, b_len + 1);
		KUNIT_EXPECT_MEMEQ(test, buf + b_len + 1, a, a_len + 1);
		KUNIT_EXPECT_STREQ(test, buf + b_len + a_len + 2, "tail");
		new_top = (bprm->vma->vm_end - SZ_2M - size) & mask;
		kthread_use_mm(mm);
		ret = setup_arg_pages(bprm, new_top, EXSTACK_DISABLE_X);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		/* Subtract < PAGE_SIZE, then round down to 16 bytes and up to a leaf. */
		KUNIT_EXPECT_LE(test, bprm->vma->vm_end, new_top);
		KUNIT_EXPECT_LE(test, new_top - bprm->vma->vm_end, PAGE_SIZE);
		KUNIT_EXPECT_EQ(test, bprm->vma->vm_end & ~mask, 0UL);
		KUNIT_EXPECT_EQ(test, mm->start_stack, bprm->p);
		KUNIT_ASSERT_EQ(test, access_remote_vm(mm, bprm->p, buf, copied, 0), (int)copied);
		KUNIT_EXPECT_MEMEQ(test, buf, b, b_len + 1);
		KUNIT_EXPECT_MEMEQ(test, buf + b_len + 1, a, a_len + 1);
		KUNIT_EXPECT_STREQ(test, buf + b_len + a_len + 2, "tail");
		/* /proc/PID/cmdline and environ read the stack with FOLL_ANON. */
		memset(buf, 0, copied);
		KUNIT_ASSERT_EQ(test, access_remote_vm(mm, bprm->p, buf, copied, FOLL_ANON),
				(int)copied);
		KUNIT_EXPECT_MEMEQ(test, buf, b, b_len + 1);
		KUNIT_EXPECT_STREQ(test, buf + b_len + a_len + 2, "tail");
		/* Interpreter argument removal scans several user granules. */
		initial = bprm->p;
		KUNIT_ASSERT_EQ(test, remove_arg_zero(bprm), 0);
		KUNIT_EXPECT_EQ(test, bprm->p, initial + b_len + 1);
		KUNIT_ASSERT_EQ(test, remove_arg_zero(bprm), 0);
		KUNIT_EXPECT_EQ(test, bprm->p, initial + b_len + a_len + 2);
		KUNIT_ASSERT_EQ(test, remove_arg_zero(bprm), 0);
		KUNIT_EXPECT_EQ(test, bprm->p, initial + copied);
		KUNIT_EXPECT_EQ(test, bprm->argc, 0);
		KUNIT_EXPECT_EQ(test, bprm->vma_pages, 0UL);
	}
}

/* Stage two strings at the top of a fresh exec stack in @mm. */
static unsigned long test_stage_exec_strings(struct kunit *test,
		struct linux_binprm *bprm, struct mm_struct *mm)
{
	unsigned long initial;

	bprm->mm = mm;
	KUNIT_ASSERT_EQ(test, create_init_stack_vma(mm, &bprm->vma, &bprm->p), 0);
	initial = bprm->p;
	bprm->argmin = initial - SZ_2M;
	bprm->rlim_stack.rlim_cur = SZ_8M;
	bprm->rlim_stack.rlim_max = RLIM_INFINITY;
	KUNIT_ASSERT_EQ(test, copy_string_kernel("arg", bprm), 0);
	KUNIT_ASSERT_EQ(test, copy_string_kernel("setid-image", bprm), 0);
	return initial - bprm->p;
}

static void test_expect_exec_strings(struct kunit *test,
		struct linux_binprm *bprm, unsigned long copied)
{
	char buf[16] = {};

	KUNIT_ASSERT_EQ(test, copied, sizeof("setid-image") + sizeof("arg"));
	KUNIT_ASSERT_EQ(test, access_remote_vm(bprm->mm, bprm->p, buf, copied, 0),
			(int)copied);
	KUNIT_EXPECT_STREQ(test, buf, "setid-image");
	KUNIT_EXPECT_STREQ(test, buf + sizeof("setid-image"), "arg");
}

/*
 * A set-user-ID, set-group-ID or secure-exec image must not run with the
 * caller's unprivileged granule preference: it is restaged at the granule
 * it gets without one, with the staged arguments intact.
 */
static void user4k_exec_privileged_test(struct kunit *test)
{
	unsigned int privileged;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm stack setup requires hardware opt-in");
		return;
	}
	for (privileged = 0; privileged < 3; privileged++) {
		struct linux_binprm *bprm = kunit_kzalloc(test, sizeof(*bprm), GFP_KERNEL);
		struct mm_struct *mm = test_vma_mm(test);
		unsigned long copied;

		KUNIT_ASSERT_NOT_NULL(test, bprm);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		copied = test_stage_exec_strings(test, bprm, mm);
		bprm->secure_page_shift = PAGE_SHIFT;
		if (privileged == 1)
			bprm->per_clear = PER_CLEAR_ON_SETID;
		else if (privileged == 2)
			bprm->secureexec = 1;
		/* A replacement drops the staging mm's reference. */
		mmget(mm);
		KUNIT_ASSERT_EQ(test, bprm_secure_page_size(bprm), 0);
		if (bprm->mm != mm)
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
					user4k_vma_mm_free, bprm->mm), 0);
		else
			mmput(mm);
		KUNIT_EXPECT_EQ(test, mm_page_shift(bprm->mm),
				privileged ? PAGE_SHIFT : 12U);
		KUNIT_EXPECT_PTR_EQ(test, bprm->vma->vm_mm, bprm->mm);
		test_expect_exec_strings(test, bprm, copied);
	}
}

/* The arch selection records the preference-free granule for begin_new_exec(). */
static void user4k_exec_secure_granule_test(struct kunit *test)
{
	unsigned int saved = current->exec_page_shift;
	unsigned int compat;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm stack setup requires hardware opt-in");
		return;
	}
	if (!cpus_have_final_cap(ARM64_HAS_USER4K_GRANULE)) {
		kunit_skip(test, "no 4K translation granule");
		return;
	}
	for (compat = 0; compat < 2; compat++) {
		struct linux_binprm *bprm = kunit_kzalloc(test, sizeof(*bprm), GFP_KERNEL);
		struct mm_struct *mm = mm_alloc_exec_page_shift(PAGE_SHIFT);
		unsigned long copied;
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, bprm);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		copied = test_stage_exec_strings(test, bprm, mm);
		current->exec_page_shift = 12;
		ret = arm64_bprm_prepare_mm(bprm, compat);
		current->exec_page_shift = saved;
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, mm_page_shift(bprm->mm), 12U);
		/* AArch32 always uses 4K when available; AArch64 defaults to native. */
		KUNIT_EXPECT_EQ(test, bprm->secure_page_shift, compat ? 12U : PAGE_SHIFT);
		bprm->per_clear = PER_CLEAR_ON_SETID;
		mm = bprm->mm;
		/* Keep the staging mm alive across a possible replacement. */
		mmget(mm);
		KUNIT_ASSERT_EQ(test, bprm_secure_page_size(bprm), 0);
		mmput(mm);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free,
				bprm->mm), 0);
		KUNIT_EXPECT_EQ(test, mm_page_shift(bprm->mm), compat ? 12U : PAGE_SHIFT);
		test_expect_exec_strings(test, bprm, copied);
	}
}

/* HugeTLB tables use native geometry; a small-granule mm must not map them. */
static void user4k_hugetlb_refused_test(struct kunit *test)
{
#ifdef CONFIG_HUGETLBFS
	const unsigned long prot = PROT_READ | PROT_WRITE;
	const unsigned long flags = MAP_SHARED | MAP_NORESERVE | MAP_FIXED_NOREPLACE;
	const unsigned long addr = SZ_1G;
	struct mm_struct *small, *native;
	unsigned long len, result[3];
	struct file *file;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mapping fixture requires hardware opt-in");
		return;
	}
	len = huge_page_size(&default_hstate);
	file = hugetlb_file_setup(HUGETLB_ANON_FILE, len, mk_vma_flags(VMA_NORESERVE_BIT),
				  HUGETLB_ANONHUGE_INODE, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	small = test_vma_mm(test);
	native = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, native), 0);

	kthread_use_mm(small);
	result[0] = vm_mmap(file, addr, len, prot, flags, 0);
	result[1] = ksys_mmap_pgoff(addr, len, prot, MAP_PRIVATE | MAP_ANONYMOUS |
				    MAP_HUGETLB | MAP_NORESERVE | MAP_FIXED_NOREPLACE,
				    -1, 0);
	kthread_unuse_mm(small);
	kthread_use_mm(native);
	result[2] = vm_mmap(file, addr, len, prot, flags, 0);
	kthread_unuse_mm(native);
	fput(file);

	KUNIT_EXPECT_EQ(test, result[0], (unsigned long)-EINVAL);
	KUNIT_EXPECT_EQ(test, result[1], (unsigned long)-EINVAL);
	KUNIT_EXPECT_EQ(test, small->map_count, 0);
	/* The same mapping is accepted with native tables. */
	KUNIT_EXPECT_EQ(test, result[2], addr);
#else
	kunit_skip(test, "requires CONFIG_HUGETLBFS");
#endif
}

static struct vm_area_struct *test_special_vma(struct mm_struct *mm,
		const struct vm_special_mapping *sm, unsigned long start, unsigned long len,
		vm_flags_t flags)
{
	struct vm_area_struct *vma;

	mmap_write_lock(mm);
	vma = _install_special_mapping(mm, start, len, flags, sm);
	mmap_write_unlock(mm);
	return vma;
}

static void user4k_special_mapping_test(struct kunit *test)
{
	struct vm_special_mapping *sm = kunit_kzalloc(test, sizeof(*sm), GFP_KERNEL);
	struct page **pages = kunit_kcalloc(test, 3, sizeof(*pages), GFP_KERNEL);
	void *data[2] = { test_alloc_page(test), test_alloc_page(test) };
	struct mm_struct *small = test_vma_mm(test), *cold = test_vma_mm(test);
	struct mm_struct *native = mm_alloc();
	struct vm_area_struct *vma[3];
	unsigned long start = SZ_2M - SZ_4K;
	unsigned int i, side;
	u8 byte;

	KUNIT_ASSERT_NOT_NULL(test, sm);
	KUNIT_ASSERT_NOT_NULL(test, pages);
	KUNIT_ASSERT_NOT_NULL(test, data[0]);
	KUNIT_ASSERT_NOT_NULL(test, data[1]);
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_NOT_NULL(test, cold);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, native), 0);
	/* Reverse the supplied native pages: folio indices cannot identify them. */
	pages[0] = virt_to_page(data[1]);
	pages[1] = virt_to_page(data[0]);
	sm->pages = pages;
	sm->name = "[user4k-special-test]";
	for (i = 0; i < 2 * TEST_SLOTS; i++)
		memset(page_address(pages[i / TEST_SLOTS]) + (i % TEST_SLOTS) * SZ_4K, 0x41 + i, SZ_4K);
	vma[0] = test_special_vma(small, sm, start, 2 * PAGE_SIZE, VM_READ | VM_MAYREAD | VM_MAYWRITE);
	vma[1] = test_special_vma(cold, sm, start, 2 * PAGE_SIZE, VM_READ | VM_MAYREAD | VM_MAYWRITE);
	vma[2] = test_special_vma(native, sm, TEST_VA, 2 * PAGE_SIZE, VM_READ | VM_MAYREAD | VM_MAYWRITE);
	for (i = 0; i < 3; i++)
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma[i]);
	for (i = 0; i < 2 * TEST_SLOTS; i++) {
		unsigned long addr = start + i * SZ_4K;

		KUNIT_ASSERT_EQ(test, access_remote_vm(small, addr, &byte, 1, 0), 1);
		KUNIT_EXPECT_EQ(test, byte, (u8)(0x41 + i));
		KUNIT_EXPECT_EQ(test, pte_phys_mm(small, test_fault_entry(test, small, addr)),
				page_to_phys(pages[i / TEST_SLOTS]) + (i % TEST_SLOTS) * SZ_4K);
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(small, MM_FILEPAGES), 2UL * TEST_SLOTS);
	/* Both present-PTE and cold-PTE forced writes copy only their quarter. */
	for (side = 0; side < 2; side++) {
		struct mm_struct *mm = side ? cold : small;

		byte = 0xe1 + side;
		KUNIT_ASSERT_EQ(test, access_remote_vm(mm, start + 2 * SZ_4K + 7,
						&byte, 1, FOLL_WRITE | FOLL_FORCE), 1);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 1UL);
		KUNIT_EXPECT_EQ(test, access_remote_vm(mm, start + 2 * SZ_4K + 7, &byte, 1, 0), 1);
		KUNIT_EXPECT_EQ(test, byte, (u8)(0xe1 + side));
		KUNIT_EXPECT_EQ(test, access_remote_vm(native, TEST_VA + 2 * SZ_4K + 7, &byte, 1, 0), 1);
		KUNIT_EXPECT_EQ(test, byte, (u8)0x43);
	}
	{
		struct user_page_fragment fragment;
		long ret = pin_user_fragments_remote(small, start + 5 * SZ_4K + 13,
						    7, 0, &fragment, 1);

		KUNIT_ASSERT_EQ(test, ret, 1L);
		KUNIT_EXPECT_NOT_NULL(test, fragment.subpage);
		KUNIT_EXPECT_EQ(test, fragment.offset, (5 * SZ_4K + 13UL) & ~PAGE_MASK);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(fragment.folio) + fragment.offset,
						    0x46, fragment.length), NULL);
		release_user_fragments(&fragment, 1, false);
	}
}

static vm_fault_t test_callback_fault(struct vm_fault *vmf)
{
	struct page **pages = vmf->vma->vm_private_data;

	if (vmf->pgoff >= 2)
		return VM_FAULT_SIGBUS;

	vmf->page = pages[vmf->pgoff];
	get_page(vmf->page);
	return 0;
}

static const struct vm_operations_struct test_callback_vm_ops = {
	.fault = test_callback_fault,
};

static void test_callback_mapping(struct kunit *test, unsigned int shift)
{
	void *data[2] = { test_alloc_page(test), test_alloc_page(test) };
	struct page **pages = kunit_kcalloc(test, 2, sizeof(*pages), GFP_KERNEL);
	struct file *file = test_cache_file(test, 0, false);
	struct vm_area_struct *vma;
	unsigned long size = 1UL << shift;
	unsigned long offset = shift == PAGE_SHIFT ? 0 : size;
	unsigned long len = 2 * PAGE_SIZE - offset;
	unsigned long addr;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, data[0]);
	KUNIT_ASSERT_NOT_NULL(test, data[1]);
	KUNIT_ASSERT_NOT_NULL(test, pages);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	pages[0] = virt_to_page(data[1]);
	pages[1] = virt_to_page(data[0]);
	for (i = 0; i < 2 * PAGE_SIZE / size; i++)
		memset(page_address(pages[(i * size) >> PAGE_SHIFT]) +
		       offset_in_page(i * size), 0x51 + i, size);

	/* The file identifies the VMA; the callback supplies independent pages. */
	vma = test_file_vma_granule(test, file, shift, false, TEST_VA, len, offset);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	mmap_write_lock(vma->vm_mm);
	vma->vm_ops = &test_callback_vm_ops;
	vma->vm_private_data = pages;
	mmap_write_unlock(vma->vm_mm);
	for (addr = vma->vm_start; addr < vma->vm_end; addr += size) {
		unsigned long source = offset + addr - vma->vm_start;
		u8 byte;

		KUNIT_ASSERT_EQ(test, test_file_fault(vma, addr, false) & VM_FAULT_ERROR, 0U);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(vma->vm_mm,
				test_fault_entry(test, vma->vm_mm, addr)),
				page_to_phys(pages[source >> PAGE_SHIFT]) + offset_in_page(source));
		KUNIT_ASSERT_EQ(test, access_remote_vm(vma->vm_mm, addr + 7, &byte, 1, 0), 1);
		KUNIT_EXPECT_EQ(test, byte, (u8)(0x51 + source / size));
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_FILEPAGES), len / size);
}

static void native_callback_mapping_test(struct kunit *test)
{
	test_callback_mapping(test, PAGE_SHIFT);
}

static void user4k_callback_mapping_test(struct kunit *test)
{
	test_callback_mapping(test, 12);
}

static void user16k_callback_mapping_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K alternative requires a larger native granule");
		return;
	}
	test_callback_mapping(test, 14);
}

static void user4k_insert_range_test(struct kunit *test)
{
	unsigned int native;

	for (native = 0; native < 2; native++) {
		void *data[2] = { test_alloc_page(test), test_alloc_page(test) };
		struct page *pages[2];
		struct mm_struct *mm = native ? mm_alloc() : test_vma_mm(test);
		struct vm_area_struct *vma;
		unsigned long size, start, offset, length, num;
		unsigned int i;
		int ret, invalid[4];

		KUNIT_ASSERT_NOT_NULL(test, data[0]);
		KUNIT_ASSERT_NOT_NULL(test, data[1]);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		if (native)
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
		pages[0] = virt_to_page(data[1]);
		pages[1] = virt_to_page(data[0]);
		size = mm_page_size(mm);
		start = pmd_size_mm(mm) - size;
		offset = native ? 0 : 3 * SZ_4K;
		length = 2 * PAGE_SIZE - offset;
		vma = test_vma_add(mm, start, length, (struct vm_page_offset) {},
			VM_READ | VM_MAYREAD | VM_MIXEDMAP | VM_IO);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma);
		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
		mmap_write_lock(mm);
		invalid[0] = vm_insert_pages_range(vma, start + 1, pages, 2, offset, length);
		invalid[1] = vm_insert_pages_range(vma, start, pages, 2, offset + 1, length);
		invalid[2] = vm_insert_pages_range(vma, start, pages, 2, offset, length + size);
		invalid[3] = vm_insert_pages_range(vma, start, pages, ULONG_MAX, 0, length);
		ret = vm_insert_pages_range(vma, start, pages, 2, offset, length);
		mmap_write_unlock(mm);
		for (i = 0; i < ARRAY_SIZE(invalid); i++)
			KUNIT_EXPECT_EQ(test, invalid[i], -EINVAL);
		KUNIT_ASSERT_EQ(test, ret, 0);
		for (i = 0; i < length / size; i++) {
			unsigned long source = offset + i * size;

			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm,
				test_fault_entry(test, mm, start + i * size)),
				page_to_phys(pages[source >> PAGE_SHIFT]) + offset_in_page(source));
		}
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_FILEPAGES), length / size);
		/* The original native-page-count API also expands all user leaves. */
		start += 4 * PAGE_SIZE;
		vma = test_vma_add(mm, start, 2 * PAGE_SIZE, (struct vm_page_offset) {},
			VM_READ | VM_MAYREAD | VM_MIXEDMAP | VM_IO);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma);
		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
		num = 2;
		mmap_write_lock(mm);
		ret = vm_insert_pages(vma, start, pages, &num);
		mmap_write_unlock(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, num, 0UL);
		for (i = 0; i < 2 * PAGE_SIZE / size; i++)
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm,
				test_fault_entry(test, mm, start + i * size)),
				page_to_phys(pages[(i * size) >> PAGE_SHIFT]) + offset_in_page(i * size));
		num = 2;
		mmap_write_lock(mm);
		ret = vm_insert_pages(vma, start, pages, &num);
		mmap_write_unlock(mm);
		KUNIT_EXPECT_EQ(test, ret, -EBUSY);
		KUNIT_EXPECT_EQ(test, num, 2UL);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_FILEPAGES),
			(length + 2 * PAGE_SIZE) / size);
		start += 4 * PAGE_SIZE;
		offset = native ? PAGE_SIZE : 3 * SZ_4K;
		length = 2 * PAGE_SIZE - offset;
		vma = test_vma_add(mm, start, length,
			(struct vm_page_offset) { .index = offset >> PAGE_SHIFT,
				.offset = offset_in_page(offset) },
			VM_READ | VM_MAYREAD | VM_MIXEDMAP | VM_IO);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma);
		vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
		mmap_write_lock(mm);
		ret = vm_map_pages(vma, pages, 2);
		mmap_write_unlock(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		for (i = 0; i < length / size; i++) {
			unsigned long source = offset + i * size;

			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm,
				test_fault_entry(test, mm, start + i * size)),
				page_to_phys(pages[source >> PAGE_SHIFT]) + offset_in_page(source));
		}
		kunit_release_action(test, user4k_vma_mm_free, mm);
		KUNIT_EXPECT_EQ(test, page_ref_count(pages[0]), 1);
		KUNIT_EXPECT_EQ(test, page_ref_count(pages[1]), 1);
	}
}

static void user4k_remap_pfn_range_test(struct kunit *test)
{
	struct folio *folio = folio_alloc(GFP_KERNEL | __GFP_ZERO, 1);

	KUNIT_ASSERT_NOT_NULL(test, folio);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	for (unsigned int shift = 12; shift <= PAGE_SHIFT; shift += 2) {
		struct mm_struct *mm = test_vma_mm_granule(test, shift);
		bool native = shift == PAGE_SHIFT;
		unsigned long size, boundaries[4];

		KUNIT_ASSERT_NOT_NULL(test, mm);
		size = mm_page_size(mm);
		boundaries[0] = pmd_size_mm(mm);
		boundaries[1] = pud_size_mm(mm);
		boundaries[2] = pgd_size_mm(mm);
		boundaries[3] = 2 * pmd_size_mm(mm);
		for (unsigned int shape = 0; shape < ARRAY_SIZE(boundaries); shape++) {
			unsigned long start = boundaries[shape] - size;
			unsigned long length = PAGE_SIZE + size;
			unsigned int physical_offset = native || !shape ? 0 : size;
			struct vm_area_struct *vma;
			int ret;

			if (shape && boundaries[shape] == boundaries[shape - 1])
				continue;
			vma = test_vma_add(mm, start, length, (struct vm_page_offset) {},
				VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE |
				VM_SHARED | VM_MAYSHARE);
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma);
			vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
			mmap_write_lock(mm);
			/* The mmap_prepare compat path modifies an attached VMA. */
			vma_start_write(vma);
			if (shape >= 2) {
				phys_addr_t base = PFN_PHYS(folio_pfn(folio)) + physical_offset;
				unsigned long available = folio_size(folio) - physical_offset;

				vma_set_page_offset(vma, (struct vm_page_offset) {
					.offset = physical_offset,
				});
				if (shape == 2) {
					KUNIT_EXPECT_EQ(test,
						vm_iomap_memory(vma, base, PAGE_SIZE), -EINVAL);
					ret = vm_iomap_memory(vma, base, available);
				} else {
					struct vm_area_desc desc;

					compat_set_desc_from_vma(&desc, NULL, vma);
					mmap_action_simple_ioremap(&desc, base, PAGE_SIZE);
					KUNIT_EXPECT_EQ(test,
						__compat_vma_mmap(&desc, vma), -EINVAL);
					mmap_action_simple_ioremap(&desc, base, available);
					ret = __compat_vma_mmap(&desc, vma);
				}
				physical_offset *= 2;
			} else if (physical_offset)
				ret = remap_pfn_range_offset(vma, start, folio_pfn(folio),
						physical_offset, length, vma->vm_page_prot);
			else
				ret = remap_pfn_range(vma, start, folio_pfn(folio), length,
						      vma->vm_page_prot);
			mmap_write_unlock(mm);
			KUNIT_ASSERT_EQ(test, ret, 0);
			for (unsigned long offset = 0; offset < length; offset += size) {
				pte_t pte = test_fault_entry(test, mm, start + offset);

				struct follow_pfnmap_args args = {
					.vma = vma, .address = start + offset + 31,
				};
				phys_addr_t expected = PFN_PHYS(folio_pfn(folio)) +
					physical_offset + offset + 31;
				int ret;
				KUNIT_EXPECT_TRUE(test, pte_present(pte));
				KUNIT_EXPECT_TRUE(test, pte_special(pte));
				KUNIT_EXPECT_TRUE(test, pte_write(pte));
				KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte),
						PFN_PHYS(folio_pfn(folio)) +
						physical_offset + offset);

				mmap_read_lock(mm);
				ret = follow_pfnmap_start(&args);
				if (!ret)
					follow_pfnmap_end(&args);
				mmap_read_unlock(mm);
				KUNIT_ASSERT_EQ(test, ret, native ? 0 : -EOPNOTSUPP);
				args.allow_subpage = true;
				mmap_read_lock(mm);
				ret = follow_pfnmap_start(&args);
				if (!ret) {
					KUNIT_EXPECT_EQ(test, args.pfn, (unsigned long)PHYS_PFN(expected));
					KUNIT_EXPECT_EQ(test, args.offset, (unsigned int)offset_in_page(expected));
					KUNIT_EXPECT_EQ(test, args.addr_mask, mm_page_mask(mm));
					KUNIT_EXPECT_TRUE(test, args.special);
					follow_pfnmap_end(&args);
				}
				mmap_read_unlock(mm);
				KUNIT_ASSERT_EQ(test, ret, 0);
			}
			KUNIT_EXPECT_TRUE(test, pte_none(test_fault_entry(test, mm, start - size)));
			KUNIT_EXPECT_TRUE(test, pte_none(test_fault_entry(test, mm, start + length)));
		}
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_FILEPAGES), 0UL);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 0UL);
		kunit_release_action(test, user4k_vma_mm_free, mm);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
	}
}

#ifdef CONFIG_HAVE_IOREMAP_PROT
static void test_iounmap(void *arg)
{
	iounmap((void __iomem *)arg);
}

static const struct vm_operations_struct test_iomem_ops = {
	.access = generic_access_phys,
};
#endif

static void user4k_iomem_access_test(struct kunit *test)
{
#if defined(CONFIG_HAVE_IOREMAP_PROT) && defined(CONFIG_OF)
	struct device_node *node;
	struct resource resource;
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	void __iomem *uart;
	unsigned long start = SZ_2M + PAGE_SIZE - SZ_4K;
	u8 reference[16], buffer[32];
	vm_fault_t fault;
	int ret;

	if (!of_machine_is_compatible("linux,dummy-virt")) {
		kunit_skip(test, "requires disposable QEMU virt PL011 identification registers");
		return;
	}
	node = of_find_compatible_node(NULL, NULL, "arm,pl011");
	KUNIT_ASSERT_NOT_NULL(test, node);
	ret = of_address_to_resource(node, 0, &resource);
	uart = ret ? NULL : of_iomap(node, 0);
	of_node_put(node);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_NOT_NULL(test, uart);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_iounmap,
						     (void *)uart), 0);
	KUNIT_ASSERT_GE(test, resource_size(&resource), (resource_size_t)SZ_4K);
	memcpy_fromio(reference, uart + 0xff0, sizeof(reference));
	KUNIT_ASSERT_NOT_NULL(test, memchr_inv(reference, 0, sizeof(reference)));
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	vma = test_vma_add(mm, start, 2 * SZ_4K, (struct vm_page_offset) {},
			VM_READ | VM_MAYREAD | VM_SHARED | VM_MAYSHARE | VM_IO | VM_PFNMAP);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma);
	vma->vm_ops = &test_iomem_ops;
	vma->vm_page_prot = pgprot_device(vm_get_page_prot(vma->vm_flags));
	mmap_read_lock(mm);
	fault = vmf_insert_pfn_prot_mkwrite_offset(vma, start,
			PHYS_PFN(resource.start), offset_in_page(resource.start),
			vma->vm_page_prot, false);
	mmap_read_unlock(mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)VM_FAULT_NOPAGE);
	memset(buffer, 0x6d, sizeof(buffer));
	mmap_read_lock(mm);
	ret = generic_access_phys(vma, start + 0xff0, buffer, sizeof(buffer), 0);
	mmap_read_unlock(mm);
	KUNIT_EXPECT_EQ(test, ret, 16);
	KUNIT_EXPECT_MEMEQ(test, buffer, reference, sizeof(reference));
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer + 16, 0x6d, 16), NULL);
	memset(buffer, 0x5a, sizeof(buffer));
	ret = access_remote_vm(mm, start + 0xff0, buffer, sizeof(buffer), 0);
	KUNIT_EXPECT_EQ(test, ret, 16);
	KUNIT_EXPECT_MEMEQ(test, buffer, reference, sizeof(reference));
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer + 16, 0x5a, 16), NULL);
	/* Writes must fail before touching the read-only device mapping. */
	mmap_read_lock(mm);
	ret = generic_access_phys(vma, start + 0xff0, buffer, 1, FOLL_WRITE);
	mmap_read_unlock(mm);
	KUNIT_EXPECT_LT(test, ret, 0);
#else
	kunit_skip(test, "requires OF and iomem access support");
#endif
}

static void user4k_pfn_offset_test(struct kunit *test)
{
	void *data = test_alloc_page(test);
	struct mm_struct *mm = test_vma_mm(test);
	struct vm_area_struct *vma;
	unsigned long start = SZ_2M - SZ_4K;
	unsigned long pfn;
	unsigned int i;
	vm_fault_t result[4], invalid[3], write;

	KUNIT_ASSERT_NOT_NULL(test, data);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	pfn = page_to_pfn(virt_to_page(data));
	vma = test_vma_add(mm, start, PAGE_SIZE, (struct vm_page_offset) {},
		VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE | VM_SHARED | VM_MAYSHARE |
		VM_PFNMAP | VM_IO);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma);
	/* Model a write-notify VMA: initial PTEs are read-only. */
	vma->vm_page_prot = vm_get_page_prot(VM_READ | VM_SHARED);
	mmap_read_lock(mm);
	for (i = 0; i < 4; i++)
		result[i] = vmf_insert_pfn_prot_mkwrite_offset(vma, start + i * SZ_4K,
			pfn, (3 - i) * SZ_4K, vma->vm_page_prot, false);
	invalid[0] = vmf_insert_pfn_prot_mkwrite_offset(vma, start + 1, pfn,
		0, vma->vm_page_prot, false);
	invalid[1] = vmf_insert_pfn_prot_mkwrite_offset(vma, start, pfn,
		1, vma->vm_page_prot, false);
	invalid[2] = vmf_insert_pfn_prot_mkwrite_offset(vma, start, pfn,
		PAGE_SIZE, vma->vm_page_prot, false);
	write = vmf_insert_pfn_prot_mkwrite_offset(vma, start + SZ_4K, pfn,
		2 * SZ_4K, vma->vm_page_prot, true);
	mmap_read_unlock(mm);
	for (i = 0; i < 4; i++) {
		pte_t entry = test_fault_entry(test, mm, start + i * SZ_4K);

		KUNIT_EXPECT_EQ(test, result[i], (vm_fault_t)VM_FAULT_NOPAGE);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, entry), PFN_PHYS(pfn) + (3 - i) * SZ_4K);
		KUNIT_EXPECT_TRUE(test, pte_special(entry));
		KUNIT_EXPECT_EQ(test, !!pte_write(entry), i == 1);
	}
	for (i = 0; i < ARRAY_SIZE(invalid); i++)
		KUNIT_EXPECT_EQ(test, invalid[i], (vm_fault_t)VM_FAULT_SIGBUS);
	KUNIT_EXPECT_EQ(test, write, (vm_fault_t)VM_FAULT_NOPAGE);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_FILEPAGES), 0UL);
	KUNIT_EXPECT_EQ(test, page_ref_count(virt_to_page(data)), 1);
	kunit_release_action(test, user4k_vma_mm_free, mm);
	KUNIT_EXPECT_EQ(test, page_ref_count(virt_to_page(data)), 1);
}

static void user16k_insert_shared_test(struct kunit *test)
{
	struct mm_struct *middle, *small, *native;
	struct vm_area_struct *vmas[3];
	struct mm_subpage_mapcounts counts;
	struct page *page;
	void *data;
	vm_fault_t result;
	unsigned int i;
	vm_flags_t flags = VM_READ | VM_MAYREAD | VM_MIXEDMAP | VM_IO;

	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "mixed16K file insertion requires larger native backing");
		return;
	}
	data = test_alloc_page(test);
	KUNIT_ASSERT_NOT_NULL(test, data);
	page = virt_to_page(data);
	middle = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, middle);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, middle), 0);
	KUNIT_ASSERT_EQ(test, test_mm_select_granule(middle, 14), 0);
	small = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, small);
	native = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, native), 0);
	vmas[0] = test_vma_add(middle, TEST_VA, PAGE_SIZE, (struct vm_page_offset) {}, flags);
	vmas[1] = test_vma_add(small, TEST_VA + SZ_4K, SZ_4K, (struct vm_page_offset) {}, flags);
	vmas[2] = test_vma_add(native, TEST_VA, PAGE_SIZE, (struct vm_page_offset) {}, flags);
	for (i = 0; i < 3; i++) {
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vmas[i]);
		vmas[i]->vm_page_prot = vm_get_page_prot(vmas[i]->vm_flags);
	}
	for (i = 0; i < PAGE_SIZE / SZ_16K; i++) {
		unsigned long address = TEST_VA + i * SZ_16K;

		mmap_read_lock(middle);
		result = vmf_insert_page_offset(vmas[0], address, page, i * SZ_16K);
		mmap_read_unlock(middle);
		KUNIT_ASSERT_EQ(test, result, (vm_fault_t)VM_FAULT_NOPAGE);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(middle, test_fault_entry(test, middle, address)),
				page_to_phys(page) + i * SZ_16K);
	}
	mmap_read_lock(small);
	result = vmf_insert_page_offset(vmas[1], vmas[1]->vm_start, page, PAGE_SIZE - SZ_4K);
	mmap_read_unlock(small);
	KUNIT_ASSERT_EQ(test, result, (vm_fault_t)VM_FAULT_NOPAGE);
	mmap_read_lock(native);
	result = vmf_insert_page_offset(vmas[2], vmas[2]->vm_start, page, 0);
	mmap_read_unlock(native);
	KUNIT_ASSERT_EQ(test, result, (vm_fault_t)VM_FAULT_NOPAGE);
	KUNIT_EXPECT_EQ(test, mm_subpage_file_mapcounts(page, &counts), 5U);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, counts.slots[i], i == TEST_SLOTS - 1 ? 2U : 1U);
	KUNIT_EXPECT_EQ(test, folio_mapcount(page_folio(page)), 6);
	KUNIT_EXPECT_EQ(test, page_ref_count(page), 7);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(middle, MM_FILEPAGES), 4UL);
	/* Real mm teardown removes whole16K leaves and preserves both aliases. */
	kunit_release_action(test, user4k_vma_mm_free, middle);
	KUNIT_EXPECT_EQ(test, mm_subpage_file_mapcounts(page, &counts), 1U);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, counts.slots[i], i == TEST_SLOTS - 1 ? 1U : 0U);
	KUNIT_EXPECT_EQ(test, folio_mapcount(page_folio(page)), 2);
	KUNIT_EXPECT_EQ(test, page_ref_count(page), 3);
	KUNIT_EXPECT_EQ(test, pte_phys_mm(small, test_fault_entry(test, small, vmas[1]->vm_start)),
			page_to_phys(page) + PAGE_SIZE - SZ_4K);
	KUNIT_EXPECT_EQ(test, pte_phys_mm(native, test_fault_entry(test, native, vmas[2]->vm_start)),
			page_to_phys(page));
	kunit_release_action(test, user4k_vma_mm_free, small);
	KUNIT_EXPECT_EQ(test, mm_subpage_file_mapcounts(page, &counts), 0U);
	KUNIT_EXPECT_EQ(test, folio_mapcount(page_folio(page)), 1);
	kunit_release_action(test, user4k_vma_mm_free, native);
	KUNIT_EXPECT_EQ(test, folio_mapcount(page_folio(page)), 0);
	KUNIT_EXPECT_EQ(test, page_ref_count(page), 1);
}

static void user4k_insert_fragment_test(struct kunit *test)
{
	void *data = test_alloc_page(test);
	struct mm_struct *mm = test_vma_mm(test);
	struct vm_area_struct *vma;
	unsigned long start = TEST_VA + SZ_4K;
	struct page *page;
	vm_fault_t result[4], invalid[4];
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, data);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	page = virt_to_page(data);
	vma = test_vma_add(mm, start, PAGE_SIZE, (struct vm_page_offset) {},
			  VM_READ | VM_MAYREAD | VM_MIXEDMAP | VM_IO);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma);
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	mmap_read_lock(mm);
	for (i = 0; i < 4; i++)
		result[i] = vmf_insert_page_offset(vma, start + i * SZ_4K, page, (3 - i) * SZ_4K);
	invalid[0] = vmf_insert_page_offset(vma, start + 1, page, 0);
	invalid[1] = vmf_insert_page_offset(vma, start, page, 1);
	invalid[2] = vmf_insert_page_offset(vma, start, page, PAGE_SIZE);
	invalid[3] = vmf_insert_page_offset(vma, vma->vm_end, page, 0);
	mmap_read_unlock(mm);
	for (i = 0; i < 4; i++) {
		KUNIT_EXPECT_EQ(test, result[i], (vm_fault_t)VM_FAULT_NOPAGE);
		KUNIT_EXPECT_EQ(test, invalid[i], (vm_fault_t)VM_FAULT_SIGBUS);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, test_fault_entry(test, mm, start + i * SZ_4K)),
				page_to_phys(page) + (3 - i) * SZ_4K);
	}
	KUNIT_EXPECT_EQ(test, page_ref_count(page), 5);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_FILEPAGES), 4UL);
	kunit_release_action(test, user4k_vma_mm_free, mm);
	KUNIT_EXPECT_EQ(test, page_ref_count(page), 1);
}

#if defined(CONFIG_GENERIC_GETTIMEOFDAY) && defined(CONFIG_VDSO_GETRANDOM)
static void user4k_vdso_mapping_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct linux_binprm bprm = {};
	unsigned long code, base, left[8];
	u8 bytes[8], magic[4];
	unsigned int i;
	int ret;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm vDSO fixture requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	mm->mmap_base = TEST_VA + SZ_4K;
	mm_flags_clear(MMF_TOPDOWN, mm);
	kthread_use_mm(mm);
	ret = arch_setup_additional_pages(&bprm, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, ret, 0);
	code = (unsigned long)mm->context.vdso;
	KUNIT_ASSERT_NE(test, code, 0UL);
	base = code - VDSO_NR_PAGES * PAGE_SIZE;
	KUNIT_EXPECT_EQ(test, base & ~PAGE_MASK, SZ_4K);
	KUNIT_ASSERT_EQ(test, access_remote_vm(mm, code, magic, sizeof(magic), 0), (int)sizeof(magic));
	KUNIT_EXPECT_MEMEQ(test, magic, "\177ELF", 4);
	/* VVAR forbids remote GUP; ordinary uaccess must still fault it in. */
	kthread_use_mm(mm);
	for (i = 0; i < 4; i++) {
		left[i] = copy_from_user(&bytes[i], (void __user *)(base + i * SZ_4K), 1);
		left[4 + i] = copy_from_user(&bytes[4 + i],
			(void __user *)(base + VDSO_RNG_PAGE_OFFSET * PAGE_SIZE + i * SZ_4K), 1);
	}
	kthread_unuse_mm(mm);
	for (i = 0; i < 4; i++) {
		KUNIT_EXPECT_EQ(test, left[i], 0UL);
		KUNIT_EXPECT_EQ(test, left[4 + i], 0UL);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, test_fault_entry(test, mm, base + i * SZ_4K)),
				page_to_phys(virt_to_page(vdso_k_time_data)) + i * SZ_4K);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, test_fault_entry(test, mm,
			base + VDSO_RNG_PAGE_OFFSET * PAGE_SIZE + i * SZ_4K)),
			page_to_phys(virt_to_page(vdso_k_rng_data)) + i * SZ_4K);
	}
	KUNIT_EXPECT_EQ(test, access_remote_vm(mm, base, magic, sizeof(magic), 0), 0);
}

#endif

static void user4k_remote_access_test(struct kunit *test)
{
	struct vm_area_struct *vma = test_fault_vma(test, TEST_VA + SZ_4K, 3 * SZ_4K);
	struct mm_struct *child = test_vma_mm(test);
	struct vm_area_struct *child_vma;
	unsigned long start = TEST_VA + SZ_4K;
	u8 *buf = kunit_kmalloc(test, 3 * SZ_4K, GFP_KERNEL);
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_NOT_NULL(test, child);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	memset(buf, 0x5a, 3 * SZ_4K);
	KUNIT_ASSERT_EQ(test, access_remote_vm(vma->vm_mm, start, buf, 3 * SZ_4K, FOLL_WRITE),
			3 * (int)SZ_4K);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, vma->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	/* Deliberately straddle quarters whose physical/virtual offsets differ. */
	memset(buf, 0x96, SZ_4K + 34);
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, start + SZ_4K - 17,
			buf, SZ_4K + 34, FOLL_WRITE), (int)SZ_4K + 34);
	memset(buf, 0, 3 * SZ_4K);
	KUNIT_EXPECT_EQ(test, access_remote_vm(vma->vm_mm, start, buf, 3 * SZ_4K, 0),
			3 * (int)SZ_4K);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf, 0x5a, 3 * SZ_4K), NULL);
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, start, buf, 3 * SZ_4K, 0), 3 * (int)SZ_4K);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf, 0x5a, SZ_4K - 17), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf + SZ_4K - 17, 0x96, SZ_4K + 34), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf + 2 * SZ_4K + 17, 0x5a, SZ_4K - 17), NULL);
	child_vma = test_vma_lookup(child, start);
	KUNIT_ASSERT_NOT_NULL(test, child_vma);
	mmap_write_lock(child);
	vm_flags_clear(child_vma, VM_WRITE);
	mmap_write_unlock(child);
	buf[0] = 0x27;
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, start + 1, buf, 1, FOLL_WRITE), 0);
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, start + 1, buf, 1,
			FOLL_WRITE | FOLL_FORCE), 1);
	buf[0] = 0;
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, start + 1, buf, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, buf[0], (u8)0x27);
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, start + 3 * SZ_4K - 5, buf, 20, 0), 5);
	KUNIT_EXPECT_EQ(test, access_remote_vm(child, start + 3 * SZ_4K, buf, 20, 0), 0);
}

static void user4k_remote_file_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, 2 * PAGE_SIZE, true);
	struct vm_area_struct *small, *native;
	u8 buf[34];

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	small = test_file_vma(test, file, false, true, TEST_VA + SZ_4K, 3 * SZ_4K, 2 * SZ_4K);
	native = test_file_vma(test, file, true, true, TEST_VA, 2 * PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_EXPECT_EQ(test, access_remote_vm(small->vm_mm, TEST_VA + 3 * SZ_4K - 17,
			buf, sizeof(buf), 0), (int)sizeof(buf));
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf, 0x34, 17), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf + 17, 0x35, 17), NULL);
	memset(buf, 0xa6, sizeof(buf));
	KUNIT_EXPECT_EQ(test, access_remote_vm(small->vm_mm, TEST_VA + 3 * SZ_4K - 17,
			buf, sizeof(buf), FOLL_WRITE), (int)sizeof(buf));
	memset(buf, 0, sizeof(buf));
	KUNIT_EXPECT_EQ(test, access_remote_vm(native->vm_mm, TEST_VA + 4 * SZ_4K - 17,
			buf, sizeof(buf), 0), (int)sizeof(buf));
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf, 0xa6, sizeof(buf)), NULL);
#ifdef CONFIG_CROSS_MEMORY_ATTACH
	{
		struct kvec vec = { .iov_base = buf, .iov_len = sizeof(buf) };
		struct iov_iter iter;

		memset(buf, 0xb7, sizeof(buf));
		iov_iter_kvec(&iter, ITER_SOURCE, &vec, 1, sizeof(buf));
		KUNIT_EXPECT_EQ(test, process_vm_rw_fragments(small->vm_mm,
			TEST_VA + 3 * SZ_4K - 17, sizeof(buf), &iter, true), 0);
		KUNIT_EXPECT_EQ(test, iov_iter_count(&iter), 0UL);
		memset(buf, 0, sizeof(buf));
		KUNIT_EXPECT_EQ(test, access_remote_vm(native->vm_mm, TEST_VA + 4 * SZ_4K - 17,
			buf, sizeof(buf), 0), (int)sizeof(buf));
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf, 0xb7, sizeof(buf)), NULL);
	}
#endif
	mmap_write_lock(small->vm_mm);
	vm_flags_clear(small, VM_WRITE);
	mmap_write_unlock(small->vm_mm);
	KUNIT_EXPECT_EQ(test, access_remote_vm(small->vm_mm, small->vm_start, buf, 1,
			FOLL_WRITE | FOLL_FORCE), 0);
#ifdef CONFIG_CROSS_MEMORY_ATTACH
	{
		struct kvec vec = { .iov_base = buf, .iov_len = sizeof(buf) };
		struct iov_iter iter;

		iov_iter_kvec(&iter, ITER_SOURCE, &vec, 1, sizeof(buf));
		KUNIT_EXPECT_EQ(test, process_vm_rw_fragments(small->vm_mm,
			small->vm_start, sizeof(buf), &iter, true), -EFAULT);
		KUNIT_EXPECT_EQ(test, iov_iter_count(&iter), sizeof(buf));
	}
#endif
}

#ifdef CONFIG_CROSS_MEMORY_ATTACH
static void user4k_process_vm_test(struct kunit *test)
{
	/* Exceed the stack batch, cross table boundaries and include a partial tail. */
	unsigned long start = SZ_2M - SZ_4K, len = 21 * SZ_4K;
	struct vm_area_struct *vma = test_fault_vma(test, start, len);
	struct mm_struct *child = test_vma_mm(test);
	struct iov_iter iter;
	struct kvec vec;
	u8 *buf = kunit_kmalloc(test, len + 17, GFP_KERNEL);
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_NOT_NULL(test, child);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	memset(buf, 0x61, len);
	vec = (struct kvec) { .iov_base = buf, .iov_len = len };
	iov_iter_kvec(&iter, ITER_SOURCE, &vec, 1, len);
	ret = process_vm_rw_fragments(vma->vm_mm, start, len, &iter, true);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, iov_iter_count(&iter), 0UL);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, vma->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	memset(buf, 0xc7, len);
	iov_iter_kvec(&iter, ITER_SOURCE, &vec, 1, len - 34);
	ret = process_vm_rw_fragments(child, start + 17, len - 34, &iter, true);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, iov_iter_count(&iter), 0UL);
	memset(buf, 0, len);
	iov_iter_kvec(&iter, ITER_DEST, &vec, 1, len);
	ret = process_vm_rw_fragments(vma->vm_mm, start, len, &iter, false);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, iov_iter_count(&iter), 0UL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf, 0x61, len), NULL);
	vec.iov_len = len + 17;
	iov_iter_kvec(&iter, ITER_DEST, &vec, 1, len + 17);
	ret = process_vm_rw_fragments(child, start, len + 17, &iter, false);
	KUNIT_EXPECT_EQ(test, ret, -EFAULT);
	KUNIT_EXPECT_EQ(test, iov_iter_count(&iter), 17UL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf, 0x61, 17), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf + 17, 0xc7, len - 34), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf + len - 17, 0x61, 17), NULL);
	/* A shorter local iterator ends successfully before the remote range. */
	iov_iter_kvec(&iter, ITER_DEST, &vec, 1, 23);
	ret = process_vm_rw_fragments(child, start + 13, len, &iter, false);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, iov_iter_count(&iter), 0UL);
}
#endif

#ifdef CONFIG_FUTEX
static void test_stop_kthread(void *arg);

struct test_futex_waiter {
	struct mm_struct *mm;
	unsigned long address;
	struct completion ready;
	struct completion done;
	int result;
};

static int test_futex_wait_run(void *arg)
{
	struct test_futex_waiter *w = arg;
	ktime_t deadline = ktime_add_ms(ktime_get(), 10000);

	kthread_use_mm(w->mm);
	complete(&w->ready);
	w->result = futex_wait((u32 __user *)w->address, FLAGS_SIZE_32 | FLAGS_SHARED,
			      0x33333333, &deadline, FUTEX_BITSET_MATCH_ANY);
	kthread_unuse_mm(w->mm);
	complete(&w->done);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);
	return 0;
}

static void user4k_futex_wake_test(struct kunit *test)
{
	struct file *file;
	struct vm_area_struct *vmas[2];
	unsigned long addresses[2] = { TEST_VA + SZ_4K + 64, TEST_VA + 2 * SZ_4K + 64 };
	unsigned int round;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm futex fixture requires hardware opt-in");
		return;
	}
	file = test_cache_file(test, PAGE_SIZE, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	vmas[0] = test_file_vma(test, file, false, true, TEST_VA + SZ_4K, SZ_4K, 2 * SZ_4K);
	vmas[1] = test_file_vma(test, file, true, true, TEST_VA, PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, vmas[0]);
	KUNIT_ASSERT_NOT_NULL(test, vmas[1]);
	/* Both directions use the real wait queue and atomic user-value read. */
	for (round = 0; round < 2; round++) {
		struct test_futex_waiter *w = kunit_kzalloc(test, sizeof(*w), GFP_KERNEL);
		struct task_struct *task;
		unsigned long until;
		int woke = 0, wrong;

		KUNIT_ASSERT_NOT_NULL(test, w);
		w->mm = vmas[round]->vm_mm;
		w->address = addresses[round];
		init_completion(&w->ready);
		init_completion(&w->done);
		task = kthread_run(test_futex_wait_run, w, "user4k-futex-%u", round);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, task);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_stop_kthread, task), 0);
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&w->ready, 5 * HZ), 0UL);
		until = jiffies + 5 * HZ;
		kthread_use_mm(vmas[round ^ 1]->vm_mm);
		wrong = futex_wake((u32 __user *)(addresses[round ^ 1] + 4),
				   FLAGS_SIZE_32 | FLAGS_SHARED, NULL, 1,
					  FUTEX_BITSET_MATCH_ANY);
		do {
			woke = futex_wake((u32 __user *)addresses[round ^ 1],
					  FLAGS_SIZE_32 | FLAGS_SHARED, NULL, 1,
					  FUTEX_BITSET_MATCH_ANY);
			if (woke || completion_done(&w->done))
				break;
			schedule_timeout_uninterruptible(1);
		} while (time_before(jiffies, until));
		kthread_unuse_mm(vmas[round ^ 1]->vm_mm);
		KUNIT_EXPECT_EQ(test, wrong, 0);
		KUNIT_EXPECT_EQ(test, woke, 1);
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&w->done, 12 * HZ), 0UL);
		KUNIT_EXPECT_EQ(test, w->result, 0);
		kunit_release_action(test, test_stop_kthread, task);
	}
}

static void user4k_futex_keys_test(struct kunit *test)
{
	struct file *file;
	struct vm_area_struct *small, *native;
	union futex_key keys[2], other;
	unsigned int i;
	int ret;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm futex fixture requires hardware opt-in");
		return;
	}
	file = test_cache_file(test, 2 * PAGE_SIZE, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	small = test_file_vma(test, file, false, true, TEST_VA + SZ_4K, 3 * SZ_4K, 2 * SZ_4K);
	native = test_file_vma(test, file, true, true, TEST_VA, 2 * PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_NOT_NULL(test, native);
	for (i = 0; i < 3; i++) {
		unsigned long va = small->vm_start + i * SZ_4K + 64;
		unsigned long fileoff = (i + 2) * SZ_4K + 64;

		kthread_use_mm(small->vm_mm);
		ret = get_futex_key((u32 __user *)va, FLAGS_SIZE_32 | FLAGS_SHARED,
				    &keys[0], FUTEX_READ);
		kthread_unuse_mm(small->vm_mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		kthread_use_mm(native->vm_mm);
		ret = get_futex_key((u32 __user *)(TEST_VA + fileoff),
				    FLAGS_SIZE_32 | FLAGS_SHARED, &keys[1], FUTEX_READ);
		kthread_unuse_mm(native->vm_mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_TRUE(test, futex_match(&keys[0], &keys[1]));
		KUNIT_EXPECT_EQ(test, keys[0].shared.pgoff, fileoff / PAGE_SIZE);
		KUNIT_EXPECT_EQ(test, keys[0].both.offset, (unsigned int)(fileoff % PAGE_SIZE) | FUT_OFF_INODE);
		kthread_use_mm(small->vm_mm);
		ret = get_futex_key((u32 __user *)(va + 4), FLAGS_SIZE_32 | FLAGS_SHARED,
				    &other, FUTEX_READ);
		kthread_unuse_mm(small->vm_mm);
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_FALSE(test, futex_match(&keys[0], &other));
	}
	mmap_write_lock(small->vm_mm);
	vm_flags_clear(small, VM_WRITE);
	mmap_write_unlock(small->vm_mm);
	kthread_use_mm(small->vm_mm);
	ret = get_futex_key((u32 __user *)(small->vm_start + 2 * SZ_4K + 64),
			    FLAGS_SIZE_32 | FLAGS_SHARED, &other, FUTEX_READ);
	kthread_unuse_mm(small->vm_mm);
	KUNIT_EXPECT_EQ(test, ret, 0);
	KUNIT_EXPECT_TRUE(test, futex_match(&keys[1], &other));
}
#endif

static void test_swap_usage_mapping_free(void *arg)
{
	struct address_space *mapping = arg;

	xa_destroy(&mapping->i_pages);
}

static void user4k_shmem_swap_usage_test(struct kunit *test)
{
	struct address_space *mapping = kunit_kzalloc(test, sizeof(*mapping), GFP_KERNEL);
	void *ret;

	KUNIT_ASSERT_NOT_NULL(test, mapping);
	xa_init_flags(&mapping->i_pages, XA_FLAGS_LOCK_IRQ);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_swap_usage_mapping_free, mapping), 0);
	/* Synthetic swap values exercise accounting without allocating swap slots. */
	ret = xa_store(&mapping->i_pages, 1, xa_mk_value(1), GFP_KERNEL);
	KUNIT_ASSERT_FALSE(test, xa_is_err(ret));
	if (IS_ENABLED(CONFIG_XARRAY_MULTI)) {
		ret = xa_store_range(&mapping->i_pages, 4, 7,
				     xa_mk_value(2), GFP_KERNEL);
		KUNIT_ASSERT_FALSE(test, xa_is_err(ret));
	} else {
		unsigned long index;

		/* Without multi-index entries, shmem stores one value per page. */
		for (index = 4; index <= 7; index++) {
			ret = xa_store(&mapping->i_pages, index,
				       xa_mk_value(2), GFP_KERNEL);
			KUNIT_ASSERT_FALSE(test, xa_is_err(ret));
		}
	}
	KUNIT_EXPECT_EQ(test, shmem_partial_swap_usage_bytes(mapping, PAGE_SIZE + SZ_4K,
			PAGE_SIZE + 3 * SZ_4K), 2UL * SZ_4K);
	KUNIT_EXPECT_EQ(test, shmem_partial_swap_usage_bytes(mapping, 3 * SZ_4K,
			2 * PAGE_SIZE + SZ_4K), PAGE_SIZE);
	KUNIT_EXPECT_EQ(test, shmem_partial_swap_usage_bytes(mapping, 4 * PAGE_SIZE + SZ_4K,
			7 * PAGE_SIZE + 3 * SZ_4K), 3 * PAGE_SIZE + 2UL * SZ_4K);
	KUNIT_EXPECT_EQ(test, shmem_partial_swap_usage_bytes(mapping, 5 * PAGE_SIZE + SZ_4K,
			5 * PAGE_SIZE + 2 * SZ_4K), (unsigned long)SZ_4K);
	KUNIT_EXPECT_EQ(test, shmem_partial_swap_usage_bytes(mapping, 0, SZ_4K), 0UL);
	KUNIT_EXPECT_EQ(test, shmem_partial_swap_usage_bytes(mapping, PAGE_SIZE, PAGE_SIZE), 0UL);
}

static void test_file_mapcounts(struct kunit *test, struct folio *folio,
			       const unsigned int expected[TEST_SLOTS], unsigned int native)
{
	struct mm_subpage_mapcounts counts;
	unsigned int i, total = 0;

	for (i = 0; i < TEST_SLOTS; i++)
		total += expected[i];
	KUNIT_EXPECT_EQ(test, mm_subpage_file_mapcounts(&folio->page, &counts), total);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, counts.slots[i], expected[i]);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)(total + native));
}

static void user4k_file_mapcounts_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, PAGE_SIZE, true);
	struct vm_area_struct *shared, *private, *native, *cvma;
	struct mm_struct *child;
	struct folio *folio;
	int ret;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	shared = test_file_vma(test, file, false, true, TEST_VA, 3 * SZ_4K, SZ_4K);
	private = test_file_vma(test, file, false, false, TEST_VA, 3 * SZ_4K, SZ_4K);
	native = test_file_vma(test, file, true, true, TEST_VA, PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, shared);
	KUNIT_ASSERT_NOT_NULL(test, private);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, test_file_fault(shared, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(private, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	folio = filemap_get_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 2, 2, 2 }, 1);
	KUNIT_ASSERT_EQ(test, test_file_fault(private, TEST_VA + 2 * SZ_4K, true) &
			VM_FAULT_ERROR, 0U);
	test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 2, 2, 1 }, 1);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, private->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	cvma = test_vma_lookup(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, cvma);
	test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 3, 3, 1 }, 1);
	KUNIT_ASSERT_EQ(test, test_file_fault(cvma, TEST_VA + SZ_4K, true) & VM_FAULT_ERROR, 0U);
	test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 3, 2, 1 }, 1);
	kunit_release_action(test, user4k_vma_mm_free, child);
	test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 2, 2, 1 }, 1);
	kunit_release_action(test, user4k_vma_mm_free, private->vm_mm);
	test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 1, 1, 1 }, 1);
	kunit_release_action(test, user4k_vma_mm_free, native->vm_mm);
	test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 1, 1, 1 }, 0);
	kunit_release_action(test, user4k_vma_mm_free, shared->vm_mm);
	test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 0, 0, 0 }, 0);
}

/* A unique RAM-backed test inode: exercise generic file faults without
 * making unsupported filesystem promotion part of this mapping fixture.
 */
static int test_coarse_read_folio(struct file *file, struct folio *folio)
{
	unsigned long i;
	loff_t base = (loff_t)folio->index << PAGE_SHIFT;
	loff_t size = i_size_read(file_inode(file));
	u8 *data = folio_address(folio);

	for (i = 0; i < folio_size(folio); i++)
		data[i] = base + i < size ? 0x31 + ((base + i) / SZ_4K) : 0;
	__folio_mark_uptodate(folio);
	folio_unlock(folio);
	return 0;
}

static struct file *test_coarse_cache_file(struct kunit *test, unsigned int order,
					 unsigned long size)
{
	static const struct address_space_operations aops = {
		.read_folio = test_coarse_read_folio,
		.dirty_folio = filemap_dirty_folio,
	};
	static const struct file_operations fops = {
		.mmap = generic_file_mmap,
		.read_iter = generic_file_read_iter,
		.llseek = generic_file_llseek,
	};
	struct file *file = anon_inode_create_getfile("user-coarse-file", &fops,
						   NULL, O_RDWR, NULL);
	struct folio *folio;
	int ret;

	if (IS_ERR(file))
		return file;
	if (kunit_add_action_or_reset(test, test_file_put, file))
		return ERR_PTR(-ENOMEM);
	file->f_mapping->a_ops = &aops;
	mapping_set_large_folios(file->f_mapping);
	i_size_write(file_inode(file), size);
	folio = folio_alloc(GFP_KERNEL, order);
	if (!folio)
		return ERR_PTR(-ENOMEM);
	ret = filemap_add_folio(file->f_mapping, folio, 0, GFP_KERNEL);
	if (!ret)
		test_coarse_read_folio(file, folio); /* Unlocks the installed folio. */
	folio_put(folio);
	return ret ? ERR_PTR(ret) : file;
}

static bool test_coarse_file_enabled(struct kunit *test)
{
	if (PAGE_SHIFT >= 16 ||
	    (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN))) {
		kunit_skip(test, "requires internal larger user granule and borrowed-mm opt-in");
		return false;
	}
	return true;
}

static void test_promote_unpin(void *data)
{
	struct user_page_fragment *fragment = data;

	if (fragment->folio) {
		release_user_fragments(fragment, 1, true);
		fragment->folio = NULL;
	}
}

static void user64k_file_promote_test(struct kunit *test)
{
	unsigned int pinned;

	if (!test_coarse_file_enabled(test))
		return;
	if (!IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE)) {
		kunit_skip(test, "cache promotion currently uses the collapse implementation");
		return;
	}
	for (pinned = 0; pinned < 2; pinned++) {
		struct file *file = test_cache_file(test, 2 * SZ_64K, true);
		struct vm_area_struct *coarse, *small;
		struct user_page_fragment *fragment;
		struct folio *folio;
		phys_addr_t old[SZ_64K / PAGE_SIZE], untouched;
		unsigned int nr = SZ_64K / PAGE_SIZE, i;
		unsigned long offset, modified = SZ_64K / 2 + 19;
		pte_t entry;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		coarse = test_file_vma_granule(test, file, 16, true, TEST_VA, SZ_64K, SZ_64K);
		small = test_file_vma(test, file, false, true, TEST_VA, SZ_64K, SZ_64K);
		KUNIT_ASSERT_NOT_NULL(test, coarse);
		KUNIT_ASSERT_NOT_NULL(test, small);
		folio = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		untouched = page_to_phys(&folio->page);
		folio_put(folio);
		for (i = 0; i < nr; i++) {
			folio = filemap_get_folio(file->f_mapping, nr + i);
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
			KUNIT_EXPECT_EQ(test, folio_order(folio), 0U);
			old[i] = page_to_phys(&folio->page);
			folio_put(folio);
		}
		for (offset = 0; offset < SZ_64K; offset += SZ_4K)
			KUNIT_ASSERT_EQ(test, test_file_fault(small, TEST_VA + offset, false) &
					VM_FAULT_ERROR, 0U);
		if (pinned) {
			fragment = kunit_kzalloc(test, sizeof(*fragment), GFP_KERNEL);
			KUNIT_ASSERT_NOT_NULL(test, fragment);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_promote_unpin,
								      fragment), 0);
			KUNIT_ASSERT_EQ(test, pin_user_fragments_remote(small->vm_mm,
					TEST_VA + modified, 1, FOLL_WRITE, fragment, 1), 1L);
			*(u8 *)(folio_address(fragment->folio) + fragment->offset) = 0xa7;
			KUNIT_EXPECT_EQ(test, collapse_file_user_page(coarse->vm_mm, file, nr), -EBUSY);
			/* A pin in the middle forces rollback of earlier isolated folios. */
			for (i = 0; i < nr; i++) {
				folio = filemap_get_folio(file->f_mapping, nr + i);
				KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
				KUNIT_EXPECT_EQ(test, page_to_phys(&folio->page), old[i]);
				KUNIT_EXPECT_EQ(test, folio_order(folio), 0U);
				folio_put(folio);
			}
			KUNIT_EXPECT_EQ(test, *(u8 *)(folio_address(fragment->folio) +
							 fragment->offset), (u8)0xa7);
			kunit_release_action(test, test_promote_unpin, fragment);
		}
		KUNIT_ASSERT_EQ(test, collapse_file_user_page(coarse->vm_mm, file, nr), 0);
		folio = filemap_get_folio(file->f_mapping, nr);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
		KUNIT_EXPECT_EQ(test, folio_order(folio), 16U - PAGE_SHIFT);
		KUNIT_EXPECT_EQ(test, folio->index, (pgoff_t)nr);
		KUNIT_EXPECT_TRUE(test, folio_test_dirty(folio));
		KUNIT_EXPECT_EQ(test, file->f_mapping->nrpages, (unsigned long)2 * nr);
		for (offset = 0; offset < SZ_64K; offset++) {
			u8 expected = pinned && offset == modified ? 0xa7 : 0x41 + offset / SZ_4K;

			if (((u8 *)folio_address(folio))[offset] != expected) {
				KUNIT_FAIL(test, "promoted data mismatch at %lu", offset);
				break;
			}
		}
		/* An already suitable folio is an idempotent success, including with a GET. */
		KUNIT_EXPECT_EQ(test, collapse_file_user_page(coarse->vm_mm, file, nr), 0);
		KUNIT_ASSERT_EQ(test, test_file_fault(coarse, TEST_VA, false) & VM_FAULT_ERROR, 0U);
		entry = test_fault_entry(test, coarse->vm_mm, TEST_VA);
		KUNIT_ASSERT_TRUE(test, pte_present(entry));
		KUNIT_EXPECT_EQ(test, pte_phys_mm(coarse->vm_mm, entry), page_to_phys(&folio->page));
		for (offset = 0; offset < SZ_64K; offset += SZ_4K) {
			KUNIT_ASSERT_EQ(test, test_file_fault(small, TEST_VA + offset, false) &
					VM_FAULT_ERROR, 0U);
			entry = test_fault_entry(test, small->vm_mm, TEST_VA + offset);
			KUNIT_EXPECT_EQ(test, pte_phys_mm(small->vm_mm, entry),
					page_to_phys(&folio->page) + offset);
		}
		{
			struct folio *first = filemap_get_folio(file->f_mapping, 0);

			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, first);
			KUNIT_EXPECT_EQ(test, page_to_phys(&first->page), untouched);
			folio_put(first);
		}
		kunit_release_action(test, user4k_vma_mm_free, coarse->vm_mm);
		kunit_release_action(test, user4k_vma_mm_free, small->vm_mm);
		lru_add_drain_all();
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)nr + 1);
	}
}

static void user64k_shmem_fault_test(struct kunit *test)
{
	unsigned int mode;

	if (!test_coarse_file_enabled(test))
		return;
	if (!IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE)) {
		kunit_skip(test, "coarse shmem promotion requires collapse support");
		return;
	}
	for (mode = 0; mode < 4; mode++) {
		bool seeded = mode & 1, shared = mode < 2;
		struct file *file = test_cache_file(test, SZ_64K, seeded);
		struct vm_area_struct *vma;
		struct folio *cache, *mapped;
		unsigned long offset;
		pte_t entry;
		vm_fault_t fault;
		unsigned int attempt;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		vma = test_file_vma_granule(test, file, 16, shared, TEST_VA, SZ_64K, 0);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		for (attempt = 0; attempt < 8; attempt++) {
			fault = test_file_fault(vma, TEST_VA, true);
			if (!(fault & VM_FAULT_NOPAGE))
				break;
		}
		KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY | VM_FAULT_NOPAGE), 0U);
		entry = test_fault_entry(test, vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_TRUE(test, pte_present(entry));
		KUNIT_EXPECT_TRUE(test, pte_write(entry));
		mapped = page_folio(pfn_to_page(PHYS_PFN(pte_phys_mm(vma->vm_mm, entry))));
		KUNIT_EXPECT_EQ(test, folio_size(mapped), SZ_64K);
		KUNIT_EXPECT_EQ(test, !!folio_test_anon(mapped), !shared);
		for (offset = 0; offset < SZ_64K; offset += SZ_4K)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mapped) + offset,
						seeded ? 0x31 + offset / SZ_4K : 0, SZ_4K), NULL);
		cache = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cache);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, cache), 0);
		KUNIT_EXPECT_EQ(test, folio_size(cache), SZ_64K);
		if (shared)
			KUNIT_EXPECT_PTR_EQ(test, mapped, cache);
		else {
			KUNIT_EXPECT_PTR_NE(test, mapped, cache);
			memset(folio_address(mapped), 0xc9, SZ_64K);
			for (offset = 0; offset < SZ_64K; offset += SZ_4K)
				KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(cache) + offset,
						seeded ? 0x31 + offset / SZ_4K : 0, SZ_4K), NULL);
		}
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm,
				shared ? MM_SHMEMPAGES : MM_ANONPAGES), 1L);
		KUNIT_EXPECT_EQ(test, file->f_mapping->nrpages, SZ_64K / PAGE_SIZE);
		{
			u8 byte = 0x83, observed = 0;

			KUNIT_EXPECT_EQ(test, access_remote_vm(vma->vm_mm, TEST_VA + SZ_64K - 1,
							     &byte, 1, FOLL_WRITE), 1);
			KUNIT_EXPECT_EQ(test, access_remote_vm(vma->vm_mm, TEST_VA + SZ_64K - 1,
							     &observed, 1, 0), 1);
			KUNIT_EXPECT_EQ(test, observed, byte);
			KUNIT_EXPECT_EQ(test, ((u8 *)folio_address(cache))[SZ_64K - 1],
					shared ? byte : (u8)(seeded ? 0x40 : 0));
		}
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
		lru_add_drain_all();
		KUNIT_EXPECT_EQ(test, folio_mapcount(cache), 0);
		KUNIT_EXPECT_EQ(test, folio_ref_count(cache), (int)(SZ_64K / PAGE_SIZE) + 1);
	}
}

static void user64k_shmem_fault_busy_test(struct kunit *test)
{
	unsigned int mode;

	if (!test_coarse_file_enabled(test))
		return;
	if (!IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE)) {
		kunit_skip(test, "coarse shmem promotion requires collapse support");
		return;
	}
	for (mode = 0; mode < 3; mode++) {
		struct file *file = test_cache_file(test, SZ_64K, mode != 0);
		struct vm_area_struct *vma;
		struct folio *held;
		vm_fault_t fault;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		vma = test_file_vma_granule(test, file, 16, true, TEST_VA, SZ_64K, 0);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		if (!mode) {
			mmap_read_lock(vma->vm_mm);
			fault = handle_mm_fault(vma, TEST_VA, FAULT_FLAG_REMOTE |
					FAULT_FLAG_ALLOW_RETRY | FAULT_FLAG_RETRY_NOWAIT, NULL);
			mmap_assert_locked(vma->vm_mm);
			mmap_read_unlock(vma->vm_mm);
			KUNIT_EXPECT_EQ(test, fault, (vm_fault_t)VM_FAULT_RETRY);
			KUNIT_EXPECT_EQ(test, file->f_mapping->nrpages, 0UL);
		} else if (mode == 1) {
			held = filemap_get_folio(file->f_mapping, SZ_64K / (2 * PAGE_SIZE));
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, held);
			folio_lock(held);
			fault = test_file_fault(vma, TEST_VA, false);
			folio_unlock(held);
			folio_put(held);
			KUNIT_EXPECT_EQ(test, fault, (vm_fault_t)VM_FAULT_NOPAGE);
			KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_SHMEMPAGES), 0L);
		} else {
			struct vm_area_struct *small = test_file_vma(test, file, false, true,
								  TEST_VA, SZ_64K, 0);
			struct user_page_fragment *fragment;

			KUNIT_ASSERT_NOT_NULL(test, small);
			fragment = kunit_kzalloc(test, sizeof(*fragment), GFP_KERNEL);
			KUNIT_ASSERT_NOT_NULL(test, fragment);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_promote_unpin,
								      fragment), 0);
			KUNIT_ASSERT_EQ(test, pin_user_fragments_remote(small->vm_mm,
					TEST_VA + SZ_64K / 2, 1, FOLL_WRITE, fragment, 1), 1L);
			fault = test_file_fault(vma, TEST_VA, false);
			KUNIT_EXPECT_TRUE(test, fault & VM_FAULT_SIGBUS);
			KUNIT_EXPECT_EQ(test, *(u8 *)(folio_address(fragment->folio) +
							 fragment->offset), (u8)0x39);
			kunit_release_action(test, test_promote_unpin, fragment);
			kunit_release_action(test, user4k_vma_mm_free, small->vm_mm);
		}
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, false) &
				(VM_FAULT_ERROR | VM_FAULT_RETRY | VM_FAULT_NOPAGE), 0U);
		KUNIT_EXPECT_TRUE(test, pte_present(test_fault_entry(test, vma->vm_mm, TEST_VA)));
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
	}
}

static void user64k_shmem_fault_hole_test(struct kunit *test)
{
#ifdef CONFIG_USERFAULTFD
	unsigned int adjacent;

	if (!test_coarse_file_enabled(test))
		return;
	if (!IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE)) {
		kunit_skip(test, "coarse shmem promotion requires collapse support");
		return;
	}
	for (adjacent = 0; adjacent < 2; adjacent++) {
		struct file *file = test_cache_file(test, 2 * SZ_64K, false);
		struct vm_area_struct *coarse, *missing, *seed;
		struct folio *folio;
		vm_fault_t fault;
		bool new;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		coarse = test_file_vma_granule(test, file, 16, true, TEST_VA, SZ_64K, 0);
		missing = test_file_vma(test, file, false, true, TEST_VA, SZ_4K,
					adjacent ? SZ_64K : PAGE_SIZE);
		KUNIT_ASSERT_NOT_NULL(test, coarse);
		KUNIT_ASSERT_NOT_NULL(test, missing);
		seed = test_file_vma(test, file, false, true, TEST_VA, SZ_4K, 0);
		KUNIT_ASSERT_NOT_NULL(test, seed);
		mmap_read_lock(seed->vm_mm);
		folio = shmem_uffd_prepare_folio(seed, TEST_VA, &new);
		mmap_read_unlock(seed->vm_mm);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_TRUE(test, new);
		memset(folio_address(folio), 0x6f, SZ_4K);
		shmem_uffd_mark_present(folio, 0, BIT(0));
		KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 0),
				GENMASK(PAGE_SIZE / SZ_4K - 1, 1));
		folio_mark_dirty(folio);
		folio_unlock(folio);
		folio_put(folio);
		/* Model the registration flag used by the collapse interval check. */
		mmap_write_lock(missing->vm_mm);
		vma_start_write(missing);
		vm_flags_set(missing, VM_UFFD_MISSING);
		mmap_write_unlock(missing->vm_mm);
		fault = test_file_fault(coarse, TEST_VA, false);
		mmap_write_lock(missing->vm_mm);
		vma_start_write(missing);
		vm_flags_clear(missing, VM_UFFD_MISSING);
		mmap_write_unlock(missing->vm_mm);
		/* A real coarse fault may populate holes covered by its entire leaf. */
		KUNIT_EXPECT_EQ(test, fault & VM_FAULT_ERROR, 0U);
		KUNIT_ASSERT_EQ(test, test_file_fault(coarse, TEST_VA, false) &
				(VM_FAULT_ERROR | VM_FAULT_RETRY | VM_FAULT_NOPAGE), 0U);
		folio = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_EQ(test, folio_size(folio), SZ_64K);
		folio_lock(folio);
		KUNIT_EXPECT_FALSE(test, shmem_uffd_range_missing(folio, 1, 0, SZ_4K));
		folio_unlock(folio);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0x6f, SZ_4K), NULL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K, 0,
						SZ_64K - SZ_4K), NULL);
		folio_put(folio);
		kunit_release_action(test, user4k_vma_mm_free, coarse->vm_mm);
		kunit_release_action(test, user4k_vma_mm_free, missing->vm_mm);
		kunit_release_action(test, user4k_vma_mm_free, seed->vm_mm);
	}
#else
	kunit_skip(test, "requires userfaultfd registration flags");
#endif
}

static void user64k_file_leaf_test(struct kunit *test)
{
	struct file *file;
	struct vm_area_struct *coarse, *native, *small, *child_vma;
	struct mm_struct *coarse_mm, *child;
	struct folio *folio;
	unsigned long offset;
	unsigned int nr = (2 * SZ_64K) / PAGE_SIZE;
	unsigned int small_nr = 2 * SZ_64K / SZ_4K - 1;
	unsigned int maps = 2 * nr + small_nr;
	u8 byte = 0xc7, observed;
	int ret;

	if (!test_coarse_file_enabled(test))
		return;
	file = test_coarse_cache_file(test, 17 - PAGE_SHIFT, 2 * SZ_64K);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	coarse = test_file_vma_granule(test, file, 16, true, TEST_VA, 2 * SZ_64K, 0);
	native = test_file_vma(test, file, true, true, TEST_VA, 2 * SZ_64K, 0);
	small = test_file_vma(test, file, false, true, TEST_VA + SZ_4K,
			     2 * SZ_64K - SZ_4K, SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, coarse);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_NOT_NULL(test, small);
	coarse_mm = coarse->vm_mm;
	KUNIT_ASSERT_EQ(test, test_file_fault(coarse, TEST_VA + SZ_64K, false) &
			VM_FAULT_ERROR, 0U);
	folio = filemap_get_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	/* One cache folio supplies two user leaves, with native references per leaf. */
	for (offset = 0; offset < 2 * SZ_64K; offset += SZ_64K) {
		pte_t entry = test_fault_entry(test, coarse_mm, TEST_VA + offset);

		KUNIT_ASSERT_TRUE(test, pte_present(entry));
		KUNIT_EXPECT_EQ(test, pte_phys_mm(coarse_mm, entry),
				page_to_phys(&folio->page) + offset);
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(coarse_mm, MM_FILEPAGES), 2L);
	for (offset = 0; offset < 2 * SZ_64K; offset += PAGE_SIZE)
		KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA + offset, false) &
				VM_FAULT_ERROR, 0U);
	for (offset = SZ_4K; offset < 2 * SZ_64K; offset += SZ_4K)
		KUNIT_ASSERT_EQ(test, test_file_fault(small, TEST_VA + offset, false) &
				VM_FAULT_ERROR, 0U);
	lru_add_drain_all();
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)maps);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)(nr + maps + 1));
	KUNIT_ASSERT_EQ(test, access_remote_vm(coarse_mm, TEST_VA + SZ_64K + 3 * SZ_4K,
					    &byte, 1, FOLL_WRITE), 1);
	KUNIT_ASSERT_EQ(test, access_remote_vm(small->vm_mm, TEST_VA + SZ_64K + 3 * SZ_4K,
					    &observed, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, observed, byte);
	byte = 0x98;
	KUNIT_ASSERT_EQ(test, access_remote_vm(small->vm_mm, TEST_VA + 7 * SZ_4K,
					    &byte, 1, FOLL_WRITE), 1);
	KUNIT_ASSERT_EQ(test, access_remote_vm(native->vm_mm, TEST_VA + 7 * SZ_4K,
					    &observed, 1, 0), 1);
	KUNIT_EXPECT_EQ(test, observed, byte);
	KUNIT_EXPECT_EQ(test, ((u8 *)folio_address(folio))[7 * SZ_4K], byte);

	child = test_vma_mm_granule(test, 16);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, coarse_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	/* Ordinary file VMAs are lazily populated in the child after fork. */
	mmap_read_lock(child);
	child_vma = find_vma(child, TEST_VA);
	mmap_read_unlock(child);
	KUNIT_ASSERT_NOT_NULL(test, child_vma);
	KUNIT_ASSERT_EQ(test, test_file_fault(child_vma, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(child_vma, TEST_VA + SZ_64K, false) &
			VM_FAULT_ERROR, 0U);
	maps += nr;
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)maps);
	kthread_use_mm(coarse_mm);
	ret = vm_munmap(TEST_VA, SZ_64K);
	kthread_unuse_mm(coarse_mm);
	KUNIT_ASSERT_EQ(test, ret, 0);
	maps -= SZ_64K / PAGE_SIZE;
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)maps);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(coarse_mm, MM_FILEPAGES), 1L);
	KUNIT_EXPECT_TRUE(test, pte_present(test_fault_entry(test, child, TEST_VA)));

	/* The native-rounded range must remove every overlapping coarse alias. */
	unmap_mapping_range(file->f_mapping, SZ_64K + SZ_4K, SZ_4K, 0);
	KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, coarse_mm, TEST_VA + SZ_64K)));
	KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, child, TEST_VA + SZ_64K)));
	KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, native->vm_mm, TEST_VA + SZ_64K)));
	KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, small->vm_mm, TEST_VA + SZ_64K + SZ_4K)));
	KUNIT_EXPECT_TRUE(test, pte_present(test_fault_entry(test, small->vm_mm,
							  TEST_VA + SZ_64K + PAGE_SIZE)));
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(coarse_mm, MM_FILEPAGES), 0L);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio),
			(int)(maps - 2 * SZ_64K / PAGE_SIZE - 1 - PAGE_SIZE / SZ_4K));

	/* An exact subpage hole keeps neighboring small leaves in that native page. */
	mmap_read_lock(coarse_mm);
	coarse = find_vma(coarse_mm, TEST_VA + SZ_64K);
	mmap_read_unlock(coarse_mm);
	KUNIT_ASSERT_NOT_NULL(test, coarse);
	KUNIT_ASSERT_EQ(test, test_file_fault(coarse, TEST_VA + SZ_64K, false) &
			VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(child_vma, TEST_VA + SZ_64K, false) &
			VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA + SZ_64K, false) &
			VM_FAULT_ERROR, 0U);
	for (offset = 0; offset < PAGE_SIZE; offset += SZ_4K)
		KUNIT_ASSERT_EQ(test, test_file_fault(small, TEST_VA + SZ_64K + offset,
						    false) & VM_FAULT_ERROR, 0U);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)maps);
	unmap_mapping_subpage_hole(file->f_mapping, SZ_64K + SZ_4K, SZ_64K + 2 * SZ_4K);
	KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, coarse_mm, TEST_VA + SZ_64K)));
	KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, child, TEST_VA + SZ_64K)));
	KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, native->vm_mm, TEST_VA + SZ_64K)));
	for (offset = 0; offset < PAGE_SIZE; offset += SZ_4K)
		KUNIT_EXPECT_EQ(test, !!pte_present(test_fault_entry(test, small->vm_mm,
						TEST_VA + SZ_64K + offset)), offset != SZ_4K);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)(maps - 2 * SZ_64K / PAGE_SIZE - 2));
	kunit_release_action(test, user4k_vma_mm_free, coarse_mm);
	kunit_release_action(test, user4k_vma_mm_free, child);
	kunit_release_action(test, user4k_vma_mm_free, native->vm_mm);
	kunit_release_action(test, user4k_vma_mm_free, small->vm_mm);
	lru_add_drain_all();
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)nr + 1);
}

static void user64k_file_cow_test(struct kunit *test)
{
	unsigned int cold;

	if (!test_coarse_file_enabled(test))
		return;
	for (cold = 0; cold < 2; cold++) {
		struct file *file = test_coarse_cache_file(test, 17 - PAGE_SHIFT, 2 * SZ_64K);
		struct vm_area_struct *vma;
		struct folio *cache, *anon;
		unsigned long offset;
		pte_t entry;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		vma = test_file_vma_granule(test, file, 16, false, TEST_VA, SZ_64K, SZ_64K);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		if (!cold)
			KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, false) & VM_FAULT_ERROR, 0U);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		entry = test_fault_entry(test, vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_TRUE(test, pte_present(entry));
		KUNIT_EXPECT_TRUE(test, pte_write(entry));
		anon = page_folio(pfn_to_page(PHYS_PFN(pte_phys_mm(vma->vm_mm, entry))));
		KUNIT_EXPECT_TRUE(test, folio_test_anon(anon));
		KUNIT_EXPECT_EQ(test, folio_size(anon), SZ_64K);
		folio_get(anon);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, anon), 0);
		for (offset = 0; offset < SZ_64K; offset += SZ_4K)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(anon) + offset,
						0x41 + offset / SZ_4K, SZ_4K), NULL);
		memset(folio_address(anon), 0x93, SZ_64K);
		cache = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cache);
		for (offset = 0; offset < 2 * SZ_64K; offset += SZ_4K)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(cache) + offset,
						0x31 + offset / SZ_4K, SZ_4K), NULL);
		KUNIT_EXPECT_EQ(test, folio_mapcount(cache), 0);
		folio_put(cache);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_FILEPAGES), 0L);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_ANONPAGES), 1L);
		lru_add_drain_all();
		KUNIT_EXPECT_EQ(test, folio_mapcount(anon), (int)(SZ_64K / PAGE_SIZE));
		KUNIT_EXPECT_EQ(test, folio_ref_count(anon), (int)(SZ_64K / PAGE_SIZE) + 1);
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
		lru_add_drain_all();
		KUNIT_EXPECT_EQ(test, folio_mapcount(anon), 0);
		KUNIT_EXPECT_EQ(test, folio_ref_count(anon), 1);
	}
}

static void user64k_file_bounds_test(struct kunit *test)
{
	struct file *file;
	struct vm_area_struct *vma;
	struct folio *folio;
	unsigned int mode;
	unsigned long size = SZ_64K + PAGE_SIZE + 19;
	pte_t entry;

	if (!test_coarse_file_enabled(test))
		return;
	/* Too-small backing and a physically unaligned source cannot form a leaf. */
	for (mode = 0; mode < 2; mode++) {
		file = test_coarse_cache_file(test, mode ? 17 - PAGE_SHIFT : 0, 2 * SZ_64K);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		vma = test_file_vma_granule(test, file, 16, false, TEST_VA, SZ_64K,
					  mode ? PAGE_SIZE : 0);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_EXPECT_TRUE(test, test_file_fault(vma, TEST_VA, false) & VM_FAULT_SIGBUS);
		KUNIT_EXPECT_TRUE(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_SIGBUS);
		KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, vma->vm_mm, TEST_VA)));
		folio = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
		folio_put(folio);
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
	}
	file = test_coarse_cache_file(test, 17 - PAGE_SHIFT, size);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	vma = test_file_vma_granule(test, file, 16, true, TEST_VA, 3 * SZ_64K, 0);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA + SZ_64K, false) & VM_FAULT_ERROR, 0U);
	entry = test_fault_entry(test, vma->vm_mm, TEST_VA + SZ_64K);
	KUNIT_ASSERT_TRUE(test, pte_present(entry));
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(vma->vm_mm, entry)) +
					  PAGE_SIZE + 19, 0, SZ_64K - PAGE_SIZE - 19), NULL);
	KUNIT_EXPECT_TRUE(test, test_file_fault(vma, TEST_VA + 2 * SZ_64K, false) & VM_FAULT_SIGBUS);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_FILEPAGES), 2L);
}

static void user4k_file_shared_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, 4 * PAGE_SIZE, true);
	struct vm_area_struct *small, *native;
	struct folio *folio;
	unsigned int i;
	pte_t entry;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	native = test_file_vma(test, file, true, true, TEST_VA, 4 * PAGE_SIZE, 0);
	/* Virtual and file offsets deliberately have different native alignment. */
	small = test_file_vma(test, file, false, true, TEST_VA + SZ_4K,
			      3 * PAGE_SIZE, 2 * SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_EQ(test, test_file_fault(small, small->vm_start + SZ_4K, false) &
			VM_FAULT_ERROR, 0U);
	/* Fault-around keeps its 64K window and populates all ready cache folios. */
	for (i = 0; i < 12; i++) {
		unsigned long addr = small->vm_start + i * SZ_4K;
		struct vm_page_offset pos = vma_page_offset_at(small, addr);

		entry = test_fault_entry(test, small->vm_mm, addr);
		KUNIT_ASSERT_TRUE(test, pte_present(entry));
		folio = filemap_get_folio(file->f_mapping, pos.index);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(small->vm_mm, entry),
			page_to_phys(folio_file_page(folio, pos.index)) + pos.offset);
		KUNIT_EXPECT_EQ(test, *(u8 *)phys_to_virt(pte_phys_mm(small->vm_mm, entry)),
			(u8)(0x33 + i));
		folio_put(folio);
	}
	KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA, true) & VM_FAULT_ERROR, 0U);
	entry = test_fault_entry(test, native->vm_mm, TEST_VA);
	KUNIT_ASSERT_TRUE(test, pte_present(entry));
	memset(phys_to_virt(pte_phys_mm(native->vm_mm, entry)) + 2 * SZ_4K, 0x72, SZ_4K);
	entry = test_fault_entry(test, small->vm_mm, small->vm_start);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(small->vm_mm, entry)),
					   0x72, SZ_4K), NULL);
	KUNIT_ASSERT_EQ(test, test_file_fault(small, small->vm_start + SZ_4K, true) &
			VM_FAULT_ERROR, 0U);
	entry = test_fault_entry(test, small->vm_mm, small->vm_start + SZ_4K);
	KUNIT_EXPECT_TRUE(test, pte_write(entry));
	memset(phys_to_virt(pte_phys_mm(small->vm_mm, entry)), 0x84, SZ_4K);
	entry = test_fault_entry(test, native->vm_mm, TEST_VA);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(native->vm_mm, entry)) +
					   3 * SZ_4K, 0x84, SZ_4K), NULL);
	kunit_release_action(test, user4k_vma_mm_free, small->vm_mm);
	kunit_release_action(test, user4k_vma_mm_free, native->vm_mm);
	folio = filemap_get_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	folio_put(folio);
}

static void user4k_file_cow_test(struct kunit *test)
{
	unsigned int cold;

	for (cold = 0; cold < 2; cold++) {
		struct file *file = test_cache_file(test, PAGE_SIZE, true);
		struct vm_area_struct *vma;
		struct folio *cache;
		pte_t entry;
		unsigned int i;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		vma = test_file_vma(test, file, false, false, TEST_VA + SZ_4K,
				    3 * SZ_4K, SZ_4K);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		if (!cold)
			KUNIT_ASSERT_EQ(test, test_file_fault(vma, vma->vm_start, false) &
					VM_FAULT_ERROR, 0U);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, vma->vm_start + SZ_4K, true) &
				VM_FAULT_ERROR, 0U);
		entry = test_fault_entry(test, vma->vm_mm, vma->vm_start + SZ_4K);
		KUNIT_ASSERT_TRUE(test, pte_present(entry));
		KUNIT_EXPECT_TRUE(test, pte_write(entry));
		KUNIT_EXPECT_TRUE(test, folio_test_anon(page_folio(pte_page(entry))));
		/* Private COW can pack into any physical quarter of its backing. */
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(vma->vm_mm, entry)),
						   0x33, SZ_4K), NULL);
		memset(phys_to_virt(pte_phys_mm(vma->vm_mm, entry)), 0x91, SZ_4K);
		cache = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cache);
		for (i = 0; i < 4; i++)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(cache) + i * SZ_4K,
							   0x31 + i, SZ_4K), NULL);
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
		KUNIT_EXPECT_EQ(test, folio_mapcount(cache), 0);
		folio_put(cache);
	}
}

static void user4k_file_eof_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, SZ_4K + 37, false);
	struct vm_area_struct *small, *native;
	unsigned int i;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	small = test_file_vma(test, file, false, true, TEST_VA, PAGE_SIZE, 0);
	native = test_file_vma(test, file, true, true, TEST_VA, PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, test_file_fault(small, TEST_VA + SZ_4K, true) & VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA + 3 * SZ_4K, false) & VM_FAULT_ERROR, 0U);
	for (i = 0; i < 4; i++) {
		vm_fault_t ret = test_file_fault(small, TEST_VA + i * SZ_4K, false);

		KUNIT_EXPECT_EQ(test, ret & VM_FAULT_ERROR, i < 2 ? 0U : VM_FAULT_SIGBUS);
		KUNIT_EXPECT_EQ(test, pte_present(test_fault_entry(test, small->vm_mm,
								 TEST_VA + i * SZ_4K)), i < 2);
	}
}


static const struct vm_operations_struct test_filemap_fault_ops = {
	.fault = filemap_fault,
	.page_mkwrite = filemap_page_mkwrite,
};

static void user4k_filemap_fault_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, 2 * SZ_4K + 17, true);
	struct vm_area_struct *vma;
	unsigned int i;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	vma = test_file_vma(test, file, false, true, TEST_VA + 3 * SZ_4K, PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	/* Exercise generic filemap_fault against an already populated cache. */
	vma->vm_ops = &test_filemap_fault_ops;
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, vma->vm_start + SZ_4K, true) & VM_FAULT_ERROR, 0U);
	for (i = 0; i < 3; i++) {
		pte_t pte = test_fault_entry(test, vma->vm_mm, vma->vm_start + i * SZ_4K);

		KUNIT_ASSERT_TRUE(test, pte_present(pte));
		KUNIT_EXPECT_EQ(test, *(u8 *)phys_to_virt(pte_phys_mm(vma->vm_mm, pte)), (u8)(0x31 + i));
	}
	KUNIT_EXPECT_EQ(test, test_file_fault(vma, vma->vm_start + 3 * SZ_4K, false) &
			VM_FAULT_ERROR, VM_FAULT_SIGBUS);
}

static void user4k_file_truncate_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, 2 * PAGE_SIZE, true);
	struct vm_area_struct *small, *cow, *native;
	struct iattr attr = { .ia_valid = ATTR_SIZE, .ia_size = SZ_4K + 37 };
	unsigned int i;
	int ret;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	small = test_file_vma(test, file, false, true, TEST_VA + SZ_4K,
			      PAGE_SIZE, SZ_4K);
	cow = test_file_vma(test, file, false, false, TEST_VA, PAGE_SIZE, 0);
	native = test_file_vma(test, file, true, false, TEST_VA, PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_NOT_NULL(test, cow);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, test_file_fault(small, small->vm_start, false) & VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(cow, TEST_VA + 3 * SZ_4K, true) & VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA, true) & VM_FAULT_ERROR, 0U);
	inode_lock(file_inode(file));
	ret = notify_change(&nop_mnt_idmap, file->f_path.dentry, &attr, NULL);
	inode_unlock(file_inode(file));
	KUNIT_ASSERT_EQ(test, ret, 0);
	for (i = 0; i < 4; i++) {
		unsigned long addr = small->vm_start + i * SZ_4K;

		if (i)
			KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, small->vm_mm, addr)));
		KUNIT_EXPECT_EQ(test, test_file_fault(small, addr, false) & VM_FAULT_ERROR,
				i ? VM_FAULT_SIGBUS : 0U);
	}
	KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, cow->vm_mm, TEST_VA + 3 * SZ_4K)));
	KUNIT_EXPECT_EQ(test, test_file_fault(cow, TEST_VA + 3 * SZ_4K, false) & VM_FAULT_ERROR,
			VM_FAULT_SIGBUS);
	KUNIT_EXPECT_TRUE(test, pte_present(test_fault_entry(test, native->vm_mm, TEST_VA)));
	KUNIT_EXPECT_EQ(test, test_file_fault(native, TEST_VA + 3 * SZ_4K, false) & VM_FAULT_ERROR, 0U);
}

static void user4k_file_invalidate_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, 2 * PAGE_SIZE, true);
	struct vm_area_struct *vma;
	unsigned int i;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	vma = test_file_vma(test, file, false, true, TEST_VA + SZ_4K,
			    4 * SZ_4K, PAGE_SIZE - SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, vma->vm_start, false) & VM_FAULT_ERROR, 0U);
	unmap_mapping_pages(file->f_mapping, 0, 1, false);
	for (i = 0; i < 4; i++)
		KUNIT_EXPECT_EQ(test, pte_present(test_fault_entry(test, vma->vm_mm,
				vma->vm_start + i * SZ_4K)), i != 0);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, vma->vm_start, false) & VM_FAULT_ERROR, 0U);
	unmap_mapping_pages(file->f_mapping, 1, 1, false);
	for (i = 0; i < 4; i++)
		KUNIT_EXPECT_EQ(test, pte_present(test_fault_entry(test, vma->vm_mm,
				vma->vm_start + i * SZ_4K)), i == 0);
}

static void user4k_file_fork_hole_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, 2 * PAGE_SIZE, true);
	struct vm_area_struct *shared, *private, *cvma;
	struct mm_struct *child;
	struct inode *inode;
	unsigned int i;
	pte_t copied;
	int ret;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	shared = test_file_vma(test, file, false, true, TEST_VA, 2 * PAGE_SIZE, 0);
	private = test_file_vma(test, file, false, false, TEST_VA, 2 * PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, shared);
	KUNIT_ASSERT_NOT_NULL(test, private);
	KUNIT_ASSERT_EQ(test, test_file_fault(shared, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, test_file_fault(private, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, private->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	mmap_read_lock(child);
	cvma = find_vma(child, TEST_VA);
	mmap_read_unlock(child);
	KUNIT_ASSERT_NOT_NULL(test, cvma);
	KUNIT_ASSERT_EQ(test, test_file_fault(cvma, TEST_VA + SZ_4K, true) & VM_FAULT_ERROR, 0U);
	copied = test_fault_entry(test, child, TEST_VA + SZ_4K);
	memset(phys_to_virt(pte_phys_mm(child, copied)), 0x82, SZ_4K);
	inode = file_inode(file);
	/* A sub-native-page hole zeros shared bytes but retains private COW. */
	ret = vfs_fallocate(file, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE, SZ_4K, SZ_4K);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, i_size_read(inode), (loff_t)(2 * PAGE_SIZE));
	for (i = 0; i < 2 * TEST_SLOTS; i++) {
		pte_t pte;

		KUNIT_ASSERT_EQ(test, test_file_fault(shared, TEST_VA + i * SZ_4K, false) & VM_FAULT_ERROR, 0U);
		pte = test_fault_entry(test, shared->vm_mm, TEST_VA + i * SZ_4K);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(shared->vm_mm, pte)),
						   i == 1 ? 0 : 0x31 + i, SZ_4K), NULL);
	}
	KUNIT_EXPECT_EQ(test, pte_phys_mm(child, test_fault_entry(test, child, TEST_VA + SZ_4K)),
			pte_phys_mm(child, copied));
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(child, copied)), 0x82, SZ_4K), NULL);
	/* A full cache-page hole exercises the file interval-tree invalidation. */
	ret = vfs_fallocate(file, FALLOC_FL_PUNCH_HOLE | FALLOC_FL_KEEP_SIZE, PAGE_SIZE, PAGE_SIZE);
	KUNIT_ASSERT_EQ(test, ret, 0);
	for (i = TEST_SLOTS; i < 2 * TEST_SLOTS; i++) {
		KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, shared->vm_mm, TEST_VA + i * SZ_4K)));
		KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, child, TEST_VA + i * SZ_4K)));
	}
}

static void test_zero_cow_granule(struct kunit *test, unsigned int shift)
{
	unsigned long size = 1UL << shift;
	struct vm_area_struct *vma = test_fault_vma_granule(test, shift, TEST_VA, PAGE_SIZE);
	struct mm_struct *mm;
	struct mm_subpage *slot;
	pte_t before[4], after;
	unsigned int i;
	vm_fault_t ret;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	mm = vma->vm_mm;
	for (i = 0; i < 4; i++) {
		mmap_read_lock(mm);
		ret = handle_mm_fault(vma, TEST_VA + i * size,
				      FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(mm);
		KUNIT_ASSERT_EQ(test, ret, (vm_fault_t)0);
		before[i] = test_fault_entry(test, mm, TEST_VA + i * size);
		KUNIT_ASSERT_TRUE(test, pte_present(before[i]));
		KUNIT_ASSERT_TRUE(test, is_zero_pfn(pte_pfn(before[i])));
		KUNIT_ASSERT_FALSE(test, pte_write(before[i]));
	}
	mmap_read_lock(mm);
	ret = handle_mm_fault(vma, TEST_VA + size + 7,
			      FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(mm);
	KUNIT_ASSERT_EQ(test, ret, (vm_fault_t)0);
	lru_add_drain_all();
	for (i = 0; i < 4; i++) {
		after = test_fault_entry(test, mm, TEST_VA + i * size);
		if (i != 1) {
			KUNIT_EXPECT_TRUE(test, pte_same(before[i], after));
			continue;
		}
		KUNIT_ASSERT_TRUE(test, pte_write(after));
		slot = mm_subpage_get_from_phys(pte_phys_mm(mm, after));
		KUNIT_ASSERT_NOT_NULL(test, slot);
		KUNIT_EXPECT_EQ(test, mm_subpage_offset(slot),
				(unsigned int)offset_in_page(pte_phys_mm(mm, after)));
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 1);
		KUNIT_EXPECT_EQ(test, folio_ref_count(mm_subpage_folio(slot)), 2);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(slot)) +
						    mm_subpage_offset(slot), 0, size), NULL);
		mm_subpage_put(slot);
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 1UL);
}

static void user4k_zero_cow_test(struct kunit *test)
{
	test_zero_cow_granule(test, 12);
}

static void user16k_zero_cow_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K alternative faults require larger native backing");
		return;
	}
	test_zero_cow_granule(test, 14);
}

static void test_cow_reuse_granule(struct kunit *test, unsigned int shift)
{
	unsigned long size = 1UL << shift;
	unsigned int mode;

	for (mode = 0; mode < 2; mode++) {
		struct vm_area_struct *vma = test_fault_vma_granule(test, shift, TEST_VA, PAGE_SIZE);
		struct mm_subpage *held = NULL;
		struct mm_struct *child;
		phys_addr_t original[4];
		unsigned int i;
		vm_fault_t fault;
		int ret;
		pte_t pte;

		KUNIT_ASSERT_NOT_NULL(test, vma);
		mmap_read_lock(vma->vm_mm);
		fault = handle_mm_fault(vma, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(vma->vm_mm);
		KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
		for (i = 0; i < 4; i++) {
			pte = test_fault_entry(test, vma->vm_mm, TEST_VA + i * size);
			original[i] = pte_phys_mm(vma->vm_mm, pte);
			memset(phys_to_virt(original[i]), 0xb0 + i, size);
		}
		if (mode) {
			held = mm_subpage_get_from_phys(original[2]);
			KUNIT_ASSERT_NOT_NULL(test, held);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_subpage_put, held), 0);
		}
		child = test_vma_mm_granule(test, shift);
		KUNIT_ASSERT_NOT_NULL(test, child);
		uprobe_start_dup_mmap();
		ret = dup_mmap(child, vma->vm_mm);
		uprobe_end_dup_mmap();
		KUNIT_ASSERT_EQ(test, ret, 0);
		kunit_release_action(test, user4k_vma_mm_free, child);
		for (i = 0; i < 4; i++) {
			unsigned long address = TEST_VA + i * size;

			mmap_read_lock(vma->vm_mm);
			fault = handle_mm_fault(vma, address, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(vma->vm_mm);
			KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
			pte = test_fault_entry(test, vma->vm_mm, address);
			KUNIT_EXPECT_EQ(test, pte_phys_mm(vma->vm_mm, pte) == original[i],
					!(mode && i == 2));
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(vma->vm_mm, pte)),
						0xb0 + i, size), NULL);
			memset(phys_to_virt(pte_phys_mm(vma->vm_mm, pte)), 0xd0 + i, size);
		}
		if (held)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(mm_subpage_phys(held)),
							    0xb2, size), NULL);
	}
}

static void user4k_cow_reuse_test(struct kunit *test)
{
	test_cow_reuse_granule(test, 12);
}

static void user16k_cow_reuse_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K alternative faults require larger native backing");
		return;
	}
	test_cow_reuse_granule(test, 14);
}

static void test_fork_cow_granule(struct kunit *test, unsigned int shift)
{
	unsigned long size = 1UL << shift;
	struct vm_area_struct *parent = test_fault_vma_granule(test, shift, TEST_VA, 4 * size);
	struct mm_struct *child = test_vma_mm_granule(test, shift);
	struct vm_area_struct *cvma;
	struct mm_subpage *held[4];
	pte_t before[4], after;
	unsigned int i, side;
	vm_fault_t fault;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, parent);
	KUNIT_ASSERT_NOT_NULL(test, child);
	mmap_read_lock(parent->vm_mm);
	fault = handle_mm_fault(parent, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(parent->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	for (i = 0; i < 4; i++) {
		before[i] = test_fault_entry(test, parent->vm_mm, TEST_VA + i * size);
		held[i] = mm_subpage_get_from_phys(pte_phys_mm(parent->vm_mm, before[i]));
		KUNIT_ASSERT_NOT_NULL(test, held[i]);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_subpage_put, held[i]), 0);
		memset(folio_address(mm_subpage_folio(held[i])) + mm_subpage_offset(held[i]),
		       0x31 + i, size);
	}
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	cvma = test_vma_lookup(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, cvma);
	mmap_read_lock(child);
	fault = handle_mm_fault(cvma, TEST_VA + size + 15,
				FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(child);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	lru_add_drain_all();
	for (side = 0; side < 2; side++) {
		struct mm_struct *mm = side ? child : parent->vm_mm;

		for (i = 0; i < 4; i++) {
			struct mm_subpage *slot;
			bool changed = side && i == 1;

			after = test_fault_entry(test, mm, TEST_VA + i * size);
			KUNIT_EXPECT_EQ(test, !!pte_write(after), changed);
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, after) != mm_subpage_phys(held[i]), changed);
			slot = mm_subpage_get_from_phys(pte_phys_mm(mm, after));
			KUNIT_ASSERT_NOT_NULL(test, slot);
			KUNIT_EXPECT_EQ(test, mm_subpage_offset(slot),
				(unsigned int)offset_in_page(pte_phys_mm(mm, after)));
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(slot)) +
					mm_subpage_offset(slot), 0x31 + i, size), NULL);
			if (changed) {
				KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 1);
				KUNIT_EXPECT_EQ(test, folio_ref_count(mm_subpage_folio(slot)), 2);
				memset(folio_address(mm_subpage_folio(slot)) +
				       mm_subpage_offset(slot), 0x77, size);
			}
			mm_subpage_put(slot);
		}
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 4UL);
	}
	for (i = 0; i < 4; i++) {
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(held[i]), i == 1 ? 1 : 2);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(held[i])) +
				mm_subpage_offset(held[i]), 0x31 + i, size), NULL);
	}
	KUNIT_EXPECT_EQ(test, folio_ref_count(mm_subpage_folio(held[0])), 11);
}

static void user4k_fork_cow_test(struct kunit *test)
{
	test_fork_cow_granule(test, 12);
}

static void user16k_fork_cow_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K alternative faults require larger native backing");
		return;
	}
	test_fork_cow_granule(test, 14);
}

static void user4k_folio_walk_geometry_test(struct kunit *test)
{
	unsigned int shifts[] = { 12, PAGE_SHIFT, 14 };
	unsigned int which;

	for (which = 0; which < ARRAY_SIZE(shifts); which++) {
		unsigned int shift = shifts[which], leaf;
		struct vm_area_struct *vma;
		struct mm_struct *mm;
		unsigned long start, size = 1UL << shift;
		vm_fault_t fault;

		if (which == 2 && PAGE_SHIFT <= 14)
			continue;
		mm = shift == PAGE_SHIFT ? mm_alloc() : test_vma_mm_granule(test, shift);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		if (shift == PAGE_SHIFT)
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
		start = pmd_size_mm(mm) - size;
		vma = test_fault_vma_in_mm(mm, start, 2 * size);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		for (leaf = 0; leaf < 2; leaf++) {
			unsigned long address = start + leaf * size;
			struct folio_walk fw;
			struct folio *folio;
			struct page *page = NULL;
			pte_t expected, walked = __pte(0);
			bool zero = false;

			mmap_read_lock(mm);
			fault = handle_mm_fault(vma, address, FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(mm);
			KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
			mmap_read_lock(mm);
			folio = folio_walk_start(&fw, vma, address, 0);
			if (folio)
				folio_walk_end(&fw, vma);
			mmap_read_unlock(mm);
			KUNIT_EXPECT_PTR_EQ(test, folio, NULL);
			mmap_read_lock(mm);
			folio = folio_walk_start(&fw, vma, address, FW_ZEROPAGE);
			if (folio) {
				zero = is_zero_folio(folio) && !fw.page && fw.level == FW_LEVEL_PTE;
				folio_walk_end(&fw, vma);
			}
			mmap_read_unlock(mm);
			KUNIT_EXPECT_TRUE(test, zero);

			mmap_read_lock(mm);
			fault = handle_mm_fault(vma, address, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(mm);
			KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
			expected = test_fault_entry(test, mm, address);
			mmap_read_lock(mm);
			folio = folio_walk_start(&fw, vma, address + size - 1, 0);
			if (folio) {
				page = fw.page;
				walked = fw.pte;
				folio_walk_end(&fw, vma);
			}
			mmap_read_unlock(mm);
			KUNIT_EXPECT_PTR_EQ(test, page, pfn_to_page(pte_pfn(expected)));
			KUNIT_EXPECT_TRUE(test, pte_same(walked, expected));
			KUNIT_EXPECT_PTR_EQ(test, folio, page_folio(pfn_to_page(pte_pfn(expected))));
		}
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

static void user4k_fragment_locked_pin_test(struct kunit *test)
{
	unsigned int shifts[] = { 12, PAGE_SHIFT, 14 };
	unsigned int which;

	for (which = 0; which < ARRAY_SIZE(shifts); which++) {
		unsigned int shift = shifts[which];
		struct mm_struct *mm;
		struct vm_area_struct *vma, *found;
		struct user_page_fragment fragment = { };
		unsigned long address;
		int ret;

		if (which == 2 && PAGE_SHIFT <= 14)
			continue;
		mm = shift == PAGE_SHIFT ? mm_alloc() : test_vma_mm_granule(test, shift);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		if (shift == PAGE_SHIFT)
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
		vma = test_fault_vma_in_mm(mm, TEST_VA, PAGE_SIZE);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		address = TEST_VA + (1UL << shift) - sizeof(long);
		mmap_read_lock(mm);
		ret = pin_user_fragment_vma_remote(mm, address, sizeof(long),
					FOLL_WRITE | FOLL_NOFAULT, &fragment, &found);
		mmap_assert_locked(mm);
		mmap_read_unlock(mm);
		KUNIT_EXPECT_EQ(test, ret, -EFAULT);
		KUNIT_EXPECT_PTR_EQ(test, found, NULL);
		mmap_read_lock(mm);
		ret = pin_user_fragment_vma_remote(mm, address, sizeof(long),
					FOLL_WRITE | FOLL_LONGTERM, &fragment, &found);
		mmap_assert_locked(mm);
		mmap_read_unlock(mm);
		KUNIT_EXPECT_EQ(test, ret, -EINVAL);
		KUNIT_EXPECT_PTR_EQ(test, found, NULL);
		mmap_read_lock(mm);
		ret = pin_user_fragment_vma_remote(mm, address, sizeof(long),
					FOLL_WRITE | FOLL_NOWAIT, &fragment, &found);
		mmap_assert_locked(mm);
		mmap_read_unlock(mm);
		KUNIT_EXPECT_EQ(test, ret, -EINVAL);
		KUNIT_EXPECT_PTR_EQ(test, found, NULL);
		/* Only the first fragment is returned, while retaining the VMA lock. */
		mmap_read_lock(mm);
		ret = pin_user_fragment_vma_remote(mm, address, 2 * sizeof(long),
					FOLL_WRITE, &fragment, &found);
		mmap_assert_locked(mm);
		mmap_read_unlock(mm);
		KUNIT_EXPECT_EQ(test, ret, 1);
		if (ret != 1)
			continue;
		KUNIT_EXPECT_PTR_EQ(test, found, vma);
		KUNIT_EXPECT_TRUE(test, fragment.pinned);
		KUNIT_EXPECT_EQ(test, fragment.length, (unsigned int)sizeof(long));
		KUNIT_EXPECT_EQ(test, !!fragment.subpage, shift != PAGE_SHIFT);
		release_user_fragments(&fragment, 1, false);
		/* With backing present, NOFAULT pins the exact same leaf successfully. */
		mmap_read_lock(mm);
		ret = pin_user_fragment_vma_remote(mm, address, sizeof(long),
					FOLL_WRITE | FOLL_NOFAULT, &fragment, &found);
		mmap_assert_locked(mm);
		mmap_read_unlock(mm);
		KUNIT_EXPECT_EQ(test, ret, 1);
		if (ret == 1) {
			KUNIT_EXPECT_PTR_EQ(test, found, vma);
			release_user_fragments(&fragment, 1, true);
		}
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

static void user16k_fragment_offset_test(struct kunit *test)
{
	struct vm_area_struct *vma;
	struct user_page_fragment fragment = {};
	unsigned long address = TEST_VA + 3 * SZ_16K + SZ_4K + 13;
	long ret;

	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "16K alternative fragments require larger native backing");
		return;
	}
	vma = test_fault_vma_granule(test, 14, TEST_VA, PAGE_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	ret = pin_user_fragments_remote(vma->vm_mm, address, 31, FOLL_WRITE, &fragment, 1);
	KUNIT_EXPECT_EQ(test, ret, 1L);
	if (ret != 1)
		return;
	KUNIT_EXPECT_NOT_NULL(test, fragment.subpage);
	KUNIT_EXPECT_EQ(test, fragment.offset, 3UL * SZ_16K + SZ_4K + 13);
	KUNIT_EXPECT_EQ(test, fragment.length, 31U);
	if (fragment.subpage)
		KUNIT_EXPECT_EQ(test, mm_subpage_shift(fragment.subpage), 14U);
	memset(folio_address(fragment.folio) + fragment.offset, 0x9c, fragment.length);
	release_user_fragments(&fragment, 1, true);
	{
		u8 buf[33];

		ret = access_remote_vm(vma->vm_mm, address - 1, buf, sizeof(buf), 0);
		KUNIT_ASSERT_EQ(test, ret, (long)sizeof(buf));
		KUNIT_EXPECT_EQ(test, buf[0], (u8)0);
		KUNIT_EXPECT_EQ(test, buf[32], (u8)0);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buf + 1, 0x9c, 31), NULL);
	}
}

static void user4k_anon_exit_test(struct kunit *test)
{
	struct anon_slot_fixture *f = anon_slot_fixture_create(test);
	struct mm_subpage *held;
	struct anon_vma *root;
	struct folio *folio;
	struct mm_struct *mm;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (i = 0; i < 4; i++) {
		struct mm_subpage *slot = mm_subpage_alloc_at(f->pool, i * SZ_4K);

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(f, i, slot), 0);
	}
	held = f->slots[0];
	folio = mm_subpage_folio(held);
	root = f->vma->anon_vma->root;
	mm_subpage_get(held);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_subpage_put, held), 0);
	mm_subpage_pool_close(f->pool);
	mm_subpage_pool_put(f->pool);
	/* Leave all four mapping refs and the tables to the real exit path. */
	mm = f->mm;
	memset(f, 0, sizeof(*f));
	mmput(mm);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
	KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(held), 0);
	KUNIT_EXPECT_EQ(test, atomic_read(&root->refcount), 1);
	for (i = 1; i < 4; i++) {
		struct mm_subpage *slot = mm_subpage_get_from_phys(mm_subpage_phys(held) + i * SZ_4K);

		KUNIT_EXPECT_PTR_EQ(test, slot, NULL);
		if (slot)
			mm_subpage_put(slot);
	}
}

#ifdef CONFIG_ARM64_CONTPTE
struct contpte_fixture {
	struct mm_struct *mm;
	struct folio *folio;
	unsigned int nr;
};

static void contpte_fixture_free(void *arg)
{
	struct contpte_fixture *f = arg;
	struct mmu_gather tlb;
	unsigned int i;

	if (f->mm) {
		mmap_write_lock(f->mm);
		for (i = 0; i <= f->nr; i++) {
			unsigned long addr = TEST_VA + i * mm_page_size(f->mm);
			spinlock_t *ptl;
			pte_t *pte = test_mm_lookup(f->mm, addr, &ptl);

			if (pte) {
				pte_clear(f->mm, addr, pte);
				pte_unmap_unlock(pte, ptl);
			}
		}
		tlb_gather_mmu_fullmm(&tlb, f->mm);
		free_pgd_range(&tlb, 0, 1UL << 48, 0, 0);
		tlb_finish_mmu(&tlb);
		mmap_write_unlock(f->mm);
		mmput(f->mm);
	}
	if (f->folio)
		folio_put(f->folio);
}

static void test_contpte_granule(struct kunit *test, unsigned int shift)
{
	struct contpte_fixture *f = kunit_kzalloc(test, sizeof(*f), GFP_KERNEL);
	struct vm_area_struct vma = {};
	struct test_mm tables = {};
	struct {
		bool initial, addresses, hints, gathered, aged, access_all, unfolded;
		bool permissions, refolded, holes, full_clear, sentinel;
	} ok;
	unsigned long size;
	phys_addr_t phys;
	pte_t base, saved, observed;
	pte_t *pte;
	spinlock_t *ptl;
	unsigned int i;
	int access_ret, access_same_ret, access_noop_ret;
#ifdef CONFIG_PERF_EVENTS
	unsigned long irqflags;
	u64 reported_size;
#endif

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, contpte_fixture_free, f), 0);
	f->mm = mm_alloc();
	KUNIT_ASSERT_NOT_NULL(test, f->mm);
	if (shift != PAGE_SHIFT)
		KUNIT_ASSERT_EQ(test, test_mm_select_granule(f->mm, shift), 0);
	f->nr = cont_ptes_mm(f->mm);
	size = mm_page_size(f->mm);
	f->folio = folio_alloc(GFP_KERNEL | __GFP_ZERO,
			       get_order(cont_pte_size_mm(f->mm)));
	KUNIT_ASSERT_NOT_NULL(test, f->folio);
	phys = PFN_PHYS(folio_pfn(f->folio));
	tables.mm = f->mm;
	KUNIT_ASSERT_EQ(test, test_map_alias(&tables, 0, TEST_VA, phys), 0);
	vma.vm_mm = f->mm;
	base = phys_pte_mm(f->mm, phys, __pgprot(TEST_PROT | PTE_RDONLY));
	memset(&ok, 1, sizeof(ok));
	mmap_read_lock(f->mm);
	pte = test_mm_lookup(f->mm, TEST_VA, &ptl);
	if (!pte) {
		mmap_read_unlock(f->mm);
		KUNIT_FAIL(test, "missing PTE table");
		return;
	}
	pte_clear(f->mm, TEST_VA, pte);
	/* The adjacent entry must never be included in this contiguous group. */
	saved = pte_mkspecial(base);
	set_pte(pte + f->nr, saved);
	set_ptes(f->mm, TEST_VA, pte, base, f->nr);
	for (i = 0; i < f->nr; i++) {
		observed = __ptep_get(pte + i);
		ok.initial &= pte_cont(observed);
		ok.addresses &= pte_phys_mm(f->mm, observed) == phys + i * size;
		ok.hints &= pte_batch_hint(pte + i, observed) == f->nr - i;
	}
#ifdef CONFIG_PERF_EVENTS
	local_irq_save(irqflags);
	reported_size = perf_get_pgtable_size(f->mm, TEST_VA + size);
	local_irq_restore(irqflags);
#endif
	__set_pte(pte + 3, pte_mkdirty(__ptep_get(pte + 3)));
	observed = ptep_get(pte + 1);
	ok.gathered &= pte_dirty(observed) && pte_young(observed);
	observed = ptep_get_lockless(pte + 2);
	ok.gathered &= pte_dirty(observed) && pte_young(observed) &&
		pte_phys_mm(f->mm, observed) == phys + 2 * size;
	clear_young_dirty_ptes(&vma, TEST_VA + 3 * size, pte + 3, 1,
			      CYDP_CLEAR_YOUNG | CYDP_CLEAR_DIRTY);
	for (i = 0; i < f->nr; i++) {
		observed = __ptep_get(pte + i);
		if (size < PAGE_SIZE)
			ok.aged &= !pte_cont(observed) &&
				pte_young(observed) == (i != 3) && pte_dirty(observed) == (i != 3);
		else
			ok.aged &= !pte_young(observed) && !pte_dirty(observed) &&
				pte_cont(observed);
	}
	observed = pte_mkdirty(pte_mkyoung(__ptep_get(pte + 3)));
	access_same_ret = ptep_set_access_flags(&vma, TEST_VA + 3 * size,
					      pte + 3, observed, 1);
	for (i = 0; i < f->nr; i++) {
		observed = __ptep_get(pte + i);
		ok.access_all &= pte_cont(observed) == (size >= PAGE_SIZE) && pte_young(observed) &&
			pte_dirty(observed) && !pte_write(observed);
	}
	access_noop_ret = ptep_set_access_flags(&vma, TEST_VA + 3 * size,
					      pte + 3, ptep_get(pte + 3), 1);
	observed = pte_mkwrite_novma(pte_mkdirty(pte_mkyoung(__ptep_get(pte + 3))));
	access_ret = ptep_set_access_flags(&vma, TEST_VA + 3 * size, pte + 3,
					 observed, 1);
	for (i = 0; i < f->nr; i++) {
		observed = __ptep_get(pte + i);
		ok.unfolded &= !pte_cont(observed);
		ok.permissions &= pte_write(observed) == (i == 3);
	}
	set_pte_at(f->mm, TEST_VA + 3 * size, pte + 3,
		   pte_advance_pfn_mm(f->mm, base, 3));
	/* Last-entry installation must fold using the physical byte offset too. */
	set_pte_at(f->mm, TEST_VA + (f->nr - 1) * size, pte + f->nr - 1,
		   pte_advance_pfn_mm(f->mm, base, f->nr - 1));
	for (i = 0; i < f->nr; i++)
		ok.refolded &= pte_cont(__ptep_get(pte + i));
	get_and_clear_full_ptes(f->mm, TEST_VA + 3 * size, pte + 3, 2, 0);
	for (i = 0; i < f->nr; i++) {
		observed = __ptep_get(pte + i);
		ok.holes &= pte_none(observed) == (i == 3 || i == 4);
		if (i != 3 && i != 4)
			ok.holes &= !pte_cont(observed) &&
				pte_phys_mm(f->mm, observed) == phys + i * size;
	}
	set_ptes(f->mm, TEST_VA + 3 * size, pte + 3,
		 pte_advance_pfn_mm(f->mm, base, 3), 2);
	set_pte_at(f->mm, TEST_VA + (f->nr - 1) * size, pte + f->nr - 1,
		   pte_advance_pfn_mm(f->mm, base, f->nr - 1));
	for (i = 0; i < f->nr; i++)
		ok.refolded &= pte_cont(__ptep_get(pte + i));
	clear_full_ptes(f->mm, TEST_VA, pte, f->nr, 0);
	for (i = 0; i < f->nr; i++)
		ok.full_clear &= pte_none(__ptep_get(pte + i));
	ok.sentinel = pte_same(__ptep_get(pte + f->nr), saved);
	pte_unmap_unlock(pte, ptl);
	mmap_read_unlock(f->mm);
	KUNIT_EXPECT_TRUE(test, ok.initial);
	KUNIT_EXPECT_TRUE(test, ok.addresses);
	KUNIT_EXPECT_TRUE(test, ok.hints);
	KUNIT_EXPECT_TRUE(test, ok.gathered);
	KUNIT_EXPECT_TRUE(test, ok.aged);
	KUNIT_EXPECT_TRUE(test, ok.access_all);
	KUNIT_EXPECT_EQ(test, access_same_ret, 1);
	KUNIT_EXPECT_EQ(test, access_noop_ret, 0);
	KUNIT_EXPECT_EQ(test, access_ret, 1);
	KUNIT_EXPECT_TRUE(test, ok.unfolded);
	KUNIT_EXPECT_TRUE(test, ok.permissions);
	KUNIT_EXPECT_TRUE(test, ok.refolded);
	KUNIT_EXPECT_TRUE(test, ok.holes);
	KUNIT_EXPECT_TRUE(test, ok.full_clear);
	KUNIT_EXPECT_TRUE(test, ok.sentinel);
#ifdef CONFIG_PERF_EVENTS
	KUNIT_EXPECT_EQ(test, reported_size, (u64)cont_pte_size_mm(f->mm));
#endif
}

static void user4k_contpte_test(struct kunit *test)
{
	test_contpte_granule(test, 12);
}

static void user16k_contpte_test(struct kunit *test)
{
	if (PAGE_SHIFT <= 14) {
		kunit_skip(test, "requires native backing larger than 16K");
		return;
	}
	test_contpte_granule(test, 14);
}

static void native_contpte_test(struct kunit *test)
{
	test_contpte_granule(test, PAGE_SHIFT);
}
#endif

struct usercopy_worker {
	struct test_mm *ctx[2];
	struct completion done;
	unsigned int id;
	unsigned int checks;
	unsigned int idmaps;
	int error;
	u64 tcr;
};

static int test_usercopy_worker(void *arg)
{
	struct usercopy_worker *worker = arg;
	u64 kernel_ttbr = read_sysreg(ttbr1_el1) & ~TTBRx_EL1_ASID_MASK;
	unsigned int round, i, byte;
	bool borrowed = false;
	struct test_mm *ctx = NULL;

	for (round = 0; round < 16; round++) {
		unsigned int small = (round + worker->id) & 1;
		unsigned int cpu = cpumask_first(cpu_online_mask);

		ctx = worker->ctx[small];
		kthread_use_mm(ctx->mm);
		borrowed = true;
		if ((round + worker->id) & 2) {
			unsigned int next = cpumask_next(cpu, cpu_online_mask);

			if (next < nr_cpu_ids)
				cpu = next;
		}
		if (set_cpus_allowed_ptr(current, cpumask_of(cpu))) {
			worker->error = 1;
			break;
		}
		/* Exercise a real scheduler switch while borrowing this mm. */
		schedule_timeout_uninterruptible(1);
		if (small && round < 4) {
			unsigned long flags;
			bool native;

			preempt_disable();
			flags = local_daif_save();
			if (round & 2)
				cpu_install_ttbr0(phys_to_ttbr(__pa_symbol(idmap_pg_dir)),
						  TCR_T0SZ(IDMAP_VA_BITS));
			else
				cpu_install_idmap();
			native = (read_sysreg(tcr_el1) & (TCR_EL1_TG0_MASK | TCR_EL1_T0SZ_MASK)) ==
				 (TCR_TG0_16K | TCR_T0SZ(IDMAP_VA_BITS));
			cpu_uninstall_idmap();
			local_daif_restore(flags);
			preempt_enable();
			if (!native) {
				worker->error = 6;
				break;
			}
			worker->idmaps++;
		}
		worker->tcr = read_sysreg(tcr_el1);
		if ((worker->tcr & TCR_EL1_TG0_MASK) != (small ? TCR_TG0_4K : TCR_TG0_16K) ||
		    (worker->tcr & TCR_EL1_T0SZ_MASK) != TCR_T0SZ(48) ||
		    (worker->tcr & TCR_EL1_TG1_MASK) != TCR_TG1_16K ||
		    (read_sysreg(ttbr1_el1) & ~TTBRx_EL1_ASID_MASK) != kernel_ttbr) {
			worker->error = 2;
			break;
		}
		for (i = 0; i < 4; i++) {
			unsigned long offset = 64 + worker->id * sizeof(u64);
			void __user *address = (void __user *)ctx->address[i];
			u64 value = 0xa5000000 | (worker->id << 16) | (round << 8) | i;
			u64 *backing = phys_to_virt(ctx->phys[i]) + offset;
			unsigned long read_left, write_left;
			u8 data[8];

			/* No VMA/fault path may be entered for these private mappings. */
			pagefault_disable();
			read_left = copy_from_user(data, address, sizeof(data));
			write_left = copy_to_user(address + offset, &value, sizeof(value));
			pagefault_enable();
			if (read_left != (i == 2 ? sizeof(data) : 0) ||
			    write_left != (i == 0 || i == 2 ? sizeof(value) : 0)) {
				worker->error = 3;
				goto out;
			}
			if (i != 2) {
				for (byte = 0; byte < sizeof(data); byte++) {
					if (data[byte] != (small ? 0x40 : 0x80) + i) {
						worker->error = 4;
						goto out;
					}
				}
			}
			if (READ_ONCE(*backing) != (i == 0 || i == 2 ? 0 : value)) {
				worker->error = 5;
				goto out;
			}
			worker->checks++;
		}
		kthread_unuse_mm(ctx->mm);
		borrowed = false;
	}
out:
	if (borrowed)
		kthread_unuse_mm(ctx->mm);
	complete(&worker->done);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);
	return 0;
}

static void test_stop_kthread(void *arg)
{
	kthread_stop(arg);
}

struct cow_reader {
	struct mm_struct *parent;
	struct mm_struct *child;
	struct completion ready;
	struct completion copied;
	struct completion done;
	u8 observed[8];
	unsigned long read_left[8], write_left[8];
	int error;
	unsigned int checks;
};

static int test_cow_reader(void *arg)
{
	struct cow_reader *r = arg;
	unsigned int i, side;

	kthread_use_mm(r->child);
	/* Resolve fork's old PTEs normally, then cache pre-COW translations. */
	for (i = 0; i < 4; i++) {
		u8 value = 0;
		unsigned long left;

		left = copy_from_user(&value, (void __user *)(TEST_VA + i * SZ_4K + 64), 1);
		if (left || value != 0x41 + i)
			r->error = 1;
	}
	complete(&r->ready);
	if (!wait_for_completion_timeout(&r->copied, 10 * HZ)) {
		r->error = 2;
		goto out;
	}
	for (side = 0; side < 2; side++) {
		if (side) {
			kthread_unuse_mm(r->child);
			kthread_use_mm(r->parent);
		}
		for (i = 0; i < 4; i++) {
			void __user *addr = (void __user *)(TEST_VA + i * SZ_4K + 64);
			u8 value = 0, changed = 0x99;
			unsigned long read_left, write_left;

			pagefault_disable();
			read_left = copy_from_user(&value, addr, 1);
			write_left = copy_to_user(addr, &changed, 1);
			pagefault_enable();
			r->observed[side * 4 + i] = value;
			r->read_left[side * 4 + i] = read_left;
			r->write_left[side * 4 + i] = write_left;
			if (read_left || value != 0x41 + i ||
			    write_left != (side == 0 && i == 1 ? 0 : 1))
				r->error = 3;
			r->checks++;
		}
	}
	kthread_unuse_mm(r->parent);
	goto done;
out:
	kthread_unuse_mm(r->child);
done:
	complete(&r->done);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);
	return 0;
}

static void user4k_fault_uaccess_test(struct kunit *test)
{
	struct vm_area_struct *parent, *child_vma;
	struct mm_struct *child;
	struct cow_reader *reader;
	struct task_struct *task;
	unsigned int i, cpu;
	vm_fault_t fault;
	int ret;
	u64 tcr = read_sysreg(tcr_el1);

	if (!user4k_test_enabled || !system_supports_4kb_granule() ||
	    system_supports_cnp() || cpus_have_final_cap(ARM64_HAS_S1PIE) ||
	    cpus_have_final_cap(ARM64_HAS_S1POE) ||
	    (tcr & (TCR_EL1_DS | TCR_EL1_A1 | TCR_EL1_T0SZ_MASK)) !=
		   (TCR_EL1_A1 | TCR_T0SZ(48))) {
		kunit_skip(test, "requires opt-in and compatible 4K TTBR0 controls");
		return;
	}
	parent = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, parent);
	KUNIT_ASSERT_NOT_NULL(test, child);
	mmap_read_lock(parent->vm_mm);
	fault = handle_mm_fault(parent, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(parent->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	for (i = 0; i < 4; i++) {
		pte_t entry = test_fault_entry(test, parent->vm_mm, TEST_VA + i * SZ_4K);

		memset(phys_to_virt(pte_phys_mm(parent->vm_mm, entry)), 0x41 + i, SZ_4K);
	}
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	child_vma = test_vma_lookup(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, child_vma);
	reader = kunit_kzalloc(test, sizeof(*reader), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, reader);
	reader->parent = parent->vm_mm;
	reader->child = child;
	init_completion(&reader->ready);
	init_completion(&reader->copied);
	init_completion(&reader->done);
	task = kthread_create(test_cow_reader, reader, "user4k-cow-reader");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, task);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_stop_kthread, task), 0);
	cpu = cpumask_last(cpu_online_mask);
	kthread_bind(task, cpu);
	wake_up_process(task);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&reader->ready, 10 * HZ), 0UL);
	mmap_read_lock(child);
	fault = handle_mm_fault(child_vma, TEST_VA + SZ_4K,
				FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(child);
	complete(&reader->copied);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&reader->done, 10 * HZ), 0UL);
	if (reader->error)
		for (i = 0; i < 8; i++)
			kunit_info(test, "access %u value=%#x read_left=%lu write_left=%lu\n",
				   i, reader->observed[i], reader->read_left[i], reader->write_left[i]);
	KUNIT_EXPECT_EQ(test, reader->error, 0);
	KUNIT_EXPECT_EQ(test, reader->checks, 8U);
	{
		pte_t entry = test_fault_entry(test, child, TEST_VA + SZ_4K);

		KUNIT_EXPECT_EQ(test, *(u8 *)(phys_to_virt(pte_phys_mm(child, entry)) + 64), (u8)0x99);
		entry = test_fault_entry(test, parent->vm_mm, TEST_VA + SZ_4K);
		KUNIT_EXPECT_EQ(test, *(u8 *)(phys_to_virt(pte_phys_mm(parent->vm_mm, entry)) + 64), (u8)0x42);
	}
}

static void user4k_context_uaccess_test(struct kunit *test)
{
	struct test_mm *ctx[2];
	struct usercopy_worker *workers;
	unsigned int i;
	u64 tcr = read_sysreg(tcr_el1);

	if (!user4k_test_enabled) {
		kunit_skip(test, "requires arm64.user4k_test=1 in a disposable VM");
		return;
	}
	if (!system_supports_4kb_granule() || system_supports_cnp() ||
	    cpus_have_final_cap(ARM64_HAS_S1PIE) || cpus_have_final_cap(ARM64_HAS_S1POE) ||
	    (tcr & (TCR_EL1_DS | TCR_EL1_A1 | TCR_EL1_T0SZ_MASK)) !=
		   (TCR_EL1_A1 | TCR_T0SZ(48))) {
		kunit_skip(test, "requires 4K, no CnP/PIE/POE, DS=0, A1=1 and 48-bit TTBR0");
		return;
	}
	ctx[0] = test_usercopy_mm(test, false);
	KUNIT_ASSERT_NOT_NULL(test, ctx[0]);
	ctx[1] = test_usercopy_mm(test, true);
	KUNIT_ASSERT_NOT_NULL(test, ctx[1]);
	workers = kunit_kcalloc(test, 2, sizeof(*workers), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, workers);
	for (i = 0; i < 2; i++) {
		struct task_struct *task;

		workers[i].ctx[0] = ctx[0];
		workers[i].ctx[1] = ctx[1];
		workers[i].id = i;
		init_completion(&workers[i].done);
		task = kthread_run(test_usercopy_worker, &workers[i], "user4k-copy-%u", i);
		KUNIT_ASSERT_FALSE(test, IS_ERR(task));
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_stop_kthread, task), 0);
	}
	for (i = 0; i < 2; i++) {
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&workers[i].done, 10 * HZ), 0UL);
		KUNIT_EXPECT_EQ_MSG(test, workers[i].error, 0, "worker %u TCR=%#llx", i, workers[i].tcr);
		KUNIT_EXPECT_EQ(test, workers[i].checks, 64U);
		KUNIT_EXPECT_EQ(test, workers[i].idmaps, 2U);
	}
	kunit_info(test, "two migrating kthreads, 32 borrowed-mm switches, %s PAN\n",
		   system_uses_ttbr0_pan() ? "software" : "hardware");
}

struct zap_reader {
	struct mm_struct *mm;
	struct task_struct *task;
	struct completion ready;
	struct completion proceed;
	struct completion done;
	unsigned int cpu;
	unsigned int checks;
	bool zapped;
	int error;
};

static int test_zap_reader(void *arg)
{
	struct zap_reader *reader = arg;
	unsigned int phase, i;

	reader->error = set_cpus_allowed_ptr(current, cpumask_of(reader->cpu));
	if (reader->error) {
		complete(&reader->ready);
		goto out;
	}
	kthread_use_mm(reader->mm);
	for (phase = 0; phase < 2; phase++) {
		if (phase) {
			complete(&reader->ready);
			wait_for_completion(&reader->proceed);
			if (!reader->zapped)
				break;
		}
		for (i = 0; i < 4; i++) {
			u64 value = 0, expected = 0xa5a50000UL + i;
			unsigned long left;

			pagefault_disable();
			left = copy_from_user(&value,
				(void __user *)((unsigned long)TEST_VA + i * SZ_4K), sizeof(value));
			pagefault_enable();
			if (phase && i == 1) {
				if (!left)
					reader->error = -EUCLEAN;
			} else if (left || value != expected) {
				reader->error = -EFAULT;
			}
			reader->checks++;
		}
	}
	kthread_unuse_mm(reader->mm);
out:
	complete(&reader->done);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);
	return 0;
}

static void test_stop_zap_reader(void *arg)
{
	struct zap_reader *reader = arg;

	complete(&reader->proceed);
	kthread_stop(reader->task);
}

static void user4k_anon_remote_zap_test(struct kunit *test)
{
	struct anon_slot_fixture *f;
	struct zap_reader *readers;
	unsigned int i, cpu;
	u64 tcr = read_sysreg(tcr_el1);

	if (!user4k_test_enabled || num_online_cpus() < 2) {
		kunit_skip(test, "requires arm64.user4k_test=1 and two CPUs in a disposable VM");
		return;
	}
	if (!system_supports_4kb_granule() || system_supports_cnp() ||
	    cpus_have_final_cap(ARM64_HAS_S1PIE) || cpus_have_final_cap(ARM64_HAS_S1POE) ||
	    (tcr & (TCR_EL1_DS | TCR_EL1_A1 | TCR_EL1_T0SZ_MASK)) !=
		   (TCR_EL1_A1 | TCR_T0SZ(48))) {
		kunit_skip(test, "requires 4K, no CnP/PIE/POE, DS=0, A1=1 and 48-bit TTBR0");
		return;
	}
	f = anon_slot_fixture_create(test);
	KUNIT_ASSERT_NOT_NULL(test, f);
	for (i = 0; i < 4; i++) {
		struct mm_subpage *slot = mm_subpage_alloc_at(f->pool, i * SZ_4K);
		u64 value = 0xa5a50000UL + i;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
		memcpy(folio_address(mm_subpage_folio(slot)) + mm_subpage_offset(slot),
		       &value, sizeof(value));
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(f, i, slot), 0);
	}
	readers = kunit_kcalloc(test, 2, sizeof(*readers), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, readers);
	cpu = cpumask_first(cpu_online_mask);
	for (i = 0; i < 2; i++) {
		readers[i].mm = f->mm;
		readers[i].cpu = cpu;
		init_completion(&readers[i].ready);
		init_completion(&readers[i].proceed);
		init_completion(&readers[i].done);
		readers[i].task = kthread_run(test_zap_reader, &readers[i], "user4k-zap-%u", i);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, readers[i].task);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_stop_zap_reader,
							     &readers[i]), 0);
		cpu = cpumask_next(cpu, cpu_online_mask);
	}
	for (i = 0; i < 2; i++) {
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&readers[i].ready, 10 * HZ), 0UL);
		KUNIT_ASSERT_EQ(test, readers[i].error, 0);
	}
	/* Keep an explicit non-mapping reference for fixture cleanup. */
	mm_subpage_get(f->slots[1]);
	mmap_read_lock(f->mm);
	zap_vma_range(f->vma, TEST_VA + SZ_4K, SZ_4K);
	f->mapped[1] = false;
	mmap_read_unlock(f->mm);
	for (i = 0; i < 2; i++) {
		readers[i].zapped = true;
		complete(&readers[i].proceed);
	}
	for (i = 0; i < 2; i++) {
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&readers[i].done, 10 * HZ), 0UL);
		KUNIT_EXPECT_EQ(test, readers[i].error, 0);
		KUNIT_EXPECT_EQ(test, readers[i].checks, 8U);
	}
}

static u64 *test_leaf(struct test_tables *tables, unsigned long va)
{
	return &tables->level[3][test_index(va, tables->shift, 3)];
}

/* Explicitly 4K, without the native PAGE_SHIFT or a granule/level hint. */
static void test_invalidate_4k(unsigned long va, u64 ttbr1)
{
	u64 operand = (va >> USER4K_SHIFT) | (ttbr1 & GENMASK_ULL(63, 48));

	dsb(ishst);
	__tlbi(vae1, operand);
	dsb(ish);
	isb();
}

static void user4k_translation_test(struct kunit *test)
{
	struct test_tables small, native;
	struct test_result *result;
	unsigned long flags;
	void *data, *other;
	phys_addr_t data_pa, other_pa;
	u64 old_tcr, old_ttbr0, old_ttbr1, old_par, tcr4k, tcr16k;
	unsigned int round, slot;

	if (!user4k_test_enabled) {
		kunit_skip(test, "requires arm64.user4k_test=1 in a disposable VM");
		return;
	}
	if (!system_supports_4kb_granule() || system_supports_cnp() ||
	    cpus_have_final_cap(ARM64_HAS_S1PIE) ||
	    cpus_have_final_cap(ARM64_HAS_S1POE)) {
		kunit_skip(test, "requires 4K, no CnP, and direct PTE permissions");
		return;
	}

	KUNIT_ASSERT_EQ(test, PAGE_SHIFT, 14);
	KUNIT_ASSERT_EQ(test, test_alloc_tables(test, &small, USER4K_SHIFT), 0);
	KUNIT_ASSERT_EQ(test, test_alloc_tables(test, &native, PAGE_SHIFT), 0);
	data = test_alloc_page(test);
	other = test_alloc_page(test);
	KUNIT_ASSERT_NOT_NULL(test, data);
	KUNIT_ASSERT_NOT_NULL(test, other);
	data_pa = virt_to_phys(data);
	other_pa = virt_to_phys(other);
	result = kunit_kcalloc(test, TEST_ROUNDS, sizeof(*result), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, result);

	/* One native mapping and four independently described subpages. */
	*test_leaf(&native, TEST_VA) = other_pa | TEST_PROT;
	*test_leaf(&small, TEST_VA) = data_pa | TEST_PROT | PTE_RDONLY;
	*test_leaf(&small, TEST_VA + USER4K_SIZE) =
		(data_pa + USER4K_SIZE) | TEST_PROT;
	/* Slot 2 deliberately unmapped. Slot 3 aliases slot 0, writable. */
	*test_leaf(&small, TEST_VA + 3 * USER4K_SIZE) = data_pa | TEST_PROT;

	preempt_disable();
	flags = local_daif_save();
	old_tcr = read_sysreg(tcr_el1);
	old_ttbr0 = read_sysreg(ttbr0_el1);
	old_ttbr1 = read_sysreg(ttbr1_el1);
	old_par = read_sysreg_par();
	/* DS affects both address ranges: never toggle it under a live TTBR1. */
	if ((old_tcr & (TCR_EL1_DS | TCR_EL1_TG1_MASK | TCR_EL1_A1)) !=
	    (TCR_TG1_16K | TCR_EL1_A1)) {
		local_daif_restore(flags);
		preempt_enable();
		kunit_skip(test, "requires 16K TTBR1, DS=0, and A1=1");
		return;
	}
	tcr4k = (old_tcr & ~(TCR_EL1_TG0_MASK | TCR_EL1_T0SZ_MASK |
			      TCR_EL1_EPD0)) | TCR_TG0_4K | TCR_T0SZ(48);
	tcr16k = (tcr4k & ~TCR_EL1_TG0_MASK) | TCR_TG0_16K;

	/* No allocations, logging, KUnit assertions or user access in this window. */
	for (round = 0; round < TEST_ROUNDS; round++) {
		struct test_result *r = &result[round];

		test_switch_ttbr0(virt_to_phys(native.level[0]), tcr16k);
		for (slot = 0; slot < PROBE_SLOTS; slot++)
			r->native[slot] = test_translate(TEST_VA + slot * USER4K_SIZE, false);
		test_switch_ttbr0(virt_to_phys(small.level[0]), tcr4k);
		r->tcr = read_sysreg(tcr_el1);
		r->ttbr1 = read_sysreg(ttbr1_el1);
		for (slot = 0; slot < PROBE_SLOTS; slot++) {
			unsigned long va = TEST_VA + slot * USER4K_SIZE;

			r->read[slot] = test_translate(va, false);
			r->write[slot] = test_translate(va, true);
		}

		/* Break-before-make one 4K PTE and invalidate exactly that VA. */
		WRITE_ONCE(*test_leaf(&small, TEST_VA + USER4K_SIZE), 0);
		test_invalidate_4k(TEST_VA + USER4K_SIZE, old_ttbr1);
		WRITE_ONCE(*test_leaf(&small, TEST_VA + USER4K_SIZE),
			   (other_pa + USER4K_SIZE) | TEST_PROT | PTE_RDONLY);
		dsb(ishst);
		isb();
		for (slot = 0; slot < PROBE_SLOTS; slot++)
			r->replaced[slot] = test_translate(TEST_VA + slot * USER4K_SIZE, true);
		r->replaced_read = test_translate(TEST_VA + USER4K_SIZE, false);
		asm volatile("at s1e1r, %0" : : "r" (data) : "memory");
		isb();
		r->kernel = read_sysreg_par();

		test_switch_ttbr0(old_ttbr0, old_tcr);
		r->restored_tcr = read_sysreg(tcr_el1);
		r->restored_ttbr0 = read_sysreg(ttbr0_el1);
		/* Private tables are inactive and their translations have been flushed. */
		WRITE_ONCE(*test_leaf(&small, TEST_VA + USER4K_SIZE),
			   (data_pa + USER4K_SIZE) | TEST_PROT);
	}
	write_sysreg(old_par, par_el1);
	isb();
	local_daif_restore(flags);
	preempt_enable();

	for (round = 0; round < TEST_ROUNDS; round++) {
		struct test_result *r = &result[round];

		KUNIT_EXPECT_EQ(test, r->tcr, tcr4k);
		KUNIT_EXPECT_EQ(test, r->ttbr1, old_ttbr1);
		KUNIT_EXPECT_EQ(test, r->restored_tcr, old_tcr);
		KUNIT_EXPECT_EQ(test, r->restored_ttbr0, old_ttbr0);
		KUNIT_EXPECT_EQ(test, r->kernel & SYS_PAR_EL1_F, 0ULL);
		KUNIT_EXPECT_EQ(test, r->kernel & SYS_PAR_EL1_PA, data_pa);
		KUNIT_EXPECT_EQ(test, r->replaced_read & SYS_PAR_EL1_F, 0ULL);
		KUNIT_EXPECT_EQ(test, r->replaced_read & SYS_PAR_EL1_PA,
				other_pa + USER4K_SIZE);
		for (slot = 0; slot < PROBE_SLOTS; slot++) {
			KUNIT_EXPECT_EQ(test, r->native[slot] & SYS_PAR_EL1_F, 0ULL);
			KUNIT_EXPECT_EQ(test, r->native[slot] & SYS_PAR_EL1_PA,
					other_pa + slot * USER4K_SIZE);
			if (slot == 2) {
				KUNIT_EXPECT_EQ(test,
						FIELD_GET(SYS_PAR_EL1_FST, r->read[slot]), 7ULL);
				KUNIT_EXPECT_EQ(test, r->read[slot] & SYS_PAR_EL1_F, 1ULL);
				KUNIT_EXPECT_EQ(test, r->write[slot] & SYS_PAR_EL1_F, 1ULL);
				KUNIT_EXPECT_EQ(test, r->replaced[slot] & SYS_PAR_EL1_F, 1ULL);
				continue;
			}
			KUNIT_EXPECT_EQ(test, r->read[slot] & SYS_PAR_EL1_F, 0ULL);
			KUNIT_EXPECT_EQ(test, r->read[slot] & SYS_PAR_EL1_PA,
					data_pa + (slot == 1 ? USER4K_SIZE : 0));
			KUNIT_EXPECT_EQ(test, r->write[slot] & SYS_PAR_EL1_F,
					slot == 0 ? 1ULL : 0ULL);
			KUNIT_EXPECT_EQ(test, r->replaced[slot] & SYS_PAR_EL1_F,
					slot == 3 ? 0ULL : 1ULL);
		}
	}
	kunit_info(test, "16K kernel, 4K EL0 translation/permissions; %u switch cycles\n",
		   TEST_ROUNDS);
}

static struct folio *test_fault_folio(struct mm_struct *mm, unsigned long address)
{
	spinlock_t *ptl;
	pte_t *pte = test_mm_lookup(mm, address, &ptl);
	struct folio *folio = NULL;

	if (pte) {
		pte_t entry = ptep_get(pte);

		if (pte_present(entry) && !pte_special(entry)) {
			folio = page_folio(pfn_to_page(pte_pfn(entry)));
			folio_get(folio);
		}
		pte_unmap_unlock(pte, ptl);
	}
	return folio;
}

static bool test_reclaim_swapcache(swp_entry_t entry)
{
	unsigned int tries;

	for (tries = 0; tries < 100; tries++) {
		struct folio *folio = swap_cache_get_folio(entry);
		LIST_HEAD(folios);

		if (!folio)
			return true;
		folio_wait_writeback(folio);
		lru_add_drain_all();
		if (folio_isolate_lru(folio))
			list_add(&folio->lru, &folios);
		folio_put(folio);
		/* Only the swap cache and reclaim isolation may retain this folio. */
		reclaim_pages(&folios);
		schedule_timeout_uninterruptible(1);
	}
	return false;
}

static void test_swap_tags_set(struct folio *folio)
{
#ifdef CONFIG_ARM64_MTE
	unsigned int offset;

	if (!system_supports_mte())
		return;
	if (try_page_mte_tagging(&folio->page))
		mte_clear_page_tags(folio_address(folio));
	for (offset = 0; offset < PAGE_SIZE; offset += MTE_GRANULE_SIZE) {
		unsigned long tag = (1UL + offset / SZ_4K) << MTE_TAG_SHIFT;

		asm volatile(".arch_extension memtag\n\tstg %0, [%1]"
			     : : "r" (tag), "r" (folio_address(folio) + offset) : "memory");
	}
	set_page_mte_tagged(&folio->page);
#endif
}

static bool test_swap_tags_match_at(struct folio *folio, unsigned int start,
				   unsigned int slot)
{
#ifdef CONFIG_ARM64_MTE
	unsigned int offset;

	if (!system_supports_mte())
		return true;
	if (!page_mte_tagged(&folio->page))
		return false;
	for (offset = start; offset < start + SZ_4K; offset += MTE_GRANULE_SIZE) {
		unsigned long tag = 0;

		asm volatile(".arch_extension memtag\n\tldg %0, [%1]"
			     : "+r" (tag) : "r" (folio_address(folio) + offset) : "memory");
		if (((tag >> MTE_TAG_SHIFT) & 0xf) != ((slot + 1) & 0xf))
			return false;
	}
#endif
	return true;
}

static bool test_swap_tags_match(struct folio *folio, unsigned int slot)
{
	return test_swap_tags_match_at(folio, slot * SZ_4K, slot);
}

struct fragment_test_pins {
	struct user_page_fragment fragments[2 * TEST_SLOTS];
	unsigned long count;
};

static void fragment_test_unpin(void *arg)
{
	struct fragment_test_pins *pins = arg;

	release_user_fragments(pins->fragments, pins->count, true);
	pins->count = 0;
}

static long fragment_test_pin(struct fragment_test_pins *pins, struct mm_struct *mm,
		unsigned long start, unsigned long length, unsigned int flags,
		unsigned long capacity)
{
	long ret;

	fragment_test_unpin(pins);
	ret = pin_user_fragments_remote(mm, start, length, flags, pins->fragments, capacity);
	pins->count = ret > 0 ? ret : 0;
	return ret;
}

static void user4k_fragment_pin_test(struct kunit *test)
{
	unsigned int native;

	for (native = 0; native < 2; native++) {
		struct fragment_test_pins *pins;
		struct vm_area_struct *vma;
		struct mm_struct *mm, *child;
		unsigned long size, total = 0;
		unsigned int i, nr;
		long ret;

		if (native) {
			mm = mm_alloc();
			KUNIT_ASSERT_NOT_NULL(test, mm);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
			vma = test_fault_vma_in_mm(mm, TEST_VA, PAGE_SIZE);
			KUNIT_ASSERT_NOT_NULL(test, vma);
		} else {
			vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
			KUNIT_ASSERT_NOT_NULL(test, vma);
			mm = vma->vm_mm;
		}
		size = mm_page_size(mm);
		nr = PAGE_SIZE / size;
		pins = kunit_kzalloc(test, sizeof(*pins), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, pins);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_test_unpin, pins), 0);
		ret = fragment_test_pin(pins, mm, TEST_VA, 1, FOLL_WRITE | FOLL_NOFAULT, ARRAY_SIZE(pins->fragments));
		KUNIT_EXPECT_EQ(test, ret, -EFAULT);
		ret = fragment_test_pin(pins, mm, TEST_VA + 123, PAGE_SIZE - 200, FOLL_WRITE, ARRAY_SIZE(pins->fragments));
		KUNIT_ASSERT_EQ(test, ret, (long)nr);
		for (i = 0; i < nr; i++) {
			struct user_page_fragment *f = &pins->fragments[i];
			unsigned int offset = i ? i * size : 123;
			unsigned int length = size - (!i ? 123 : 0) - (i == nr - 1 ? 77 : 0);

			KUNIT_EXPECT_EQ(test, f->offset, (unsigned long)offset);
			KUNIT_EXPECT_EQ(test, f->length, length);
			KUNIT_EXPECT_EQ(test, !!f->subpage, !native);
			memset(folio_address(f->folio) + f->offset, 0x71 + i, f->length);
			total += f->length;
		}
		KUNIT_EXPECT_EQ(test, total, PAGE_SIZE - 200);
		fragment_test_unpin(pins);
		child = native ? mm_alloc() : test_vma_mm(test);
		KUNIT_ASSERT_NOT_NULL(test, child);
		if (native)
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, child), 0);
		uprobe_start_dup_mmap();
		ret = dup_mmap(child, mm);
		uprobe_end_dup_mmap();
		KUNIT_ASSERT_EQ(test, ret, 0L);
		/* Ordinary references retain shared COW backing, as native FOLL_GET does. */
		ret = get_user_fragments_remote(child, TEST_VA + 123, PAGE_SIZE - 200,
					       0, pins->fragments, ARRAY_SIZE(pins->fragments));
		pins->count = ret > 0 ? ret : 0;
		KUNIT_ASSERT_EQ(test, ret, (long)nr);
		for (i = 0; i < nr; i++) {
			struct user_page_fragment *f = &pins->fragments[i];
			pte_t parent_pte = test_fault_entry(test, mm, TEST_VA + i * size);
			pte_t child_pte = test_fault_entry(test, child, TEST_VA + i * size);

			KUNIT_EXPECT_FALSE(test, f->pinned);
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, parent_pte), pte_phys_mm(child, child_pte));
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(f->folio) + f->offset,
							    0x71 + i, f->length), NULL);
		}
		fragment_test_unpin(pins);
		/* Read pins must unshare COW mappings without enabling PTE writes. */
		ret = fragment_test_pin(pins, child, TEST_VA + 123, PAGE_SIZE - 200, FOLL_NOFAULT, ARRAY_SIZE(pins->fragments));
		KUNIT_EXPECT_EQ(test, ret, -EFAULT);
		ret = fragment_test_pin(pins, child, TEST_VA + 123, PAGE_SIZE - 200, 0, ARRAY_SIZE(pins->fragments));
		KUNIT_ASSERT_EQ(test, ret, (long)nr);
		for (i = 0; i < nr; i++) {
			struct user_page_fragment *f = &pins->fragments[i];
			pte_t pte = test_fault_entry(test, child, TEST_VA + i * size);

			KUNIT_EXPECT_FALSE(test, pte_write(pte));
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(f->folio) + f->offset,
							    0x71 + i, f->length), NULL);
			if (!native) {
				pte_t parent_pte = test_fault_entry(test, mm, TEST_VA + i * size);

				KUNIT_EXPECT_TRUE(test, mm_subpage_is_exclusive(f->subpage));
				KUNIT_EXPECT_NE(test, pte_phys_mm(child, pte),
						pte_phys_mm(mm, parent_pte));
				if (i)
					KUNIT_EXPECT_NE(test, mm_subpage_phys(f->subpage),
							mm_subpage_phys(pins->fragments[0].subpage));
			}
		}
		fragment_test_unpin(pins);
		/* Return a valid partial result when the byte range reaches an unmapped VMA. */
		ret = fragment_test_pin(pins, mm, TEST_VA, PAGE_SIZE + size, FOLL_WRITE, ARRAY_SIZE(pins->fragments));
		KUNIT_EXPECT_EQ(test, ret, (long)nr);
		fragment_test_unpin(pins);
		ret = fragment_test_pin(pins, mm, TEST_VA, PAGE_SIZE, FOLL_WRITE, 1);
		KUNIT_EXPECT_EQ(test, ret, 1L);
		KUNIT_EXPECT_EQ(test, pins->fragments[0].length, (unsigned int)size);
		fragment_test_unpin(pins);
		ret = fragment_test_pin(pins, mm, TEST_VA, size, FOLL_LONGTERM, ARRAY_SIZE(pins->fragments));
		KUNIT_EXPECT_EQ(test, ret, 1L);
		ret = fragment_test_pin(pins, mm, ULONG_MAX - 15, ULONG_MAX, 0, ARRAY_SIZE(pins->fragments));
		KUNIT_EXPECT_EQ(test, ret, -EFAULT);
		kunit_release_action(test, user4k_vma_mm_free, child);
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

#ifdef CONFIG_BLOCK
struct bio_fragment_test {
	struct bio *bio;
	struct bio *clone;
	struct user_page_fragment held[TEST_SLOTS];
	unsigned int count;
};

static void bio_fragment_test_free(void *arg)
{
	struct bio_fragment_test *state = arg;

	if (state->clone)
		bio_put(state->clone);
	if (state->bio) {
		bio_release_pages(state->bio, false);
		bio_put(state->bio);
	}
	release_user_fragments(state->held, state->count, false);
}

static void user4k_bio_fragments_test(struct kunit *test)
{
	struct vm_area_struct *vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	struct bio_fragment_test *state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	struct iov_iter iter;
	unsigned int i, bytes = round_down(PAGE_SIZE - 200, 512);
	long nr;
	int ret;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm bio fixture requires hardware opt-in");
		return;
	}
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_NOT_NULL(test, state);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, bio_fragment_test_free, state), 0);
	/* Populate real writable anonymous mappings and retain independent GETs. */
	nr = get_user_fragments_remote(vma->vm_mm, TEST_VA, PAGE_SIZE, FOLL_WRITE,
				       state->held, ARRAY_SIZE(state->held));
	state->count = nr > 0 ? nr : 0;
	KUNIT_ASSERT_EQ(test, nr, (long)TEST_SLOTS);
	state->bio = bio_alloc(NULL, 8, REQ_OP_READ, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->bio);
	iov_iter_ubuf(&iter, ITER_DEST, (void __user *)(TEST_VA + 123), PAGE_SIZE - 200);
	kthread_use_mm(vma->vm_mm);
	ret = bio_iov_vecs_to_alloc(&iter, BIO_MAX_VECS);
	kthread_unuse_mm(vma->vm_mm);
	KUNIT_EXPECT_EQ(test, ret, (int)TEST_SLOTS);
	kthread_use_mm(vma->vm_mm);
	ret = bio_iov_iter_get_pages(state->bio, &iter, 0, 511);
	kthread_unuse_mm(vma->vm_mm);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, state->bio->bi_iter.bi_size, bytes);
	KUNIT_EXPECT_EQ(test, iov_iter_count(&iter), PAGE_SIZE - 200 - bytes);
	KUNIT_EXPECT_EQ(test, state->bio->bi_nr_user_fragments, (int)TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, state->bio->bi_vcnt, 1);
	KUNIT_EXPECT_EQ(test, state->bio->bi_io_vec[0].bv_offset, 123U);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, mm_subpage_pincount(state->held[i].subpage), 1U);
	state->clone = bio_alloc_clone(NULL, state->bio, GFP_KERNEL, &fs_bio_set);
	KUNIT_ASSERT_NOT_NULL(test, state->clone);
	KUNIT_EXPECT_PTR_EQ(test, state->clone->bi_user_fragments, NULL);
	KUNIT_EXPECT_PTR_EQ(test, state->clone->bi_io_vec, state->bio->bi_io_vec);
	/* Completion ownership must outlive all originating VMAs. */
	kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, mm_subpage_pincount(state->held[i].subpage), 1U);
	bio_put(state->clone);
	state->clone = NULL;
	bio_release_pages(state->bio, false);
	KUNIT_EXPECT_PTR_EQ(test, state->bio->bi_user_fragments, NULL);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, mm_subpage_pincount(state->held[i].subpage), 0U);
}

static void user4k_bio_trim_test(struct kunit *test)
{
	struct vm_area_struct *vma = test_fault_vma(test, TEST_VA, SZ_4K);
	struct bio_fragment_test *state = kunit_kzalloc(test, sizeof(*state), GFP_KERNEL);
	struct iov_iter iter;
	unsigned int i;
	long nr;
	int ret, refs;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm bio fixture requires hardware opt-in");
		return;
	}
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_NOT_NULL(test, state);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, bio_fragment_test_free, state), 0);
	nr = get_user_fragments_remote(vma->vm_mm, TEST_VA, SZ_4K, FOLL_WRITE, state->held, 1);
	state->count = nr > 0 ? nr : 0;
	KUNIT_ASSERT_EQ(test, nr, 1L);
	refs = folio_ref_count(state->held[0].folio);
	state->bio = bio_alloc(NULL, 4, REQ_OP_READ, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, state->bio);
	for (i = 0; i < 1024; i++) {
		iov_iter_ubuf(&iter, ITER_DEST, (void __user *)(TEST_VA + SZ_4K - 256), 512);
		kthread_use_mm(vma->vm_mm);
		ret = bio_iov_iter_get_pages(state->bio, &iter, 0, 511);
		kthread_unuse_mm(vma->vm_mm);
		KUNIT_ASSERT_EQ(test, ret, -EFAULT);
		KUNIT_EXPECT_EQ(test, iov_iter_count(&iter), 512UL);
		KUNIT_EXPECT_EQ(test, state->bio->bi_vcnt, 0);
		KUNIT_EXPECT_PTR_EQ(test, state->bio->bi_user_fragments, NULL);
		KUNIT_EXPECT_EQ(test, mm_subpage_pincount(state->held[0].subpage), 0U);
		KUNIT_EXPECT_EQ(test, folio_ref_count(state->held[0].folio), refs);
		bio_reset(state->bio, NULL, REQ_OP_READ);
	}
}
#endif

static void user4k_fragment_longterm_test(struct kunit *test)
{
	struct vm_area_struct *vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	struct fragment_test_pins *pins = kunit_kzalloc(test, sizeof(*pins), GFP_KERNEL);
	phys_addr_t before;
	bool movable;
	unsigned int i;
	long ret;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_NOT_NULL(test, pins);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_test_unpin, pins), 0);
	ret = fragment_test_pin(pins, vma->vm_mm, TEST_VA, PAGE_SIZE, FOLL_WRITE, ARRAY_SIZE(pins->fragments));
	KUNIT_ASSERT_EQ(test, ret, (long)TEST_SLOTS);
	before = PFN_PHYS(folio_pfn(pins->fragments[0].folio));
	movable = !folio_is_longterm_pinnable(pins->fragments[0].folio);
	for (i = 0; i < TEST_SLOTS; i++)
		memset(folio_address(pins->fragments[i].folio) + pins->fragments[i].offset,
		       0x91 + i, pins->fragments[i].length);
	fragment_test_unpin(pins);
	ret = fragment_test_pin(pins, vma->vm_mm, TEST_VA, PAGE_SIZE,
				FOLL_WRITE | FOLL_LONGTERM, ARRAY_SIZE(pins->fragments));
	KUNIT_ASSERT_EQ(test, ret, (long)TEST_SLOTS);
	for (i = 0; i < TEST_SLOTS; i++) {
		struct user_page_fragment *f = &pins->fragments[i];

		KUNIT_EXPECT_TRUE(test, folio_is_longterm_pinnable(f->folio));
		KUNIT_EXPECT_EQ(test, mm_subpage_pincount(f->subpage), 1U);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(f->folio) + f->offset,
						   0x91 + i, f->length), NULL);
	}
	if (movable)
		KUNIT_EXPECT_NE(test, PFN_PHYS(folio_pfn(pins->fragments[0].folio)), before);
	kunit_info(test, "long-term source movable=%u migrated=%u\n", movable,
		   PFN_PHYS(folio_pfn(pins->fragments[0].folio)) != before);
	fragment_test_unpin(pins);
	/* A long-term flag must never silently create an ordinary GET. */
	ret = get_user_fragments_remote(vma->vm_mm, TEST_VA, SZ_4K, FOLL_LONGTERM,
				       pins->fragments, 1);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static void user4k_file_fragments_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, PAGE_SIZE, true);
	struct vm_area_struct *shared, *private;
	struct fragment_test_pins *pins;
	phys_addr_t shared_phys;
	long ret;
	unsigned int i;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	shared = test_file_vma(test, file, false, true, TEST_VA + SZ_4K, 3 * SZ_4K, SZ_4K);
	private = test_file_vma(test, file, false, false, TEST_VA + SZ_4K, 3 * SZ_4K, SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, shared);
	KUNIT_ASSERT_NOT_NULL(test, private);
	pins = kunit_kzalloc(test, sizeof(*pins), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pins);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_test_unpin, pins), 0);
	ret = fragment_test_pin(pins, shared->vm_mm, shared->vm_start + 17,
			       3 * SZ_4K - 33, FOLL_WRITE, 8);
	KUNIT_ASSERT_EQ(test, ret, 3L);
	shared_phys = PFN_PHYS(folio_pfn(pins->fragments[0].folio));
	for (i = 0; i < 3; i++) {
		struct user_page_fragment *f = &pins->fragments[i];

		KUNIT_EXPECT_PTR_EQ(test, f->subpage, NULL);
		KUNIT_EXPECT_EQ(test, f->offset, (unsigned long)((i + 1) * SZ_4K + (!i ? 17 : 0)));
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(f->folio) + f->offset,
						   0x32 + i, f->length), NULL);
	}
	/* File pins stay live across PTE removal and mm exit. */
	kunit_release_action(test, user4k_vma_mm_free, shared->vm_mm);
	for (i = 0; i < 3; i++)
		KUNIT_EXPECT_TRUE(test, folio_maybe_dma_pinned(pins->fragments[i].folio));
	fragment_test_unpin(pins);
	ret = get_user_fragments_remote(private->vm_mm, private->vm_start, 3 * SZ_4K,
					0, pins->fragments, 8);
	pins->count = ret > 0 ? ret : 0;
	KUNIT_ASSERT_EQ(test, ret, 3L);
	for (i = 0; i < 3; i++) {
		KUNIT_EXPECT_PTR_EQ(test, pins->fragments[i].subpage, NULL);
		KUNIT_EXPECT_EQ(test, PFN_PHYS(folio_pfn(pins->fragments[i].folio)), shared_phys);
	}
	fragment_test_unpin(pins);
	ret = fragment_test_pin(pins, private->vm_mm, private->vm_start, 3 * SZ_4K, 0, 8);
	KUNIT_ASSERT_EQ(test, ret, 3L);
	for (i = 0; i < 3; i++) {
		struct user_page_fragment *f = &pins->fragments[i];

		KUNIT_EXPECT_NOT_NULL(test, f->subpage);
		KUNIT_EXPECT_NE(test, PFN_PHYS(folio_pfn(f->folio)), shared_phys);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(f->folio) + f->offset,
						   0x32 + i, f->length), NULL);
		KUNIT_EXPECT_FALSE(test, pte_write(test_fault_entry(test, private->vm_mm,
								  private->vm_start + i * SZ_4K)));
	}
	fragment_test_unpin(pins);
}

/* Exercise the same implementation used by the arm64 syscall entry. */
asmlinkage long __arm64_sys_mprotect(const struct pt_regs *regs);

static long test_mprotect(unsigned long start, unsigned long len, unsigned long prot)
{
	struct pt_regs regs = { .regs = { start, len, prot }, .orig_x0 = start };

	return __arm64_sys_mprotect(&regs);
}

static unsigned long test_copy_word(unsigned long addr, u64 *value, bool write)
{
	unsigned long left;

	pagefault_disable();
	if (write)
		left = copy_to_user((void __user *)addr, value, sizeof(*value));
	else
		left = copy_from_user(value, (void __user *)addr, sizeof(*value));
	pagefault_enable();
	return left;
}

static void user4k_mprotect_test(struct kunit *test)
{
	const unsigned long starts[] = {
		TEST_VA, SZ_2M - SZ_4K, SZ_1G - SZ_4K, BIT(39) - SZ_4K, TEST_VA,
	};
	unsigned int shape;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm permission fixture requires hardware opt-in");
		return;
	}
	for (shape = 0; shape < ARRAY_SIZE(starts); shape++) {
		struct file *file = shape == 4 ? test_cache_file(test, PAGE_SIZE, true) : NULL;
		struct mm_struct *mm = test_vma_mm(test);
		unsigned long start = starts[shape], mapped;
		unsigned long flags = MAP_FIXED_NOREPLACE | (file ? MAP_SHARED : MAP_PRIVATE | MAP_ANONYMOUS);
		phys_addr_t before[4] = { };
		long protect[4] = { };
		unsigned long initial = 0, final = 0, read_ro = 0, write_ro = 0, read_none = 0;
		u64 value, ro_value = 0;
		unsigned int i;
		vm_fault_t fault = 0;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		if (shape == 4)
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		kthread_use_mm(mm);
		mapped = vm_mmap(file, start, PAGE_SIZE, PROT_READ | PROT_WRITE, flags, 0);
		if (IS_ERR_VALUE(mapped))
			goto detach;
		for (i = 0; i < 4; i++) {
			struct vm_area_struct *vma = test_vma_lookup(mm, start + i * SZ_4K);

			fault |= test_file_fault(vma, start + i * SZ_4K, true);
			if (fault & VM_FAULT_ERROR)
				goto detach;
			before[i] = pte_phys_mm(mm, test_fault_entry(test, mm, start + i * SZ_4K));
			value = 0x88110000 + i;
			initial |= test_copy_word(start + i * SZ_4K, &value, true);
		}
		/* Keep the mm attached across permission changes and cached accesses. */
		protect[0] = test_mprotect(start + SZ_4K, SZ_4K, PROT_READ);
		value = 0xaa;
		write_ro = test_copy_word(start + SZ_4K, &value, true);
		final |= test_copy_word(start, &value, true);
		final |= test_copy_word(start + 3 * SZ_4K, &value, true);
		protect[1] = test_mprotect(start + 2 * SZ_4K, SZ_4K, PROT_NONE);
		read_none = test_copy_word(start + 2 * SZ_4K, &value, false);
		read_ro = test_copy_word(start + SZ_4K, &ro_value, false);
		protect[2] = test_mprotect(start, PAGE_SIZE, PROT_READ);
		protect[3] = test_mprotect(start, PAGE_SIZE, PROT_READ | PROT_WRITE);
		for (i = 0; i < 4; i++) {
			value = 0x99220000 + i;
			final |= test_copy_word(start + i * SZ_4K, &value, true);
		}
detach:
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, mapped, start);
		KUNIT_ASSERT_EQ(test, fault & VM_FAULT_ERROR, 0U);
		KUNIT_EXPECT_EQ(test, initial, 0UL);
		KUNIT_EXPECT_EQ(test, final, 0UL);
		KUNIT_EXPECT_EQ(test, write_ro, (unsigned long)sizeof(u64));
		KUNIT_EXPECT_EQ(test, read_none, (unsigned long)sizeof(u64));
		KUNIT_EXPECT_EQ(test, read_ro, 0UL);
		KUNIT_EXPECT_EQ(test, ro_value, 0x88110001ULL);
		for (i = 0; i < 4; i++) {
			pte_t pte = test_fault_entry(test, mm, start + i * SZ_4K);

			KUNIT_EXPECT_EQ(test, protect[i], 0L);
			KUNIT_EXPECT_TRUE(test, pte_present(pte));
			KUNIT_EXPECT_TRUE(test, pte_write(pte));
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte), before[i]);
		}
		KUNIT_EXPECT_EQ(test, mm->total_vm, (unsigned long)TEST_SLOTS);
		KUNIT_EXPECT_EQ(test, mm->map_count, 1);
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

#ifdef CONFIG_NUMA_BALANCING
static void user_subpage_numa_cow_batch_test(struct kunit *test)
{
	unsigned int shift;

	for (shift = 12; shift < PAGE_SHIFT; shift += 2) {
		unsigned long size = 1UL << shift, start = TEST_VA;
		struct vm_area_struct *vma;
		struct mm_struct *child = test_vma_mm_granule(test, shift);
		struct mm_struct *mm = test_vma_mm_granule(test, shift);
		phys_addr_t before;
		unsigned long marked, mapped;
		long ret;
		unsigned int i;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		KUNIT_ASSERT_NOT_NULL(test, child);
		/* Use mmap accounting because the child will partially unmap. */
		kthread_use_mm(mm);
		mapped = vm_mmap(NULL, start, 4 * size, PROT_READ | PROT_WRITE,
				 MAP_FIXED_NOREPLACE | MAP_PRIVATE | MAP_ANONYMOUS, 0);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, mapped, start);
		vma = test_vma_lookup(mm, start);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		for (i = 0; i < 4; i++)
			KUNIT_ASSERT_EQ(test, test_file_fault(vma, start + i * size, true),
					(vm_fault_t)0);
		before = pte_phys_mm(mm, test_fault_entry(test, mm, start + size));
		uprobe_start_dup_mmap();
		ret = dup_mmap(child, mm);
		uprobe_end_dup_mmap();
		KUNIT_ASSERT_EQ(test, ret, 0L);
		kthread_use_mm(child);
		ret = vm_munmap(start + size, size);
		kthread_unuse_mm(child);
		KUNIT_ASSERT_EQ(test, ret, 0L);
		/* Reuse the now-private slot without replacing its native owner. */
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, start + size, true), (vm_fault_t)0);
		KUNIT_ASSERT_EQ(test, pte_phys_mm(mm,
			test_fault_entry(test, mm, start + size)), before);
		kthread_use_mm(mm);
		ret = test_mprotect(start, 4 * size, PROT_READ);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0L);
		vma = test_vma_lookup(mm, start);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		/* Permit local-node hinting, which normally skips a single user. */
		mmget(mm);
		mmap_read_lock(mm);
		marked = change_prot_numa(vma, start, start + 4 * size);
		mmap_read_unlock(mm);
		mmput(mm);
		KUNIT_EXPECT_EQ(test, marked, 1UL);
		for (i = 0; i < 4; i++) {
			pte_t pte = test_fault_entry(test, mm, start + i * size);

			KUNIT_EXPECT_EQ(test, !!pte_protnone(pte), i == 1);
			KUNIT_EXPECT_FALSE(test, pte_write(pte));
		}
		kunit_release_action(test, user4k_vma_mm_free, child);
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}
#endif

static void user4k_mprotect_cow_batch_test(struct kunit *test)
{
	struct mm_struct *mm, *child;
	struct vm_area_struct *vma;
	struct fragment_test_pins *pins;
	phys_addr_t before[4];
	unsigned int i;
	unsigned long mapped;
	long ret[2];
	int copied;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm permission fixture requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	KUNIT_ASSERT_NOT_NULL(test, child);
	pins = kunit_kzalloc(test, sizeof(*pins), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pins);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_test_unpin, pins), 0);
	kthread_use_mm(mm);
	mapped = vm_mmap(NULL, TEST_VA, PAGE_SIZE, PROT_READ | PROT_WRITE,
			 MAP_FIXED_NOREPLACE | MAP_PRIVATE | MAP_ANONYMOUS, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, mapped, TEST_VA);
	vma = test_vma_lookup(mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
	KUNIT_ASSERT_EQ(test, fragment_test_pin(pins, mm, TEST_VA + SZ_4K, SZ_4K, FOLL_WRITE, 8), 1L);
	for (i = 0; i < 4; i++)
		before[i] = pte_phys_mm(mm, test_fault_entry(test, mm, TEST_VA + i * SZ_4K));
	uprobe_start_dup_mmap();
	copied = dup_mmap(child, mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, copied, 0);
	kthread_use_mm(mm);
	ret[0] = test_mprotect(TEST_VA, PAGE_SIZE, PROT_READ);
	ret[1] = test_mprotect(TEST_VA, PAGE_SIZE, PROT_READ | PROT_WRITE);
	kthread_unuse_mm(mm);
	KUNIT_EXPECT_EQ(test, ret[0], 0L);
	KUNIT_EXPECT_EQ(test, ret[1], 0L);
	for (i = 0; i < 4; i++) {
		pte_t pte = test_fault_entry(test, mm, TEST_VA + i * SZ_4K);

		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte), before[i]);
		/* One native folio contains both exclusive and shared COW slots. */
		KUNIT_EXPECT_EQ(test, pte_write(pte), i == 1);
	}
	fragment_test_unpin(pins);
}

static void user4k_mprotect_accounting_test(struct kunit *test)
{
	struct mm_struct *mm = test_vma_mm(test);
	unsigned long mapped, committed[4], data[4];
	s64 global[4], before;
	long ret[4];
	unsigned int i;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm permission fixture requires hardware opt-in");
		return;
	}
	KUNIT_ASSERT_NOT_NULL(test, mm);
	before = test_commit_baseline(test);
	kthread_use_mm(mm);
	mapped = vm_mmap(NULL, TEST_VA, PAGE_SIZE, PROT_READ | PROT_WRITE,
			 MAP_FIXED_NOREPLACE | MAP_PRIVATE | MAP_ANONYMOUS, 0);
	if (!IS_ERR_VALUE(mapped)) {
		ret[0] = test_mprotect(TEST_VA + SZ_4K, SZ_4K, PROT_READ);
		committed[0] = mm->committed_user_pages;
		data[0] = mm->data_vm;
		global[0] = percpu_counter_sum(&vm_committed_as) - before;
		ret[1] = test_mprotect(TEST_VA, PAGE_SIZE, PROT_READ);
		committed[1] = mm->committed_user_pages;
		data[1] = mm->data_vm;
		global[1] = percpu_counter_sum(&vm_committed_as) - before;
		ret[2] = test_mprotect(TEST_VA + 2 * SZ_4K, SZ_4K, PROT_READ | PROT_WRITE);
		committed[2] = mm->committed_user_pages;
		data[2] = mm->data_vm;
		global[2] = percpu_counter_sum(&vm_committed_as) - before;
		ret[3] = vm_munmap(TEST_VA, PAGE_SIZE);
		committed[3] = mm->committed_user_pages;
		data[3] = mm->data_vm;
		global[3] = percpu_counter_sum(&vm_committed_as) - before;
	}
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, mapped, TEST_VA);
	for (i = 0; i < 4; i++) {
		unsigned long expected = i == 0 ? TEST_SLOTS - 1 : i == 2 ? 1 : 0;

		KUNIT_EXPECT_EQ(test, ret[i], 0L);
		KUNIT_EXPECT_EQ(test, committed[i], expected);
		KUNIT_EXPECT_EQ(test, data[i], expected);
		KUNIT_EXPECT_EQ(test, global[i], expected ? 1LL : 0LL);
	}
	KUNIT_EXPECT_EQ(test, mm->map_count, 0);
}

static void user4k_fragment_permissions_test(struct kunit *test)
{
	unsigned long start = TEST_VA + SZ_4K;
	struct vm_area_struct *vma = test_fault_vma(test, start, SZ_4K);
	struct fragment_test_pins *pins;
	struct user_page_fragment *fragment;
	struct mm_struct *mm;
	spinlock_t *ptl;
	pte_t *ptep;
	long ret;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	mm = vma->vm_mm;
	pins = kunit_kzalloc(test, sizeof(*pins), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pins);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_test_unpin, pins), 0);
	fragment = &pins->fragments[0];
	{
		struct page *page = NULL;
		int locked = 1;

		mmap_read_lock(mm);
		ret = pin_user_pages_remote(mm, start, 1, 0, &page, &locked);
		if (locked)
			mmap_read_unlock(mm);
		if (ret == 1)
			unpin_user_page(page);
		KUNIT_EXPECT_EQ(test, ret, -EOPNOTSUPP);
	}
	ret = fragment_test_pin(pins, mm, start + 17, 31, 0, 8);
	KUNIT_ASSERT_EQ(test, ret, 1L);
	KUNIT_EXPECT_TRUE(test, is_zero_folio(fragment->folio));
	KUNIT_EXPECT_PTR_EQ(test, fragment->subpage, NULL);
	/* Zero-page PTEs alias the first physical 4K despite this VA's slot 1. */
	KUNIT_EXPECT_EQ(test, fragment->offset, 17UL);
	KUNIT_EXPECT_EQ(test, fragment->length, 31U);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(fragment->folio) + fragment->offset,
					    0, fragment->length), NULL);
	ret = fragment_test_pin(pins, mm, start + 17, 31, FOLL_WRITE, 8);
	KUNIT_ASSERT_EQ(test, ret, 1L);
	KUNIT_EXPECT_NOT_NULL(test, fragment->subpage);
	/* COW backing may use a different quarter from the virtual address. */
	KUNIT_EXPECT_EQ(test, page_to_phys(&fragment->folio->page) + fragment->offset,
			pte_phys_mm(mm, test_fault_entry(test, mm, start)) + 17);
	memset(folio_address(fragment->folio) + fragment->offset, 0xa4, fragment->length);
	fragment_test_unpin(pins);
	mmap_write_lock(mm);
	vm_flags_clear(vma, VM_WRITE);
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	ptep = test_mm_lookup(mm, start, &ptl);
	if (ptep) {
		ptep_set_wrprotect(mm, start, ptep);
		pte_unmap_unlock(ptep, ptl);
	}
	flush_tlb_range(vma, start, start + SZ_4K);
	mmap_write_unlock(mm);
	KUNIT_ASSERT_NOT_NULL(test, ptep);
	ret = fragment_test_pin(pins, mm, start + 17, 31, FOLL_WRITE, 8);
	KUNIT_EXPECT_EQ(test, ret, -EFAULT);
	ret = fragment_test_pin(pins, mm, start + 17, 31, FOLL_WRITE | FOLL_FORCE, 8);
	KUNIT_ASSERT_EQ(test, ret, 1L);
	KUNIT_EXPECT_FALSE(test, pte_write(test_fault_entry(test, mm, start)));
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(fragment->folio) + fragment->offset,
					    0xa4, fragment->length), NULL);
	fragment_test_unpin(pins);
	mmap_write_lock(mm);
	vm_flags_clear(vma, VM_READ);
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	mmap_write_unlock(mm);
	ret = fragment_test_pin(pins, mm, start + 17, 31, 0, 8);
	KUNIT_EXPECT_EQ(test, ret, -EFAULT);
	ret = fragment_test_pin(pins, mm, start + 17, 31, FOLL_FORCE, 8);
	KUNIT_EXPECT_EQ(test, ret, 1L);
	fragment_test_unpin(pins);
	ret = fragment_test_pin(pins, mm, start, 1, FOLL_GET, 8);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static void test_subpage_unpin(void *arg)
{
	mm_subpage_unpin(arg);
}

static void user4k_slot_pin_test(struct kunit *test)
{
	struct vm_area_struct *vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	struct mm_struct *child;
	struct mm_subpage *slots[4];
	struct folio *folio, *dst;
	unsigned int i;
	vm_fault_t fault;
	spinlock_t *ptl;
	pte_t *ptep, pte;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	mmap_read_lock(vma->vm_mm);
	fault = handle_mm_fault(vma, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(vma->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	for (i = 0; i < 4; i++) {
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA + i * SZ_4K);
		slots[i] = mm_subpage_get_from_phys(pte_phys_mm(vma->vm_mm, pte));
		KUNIT_ASSERT_NOT_NULL(test, slots[i]);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_subpage_put, slots[i]), 0);
		KUNIT_EXPECT_TRUE(test, mm_subpage_is_exclusive(slots[i]));
		memset(folio_address(mm_subpage_folio(slots[i])) + i * SZ_4K, 0x61 + i, SZ_4K);
	}
	folio = mm_subpage_folio(slots[1]);
	test_swap_tags_set(folio);
	for (i = 0; i < 2; i++) {
		ptep = test_mm_lookup(vma->vm_mm, TEST_VA + SZ_4K, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, ptep);
		ret = mm_subpage_pin(slots[1]);
		pte_unmap_unlock(ptep, ptl);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_subpage_unpin, slots[1]), 0);
	}
	KUNIT_EXPECT_EQ(test, mm_subpage_pincount(slots[1]), 2U);
	KUNIT_EXPECT_TRUE(test, folio_maybe_dma_pinned(folio));
	KUNIT_EXPECT_FALSE(test, mm_subpage_try_share(slots[1]));
	dst = folio_alloc(GFP_KERNEL, 0);
	KUNIT_ASSERT_NOT_NULL(test, dst);
	folio_lock(folio);
	folio_lock(dst);
	ret = mm_subpage_migrate_prepare(folio, dst);
	/* Finish even unexpected success so a failing assertion cannot leak metadata. */
	mm_subpage_migrate_finish(folio);
	mm_subpage_migrate_finish(dst);
	folio_unlock(dst);
	folio_unlock(folio);
	folio_put(dst);
	KUNIT_EXPECT_EQ(test, ret, -EBUSY);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, vma->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	for (i = 0; i < 4; i++) {
		struct mm_subpage *copy;

		pte = test_fault_entry(test, child, TEST_VA + i * SZ_4K);
		KUNIT_ASSERT_TRUE(test, pte_present(pte));
		KUNIT_EXPECT_EQ(test, pte_phys_mm(child, pte) == mm_subpage_phys(slots[i]), i != 1);
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slots[i]), i == 1 ? 1 : 2);
		KUNIT_EXPECT_EQ(test, mm_subpage_is_exclusive(slots[i]), i == 1);
		copy = mm_subpage_get_from_phys(pte_phys_mm(child, pte));
		KUNIT_ASSERT_NOT_NULL(test, copy);
		KUNIT_EXPECT_EQ(test, mm_subpage_is_exclusive(copy), i == 1);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(copy)) + mm_subpage_offset(copy),
						    0x61 + i, SZ_4K), NULL);
		KUNIT_EXPECT_TRUE(test, test_swap_tags_match_at(mm_subpage_folio(copy),
							     mm_subpage_offset(copy), i));
		mm_subpage_put(copy);
	}
	/* A read pin on a shared slot must request unsharing, not pin shared COW data. */
	ptep = test_mm_lookup(vma->vm_mm, TEST_VA, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, ptep);
	ret = mm_subpage_pin(slots[0]);
	pte_unmap_unlock(ptep, ptl);
	if (!ret)
		mm_subpage_unpin(slots[0]);
	KUNIT_EXPECT_EQ(test, ret, -EAGAIN);
	/* Re-enabling writes must retain the exact backing of an exclusive pinned slot. */
	ptep = test_mm_lookup(vma->vm_mm, TEST_VA + SZ_4K, &ptl);
	KUNIT_ASSERT_NOT_NULL(test, ptep);
	ptep_set_wrprotect(vma->vm_mm, TEST_VA + SZ_4K, ptep);
	pte_unmap_unlock(ptep, ptl);
	flush_tlb_range(vma, TEST_VA + SZ_4K, TEST_VA + 2 * SZ_4K);
	mmap_read_lock(vma->vm_mm);
	fault = handle_mm_fault(vma, TEST_VA + SZ_4K, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(vma->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	pte = test_fault_entry(test, vma->vm_mm, TEST_VA + SZ_4K);
	KUNIT_EXPECT_EQ(test, pte_phys_mm(vma->vm_mm, pte), mm_subpage_phys(slots[1]));
	KUNIT_EXPECT_TRUE(test, pte_write(pte));
	*((u8 *)folio_address(folio) + SZ_4K) = 0xab;
	pte = test_fault_entry(test, child, TEST_VA + SZ_4K);
	KUNIT_EXPECT_EQ(test, *(u8 *)phys_to_virt(pte_phys_mm(child, pte)), (u8)0x62);
	kunit_release_action(test, user4k_vma_mm_free, child);
	kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
	KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slots[1]), 0);
	KUNIT_EXPECT_EQ(test, mm_subpage_pincount(slots[1]), 2U);
	KUNIT_EXPECT_EQ(test, *((u8 *)folio_address(folio) + SZ_4K), (u8)0xab);
	kunit_release_action(test, test_subpage_unpin, slots[1]);
	KUNIT_EXPECT_EQ(test, mm_subpage_pincount(slots[1]), 1U);
	kunit_release_action(test, test_subpage_unpin, slots[1]);
	KUNIT_EXPECT_EQ(test, mm_subpage_pincount(slots[1]), 0U);
	KUNIT_EXPECT_FALSE(test, folio_maybe_dma_pinned(folio));
}

#ifdef CONFIG_MIGRATION
static struct folio *test_migration_alloc(struct folio *src, unsigned long private)
{
	return folio_alloc(GFP_HIGHUSER_MOVABLE, 0);
}

asmlinkage long __arm64_sys_madvise(const struct pt_regs *regs);

static long test_madvise(unsigned long addr, unsigned long size, unsigned long behavior)
{
	struct pt_regs regs = { .regs = { addr, size, behavior }, .orig_x0 = addr };

	return __arm64_sys_madvise(&regs);
}
#endif

static void user_granule_large_vma_split_test(struct kunit *test)
{
	unsigned int shifts[] = { 12, 14 }, i;

	if (!IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE) ||
	    (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN))) {
		kunit_skip(test, "requires THP boundary splitting and borrowed-mm execution");
		return;
	}
	for (i = 0; i < ARRAY_SIZE(shifts); i++) {
		struct mm_struct *mm;
		struct vm_area_struct *vma;
		unsigned long size = 1UL << shifts[i];
		unsigned long base = 4UL * HPAGE_PMD_SIZE, mapped;
		pte_t pte;
		long ret;

		if (shifts[i] >= PAGE_SHIFT)
			continue;
		mm = mm_alloc();
		KUNIT_ASSERT_NOT_NULL(test, mm);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
		KUNIT_ASSERT_EQ(test, test_mm_select_granule(mm, shifts[i]), 0);
		kthread_use_mm(mm);
		mapped = vm_mmap(NULL, base, 2UL * HPAGE_PMD_SIZE, PROT_READ | PROT_WRITE,
				MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE | MAP_NORESERVE, 0);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, mapped, base);
		vma = test_vma_lookup(mm, base);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, base, true) & VM_FAULT_ERROR, 0U);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, base + size, true) & VM_FAULT_ERROR, 0U);
		/* The generic VMA split probes the native THP boundary for this range. */
		kthread_use_mm(mm);
		ret = test_mprotect(base + size, size, PROT_READ);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0L);
		pte = test_fault_entry(test, mm, base + size);
		KUNIT_EXPECT_TRUE(test, pte_present(pte));
		KUNIT_EXPECT_FALSE(test, pte_write(pte));
		pte = test_fault_entry(test, mm, base);
		KUNIT_EXPECT_TRUE(test, pte_write(pte));
		kthread_use_mm(mm);
		ret = test_mprotect(base + size, size, PROT_READ | PROT_WRITE);
		kthread_unuse_mm(mm);
		KUNIT_EXPECT_EQ(test, ret, 0L);
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

static void user_granule_brk_extent_test(struct kunit *test)
{
	unsigned int shifts[] = { PAGE_SHIFT, 12, 14 };
	unsigned int count = PAGE_SHIFT > 14 ? 3 : 2, i;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm brk fixture requires hardware opt-in");
		return;
	}
	for (i = 0; i < count; i++) {
		struct mm_struct *mm = mm_alloc();
		struct vm_area_struct *vma;
		unsigned long size = 1UL << shifts[i], neighbor;
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, user4k_vma_mm_free, mm), 0);
		KUNIT_ASSERT_EQ(test, test_mm_select_granule(mm, shifts[i]), 0);
		kthread_use_mm(mm);
		ret = vm_brk_flags(TEST_VA, size + 1, false);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0);
		vma = test_vma_lookup(mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_EXPECT_EQ(test, vma->vm_start, TEST_VA);
		KUNIT_EXPECT_EQ(test, vma->vm_end, TEST_VA + 2 * size);
		KUNIT_EXPECT_EQ(test, mm->total_vm, 2UL);
		/* FEX reserves exactly this page after its initial ELF/BSS break. */
		kthread_use_mm(mm);
		neighbor = vm_mmap(NULL, TEST_VA + 2 * size, size, PROT_NONE,
				   MAP_FIXED_NOREPLACE | MAP_PRIVATE | MAP_ANONYMOUS, 0);
		kthread_unuse_mm(mm);
		KUNIT_EXPECT_EQ(test, neighbor, TEST_VA + 2 * size);
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

#ifdef CONFIG_MIGRATION
static void user4k_droppable_reclaim_test(struct kunit *test)
{
	unsigned int mode;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm droppable fixture requires hardware opt-in");
		return;
	}
	/* Dirty discard, retained GET, retained PIN, migration, zero-page COW. */
	for (mode = 0; mode < 5; mode++) {
		struct mm_struct *mm = test_vma_mm(test);
		struct vm_area_struct *vma;
		struct user_page_fragment held = {};
		struct folio *folio;
		unsigned int i, succeeded = 0;
		unsigned long mapped;
		long ret;
		pte_t pte;
		LIST_HEAD(folios);

		KUNIT_ASSERT_NOT_NULL(test, mm);
		kthread_use_mm(mm);
		mapped = vm_mmap(NULL, TEST_VA, PAGE_SIZE, PROT_READ | PROT_WRITE,
				MAP_FIXED_NOREPLACE | MAP_ANONYMOUS | MAP_DROPPABLE, 0);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, mapped, TEST_VA);
		vma = test_vma_lookup(mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_EXPECT_TRUE(test, vma->vm_flags & VM_WIPEONFORK);
		KUNIT_EXPECT_TRUE(test, vma->vm_flags & VM_DONTDUMP);
		if (mode == 4) {
			KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, false) & VM_FAULT_ERROR, 0U);
			pte = test_fault_entry(test, mm, TEST_VA);
			KUNIT_EXPECT_TRUE(test, is_zero_pfn(pte_pfn(pte)));
		}
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		if (mode != 4)
			for (i = 1; i < PAGE_SIZE / SZ_4K; i++)
				KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA + i * SZ_4K,
								    true) & VM_FAULT_ERROR, 0U);
		folio = test_fault_folio(mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		KUNIT_EXPECT_FALSE(test, folio_test_swapbacked(folio));
		memset(folio_address(folio), 0xa7, PAGE_SIZE);
		folio_put(folio);
		if (mode == 1 || mode == 2) {
			ret = mode == 1 ? get_user_fragments_remote(mm, TEST_VA, SZ_4K, 0, &held, 1) :
				pin_user_fragments_remote(mm, TEST_VA, SZ_4K, 0, &held, 1);
			KUNIT_ASSERT_EQ(test, ret, 1L);
		}
		if (mode == 3) {
			lru_add_drain_all();
			folio = test_fault_folio(mm, TEST_VA);
			KUNIT_ASSERT_NOT_NULL(test, folio);
			ret = isolate_folio_to_list(folio, &folios);
			folio_put(folio);
			KUNIT_ASSERT_EQ(test, ret, 1L);
			ret = migrate_pages(&folios, test_migration_alloc, NULL, 0,
					    MIGRATE_SYNC, MR_SYSCALL, &succeeded);
			putback_movable_pages(&folios);
			KUNIT_EXPECT_EQ(test, ret, 0L);
			KUNIT_EXPECT_EQ(test, succeeded, 1U);
		}
		lru_add_drain_all();
		folio = test_fault_folio(mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		folio_lock(folio);
		try_to_unmap(folio, TTU_SYNC);
		folio_unlock(folio);
		KUNIT_EXPECT_FALSE(test, folio_test_swapbacked(folio));
		KUNIT_EXPECT_FALSE(test, folio_test_swapcache(folio));
		if (mode == 1 || mode == 2) {
			pte = test_fault_entry(test, mm, TEST_VA);
			KUNIT_EXPECT_TRUE(test, pte_present(pte));
			KUNIT_EXPECT_EQ(test, *(unsigned char *)folio_address(folio), 0xa7);
			release_user_fragments(&held, 1, false);
			folio_lock(folio);
			try_to_unmap(folio, TTU_SYNC);
			folio_unlock(folio);
		}
		folio_put(folio);
		for (i = 0; i < (mode == 4 ? 1 : PAGE_SIZE / SZ_4K); i++) {
			pte = test_fault_entry(test, mm, TEST_VA + i * SZ_4K);
			KUNIT_EXPECT_TRUE(test, pte_none(pte));
		}
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 0L);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_SWAPENTS), 0L);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		folio = test_fault_folio(mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0, PAGE_SIZE), NULL);
		KUNIT_EXPECT_FALSE(test, folio_test_swapbacked(folio));
		folio_put(folio);
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

static void test_release_fragment(void *arg)
{
	release_user_fragments(arg, 1, false);
}

static void user4k_lazyfree_test(struct kunit *test)
{
	unsigned int mode;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm advice requires hardware opt-in");
		return;
	}
	/* Partial, redirty, retained GET, typed write, all leaves, migration. */
	for (mode = 0; mode < 11; mode++) {
		struct vm_area_struct *vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
		struct mm_struct *mm;
		struct user_page_fragment *held;
		struct folio *folio;
		struct mm_subpage *slot;
		unsigned int i, succeeded = 0;
		long ret;
		pte_t pte;
		LIST_HEAD(folios);

		KUNIT_ASSERT_NOT_NULL(test, vma);
		held = kunit_kzalloc(test, sizeof(*held), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, held);
		KUNIT_ASSERT_EQ(test,
			kunit_add_action_or_reset(test, test_release_fragment, held), 0);
		mm = vma->vm_mm;
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		lru_add_drain_all();
		if (mode == 6) {
			ret = get_user_fragments_remote(mm, TEST_VA, SZ_4K, 0, held, 1);
			KUNIT_ASSERT_EQ(test, ret, 1L);
		}
		kthread_use_mm(mm);
		ret = test_madvise(TEST_VA,
				   (mode == 4 || mode == 9 || mode == 10) ? PAGE_SIZE : SZ_4K,
				   MADV_FREE);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, ret, 0L);
		pte = test_fault_entry(test, mm, TEST_VA);
		KUNIT_EXPECT_EQ(test, pte_dirty(pte), mode == 6);
		slot = mm_subpage_get_from_phys(pte_phys_mm(mm, pte));
		KUNIT_ASSERT_NOT_NULL(test, slot);
		KUNIT_EXPECT_EQ(test, mm_subpage_is_lazyfree(slot), mode != 6);
		mm_subpage_put(slot);
		if (mode == 1)
			KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);

		if (mode == 2 || mode == 3 || mode == 8 || mode == 10) {
			ret = get_user_fragments_remote(mm, TEST_VA, SZ_4K,
						      mode == 3 ? FOLL_WRITE : 0, held, 1);
			KUNIT_ASSERT_EQ(test, ret, 1L);
			if (mode == 3 || mode == 8) {
				memset(folio_address(held->folio) + held->offset, 0xd3, SZ_4K);
				release_user_fragments(held, 1, true);
			}
		}
		if (mode == 7) {
			ret = pin_user_fragments_remote(mm, TEST_VA, SZ_4K, 0, held, 1);
			KUNIT_ASSERT_EQ(test, ret, 1L);
		}
		if (mode == 5 || mode == 9) {
			lru_add_drain_all();
			folio = test_fault_folio(mm, TEST_VA);
			KUNIT_ASSERT_NOT_NULL(test, folio);
			ret = isolate_folio_to_list(folio, &folios);
			folio_put(folio);
			KUNIT_ASSERT_EQ(test, ret, 1L);
			ret = migrate_pages(&folios, test_migration_alloc, NULL, 0,
					    MIGRATE_SYNC, MR_SYSCALL, &succeeded);
			putback_movable_pages(&folios);
			KUNIT_EXPECT_EQ(test, ret, 0L);
			KUNIT_EXPECT_EQ(test, succeeded, 1U);
		}
		lru_add_drain_all();
		/* A retained GET can coexist with a temporary migration PTE. */
		folio = held->folio;
		if (folio)
			folio_get(folio);
		else
			folio = test_fault_folio(mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		folio_lock(folio);
		try_to_unmap(folio, TTU_SYNC);
		if (mode == 10) {
			/* A temporary reader leaves clean whole-folio advice intact. */
			KUNIT_EXPECT_FALSE(test, folio_test_swapbacked(folio));
			KUNIT_EXPECT_FALSE(test, folio_test_dirty(folio));
			KUNIT_EXPECT_FALSE(test, pte_none(test_fault_entry(test, mm, TEST_VA)));
			release_user_fragments(held, 1, false);
			try_to_unmap(folio, TTU_SYNC);
		}
		folio_unlock(folio);
		folio_put(folio);
		for (i = 0; i < 4; i++) {
			pte = test_fault_entry(test, mm, TEST_VA + i * SZ_4K);
			/* Migration may refold dirty neighbors and retain an advised slot. */
			if (mode == 5 && !i && pte_present(pte)) {
				KUNIT_EXPECT_TRUE(test, pte_dirty(pte));
				continue;
			}
			KUNIT_EXPECT_EQ_MSG(test, pte_none(pte),
					(mode == 4 || mode == 9 || mode == 10) ||
					(i == 0 && (mode == 0 || mode == 5)),
					"mode=%u slot=%u pte=%llx", mode, i,
					(unsigned long long)pte_val(pte));
		}
		if (mode == 2 || mode == 6 || mode == 7)
			release_user_fragments(held, 1, false);
		kunit_release_action(test, test_release_fragment, held);
		kunit_kfree(test, held);
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

static void user4k_mprotect_migration_test(struct kunit *test)
{
	unsigned int file_case;

	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm permission fixture requires hardware opt-in");
		return;
	}
	for (file_case = 0; file_case < 2; file_case++) {
		struct mm_struct *mm = test_vma_mm(test);
		struct file *file = file_case ? test_cache_file(test, PAGE_SIZE, true) : NULL;
		struct vm_area_struct *vma;
		struct folio *src, *dst;
		unsigned long mapped;
		unsigned int i;
		int prepared, unmapped = -1;
		long protected = -EINVAL;
		bool offsets = true;
		phys_addr_t phys;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		if (file_case)
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		kthread_use_mm(mm);
		mapped = vm_mmap(file, TEST_VA, PAGE_SIZE, PROT_READ | PROT_WRITE,
			MAP_FIXED_NOREPLACE | (file ? MAP_SHARED : MAP_PRIVATE | MAP_ANONYMOUS), 0);
		kthread_unuse_mm(mm);
		KUNIT_ASSERT_EQ(test, mapped, TEST_VA);
		vma = test_vma_lookup(mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		dst = folio_alloc(GFP_KERNEL, 0);
		KUNIT_ASSERT_NOT_NULL(test, dst);
		src = test_fault_folio(mm, TEST_VA);
		if (!src) {
			folio_put(dst);
			KUNIT_FAIL(test, "missing migration source");
			return;
		}
		phys = PFN_PHYS(folio_pfn(src));
		folio_lock(src);
		folio_lock(dst);
		prepared = mm_subpage_migrate_prepare(src, dst);
		if (!prepared) {
			try_to_migrate(src, TTU_SYNC);
			unmapped = folio_mapcount(src);
			kthread_use_mm(mm);
			protected = test_mprotect(TEST_VA, PAGE_SIZE, PROT_READ);
			kthread_unuse_mm(mm);
			for (i = 0; i < 4; i++) {
				pte_t pte = test_fault_entry(test, mm, TEST_VA + i * SZ_4K);

				offsets &= softleaf_is_migration(softleaf_from_pte(pte)) &&
					!softleaf_is_migration_write(softleaf_from_pte(pte)) &&
					pte_swp_subpage_offset(pte) == i * SZ_4K;
			}
			remove_migration_ptes(src, src, 0);
			mm_subpage_migrate_finish(src);
			mm_subpage_migrate_finish(dst);
		}
		folio_unlock(dst);
		folio_unlock(src);
		folio_put(dst);
		folio_put(src);
		KUNIT_ASSERT_EQ(test, prepared, 0);
		KUNIT_EXPECT_EQ(test, unmapped, 0);
		KUNIT_EXPECT_EQ(test, protected, 0L);
		KUNIT_EXPECT_TRUE(test, offsets);
		for (i = 0; i < 4; i++) {
			pte_t pte = test_fault_entry(test, mm, TEST_VA + i * SZ_4K);

			KUNIT_EXPECT_TRUE(test, pte_present(pte));
			KUNIT_EXPECT_FALSE(test, pte_write(pte));
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte), phys + i * SZ_4K);
		}
	}
}

/* Independent roots and VA offsets must not change physical slot identity. */
static void user4k_nonlinear_rmap_test(struct kunit *test)
{
	struct anon_slot_fixture *a = anon_slot_fixture_create(test);
	struct anon_slot_fixture *b = anon_slot_fixture_create(test);
	struct mm_struct *child = test_vma_mm(test);
	struct mm_subpage *slot;
	struct folio *folio, *dst;
	phys_addr_t phys;
	vma_flags_t flags;
	unsigned int succeeded = 0, side;
	int ret, unmapped = -1;
	bool isolated;
	LIST_HEAD(folios);

	KUNIT_ASSERT_NOT_NULL(test, a);
	KUNIT_ASSERT_NOT_NULL(test, b);
	KUNIT_ASSERT_NOT_NULL(test, child);
	slot = mm_subpage_alloc_at(a->pool, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
	folio = mm_subpage_folio(slot);
	/* Unlike the lightweight rmap fixture, real LRU migration needs a charge. */
	ret = mem_cgroup_charge(folio, a->mm, GFP_KERNEL);
	if (ret) {
		mm_subpage_put(slot);
		KUNIT_FAIL(test, "cannot charge nonlinear fixture folio");
		return;
	}
	KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(a, 0, slot), 0);
	phys = mm_subpage_phys(slot);
	slot = mm_subpage_alloc_at(a->pool, 3 * SZ_4K);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
	/* Same native folio, different root and different VA/physical offset. */
	KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(b, 1, slot), 0);
	KUNIT_EXPECT_PTR_EQ(test, mm_subpage_folio(slot), folio);
	memset(folio_address(folio), 0x53, SZ_4K);
	memset(folio_address(folio) + 3 * SZ_4K, 0xa7, SZ_4K);
	folio_lock(folio);
	ret = folio_referenced(folio, 1, NULL, &flags);
	folio_unlock(folio);
	KUNIT_EXPECT_EQ(test, ret, 2);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, b->mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 2);
	KUNIT_EXPECT_EQ(test, pte_phys_mm(child,
		test_fault_entry(test, child, TEST_VA + SZ_4K)), phys + 3 * SZ_4K);
	kunit_release_action(test, user4k_vma_mm_free, child);

	/* Rollback must find migration entries through both roots. */
	dst = folio_alloc(GFP_KERNEL, 0);
	KUNIT_ASSERT_NOT_NULL(test, dst);
	folio_get(folio);
	folio_lock(folio);
	folio_lock(dst);
	ret = mm_subpage_migrate_prepare(folio, dst);
	if (!ret) {
		try_to_migrate(folio, TTU_SYNC);
		unmapped = folio_mapcount(folio);
		remove_migration_ptes(folio, folio, 0);
		mm_subpage_migrate_finish(folio);
		mm_subpage_migrate_finish(dst);
	}
	folio_unlock(dst);
	folio_unlock(folio);
	folio_put(dst);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, unmapped, 0);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 2);

	/* Successful migration exercises copied identities in the new owner. */
	folio_add_lru(folio);
	lru_add_drain_all();
	folio_get(folio);
	isolated = isolate_folio_to_list(folio, &folios);
	folio_put(folio);
	KUNIT_ASSERT_TRUE(test, isolated);
	ret = migrate_pages(&folios, test_migration_alloc, NULL, 0,
			    MIGRATE_SYNC, MR_SYSCALL, &succeeded);
	putback_movable_pages(&folios);
	if (ret) {
		KUNIT_FAIL(test, "nonlinear migration failed: %d", ret);
		return;
	}
	/* Refresh fixture references before assertions/cleanup after migration. */
	for (side = 0; side < 2; side++) {
		struct anon_slot_fixture *f = side ? b : a;
		pte_t pte = test_fault_entry(test, f->mm, TEST_VA + side * SZ_4K);

		f->slots[side] = NULL;
		f->mapped[side] = false;
		if (!pte_present(pte))
			continue;
		slot = mm_subpage_get_from_phys(pte_phys_mm(f->mm, pte));
		if (!slot)
			continue;
		f->slots[side] = slot;
		f->mapped[side] = true;
		mm_subpage_put(slot);
	}
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, succeeded, 1U);
	for (side = 0; side < 2; side++) {
		struct anon_slot_fixture *f = side ? b : a;

		slot = f->slots[side];
		KUNIT_ASSERT_NOT_NULL(test, slot);
		KUNIT_EXPECT_EQ(test, mm_subpage_offset(slot), side ? 3U * SZ_4K : 0U);
		KUNIT_EXPECT_NE(test, mm_subpage_phys(slot), phys + (side ? 3 * SZ_4K : 0));
		KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), 1);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(slot)) +
			mm_subpage_offset(slot), side ? 0xa7 : 0x53, SZ_4K), NULL);
	}
}

static void user4k_migration_entries_test(struct kunit *test)
{
	struct vm_area_struct *vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	struct mm_struct *child = test_vma_mm(test);
	struct folio *src, *dst;
	unsigned int i, side;
	vm_fault_t fault;
	bool encoded = true, copied = true, exact = true;
	int ret, fork_ret = -EINVAL, unmapped = -1;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_NOT_NULL(test, child);
	mmap_read_lock(vma->vm_mm);
	fault = handle_mm_fault(vma, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(vma->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	dst = folio_alloc(GFP_KERNEL, 0);
	KUNIT_ASSERT_NOT_NULL(test, dst);
	src = test_fault_folio(vma->vm_mm, TEST_VA);
	if (!src) {
		folio_put(dst);
		KUNIT_FAIL(test, "missing source folio");
		return;
	}
	folio_lock(src);
	folio_lock(dst);
	ret = mm_subpage_migrate_prepare(src, dst);
	if (!ret) {
		try_to_migrate(src, TTU_SYNC);
		unmapped = folio_mapcount(src);
		for (i = 0; i < 4; i++) {
			pte_t pte = test_fault_entry(test, vma->vm_mm, TEST_VA + i * SZ_4K);
			struct rmap_walk_range range = {
				.address = TEST_VA + i * SZ_4K,
				.subpage = true,
				.subpage_offset = i * SZ_4K,
			};
			DEFINE_FOLIO_RMAP_WALK(walk, src, vma, range,
					     PVMW_SYNC | PVMW_MIGRATION);
			unsigned int count = 0;

			encoded &= softleaf_is_migration_write(softleaf_from_pte(pte)) &&
				pte_swp_subpage_offset(pte) == i * SZ_4K;
			while (page_vma_mapped_walk(&walk)) {
				exact &= walk.address == TEST_VA + i * SZ_4K;
				count++;
			}
			exact &= count == 1;
		}
		uprobe_start_dup_mmap();
		fork_ret = dup_mmap(child, vma->vm_mm);
		uprobe_end_dup_mmap();
		if (!fork_ret) {
			for (side = 0; side < 2; side++) {
				struct mm_struct *mm = side ? child : vma->vm_mm;

				for (i = 0; i < 4; i++) {
					pte_t pte = test_fault_entry(test, mm, TEST_VA + i * SZ_4K);

					copied &= softleaf_is_migration_read(softleaf_from_pte(pte)) &&
						pte_swp_subpage_offset(pte) == i * SZ_4K;
				}
			}
		}
		/* Drop one migration entry while the other mm still retains it. */
		mmap_read_lock(vma->vm_mm);
		zap_vma_range(vma, TEST_VA + SZ_4K, SZ_4K);
		mmap_read_unlock(vma->vm_mm);
		remove_migration_ptes(src, src, 0);
		mm_subpage_migrate_finish(src);
		mm_subpage_migrate_finish(dst);
	}
	folio_unlock(dst);
	folio_unlock(src);
	folio_put(dst);
	folio_put(src);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, fork_ret, 0);
	KUNIT_EXPECT_EQ(test, unmapped, 0);
	KUNIT_EXPECT_TRUE(test, encoded);
	KUNIT_EXPECT_TRUE(test, copied);
	KUNIT_EXPECT_TRUE(test, exact);
	for (side = 0; side < 2; side++) {
		struct mm_struct *mm = side ? child : vma->vm_mm;

		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), side ? (unsigned long)TEST_SLOTS : TEST_SLOTS - 1UL);
		for (i = 0; i < 4; i++) {
			pte_t pte = test_fault_entry(test, mm, TEST_VA + i * SZ_4K);
			struct mm_subpage *slot;

			if (!side && i == 1) {
				KUNIT_EXPECT_TRUE(test, pte_none(pte));
				continue;
			}
			KUNIT_ASSERT_TRUE(test, pte_present(pte));
			KUNIT_EXPECT_FALSE(test, pte_write(pte));
			slot = mm_subpage_get_from_phys(pte_phys_mm(mm, pte));
			KUNIT_ASSERT_NOT_NULL(test, slot);
			KUNIT_EXPECT_EQ(test, mm_subpage_offset(slot), i * (unsigned int)SZ_4K);
			KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), i == 1 ? 1 : 2);
			mm_subpage_put(slot);
		}
	}
}

static struct folio *test_file_migration_alloc(struct folio *src, unsigned long private)
{
	return folio_alloc(GFP_HIGHUSER_MOVABLE, folio_order(src));
}

static void user64k_file_migration_test(struct kunit *test)
{
	unsigned int mode;

	if (!test_coarse_file_enabled(test))
		return;
	if (!IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE)) {
		kunit_skip(test, "coarse shmem promotion requires collapse support");
		return;
	}
	for (mode = 0; mode < 6; mode++) {
		bool shmem = mode & 1, fail = mode >= 2, pin = mode >= 4;
		bool isolated;
		unsigned long len = shmem ? SZ_64K : 2 * SZ_64K;
		unsigned long base = shmem ? SZ_64K : 0, offset;
		unsigned int nr = len / PAGE_SIZE, succeeded = 0;
		unsigned int maps = 2 * nr + len / SZ_4K - 1;
		struct file *file = shmem ? test_cache_file(test, 2 * SZ_64K, true) :
			test_coarse_cache_file(test, 17 - PAGE_SHIFT, 2 * SZ_64K);
		struct vm_area_struct *coarse, *native, *small;
		struct folio *folio;
		struct user_page_fragment *fragment = NULL;
		phys_addr_t old_phys, phys;
		int ret;
		LIST_HEAD(folios);

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		coarse = test_file_vma_granule(test, file, 16, true, TEST_VA, len, base);
		native = test_file_vma(test, file, true, true, TEST_VA, len, base);
		/* Shifted file offset and a small-page table boundary in the alias. */
		small = test_file_vma(test, file, false, true, SZ_2M - SZ_4K,
				     len - SZ_4K, base + SZ_4K);
		KUNIT_ASSERT_NOT_NULL(test, coarse);
		KUNIT_ASSERT_NOT_NULL(test, native);
		KUNIT_ASSERT_NOT_NULL(test, small);
		for (offset = 0; offset < len; offset += SZ_64K)
			KUNIT_ASSERT_EQ(test, test_file_fault(coarse, TEST_VA + offset, shmem) &
					VM_FAULT_ERROR, 0U);
		for (offset = 0; offset < len; offset += PAGE_SIZE)
			KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA + offset, shmem) &
					VM_FAULT_ERROR, 0U);
		for (offset = 0; offset < len - SZ_4K; offset += SZ_4K)
			KUNIT_ASSERT_EQ(test, test_file_fault(small, small->vm_start + offset,
							    shmem) & VM_FAULT_ERROR, 0U);
		lru_add_drain_all();
		folio = filemap_get_folio(file->f_mapping, base >> PAGE_SHIFT);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_EQ(test, folio_size(folio), len);
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)maps);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)(nr + maps + 1));
		old_phys = page_to_phys(&folio->page);
		if (pin) {
			fragment = kunit_kzalloc(test, sizeof(*fragment), GFP_KERNEL);
			KUNIT_ASSERT_NOT_NULL(test, fragment);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
						test_promote_unpin, fragment), 0);
			KUNIT_ASSERT_EQ(test, pin_user_fragments_remote(small->vm_mm,
					small->vm_end - SZ_4K + 37, 1, 0, fragment, 1), 1L);
			KUNIT_EXPECT_PTR_EQ(test, fragment->folio, folio);
			KUNIT_EXPECT_EQ(test, fragment->offset, len - SZ_4K + 37);
		}
		isolated = isolate_folio_to_list(folio, &folios);
		/* Retain either a GET or a typed PIN to force migration rollback. */
		if (!fail || pin || !isolated)
			folio_put(folio);
		KUNIT_ASSERT_TRUE(test, isolated);
		ret = migrate_pages(&folios, test_file_migration_alloc, NULL, 0,
				    MIGRATE_SYNC, MR_SYSCALL, &succeeded);
		putback_movable_pages(&folios);
		if (pin) {
			KUNIT_EXPECT_EQ(test, page_to_phys(&fragment->folio->page), old_phys);
			KUNIT_EXPECT_EQ(test, *(u8 *)(folio_address(fragment->folio) +
							 fragment->offset),
					(u8)(0x31 + (base + len - SZ_4K) / SZ_4K));
			kunit_release_action(test, test_promote_unpin, fragment);
		} else if (fail) {
			folio_put(folio);
		}
		KUNIT_EXPECT_EQ_MSG(test, ret, fail ? 1 : 0, "mode=%u", mode);
		KUNIT_EXPECT_EQ(test, succeeded, fail ? 0U : nr);
		folio = filemap_get_folio(file->f_mapping, base >> PAGE_SHIFT);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
		phys = page_to_phys(&folio->page);
		KUNIT_EXPECT_EQ(test, phys == old_phys, fail);
		KUNIT_EXPECT_EQ(test, folio_size(folio), len);
		KUNIT_EXPECT_EQ(test, folio->index, base >> PAGE_SHIFT);
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)maps);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)(nr + maps + 1));
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(coarse->vm_mm,
				shmem ? MM_SHMEMPAGES : MM_FILEPAGES), (long)(len / SZ_64K));
		for (offset = 0; offset < len; offset += SZ_4K) {
			pte_t pte;

			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + offset,
						0x31 + (base + offset) / SZ_4K, SZ_4K), NULL);
			if (offset) {
				pte = test_fault_entry(test, small->vm_mm,
						small->vm_start + offset - SZ_4K);
				KUNIT_EXPECT_TRUE(test, pte_present(pte));
				KUNIT_EXPECT_EQ(test, !!pte_write(pte), shmem);
				KUNIT_EXPECT_EQ(test, pte_phys_mm(small->vm_mm, pte), phys + offset);
			}
			if (!(offset % PAGE_SIZE)) {
				pte = test_fault_entry(test, native->vm_mm, TEST_VA + offset);
				KUNIT_EXPECT_TRUE(test, pte_present(pte));
				KUNIT_EXPECT_EQ(test, !!pte_write(pte), shmem);
				KUNIT_EXPECT_EQ(test, pte_phys_mm(native->vm_mm, pte), phys + offset);
			}
			if (!(offset % SZ_64K)) {
				pte = test_fault_entry(test, coarse->vm_mm, TEST_VA + offset);
				KUNIT_EXPECT_TRUE(test, pte_present(pte));
				KUNIT_EXPECT_EQ(test, !!pte_write(pte), shmem);
				KUNIT_EXPECT_EQ(test, pte_phys_mm(coarse->vm_mm, pte), phys + offset);
			}
		}
		kunit_release_action(test, user4k_vma_mm_free, coarse->vm_mm);
		kunit_release_action(test, user4k_vma_mm_free, native->vm_mm);
		kunit_release_action(test, user4k_vma_mm_free, small->vm_mm);
		lru_add_drain_all();
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)nr + 1);
	}
}

static void user64k_anon_migration_test(struct kunit *test)
{
	unsigned int mode, nr = SZ_64K / PAGE_SIZE;

	if (!test_coarse_file_enabled(test))
		return;
	for (mode = 0; mode < 5; mode++) {
		bool forked = mode & 1, fail = mode >= 2, pin = mode == 4;
		struct vm_area_struct *vma, *cvma = NULL;
		struct mm_struct *child = NULL;
		struct user_page_fragment *fragment = NULL;
		struct folio *folio;
		phys_addr_t old_phys, phys;
		unsigned int i, succeeded = 0;
		pte_t pte;
		bool isolated;
		int ret;
		LIST_HEAD(folios);

		vma = test_fault_vma_granule(test, 16, TEST_VA, SZ_64K);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA + SZ_64K - 1, true) &
				VM_FAULT_ERROR, 0U);
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_TRUE(test, pte_present(pte));
		old_phys = pte_phys_mm(vma->vm_mm, pte);
		folio = page_folio(pfn_to_page(PHYS_PFN(old_phys)));
		for (i = 0; i < nr; i++)
			memset(page_address(folio_page(folio, i)), 0x51 + i, PAGE_SIZE);
		if (forked) {
			child = test_vma_mm_granule(test, 16);
			KUNIT_ASSERT_NOT_NULL(test, child);
			uprobe_start_dup_mmap();
			ret = dup_mmap(child, vma->vm_mm);
			uprobe_end_dup_mmap();
			KUNIT_ASSERT_EQ(test, ret, 0);
			cvma = test_vma_lookup(child, TEST_VA);
			KUNIT_ASSERT_NOT_NULL(test, cvma);
		}
		if (pin) {
			fragment = kunit_kzalloc(test, sizeof(*fragment), GFP_KERNEL);
			KUNIT_ASSERT_NOT_NULL(test, fragment);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
						test_promote_unpin, fragment), 0);
			KUNIT_ASSERT_EQ(test, pin_user_fragments_remote(vma->vm_mm,
					TEST_VA + SZ_64K - 1, 1, FOLL_WRITE, fragment, 1), 1L);
			KUNIT_EXPECT_PTR_EQ(test, fragment->folio, folio);
			KUNIT_EXPECT_EQ(test, fragment->offset, (unsigned int)SZ_64K - 1);
		}
		lru_add_drain_all();
		folio_get(folio);
		isolated = isolate_folio_to_list(folio, &folios);
		if (!fail || pin || !isolated)
			folio_put(folio);
		KUNIT_ASSERT_TRUE(test, isolated);
		ret = migrate_pages(&folios, test_file_migration_alloc, NULL, 0,
				    MIGRATE_SYNC, MR_SYSCALL, &succeeded);
		putback_movable_pages(&folios);
		if (pin)
			kunit_release_action(test, test_promote_unpin, fragment);
		else if (fail)
			folio_put(folio);
		KUNIT_EXPECT_EQ_MSG(test, ret, fail ? 1 : 0, "mode=%u", mode);
		KUNIT_EXPECT_EQ(test, succeeded, fail ? 0U : nr);
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_TRUE(test, pte_present(pte));
		KUNIT_EXPECT_EQ(test, !!pte_write(pte), !forked);
		phys = pte_phys_mm(vma->vm_mm, pte);
		KUNIT_EXPECT_EQ(test, phys == old_phys, fail);
		folio = page_folio(pfn_to_page(PHYS_PFN(phys)));
		folio_get(folio);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
		KUNIT_EXPECT_EQ(test, folio_size(folio), (unsigned long)SZ_64K);
		expect_coarse_anon_state(test, folio, forked ? 2 : 1, !forked);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)(nr * (forked ? 2 : 1) + 1));
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_ANONPAGES), 1L);
		if (forked) {
			struct folio *copied;

			pte = test_fault_entry(test, child, TEST_VA);
			KUNIT_EXPECT_EQ(test, pte_phys_mm(child, pte), phys);
			KUNIT_EXPECT_FALSE(test, pte_write(pte));
			KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_ANONPAGES), 1L);
			/* A restored shared migration entry must still perform full-leaf COW. */
			KUNIT_ASSERT_EQ(test, test_file_fault(cvma, TEST_VA + SZ_64K - 1,
							    true) & VM_FAULT_ERROR, 0U);
			pte = test_fault_entry(test, child, TEST_VA);
			KUNIT_EXPECT_NE(test, pte_phys_mm(child, pte), phys);
			copied = page_folio(pfn_to_page(PHYS_PFN(pte_phys_mm(child, pte))));
			expect_coarse_anon_state(test, copied, 1, true);
			for (i = 0; i < nr; i++)
				KUNIT_EXPECT_PTR_EQ(test, memchr_inv(page_address(folio_page(copied, i)),
								0x51 + i, PAGE_SIZE), NULL);
			memset(folio_address(copied), 0xa7, SZ_64K);
			kunit_release_action(test, user4k_vma_mm_free, child);
		}
		for (i = 0; i < nr; i++)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(page_address(folio_page(folio, i)),
							0x51 + i, PAGE_SIZE), NULL);
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
		lru_add_drain_all();
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
	}
}

static void user4k_file_migration_test(struct kunit *test)
{
	unsigned int fail;

	for (fail = 0; fail < 2; fail++) {
		struct file *file = test_cache_file(test, PAGE_SIZE, true);
		struct vm_area_struct *small, *native;
		struct folio *folio;
		phys_addr_t old_phys;
		unsigned int i, succeeded = 0;
		int ret;
		bool isolated;
		LIST_HEAD(folios);

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		small = test_file_vma(test, file, false, true, SZ_2M - SZ_4K, PAGE_SIZE, 0);
		native = test_file_vma(test, file, true, true, TEST_VA, PAGE_SIZE, 0);
		KUNIT_ASSERT_NOT_NULL(test, small);
		KUNIT_ASSERT_NOT_NULL(test, native);
		for (i = 0; i < 4; i++)
			KUNIT_ASSERT_EQ(test, test_file_fault(small, small->vm_start + i * SZ_4K,
							    true) & VM_FAULT_ERROR, 0U);
		KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		lru_add_drain_all();
		folio = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ [0 ... TEST_SLOTS - 1] = 1 }, 1);
		old_phys = PFN_PHYS(folio_pfn(folio));
		isolated = isolate_folio_to_list(folio, &folios);
		if (!fail || !isolated)
			folio_put(folio);
		KUNIT_ASSERT_TRUE(test, isolated);
		ret = migrate_pages(&folios, test_migration_alloc, NULL, 0, MIGRATE_SYNC,
				    MR_SYSCALL, &succeeded);
		putback_movable_pages(&folios);
		if (fail)
			folio_put(folio);
		KUNIT_EXPECT_EQ(test, ret, fail ? 1 : 0);
		KUNIT_EXPECT_EQ(test, succeeded, fail ? 0U : 1U);
		folio = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ [0 ... TEST_SLOTS - 1] = 1 }, 1);
		for (i = 0; i < 4; i++) {
			pte_t entry = test_fault_entry(test, small->vm_mm, small->vm_start + i * SZ_4K);

			KUNIT_ASSERT_TRUE(test, pte_present(entry));
			KUNIT_EXPECT_TRUE(test, pte_write(entry));
			KUNIT_EXPECT_EQ(test, pte_phys_mm(small->vm_mm, entry),
					PFN_PHYS(folio_pfn(folio)) + i * SZ_4K);
			KUNIT_EXPECT_EQ(test, pte_phys_mm(small->vm_mm, entry) == old_phys + i * SZ_4K, !!fail);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + i * SZ_4K,
							   0x31 + i, SZ_4K), NULL);
		}
		KUNIT_EXPECT_EQ(test, pte_phys_mm(native->vm_mm,
				test_fault_entry(test, native->vm_mm, TEST_VA)), PFN_PHYS(folio_pfn(folio)));
		/* File reclaim must visit every 4K alias, including across PTE tables. */
		folio_lock(folio);
		try_to_unmap(folio, TTU_SYNC);
		KUNIT_EXPECT_FALSE(test, folio_mapped(folio));
		test_file_mapcounts(test, folio, (unsigned int[TEST_SLOTS]){ 0, 0, 0, 0 }, 0);
		folio_unlock(folio);
		folio_put(folio);
		for (i = 0; i < 4; i++) {
			KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, small->vm_mm,
								   small->vm_start + i * SZ_4K)));
		}
		for (i = 0; i < 4; i++)
			KUNIT_ASSERT_EQ(test, test_file_fault(small, small->vm_start + i * SZ_4K,
							    false) & VM_FAULT_ERROR, 0U);
	}
}

static void user4k_migration_test(struct kunit *test)
{
	const unsigned long boundaries[] = {
		SZ_2M - SZ_4K, SZ_2M, SZ_1G - SZ_4K, SZ_1G,
		BIT(39) - SZ_4K, BIT(39),
	};
	unsigned int mode;

	/* Dense/sparse, forks, refcount rollback, batched TLBI and table edges. */
	for (mode = 0; mode < 16; mode++) {
		bool edge = mode >= 6 && mode < 12;
		bool locked = mode >= 12;
		bool sparse = mode == 1 || edge || mode == 13;
		bool forked = mode == 2 || mode == 4 || edge || mode == 14;
		bool fail = mode == 3 || mode == 4 || mode == 15;
		unsigned long start = edge ? boundaries[mode - 6] :
			TEST_VA + (sparse ? SZ_4K : 0);
		unsigned int nr = sparse ? 1 : 4, i, side, succeeded = 0;
		unsigned int offsets[TEST_SLOTS];
		struct vm_area_struct *vma = test_fault_vma(test, start, nr * SZ_4K);
		struct mm_struct *child = NULL;
		struct folio *folio;
		phys_addr_t old_phys;
		vm_fault_t fault;
		int ret;
		bool isolated;
		LIST_HEAD(folios);

		KUNIT_ASSERT_NOT_NULL(test, vma);
		if (locked) {
			mmap_write_lock(vma->vm_mm);
			vm_flags_set(vma, VM_LOCKED);
			vma->vm_mm->locked_vm = nr;
			mmap_write_unlock(vma->vm_mm);
		}
		mmap_read_lock(vma->vm_mm);
		fault = handle_mm_fault(vma, start, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(vma->vm_mm);
		KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
		folio = test_fault_folio(vma->vm_mm, start);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		old_phys = PFN_PHYS(folio_pfn(folio));
		for (i = 0; i < TEST_SLOTS; i++)
			memset(folio_address(folio) + i * SZ_4K, 0x41 + i + mode, SZ_4K);
		test_swap_tags_set(folio);
		for (i = 0; i < nr; i++)
			offsets[i] = offset_in_page(pte_phys_mm(vma->vm_mm,
					test_fault_entry(test, vma->vm_mm, start + i * SZ_4K)));
		folio_put(folio);
		if (forked) {
			child = test_vma_mm(test);
			KUNIT_ASSERT_NOT_NULL(test, child);
			uprobe_start_dup_mmap();
			ret = dup_mmap(child, vma->vm_mm);
			uprobe_end_dup_mmap();
			KUNIT_ASSERT_EQ(test, ret, 0);
		}
		lru_add_drain_all();
		folio = test_fault_folio(vma->vm_mm, start);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		isolated = isolate_folio_to_list(folio, &folios);
		/* An extra native reference forces the normal migration rollback. */
		if (!fail || !isolated)
			folio_put(folio);
		KUNIT_ASSERT_TRUE(test, isolated);
		ret = migrate_pages(&folios, test_migration_alloc, NULL, 0,
				    mode == 5 ? MIGRATE_ASYNC : MIGRATE_SYNC,
				    MR_SYSCALL, &succeeded);
		putback_movable_pages(&folios);
		if (fail)
			folio_put(folio);
		KUNIT_EXPECT_EQ(test, ret, fail ? 1 : 0);
		KUNIT_EXPECT_EQ(test, succeeded, fail ? 0U : 1U);
		for (side = 0; side <= (unsigned int)forked; side++) {
			struct mm_struct *mm = side ? child : vma->vm_mm;

			for (i = 0; i < nr; i++) {
				unsigned int index = offsets[i] / SZ_4K;
				pte_t pte = test_fault_entry(test, mm, start + i * SZ_4K);
				struct mm_subpage *slot;

				KUNIT_ASSERT_TRUE(test, pte_present(pte));
				KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte) == old_phys + index * SZ_4K, fail);
				KUNIT_EXPECT_EQ(test, !!pte_write(pte), !forked);
				slot = mm_subpage_get_from_phys(pte_phys_mm(mm, pte));
				KUNIT_ASSERT_NOT_NULL(test, slot);
				folio = mm_subpage_folio(slot);
				KUNIT_EXPECT_EQ(test, mm_subpage_offset(slot), index * (unsigned int)SZ_4K);
				KUNIT_EXPECT_EQ(test, mm_subpage_mapcount(slot), forked ? 2 : 1);
				KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + index * SZ_4K,
								    0x41 + index + mode, SZ_4K), NULL);
				KUNIT_EXPECT_TRUE(test, test_swap_tags_match(folio, index));
				mm_subpage_put(slot);
			}
			KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), (unsigned long)nr);
		}
		if (locked) {
			lru_add_drain_all();
			folio = test_fault_folio(vma->vm_mm, start);
			KUNIT_ASSERT_NOT_NULL(test, folio);
			KUNIT_EXPECT_TRUE(test, folio_test_mlocked(folio));
			if (!fail)
				KUNIT_EXPECT_EQ(test, folio->mlock_count, nr);
			folio_put(folio);
		}
		if (child)
			kunit_release_action(test, user4k_vma_mm_free, child);
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
	}
}
struct fragment_pin_worker {
	struct mm_struct *mm;
	struct task_struct *task;
	struct completion ready;
	unsigned long checks;
	int error;
};

static int fragment_pin_worker_run(void *arg)
{
	struct fragment_pin_worker *worker = arg;
	struct user_page_fragment fragments[4];

	while (!kthread_should_stop()) {
		long nr = pin_user_fragments_remote(worker->mm, TEST_VA, PAGE_SIZE,
						   0, fragments, ARRAY_SIZE(fragments));
		unsigned int i;

		if (nr != 4) {
			worker->error = nr < 0 ? nr : -EIO;
		} else {
			for (i = 0; i < nr; i++) {
				struct user_page_fragment *f = &fragments[i];

				if (memchr_inv(folio_address(f->folio) + f->offset, 0x51 + i, f->length))
					worker->error = -EILSEQ;
				worker->checks++;
			}
		}
		if (nr > 0)
			release_user_fragments(fragments, nr, false);
		complete_all(&worker->ready);
		if (worker->error)
			break;
		cond_resched();
	}
	return 0;
}

static void fragment_pin_worker_stop(void *arg)
{
	struct fragment_pin_worker *worker = arg;

	if (worker->task) {
		kthread_stop(worker->task);
		put_task_struct(worker->task);
		worker->task = NULL;
	}
}

static void user4k_fragment_pin_race_test(struct kunit *test)
{
	struct vm_area_struct *vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	struct fragment_test_pins *pins;
	struct fragment_pin_worker *workers;
	unsigned int i, round, migrated = 0, busy = 0;
	long ret;

	KUNIT_ASSERT_NOT_NULL(test, vma);
	pins = kunit_kzalloc(test, sizeof(*pins), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pins);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_test_unpin, pins), 0);
	ret = fragment_test_pin(pins, vma->vm_mm, TEST_VA, PAGE_SIZE, FOLL_WRITE, 4);
	KUNIT_ASSERT_EQ(test, ret, 4L);
	for (i = 0; i < 4; i++)
		memset(folio_address(pins->fragments[i].folio) + pins->fragments[i].offset,
		       0x51 + i, pins->fragments[i].length);
	fragment_test_unpin(pins);
	workers = kunit_kcalloc(test, 2, sizeof(*workers), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, workers);
	for (i = 0; i < 2; i++) {
		workers[i].mm = vma->vm_mm;
		init_completion(&workers[i].ready);
		workers[i].task = kthread_create(fragment_pin_worker_run, &workers[i],
					       "user4k-pin-%u", i);
		if (IS_ERR(workers[i].task)) {
			workers[i].task = NULL;
			KUNIT_FAIL(test, "pin worker creation failed");
			return;
		}
		get_task_struct(workers[i].task);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_pin_worker_stop, &workers[i]), 0);
		wake_up_process(workers[i].task);
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&workers[i].ready, 10 * HZ), 0UL);
	}
	for (round = 0; round < 32; round++) {
		struct mm_struct *child = test_vma_mm(test);
		unsigned long addr = TEST_VA + (round % 4) * SZ_4K;
		struct vm_area_struct *cvma;
		struct folio *folio;
		unsigned int succeeded = 0;
		vm_fault_t fault;
		struct user_page_fragment write_fragment;
		LIST_HEAD(folios);

		KUNIT_ASSERT_NOT_NULL(test, child);
		uprobe_start_dup_mmap();
		ret = dup_mmap(child, vma->vm_mm);
		uprobe_end_dup_mmap();
		KUNIT_ASSERT_EQ(test, ret, 0L);
		cvma = test_vma_lookup(child, addr);
		KUNIT_ASSERT_NOT_NULL(test, cvma);
		mmap_read_lock(child);
		fault = handle_mm_fault(cvma, addr, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(child);
		KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
		ret = pin_user_fragments_remote(child, addr, 1, FOLL_WRITE, &write_fragment, 1);
		KUNIT_ASSERT_EQ(test, ret, 1L);
		*((u8 *)folio_address(write_fragment.folio) + write_fragment.offset) = 0xa0 + round;
		release_user_fragments(&write_fragment, 1, true);
		lru_add_drain_all();
		folio = test_fault_folio(vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		ret = isolate_folio_to_list(folio, &folios);
		folio_put(folio);
		if (ret) {
			ret = migrate_pages(&folios, test_migration_alloc, NULL, 0,
					    MIGRATE_ASYNC, MR_SYSCALL, &succeeded);
			putback_movable_pages(&folios);
			KUNIT_EXPECT_TRUE(test, ret == 0 || ret == 1);
			migrated += succeeded;
			busy += ret == 1;
		}
		kunit_release_action(test, user4k_vma_mm_free, child);
	}
	for (i = 0; i < 2; i++) {
		fragment_pin_worker_stop(&workers[i]);
		KUNIT_EXPECT_EQ(test, workers[i].error, 0);
		KUNIT_EXPECT_GE(test, workers[i].checks, 4UL);
	}
	kunit_info(test, "32 fork/COW/exit cycles against two pin workers; migrations=%u not_migrated=%u\n",
		   migrated, busy);
}

#endif

static int test_swap_control(const char *operation)
{
	char *argv[] = { "/init", (char *)operation, NULL };
	char *envp[] = { NULL };

	return call_usermodehelper(argv[0], argv, envp, UMH_WAIT_PROC);
}

static void test_reenable_swap(void *arg)
{
	bool *disabled = arg;

	if (*disabled)
		test_swap_control("--swapon-helper");
}

/* Exclusive slots must remain packed after read swap-in and later writes. */
static void test_user4k_swap_exclusive(struct kunit *test, unsigned int mode)
{
	struct vm_area_struct *vma;
	struct folio *folio;
	struct mm_subpage *held = NULL;
	bool *disabled;
	phys_addr_t restored[TEST_SLOTS];
	swp_entry_t entry;
	vm_fault_t fault;
	unsigned int i;
	int ret, mapped, hold_err = 0;
	pte_t pte;

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	disabled = kunit_kzalloc(test, sizeof(*disabled), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, disabled);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_reenable_swap, disabled), 0);
	vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	mmap_read_lock(vma->vm_mm);
	fault = handle_mm_fault(vma, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(vma->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	lru_add_drain_all();
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	folio_lock(folio);
	for (i = 0; i < TEST_SLOTS; i++)
		memset(folio_address(folio) + i * SZ_4K, 0xc0 + i, SZ_4K);
	test_swap_tags_set(folio);
	if (mode == 5) {
		held = mm_subpage_get_from_phys(PFN_PHYS(folio_pfn(folio)) + 2 * SZ_4K);
		if (held) {
			hold_err = kunit_add_action_or_reset(test, test_subpage_put, held);
			if (hold_err)
				held = NULL;
		}
	}
	ret = folio_alloc_swap(folio);
	entry = folio->swap;
	if (!ret)
		try_to_unmap(folio, TTU_SYNC);
	mapped = folio_mapcount(folio);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, mapped, 0);
	if (mode == 5) {
		KUNIT_ASSERT_EQ(test, hold_err, 0);
		KUNIT_ASSERT_NOT_NULL(test, held);
	}
	for (i = 0; i < TEST_SLOTS; i++) {
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA + i * SZ_4K);
		KUNIT_ASSERT_TRUE(test, softleaf_is_swap(softleaf_from_pte(pte)));
		KUNIT_EXPECT_TRUE(test, pte_swp_exclusive(pte));
	}
	if (mode >= 3) {
		struct mm_struct *child = test_vma_mm(test);

		KUNIT_ASSERT_NOT_NULL(test, child);
		uprobe_start_dup_mmap();
		ret = dup_mmap(child, vma->vm_mm);
		uprobe_end_dup_mmap();
		KUNIT_ASSERT_EQ(test, ret, 0);
		kunit_release_action(test, user4k_vma_mm_free, child);
		for (i = 0; i < TEST_SLOTS; i++) {
			pte = test_fault_entry(test, vma->vm_mm, TEST_VA + i * SZ_4K);
			KUNIT_EXPECT_FALSE(test, pte_swp_exclusive(pte));
			KUNIT_EXPECT_EQ(test, swap_subpage_count(entry, i * SZ_4K), 1U);
		}
	}
	if (mode == 1 || mode == 2 || mode == 4 || mode == 6)
		KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(entry));
	if (mode == 2 || mode == 6) {
		ret = test_swap_control("--swapoff-helper");
		*disabled = !total_swap_pages;
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_TRUE(test, *disabled);
	}
	for (i = 0; i < TEST_SLOTS; i++) {
		struct mm_subpage *slot;
		unsigned long address = TEST_VA + i * SZ_4K;

		mmap_read_lock(vma->vm_mm);
		fault = handle_mm_fault(vma, address, FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(vma->vm_mm);
		KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
		pte = test_fault_entry(test, vma->vm_mm, address);
		KUNIT_ASSERT_TRUE(test, pte_present(pte));
		restored[i] = pte_phys_mm(vma->vm_mm, pte);
		slot = mm_subpage_get_from_phys(restored[i]);
		KUNIT_ASSERT_NOT_NULL(test, slot);
		KUNIT_EXPECT_EQ(test, mm_subpage_is_exclusive(slot), !(held && i == 2));
		KUNIT_EXPECT_EQ(test, restored[i], restored[0] + i * SZ_4K);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(slot)) +
				mm_subpage_offset(slot), 0xc0 + i, SZ_4K), NULL);
		KUNIT_EXPECT_TRUE(test, test_swap_tags_match(mm_subpage_folio(slot), i));
		mm_subpage_put(slot);
	}
	for (i = 0; i < TEST_SLOTS; i++) {
		unsigned long address = TEST_VA + i * SZ_4K;

		mmap_read_lock(vma->vm_mm);
		fault = handle_mm_fault(vma, address, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(vma->vm_mm);
		KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
		pte = test_fault_entry(test, vma->vm_mm, address);
		KUNIT_EXPECT_TRUE(test, pte_write(pte));
		if (held && i == 2) {
			KUNIT_EXPECT_NE(test, pte_phys_mm(vma->vm_mm, pte), restored[i]);
			memset(phys_to_virt(pte_phys_mm(vma->vm_mm, pte)), 0xdd, SZ_4K);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(held)) +
					mm_subpage_offset(held), 0xc2, SZ_4K), NULL);
		} else {
			KUNIT_EXPECT_EQ(test, pte_phys_mm(vma->vm_mm, pte), restored[i]);
		}
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_ANONPAGES), (unsigned long)TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_SWAPENTS), 0UL);
	if (*disabled) {
		ret = test_swap_control("--swapon-helper");
		if (!ret)
			*disabled = false;
		KUNIT_EXPECT_EQ(test, ret, 0);
	}
}

static void user4k_swap_exclusive_cached_test(struct kunit *test)
{
	test_user4k_swap_exclusive(test, 0);
}

static void user4k_swap_exclusive_disk_test(struct kunit *test)
{
	test_user4k_swap_exclusive(test, 1);
}

static void user4k_swap_exclusive_swapoff_test(struct kunit *test)
{
	test_user4k_swap_exclusive(test, 2);
}

static void user4k_swap_reuse_cached_test(struct kunit *test)
{
	test_user4k_swap_exclusive(test, 3);
}

static void user4k_swap_reuse_disk_test(struct kunit *test)
{
	test_user4k_swap_exclusive(test, 4);
}

static void user4k_swap_reuse_get_test(struct kunit *test)
{
	test_user4k_swap_exclusive(test, 5);
}

static void user4k_swap_reuse_swapoff_test(struct kunit *test)
{
	test_user4k_swap_exclusive(test, 6);
}

static void test_user4k_swap_roundtrip(struct kunit *test, unsigned int mode, u8 base)
{
	const bool evict = mode != 0;
	struct vm_area_struct *parent, *cvma;
	struct mm_struct *child;
	struct folio *folio;
	swp_entry_t entry = { 0 };
	unsigned int i, count;
	int err, mapped;
	vm_fault_t fault;
	pte_t pte;

	if (!total_swap_pages) {
		kunit_skip(test, "rerun this suite after enabling the disposable guest swap disk");
		return;
	}
	parent = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, parent);
	mmap_read_lock(parent->vm_mm);
	fault = handle_mm_fault(parent, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(parent->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	lru_add_drain_all();
	folio = test_fault_folio(parent->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	folio_lock(folio);
	for (i = 0; i < TEST_SLOTS; i++)
		memset(folio_address(folio) + i * SZ_4K, base + i, SZ_4K);
	test_swap_tags_set(folio);
	if (mode == 2) {
		LIST_HEAD(folios);

		folio_unlock(folio);
		err = folio_isolate_lru(folio) ? 0 : -EBUSY;
		if (!err)
			list_add(&folio->lru, &folios);
		folio_put(folio);
		/* Let the normal reclaim path allocate swap and unmap all slots. */
		reclaim_pages(&folios);
		mapped = 0;
		for (i = 0; i < TEST_SLOTS; i++) {
			pte = test_fault_entry(test, parent->vm_mm, TEST_VA + i * SZ_4K);
			mapped += pte_present(pte);
		}
		entry = softleaf_from_pte(pte);
	} else {
		err = folio_alloc_swap(folio);
		if (!err) {
			entry = folio->swap;
			try_to_unmap(folio, TTU_SYNC | (evict ? TTU_BATCH_FLUSH : 0));
		}
		mapped = folio_mapcount(folio);
		folio_unlock(folio);
		try_to_unmap_flush();
		folio_put(folio);
	}
	KUNIT_ASSERT_EQ(test, err, 0);
	KUNIT_ASSERT_EQ(test, mapped, 0);
	KUNIT_ASSERT_TRUE(test, softleaf_is_swap(entry));
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(parent->vm_mm, MM_ANONPAGES), 0UL);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(parent->vm_mm, MM_SWAPENTS), (unsigned long)TEST_SLOTS);
	for (i = 0; i < TEST_SLOTS; i++) {
		pte = test_fault_entry(test, parent->vm_mm, TEST_VA + i * SZ_4K);
		KUNIT_ASSERT_FALSE(test, pte_present(pte));
		KUNIT_EXPECT_EQ(test, softleaf_from_pte(pte).val, entry.val);
		KUNIT_EXPECT_EQ(test, pte_swp_subpage_offset(pte), i * (unsigned int)SZ_4K);
		KUNIT_EXPECT_TRUE(test, pte_swp_exclusive(pte));
	}
	/* Include extended counts when a native page has 16 user slots. */
	count = swp_swapcount(entry);
	KUNIT_EXPECT_EQ(test, count, (unsigned int)TEST_SLOTS);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, swap_subpage_count(entry, i * SZ_4K), 1U);
	if (evict)
		KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(entry));

	/* Fork while all slots are nonpresent, preserving each byte offset. */
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	err = dup_mmap(child, parent->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, err, 0);
	cvma = test_vma_lookup(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, cvma);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_SWAPENTS), (unsigned long)TEST_SLOTS);
	/* Include extended counts when a native page has 16 user slots. */
	count = swp_swapcount(entry);
	KUNIT_EXPECT_EQ(test, count, 2U * TEST_SLOTS);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, swap_subpage_count(entry, i * SZ_4K), 2U);
	for (i = 0; i < TEST_SLOTS; i++) {
		unsigned long address = TEST_VA + ((i * 5 + 2) % TEST_SLOTS) * SZ_4K;
		struct mm_subpage *slot;
		spinlock_t *ptl;
		pte_t *ptep;
		bool intact = false;

		pte = test_fault_entry(test, parent->vm_mm, address);
		KUNIT_EXPECT_FALSE(test, pte_swp_exclusive(pte));
		pte = test_fault_entry(test, child, address);
		KUNIT_EXPECT_FALSE(test, pte_swp_exclusive(pte));
		KUNIT_EXPECT_EQ(test, softleaf_from_pte(pte).val, entry.val);
		KUNIT_EXPECT_EQ(test, pte_swp_subpage_offset(pte), ((i * 5 + 2) % TEST_SLOTS) * (unsigned int)SZ_4K);
		mmap_read_lock(parent->vm_mm);
		fault = handle_mm_fault(parent, address, FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(parent->vm_mm);
		KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
		KUNIT_EXPECT_EQ(test, !!(fault & VM_FAULT_MAJOR), evict && !i);
		ptep = test_mm_lookup(parent->vm_mm, address, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, ptep);
		pte = ptep_get(ptep);
		slot = pte_present(pte) ? mm_subpage_get_from_phys(pte_phys_mm(parent->vm_mm, pte)) : NULL;
		if (slot) {
			intact = !memchr_inv(folio_address(mm_subpage_folio(slot)) +
					    mm_subpage_offset(slot), base + ((i * 5 + 2) % TEST_SLOTS), SZ_4K) &&
				test_swap_tags_match(mm_subpage_folio(slot), ((i * 5 + 2) % TEST_SLOTS));
			mm_subpage_put(slot);
		}
		pte_unmap_unlock(ptep, ptl);
		KUNIT_EXPECT_TRUE(test, intact);
		KUNIT_EXPECT_FALSE(test, pte_write(pte));
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(parent->vm_mm, MM_ANONPAGES), (unsigned long)TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(parent->vm_mm, MM_SWAPENTS), 0UL);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, swap_subpage_count(entry, i * SZ_4K), 1U);
	/* A write fault must swap in and then COW just the child's selected slot. */
	mmap_read_lock(child);
	fault = handle_mm_fault(cvma, TEST_VA + SZ_4K, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(child);
	KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
	pte = test_fault_entry(test, child, TEST_VA + SZ_4K);
	KUNIT_EXPECT_TRUE(test, pte_write(pte));
	folio = test_fault_folio(child, TEST_VA + SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	{
		unsigned int offset = pte_phys_mm(child, pte) - PFN_PHYS(folio_pfn(folio));

		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + offset,
						    base + 1, SZ_4K), NULL);
		KUNIT_EXPECT_TRUE(test, test_swap_tags_match_at(folio, offset, 1));
		memset(folio_address(folio) + offset, 0xa7, SZ_4K);
	}
	folio_put(folio);
	folio = test_fault_folio(parent->vm_mm, TEST_VA + SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	{
		pte_t parent_pte = test_fault_entry(test, parent->vm_mm, TEST_VA + SZ_4K);
		unsigned int offset = pte_phys_mm(parent->vm_mm, parent_pte) -
				      PFN_PHYS(folio_pfn(folio));

		KUNIT_EXPECT_NE(test, pte_phys_mm(child, pte),
				pte_phys_mm(parent->vm_mm, parent_pte));
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + offset,
						    base + 1, SZ_4K), NULL);
	}
	folio_put(folio);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_ANONPAGES), 1UL);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_SWAPENTS), TEST_SLOTS - 1UL);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, swap_subpage_count(entry, i * SZ_4K), i == 1 ? 0U : 1U);
	/* Exit with three swapped slots, then tear down the restored parent. */
	kunit_release_action(test, user4k_vma_mm_free, child);
	/* Include extended counts when a native page has 16 user slots. */
	count = swp_swapcount(entry);
	KUNIT_EXPECT_EQ(test, count, 0U);
	for (i = 0; i < TEST_SLOTS; i++)
		KUNIT_EXPECT_EQ(test, swap_subpage_count(entry, i * SZ_4K), 0U);
	kunit_release_action(test, user4k_vma_mm_free, parent->vm_mm);
}

static void user4k_swap_mremap_test(struct kunit *test)
{
	struct mm_struct *mm;
	struct folio *folio;
	unsigned long old = TEST_VA, new = SZ_1G - SZ_4K, mapped, moved;
	pte_t saved[TEST_SLOTS];
	u8 byte = 0x55;
	int ret;
	unsigned int i;

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	if (!user4k_test_enabled && !IS_ENABLED(CONFIG_KASAN)) {
		kunit_skip(test, "borrowed-mm mremap fixture requires hardware opt-in");
		return;
	}
	mm = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	kthread_use_mm(mm);
	mapped = vm_mmap(NULL, old, PAGE_SIZE, PROT_READ | PROT_WRITE,
		MAP_PRIVATE | MAP_ANONYMOUS | MAP_FIXED_NOREPLACE, 0);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, mapped, old);
	KUNIT_ASSERT_EQ(test, access_remote_vm(mm, old, &byte, 1, FOLL_WRITE), 1);
	folio = test_fault_folio(mm, old);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	folio_lock(folio);
	for (i = 0; i < TEST_SLOTS; i++)
		memset(folio_address(folio) + i * SZ_4K, 0x81 + i, SZ_4K);
	test_swap_tags_set(folio);
	ret = folio_alloc_swap(folio);
	if (!ret)
		try_to_unmap(folio, TTU_SYNC);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	for (i = 0; i < TEST_SLOTS; i++) {
		saved[i] = test_fault_entry(test, mm, old + i * SZ_4K);
		KUNIT_ASSERT_TRUE(test, softleaf_is_swap(softleaf_from_pte(saved[i])));
	}
	kthread_use_mm(mm);
	moved = test_mremap(old, PAGE_SIZE, PAGE_SIZE, MREMAP_FIXED | MREMAP_MAYMOVE, new);
	kthread_unuse_mm(mm);
	KUNIT_ASSERT_EQ(test, moved, new);
	KUNIT_EXPECT_EQ(test, mm->committed_user_pages, (unsigned long)TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_SWAPENTS), (unsigned long)TEST_SLOTS);
	for (i = 0; i < TEST_SLOTS; i++) {
		pte_t entry = test_fault_entry(test, mm, new + i * SZ_4K);

		KUNIT_EXPECT_EQ(test, softleaf_from_pte(entry).val, softleaf_from_pte(saved[i]).val);
		KUNIT_EXPECT_EQ(test, pte_swp_subpage_offset(entry), i * (unsigned int)SZ_4K);
	}
	for (i = 0; i < TEST_SLOTS; i++) {
		unsigned int slot = (i + 2) % TEST_SLOTS;
		unsigned long addr = new + slot * SZ_4K;

		KUNIT_ASSERT_EQ(test, access_remote_vm(mm, addr, &byte, 1, 0), 1);
		KUNIT_EXPECT_EQ(test, byte, (u8)(0x81 + slot));
		folio = test_fault_folio(mm, addr);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		KUNIT_EXPECT_TRUE(test, test_swap_tags_match(folio, slot));
		folio_put(folio);
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_SWAPENTS), 0UL);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), (unsigned long)TEST_SLOTS);
}

static void user4k_swap_pin_test(struct kunit *test)
{
	struct vm_area_struct *vma;
	struct fragment_test_pins *pins;
	struct folio *folio;
	phys_addr_t pinned_phys;
	unsigned int i;
	vm_fault_t fault;
	long ret;
	pte_t pte;

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	pins = kunit_kzalloc(test, sizeof(*pins), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pins);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_test_unpin, pins), 0);
	ret = fragment_test_pin(pins, vma->vm_mm, TEST_VA, PAGE_SIZE, FOLL_WRITE, 4);
	KUNIT_ASSERT_EQ(test, ret, 4L);
	for (i = 0; i < 4; i++)
		memset(folio_address(pins->fragments[i].folio) + pins->fragments[i].offset,
		       0xb1 + i, pins->fragments[i].length);
	test_swap_tags_set(pins->fragments[0].folio);
	fragment_test_unpin(pins);
	ret = fragment_test_pin(pins, vma->vm_mm, TEST_VA + SZ_4K, SZ_4K, FOLL_WRITE, 1);
	KUNIT_ASSERT_EQ(test, ret, 1L);
	pinned_phys = mm_subpage_phys(pins->fragments[0].subpage);
	folio = pins->fragments[0].folio;
	folio_lock(folio);
	ret = folio_alloc_swap(folio);
	if (!ret)
		try_to_unmap(folio, TTU_SYNC);
	folio_unlock(folio);
	KUNIT_ASSERT_EQ(test, ret, 0L);
	pte = test_fault_entry(test, vma->vm_mm, TEST_VA + SZ_4K);
	KUNIT_EXPECT_TRUE(test, pte_present(pte));
	KUNIT_EXPECT_EQ(test, pte_phys_mm(vma->vm_mm, pte), pinned_phys);
	KUNIT_EXPECT_TRUE(test, mm_subpage_is_exclusive(pins->fragments[0].subpage));
	KUNIT_EXPECT_EQ(test, mm_subpage_pincount(pins->fragments[0].subpage), 1U);
	memset(folio_address(folio) + SZ_4K, 0xd2, SZ_4K);
	fragment_test_unpin(pins);
	/* Once released, the formerly pinned PTE can be swapped normally. */
	folio = test_fault_folio(vma->vm_mm, TEST_VA + SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	folio_lock(folio);
	try_to_unmap(folio, TTU_SYNC);
	ret = folio_mapcount(folio);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0L);
	for (i = 0; i < 4; i++) {
		unsigned long addr = TEST_VA + i * SZ_4K;

		mmap_read_lock(vma->vm_mm);
		fault = handle_mm_fault(vma, addr, FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(vma->vm_mm);
		KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
		folio = test_fault_folio(vma->vm_mm, addr);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + i * SZ_4K,
						    i == 1 ? 0xd2 : 0xb1 + i, SZ_4K), NULL);
		KUNIT_EXPECT_TRUE(test, test_swap_tags_match(folio, i));
		folio_put(folio);
	}
}

#ifdef CONFIG_MIGRATION
static void user4k_swap_migration_test(struct kunit *test)
{
	struct vm_area_struct *vma, *cvma;
	struct mm_struct *child;
	struct folio *folio;
	phys_addr_t old_phys;
	unsigned int i, side, succeeded = 0;
	vm_fault_t fault;
	int ret;
	LIST_HEAD(folios);

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	vma = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	mmap_read_lock(vma->vm_mm);
	fault = handle_mm_fault(vma, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(vma->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	old_phys = PFN_PHYS(folio_pfn(folio));
	folio_lock(folio);
	for (i = 0; i < TEST_SLOTS; i++)
		memset(folio_address(folio) + i * SZ_4K, 0xa1 + i, SZ_4K);
	test_swap_tags_set(folio);
	ret = folio_alloc_swap(folio);
	if (!ret)
		try_to_unmap(folio, TTU_SYNC);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, vma->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	cvma = test_vma_lookup(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, cvma);
	mmap_read_lock(child);
	fault = handle_mm_fault(cvma, TEST_VA + SZ_4K, FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(child);
	KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
	lru_add_drain_all();
	folio = test_fault_folio(child, TEST_VA + SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	KUNIT_EXPECT_TRUE(test, folio_test_swapcache(folio));
	ret = isolate_folio_to_list(folio, &folios) ? 0 : -EBUSY;
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = migrate_pages(&folios, test_migration_alloc, NULL, 0,
			    MIGRATE_SYNC, MR_SYSCALL, &succeeded);
	putback_movable_pages(&folios);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, succeeded, 1U);
	/* One present slot and seven swap entries must all follow the new cache folio. */
	for (side = 0; side < 2; side++) {
		struct vm_area_struct *area = side ? cvma : vma;
		struct mm_struct *mm = area->vm_mm;

		for (i = 0; i < TEST_SLOTS; i++) {
			unsigned long addr = TEST_VA + i * SZ_4K;

			mmap_read_lock(mm);
			fault = handle_mm_fault(area, addr, FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(mm);
			KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
			folio = test_fault_folio(mm, addr);
			KUNIT_ASSERT_NOT_NULL(test, folio);
			KUNIT_EXPECT_NE(test, PFN_PHYS(folio_pfn(folio)), old_phys);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + i * SZ_4K,
							    0xa1 + i, SZ_4K), NULL);
			KUNIT_EXPECT_TRUE(test, test_swap_tags_match(folio, i));
			folio_put(folio);
		}
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_SWAPENTS), 0UL);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), (unsigned long)TEST_SLOTS);
	}
}
#endif

static void user4k_swap_cached_test(struct kunit *test)
{
	test_user4k_swap_roundtrip(test, 0, 0x51);
}

static void user4k_swap_disk_test(struct kunit *test)
{
	test_user4k_swap_roundtrip(test, 1, 0x61);
}

static void user4k_swap_reclaim_test(struct kunit *test)
{
	test_user4k_swap_roundtrip(test, 2, 0x71);
}

static void user4k_swap_stress_test(struct kunit *test)
{
	unsigned int i;

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	for (i = 0; i < 32; i++)
		test_user4k_swap_roundtrip(test, i % 3, 0x21 + i * 5);
	kunit_info(test, "32 swap/fork/COW/exit cycles with distinct data patterns\n");
}

static void user4k_swapoff_test(struct kunit *test)
{
	struct vm_area_struct *parent, *cvma;
	struct mm_struct *child;
	struct folio *folio;
	bool *disabled;
	vm_fault_t fault;
	unsigned int i, side;
	swp_entry_t entry;
	int ret;
	pte_t pte;
	LIST_HEAD(folios);

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	disabled = kunit_kzalloc(test, sizeof(*disabled), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, disabled);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_reenable_swap, disabled), 0);
	parent = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, parent);
	mmap_read_lock(parent->vm_mm);
	fault = handle_mm_fault(parent, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(parent->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	lru_add_drain_all();
	folio = test_fault_folio(parent->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	for (i = 0; i < TEST_SLOTS; i++)
		memset(folio_address(folio) + i * SZ_4K, 0x91 + i, SZ_4K);
	test_swap_tags_set(folio);
	ret = folio_isolate_lru(folio) ? 0 : -EBUSY;
	if (!ret)
		list_add(&folio->lru, &folios);
	folio_put(folio);
	reclaim_pages(&folios);
	KUNIT_ASSERT_EQ(test, ret, 0);
	pte = test_fault_entry(test, parent->vm_mm, TEST_VA);
	entry = softleaf_from_pte(pte);
	KUNIT_ASSERT_TRUE(test, softleaf_is_swap(entry));
	KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(entry));
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(parent->vm_mm, MM_SWAPENTS), (unsigned long)TEST_SLOTS);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_SWAPENTS), (unsigned long)TEST_SLOTS);
	ret = test_swap_control("--swapoff-helper");
	*disabled = !total_swap_pages;
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_TRUE(test, *disabled);
	for (side = 0; side < 2; side++) {
		struct mm_struct *mm = side ? child : parent->vm_mm;

		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_SWAPENTS), 0UL);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), (unsigned long)TEST_SLOTS);
		for (i = 0; i < TEST_SLOTS; i++) {
			pte = test_fault_entry(test, mm, TEST_VA + i * SZ_4K);
			KUNIT_ASSERT_TRUE(test, pte_present(pte));
			KUNIT_EXPECT_FALSE(test, pte_write(pte));
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte) & ~PAGE_MASK,
					i * (unsigned long)SZ_4K);
			folio = test_fault_folio(mm, TEST_VA + i * SZ_4K);
			KUNIT_ASSERT_NOT_NULL(test, folio);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + i * SZ_4K,
							    0x91 + i, SZ_4K), NULL);
			KUNIT_EXPECT_TRUE(test, test_swap_tags_match(folio, i));
			folio_put(folio);
		}
	}
	cvma = test_vma_lookup(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, cvma);
	mmap_read_lock(child);
	fault = handle_mm_fault(cvma, TEST_VA + 3 * SZ_4K,
				FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(child);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	pte = test_fault_entry(test, child, TEST_VA + 3 * SZ_4K);
	KUNIT_EXPECT_TRUE(test, pte_write(pte));
	KUNIT_EXPECT_NE(test, pte_phys_mm(child, pte), pte_phys_mm(parent->vm_mm,
			test_fault_entry(test, parent->vm_mm, TEST_VA + 3 * SZ_4K)));
	ret = test_swap_control("--swapon-helper");
	if (!ret)
		*disabled = false;
	KUNIT_EXPECT_EQ(test, ret, 0);
}

static void user4k_swapoff_boundaries_test(struct kunit *test)
{
	const unsigned long boundaries[] = { SZ_2M, SZ_1G, 1UL << 39 };
	bool *disabled;
	unsigned int boundary;

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	disabled = kunit_kzalloc(test, sizeof(*disabled), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, disabled);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_reenable_swap, disabled), 0);
	for (boundary = 0; boundary < ARRAY_SIZE(boundaries); boundary++) {
		unsigned long start = boundaries[boundary] - SZ_4K;
		struct vm_area_struct *parent = test_fault_vma(test, start, 2 * SZ_4K);
		struct mm_struct *child;
		unsigned int i, side;
		unsigned int offsets[2];
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, parent);
		for (i = 0; i < 2; i++) {
			unsigned long addr = start + i * SZ_4K;
			unsigned int offset;
			struct folio *folio;
			vm_fault_t fault;
			swp_entry_t entry;
			LIST_HEAD(folios);

			mmap_read_lock(parent->vm_mm);
			fault = handle_mm_fault(parent, addr, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(parent->vm_mm);
			KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
			offset = offset_in_page(pte_phys_mm(parent->vm_mm,
					test_fault_entry(test, parent->vm_mm, addr)));
			offsets[i] = offset;
			lru_add_drain_all();
			folio = test_fault_folio(parent->vm_mm, addr);
			KUNIT_ASSERT_NOT_NULL(test, folio);
			memset(folio_address(folio) + offset, 0xb0 + boundary * 2 + i, SZ_4K);
			test_swap_tags_set(folio);
			ret = folio_isolate_lru(folio) ? 0 : -EBUSY;
			if (!ret)
				list_add(&folio->lru, &folios);
			folio_put(folio);
			reclaim_pages(&folios);
			KUNIT_ASSERT_EQ(test, ret, 0);
			entry = softleaf_from_pte(test_fault_entry(test, parent->vm_mm, addr));
			KUNIT_ASSERT_TRUE(test, softleaf_is_swap(entry));
			KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(entry));
		}
		child = test_vma_mm(test);
		KUNIT_ASSERT_NOT_NULL(test, child);
		uprobe_start_dup_mmap();
		ret = dup_mmap(child, parent->vm_mm);
		uprobe_end_dup_mmap();
		KUNIT_ASSERT_EQ(test, ret, 0);
		ret = test_swap_control("--swapoff-helper");
		*disabled = !total_swap_pages;
		KUNIT_ASSERT_EQ(test, ret, 0);
		for (side = 0; side < 2; side++) {
			struct mm_struct *mm = side ? child : parent->vm_mm;

			KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_SWAPENTS), 0UL);
			KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 2UL);
			for (i = 0; i < 2; i++) {
				unsigned long addr = start + i * SZ_4K;
				unsigned int offset = offsets[i];
				pte_t pte = test_fault_entry(test, mm, addr);
				struct folio *folio;

				KUNIT_ASSERT_TRUE(test, pte_present(pte));
				KUNIT_EXPECT_FALSE(test, pte_write(pte));
				KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, pte) & ~PAGE_MASK,
						(unsigned long)offset);
				folio = test_fault_folio(mm, addr);
				KUNIT_ASSERT_NOT_NULL(test, folio);
				KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + offset,
								    0xb0 + boundary * 2 + i, SZ_4K), NULL);
				KUNIT_EXPECT_TRUE(test, test_swap_tags_match(folio, offset / SZ_4K));
				folio_put(folio);
			}
		}
		ret = test_swap_control("--swapon-helper");
		if (!ret)
			*disabled = false;
		KUNIT_ASSERT_EQ(test, ret, 0);
		kunit_release_action(test, user4k_vma_mm_free, child);
		kunit_release_action(test, user4k_vma_mm_free, parent->vm_mm);
	}
}

struct live_swap_reader {
	struct mm_struct *mm;
	struct task_struct *task;
	struct completion ready, resume, entered, done;
	unsigned int id, checks;
	int error;
};

static int live_swap_reader_run(void *arg)
{
	struct live_swap_reader *r = arg;
	unsigned int phase, i;

	kthread_use_mm(r->mm);
	for (phase = 0; phase < 2; phase++) {
		if (phase)
			complete(&r->entered);
		for (i = 0; i < 4; i++) {
			u8 value = 0;
			void __user *addr = (void __user *)(TEST_VA + i * SZ_4K + 64 + r->id);

			/* Resolve AF, swap and COW faults through the real uaccess path. */
			if (copy_from_user(&value, addr, 1) || value != 0xd1 + i)
				r->error = phase ? 2 : 1;
			r->checks++;
		}
		if (!phase) {
			complete(&r->ready);
			if (!wait_for_completion_timeout(&r->resume, 15 * HZ)) {
				r->error = 3;
				goto out;
			}
		}
	}
	{
		u8 value = 0xe0 + r->id;
		void __user *addr = (void __user *)(TEST_VA + SZ_4K + 64 + r->id);

		if (copy_to_user(addr, &value, 1))
			r->error = 4;
		r->checks++;
	}
out:
	kthread_unuse_mm(r->mm);
	complete(&r->done);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);
	return 0;
}

static void live_swap_reader_stop(void *arg)
{
	struct live_swap_reader *r = arg;

	if (r->task) {
		complete_all(&r->resume);
		kthread_stop(r->task);
		r->task = NULL;
	}
}

static void test_live_fault_roundtrip(struct kunit *test, unsigned int mode)
{
	bool migration = mode != 0;
	struct vm_area_struct *parent;
	struct mm_struct *child;
	struct live_swap_reader *readers;
	struct folio *folio;
	vm_fault_t fault;
	unsigned int i, cpu;
	swp_entry_t entry;
	u64 tcr = read_sysreg(tcr_el1);
	int ret;
	LIST_HEAD(folios);

	if ((!migration && !total_swap_pages) || !user4k_test_enabled || num_online_cpus() < 2 ||
	    !system_supports_4kb_granule() || system_supports_cnp() ||
	    cpus_have_final_cap(ARM64_HAS_S1PIE) || cpus_have_final_cap(ARM64_HAS_S1POE) ||
	    (tcr & (TCR_EL1_DS | TCR_EL1_A1 | TCR_EL1_T0SZ_MASK)) !=
		   (TCR_EL1_A1 | TCR_T0SZ(48))) {
		kunit_skip(test, "requires two CPUs, compatible opt-in 4K TTBR0 controls, and swap for reclaim");
		return;
	}
	parent = test_fault_vma(test, TEST_VA, PAGE_SIZE);
	KUNIT_ASSERT_NOT_NULL(test, parent);
	mmap_read_lock(parent->vm_mm);
	fault = handle_mm_fault(parent, TEST_VA, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(parent->vm_mm);
	KUNIT_ASSERT_EQ(test, fault, (vm_fault_t)0);
	folio = test_fault_folio(parent->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	for (i = 0; i < 4; i++)
		memset(folio_address(folio) + i * SZ_4K, 0xd1 + i, SZ_4K);
	folio_put(folio);
	child = test_vma_mm(test);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent->vm_mm);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	readers = kunit_kcalloc(test, 2, sizeof(*readers), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, readers);
	cpu = cpumask_first(cpu_online_mask);
	for (i = 0; i < 2; i++) {
		readers[i].mm = child;
		readers[i].id = i;
		init_completion(&readers[i].ready);
		init_completion(&readers[i].resume);
		init_completion(&readers[i].entered);
		init_completion(&readers[i].done);
		readers[i].task = kthread_create(live_swap_reader_run, &readers[i],
						"user4k-swap-read-%u", i);
		if (IS_ERR(readers[i].task)) {
			readers[i].task = NULL;
			KUNIT_FAIL(test, "live swap worker creation failed");
			return;
		}
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, live_swap_reader_stop,
							      &readers[i]), 0);
		kthread_bind(readers[i].task, cpu);
		cpu = cpumask_next(cpu, cpu_online_mask);
		wake_up_process(readers[i].task);
	}
	for (i = 0; i < 2; i++)
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&readers[i].ready, 10 * HZ), 0UL);
	lru_add_drain_all();
	folio = test_fault_folio(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	if (mode == 2) {
#ifdef CONFIG_MIGRATION
		struct folio *dst = folio_alloc(GFP_KERNEL, 0);
		bool entered = true, blocked = true, unmapped = false;

		if (!dst) {
			folio_put(folio);
			KUNIT_FAIL(test, "migration waiter destination allocation failed");
			return;
		}
		folio_lock(folio);
		folio_lock(dst);
		ret = mm_subpage_migrate_prepare(folio, dst);
		if (!ret) {
			try_to_migrate(folio, TTU_SYNC);
			unmapped = !folio_mapped(folio);
			for (i = 0; i < 2; i++)
				complete(&readers[i].resume);
			for (i = 0; i < 2; i++) {
				entered &= !!wait_for_completion_timeout(&readers[i].entered, HZ);
				if (wait_for_completion_timeout(&readers[i].done, max(1UL, HZ / 20UL))) {
					blocked = false;
					complete(&readers[i].done);
				}
			}
			remove_migration_ptes(folio, folio, 0);
			mm_subpage_migrate_finish(folio);
			mm_subpage_migrate_finish(dst);
		}
		folio_unlock(dst);
		folio_unlock(folio);
		folio_put(dst);
		folio_put(folio);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_TRUE(test, entered);
		KUNIT_EXPECT_TRUE(test, blocked);
		KUNIT_EXPECT_TRUE(test, unmapped);
#endif
	} else if (migration) {
#ifdef CONFIG_MIGRATION
		unsigned int succeeded = 0;
		phys_addr_t old_phys = PFN_PHYS(folio_pfn(folio));

		ret = isolate_folio_to_list(folio, &folios) ? 0 : -EBUSY;
		folio_put(folio);
		KUNIT_ASSERT_EQ(test, ret, 0);
		ret = migrate_pages(&folios, test_migration_alloc, NULL, 0,
				    MIGRATE_ASYNC, MR_SYSCALL, &succeeded);
		putback_movable_pages(&folios);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, succeeded, 1U);
		KUNIT_EXPECT_NE(test, pte_phys_mm(child, test_fault_entry(test, child, TEST_VA)), old_phys);
#endif
	} else {
		ret = folio_isolate_lru(folio) ? 0 : -EBUSY;
		if (!ret)
			list_add(&folio->lru, &folios);
		folio_put(folio);
		reclaim_pages(&folios);
		KUNIT_ASSERT_EQ(test, ret, 0);
		entry = softleaf_from_pte(test_fault_entry(test, child, TEST_VA));
		KUNIT_ASSERT_TRUE(test, softleaf_is_swap(entry));
		KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(entry));
	}
	for (i = 0; i < 2; i++)
		complete(&readers[i].resume);
	for (i = 0; i < 2; i++) {
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&readers[i].done, 15 * HZ), 0UL);
		live_swap_reader_stop(&readers[i]);
		KUNIT_EXPECT_EQ(test, readers[i].error, 0);
		KUNIT_EXPECT_EQ(test, readers[i].checks, 9U);
	}
	folio = test_fault_folio(child, TEST_VA + SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	{
		pte_t pte = test_fault_entry(test, child, TEST_VA + SZ_4K);
		unsigned long offset = pte_phys_mm(child, pte) - PFN_PHYS(folio_pfn(folio));

		for (i = 0; i < 2; i++)
			KUNIT_EXPECT_EQ(test, *((u8 *)folio_address(folio) + offset + 64 + i),
					(u8)(0xe0 + i));
	}
	folio_put(folio);
	mmap_read_lock(parent->vm_mm);
	fault = handle_mm_fault(parent, TEST_VA + SZ_4K, FAULT_FLAG_REMOTE, NULL);
	mmap_read_unlock(parent->vm_mm);
	KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
	folio = test_fault_folio(parent->vm_mm, TEST_VA + SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K, 0xd2, SZ_4K), NULL);
	folio_put(folio);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_SWAPENTS), 0UL);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_ANONPAGES), 4UL);
	kunit_info(test, "two CPUs access through uaccess after %s, then write one child slot\n",
		   mode == 2 ? "migration-entry rollback" : migration ? "migration" : "reclaim");
}

static void user4k_live_swap_fault_test(struct kunit *test)
{
	test_live_fault_roundtrip(test, 0);
}

#ifdef CONFIG_MIGRATION
static void user4k_live_migration_test(struct kunit *test)
{
	test_live_fault_roundtrip(test, 1);
}

static void user4k_migration_wait_test(struct kunit *test)
{
	test_live_fault_roundtrip(test, 2);
}
#endif

/* Transfer PTE ownership to normal mm teardown, which also handles swap. */
static int anon_slot_fixture_handoff(struct kunit *test, struct anon_slot_fixture *f)
{
	struct mm_struct *mm = f->mm;

	f->mm = NULL;
	f->vma = NULL;
	memset(f->slots, 0, sizeof(f->slots));
	memset(f->mapped, 0, sizeof(f->mapped));
	return kunit_add_action_or_reset(test, user4k_vma_mm_free, mm);
}

static void test_nonlinear_swap(struct kunit *test, bool evict)
{
	unsigned int first;

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	for (first = 0; first < 2; first++) {
		struct anon_slot_fixture *a = anon_slot_fixture_create(test);
		struct anon_slot_fixture *b = anon_slot_fixture_create(test);
		struct vm_area_struct *vmas[2];
		struct mm_struct *child;
		struct mm_subpage *slot;
		struct folio *folio;
		swp_entry_t entry = { };
		unsigned int side;
		vm_fault_t fault;
		int err, mapped;

		KUNIT_ASSERT_NOT_NULL(test, a);
		KUNIT_ASSERT_NOT_NULL(test, b);
		vmas[0] = a->vma;
		vmas[1] = b->vma;
		slot = mm_subpage_alloc_at(a->pool, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
		folio = mm_subpage_folio(slot);
		err = mem_cgroup_charge(folio, a->mm, GFP_KERNEL);
		if (err) {
			mm_subpage_put(slot);
			KUNIT_FAIL(test, "cannot charge swap fixture: %d", err);
			return;
		}
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(a, 0, slot), 0);
		folio_lock(folio);
		mm_subpage_set_exclusive(slot);
		folio_unlock(folio);
		slot = mm_subpage_alloc_at(a->pool, 3 * SZ_4K);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, slot);
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_map(b, 1, slot), 0);
		folio_lock(folio);
		mm_subpage_set_exclusive(slot);
		folio_unlock(folio);
		memset(folio_address(folio), 0x5b, SZ_4K);
		memset(folio_address(folio) + 3 * SZ_4K, 0xd3, SZ_4K);
		test_swap_tags_set(folio);
		folio_mark_dirty(folio);
		folio_add_lru(folio);
		lru_add_drain_all();
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_handoff(test, a), 0);
		KUNIT_ASSERT_EQ(test, anon_slot_fixture_handoff(test, b), 0);
		folio_get(folio);
		folio_lock(folio);
		err = folio_alloc_swap(folio);
		if (!err) {
			entry = folio->swap;
			try_to_unmap(folio, TTU_SYNC);
		}
		mapped = folio_mapcount(folio);
		folio_unlock(folio);
		folio_put(folio);
		KUNIT_ASSERT_EQ(test, err, 0);
		KUNIT_ASSERT_EQ(test, mapped, 0);
		for (side = 0; side < 2; side++) {
			pte_t pte = test_fault_entry(test, vmas[side]->vm_mm,
						    TEST_VA + side * SZ_4K);

			KUNIT_EXPECT_EQ(test, softleaf_from_pte(pte).val, entry.val);
			KUNIT_EXPECT_EQ(test, pte_swp_subpage_offset(pte), side ? 3U * SZ_4K : 0U);
			KUNIT_EXPECT_EQ(test, swap_subpage_count(entry, side ? 3 * SZ_4K : 0), 1U);
		}
		if (evict)
			KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(entry));
		/* Preserve physical offsets when a nonpresent nonlinear slot is forked. */
		child = test_vma_mm(test);
		KUNIT_ASSERT_NOT_NULL(test, child);
		uprobe_start_dup_mmap();
		err = dup_mmap(child, vmas[1]->vm_mm);
		uprobe_end_dup_mmap();
		KUNIT_ASSERT_EQ(test, err, 0);
		for (side = 0; side < 2; side++) {
			unsigned int which = side ^ first, physical = which ? 3 : 0;
			struct vm_area_struct *vma = vmas[which];
			unsigned long address = TEST_VA + which * SZ_4K;
			pte_t pte;

			mmap_read_lock(vma->vm_mm);
			fault = handle_mm_fault(vma, address, FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(vma->vm_mm);
			KUNIT_ASSERT_EQ_MSG(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY),
				(vm_fault_t)0, "evict=%d first=%u side=%u", evict, first, which);
			KUNIT_EXPECT_EQ(test, !!(fault & VM_FAULT_MAJOR), evict && !side);
			pte = test_fault_entry(test, vma->vm_mm, address);
			slot = mm_subpage_get_from_phys(pte_phys_mm(vma->vm_mm, pte));
			KUNIT_ASSERT_NOT_NULL(test, slot);
			KUNIT_EXPECT_EQ(test, mm_subpage_offset(slot), physical * (unsigned int)SZ_4K);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(slot)) +
				mm_subpage_offset(slot), which ? 0xd3 : 0x5b, SZ_4K), NULL);
			KUNIT_EXPECT_TRUE(test, test_swap_tags_match(mm_subpage_folio(slot), physical));
			mm_subpage_put(slot);
		}
		/* A child's write must copy only its slot at the new virtual offset. */
		{
			struct vm_area_struct *vma = test_vma_lookup(child, TEST_VA + SZ_4K);
			pte_t pte;

			KUNIT_ASSERT_NOT_NULL(test, vma);
			mmap_read_lock(child);
			fault = handle_mm_fault(vma, TEST_VA + SZ_4K,
						FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(child);
			KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
			pte = test_fault_entry(test, child, TEST_VA + SZ_4K);
			KUNIT_EXPECT_TRUE(test, pte_write(pte));
			slot = mm_subpage_get_from_phys(pte_phys_mm(child, pte));
			KUNIT_ASSERT_NOT_NULL(test, slot);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(slot)) +
				mm_subpage_offset(slot), 0xd3, SZ_4K), NULL);
			memset(folio_address(mm_subpage_folio(slot)) + mm_subpage_offset(slot), 0x91, SZ_4K);
			mm_subpage_put(slot);
			pte = test_fault_entry(test, vmas[1]->vm_mm, TEST_VA + SZ_4K);
			slot = mm_subpage_get_from_phys(pte_phys_mm(vmas[1]->vm_mm, pte));
			KUNIT_ASSERT_NOT_NULL(test, slot);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(mm_subpage_folio(slot)) +
				mm_subpage_offset(slot), 0xd3, SZ_4K), NULL);
			mm_subpage_put(slot);
		}
		kunit_release_action(test, user4k_vma_mm_free, child);
		kunit_release_action(test, user4k_vma_mm_free, vmas[1]->vm_mm);
		kunit_release_action(test, user4k_vma_mm_free, vmas[0]->vm_mm);
	}
}

static void user4k_nonlinear_swap_cached_test(struct kunit *test)
{
	test_nonlinear_swap(test, false);
}

static void user4k_nonlinear_swap_disk_test(struct kunit *test)
{
	test_nonlinear_swap(test, true);
}

#ifdef CONFIG_USERFAULTFD
/* The context is attached only during the call; ordinary mm cleanup owns the VMAs. */
static ssize_t test_uffd_move(struct vm_area_struct *dst, unsigned long dst_addr,
			    unsigned long src_addr, unsigned long size)
{
	struct userfaultfd_ctx ctx = { .mm = dst->vm_mm };
	ssize_t ret;

	init_rwsem(&ctx.map_changing_lock);
	atomic_set(&ctx.mmap_changing, 0);
	mmap_write_lock(ctx.mm);
	vma_start_write(dst);
	dst->vm_userfaultfd_ctx.ctx = &ctx;
	mmap_write_unlock(ctx.mm);
	ret = move_pages(&ctx, dst_addr, src_addr, size, 0);
	mmap_write_lock(ctx.mm);
	vma_start_write(dst);
	dst->vm_userfaultfd_ctx.ctx = NULL;
	mmap_write_unlock(ctx.mm);
	return ret;
}

#ifdef CONFIG_MIGRATION
struct move_migration_worker {
	struct userfaultfd_ctx ctx;
	struct vm_area_struct *vma[2];
	struct task_struct *task;
	struct completion ready;
	unsigned long address[2], size;
	unsigned long moves, busy;
	int error;
};

static int move_migration_worker_run(void *arg)
{
	struct move_migration_worker *w = arg;

	while (!kthread_should_stop()) {
		unsigned int side = w->moves & 1;
		ssize_t ret = move_pages(&w->ctx, w->address[side ^ 1],
					w->address[side], w->size, 0);

		if (ret == w->size)
			WRITE_ONCE(w->moves, w->moves + 1);
		else if (ret == -EAGAIN || ret == -EBUSY)
			w->busy++;
		else
			w->error = ret < 0 ? ret : -EIO;
		if (w->moves || w->error)
			complete_all(&w->ready);
		if (w->error) {
			/* Keep the task valid until the owner joins it. */
			set_current_state(TASK_INTERRUPTIBLE);
			if (!kthread_should_stop())
				schedule();
			__set_current_state(TASK_RUNNING);
		}
		/* Leave bounded gaps so migration can complete between contended moves. */
		if (ret == w->size)
			usleep_range(1000, 2000);
		else
			cond_resched();
	}
	return 0;
}

static void move_migration_worker_stop(void *arg)
{
	struct move_migration_worker *w = arg;
	unsigned int i;

	if (w->task) {
		kthread_stop(w->task);
		w->task = NULL;
	}
	mmap_write_lock(w->ctx.mm);
	for (i = 0; i < 2; i++) {
		vma_start_write(w->vma[i]);
		w->vma[i]->vm_userfaultfd_ctx.ctx = NULL;
	}
	mmap_write_unlock(w->ctx.mm);
}

static void user4k_move_migration_race_test(struct kunit *test)
{
	unsigned int shift;

	for (shift = 12; shift < PAGE_SHIFT; shift += 2) {
		struct mm_struct *mm = test_vma_mm_granule(test, shift);
		struct move_migration_worker *w;
		unsigned long size = 1UL << shift, i, initial_moves;
		unsigned int round, migrated = 0, failed = 0;
		pte_t pte;
		vm_fault_t fault;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		w = kunit_kzalloc(test, sizeof(*w), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, w);
		w->ctx.mm = mm;
		init_rwsem(&w->ctx.map_changing_lock);
		atomic_set(&w->ctx.mmap_changing, 0);
		init_completion(&w->ready);
		w->size = size;
		w->address[0] = TEST_VA + 3 * size;
		w->address[1] = TEST_VA + SZ_128M + size;
		w->vma[0] = test_fault_vma_in_mm(mm, TEST_VA, PAGE_SIZE);
		w->vma[1] = test_fault_vma_in_mm(mm, TEST_VA + SZ_128M, PAGE_SIZE);
		KUNIT_ASSERT_NOT_NULL(test, w->vma[0]);
		KUNIT_ASSERT_NOT_NULL(test, w->vma[1]);
		for (i = 0; i < PAGE_SIZE; i += size) {
			mmap_read_lock(mm);
			fault = handle_mm_fault(w->vma[0], TEST_VA + i,
					FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(mm);
			KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
			pte = test_fault_entry(test, mm, TEST_VA + i);
			memset(phys_to_virt(pte_phys_mm(mm, pte)), 0x60 + i / size, size);
		}
		/* Both directions are registered before starting the concurrent mover. */
		mmap_write_lock(mm);
		for (i = 0; i < 2; i++) {
			vma_start_write(w->vma[i]);
			w->vma[i]->vm_userfaultfd_ctx.ctx = &w->ctx;
		}
		mmap_write_unlock(mm);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, move_migration_worker_stop, w), 0);
		w->task = kthread_create(move_migration_worker_run, w, "user4k-move");
		if (IS_ERR(w->task)) {
			w->task = NULL;
			KUNIT_FAIL(test, "MOVE worker creation failed");
			return;
		}
		if (num_online_cpus() > 1)
			kthread_bind(w->task, cpumask_last(cpu_online_mask));
		wake_up_process(w->task);
		KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&w->ready, 10 * HZ), 0UL);
		initial_moves = READ_ONCE(w->moves);
		lru_add_drain_all();
		for (round = 0; round < 64; round++) {
			struct folio *folio;
			unsigned int succeeded = 0;
			bool isolated;
			int ret;
			LIST_HEAD(folios);

			lru_add_drain_all();
			folio = test_fault_folio(mm, TEST_VA);
			KUNIT_ASSERT_NOT_NULL(test, folio);
			isolated = isolate_folio_to_list(folio, &folios);
			folio_put(folio);
			KUNIT_ASSERT_TRUE_MSG(test, isolated, "round=%u", round);
			ret = migrate_pages(&folios, test_migration_alloc, NULL, 0,
					    MIGRATE_SYNC, MR_SYSCALL, &succeeded);
			putback_movable_pages(&folios);
			KUNIT_EXPECT_GE(test, ret, 0);
			migrated += succeeded;
			failed += ret > 0;
			cond_resched();
		}
		kunit_release_action(test, move_migration_worker_stop, w);
		kunit_info(test, "%uK MOVE/migration: moves=%lu retries=%lu migrated=%u busy=%u\n",
			   1U << (shift - 10), w->moves, w->busy, migrated, failed);
		KUNIT_EXPECT_EQ(test, w->error, 0);
		KUNIT_EXPECT_GT(test, w->moves, initial_moves);
		KUNIT_EXPECT_GT(test, migrated, 0U);
		for (i = 0; i < PAGE_SIZE; i += size) {
			unsigned long addr = i == 3 * size ? w->address[w->moves & 1] : TEST_VA + i;

			pte = test_fault_entry(test, mm, addr);
			KUNIT_ASSERT_TRUE(test, pte_present(pte));
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(mm, pte)),
							  0x60 + i / size, size), NULL);
		}
		KUNIT_EXPECT_TRUE(test, pte_none(test_fault_entry(test, mm, w->address[(w->moves & 1) ^ 1])));
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}

#endif

static void user4k_move_swap_get_test(struct kunit *test)
{
	unsigned int variant;

	if (!total_swap_pages) {
		kunit_skip(test, "requires the disposable guest swap runner");
		return;
	}
	for (variant = 0; variant < PAGE_SHIFT - 12; variant++) {
		unsigned int shift = 12 + (variant / 2) * 2;
		bool forked = variant & 1;
		unsigned long size = 1UL << shift;
		unsigned long src_addr = TEST_VA + 3 * size;
		unsigned long dst_addr = TEST_VA + SZ_128M + size;
		struct mm_struct *mm = test_vma_mm_granule(test, shift);
		struct vm_area_struct *src, *dst;
		struct mm_subpage *held;
		struct folio *folio;
		phys_addr_t physical;
		swp_entry_t entry = { };
		pte_t before, after;
		vm_fault_t fault;
		unsigned long i;
		int err, mapped;

		KUNIT_ASSERT_NOT_NULL(test, mm);
		src = test_fault_vma_in_mm(mm, TEST_VA, PAGE_SIZE);
		dst = test_fault_vma_in_mm(mm, TEST_VA + SZ_128M, PAGE_SIZE);
		KUNIT_ASSERT_NOT_NULL(test, src);
		KUNIT_ASSERT_NOT_NULL(test, dst);
		for (i = 0; i < PAGE_SIZE; i += size) {
			mmap_read_lock(mm);
			fault = handle_mm_fault(src, TEST_VA + i,
					FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(mm);
			KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
			before = test_fault_entry(test, mm, TEST_VA + i);
			memset(phys_to_virt(pte_phys_mm(mm, before)), 0x50 + i / size, size);
		}
		before = test_fault_entry(test, mm, src_addr);
		physical = pte_phys_mm(mm, before);
		held = mm_subpage_get_from_phys(physical);
		KUNIT_ASSERT_NOT_NULL(test, held);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_subpage_put, held), 0);
		KUNIT_ASSERT_EQ(test, mm_subpage_offset(held), (unsigned int)(3 * size));
		folio = mm_subpage_folio(held);
		lru_add_drain_all();
		folio_lock(folio);
		err = folio_alloc_swap(folio);
		if (!err) {
			entry = folio->swap;
			try_to_unmap(folio, TTU_SYNC);
		}
		mapped = folio_mapcount(folio);
		folio_unlock(folio);
		KUNIT_ASSERT_EQ(test, err, 0);
		KUNIT_ASSERT_EQ(test, mapped, 0);
		KUNIT_ASSERT_EQ(test, mm_subpage_mapcount(held), 0);
		KUNIT_ASSERT_EQ(test, swap_subpage_count(entry, 3 * size), 1U);
		before = test_fault_entry(test, mm, src_addr);
		KUNIT_ASSERT_TRUE(test, softleaf_is_swap(softleaf_from_pte(before)));
		if (forked) {
			struct mm_struct *child = test_vma_mm_granule(test, shift);

			KUNIT_ASSERT_NOT_NULL(test, child);
			uprobe_start_dup_mmap();
			err = dup_mmap(child, mm);
			uprobe_end_dup_mmap();
			KUNIT_ASSERT_EQ(test, err, 0);
			KUNIT_EXPECT_EQ(test, test_uffd_move(dst, dst_addr, src_addr, size),
					(ssize_t)-EBUSY);
			KUNIT_EXPECT_EQ(test, softleaf_from_pte(test_fault_entry(test, mm,
									    src_addr)).val, entry.val);
			kunit_release_action(test, user4k_vma_mm_free, child);
			before = test_fault_entry(test, mm, src_addr);
			KUNIT_EXPECT_FALSE(test, pte_swp_exclusive(before));
			KUNIT_ASSERT_EQ(test, swap_subpage_count(entry, 3 * size), 1U);
		}
		KUNIT_ASSERT_EQ(test, test_uffd_move(dst, dst_addr, src_addr, size), (ssize_t)size);
		after = test_fault_entry(test, mm, dst_addr);
		KUNIT_EXPECT_EQ(test, softleaf_from_pte(after).val, entry.val);
		KUNIT_EXPECT_EQ(test, pte_swp_subpage_offset(after), (unsigned int)(3 * size));
		KUNIT_EXPECT_TRUE(test, pte_none(test_fault_entry(test, mm, src_addr)));
		/* Restore with the retained metadata already rebound to the new root. */
		mmap_read_lock(mm);
		fault = handle_mm_fault(dst, dst_addr, FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(mm);
		KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
		after = test_fault_entry(test, mm, dst_addr);
		KUNIT_ASSERT_TRUE(test, pte_present(after));
		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, after), physical);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(physical), 0x53, size), NULL);
		KUNIT_EXPECT_EQ(test, mm_subpage_is_exclusive(held), !forked);
		/* Fork-cleared exclusivity requires COW while a GET is retained. */
		mmap_read_lock(mm);
		fault = handle_mm_fault(dst, dst_addr, FAULT_FLAG_WRITE | FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(mm);
		KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
		after = test_fault_entry(test, mm, dst_addr);
		KUNIT_ASSERT_TRUE(test, pte_write(after));
		if (forked)
			KUNIT_EXPECT_NE(test, pte_phys_mm(mm, after), physical);
		else
			KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, after), physical);
		memset(phys_to_virt(pte_phys_mm(mm, after)), 0xa9, size);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(physical),
						  forked ? 0x53 : 0xa9, size), NULL);
		for (i = 0; i < PAGE_SIZE; i += size) {
			if (i == 3 * size)
				continue;
			mmap_read_lock(mm);
			fault = handle_mm_fault(src, TEST_VA + i, FAULT_FLAG_REMOTE, NULL);
			mmap_read_unlock(mm);
			KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), (vm_fault_t)0);
			after = test_fault_entry(test, mm, TEST_VA + i);
			KUNIT_ASSERT_TRUE(test, pte_present(after));
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(phys_to_virt(pte_phys_mm(mm, after)),
							  0x50 + i / size, size), NULL);
		}
		kunit_release_action(test, test_subpage_put, held);
		kunit_release_action(test, user4k_vma_mm_free, mm);
	}
}
#endif

static struct kunit_case user4k_swap_cases[] = {
	KUNIT_CASE(user4k_swap_reuse_get_test),
	KUNIT_CASE(user4k_swap_reuse_swapoff_test),
	KUNIT_CASE(user4k_swap_reuse_cached_test),
	KUNIT_CASE(user4k_swap_reuse_disk_test),
	KUNIT_CASE(user4k_swap_exclusive_cached_test),
	KUNIT_CASE(user4k_swap_exclusive_disk_test),
	KUNIT_CASE(user4k_swap_exclusive_swapoff_test),
	KUNIT_CASE(user4k_swap_cached_test),
	KUNIT_CASE(user4k_swap_pin_test),
	KUNIT_CASE(user4k_swap_mremap_test),
#ifdef CONFIG_MIGRATION
	KUNIT_CASE(user4k_swap_migration_test),
#endif
	KUNIT_CASE(user4k_swap_disk_test),
	KUNIT_CASE(user4k_swap_reclaim_test),
	KUNIT_CASE(user4k_swapoff_test),
	KUNIT_CASE(user4k_swap_stress_test),
	KUNIT_CASE(user4k_swapoff_boundaries_test),
	KUNIT_CASE(user4k_live_swap_fault_test),
	{}
};

static void user4k_swap_suite_exit(struct kunit_suite *suite)
{
	kvfree_rcu_barrier();
}

#if defined(CONFIG_SHMEM) && defined(CONFIG_MM_SUBPAGE) && defined(CONFIG_USERFAULTFD)
static void test_shmem_uffd_swap(struct kunit *test, unsigned int mode)
{
	const unsigned long missing = GENMASK(PAGE_SIZE / SZ_4K - 1, mode == 3 ? 2 : 1);
	struct swap_io_ctx ctx = {};
	struct file *file;
	struct folio *folio;
	void *entry;
	bool *disabled;
	int ret;
	LIST_HEAD(folios);

	if (!total_swap_pages) {
		kunit_skip(test, "requires disposable guest swap");
		return;
	}
	disabled = kunit_kzalloc(test, sizeof(*disabled), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, disabled);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_reenable_swap, disabled), 0);
	file = test_cache_file(test, PAGE_SIZE, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	ret = shmem_get_folio(file_inode(file), 0, 0, &folio, SGP_CACHE);
	KUNIT_ASSERT_EQ(test, ret, 0);
	memset(folio_address(folio), 0xb6, mode == 3 ? 2 * SZ_4K : SZ_4K);
	ret = shmem_uffd_set_missing(folio, 0, missing);
	folio_mark_dirty(folio);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	lru_add_drain_all();
	folio = filemap_lock_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	/* Match pageout(): submitted data is clean unless writeout redirties it. */
	KUNIT_EXPECT_TRUE(test, folio_clear_dirty_for_io(folio));
	ret = shmem_writeout(&ctx, folio, &folios);
	swap_write_submit(&ctx);
	if (ret == AOP_WRITEPAGE_ACTIVATE)
		folio_unlock(folio);
	folio_wait_writeback(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_TRUE(test, list_empty(&folios));
	entry = filemap_get_entry(file->f_mapping, 0);
	if (entry && !xa_is_value(entry))
		folio_put(entry);
	KUNIT_ASSERT_TRUE(test, xa_is_value(entry));
	KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(radix_to_swp_entry(entry)));
	KUNIT_EXPECT_EQ(test, xa_to_value(xa_load(&SHMEM_I(file_inode(file))->uffd_missing, 0)), missing);
	if (mode == 1) {
		shmem_truncate_range(file_inode(file), 0, PAGE_SIZE - 1);
		KUNIT_EXPECT_TRUE(test, xa_empty(&SHMEM_I(file_inode(file))->uffd_missing));
	} else if (mode == 3) {
		shmem_truncate_range(file_inode(file), 0, SZ_4K - 1);
	} else if (mode == 2) {
		ret = test_swap_control("--swapoff-helper");
		*disabled = ret == 0;
		KUNIT_ASSERT_EQ(test, ret, 0);
	}
	ret = shmem_get_folio(file_inode(file), 0, 0, &folio, SGP_CACHE);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 0),
			mode == 1 ? 0UL : mode == 3 ? missing | BIT(0) : missing);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio),
			mode == 1 || mode == 3 ? 0 : 0xb6, SZ_4K), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K,
			mode == 3 ? 0xb6 : 0, SZ_4K), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + 2 * SZ_4K,
			0, PAGE_SIZE - 2 * SZ_4K), NULL);
	KUNIT_EXPECT_TRUE(test, xa_empty(&SHMEM_I(file_inode(file))->uffd_missing));
	folio_unlock(folio);
	folio_put(folio);
}

static void user4k_shmem_uffd_swapin_test(struct kunit *test)
{
	test_shmem_uffd_swap(test, 0);
}

static void user4k_shmem_uffd_swapped_truncate_test(struct kunit *test)
{
	test_shmem_uffd_swap(test, 1);
}

static void user4k_shmem_uffd_swapped_partial_hole_test(struct kunit *test)
{
	test_shmem_uffd_swap(test, 3);
}

static void user4k_shmem_uffd_swapoff_test(struct kunit *test)
{
	test_shmem_uffd_swap(test, 2);
}

#if defined(CONFIG_FAILSLAB) && defined(CONFIG_FAULT_INJECTION_DEBUG_FS)
static void test_disable_failslab(void *unused)
{
	test_swap_control("--failslab-off-helper");
}
#endif

static void user4k_shmem_uffd_swap_alloc_fail_test(struct kunit *test)
{
#if defined(CONFIG_FAILSLAB) && defined(CONFIG_FAULT_INJECTION_DEBUG_FS)
	unsigned int attempt;

	if (!total_swap_pages) {
		kunit_skip(test, "requires disposable guest swap");
		return;
	}
	KUNIT_ASSERT_EQ(test, test_swap_control("--failslab-on-helper"), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_disable_failslab, NULL), 0);
	for (attempt = 1; attempt <= 2; attempt++) {
		pgoff_t index = attempt == 1 ? 1 : 64;
		const unsigned long missing = GENMASK(PAGE_SIZE / SZ_4K - 1, 1);
		struct file *file = test_cache_file(test, (index + 1) * PAGE_SIZE, false);
		struct swap_io_ctx ctx = {};
		struct folio *folio;
		unsigned int pending;
		unsigned long pfn;
		void *entry;
		int ret;
		LIST_HEAD(folios);

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		ret = shmem_get_folio(file_inode(file), index, 0, &folio, SGP_CACHE);
		KUNIT_ASSERT_EQ(test, ret, 0);
		memset(folio_address(folio), 0xb6, SZ_4K);
		ret = shmem_uffd_set_missing(folio, index, missing);
		folio_mark_dirty(folio);
		folio_unlock(folio);
		folio_put(folio);
		KUNIT_ASSERT_EQ(test, ret, 0);
		lru_add_drain_all();
		folio = filemap_lock_folio(file->f_mapping, index);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		pfn = folio_pfn(folio);
		KUNIT_EXPECT_TRUE(test, folio_clear_dirty_for_io(folio));
		/* Fail this task's selected allocation and its allocator retries. */
		WRITE_ONCE(current->make_it_fail, 1);
		WRITE_ONCE(current->fail_nth, attempt);
		ret = shmem_writeout(&ctx, folio, &folios);
		pending = READ_ONCE(current->fail_nth);
		WRITE_ONCE(current->fail_nth, 0);
		WRITE_ONCE(current->make_it_fail, 0);
		KUNIT_EXPECT_EQ(test, pending, 0U);
		KUNIT_EXPECT_EQ(test, ret, AOP_WRITEPAGE_ACTIVATE);
		if (ret != AOP_WRITEPAGE_ACTIVATE) {
			swap_write_submit(&ctx);
			folio_wait_writeback(folio);
			folio_put(folio);
			return;
		}
		KUNIT_EXPECT_TRUE(test, folio_test_locked(folio));
		KUNIT_EXPECT_TRUE(test, folio_test_dirty(folio));
		KUNIT_EXPECT_FALSE(test, folio_test_swapcache(folio));
		KUNIT_EXPECT_PTR_EQ(test, folio->mapping, file->f_mapping);
		KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, index), missing);
		/* A failed multi-level insertion may retain an empty internal node. */
		{
			unsigned long cursor = 0;

			KUNIT_EXPECT_PTR_EQ(test, xa_find(&SHMEM_I(file_inode(file))->uffd_missing,
					&cursor, ULONG_MAX, XA_PRESENT), NULL);
		}
		KUNIT_EXPECT_PTR_EQ(test, ctx.sio, NULL);
		KUNIT_EXPECT_TRUE(test, list_empty(&folios));
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0xb6, SZ_4K), NULL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K, 0, PAGE_SIZE - SZ_4K), NULL);
		/* No restart or cache repair: retry exactly the retained folio. */
		KUNIT_EXPECT_EQ(test, folio_pfn(folio), pfn);
		KUNIT_EXPECT_TRUE(test, folio_clear_dirty_for_io(folio));
		ret = shmem_writeout(&ctx, folio, &folios);
		swap_write_submit(&ctx);
		if (ret == AOP_WRITEPAGE_ACTIVATE)
			folio_unlock(folio);
		folio_wait_writeback(folio);
		folio_put(folio);
		KUNIT_ASSERT_EQ(test, ret, 0);
		entry = filemap_get_entry(file->f_mapping, index);
		if (entry && !xa_is_value(entry))
			folio_put(entry);
		KUNIT_ASSERT_TRUE(test, xa_is_value(entry));
		KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(radix_to_swp_entry(entry)));
		KUNIT_EXPECT_EQ(test, xa_to_value(xa_load(&SHMEM_I(file_inode(file))->uffd_missing, index)), missing);
		ret = shmem_get_folio(file_inode(file), index, 0, &folio, SGP_CACHE);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, index), missing);
		KUNIT_EXPECT_TRUE(test, xa_empty(&SHMEM_I(file_inode(file))->uffd_missing));
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0xb6, SZ_4K), NULL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K, 0, PAGE_SIZE - SZ_4K), NULL);
		folio_unlock(folio);
		folio_put(folio);
		kunit_info(test, "failed metadata allocation %u at index %lu; same-folio retry and disk swap-in preserved bytes/mask", attempt, index);
	}
#else
	kunit_skip(test, "requires FAILSLAB and fault-injection debugfs");
#endif
}

#ifdef CONFIG_MEMCG
static void test_memcg_put(void *memcg)
{
	mem_cgroup_put(memcg);
}
#endif

static void user4k_shmem_uffd_swap_charge_test(struct kunit *test)
{
#ifdef CONFIG_MEMCG
	struct cgroup *cgroup;
	struct cgroup_subsys_state *css;
	struct mem_cgroup *memcg, *saved;
	struct file *file;
	struct folio *folio;
	struct swap_io_ctx ctx = {};
	void *entry;
	const unsigned long missing = GENMASK(PAGE_SIZE / SZ_4K - 1, 1);
	int ret;
	LIST_HEAD(folios);

	if (!total_swap_pages || mem_cgroup_disabled()) {
		kunit_skip(test, "requires disposable guest swap and memcg");
		return;
	}
	KUNIT_ASSERT_EQ(test, test_swap_control("--memcg-setup-helper"), 0);
	cgroup = cgroup_get_from_path("/uffd-metadata");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cgroup);
	css = cgroup_get_e_css(cgroup, &memory_cgrp_subsys);
	cgroup_put(cgroup);
	KUNIT_ASSERT_NOT_NULL(test, css);
	memcg = mem_cgroup_from_css(css);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_memcg_put, memcg), 0);
	KUNIT_ASSERT_PTR_NE(test, memcg, root_mem_cgroup);
	file = test_cache_file(test, 65 * PAGE_SIZE, false);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	saved = set_active_memcg(memcg);
	ret = shmem_get_folio(file_inode(file), 64, 0, &folio, SGP_CACHE);
	set_active_memcg(saved);
	KUNIT_ASSERT_EQ(test, ret, 0);
	rcu_read_lock();
	KUNIT_EXPECT_PTR_EQ(test, folio_memcg(folio), memcg);
	rcu_read_unlock();
	ret = shmem_uffd_set_missing(folio, 64, missing);
	folio_mark_dirty(folio);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	lru_add_drain_all();
	folio = filemap_lock_folio(file->f_mapping, 64);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_EXPECT_TRUE(test, folio_clear_dirty_for_io(folio));
	/* Reclaim from the root charge domain, not the folio's cgroup. */
	saved = set_active_memcg(root_mem_cgroup);
	ret = shmem_writeout(&ctx, folio, &folios);
	set_active_memcg(saved);
	swap_write_submit(&ctx);
	if (ret == AOP_WRITEPAGE_ACTIVATE)
		folio_unlock(folio);
	folio_wait_writeback(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	rcu_read_lock();
	entry = rcu_dereference(SHMEM_I(file_inode(file))->uffd_missing.xa_head);
	if (xa_is_node(entry))
		KUNIT_EXPECT_PTR_EQ(test, mem_cgroup_from_virt(xa_to_node(entry)), memcg);
	rcu_read_unlock();
	KUNIT_ASSERT_TRUE(test, xa_is_node(entry));
	entry = filemap_get_entry(file->f_mapping, 64);
	if (entry && !xa_is_value(entry))
		folio_put(entry);
	KUNIT_ASSERT_TRUE(test, xa_is_value(entry));
	KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(radix_to_swp_entry(entry)));
	ret = shmem_get_folio(file_inode(file), 64, 0, &folio, SGP_CACHE);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 64), missing);
	KUNIT_EXPECT_TRUE(test, xa_empty(&SHMEM_I(file_inode(file))->uffd_missing));
	folio_unlock(folio);
	folio_put(folio);
#else
	kunit_skip(test, "requires memcg");
#endif
}

static bool test_coarse_swap_enabled(struct kunit *test)
{
	if (!test_coarse_file_enabled(test))
		return false;
	if (!total_swap_pages || !IS_ENABLED(CONFIG_TRANSPARENT_HUGEPAGE)) {
		kunit_skip(test, "requires disposable coarse-shmem swap runner");
		return false;
	}
	return true;
}

static bool test_coarse_swap_out(struct kunit *test, struct file *file, pgoff_t index)
{
	unsigned int tries;

	for (tries = 0; tries < 100; tries++) {
		struct folio *folio;
		void *entry = filemap_get_entry(file->f_mapping, index);
		LIST_HEAD(folios);

		if (xa_is_value(entry))
			return test_reclaim_swapcache(radix_to_swp_entry(entry));
		if (!entry) {
			KUNIT_FAIL(test, "reclaim removed data without a swap entry");
			return false;
		}
		folio = entry;
		lru_add_drain_all();
		if (folio_isolate_lru(folio))
			list_add(&folio->lru, &folios);
		folio_put(folio);
		/* Real reclaim owns any folios produced by a large-folio split. */
		reclaim_pages(&folios);
		schedule_timeout_uninterruptible(1);
	}
	KUNIT_FAIL(test, "file index %lu did not reach swap", index);
	return false;
}

static void test_coarse_cold_shmem(struct kunit *test, bool promoted)
{
	struct file *file;
	struct vm_area_struct *coarse, *small;
	struct folio *folio;
	unsigned int nr = SZ_64K / PAGE_SIZE, i;
	unsigned long offset;
	vm_fault_t fault;

	if (!test_coarse_swap_enabled(test))
		return;
	file = test_cache_file(test, SZ_64K, true);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	coarse = test_file_vma_granule(test, file, 16, true, TEST_VA, SZ_64K, 0);
	small = test_file_vma(test, file, false, true, TEST_VA + SZ_4K,
			     SZ_64K - SZ_4K, SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, coarse);
	KUNIT_ASSERT_NOT_NULL(test, small);
	if (promoted) {
		KUNIT_ASSERT_EQ(test, test_file_fault(coarse, TEST_VA, false) & VM_FAULT_ERROR, 0U);
		folio = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_EQ(test, folio_size(folio), SZ_64K);
		folio_put(folio);
	}
	for (offset = SZ_4K; offset < SZ_64K; offset += SZ_4K)
		KUNIT_ASSERT_EQ(test, test_file_fault(small, TEST_VA + offset, false) &
				VM_FAULT_ERROR, 0U);
	lru_add_drain_all();
	for (i = 0; i < nr; i++)
		KUNIT_ASSERT_TRUE(test, test_coarse_swap_out(test, file, i));
	/* XArray swap entries and absent swapcache prove this is a cold refault. */
	for (i = 0; i < nr; i++) {
		void *entry = filemap_get_entry(file->f_mapping, i);
		struct folio *cached = NULL;

		KUNIT_EXPECT_TRUE(test, xa_is_value(entry));
		if (entry && !xa_is_value(entry)) {
			folio_put(entry);
			return;
		}
		KUNIT_ASSERT_NOT_NULL(test, entry);
		cached = swap_cache_get_folio(radix_to_swp_entry(entry));
		KUNIT_EXPECT_PTR_EQ(test, cached, NULL);
		if (cached)
			folio_put(cached);
	}
	KUNIT_EXPECT_EQ(test, file->f_mapping->nrpages, 0UL);
	KUNIT_EXPECT_EQ(test, SHMEM_I(file_inode(file))->swapped, (unsigned long)nr);
	fault = test_file_fault(coarse, TEST_VA, false);
	KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY | VM_FAULT_NOPAGE), 0U);
	KUNIT_EXPECT_TRUE(test, fault & VM_FAULT_MAJOR);
	folio = filemap_get_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	KUNIT_EXPECT_EQ(test, folio_size(folio), SZ_64K);
	for (offset = 0; offset < SZ_64K; offset += SZ_4K)
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + offset,
						0x31 + offset / SZ_4K, SZ_4K), NULL);
	KUNIT_EXPECT_EQ(test, file->f_mapping->nrpages, (unsigned long)nr);
	KUNIT_EXPECT_EQ(test, SHMEM_I(file_inode(file))->swapped, 0UL);
	for (offset = SZ_4K; offset < SZ_64K; offset += SZ_4K) {
		pte_t entry;

		KUNIT_ASSERT_EQ(test, test_file_fault(small, TEST_VA + offset, false) &
				VM_FAULT_ERROR, 0U);
		entry = test_fault_entry(test, small->vm_mm, TEST_VA + offset);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(small->vm_mm, entry),
				page_to_phys(&folio->page) + offset);
	}
	kunit_release_action(test, user4k_vma_mm_free, coarse->vm_mm);
	kunit_release_action(test, user4k_vma_mm_free, small->vm_mm);
	lru_add_drain_all();
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), (int)nr + 1);
}

static void user64k_shmem_cold_constituents_test(struct kunit *test)
{
	test_coarse_cold_shmem(test, false);
}

static void user64k_shmem_cold_promoted_test(struct kunit *test)
{
	test_coarse_cold_shmem(test, true);
}

#if defined(CONFIG_FAILSLAB) && defined(CONFIG_FAIL_PAGE_ALLOC) && defined(CONFIG_MEMCG)
static void test_coarse_injection_off(void *unused)
{
	test_swap_control("--coarse-injection-off");
}

#endif

static void user64k_shmem_fault_alloc_fail_test(struct kunit *test)
{
#if defined(CONFIG_FAILSLAB) && defined(CONFIG_FAIL_PAGE_ALLOC) && defined(CONFIG_MEMCG)
	unsigned int mode;

	if (!test_coarse_swap_enabled(test))
		return;
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_coarse_injection_off, NULL), 0);
	for (mode = 0; mode < 3; mode++) {
		struct file *file = test_cache_file(test, SZ_64K, false);
		struct vm_area_struct *vma;
		struct folio *folio;
		phys_addr_t original[SZ_64K / PAGE_SIZE];
		unsigned char *buffer;
		loff_t pos = 0;
		unsigned int i, pending = 0;
		vm_fault_t result;
		bool prior_fail;
		char operation[80];
		int enter = 0, leave = 0;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		/* Make the zero-limit charge case nonblocking and avoid an OOM kill. */
		if (mode == 2)
			mapping_set_gfp_mask(file->f_mapping,
				mapping_gfp_mask(file->f_mapping) & ~__GFP_DIRECT_RECLAIM);
		buffer = kunit_kmalloc(test, SZ_64K, GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, buffer);
		memset(buffer, 0x95, SZ_64K);
		KUNIT_ASSERT_EQ(test, kernel_write(file, buffer, SZ_64K, &pos), (ssize_t)SZ_64K);
		vma = test_file_vma_granule(test, file, 16, true, TEST_VA, SZ_64K, 0);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		for (i = 0; i < SZ_64K / PAGE_SIZE; i++) {
			folio = filemap_get_folio(file->f_mapping, i);
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
			KUNIT_EXPECT_EQ(test, folio_order(folio), 0U);
			original[i] = page_to_phys(&folio->page);
			folio_put(folio);
		}
		if (mode < 2) {
			KUNIT_ASSERT_EQ(test, test_swap_control(mode ? "--coarse-page-on" :
								"--coarse-slab-on"), 0);
		} else {
			snprintf(operation, sizeof(operation), "--coarse-memcg-enter=%d",
				 task_pid_nr(current));
			enter = test_swap_control(operation);
		}
		/* Direct callback isolates backing allocation from page-table allocation. */
		if (!enter) {
			struct vm_fault vmf = { .vma = vma, .address = TEST_VA,
				.pgoff = 0, .flags = FAULT_FLAG_REMOTE };

			mmap_read_lock(vma->vm_mm);
			prior_fail = current->make_it_fail;
			if (!mode)
				WRITE_ONCE(current->fail_nth, 1);
			else if (mode == 1)
				current->make_it_fail = true;
			result = vma->vm_ops->fault(&vmf);
			pending = READ_ONCE(current->fail_nth);
			WRITE_ONCE(current->fail_nth, 0);
			current->make_it_fail = prior_fail;
			mmap_read_unlock(vma->vm_mm);
			/* Keep unexpected success safe for test cleanup. */
			if (vmf.page) {
				if (result & VM_FAULT_LOCKED)
					unlock_page(vmf.page);
				put_page(vmf.page);
			}
		} else {
			result = 0;
		}
		if (mode == 2) {
			snprintf(operation, sizeof(operation), "--coarse-memcg-leave=%d",
				 task_pid_nr(current));
			leave = test_swap_control(operation);
		} else {
			leave = test_swap_control("--coarse-injection-off");
		}
		KUNIT_ASSERT_EQ(test, enter, 0);
		KUNIT_ASSERT_EQ(test, leave, 0);
		KUNIT_EXPECT_EQ(test, pending, 0U);
		KUNIT_ASSERT_TRUE(test, result & VM_FAULT_OOM);
		for (i = 0; i < SZ_64K / PAGE_SIZE; i++) {
			folio = filemap_get_folio(file->f_mapping, i);
			KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
			KUNIT_EXPECT_EQ(test, folio_order(folio), 0U);
			KUNIT_EXPECT_EQ(test, page_to_phys(&folio->page), original[i]);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0x95, PAGE_SIZE), NULL);
			folio_put(folio);
		}
		KUNIT_EXPECT_EQ(test, file->f_mapping->nrpages, SZ_64K / PAGE_SIZE);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_SHMEMPAGES), 0L);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, false) &
				(VM_FAULT_ERROR | VM_FAULT_RETRY | VM_FAULT_NOPAGE), 0U);
		folio = filemap_get_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_EQ(test, folio_size(folio), SZ_64K);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0x95, SZ_64K), NULL);
		folio_put(folio);
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
		kunit_info(test, "coarse allocation failure mode %u preserved data and recovered", mode);
	}
#else
	kunit_skip(test, "requires slab/page fault injection and memcg");
#endif
}

static void user64k_anon_swap_test(struct kunit *test)
{
	unsigned int mode, nr = SZ_64K / PAGE_SIZE;

	if (!test_coarse_swap_enabled(test))
		return;
	for (mode = 0; mode < 7; mode++) {
		struct vm_area_struct *vma, *cvma = NULL;
		struct mm_struct *child = NULL;
		struct folio *folio, *held = NULL;
		swp_entry_t entry, part;
		unsigned int i, tries;
		pte_t pte;
		bool *disabled;
		bool warm = !mode || mode == 6;
		vm_fault_t fault;
		int err;

		vma = test_fault_vma_granule(test, 16, TEST_VA, SZ_64K);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
		folio = page_folio(pfn_to_page(PHYS_PFN(pte_phys_mm(vma->vm_mm, pte))));
		for (i = 0; i < SZ_64K / SZ_4K; i++)
			memset(folio_address(folio) + i * SZ_4K, 0x31 + i, SZ_4K);
		if (mode == 2 || mode == 6) {
			child = test_vma_mm_granule(test, 16);
			KUNIT_ASSERT_NOT_NULL(test, child);
			uprobe_start_dup_mmap();
			err = dup_mmap(child, vma->vm_mm);
			uprobe_end_dup_mmap();
			KUNIT_ASSERT_EQ(test, err, 0);
			cvma = test_vma_lookup(child, TEST_VA);
			KUNIT_ASSERT_NOT_NULL(test, cvma);
		}
		if (warm) {
			folio_get(folio);
			held = folio;
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, held), 0);
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
			KUNIT_EXPECT_EQ(test, min_order_for_split(folio), 16U - PAGE_SHIFT);
#endif
			folio_lock(folio);
			KUNIT_EXPECT_EQ(test, folio_anon_min_order(folio), 16U - PAGE_SHIFT);
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
			KUNIT_EXPECT_EQ(test, split_huge_page_to_list_to_order(&folio->page, NULL, 0),
					-EBUSY);
			KUNIT_EXPECT_EQ(test, folio_order(folio), 16U - PAGE_SHIFT);
			expect_coarse_anon_state(test, folio, child ? 2 : 1, !child);
#endif
			err = folio_alloc_swap(folio);
			if (!err)
				try_to_unmap(folio, TTU_SYNC);
			folio_unlock(folio);
			KUNIT_ASSERT_EQ(test, err, 0);
		} else {
			for (tries = 0; tries < 100; tries++) {
				LIST_HEAD(folios);

				pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
				if (!pte_present(pte))
					break;
				folio = page_folio(pfn_to_page(PHYS_PFN(pte_phys_mm(vma->vm_mm, pte))));
				folio_get(folio);
				lru_add_drain_all();
				if (folio_isolate_lru(folio))
					list_add(&folio->lru, &folios);
				folio_put(folio);
				reclaim_pages(&folios);
				schedule_timeout_uninterruptible(1);
			}
		}
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_FALSE_MSG(test, pte_present(pte), "mode=%u", mode);
		entry = softleaf_from_pte(pte);
		KUNIT_ASSERT_TRUE(test, softleaf_is_swap(entry));
		KUNIT_EXPECT_TRUE(test, IS_ALIGNED(swp_offset(entry), nr));
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_ANONPAGES), 0L);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_SWAPENTS), 1L);
		if (mode == 3) {
			child = test_vma_mm_granule(test, 16);
			KUNIT_ASSERT_NOT_NULL(test, child);
			uprobe_start_dup_mmap();
			err = dup_mmap(child, vma->vm_mm);
			uprobe_end_dup_mmap();
			KUNIT_ASSERT_EQ(test, err, 0);
		}
		for (i = 0; i < nr; i++) {
			part = entry;
			part.val += i;
			KUNIT_EXPECT_EQ(test, swp_swapcount(part), child ? 2 : 1);
			if (!warm)
				KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(part));
		}
		if (mode == 3) {
			/* Exit a swapped child: every constituent count must be released. */
			kunit_release_action(test, user4k_vma_mm_free, child);
			child = NULL;
			for (i = 0; i < nr; i++) {
				part = entry;
				part.val += i;
				KUNIT_EXPECT_EQ(test, swp_swapcount(part), 1);
			}
		}
		if (mode == 5) {
#if defined(CONFIG_FAIL_PAGE_ALLOC) && defined(CONFIG_FAULT_INJECTION)
			bool old = current->make_it_fail;

			KUNIT_ASSERT_EQ(test, test_swap_control("--coarse-page-on"), 0);
			current->make_it_fail = true;
			fault = test_file_fault(vma, TEST_VA, false);
			current->make_it_fail = old;
			err = test_swap_control("--coarse-injection-off");
			KUNIT_ASSERT_EQ(test, err, 0);
			KUNIT_EXPECT_TRUE(test, fault & VM_FAULT_OOM);
			KUNIT_EXPECT_TRUE(test, pte_same(test_fault_entry(test, vma->vm_mm, TEST_VA), pte));
			for (i = 0; i < nr; i++) {
				part = entry;
				part.val += i;
				KUNIT_EXPECT_EQ(test, swp_swapcount(part), 1);
			}
#endif
		}
		if (mode == 4) {
			disabled = kunit_kzalloc(test, sizeof(*disabled), GFP_KERNEL);
			KUNIT_ASSERT_NOT_NULL(test, disabled);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_reenable_swap, disabled), 0);
			err = test_swap_control("--swapoff-helper");
			if (!err)
				*disabled = true;
			KUNIT_ASSERT_EQ(test, err, 0);
		} else {
			fault = test_file_fault(vma, TEST_VA + SZ_64K - 1, false);
			KUNIT_ASSERT_EQ_MSG(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), 0U,
					   "mode=%u fault=%x", mode, fault);
			if (!warm && mode != 5)
				KUNIT_EXPECT_TRUE(test, fault & VM_FAULT_MAJOR);
		}
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_TRUE_MSG(test, pte_present(pte), "mode=%u", mode);
		folio = page_folio(pfn_to_page(PHYS_PFN(pte_phys_mm(vma->vm_mm, pte))));
		KUNIT_EXPECT_EQ(test, folio_size(folio), (unsigned long)SZ_64K);
		if (held) {
			KUNIT_EXPECT_PTR_EQ(test, folio, held); /* Warm aligned cache: no copy. */
		} else {
			folio_get(folio);
			held = folio;
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, held), 0);
		}
		for (i = 0; i < SZ_64K / SZ_4K; i++)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + i * SZ_4K,
							0x31 + i, SZ_4K), NULL);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_SWAPENTS), 0L);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_ANONPAGES), 1L);
		if (child) {
			struct folio *copied;

			KUNIT_ASSERT_EQ(test, test_file_fault(cvma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
			pte = test_fault_entry(test, child, TEST_VA);
			KUNIT_ASSERT_TRUE(test, pte_present(pte));
			KUNIT_EXPECT_NE(test, pte_phys_mm(child, pte), page_to_phys(&folio->page));
			KUNIT_EXPECT_TRUE(test, pte_write(pte));
			copied = page_folio(pfn_to_page(PHYS_PFN(pte_phys_mm(child, pte))));
			expect_coarse_anon_state(test, copied, 1, true);
			for (i = 0; i < SZ_64K / SZ_4K; i++)
				KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(copied) + i * SZ_4K,
								0x31 + i, SZ_4K), NULL);
			memset(folio_address(copied), 0xa7, SZ_64K);
			for (i = 0; i < SZ_64K / SZ_4K; i++)
				KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + i * SZ_4K,
								0x31 + i, SZ_4K), NULL);
			kunit_release_action(test, user4k_vma_mm_free, child);
		}
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
		lru_add_drain_all();
		if (held) {
			KUNIT_EXPECT_EQ(test, folio_mapcount(held), 0);
			KUNIT_EXPECT_EQ(test, folio_ref_count(held), 1);
		}
		for (i = 0; i < nr; i++) {
			struct swap_info_struct *si;

			part = entry;
			part.val += i;
			si = get_swap_device(part);
			if (si) {
				KUNIT_EXPECT_FALSE(test, swap_entry_swapped(si, part));
				put_swap_device(si);
			} else {
				KUNIT_EXPECT_EQ(test, mode, 4U);
			}
		}
		if (mode == 4)
			kunit_release_action(test, test_reenable_swap, disabled);
		kunit_info(test, "completed coarse anonymous swap mode %u", mode);
	}
}

/* The caller's private fixture has no concurrent PTE writers during reclaim. */
static void test_coarse_anon_reclaim_once(struct vm_area_struct *vma)
{
	struct folio *folio = test_fault_folio(vma->vm_mm, vma->vm_start);
	LIST_HEAD(folios);

	if (!folio)
		return;
	lru_add_drain_all();
	if (folio_isolate_lru(folio))
		list_add(&folio->lru, &folios);
	folio_put(folio);
	reclaim_pages(&folios);
	schedule_timeout_uninterruptible(1);
}

static void user64k_anon_swap_limit_test(struct kunit *test)
{
#ifdef CONFIG_MEMCG
	struct vm_area_struct *vma;
	struct cgroup *cgroup;
	struct cgroup_subsys_state *css;
	struct mem_cgroup *memcg;
	struct folio *folio;
	swp_entry_t entry, part;
	phys_addr_t original;
	unsigned int i, tries, nr = SZ_64K / PAGE_SIZE;
	long before, denied;
	char operation[80];
	vm_fault_t fault;
	pte_t pte;
	int ret, leave;

	if (!test_coarse_swap_enabled(test))
		return;
	if (mem_cgroup_disabled()) {
		kunit_skip(test, "requires memory cgroups");
		return;
	}
	snprintf(operation, sizeof(operation), "--coarse-swap-enter=%d", task_pid_nr(current));
	KUNIT_ASSERT_EQ(test, test_swap_control(operation), 0);
	vma = test_fault_vma_granule(test, 16, TEST_VA, SZ_64K);
	fault = vma ? test_file_fault(vma, TEST_VA, true) : VM_FAULT_OOM;
	snprintf(operation, sizeof(operation), "--coarse-swap-leave=%d", task_pid_nr(current));
	leave = test_swap_control(operation);
	KUNIT_ASSERT_EQ(test, leave, 0);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, fault & VM_FAULT_ERROR, 0U);
	cgroup = cgroup_get_from_path("/coarse-swap-limit");
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, cgroup);
	css = cgroup_get_e_css(cgroup, &memory_cgrp_subsys);
	cgroup_put(cgroup);
	KUNIT_ASSERT_NOT_NULL(test, css);
	memcg = mem_cgroup_from_css(css);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_memcg_put, memcg), 0);
	KUNIT_ASSERT_PTR_NE(test, memcg, root_mem_cgroup);
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	rcu_read_lock();
	KUNIT_EXPECT_PTR_EQ(test, folio_memcg(folio), memcg);
	rcu_read_unlock();
	original = page_to_phys(&folio->page);
	memset(folio_address(folio), 0xc9, SZ_64K);
	before = atomic_long_read(&memcg->memory_events[MEMCG_SWAP_MAX]);
	folio_lock(folio);
	ret = folio_alloc_swap(folio);
	KUNIT_EXPECT_FALSE(test, folio_test_swapcache(folio));
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, -ENOMEM);
	denied = atomic_long_read(&memcg->memory_events[MEMCG_SWAP_MAX]);
	KUNIT_EXPECT_GT(test, denied, before);
	KUNIT_EXPECT_EQ(test, page_counter_read(&memcg->swap), 0UL);
	/* Exercise vmscan's failed-allocation/split fallback without a retained GET. */
	for (tries = 0; tries < 8; tries++)
		test_coarse_anon_reclaim_once(vma);
	KUNIT_EXPECT_GT(test, atomic_long_read(&memcg->memory_events[MEMCG_SWAP_MAX]), denied);
	pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_TRUE(test, pte_present(pte));
	KUNIT_EXPECT_EQ(test, pte_phys_mm(vma->vm_mm, pte), original);
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	KUNIT_EXPECT_EQ(test, folio_size(folio), (unsigned long)SZ_64K);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0xc9, SZ_64K), NULL);
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)nr);
	folio_put(folio);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_ANONPAGES), 1L);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_SWAPENTS), 0L);
	KUNIT_EXPECT_EQ(test, page_counter_read(&memcg->swap), 0UL);
	KUNIT_ASSERT_EQ(test, test_swap_control("--coarse-swap-unlimit"), 0);
	for (tries = 0; tries < 100; tries++) {
		test_coarse_anon_reclaim_once(vma);
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
		if (!pte_present(pte))
			break;
	}
	KUNIT_ASSERT_FALSE(test, pte_present(pte));
	entry = softleaf_from_pte(pte);
	KUNIT_ASSERT_TRUE(test, softleaf_is_swap(entry));
	KUNIT_EXPECT_EQ(test, page_counter_read(&memcg->swap), (unsigned long)nr);
	for (i = 0; i < nr; i++) {
		part = entry;
		part.val += i;
		KUNIT_EXPECT_EQ(test, swp_swapcount(part), 1);
		KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(part));
	}
	fault = test_file_fault(vma, TEST_VA, false);
	KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), 0U);
	KUNIT_EXPECT_TRUE(test, fault & VM_FAULT_MAJOR);
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0xc9, SZ_64K), NULL);
	KUNIT_EXPECT_EQ(test, folio_size(folio), (unsigned long)SZ_64K);
	kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
	lru_add_drain_all();
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
	KUNIT_EXPECT_EQ(test, page_counter_read(&memcg->swap), 0UL);
#else
	kunit_skip(test, "requires memory cgroups");
#endif
}

struct coarse_swap_worker {
	struct mm_struct *mm;
	struct task_struct *task;
	struct completion ready, go, entered, done;
	u8 *buffer;
	unsigned int id;
	bool *disabled;
	int error;
};

static int coarse_swap_worker_run(void *arg)
{
	struct coarse_swap_worker *w = arg;
	unsigned int i;
	u8 value = 0x91 + w->id;

	complete(&w->ready);
	if (!wait_for_completion_timeout(&w->go, 15 * HZ) || kthread_should_stop()) {
		w->error = -ETIMEDOUT;
		goto out;
	}
	if (!w->mm) {
		complete(&w->entered);
		w->error = test_swap_control("--swapoff-helper");
		if (!w->error)
			WRITE_ONCE(*w->disabled, true);
		goto out;
	}
	kthread_use_mm(w->mm);
	complete(&w->entered);
	if (copy_from_user(w->buffer, (void __user *)TEST_VA, SZ_64K)) {
		w->error = 1;
		goto unuse;
	}
	for (i = 0; i < SZ_64K; i++) {
		if (w->buffer[i] != (u8)(0x31 + i / SZ_4K)) {
			w->error = 2;
			goto unuse;
		}
	}
	if (copy_to_user((void __user *)(TEST_VA + SZ_64K - 1), &value, 1) ||
	    copy_from_user(w->buffer, (void __user *)TEST_VA, SZ_64K)) {
		w->error = 3;
		goto unuse;
	}
	for (i = 0; i < SZ_64K; i++) {
		u8 expected = i == SZ_64K - 1 ? value : 0x31 + i / SZ_4K;

		if (w->buffer[i] != expected) {
			w->error = 4;
			break;
		}
	}
unuse:
	kthread_unuse_mm(w->mm);
out:
	complete(&w->done);
	while (!kthread_should_stop())
		schedule_timeout_interruptible(1);
	return 0;
}

static void coarse_swap_worker_stop(void *arg)
{
	struct coarse_swap_worker *w = arg;

	if (w->task) {
		complete_all(&w->go);
		kthread_stop(w->task);
		w->task = NULL;
	}
}

static void user64k_anon_swap_race_test(struct kunit *test)
{
	unsigned int round, nr = SZ_64K / PAGE_SIZE;

	if (!test_coarse_swap_enabled(test))
		return;
	if (num_online_cpus() < 2 || !system_supports_64kb_granule()) {
		kunit_skip(test, "requires two CPUs and 64K TTBR0 support");
		return;
	}
	for (round = 0; round < 8; round++) {
		struct vm_area_struct *vma;
		struct mm_struct *child;
		struct coarse_swap_worker *workers;
		struct folio *folio, *blocker;
		struct swap_info_struct *si;
		swp_entry_t entry, part;
		bool *disabled, started = true, blocked;
		bool off_started = false, cold = round & 1;
		unsigned int i, j, tries;
		pte_t pte;
		int ret, cpu;

		disabled = kunit_kzalloc(test, sizeof(*disabled), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, disabled);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_reenable_swap, disabled), 0);
		vma = test_fault_vma_granule(test, 16, TEST_VA, SZ_64K);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		folio = test_fault_folio(vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		for (i = 0; i < SZ_64K / SZ_4K; i++)
			memset(folio_address(folio) + i * SZ_4K, 0x31 + i, SZ_4K);
		child = test_vma_mm_granule(test, 16);
		KUNIT_ASSERT_NOT_NULL(test, child);
		uprobe_start_dup_mmap();
		ret = dup_mmap(child, vma->vm_mm);
		uprobe_end_dup_mmap();
		if (!ret) {
			folio_lock(folio);
			ret = folio_alloc_swap(folio);
			if (!ret)
				try_to_unmap(folio, TTU_SYNC);
			folio_unlock(folio);
		}
		folio_put(folio);
		KUNIT_ASSERT_EQ(test, ret, 0);
		pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_FALSE(test, pte_present(pte));
		entry = softleaf_from_pte(pte);
		KUNIT_ASSERT_TRUE(test, softleaf_is_swap(entry));
		for (i = 0; i < nr; i++) {
			part = entry;
			part.val += i;
			KUNIT_EXPECT_EQ(test, swp_swapcount(part), 2);
			if (cold)
				KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(part));
		}
		part = entry;
		part.val += nr - 1;
		{
			struct swap_io_ctx ctx = {};

			blocker = read_swap_cache_async(&ctx, part, GFP_KERNEL, vma,
							TEST_VA + SZ_64K - PAGE_SIZE);
			swap_read_submit(&ctx);
		}
		KUNIT_ASSERT_NOT_NULL(test, blocker);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, blocker), 0);
		KUNIT_EXPECT_EQ(test, folio_size(blocker), cold ? PAGE_SIZE : (unsigned long)SZ_64K);
		workers = kunit_kcalloc(test, 3, sizeof(*workers), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, workers);
		cpu = cpumask_first(cpu_online_mask);
		for (i = 0; i < 3; i++) {
			workers[i].mm = i < 2 ? (i ? child : vma->vm_mm) : NULL;
			workers[i].id = i;
			workers[i].disabled = disabled;
			workers[i].buffer = kunit_kmalloc(test, SZ_64K, GFP_KERNEL);
			KUNIT_ASSERT_NOT_NULL(test, workers[i].buffer);
			init_completion(&workers[i].ready);
			init_completion(&workers[i].go);
			init_completion(&workers[i].entered);
			init_completion(&workers[i].done);
			workers[i].task = kthread_create(coarse_swap_worker_run, &workers[i],
							"user64k-swap-%u", i);
			if (IS_ERR(workers[i].task)) {
				workers[i].task = NULL;
				KUNIT_FAIL(test, "coarse swap worker creation failed");
				return;
			}
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
						coarse_swap_worker_stop, &workers[i]), 0);
			if (i < 2) {
				kthread_bind(workers[i].task, cpu);
				cpu = cpumask_next(cpu, cpu_online_mask);
			}
			wake_up_process(workers[i].task);
		}
		for (i = 0; i < 3; i++)
			KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&workers[i].ready, 10 * HZ), 0UL);
		folio_lock(blocker);
		for (i = 0; i < 3; i++)
			complete(&workers[i].go);
		for (i = 0; i < 3; i++)
			started &= !!wait_for_completion_timeout(&workers[i].entered, HZ);
		for (tries = 0; tries < 3 * HZ; tries++) {
			si = get_swap_device(entry);
			if (si) {
				off_started = !(READ_ONCE(si->flags) & SWP_WRITEOK);
				put_swap_device(si);
			}
			if (off_started)
				break;
			schedule_timeout_uninterruptible(1);
		}
		blocked = !completion_done(&workers[0].done) &&
			  !completion_done(&workers[1].done) && !completion_done(&workers[2].done);
		/* Never execute a fatal assertion while retaining the blocking lock. */
		folio_unlock(blocker);
		KUNIT_EXPECT_TRUE(test, started);
		KUNIT_EXPECT_TRUE(test, off_started);
		KUNIT_EXPECT_TRUE(test, blocked);
		for (i = 0; i < 3; i++) {
			KUNIT_ASSERT_NE(test, wait_for_completion_timeout(&workers[i].done, 20 * HZ), 0UL);
			KUNIT_EXPECT_EQ_MSG(test, workers[i].error, 0, "round=%u worker=%u", round, i);
			kunit_release_action(test, coarse_swap_worker_stop, &workers[i]);
		}
		KUNIT_EXPECT_TRUE(test, *disabled);
		for (i = 0; i < 2; i++) {
			struct mm_struct *mm = i ? child : vma->vm_mm;

			folio = test_fault_folio(mm, TEST_VA);
			KUNIT_ASSERT_NOT_NULL(test, folio);
			KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
			for (j = 0; j < SZ_64K; j++) {
				u8 expected = j == SZ_64K - 1 ? 0x91 + i : 0x31 + j / SZ_4K;

				if (((u8 *)folio_address(folio))[j] != expected) {
					KUNIT_FAIL(test, "round=%u side=%u byte=%u", round, i, j);
					break;
				}
			}
			KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_SWAPENTS), 0L);
			KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 1L);
			kunit_release_action(test, user4k_vma_mm_free, mm);
			lru_add_drain_all();
			KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
			KUNIT_EXPECT_EQ(test, folio_ref_count(folio), folio == blocker ? 2 : 1);
			kunit_release_action(test, test_folio_put, folio);
		}
		KUNIT_EXPECT_FALSE(test, folio_test_swapcache(blocker));
		KUNIT_EXPECT_EQ(test, folio_ref_count(blocker), 1);
		kunit_release_action(test, test_folio_put, blocker);
		kunit_release_action(test, test_reenable_swap, disabled);
		KUNIT_ASSERT_GT(test, total_swap_pages, 0UL);
	}
	kunit_info(test, "8 controlled warm/cold fault+COW+swapoff rounds completed");
}

struct coarse_swap_reservation {
	struct folio **folios;
	unsigned int count;
};

static void test_coarse_release_reservation(struct folio *folio)
{
	folio_lock(folio);
	folio_put_swap(folio, NULL);
	folio_free_swap(folio);
	folio_unlock(folio);
	folio_put(folio);
}

static void test_coarse_release_reservations(void *arg)
{
	struct coarse_swap_reservation *r = arg;
	unsigned int i;

	for (i = 0; i < r->count; i++) {
		if (!r->folios[i])
			continue;
		test_coarse_release_reservation(r->folios[i]);
		r->folios[i] = NULL;
		cond_resched();
	}
}

static void test_coarse_swap_refusal(struct kunit *test, struct vm_area_struct *vma,
				     phys_addr_t original)
{
	struct folio *folio = test_fault_folio(vma->vm_mm, TEST_VA);
	pte_t pte;
	unsigned int i;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, folio);
	folio_lock(folio);
	ret = folio_alloc_swap(folio);
	KUNIT_EXPECT_FALSE(test, folio_test_swapcache(folio));
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, -ENOMEM);
	for (i = 0; i < 4; i++)
		test_coarse_anon_reclaim_once(vma);
	pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_TRUE(test, pte_present(pte));
	KUNIT_EXPECT_EQ(test, pte_phys_mm(vma->vm_mm, pte), original);
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	KUNIT_EXPECT_EQ(test, folio_size(folio), (unsigned long)SZ_64K);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0xd6, SZ_64K), NULL);
	folio_put(folio);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_ANONPAGES), 1L);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(vma->vm_mm, MM_SWAPENTS), 0L);
}

static void user64k_anon_swap_fragment_test(struct kunit *test)
{
	struct coarse_swap_reservation *r;
	struct vm_area_struct *vma;
	struct folio *folio;
	unsigned int i, nr = SZ_64K / PAGE_SIZE, released = 0;
	unsigned long hole, free_before;
	phys_addr_t original;
	swp_entry_t entry, part;
	vm_fault_t fault;
	pte_t pte;
	int ret = 0;

	if (!test_coarse_swap_enabled(test))
		return;
	/* Exhaust only the bounded disposable disk owned by this test's PID1. */
	KUNIT_ASSERT_EQ(test, test_swap_control("--coarse-fixture-check"), 0);
	KUNIT_ASSERT_LT(test, total_swap_pages, (unsigned long)(SZ_256M / PAGE_SIZE));
	KUNIT_ASSERT_GT(test, total_swap_pages, 2UL * SWAPFILE_CLUSTER);
	vma = test_fault_vma_granule(test, 16, TEST_VA, SZ_64K);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	original = page_to_phys(&folio->page);
	memset(folio_address(folio), 0xd6, SZ_64K);
	folio_put(folio);
	r = kunit_kzalloc(test, sizeof(*r), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, r);
	r->folios = kunit_kcalloc(test, total_swap_pages + 1, sizeof(*r->folios), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, r->folios);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_coarse_release_reservations, r), 0);
	/* Each cache-only folio has one explicit slot owner, so allocation cannot reclaim it. */
	for (i = 0; i <= total_swap_pages; i++) {
		folio = folio_alloc(GFP_KERNEL | __GFP_ZERO, 0);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		ret = mem_cgroup_charge(folio, vma->vm_mm, GFP_KERNEL);
		if (ret) {
			folio_put(folio);
			KUNIT_FAIL(test, "reservation memory charge failed");
			return;
		}
		__folio_set_swapbacked(folio);
		__folio_mark_uptodate(folio);
		folio_lock(folio);
		ret = folio_alloc_swap(folio);
		if (!ret) {
			ret = folio_dup_swap(folio, NULL);
			if (ret)
				folio_free_swap(folio);
		}
		folio_unlock(folio);
		if (ret) {
			folio_put(folio);
			break;
		}
		r->folios[r->count++] = folio;
		cond_resched();
	}
	KUNIT_ASSERT_EQ(test, ret, -ENOMEM);
	KUNIT_ASSERT_EQ(test, atomic_long_read(&nr_swap_pages), 0L);
	test_coarse_swap_refusal(test, vma, original);
	/* Retain exactly one native entry in each naturally aligned coarse group. */
	for (i = 0; i < r->count; i++) {
		folio = r->folios[i];
		if (swp_offset(folio->swap) % nr == 1)
			continue;
		test_coarse_release_reservation(folio);
		r->folios[i] = NULL;
	}
	free_before = atomic_long_read(&nr_swap_pages);
	KUNIT_EXPECT_GT(test, free_before, total_swap_pages / 2);
	KUNIT_EXPECT_LT(test, free_before, total_swap_pages);
	test_coarse_swap_refusal(test, vma, original);
	/* Free one aligned group near the end, without freeing a whole native-order cluster. */
	hole = round_down(total_swap_pages + 1, (unsigned long)SWAPFILE_CLUSTER) - nr;
	for (i = 0; i < r->count; i++) {
		folio = r->folios[i];
		if (!folio || swp_offset(folio->swap) < hole || swp_offset(folio->swap) >= hole + nr)
			continue;
		test_coarse_release_reservation(folio);
		r->folios[i] = NULL;
		released++;
	}
	KUNIT_ASSERT_EQ(test, released, 1U);
	KUNIT_EXPECT_EQ(test, atomic_long_read(&nr_swap_pages), (long)free_before + 1);
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	folio_lock(folio);
	ret = folio_alloc_swap(folio);
	if (!ret)
		try_to_unmap(folio, TTU_SYNC);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ_MSG(test, ret, 0, "a complete aligned group is free in an order-0 cluster");
	pte = test_fault_entry(test, vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_FALSE(test, pte_present(pte));
	entry = softleaf_from_pte(pte);
	KUNIT_ASSERT_TRUE(test, softleaf_is_swap(entry));
	KUNIT_EXPECT_EQ(test, swp_offset(entry), hole);
	for (i = 0; i < r->count; i++) {
		if (r->folios[i])
			KUNIT_EXPECT_EQ(test, swp_swapcount(r->folios[i]->swap), 1);
	}
	kunit_release_action(test, test_coarse_release_reservations, r);
	for (i = 0; i < nr; i++) {
		part = entry;
		part.val += i;
		KUNIT_EXPECT_EQ(test, swp_swapcount(part), 1);
		KUNIT_ASSERT_TRUE(test, test_reclaim_swapcache(part));
	}
	fault = test_file_fault(vma, TEST_VA, false);
	KUNIT_ASSERT_EQ(test, fault & (VM_FAULT_ERROR | VM_FAULT_RETRY), 0U);
	KUNIT_EXPECT_TRUE(test, fault & VM_FAULT_MAJOR);
	folio = test_fault_folio(vma->vm_mm, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, folio);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0xd6, SZ_64K), NULL);
	kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
	lru_add_drain_all();
	KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
	KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
	kunit_info(test, "exhaustion, scattered free space and one-group recovery checked");
}

struct coarse_swap_io_mode {
	struct swap_info_struct *si;
	bool synchronous;
};

static void test_coarse_restore_io_mode(void *arg)
{
	struct coarse_swap_io_mode *mode = arg;

	spin_lock(&mode->si->lock);
	if (mode->synchronous)
		mode->si->flags |= SWP_SYNCHRONOUS_IO;
	else
		mode->si->flags &= ~SWP_SYNCHRONOUS_IO;
	spin_unlock(&mode->si->lock);
	put_swap_device(mode->si);
}

static void user64k_anon_swap_extent_test(struct kunit *test)
{
	unsigned int mode, i;

	if (!test_coarse_swap_enabled(test))
		return;
	KUNIT_ASSERT_EQ(test, test_swap_control("--coarse-fixture-check"), 0);
	for (mode = 0; mode < 2; mode++) {
		struct vm_area_struct *vma;
		struct coarse_swap_io_mode *saved;
		struct folio *folio;
		unsigned long nr = SZ_64K / PAGE_SIZE, run = nr;
		swp_entry_t entry;
		sector_t first, next;
		bool file;
		int ret;

		vma = test_fault_vma_granule(test, 16, TEST_VA, SZ_64K);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
		folio = test_fault_folio(vma->vm_mm, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, folio);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_folio_put, folio), 0);
		for (i = 0; i < SZ_64K / SZ_4K; i++)
			memset(folio_address(folio) + i * SZ_4K, 0x63 + i, SZ_4K);
		folio_lock(folio);
		ret = folio_alloc_swap(folio);
		if (!ret)
			try_to_unmap(folio, TTU_SYNC);
		folio_unlock(folio);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_ASSERT_EQ(test, folio_mapcount(folio), 0);
		entry = folio->swap;
		saved = kunit_kzalloc(test, sizeof(*saved), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, saved);
		saved->si = get_swap_device(entry);
		KUNIT_ASSERT_NOT_NULL(test, saved->si);
		saved->synchronous = saved->si->flags & SWP_SYNCHRONOUS_IO;
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_coarse_restore_io_mode, saved), 0);
		file = !(saved->si->flags & SWP_BLKDEV);
		KUNIT_ASSERT_PTR_EQ(test, saved->si->ops, &swap_bdev_ops);
		first = swap_extent_sector(entry, &run);
		if (file) {
			unsigned long remaining;
			swp_entry_t part = entry;

			KUNIT_ASSERT_LT_MSG(test, run, nr, "fragmented fixture must cross an extent");
			part.val += run;
			remaining = nr - run;
			next = swap_extent_sector(part, &remaining);
			KUNIT_EXPECT_NE(test, next, first + (run << (PAGE_SHIFT - 9)));
		} else {
			KUNIT_EXPECT_EQ(test, run, nr);
		}
		/* The verified disposable fixture owns this otherwise idle swap area. */
		spin_lock(&saved->si->lock);
		if (mode)
			saved->si->flags |= SWP_SYNCHRONOUS_IO;
		else
			saved->si->flags &= ~SWP_SYNCHRONOUS_IO;
		spin_unlock(&saved->si->lock);
		folio_lock(folio);
		folio_clear_dirty_for_io(folio);
		{
			struct swap_io_ctx ctx = {};

			__swap_writepage(&ctx, folio);
			swap_write_submit(&ctx);
		}
		folio_wait_writeback(folio);
		folio_lock(folio);
		KUNIT_EXPECT_FALSE(test, folio_test_dirty(folio));
		/* A full-folio read must reconstruct all physical runs, not warm cache. */
		memset(folio_address(folio), 0, SZ_64K);
		folio_clear_uptodate(folio);
		{
			struct swap_io_ctx ctx = {};

			swap_read_folio(&ctx, folio);
			swap_read_submit(&ctx);
		}
		folio_lock(folio);
		KUNIT_EXPECT_TRUE(test, folio_test_uptodate(folio));
		for (i = 0; i < SZ_64K / SZ_4K; i++)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + i * SZ_4K,
							0x63 + i, SZ_4K), NULL);
		folio_unlock(folio);
		kunit_release_action(test, test_coarse_restore_io_mode, saved);
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, false) & VM_FAULT_ERROR, 0U);
		kunit_release_action(test, user4k_vma_mm_free, vma->vm_mm);
		lru_add_drain_all();
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), 0);
		KUNIT_EXPECT_EQ(test, folio_ref_count(folio), 1);
		kunit_info(test, "64K full-folio %s I/O: file=%u first-run=%lu/%lu native pages",
			   mode ? "synchronous" : "asynchronous", file, run, nr);
	}
}

static struct kunit_case user4k_coarse_shmem_cases[] = {
	KUNIT_CASE(user64k_anon_swap_extent_test),
	KUNIT_CASE(user64k_anon_swap_test),
	KUNIT_CASE(user64k_anon_swap_limit_test),
	KUNIT_CASE(user64k_anon_swap_fragment_test),
	KUNIT_CASE(user64k_anon_swap_race_test),
	KUNIT_CASE(user64k_shmem_cold_constituents_test),
	KUNIT_CASE(user64k_shmem_cold_promoted_test),
	KUNIT_CASE(user64k_shmem_fault_alloc_fail_test),
	{}
};

static struct kunit_suite user4k_coarse_shmem_suite = {
	.name = "arm64-user4k-coarse-shmem",
	.test_cases = user4k_coarse_shmem_cases,
	.suite_exit = user4k_swap_suite_exit,
};

static struct kunit_case user4k_shmem_swap_cases[] = {
	KUNIT_CASE(user4k_shmem_uffd_swap_alloc_fail_test),
	KUNIT_CASE(user4k_shmem_uffd_swap_charge_test),
	KUNIT_CASE(user4k_shmem_uffd_swapin_test),
	KUNIT_CASE(user4k_shmem_uffd_swapped_truncate_test),
	KUNIT_CASE(user4k_shmem_uffd_swapoff_test),
	KUNIT_CASE(user4k_shmem_uffd_swapped_partial_hole_test),
	{}
};

static struct kunit_suite user4k_shmem_swap_suite = {
	.name = "arm64-user4k-shmem-swap",
	.test_cases = user4k_shmem_swap_cases,
	.suite_exit = user4k_swap_suite_exit,
};
#endif

static struct kunit_case user4k_nonlinear_swap_cases[] = {
#ifdef CONFIG_USERFAULTFD
	KUNIT_CASE(user4k_move_swap_get_test),
#ifdef CONFIG_MIGRATION
	KUNIT_CASE(user4k_move_migration_race_test),
#endif
#endif
	KUNIT_CASE(user4k_nonlinear_swap_cached_test),
	KUNIT_CASE(user4k_nonlinear_swap_disk_test),
	{}
};

static struct kunit_suite user4k_nonlinear_swap_suite = {
	.name = "arm64-user4k-nonlinear-swap",
	.test_cases = user4k_nonlinear_swap_cases,
	.suite_exit = user4k_swap_suite_exit,
};

static struct kunit_suite user4k_swap_suite = {
	.name = "arm64-user4k-swap",
	.test_cases = user4k_swap_cases,
	.suite_exit = user4k_swap_suite_exit,
};

static void user4k_thp_eligibility_test(struct kunit *test)
{
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	struct mm_struct *small = test_vma_mm(test), *native = mm_alloc();
	const enum tva_type types[] = {
		TVA_PAGEFAULT, TVA_SMAPS, TVA_KHUGEPAGED, TVA_FORCED_COLLAPSE,
	};
	struct vm_area_struct *vma, *normal;
	vm_flags_t flags = VM_READ | VM_WRITE | VM_MAYREAD | VM_MAYWRITE |
			   VM_HUGEPAGE;

	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
			user4k_vma_mm_free, native), 0);
	KUNIT_ASSERT_NOT_NULL(test, small);
	vma = test_vma_add(small, SZ_1G, SZ_1G, (struct vm_page_offset) {}, flags);
	normal = test_vma_add(native, SZ_1G, SZ_1G, (struct vm_page_offset) {}, flags);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, vma);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, normal);
	/* Neither user advice nor forced collapse can bypass geometry limits. */
	KUNIT_EXPECT_TRUE(test, vma_thp_disabled(vma, flags, false));
	KUNIT_EXPECT_TRUE(test, vma_thp_disabled(vma, flags, true));
	KUNIT_EXPECT_FALSE(test, vma_thp_disabled(normal, flags, false));
	KUNIT_EXPECT_FALSE(test, vma_thp_disabled(normal, flags, true));
	for (unsigned int i = 0; i < ARRAY_SIZE(types); i++)
		KUNIT_EXPECT_EQ(test, __thp_vma_allowable_orders(vma, flags,
				types[i], THP_ORDERS_ALL_ANON), 0UL);
#else
	kunit_skip(test, "requires transparent hugepage support");
#endif
}

static void user4k_aslr_window_test(struct kunit *test)
{
	struct mm_struct *small = test_vma_mm(test), *native = mm_alloc();
	const unsigned long windows[] = { SZ_8M, SZ_32M, SZ_1G };

	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
			user4k_vma_mm_free, native), 0);
	KUNIT_ASSERT_NOT_NULL(test, small);
	for (unsigned int i = 0; i < ARRAY_SIZE(windows); i++) {
		unsigned long mask = (windows[i] >> PAGE_SHIFT) - 1;
		unsigned long converted = mm_aslr_mask(small, mask);

		KUNIT_EXPECT_EQ(test, mm_aslr_mask(native, mask), mask);
		KUNIT_EXPECT_EQ(test, (converted + 1) << mm_page_shift(small),
				windows[i]);
		KUNIT_EXPECT_EQ(test, converted & (TEST_SLOTS - 1),
				(unsigned long)TEST_SLOTS - 1);
	}
}

static void user4k_exec_granule_policy_test(struct kunit *test)
{
	/* Model all native granules, including AArch32 with NO 4K support. */
	const unsigned int native[] = { 12, 14, 16 };

	for (unsigned int i = 0; i < ARRAY_SIZE(native); i++) {
		KUNIT_EXPECT_EQ(test, arm64_exec_page_shift(true, false, native[i], 0),
				native[i]);
		KUNIT_EXPECT_EQ(test, arm64_exec_page_shift(true, false, native[i], 12),
				native[i]);
		KUNIT_EXPECT_EQ(test, arm64_exec_page_shift(true, true, native[i], 0),
				12U);
		KUNIT_EXPECT_EQ(test, arm64_exec_page_shift(false, true, native[i], 0),
				native[i]);
		KUNIT_EXPECT_EQ(test, arm64_exec_page_shift(false, true, native[i], 12),
				12U);
	}
}

/* Exec page-size requests are refused when booted with user4k=off. */
static void user4k_page_size_request_test(struct kunit *test)
{
	static const struct {
		unsigned long size;
		int off, on, on_no4k;
	} cases[] = {
		{ 0, 0, 0, 0 },
		{ PAGE_SIZE, 0, 0, 0 },
		{ SZ_4K, -EINVAL, 12, -EOPNOTSUPP },
		{ SZ_8K, -EINVAL, -EINVAL, -EINVAL },
		{ SZ_2M, -EINVAL, -EINVAL, -EINVAL },
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cases); i++) {
		unsigned long size = cases[i].size;

		KUNIT_EXPECT_EQ(test, arm64_exec_page_size_request(size, false, true, true),
				cases[i].off);
		KUNIT_EXPECT_EQ(test, arm64_exec_page_size_request(size, false, false, false),
				cases[i].off);
		KUNIT_EXPECT_EQ(test, arm64_exec_page_size_request(size, true, true, true),
				cases[i].on);
		KUNIT_EXPECT_EQ(test, arm64_exec_page_size_request(size, true, false, false),
				cases[i].on_no4k);
	}
	/* Larger-than-native process granules are never offered. */
	if (PAGE_SIZE == SZ_16K) {
		KUNIT_EXPECT_EQ(test, arm64_exec_page_size_request(SZ_64K, false, true, true),
				-EINVAL);
		KUNIT_EXPECT_EQ(test, arm64_exec_page_size_request(SZ_64K, true, true, true),
				-EINVAL);
	}
}

asmlinkage long __arm64_sys_prctl(const struct pt_regs *regs);

static long test_prctl(unsigned long option, unsigned long arg2)
{
	struct pt_regs regs = { .regs = { option, arg2 }, .orig_x0 = option };

	return __arm64_sys_prctl(&regs);
}

/* Both SET prctls follow the boot switch; native requests always succeed. */
static void user4k_boot_switch_prctl_test(struct kunit *test)
{
	static const unsigned long options[][2] = {
		{ PR_AURORA_SET_EXEC_PAGE_SIZE, PR_AURORA_GET_EXEC_PAGE_SIZE },
		{ PR_AURORA_SET_DEFAULT_PAGE_SIZE, PR_AURORA_GET_DEFAULT_PAGE_SIZE },
	};
	unsigned int saved_exec = current->exec_page_shift;
	unsigned int saved_default = current->default_exec_page_shift;
	bool has_4k = cpus_have_final_cap(ARM64_HAS_USER4K_GRANULE);
	long expect = !user4k_enabled() ? -EINVAL : has_4k ? 0 : -EOPNOTSUPP;
	unsigned int i;

	/* With user4k=off the CPUs report no 4K user granule either. */
	if (!user4k_enabled())
		KUNIT_EXPECT_FALSE(test, has_4k);
	for (i = 0; i < ARRAY_SIZE(options); i++) {
		unsigned long set = options[i][0], get = options[i][1];

		current->exec_page_shift = 0;
		current->default_exec_page_shift = 0;
		KUNIT_EXPECT_EQ(test, test_prctl(set, SZ_4K), expect);
		KUNIT_EXPECT_EQ(test, test_prctl(get, 0), expect ? 0L : (long)SZ_4K);
		if (PAGE_SIZE == SZ_16K) {
			KUNIT_EXPECT_EQ(test, test_prctl(set, SZ_64K), (long)-EINVAL);
			KUNIT_EXPECT_EQ(test, test_prctl(get, 0), expect ? 0L : (long)SZ_4K);
		}
		KUNIT_EXPECT_EQ(test, test_prctl(set, PAGE_SIZE), 0L);
		KUNIT_EXPECT_EQ(test, test_prctl(set, 0), 0L);
		KUNIT_EXPECT_EQ(test, test_prctl(get, 0), 0L);
	}
	current->exec_page_shift = saved_exec;
	current->default_exec_page_shift = saved_default;
}

/* With the switch off, exec ignores any preference and stays native. */
static void user4k_boot_switch_exec_test(struct kunit *test)
{
	unsigned int saved_exec = current->exec_page_shift;
	unsigned int saved_default = current->default_exec_page_shift;
	struct mm_struct *mm;
	unsigned int compat;

	mm = mm_alloc_exec_page_shift(12);
	KUNIT_ASSERT_NOT_NULL(test, mm);
	KUNIT_EXPECT_EQ(test, mm_page_shift(mm), user4k_enabled() ? 12U : PAGE_SHIFT);
	mmput(mm);
	/* With user4k enabled, user4k_exec_secure_granule_test covers staging. */
	if (user4k_enabled())
		return;
	for (compat = 0; compat < 2; compat++) {
		struct linux_binprm *bprm = kunit_kzalloc(test, sizeof(*bprm), GFP_KERNEL);
		int ret;

		KUNIT_ASSERT_NOT_NULL(test, bprm);
		mm = mm_alloc_exec_page_shift(PAGE_SHIFT);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		bprm->mm = mm;
		current->exec_page_shift = 12;
		current->default_exec_page_shift = 12;
		ret = arm64_bprm_prepare_mm(bprm, compat);
		current->exec_page_shift = saved_exec;
		current->default_exec_page_shift = saved_default;
		KUNIT_EXPECT_EQ(test, ret, 0);
		KUNIT_EXPECT_PTR_EQ(test, bprm->mm, mm);
		KUNIT_EXPECT_EQ(test, mm_page_shift(bprm->mm), PAGE_SHIFT);
		/* AArch32 also keeps the native granule. */
		KUNIT_EXPECT_EQ(test, bprm->secure_page_shift, PAGE_SHIFT);
		mmput(mm);
	}
}

/* Native pages never have a 4K owner; with user4k=off there is no record. */
static void user4k_boot_switch_page_ext_test(struct kunit *test)
{
	struct mm_subpage_mapcounts counts;
	struct page *page = alloc_page(GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, page);
	memset(&counts, 0xff, sizeof(counts));
	KUNIT_EXPECT_EQ(test, mm_subpage_file_mapcounts(page, &counts), 0U);
	for (i = 0; i < MM_SUBPAGES_PER_PAGE; i++)
		KUNIT_EXPECT_EQ(test, counts.slots[i], 0U);
	KUNIT_EXPECT_NULL(test, mm_subpage_get_from_phys(page_to_phys(page)));
	if (!user4k_enabled())
		KUNIT_EXPECT_EQ(test, mm_subpage_shmem_missing(page), 0UL);
	__free_page(page);
}

static void user_granule_zero_region_test(struct kunit *test)
{
	unsigned long base_pfn = zero_pfn(0);
	unsigned int shift, i;
	u8 *buffer;

	KUNIT_EXPECT_TRUE(test, IS_ALIGNED(PFN_PHYS(base_pfn), SZ_64K));
	KUNIT_EXPECT_FALSE(test, is_zero_pfn(base_pfn - 1));
	KUNIT_EXPECT_FALSE(test, is_zero_pfn(base_pfn + SZ_64K / PAGE_SIZE));
	for (i = 0; i < SZ_64K / PAGE_SIZE; i++) {
		KUNIT_EXPECT_TRUE(test, is_zero_pfn(base_pfn + i));
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(page_address(pfn_to_page(base_pfn + i)),
						   0, PAGE_SIZE), NULL);
	}
	buffer = kunit_kmalloc(test, SZ_64K, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, buffer);
	for (shift = 12; shift <= 16; shift += 2) {
		struct vm_area_struct *vma;
		struct mm_struct *mm;
		unsigned long size = 1UL << shift, left;
		vm_fault_t ret;
		spinlock_t *ptl;
		pte_t *pte, entry;

		if ((shift == 12 && !system_supports_4kb_granule()) ||
		    (shift == 14 && !system_supports_16kb_granule()) ||
		    (shift == 16 && !system_supports_64kb_granule())) {
			kunit_info(test, "hardware zero mapping skipped for shift %u", shift);
			continue;
		}
		/* Internal coarse mm: the public opt-in remains disabled. */
		mm = test_vma_mm_granule(test, shift);
		KUNIT_ASSERT_NOT_NULL(test, mm);
		vma = test_fault_vma_in_mm(mm, TEST_VA, size);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		mmap_read_lock(mm);
		ret = handle_mm_fault(vma, TEST_VA + size - 1, FAULT_FLAG_REMOTE, NULL);
		mmap_read_unlock(mm);
		KUNIT_ASSERT_EQ(test, ret, (vm_fault_t)0);
		pte = test_mm_lookup(mm, TEST_VA, &ptl);
		KUNIT_ASSERT_NOT_NULL(test, pte);
		entry = ptep_get(pte);
		pte_unmap_unlock(pte, ptl);
		KUNIT_EXPECT_TRUE(test, pte_present(entry));
		KUNIT_EXPECT_TRUE(test, pte_special(entry));
		KUNIT_EXPECT_FALSE(test, pte_write(entry));
		KUNIT_EXPECT_EQ(test, pte_phys_mm(mm, entry), PFN_PHYS(base_pfn));
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 0L);

		memset(buffer, 0xa5, size);
		kthread_use_mm(mm);
		left = copy_from_user(buffer, (void __user *)TEST_VA, size);
		kthread_unuse_mm(mm);
		KUNIT_EXPECT_EQ(test, left, 0UL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(buffer, 0, size), NULL);
		mmap_read_lock(mm);
		zap_vma_range(vma, TEST_VA, size);
		mmap_read_unlock(mm);
		KUNIT_EXPECT_EQ(test, get_mm_counter_sum(mm, MM_ANONPAGES), 0L);
	}
}

#ifdef CONFIG_COREDUMP
static void user_granule_core_offsets_test(struct kunit *test)
{
	struct mm_struct *mm = kunit_kzalloc(test, sizeof(*mm), GFP_KERNEL);
	struct vm_area_struct *vma = kunit_kzalloc(test, sizeof(*vma), GFP_KERNEL);
	unsigned int shift;

	KUNIT_ASSERT_NOT_NULL(test, mm);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	vma->vm_mm = mm;
	for (shift = 12; shift <= 16; shift += 2) {
		unsigned long bytes = (1UL << 55) + 3 * SZ_64K + (1UL << shift);

		mm->context.flags = shift == 12 ? MMCF_USER_4K :
				    shift == 14 ? MMCF_USER_16K : MMCF_USER_64K;
		vma_set_page_offset(vma, (struct vm_page_offset) {
			.index = bytes >> PAGE_SHIFT, .offset = bytes & ~PAGE_MASK,
		});
		KUNIT_EXPECT_EQ(test, core_vma_pgoff(vma), bytes >> shift);
		KUNIT_EXPECT_EQ(test, mm_user_fragment_size(mm), min(1UL << shift, PAGE_SIZE));
		vma_set_page_offset(vma, (struct vm_page_offset) {});
		KUNIT_EXPECT_EQ(test, core_vma_pgoff(vma), 0UL);
	}
}
#endif

static void user64k_fragment_ownership_test(struct kunit *test)
{
	struct user_page_fragment fragments[SZ_64K / PAGE_SIZE] = {};
	struct mm_struct *parent, *child;
	struct vm_area_struct *vma, *cvma;
	struct folio *original, *copied;
	unsigned int i, nr = ARRAY_SIZE(fragments);
	long count;
	int ret, refs;
	pte_t entry;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires a user leaf larger than native pages");
		return;
	}
	parent = test_vma_mm_granule(test, 16);
	KUNIT_ASSERT_NOT_NULL(test, parent);
	vma = test_fault_vma_in_mm(parent, TEST_VA, SZ_64K);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	count = get_user_fragments_remote(parent, TEST_VA + 17, SZ_64K - 34,
					 0, fragments, nr);
	if (count > 0) {
		for (i = 0; i < count; i++) {
			struct user_page_fragment *f = &fragments[i];
			unsigned int bytes = PAGE_SIZE - (i == 0 || i == nr - 1 ? 17 : 0);

			KUNIT_EXPECT_TRUE(test, is_zero_folio(f->folio));
			KUNIT_EXPECT_EQ(test, folio_pfn(f->folio), zero_pfn(TEST_VA) + i);
			KUNIT_EXPECT_PTR_EQ(test, f->subpage, NULL);
			KUNIT_EXPECT_EQ(test, f->offset, i == 0 ? 17UL : 0UL);
			KUNIT_EXPECT_EQ(test, f->length, bytes);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(f->folio) + f->offset,
							   0, f->length), NULL);
		}
		release_user_fragments(fragments, count, false);
	}
	KUNIT_ASSERT_EQ(test, count, (long)nr);

	KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA, true) & VM_FAULT_ERROR, 0U);
	entry = test_fault_entry(test, parent, TEST_VA);
	original = page_folio(pfn_to_page(pte_pfn(entry)));
	for (i = 0; i < nr; i++)
		memset(page_address(folio_page(original, i)), 0x51 + i, PAGE_SIZE);
	lru_add_drain_all();
	refs = folio_ref_count(original);
	count = get_user_fragments_remote(parent, TEST_VA + 17, SZ_64K - 34,
					 0, fragments, nr);
	if (count > 0) {
		KUNIT_EXPECT_EQ(test, folio_ref_count(original), refs + (int)count);
		for (i = 0; i < count; i++) {
			struct user_page_fragment *f = &fragments[i];

			KUNIT_EXPECT_PTR_EQ(test, f->folio, original);
			KUNIT_EXPECT_EQ(test, f->offset, i * PAGE_SIZE + (i == 0 ? 17UL : 0UL));
			KUNIT_EXPECT_LE(test, offset_in_page(f->offset) + f->length, PAGE_SIZE);
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(f->folio) + f->offset,
							   0x51 + i, f->length), NULL);
		}
		release_user_fragments(fragments, count, false);
	}
	KUNIT_ASSERT_EQ(test, count, (long)nr);
	KUNIT_EXPECT_EQ(test, folio_ref_count(original), refs);

	child = test_vma_mm_granule(test, 16);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	cvma = test_vma_lookup(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, cvma);
	/* A read pin after fork must unshare the complete coarse leaf. */
	count = pin_user_fragments_remote(child, TEST_VA, SZ_64K, 0, fragments, nr);
	if (count > 0) {
		copied = fragments[0].folio;
		KUNIT_EXPECT_PTR_NE(test, copied, original);
		KUNIT_EXPECT_TRUE(test, folio_maybe_dma_pinned(copied));
		KUNIT_EXPECT_FALSE(test, pte_write(test_fault_entry(test, child, TEST_VA)));
		for (i = 0; i < count; i++) {
			KUNIT_EXPECT_PTR_EQ(test, fragments[i].folio, copied);
			KUNIT_EXPECT_PTR_EQ(test, fragments[i].subpage, NULL);
			KUNIT_EXPECT_TRUE(test, fragments[i].pinned);
			KUNIT_EXPECT_TRUE(test, PageAnonExclusive(folio_page(copied, i)));
			KUNIT_EXPECT_EQ(test, fragments[i].offset, i * PAGE_SIZE);
			KUNIT_EXPECT_EQ(test, fragments[i].length, (unsigned int)PAGE_SIZE);
		}
		mmap_read_lock(child);
		zap_vma_range(cvma, TEST_VA, SZ_64K);
		mmap_read_unlock(child);
		KUNIT_EXPECT_EQ(test, folio_mapcount(copied), 0);
		for (i = 0; i < count; i++)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(copied) + i * PAGE_SIZE,
							   0x51 + i, PAGE_SIZE), NULL);
		release_user_fragments(fragments, count, false);
	}
	KUNIT_ASSERT_EQ(test, count, (long)nr);
	for (i = 0; i < nr; i++)
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(original) + i * PAGE_SIZE,
						   0x51 + i, PAGE_SIZE), NULL);
}

static void user64k_anon_fault_cow_test(struct kunit *test)
{
	struct mm_struct *parent, *child;
	struct vm_area_struct *vma, *cvma;
	pte_t orig[2], copied;
	struct folio *folio;
	unsigned int i, j, nr = SZ_64K / PAGE_SIZE;
	int ret;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires coarse anonymous faults");
		return;
	}
	parent = test_vma_mm_granule(test, 16);
	KUNIT_ASSERT_NOT_NULL(test, parent);
	vma = test_fault_vma_in_mm(parent, TEST_VA, 2 * SZ_64K);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	/* Leaf0 exercises zero-page COW; leaf1 exercises a direct write fault. */
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA + SZ_64K - 1, false) & VM_FAULT_ERROR, 0U);
	KUNIT_EXPECT_TRUE(test, pte_special(test_fault_entry(test, parent, TEST_VA)));
	for (i = 0; i < 2; i++) {
		unsigned long addr = TEST_VA + i * SZ_64K;

		KUNIT_ASSERT_EQ(test, test_file_fault(vma, addr + SZ_64K - 1, true) & VM_FAULT_ERROR, 0U);
		orig[i] = test_fault_entry(test, parent, addr);
		KUNIT_ASSERT_TRUE(test, pte_present(orig[i]));
		KUNIT_EXPECT_TRUE(test, pte_write(orig[i]));
		KUNIT_EXPECT_FALSE(test, pte_special(orig[i]));
		folio = page_folio(pfn_to_page(pte_pfn(orig[i])));
		KUNIT_EXPECT_EQ(test, folio_size(folio), (unsigned long)SZ_64K);
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)nr);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0, SZ_64K), NULL);
		for (j = 0; j < nr; j++) {
			KUNIT_EXPECT_TRUE(test, PageAnonExclusive(folio_page(folio, j)));
			memset(page_address(folio_page(folio, j)), 0x31 + i * nr + j, PAGE_SIZE);
		}
	}
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(parent, MM_ANONPAGES), 2L);
	child = test_vma_mm_granule(test, 16);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	cvma = test_vma_lookup(child, TEST_VA);
	KUNIT_ASSERT_NOT_NULL(test, cvma);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_ANONPAGES), 2L);
	for (i = 0; i < 2; i++) {
		unsigned long addr = TEST_VA + i * SZ_64K;
		struct folio *old = page_folio(pfn_to_page(pte_pfn(orig[i])));

		KUNIT_EXPECT_EQ(test, pte_phys_mm(child, test_fault_entry(test, child, addr)),
				pte_phys_mm(parent, orig[i]));
		KUNIT_EXPECT_FALSE(test, pte_write(test_fault_entry(test, parent, addr)));
		KUNIT_EXPECT_EQ(test, folio_mapcount(old), (int)(2 * nr));
		KUNIT_ASSERT_EQ(test, test_file_fault(cvma, addr + SZ_64K - 1, true) & VM_FAULT_ERROR, 0U);
		copied = test_fault_entry(test, child, addr);
		KUNIT_EXPECT_NE(test, pte_phys_mm(child, copied), pte_phys_mm(parent, orig[i]));
		folio = page_folio(pfn_to_page(pte_pfn(copied)));
		KUNIT_EXPECT_EQ(test, folio_mapcount(old), (int)nr);
		KUNIT_EXPECT_EQ(test, folio_mapcount(folio), (int)nr);
		for (j = 0; j < nr; j++) {
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(page_address(folio_page(folio, j)),
							   0x31 + i * nr + j, PAGE_SIZE), NULL);
			KUNIT_EXPECT_TRUE(test, PageAnonExclusive(folio_page(folio, j)));
		}
		memset(folio_address(folio), 0xa5, SZ_64K);
		for (j = 0; j < nr; j++)
			KUNIT_EXPECT_PTR_EQ(test, memchr_inv(page_address(folio_page(old, j)),
							   0x31 + i * nr + j, PAGE_SIZE), NULL);
		/* Parent reuse must restore exclusivity across the entire leaf. */
		lru_add_drain_all();
		KUNIT_ASSERT_EQ(test, test_file_fault(vma, addr + SZ_64K - 1, true) & VM_FAULT_ERROR, 0U);
		KUNIT_EXPECT_EQ(test, pte_phys_mm(parent, test_fault_entry(test, parent, addr)),
				pte_phys_mm(parent, orig[i]));
		for (j = 0; j < nr; j++)
			KUNIT_EXPECT_TRUE(test, PageAnonExclusive(folio_page(old, j)));
	}
	if (system_supports_64kb_granule()) {
		long protect[2];
		bool writable = false;

		kthread_use_mm(parent);
		protect[0] = test_mprotect(TEST_VA, 2 * SZ_64K, PROT_READ);
		kthread_unuse_mm(parent);
		for (i = 0; i < 2; i++)
			writable |= pte_write(test_fault_entry(test, parent, TEST_VA + i * SZ_64K));
		KUNIT_EXPECT_EQ(test, protect[0], 0L);
		KUNIT_EXPECT_FALSE(test, writable);
		kthread_use_mm(parent);
		protect[1] = test_mprotect(TEST_VA, 2 * SZ_64K, PROT_READ | PROT_WRITE);
		kthread_unuse_mm(parent);
		KUNIT_EXPECT_EQ(test, protect[1], 0L);
		for (i = 0; i < 2; i++)
			KUNIT_EXPECT_TRUE(test, pte_write(test_fault_entry(test, parent, TEST_VA + i * SZ_64K)));
		vma = test_vma_lookup(parent, TEST_VA);
		KUNIT_ASSERT_NOT_NULL(test, vma);
	}
	mmap_read_lock(child);
	zap_vma_range(cvma, TEST_VA, 2 * SZ_64K);
	mmap_read_unlock(child);
	mmap_read_lock(parent);
	zap_vma_range(vma, TEST_VA, 2 * SZ_64K);
	mmap_read_unlock(parent);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_ANONPAGES), 0L);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(parent, MM_ANONPAGES), 0L);
}

struct coarse_test_pin {
	struct page *page;
};

static void coarse_test_unpin(void *arg)
{
	struct coarse_test_pin *pin = arg;

	if (pin->page) {
		unpin_user_page(pin->page);
		pin->page = NULL;
	}
}

static void user64k_pinned_fork_test(struct kunit *test)
{
	struct mm_struct *parent, *child;
	struct vm_area_struct *vma;
	struct coarse_test_pin *pin;
	struct folio *old, *new;
	pte_t original, copied;
	unsigned int i, nr = SZ_64K / PAGE_SIZE;
	int ret;

	if (PAGE_SHIFT >= 16) {
		kunit_skip(test, "requires pinned coarse leaf");
		return;
	}
	parent = test_vma_mm_granule(test, 16);
	KUNIT_ASSERT_NOT_NULL(test, parent);
	vma = test_fault_vma_in_mm(parent, TEST_VA, SZ_64K);
	KUNIT_ASSERT_NOT_NULL(test, vma);
	KUNIT_ASSERT_EQ(test, test_file_fault(vma, TEST_VA + SZ_64K - 1, true) & VM_FAULT_ERROR, 0U);
	original = test_fault_entry(test, parent, TEST_VA);
	old = page_folio(pfn_to_page(pte_pfn(original)));
	for (i = 0; i < nr; i++)
		memset(page_address(folio_page(old, i)), 0x70 + i, PAGE_SIZE);
	pin = kunit_kzalloc(test, sizeof(*pin), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, pin);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, coarse_test_unpin, pin), 0);
	mm_flags_set(MMF_HAS_PINNED, parent);
	ret = try_grab_folio(old, 1, FOLL_PIN);
	if (!ret)
		pin->page = folio_page(old, nr - 1);
	KUNIT_ASSERT_EQ(test, ret, 0);
	child = test_vma_mm_granule(test, 16);
	KUNIT_ASSERT_NOT_NULL(test, child);
	uprobe_start_dup_mmap();
	ret = dup_mmap(child, parent);
	uprobe_end_dup_mmap();
	KUNIT_ASSERT_EQ(test, ret, 0);
	copied = test_fault_entry(test, child, TEST_VA);
	KUNIT_EXPECT_NE(test, pte_phys_mm(child, copied), pte_phys_mm(parent, original));
	KUNIT_EXPECT_EQ(test, pte_phys_mm(parent, test_fault_entry(test, parent, TEST_VA)),
			pte_phys_mm(parent, original));
	new = page_folio(pfn_to_page(pte_pfn(copied)));
	KUNIT_EXPECT_EQ(test, folio_mapcount(old), (int)nr);
	KUNIT_EXPECT_EQ(test, folio_mapcount(new), (int)nr);
	for (i = 0; i < nr; i++) {
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(page_address(folio_page(new, i)),
						   0x70 + i, PAGE_SIZE), NULL);
		KUNIT_EXPECT_TRUE(test, PageAnonExclusive(folio_page(new, i)));
		KUNIT_EXPECT_TRUE(test, PageAnonExclusive(folio_page(old, i)));
	}
	memset(page_address(pin->page), 0xe3, PAGE_SIZE);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(page_address(folio_page(new, nr - 1)),
					   0x70 + nr - 1, PAGE_SIZE), NULL);
	coarse_test_unpin(pin);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(parent, MM_ANONPAGES), 1L);
	KUNIT_EXPECT_EQ(test, get_mm_counter_sum(child, MM_ANONPAGES), 1L);
}

#if defined(CONFIG_SHMEM) && defined(CONFIG_MM_SUBPAGE) && defined(CONFIG_USERFAULTFD)
static void user4k_shmem_partial_hole_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, 3 * PAGE_SIZE, true);
	struct folio *folio;
	loff_t start = PAGE_SIZE + SZ_4K + 17, end = PAGE_SIZE + 3 * SZ_4K + 16;
	unsigned int i;
	unsigned char byte = 0xa5;
	loff_t write_pos = PAGE_SIZE + 2 * SZ_4K + 23;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	shmem_truncate_range(file_inode(file), start, end);
	folio = filemap_lock_folio(file->f_mapping, 1);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 1), BIT(2));
	for (i = 0; i < PAGE_SIZE; i++) {
		unsigned char expected = (i + PAGE_SIZE >= start && i + PAGE_SIZE <= end) ?
			0 : 0x31 + (PAGE_SIZE + i) / SZ_4K;

		if (((unsigned char *)folio_address(folio))[i] != expected) {
			KUNIT_FAIL(test, "partial hole changed byte %u outside its range", i);
			break;
		}
	}
	/* Resident hole metadata has no dynamically allocated inode record. */
	KUNIT_EXPECT_TRUE(test, xa_empty(&SHMEM_I(file_inode(file))->uffd_missing));
	folio_unlock(folio);
	folio_put(folio);
	shmem_truncate_range(file_inode(file), PAGE_SIZE + SZ_4K, PAGE_SIZE + 2 * SZ_4K - 1);
	KUNIT_ASSERT_EQ(test, kernel_write(file, &byte, 1, &write_pos), 1L);
	folio = filemap_lock_folio(file->f_mapping, 1);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 1), BIT(1));
	KUNIT_EXPECT_EQ(test, ((unsigned char *)folio_address(folio))[2 * SZ_4K + 23], byte);
	folio_unlock(folio);
	folio_put(folio);
}

/* A surviving writable pin exposes bytes even after its PTE is revoked. */
static void user4k_shmem_pinned_hole_test(struct kunit *test)
{
	unsigned int native;

	for (native = 0; native < 2; native++) {
		struct file *file = test_cache_file(test, PAGE_SIZE, true);
		struct vm_area_struct *vma;
		struct fragment_test_pins *pins;
		struct folio *folio;
		unsigned long offset = native ? 0 : SZ_4K;
		unsigned long length = native ? PAGE_SIZE : SZ_4K;
		unsigned long pfn;

		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
		vma = test_file_vma(test, file, native, true, TEST_VA, length, offset);
		KUNIT_ASSERT_NOT_NULL(test, vma);
		pins = kunit_kzalloc(test, sizeof(*pins), GFP_KERNEL);
		KUNIT_ASSERT_NOT_NULL(test, pins);
		KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, fragment_test_unpin, pins), 0);
		KUNIT_ASSERT_EQ(test, fragment_test_pin(pins, vma->vm_mm, TEST_VA,
				length, FOLL_WRITE | FOLL_LONGTERM, 1), 1L);
		folio = pins->fragments[0].folio;
		pfn = folio_pfn(folio);
		KUNIT_ASSERT_TRUE(test, folio_maybe_dma_pinned(folio));
		shmem_truncate_range(file_inode(file), SZ_4K, 2 * SZ_4K - 1);
		KUNIT_EXPECT_FALSE(test, pte_present(test_fault_entry(test, vma->vm_mm, TEST_VA)));
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K, 0, SZ_4K), NULL);
		/* Model a completed device write through the original pin. */
		memset(folio_address(folio) + SZ_4K, 0xb7, SZ_4K);
		folio_lock(folio);
		KUNIT_EXPECT_FALSE(test, shmem_uffd_range_missing(folio, 0, SZ_4K, SZ_4K));
		folio_unlock(folio);
		fragment_test_unpin(pins);
		folio = filemap_lock_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_EQ(test, folio_pfn(folio), pfn);
		KUNIT_EXPECT_FALSE(test, shmem_uffd_range_missing(folio, 0, SZ_4K, SZ_4K));
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K, 0xb7, SZ_4K), NULL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0x31, SZ_4K), NULL);
		KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + 2 * SZ_4K, 0x33, SZ_4K), NULL);
		folio_unlock(folio);
		folio_put(folio);
		/* With the external writer gone, another hole can be missing. */
		shmem_truncate_range(file_inode(file), SZ_4K, 2 * SZ_4K - 1);
		folio = filemap_lock_folio(file->f_mapping, 0);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_TRUE(test, shmem_uffd_range_missing(folio, 0, SZ_4K, SZ_4K));
		folio_unlock(folio);
		folio_put(folio);
	}
}

static void test_unpin_folio(void *folio)
{
	unpin_folio(folio);
}

/* The direct memfd API must publish cache exposure without a user PTE. */
static void user4k_shmem_memfd_pin_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, PAGE_SIZE, true);
	struct folio *pin, *folio;
	pgoff_t offset;
	long ret;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	shmem_truncate_range(file_inode(file), SZ_4K, 2 * SZ_4K - 1);
	ret = memfd_pin_folios(file, SZ_4K, 2 * SZ_4K - 1, &pin, 1, &offset);
	KUNIT_ASSERT_EQ(test, ret, 1L);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test, test_unpin_folio, pin), 0);
	KUNIT_EXPECT_EQ(test, offset, (pgoff_t)SZ_4K);
	KUNIT_EXPECT_TRUE(test, folio_maybe_dma_pinned(pin));
	memset(folio_address(pin) + offset, 0xc8, SZ_4K);
	folio_lock(pin);
	KUNIT_EXPECT_FALSE(test, shmem_uffd_range_missing(pin, 0, SZ_4K, SZ_4K));
	folio_unlock(pin);
	kunit_release_action(test, test_unpin_folio, pin);
	folio = filemap_lock_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_EXPECT_FALSE(test, shmem_uffd_range_missing(folio, 0, SZ_4K, SZ_4K));
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K, 0xc8, SZ_4K), NULL);
	folio_unlock(folio);
	folio_put(folio);
}

static void user4k_shmem_replace_validity_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, PAGE_SIZE, true);
	struct folio *old, *new;
	unsigned long old_pfn;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	shmem_truncate_range(file_inode(file), SZ_4K, 2 * SZ_4K - 1);
	new = folio_alloc(GFP_KERNEL, 0);
	KUNIT_ASSERT_NOT_NULL(test, new);
	folio_lock(new);
	old = filemap_lock_folio(file->f_mapping, 0);
	if (IS_ERR(old)) {
		folio_unlock(new);
		folio_put(new);
		KUNIT_FAIL(test, "could not lock source cache folio");
		return;
	}
	old_pfn = folio_pfn(old);
	memcpy(folio_address(new), folio_address(old), PAGE_SIZE);
	__folio_set_swapbacked(new);
	__folio_mark_uptodate(new);
	folio_mark_dirty(new);
	replace_page_cache_folio(old, new);
	folio_add_lru(new);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(new, 0), BIT(1));
	KUNIT_EXPECT_NE(test, folio_pfn(new), old_pfn);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(new), 0x31, SZ_4K), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(new) + SZ_4K, 0, SZ_4K), NULL);
	folio_unlock(old);
	folio_put(old);
	folio_unlock(new);
	folio_put(new);
}

static void user4k_shmem_uffd_alias_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, PAGE_SIZE, false);
	const unsigned long all = GENMASK(PAGE_SIZE / SZ_4K - 1, 0);
	struct vm_area_struct *small, *native;
	struct folio *folio;
	int ret;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	ret = shmem_get_folio(file_inode(file), 0, 0, &folio, SGP_CACHE);
	KUNIT_ASSERT_EQ(test, ret, 0);
	ret = shmem_uffd_set_missing(folio, 0, all);
	KUNIT_EXPECT_TRUE(test, shmem_uffd_has_missing_native_page(folio));
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	small = test_file_vma(test, file, false, true, TEST_VA, SZ_4K, SZ_4K);
	KUNIT_ASSERT_NOT_NULL(test, small);
	KUNIT_ASSERT_EQ(test, test_file_fault(small, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	folio = filemap_lock_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 0), all & ~BIT(1));
	KUNIT_EXPECT_FALSE(test, shmem_uffd_has_missing_native_page(folio));
	folio_unlock(folio);
	folio_put(folio);

	native = test_file_vma(test, file, true, true, TEST_VA, PAGE_SIZE, 0);
	KUNIT_ASSERT_NOT_NULL(test, native);
	KUNIT_ASSERT_EQ(test, test_file_fault(native, TEST_VA, false) & VM_FAULT_ERROR, 0U);
	folio = filemap_lock_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 0), 0UL);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_EXPECT_TRUE(test, xa_empty(&SHMEM_I(file_inode(file))->uffd_missing));
}

static void user4k_shmem_uffd_state_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, 3 * PAGE_SIZE, false);
	const unsigned long all = GENMASK(PAGE_SIZE / SZ_4K - 1, 0);
	struct folio *folio;
	unsigned long mask;
	unsigned int i;
	int ret;

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	for (i = 0; i < 3; i++) {
		ret = shmem_get_folio(file_inode(file), i, 0, &folio, SGP_CACHE);
		KUNIT_ASSERT_EQ(test, ret, 0);
		KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, i), 0UL);
		ret = shmem_uffd_set_missing(folio, i, all & ~BIT(0));
		KUNIT_EXPECT_EQ(test, ret, 0);
		shmem_uffd_mark_present(folio, i, BIT(1));
		mask = shmem_uffd_missing_mask(folio, i);
		KUNIT_EXPECT_EQ(test, mask, all & ~GENMASK(1, 0));
		ret = shmem_uffd_set_missing(folio, i, BIT(PAGE_SIZE / SZ_4K));
		KUNIT_EXPECT_EQ(test, ret, -EINVAL);
		KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, i), mask);
		folio_unlock(folio);
		folio_put(folio);
	}
	/* Real truncate removes one index without discarding neighbouring state. */
	shmem_truncate_range(file_inode(file), PAGE_SIZE, 2 * PAGE_SIZE - 1);
	KUNIT_EXPECT_PTR_EQ(test, xa_load(&SHMEM_I(file_inode(file))->uffd_missing, 1), NULL);
	for (i = 0; i < 3; i += 2) {
		folio = filemap_lock_folio(file->f_mapping, i);
		KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
		KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, i), all & ~GENMASK(1, 0));
		folio_unlock(folio);
		folio_put(folio);
	}
	/* Generic batch deletion must not strand inode metadata either. */
	truncate_inode_pages(file->f_mapping, 0);
	KUNIT_EXPECT_TRUE(test, xa_empty(&SHMEM_I(file_inode(file))->uffd_missing));
	ret = shmem_get_folio(file_inode(file), 1, 0, &folio, SGP_CACHE);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 1), 0UL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0, PAGE_SIZE), NULL);
	folio_unlock(folio);
	folio_put(folio);
}

#ifdef CONFIG_MIGRATION
static void user4k_shmem_uffd_migration_test(struct kunit *test)
{
	struct file *file = test_cache_file(test, PAGE_SIZE, false);
	const unsigned long missing = GENMASK(PAGE_SIZE / SZ_4K - 1, 1);
	struct folio *folio;
	unsigned long old_pfn;
	unsigned int succeeded = 0;
	bool isolated;
	int ret;
	LIST_HEAD(folios);

	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, file);
	ret = shmem_get_folio(file_inode(file), 0, 0, &folio, SGP_CACHE);
	KUNIT_ASSERT_EQ(test, ret, 0);
	memset(folio_address(folio), 0xa5, SZ_4K);
	folio_mark_dirty(folio);
	ret = shmem_uffd_set_missing(folio, 0, missing);
	old_pfn = folio_pfn(folio);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_ASSERT_EQ(test, ret, 0);
	lru_add_drain_all();
	folio = filemap_get_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	isolated = isolate_folio_to_list(folio, &folios);
	folio_put(folio);
	KUNIT_ASSERT_TRUE(test, isolated);
	ret = migrate_pages(&folios, test_migration_alloc, NULL, 0, MIGRATE_SYNC,
			    MR_SYSCALL, &succeeded);
	putback_movable_pages(&folios);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_ASSERT_EQ(test, succeeded, 1U);
	folio = filemap_lock_folio(file->f_mapping, 0);
	KUNIT_ASSERT_NOT_ERR_OR_NULL(test, folio);
	KUNIT_EXPECT_NE(test, folio_pfn(folio), old_pfn);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 0), missing);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio), 0xa5, SZ_4K), NULL);
	KUNIT_EXPECT_PTR_EQ(test, memchr_inv(folio_address(folio) + SZ_4K, 0,
					   PAGE_SIZE - SZ_4K), NULL);
	shmem_uffd_mark_present(folio, 0, missing);
	KUNIT_EXPECT_EQ(test, shmem_uffd_missing_mask(folio, 0), 0UL);
	folio_unlock(folio);
	folio_put(folio);
	KUNIT_EXPECT_TRUE(test, xa_empty(&SHMEM_I(file_inode(file))->uffd_missing));
}
#endif
#endif

static struct kunit_case user4k_test_cases[] = {
	KUNIT_CASE(user4k_exec_granule_policy_test),
	KUNIT_CASE(user4k_aslr_window_test),
	KUNIT_CASE(user4k_thp_eligibility_test),
	KUNIT_CASE(user4k_geometry_test),
	KUNIT_CASE(user4k_leaf_encoding_test),
	KUNIT_CASE(user16k_leaf_encoding_test),
	KUNIT_CASE(user4k_rmap_walk_test),
	KUNIT_CASE(user4k_rmap_boundary_test),
	KUNIT_CASE(native_rmap_walk_test),
	KUNIT_CASE(user64k_rmap_lookup_test),
	KUNIT_CASE(user64k_anon_rmap_span_test),
	KUNIT_CASE(user64k_anon_zap_batch_test),
	KUNIT_CASE(user_granule_zero_region_test),
#ifdef CONFIG_COREDUMP
	KUNIT_CASE(user_granule_core_offsets_test),
#endif
	KUNIT_CASE(user64k_fragment_ownership_test),
	KUNIT_CASE(user64k_anon_fault_cow_test),
	KUNIT_CASE(user64k_pinned_fork_test),
	KUNIT_CASE(user64k_file_rmap_span_test),
	KUNIT_CASE(user4k_vma_offsets_test),
	KUNIT_CASE(user4k_vma_intervals_test),
	KUNIT_CASE(user4k_vma_split_test),
	KUNIT_CASE(user4k_vma_merge_test),
	KUNIT_CASE(user4k_vma_partial_merge_test),
	KUNIT_CASE(user4k_vma_copy_shrink_test),
	KUNIT_CASE(user4k_anon_rmap_test),
	KUNIT_CASE(user4k_anon_fork_rmap_test),
	KUNIT_CASE(user4k_nonlinear_rmap_reuse_test),
	KUNIT_CASE(user4k_dup_mmap_test),
	KUNIT_CASE(user4k_anon_root_lifetime_test),
	KUNIT_CASE(user4k_data_tlb_batch_test),
	KUNIT_CASE(user4k_anon_zap_test),
	KUNIT_CASE(user4k_anon_writable_slots_test),
	KUNIT_CASE(user4k_anon_write_fault_test),
	KUNIT_CASE(user16k_anon_write_fault_test),
	KUNIT_CASE(user4k_mlock_counts_test),
	KUNIT_CASE(user4k_mlock_shared_test),
	KUNIT_CASE(user4k_mlock_repair_test),
	KUNIT_CASE(user_granule_large_vma_split_test),
	KUNIT_CASE(user_granule_brk_extent_test),
#ifdef CONFIG_MIGRATION
	KUNIT_CASE(user4k_droppable_reclaim_test),
	KUNIT_CASE(user4k_lazyfree_test),
#endif
	KUNIT_CASE(user4k_zero_cow_test),
	KUNIT_CASE(user16k_zero_cow_test),
	KUNIT_CASE(user4k_shmem_swap_usage_test),
#if defined(CONFIG_SHMEM) && defined(CONFIG_MM_SUBPAGE) && defined(CONFIG_USERFAULTFD)
	KUNIT_CASE(user4k_shmem_partial_hole_test),
	KUNIT_CASE(user4k_shmem_pinned_hole_test),
	KUNIT_CASE(user4k_shmem_memfd_pin_test),
	KUNIT_CASE(user4k_shmem_replace_validity_test),
	KUNIT_CASE(user4k_shmem_uffd_alias_test),
	KUNIT_CASE(user4k_shmem_uffd_state_test),
#ifdef CONFIG_MIGRATION
	KUNIT_CASE(user4k_shmem_uffd_migration_test),
#endif
#endif
	KUNIT_CASE(user4k_file_mapcounts_test),
	KUNIT_CASE(user64k_file_promote_test),
	KUNIT_CASE(user64k_shmem_fault_test),
	KUNIT_CASE(user64k_shmem_fault_busy_test),
	KUNIT_CASE(user64k_shmem_fault_hole_test),
	KUNIT_CASE(user64k_file_leaf_test),
	KUNIT_CASE(user64k_file_cow_test),
	KUNIT_CASE(user64k_file_bounds_test),
	KUNIT_CASE(user4k_file_shared_test),
	KUNIT_CASE(user4k_iommufd_lifetime_test),
	KUNIT_CASE(user4k_iommufd_rollback_test),
	KUNIT_CASE(user4k_iommufd_granule_test),
	KUNIT_CASE(user4k_iommufd_shared_test),
	KUNIT_CASE(user4k_iommufd_exit_account_test),
	KUNIT_CASE(user4k_iova_bitmap_test),
	KUNIT_CASE(user4k_iova_bitmap_window_test),
	KUNIT_CASE(user4k_iova_bitmap_retained_test),
	KUNIT_CASE(user4k_iova_bitmap_partial_test),
	KUNIT_CASE(user4k_mremap_move_test),
	KUNIT_CASE(user4k_mremap_resize_test),
	KUNIT_CASE(user4k_stack_relocate_test),
	KUNIT_CASE(user4k_exec_arguments_test),
	KUNIT_CASE(user4k_exec_privileged_test),
	KUNIT_CASE(user4k_exec_secure_granule_test),
	KUNIT_CASE(user4k_hugetlb_refused_test),
	KUNIT_CASE(user4k_special_mapping_test),
	KUNIT_CASE(native_callback_mapping_test),
	KUNIT_CASE(user4k_callback_mapping_test),
	KUNIT_CASE(user16k_callback_mapping_test),
	KUNIT_CASE(user4k_insert_fragment_test),
	KUNIT_CASE(user16k_insert_shared_test),
	KUNIT_CASE(user4k_insert_range_test),
	KUNIT_CASE(user4k_pfn_offset_test),
	KUNIT_CASE(user4k_remap_pfn_range_test),
	KUNIT_CASE(user4k_iomem_access_test),
#if defined(CONFIG_GENERIC_GETTIMEOFDAY) && defined(CONFIG_VDSO_GETRANDOM)
	KUNIT_CASE(user4k_vdso_mapping_test),
#endif
	KUNIT_CASE(user4k_mremap_cow_test),
	KUNIT_CASE(user4k_mremap_file_test),
	KUNIT_CASE(user4k_msync_offsets_test),
	KUNIT_CASE(user4k_prefault_dirty_test),
	KUNIT_CASE(user4k_cold_batch_test),
	KUNIT_CASE(user4k_remote_access_test),
	KUNIT_CASE(user4k_remote_file_test),
#ifdef CONFIG_CROSS_MEMORY_ATTACH
	KUNIT_CASE(user4k_process_vm_test),
#endif
#ifdef CONFIG_FUTEX
	KUNIT_CASE(user4k_futex_keys_test),
	KUNIT_CASE(user4k_futex_wake_test),
#endif
	KUNIT_CASE(user4k_mmap_descriptor_test),
	KUNIT_CASE(user4k_mmap_region_offsets_test),
	KUNIT_CASE(user4k_commit_accounting_test),
	KUNIT_CASE(mixed_locked_account_units_test),
	KUNIT_CASE(user64k_commit_accounting_test),
	KUNIT_CASE(user64k_page_table_check_test),
#ifdef CONFIG_PROC_FS
	KUNIT_CASE(user4k_accounting_readers_test),
#endif
	KUNIT_CASE(user_granule_prefault_test),
	KUNIT_CASE(user4k_mmap_accounting_test),
	KUNIT_CASE(user4k_vm_mmap_test),
	KUNIT_CASE(user4k_mprotect_test),
	KUNIT_CASE(user4k_mprotect_cow_batch_test),
#ifdef CONFIG_NUMA_BALANCING
	KUNIT_CASE(user_subpage_numa_cow_batch_test),
#endif
	KUNIT_CASE(user4k_mprotect_accounting_test),
	KUNIT_CASE(user4k_file_cow_test),
	KUNIT_CASE(user4k_file_eof_test),
	KUNIT_CASE(user4k_file_truncate_test),
	KUNIT_CASE(user4k_filemap_fault_test),
	KUNIT_CASE(user4k_file_invalidate_test),
	KUNIT_CASE(user4k_file_fragments_test),
	KUNIT_CASE(user4k_file_fork_hole_test),
	KUNIT_CASE(user4k_swap_encoding_test),
	KUNIT_CASE(user4k_cow_reuse_test),
	KUNIT_CASE(user16k_cow_reuse_test),
	KUNIT_CASE(user4k_fork_cow_test),
	KUNIT_CASE(user16k_fork_cow_test),
	KUNIT_CASE(user4k_slot_pin_test),
	KUNIT_CASE(user4k_fragment_pin_test),
	KUNIT_CASE(user4k_fragment_locked_pin_test),
	KUNIT_CASE(user4k_folio_walk_geometry_test),
	KUNIT_CASE(user4k_fragment_longterm_test),
#ifdef CONFIG_BLOCK
	KUNIT_CASE(user4k_bio_fragments_test),
	KUNIT_CASE(user4k_bio_trim_test),
#endif
	KUNIT_CASE(user4k_fragment_permissions_test),
	KUNIT_CASE(user16k_fragment_offset_test),
#ifdef CONFIG_MIGRATION
	KUNIT_CASE(user4k_migration_test),
	KUNIT_CASE(user4k_file_migration_test),
	KUNIT_CASE(user64k_file_migration_test),
	KUNIT_CASE(user64k_anon_migration_test),
	KUNIT_CASE(user4k_fragment_pin_race_test),
	KUNIT_CASE(user4k_migration_entries_test),
	KUNIT_CASE(user4k_nonlinear_rmap_test),
	KUNIT_CASE(user4k_mprotect_migration_test),
	KUNIT_CASE(user4k_live_migration_test),
	KUNIT_CASE(user4k_migration_wait_test),
#endif
	KUNIT_CASE(user4k_anon_exit_test),
	KUNIT_CASE(user4k_anon_remote_zap_test),
#ifdef CONFIG_ARM64_CONTPTE
	KUNIT_CASE(user4k_contpte_test),
	KUNIT_CASE(user16k_contpte_test),
	KUNIT_CASE(native_contpte_test),
#endif
#ifdef CONFIG_PERF_EVENTS
	KUNIT_CASE(user4k_perf_page_size_test),
	KUNIT_CASE(user16k_perf_page_size_test),
	KUNIT_CASE(native_perf_page_size_test),
#endif
	KUNIT_CASE(user4k_mm_tables_test),
	KUNIT_CASE(user16k_mm_tables_test),
	KUNIT_CASE(user64k_mm_tables_test),
	KUNIT_CASE(native_mm_tables_test),
	KUNIT_CASE(user4k_anon_read_fault_test),
	KUNIT_CASE(user4k_mm_hardware_test),
	KUNIT_CASE(user4k_fragment_lifetime_test),
	KUNIT_CASE(user16k_fragment_lifetime_test),
	KUNIT_CASE(user64k_table_owner_test),
#ifdef CONFIG_TRANSPARENT_HUGEPAGE
	KUNIT_CASE(native_pgtable_deposit_test),
#endif
	KUNIT_CASE(user4k_translation_test),
	KUNIT_CASE(user4k_context_uaccess_test),
	KUNIT_CASE(user4k_fault_uaccess_test),
	{}
};

static struct kunit_suite user4k_test_suite = {
	.name = "arm64-user4k",
	.test_cases = user4k_test_cases,
};

/* Safe with or without user4k=off; the other suites need it on. */
static struct kunit_case user4k_policy_cases[] = {
	KUNIT_CASE(user4k_page_size_request_test),
	KUNIT_CASE(user4k_boot_switch_prctl_test),
	KUNIT_CASE(user4k_boot_switch_exec_test),
	KUNIT_CASE(user4k_boot_switch_page_ext_test),
	{}
};

static struct kunit_suite user4k_policy_suite = {
	.name = "arm64-user4k-policy",
	.test_cases = user4k_policy_cases,
};

#if defined(CONFIG_SHMEM) && defined(CONFIG_MM_SUBPAGE) && defined(CONFIG_USERFAULTFD)
kunit_test_suites(&user4k_policy_suite, &user4k_test_suite, &user4k_swap_suite,
		  &user4k_shmem_swap_suite, &user4k_coarse_shmem_suite,
		  &user4k_nonlinear_swap_suite);
#else
kunit_test_suites(&user4k_policy_suite, &user4k_test_suite, &user4k_swap_suite,
		  &user4k_nonlinear_swap_suite);
#endif

MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("EXPORTED_FOR_KUNIT_TESTING");
