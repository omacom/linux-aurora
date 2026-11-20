// SPDX-License-Identifier: GPL-2.0-only OR MIT
/*
 * Long-term pins of user memory as whole native pages, for GEM_USERPTR.
 *
 * The GPU maps memory in native pages. A process whose user page size is
 * smaller (CONFIG_MM_SUBPAGE) maps each of its pages to a fragment of a
 * native page, and only the fragment API can pin those. A native page can be
 * given to the GPU only where the process maps all of it, its fragments in
 * order at consecutive addresses: then the GPU sees what the CPU sees, and no
 * memory the process does not map.
 */

#include <linux/mm.h>
#include <linux/mm_subpage.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include "user_pages.h"

#ifdef CONFIG_MM_SUBPAGE
/* Pinned fragments of a range; released only through asahi_unpin_user_fragments(). */
struct asahi_user_fragments {
	unsigned long count;
	struct user_page_fragment records[];
};

bool asahi_user_mm_is_subpage(void)
{
	return current->mm && mm_page_size(current->mm) < PAGE_SIZE;
}

void asahi_unpin_user_fragments(struct asahi_user_fragments *frags, bool dirty)
{
	if (!frags)
		return;
	release_user_fragments(frags->records, frags->count, dirty);
	kvfree(frags);
}

/* The native page that @rec and its @n - 1 successors map in order, or NULL. */
static struct page *whole_native_page(const struct user_page_fragment *rec,
				      unsigned long n, unsigned long granule)
{
	unsigned long i;

	if (!IS_ALIGNED(rec->offset, PAGE_SIZE))
		return NULL;
	for (i = 0; i < n; i++)
		if (rec[i].folio != rec->folio || rec[i].length != granule ||
		    rec[i].offset != rec->offset + i * granule)
			return NULL;
	return folio_page(rec->folio, rec->offset >> PAGE_SHIFT);
}

/*
 * Pins @nr_pages native pages' worth of the current process's memory at
 * @start (native-page aligned) and stores the native pages in @pages. All or
 * nothing: -EFAULT if a page cannot be pinned or a native page is not mapped
 * whole and in order; -ENOMEM if the records cannot be allocated.
 */
long asahi_pin_user_fragments(unsigned long start, unsigned long nr_pages,
			      unsigned int gup_flags, struct page **pages,
			      struct asahi_user_fragments **out)
{
	struct mm_struct *mm = current->mm;
	unsigned long granule = mm_page_size(mm);
	unsigned long per_page = PAGE_SIZE / granule;
	unsigned long length = nr_pages << PAGE_SHIFT, done = 0, i;
	struct asahi_user_fragments *frags;
	long ret = 0;

	if (nr_pages > (ULONG_MAX >> PAGE_SHIFT) / per_page)
		return -ENOMEM;
	frags = kvzalloc(struct_size(frags, records, nr_pages * per_page),
			 GFP_KERNEL_ACCOUNT);
	if (!frags)
		return -ENOMEM;
	while (done < length) {
		long n = pin_user_fragments_remote(mm, start + done, length - done,
						   gup_flags, frags->records + frags->count,
						   nr_pages * per_page - frags->count);

		if (n <= 0) {
			ret = n ?: -EFAULT;
			goto fail;
		}
		for (i = frags->count; i < frags->count + n; i++)
			done += frags->records[i].length;
		frags->count += n;
	}
	for (i = 0; i < nr_pages; i++) {
		pages[i] = whole_native_page(&frags->records[i * per_page], per_page, granule);
		if (!pages[i]) {
			ret = -EFAULT;
			goto fail;
		}
	}
	*out = frags;
	return 0;

fail:
	asahi_unpin_user_fragments(frags, false);
	return ret;
}
#else
bool asahi_user_mm_is_subpage(void)
{
	return false;
}

void asahi_unpin_user_fragments(struct asahi_user_fragments *frags, bool dirty)
{
}

long asahi_pin_user_fragments(unsigned long start, unsigned long nr_pages,
			      unsigned int gup_flags, struct page **pages,
			      struct asahi_user_fragments **out)
{
	return -EOPNOTSUPP;
}
#endif
