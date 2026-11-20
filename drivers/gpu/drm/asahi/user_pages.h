/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _ASAHI_USER_PAGES_H
#define _ASAHI_USER_PAGES_H

#include <linux/types.h>

struct asahi_user_fragments;
struct page;

/* Called from gem.rs. */
bool asahi_user_mm_is_subpage(void);
long asahi_pin_user_fragments(unsigned long start, unsigned long nr_pages,
			      unsigned int gup_flags, struct page **pages,
			      struct asahi_user_fragments **out);
void asahi_unpin_user_fragments(struct asahi_user_fragments *frags, bool dirty);

#endif
