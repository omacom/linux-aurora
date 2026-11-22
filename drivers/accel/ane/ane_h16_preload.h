/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef __ANE_H16_PRELOAD_H__
#define __ANE_H16_PRELOAD_H__

#include <crypto/sha2.h>

/* The TEXT and DATA sizes of a rule that names none (26.5 to 27.0 layouts). */
#define ANE_H16_PRELOAD_TEXT_SIZE 0xc0000
#define ANE_H16_PRELOAD_DATA_SIZE 0x2ac000

/*
 * One aether patchbay record in DATA: a four-byte tag and a 32-bit width,
 * followed by the value cell at @value. A mutable cell is written by the
 * bootloader and is normalized to zero before hashing.
 */
struct ane_h16_preload_cell {
	char tag[4];
	u32 value;
	u8 width;
	bool mutable;
};

/*
 * How to recognize one firmware build from its iBoot preload: the ordered
 * patchbay records, the DATA range the bootloader rewrites (@zero_start up to
 * @zero_end, normalized to zero), and the SHA-256 of the normalized TEXT and
 * DATA. Adding a firmware build is a new rule, not new code. @text_size and
 * @data_size are the preload sizes the rule was made for; 0 means
 * ANE_H16_PRELOAD_TEXT_SIZE and ANE_H16_PRELOAD_DATA_SIZE.
 */
struct ane_h16_preload_rule {
	const struct ane_h16_preload_cell *cells;
	unsigned int cell_count;
	u32 zero_start;
	u32 zero_end;
	u32 text_size;
	u32 data_size;
	u8 digest[SHA256_DIGEST_SIZE];
};

/* Inputs are stable, host-owned snapshots of the reserved live pages.
 * Computing a digest validates the rule's framing but not the identity.
 * Neither function changes the snapshots or the live bootloader values.
 */
int ane_h16_preload_digest(const struct ane_h16_preload_rule *rule,
			   const void *text, size_t text_size,
			   const void *data, size_t data_size,
			   u8 digest[SHA256_DIGEST_SIZE]);
/* 0 when the snapshots are the rule's build, -ENOEXEC when they are not. */
int ane_h16_preload_validate(const struct ane_h16_preload_rule *rule,
			     const void *text, size_t text_size,
			     const void *data, size_t data_size);

#endif /* __ANE_H16_PRELOAD_H__ */
