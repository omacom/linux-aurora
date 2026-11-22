// SPDX-License-Identifier: GPL-2.0-only
/* Apple ProRes APRI initialization firmware parser. */
#include <linux/errno.h>
#include <linux/export.h>
#include <linux/kernel.h>
#include <linux/overflow.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include <kunit/visibility.h>

#include "apple-prores.h"

#define APRI_HEADER_SIZE	16
#define APRI_BLOCK_HEADER_SIZE	8
#define APRI_ENTRY_SIZE		12
#define APRI_VERSION		1

static_assert(APRI_T6041_FIRMWARE_SIZE ==
	      APRI_HEADER_SIZE + APRI_T6041_BLOCKS * APRI_BLOCK_HEADER_SIZE +
	      (APRI_SYSTEM_COUNT + APRI_CORE_COUNT + APRI_WRAPPER_COUNT) *
	      APRI_ENTRY_SIZE);
static_assert(APRI_T6050_FIRMWARE_SIZE ==
	      APRI_HEADER_SIZE + APRI_T6050_BLOCKS * APRI_BLOCK_HEADER_SIZE +
	      (APRI_SYSTEM_COUNT + APRI_CORE_COUNT + APRI_WRAPPER_COUNT +
	       APRI_CONTROL_COUNT) * APRI_ENTRY_SIZE);
static_assert(APRI_T8132_FIRMWARE_SIZE ==
	      APRI_HEADER_SIZE + APRI_T6041_BLOCKS * APRI_BLOCK_HEADER_SIZE +
	      (APRI_SYSTEM_COUNT + APRI_CORE_COUNT + APRI_T8132_WRAPPER_COUNT) *
	      APRI_ENTRY_SIZE);

static_assert(APRI_T8142_FIRMWARE_SIZE ==
	      APRI_HEADER_SIZE + APRI_T8142_BLOCKS * APRI_BLOCK_HEADER_SIZE +
	      (APRI_SYSTEM_COUNT + APRI_CORE_COUNT + APRI_T8142_WRAPPER_COUNT +
	       APRI_CONTROL_COUNT) * APRI_ENTRY_SIZE);

const struct apri_layout apri_t6041 = {
	.soc = 0x6041,
	.blocks = APRI_T6041_BLOCKS,
	.size = APRI_T6041_FIRMWARE_SIZE,
	.counts = { APRI_SYSTEM_COUNT, APRI_CORE_COUNT, APRI_WRAPPER_COUNT },
	.limits = { 0x4000, 0x4000, 0x1000000 },
};
EXPORT_SYMBOL_IF_KUNIT(apri_t6041);

/* T6050 adds the SCODEC control window, applied after the system block. */
const struct apri_layout apri_t6050 = {
	.soc = 0x6050,
	.blocks = APRI_T6050_BLOCKS,
	.size = APRI_T6050_FIRMWARE_SIZE,
	.counts = { APRI_SYSTEM_COUNT, APRI_CORE_COUNT, APRI_WRAPPER_COUNT,
		    APRI_CONTROL_COUNT },
	.limits = { 0x4000, 0x4000, 0x1000000, 0x4000 },
};
EXPORT_SYMBOL_IF_KUNIT(apri_t6050);

/* T8132 has T6041's three blocks; its wrapper table is 48 rows. */
const struct apri_layout apri_t8132 = {
	.soc = 0x8132,
	.blocks = APRI_T6041_BLOCKS,
	.size = APRI_T8132_FIRMWARE_SIZE,
	.counts = { APRI_SYSTEM_COUNT, APRI_CORE_COUNT, APRI_T8132_WRAPPER_COUNT },
	.limits = { 0x4000, 0x4000, 0x1000000 },
};
EXPORT_SYMBOL_IF_KUNIT(apri_t8132);

/*
 * T8142 has T6050's four blocks. macOS 27.2.1 on a J815 writes the same 82
 * system, 80 core and 12 control offsets, and 38 wrapper rows.
 */
const struct apri_layout apri_t8142 = {
	.soc = 0x8142,
	.blocks = APRI_T8142_BLOCKS,
	.size = APRI_T8142_FIRMWARE_SIZE,
	.counts = { APRI_SYSTEM_COUNT, APRI_CORE_COUNT, APRI_T8142_WRAPPER_COUNT,
		    APRI_CONTROL_COUNT },
	.limits = { 0x4000, 0x4000, 0x1000000, 0x4000 },
};
EXPORT_SYMBOL_IF_KUNIT(apri_t8142);

static bool apri_offset_seen(const struct apr_init_block *block, u32 offset)
{
	u32 i;

	for (i = 0; i < block->count; i++)
		if (block->entries[i].offset == offset)
			return true;

	return false;
}

int apr_parse_init(const struct apri_layout *layout, const u8 *data,
		   size_t size, struct apr_init_data *init)
{
	size_t pos = APRI_HEADER_SIZE;
	u32 block_index;

	if (!layout || !data || !init || size != layout->size)
		return -EINVAL;
	if (memcmp(data, "APRI", 4) ||
	    get_unaligned_le32(data + 4) != APRI_VERSION ||
	    get_unaligned_le32(data + 8) != layout->soc ||
	    get_unaligned_le32(data + 12) != layout->blocks)
		return -EINVAL;

	memset(init, 0, sizeof(*init));
	for (block_index = 0; block_index < layout->blocks; block_index++) {
		struct apr_init_block *block = &init->blocks[block_index];
		size_t entries_size, end;
		u32 entry_index;

		if (check_add_overflow(pos, (size_t)APRI_BLOCK_HEADER_SIZE, &end) ||
		    end > size)
			return -EINVAL;
		if (get_unaligned_le32(data + pos) != block_index + 1 ||
		    get_unaligned_le32(data + pos + 4) != layout->counts[block_index])
			return -EINVAL;
		pos = end;

		if (check_mul_overflow((size_t)layout->counts[block_index],
				       (size_t)APRI_ENTRY_SIZE, &entries_size) ||
		    check_add_overflow(pos, entries_size, &end) || end > size)
			return -EINVAL;

		for (entry_index = 0;
		     entry_index < layout->counts[block_index]; entry_index++) {
			struct apr_tunable *entry = &block->entries[entry_index];

			entry->offset = get_unaligned_le32(data + pos);
			entry->mask = get_unaligned_le32(data + pos + 4);
			entry->value = get_unaligned_le32(data + pos + 8);
			if (!IS_ALIGNED(entry->offset, sizeof(u32)) ||
			    entry->offset >= layout->limits[block_index] ||
			    !entry->mask || entry->value & ~entry->mask ||
			    apri_offset_seen(block, entry->offset))
				return -EINVAL;
			block->count++;
			pos += APRI_ENTRY_SIZE;
		}
	}

	return pos == size ? 0 : -EINVAL;
}
EXPORT_SYMBOL_IF_KUNIT(apr_parse_init);
