// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <linux/module.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "ane_h16_preload.h"

/*
 * A rule is data, so check its shape before trusting it: records in DATA
 * order, each record header and value inside DATA and clear of the previous
 * value, and the rewritten range after the last value.
 */
static size_t ane_h16_preload_text_size(const struct ane_h16_preload_rule *rule)
{
	return rule->text_size ?: ANE_H16_PRELOAD_TEXT_SIZE;
}

static size_t ane_h16_preload_data_size(const struct ane_h16_preload_rule *rule)
{
	return rule->data_size ?: ANE_H16_PRELOAD_DATA_SIZE;
}

static bool ane_h16_preload_rule_valid(const struct ane_h16_preload_rule *rule)
{
	size_t data_size;
	u32 next = 8;
	unsigned int i;

	if (!rule->cells || !rule->cell_count)
		return false;
	data_size = ane_h16_preload_data_size(rule);
	for (i = 0; i < rule->cell_count; i++) {
		const struct ane_h16_preload_cell *c = &rule->cells[i];

		if (!c->width || c->value < next ||
		    c->value + c->width > data_size)
			return false;
		next = c->value + c->width + 8;
	}
	return rule->zero_start >= next - 8 && rule->zero_start <= rule->zero_end &&
	       rule->zero_end <= data_size;
}

static void ane_h16_hash_zeros(struct sha256_ctx *ctx, size_t length)
{
	static const u8 zero[64];

	while (length) {
		size_t chunk = min(length, sizeof(zero));

		sha256_update(ctx, zero, chunk);
		length -= chunk;
	}
}

int ane_h16_preload_digest(const struct ane_h16_preload_rule *rule,
			   const void *text, size_t text_size,
			   const void *data, size_t data_size,
			   u8 digest[SHA256_DIGEST_SIZE])
{
	const u8 *bytes = data;
	struct sha256_ctx ctx;
	size_t cursor = 0;
	unsigned int i;

	if (!rule || !text || !data || !digest ||
	    text_size != ane_h16_preload_text_size(rule) ||
	    data_size != ane_h16_preload_data_size(rule) || !ane_h16_preload_rule_valid(rule))
		return -EINVAL;
	/* Validate all framing before producing a digest. Odd offsets are valid. */
	for (i = 0; i < rule->cell_count; i++) {
		const struct ane_h16_preload_cell *c = &rule->cells[i];

		if (memcmp(bytes + c->value - 8, c->tag, 4) ||
		    get_unaligned_le32(bytes + c->value - 4) != c->width)
			return -ENOEXEC;
	}
	sha256_init(&ctx);
	sha256_update(&ctx, text, text_size);
	for (i = 0; i < rule->cell_count; i++) {
		const struct ane_h16_preload_cell *c = &rule->cells[i];

		if (!c->mutable)
			continue;
		sha256_update(&ctx, bytes + cursor, c->value - cursor);
		ane_h16_hash_zeros(&ctx, c->width);
		cursor = c->value + c->width;
	}
	sha256_update(&ctx, bytes + cursor, rule->zero_start - cursor);
	ane_h16_hash_zeros(&ctx, rule->zero_end - rule->zero_start);
	cursor = rule->zero_end;
	sha256_update(&ctx, bytes + cursor, data_size - cursor);
	sha256_final(&ctx, digest);
	return 0;
}
EXPORT_SYMBOL_GPL(ane_h16_preload_digest);

int ane_h16_preload_validate(const struct ane_h16_preload_rule *rule,
			     const void *text, size_t text_size,
			     const void *data, size_t data_size)
{
	u8 digest[SHA256_DIGEST_SIZE];
	int ret;

	ret = ane_h16_preload_digest(rule, text, text_size, data, data_size, digest);
	if (ret)
		return ret;
	return memcmp(digest, rule->digest, sizeof(digest)) ? -ENOEXEC : 0;
}
EXPORT_SYMBOL_GPL(ane_h16_preload_validate);

MODULE_DESCRIPTION("H16 resident firmware identity validation");
MODULE_LICENSE("Dual MIT/GPL");
