/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_DCPEXT26_WIRE_H
#define APPLE_DCPEXT26_WIRE_H

#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

/* J613 26.6.2 external personality; neither classic 14.7 nor H17P. */
#define DCPEXT26_SUBMIT_BYTES 0x1bd8
#define DCPEXT26_SURFACE_BYTES 0x22c

struct dcpext26_primary {
	u64 address;
	u64 backing_bytes;
	u32 width;
	u32 height;
	u32 pitch;
};

/* Validate all arithmetic before changing a caller-owned command buffer. */
static inline int dcpext26_primary_check(const struct dcpext26_primary *p)
{
	u64 bytes = (u64)p->pitch * p->height;

	if (!p->address || !p->width || !p->height ||
	    (p->pitch & 3) || (u64)p->width * 4 > p->pitch)
		return -EINVAL;
	if (bytes > (u32)~0U || bytes > p->backing_bytes ||
	    p->address > (u64)~0ULL - bytes)
		return -ERANGE;
	return 0;
}

/* Full-frame linear opaque BGRA. All unused layers stay explicitly null. */
static inline int dcpext26_primary_encode(void *buffer, size_t capacity,
					 u32 id, unsigned int layer,
					 const struct dcpext26_primary *p)
{
	u8 *wire = buffer, *surface;
	u32 bytes;
	unsigned int i;
	int ret;

	if (!buffer || capacity != DCPEXT26_SUBMIT_BYTES || !id || layer >= 4)
		return -EINVAL;
	if (p) {
		ret = dcpext26_primary_check(p);
		if (ret)
			return ret;
	}
	memset(wire, 0, capacity);
	put_unaligned_le32(id, wire + 0x98);
	put_unaligned_le32(0x80000007, wire + 0x14c);
	put_unaligned_le32(0x80000007, wire + 0x150);
	put_unaligned_le32(0xff000000, wire + 0x154);
	/* Ten nullable surfaces, followed by the two output-pointer flags. */
	memset(wire + 0x1bcb, 1, 10);
	wire[0x1bd6] = 1;
	wire[0x1bd7] = 1;
	if (!p)
		return 0;

	surface = wire + 0x588 + layer * DCPEXT26_SURFACE_BYTES;
	bytes = p->pitch * p->height;
	{
		const struct { unsigned int offset; u32 value; } words[] = {
			{0x21, p->width}, {0x59, p->width},
			{0x25, p->height}, {0x5d, p->height},
			{0x15, p->pitch}, {0x69, p->pitch},
			{0x29, bytes}, {0x6d, bytes},
			{3, 1}, {7, 1}, {0xb, 0x42475241}, {0x35, 1},
		};
		const struct { unsigned int offset; u8 value; } octets[] = {
			{2, 1}, {0x13, 13}, {0x14, 12}, {0x19, 1},
			{0x1b, 1}, {0x1c, 1}, {0x71, 4}, {0x73, 1}, {0x74, 1},
		};

		for (i = 0; i < sizeof(words) / sizeof(words[0]); i++)
			put_unaligned_le32(words[i].value, surface + words[i].offset);
		for (i = 0; i < sizeof(octets) / sizeof(octets[0]); i++)
			surface[octets[i].offset] = octets[i].value;
		put_unaligned_le64(1, surface + 0x149);
		put_unaligned_le64(1, surface + 0x51);
	}
	put_unaligned_le64(p->address, wire + 0xe38 + layer * 8);
	put_unaligned_le32(1, wire + 0x9c + layer * 4);
	for (i = 0; i < 2; i++) {
		u8 *rect = wire + (i ? 0x10c : 0xac) + layer * 16;

		put_unaligned_le32(p->width, rect + 8);
		put_unaligned_le32(p->height, rect + 12);
	}
	wire[0x1bcb + layer] = 0;
	return 0;
}

static inline int dcpext26_start_reply(const void *reply, size_t length, u32 *id)
{
	const u8 *wire = reply;
	u32 value;

	if (!reply || !id || length != 8)
		return -EINVAL;
	value = get_unaligned_le32(wire);
	if (!value || get_unaligned_le32(wire + 4))
		return -EPROTO;
	*id = value;
	return 0;
}

static inline int dcpext26_submit_reply(const void *reply, size_t length)
{
	const u8 *wire = reply;

	if (!reply || length != 12)
		return -EINVAL;
	return get_unaligned_le32(wire + 5) ? -EPROTO : 0;
}

#endif
