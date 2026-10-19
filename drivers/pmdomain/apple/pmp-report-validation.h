/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef _APPLE_PMP_REPORT_VALIDATION_H
#define _APPLE_PMP_REPORT_VALIDATION_H

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define PMP_SOC_DEVICE_SIZE	0x7c
#define PMP_SOC_DEVICE_ACK	BIT(1)
#define PMP_PTD_RANGE_SIZE	32
#define PMP_PTD_RANGE_NAME	16
#define PMP_PTD_RANGE_NAME_LEN	16
#define PMP_PTD_RANGE_DVFS_STATE	3
#define PMP_PTD_RANGE_REQUEST	10
#define PMP_PTD_RANGE_ACK	11
#define PMP_PTD_RANGE_SOC_DEV_DVFS	12

/* The PTD request read, request update and acknowledgment apertures. */
static inline bool apple_pmp_ranges_valid(const u8 *table, size_t len,
					  u32 request_read, u32 request_write,
					 u32 actual)
{
	unsigned int found = 0;
	size_t pos;

	if (!table || !len || len % PMP_PTD_RANGE_SIZE)
		return false;
	for (pos = 0; pos < len; pos += PMP_PTD_RANGE_SIZE) {
		u32 id = get_unaligned_le32(table + pos);
		u64 base = get_unaligned_le32(table + pos + 4);
		u32 count = get_unaligned_le32(table + pos + 8);

		if (id == PMP_PTD_RANGE_REQUEST) {
			if ((found & BIT(0)) || !count || base * 16 != request_read ||
			    0x10000 + base * 8 != request_write)
				return false;
			found |= BIT(0);
		} else if (id == PMP_PTD_RANGE_ACK) {
			if ((found & BIT(1)) || !count || base * 16 != actual)
				return false;
			found |= BIT(1);
		}
	}
	return found == (BIT(0) | BIT(1));
}

/*
 * The base entry and entry count of the single PTD range with @id, whose
 * name (16 bytes at +16, NUL padded) must be @name. The lookup is by id;
 * the name guards against a table whose ids mean something else.
 */
static inline bool apple_pmp_range_find(const u8 *table, size_t len, u32 id,
					const char *name, u32 *base, u32 *count)
{
	bool found = false;
	size_t pos;

	if (!table || !len || len % PMP_PTD_RANGE_SIZE)
		return false;
	for (pos = 0; pos < len; pos += PMP_PTD_RANGE_SIZE) {
		const u8 *rec = table + pos;

		if (get_unaligned_le32(rec) != id)
			continue;
		if (found ||
		    strncmp((const char *)rec + PMP_PTD_RANGE_NAME, name, PMP_PTD_RANGE_NAME_LEN) ||
		    !get_unaligned_le32(rec + 8))
			return false;
		*base = get_unaligned_le32(rec + 4);
		*count = get_unaligned_le32(rec + 8);
		found = true;
	}
	return found;
}

/* Request bit N names device ID N + 1, independent of table order. */
static inline bool apple_pmp_devices_valid(const u8 *table, size_t len,
					   u64 seed, u64 ack, bool ordered)
{
	u64 found = 0;
	size_t pos;

	if (!table || !len || len % PMP_SOC_DEVICE_SIZE || (ack & ~seed))
		return false;
	for (pos = 0; pos < len; pos += PMP_SOC_DEVICE_SIZE) {
		const u8 *dev = table + pos;
		u32 id = get_unaligned_le32(dev);
		u64 bit;

		if (!id || id > 64)
			continue;
		bit = BIT_ULL(id - 1);
		if (!(seed & bit))
			continue;
		if ((found & bit) || (ordered && id != pos / PMP_SOC_DEVICE_SIZE + 1) ||
		    !!(get_unaligned_le32(dev + 8) & PMP_SOC_DEVICE_ACK) !=
		    !!(ack & bit))
			return false;
		found |= bit;
	}
	return found == seed;
}

/*
 * The index of the soc-device record that holds device ID @id, or -ENOENT if
 * no record or more than one does. Where the PTD request bits are indexed by
 * soc-device record (T8122), the device's bit is that index, not ID - 1.
 */
static inline int apple_pmp_record_of_id(const u8 *table, size_t len, u32 id)
{
	int found = -ENOENT;
	size_t pos;

	if (!table || !len || len % PMP_SOC_DEVICE_SIZE || !id)
		return -ENOENT;
	for (pos = 0; pos < len; pos += PMP_SOC_DEVICE_SIZE) {
		if (get_unaligned_le32(table + pos) != id)
			continue;
		if (found >= 0)
			return -ENOENT;
		found = pos / PMP_SOC_DEVICE_SIZE;
	}
	return found;
}

/*
 * Request bit N names soc-device record N. Each seeded record must exist and
 * be acknowledged exactly when its flags say so.
 */
static inline bool apple_pmp_records_valid(const u8 *table, size_t len,
					   u64 seed, u64 ack)
{
	u64 bits = seed;

	if (!table || !len || len % PMP_SOC_DEVICE_SIZE || (ack & ~seed))
		return false;
	while (bits) {
		unsigned int n = __ffs64(bits);
		const u8 *dev;

		bits &= bits - 1;
		if ((size_t)(n + 1) * PMP_SOC_DEVICE_SIZE > len)
			return false;
		dev = table + n * PMP_SOC_DEVICE_SIZE;
		if (!get_unaligned_le32(dev) ||
		    !!(get_unaligned_le32(dev + 8) & PMP_SOC_DEVICE_ACK) !=
		    !!(ack & BIT_ULL(n)))
			return false;
	}
	return true;
}

/* Packed integer arguments are validated before the PMP driver patches them. */
static inline bool apple_pmp_bootargs_valid(const u8 *args, size_t len)
{
	unsigned int found = 0;
	size_t pos = 0;

	if (!args || !len)
		return false;
	while (pos < len) {
		u32 key, size;

		if (len - pos < 8)
			return false;
		key = get_unaligned_le32(args + pos);
		size = get_unaligned_le32(args + pos + 4);
		pos += 8;
		if (size > len - pos)
			return false;
		if (key == 0x42444944 || key == 0x44564944) { /* BDID or DVID */
			if (!size || size > 8)
				return false;
			found |= key == 0x42444944 ? BIT(0) : BIT(1);
		}
		pos += size;
	}
	return found == (BIT(0) | BIT(1));
}

#endif
