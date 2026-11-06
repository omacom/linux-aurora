/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef APPLE_J613_25G83_H
#define APPLE_J613_25G83_H

#include <linux/bitops.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#define J613_25G83_DCP_UUID "C042E95C-B9D8-3F0E-94B3-582A08AA6FDD"
#define J613_25G83_PMP_UUID "EFD60284-7C58-33A9-8522-CB3366AF6040"
#define J613_25G83_HANDOFF "apple,j613-25g83-mapping-handoff"
/* J615 requires an explicit chosen switch in addition to the 25G83 profile. */
#define J615_25G83_OPT_IN "asahi,j615-25g83-experimental"

/* The board half of the identity: a J613, or a J615 with the opt-in. */
static inline bool t8122_25g83_board(bool j613, bool j615, const char *optin,
				     int optin_len)
{
	return (j613 && !j615) || (!j613 && j615 && optin && optin_len == 2 &&
		optin[0] == '1' && !optin[1]);
}

/* `board`: t8122_25g83_board() for the running machine. */
static inline bool j613_25g83_identity(bool board, u32 profile,
				     const u32 *version, size_t cells,
				     const char *uuid)
{
	return board && profile == 1 && version && cells == 3 &&
	       version[0] == 26 && version[1] == 6 && version[2] == 2 &&
	       uuid && !strcmp(uuid, J613_25G83_DCP_UUID);
}

static inline bool j613_25g83_clock(const u32 *witness, size_t cells)
{
	return witness && cells == 5 && witness[0] == 1 && witness[1] == 345 &&
	       witness[2] == 416 && witness[3] && !witness[4];
}

static inline bool j613_25g83_mappings(const u32 *markers, size_t cells)
{
	return markers && cells == 3 && markers[0] == 1 && markers[1] == 1 &&
	       markers[2] == 1;
}

/* This profile adopts a powered RUN session through a locked firmware DART. */
static inline bool j613_25g83_external_state(const u32 *state, size_t cells)
{
	return state && cells == 4 && state[0] == 1 &&
	       (state[1] & BIT(4)) && (state[3] & BIT(0));
}

/*
 * Local copies of the PMP table checks of
 * drivers/pmdomain/apple/pmp-report-validation.h, which is not in this tree
 * yet; the real header wins when it is included first.
 */
#ifndef _APPLE_PMP_REPORT_VALIDATION_H
/* The PTD request read, request update and acknowledgment apertures. */
static inline bool apple_pmp_ranges_valid(const u8 *table, size_t len,
					  u32 request_read, u32 request_write,
					  u32 actual)
{
	unsigned int found = 0;
	size_t pos;

	if (!table || !len || len % 32)
		return false;
	for (pos = 0; pos < len; pos += 32) {
		u32 id = get_unaligned_le32(table + pos);
		u64 base = get_unaligned_le32(table + pos + 4);
		u32 count = get_unaligned_le32(table + pos + 8);

		if (id == 10) {
			if ((found & BIT(0)) || !count || base * 16 != request_read ||
			    0x10000 + base * 8 != request_write)
				return false;
			found |= BIT(0);
		} else if (id == 11) {
			if ((found & BIT(1)) || !count || base * 16 != actual)
				return false;
			found |= BIT(1);
		}
	}
	return found == (BIT(0) | BIT(1));
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

#endif
