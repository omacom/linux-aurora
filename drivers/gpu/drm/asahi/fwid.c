// SPDX-License-Identifier: GPL-2.0-only OR MIT
//
// SHA-256 for the firmware identity check (fwid.rs).

#include <crypto/sha2.h>
#include <linux/types.h>

void asahi_fw_sha256(const u8 *data, size_t len, u8 *out);

void asahi_fw_sha256(const u8 *data, size_t len, u8 *out)
{
	sha256(data, len, out);
}
