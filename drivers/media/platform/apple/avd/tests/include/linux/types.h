/* SPDX-License-Identifier: GPL-2.0-only OR MIT */
#ifndef AVD_HOST_TYPES_H_
#define AVD_HOST_TYPES_H_
#include <stdbool.h>
#include <stddef.h>
#include_next <linux/types.h>
typedef __u8 u8;
typedef __u32 u32;
typedef __u64 u64;
typedef __u64 dma_addr_t;
#endif
