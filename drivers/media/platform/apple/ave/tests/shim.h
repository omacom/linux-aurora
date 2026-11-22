/* SPDX-License-Identifier: GPL-2.0-only */
/* Host types/primitives, never an alternative command implementation. */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int32_t s32;
typedef int64_t s64;
typedef uint64_t dma_addr_t;
#define AVE_BUF_CMD 0
#define AVE_BUF_PROC(s) (17 + (s))
#define SZ_16K 0x4000U
#define SZ_1M 0x100000U
#define U32_MAX UINT32_MAX
#define GENMASK(h, l) ((~0UL >> (63 - (h))) & (~0UL << (l)))
#define GENMASK_ULL(h, l) ((~0ULL >> (63 - (h))) & (~0ULL << (l)))
#define BIT(n) (1U << (n))
#define BIT_ULL(n) (1ULL << (n))
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define DIV_ROUND_UP(x, a) (((x) + (a) - 1) / (a))
#define ALIGN(x, a) (((x) + (a) - 1) & ~((a) - 1))
#define IS_ALIGNED(x, a) (!((x) & ((typeof(x))(a) - 1)))
#define max(x, y) ((x) > (y) ? (x) : (y))
#define max_t(t, x, y) ((t)(x) > (t)(y) ? (t)(x) : (t)(y))
#define GFP_KERNEL 0
#define DMA_ATTR_NO_KERNEL_MAPPING 1
static inline void *kcalloc(size_t n, size_t size, int flags)
{
	return calloc(n, size);
}

#define kzalloc(size, flags) ((void)(flags), calloc(1, (size)))

#define kfree(p) free(p)
#define kvfree(p) free(p)
static inline u32 crc32_le(u32 crc, const void *p, size_t len)
{
	const unsigned char *b = p;

	while (len--) {
		crc ^= *b++;
		for (int k = 0; k < 8; k++)
			crc = (crc >> 1) ^ (0xedb88320U & -(crc & 1));
	}
	return crc;
}
#define IOMMU_DOMAIN_DMA 3
#define IOMMU_DOMAIN_DMA_FQ 7
static inline void clear_bit(int nr, unsigned long *p) { *p &= ~(1UL << nr); }
static inline void set_bit(int nr, unsigned long *p) { *p |= 1UL << nr; }
static inline int test_bit(int nr, const unsigned long *p) { return (*p >> nr) & 1; }
static inline int test_and_set_bit(int nr, unsigned long *p)
{
	int old = test_bit(nr, p);

	set_bit(nr, p);
	return old;
}
struct iommu_domain { unsigned int type; };
#define iommu_get_domain_for_dev(d) ((void)(d), (struct iommu_domain *)NULL)
#define iommu_flush_iotlb_all(d) ((void)(d))
static inline void sort(void *p, size_t n, size_t size,
			int (*cmp)(const void *, const void *), void *unused)
{
	qsort(p, n, size, cmp);
}

#define dev_err(...) test_log(__VA_ARGS__)
#define dev_info(...) test_log(__VA_ARGS__)
#define dev_dbg(...) test_log(__VA_ARGS__)
#define dev_warn(...) test_log(__VA_ARGS__)
#define dev_warn_once(...) test_log(__VA_ARGS__)
#define WARN_ON_ONCE(c) ((c) ? (abort(), 1) : 0)
#define ENOTSUPP 524
static inline void test_log(void *dev, const char *fmt, ...)
{
}

#define atomic_inc(p) (++*(p))
static unsigned long jiffies = 42;
static inline void put_unaligned_le16(u16 v, void *p)
{
	u8 *b = p;

	for (unsigned int i = 0; i < 2; i++)
		b[i] = v >> (8 * i);
}
static inline void put_unaligned_le32(u32 v, void *p)
{
	u8 *b = p;

	for (unsigned int i = 0; i < 4; i++)
		b[i] = v >> (8 * i);
}

static inline void put_unaligned_le64(u64 v, void *p)
{
	u8 *b = p;

	for (unsigned int i = 0; i < 8; i++)
		b[i] = v >> (8 * i);
}

#ifdef TEST_PER_ABI
static inline u16 get_unaligned_le16(const void *p)
{
	const u8 *b = p;

	return (u16)b[0] | (u16)b[1] << 8;
}

#endif
static inline u32 get_unaligned_le32(const void *p)
{
	const u8 *b = p;

	return (u32)b[0] | (u32)b[1] << 8 | (u32)b[2] << 16 | (u32)b[3] << 24;
}

static inline u64 get_unaligned_le64(const void *p)
{
	return get_unaligned_le32(p) |
	       (u64)get_unaligned_le32((const u8 *)p + 4) << 32;
}

static inline u64 mul_u64_u64_div_u64(u64 a, u64 b, u64 c)
{
	return (__uint128_t)a * b / c;
}

static inline u32 roundup_pow_of_two(u32 v)
{
	u32 n = 1;

	while (n < v)
		n *= 2;
	return n;
}
