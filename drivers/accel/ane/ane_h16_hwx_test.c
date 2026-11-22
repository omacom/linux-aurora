// SPDX-License-Identifier: GPL-2.0-only OR MIT
#include <kunit/test.h>
#include <linux/module.h>

#include "ane_h16_hwx.h"
#include "ane_h16_tasks.h"
#include "ane_h16_load.h"

#include "ane_h16_hwx_test_fixture.h"

static void hwx_forms_test(struct kunit *test)
{
	unsigned int variant;

	for (variant = 0; variant < 4; variant++) {
		struct hwx_fixture *f = fixture_create(test, variant & 1, variant & 2);
		struct ane_h16_hwx info = {};

		KUNIT_ASSERT_NOT_NULL(test, f);
		KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
		KUNIT_EXPECT_EQ(test, info.surface_count, 2U);
		KUNIT_EXPECT_EQ(test, info.surfaces[0].span, 256ULL);
		KUNIT_EXPECT_EQ(test, info.tasks, 1U);
		KUNIT_EXPECT_EQ(test, info.text.ordinal, (variant & 1) ? 4U : 3U);
		KUNIT_EXPECT_EQ(test, info.text.nreloc, (variant & 2) ? 1U : 0U);
	}
}

static void hwx_header_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx info;
	u32 offset;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (offset = 0; offset < 32; offset += 4) {
		u32 saved = get_unaligned_le32(f->bytes + offset);

		put_unaligned_le32(U32_MAX, f->bytes + offset);
		KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
		put_unaligned_le32(saved, f->bytes + offset);
	}
	for (offset = 0; offset < 32; offset++)
		KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, offset, &info), -ENOEXEC);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size - 1, &info), -ENOEXEC);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, ANE_H16_HWX_MAX + 1, &info), -ENOEXEC);
	/* A truncated command must not inherit a previous command's success. */
	put_unaligned_le32(f->cursor - 33, f->bytes + 20);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
}

static void hwx_command_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx info;
	u32 pos;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (pos = 32; pos < f->cursor;) {
		u32 command = get_unaligned_le32(f->bytes + pos);
		u32 length = get_unaligned_le32(f->bytes + pos + 4);

		put_unaligned_le32(0xffffffff, f->bytes + pos);
		KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
		put_unaligned_le32(command, f->bytes + pos);
		put_unaligned_le32(0, f->bytes + pos + 4);
		KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
		put_unaligned_le32(length, f->bytes + pos + 4);
		pos += length;
	}
}

static void hwx_tensor_test(struct kunit *test)
{
	static const u32 fields[] = {
		0xc, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24, 0x28, 0x2c,
		0x30, 0x34, 0x38, 0x3c, 0x50, 0x58, 0x60, 0x68, 0x70,
		0x78, 0x7c, 0x80, 0x88, 0xd38, 0xd44,
	};
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx info;
	u8 *tensor;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	tensor = f->bytes + f->tensor[0];
	for (i = 0; i < ARRAY_SIZE(fields); i++) {
		u32 saved = get_unaligned_le32(tensor + fields[i]);

		put_unaligned_le32(U32_MAX, tensor + fields[i]);
		KUNIT_EXPECT_EQ_MSG(test, ane_h16_hwx_validate(f->bytes, f->size, &info),
				    -ENOEXEC, "tensor field %#x", fields[i]);
		put_unaligned_le32(saved, tensor + fields[i]);
	}
	/* Four batches fit in the allocation; span is not just one batch. */
	put_unaligned_le32(4, tensor + 0x28);
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
	KUNIT_EXPECT_EQ(test, info.surfaces[0].span, 1024ULL);
	put_unaligned_le64(U64_MAX, tensor + 0x50);
	put_unaligned_le64(U64_MAX, tensor + 0x70);
	put_unaligned_le64(U64_MAX, tensor + 0x80);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
}

/*
 * fp16, int8 and uint8 surfaces, as Apple's H16 and H17 objects describe
 * them: format code at +0x24, element bytes at +0x68 (a u64). An 8-bit
 * surface's rows need width bytes, not 2 * width. Only the known pairs load.
 */
static void hwx_element_format_test(struct kunit *test)
{
	static const struct {
		u32 format;
		u64 element;
		int ret;
	} pairs[] = {
		{ 5, 2, 0 }, { 2, 1, 0 }, { 3, 1, 0 },
		{ 5, 1, -ENOEXEC }, { 2, 2, -ENOEXEC }, { 3, 2, -ENOEXEC },
		{ 2, 0, -ENOEXEC }, { 0, 1, -ENOEXEC }, { 1, 1, -ENOEXEC },
		{ 4, 1, -ENOEXEC }, { 4, 4, -ENOEXEC }, { 6, 2, -ENOEXEC },
		/* fp32, int16 and uint16: seen in Apple's objects, not yet admitted */
		{ 6, 4, -ENOEXEC }, { 4, 2, -ENOEXEC }, { 15, 2, -ENOEXEC },
		{ 2, 1ULL << 32 | 1, -ENOEXEC }, { 5, 1ULL << 32 | 2, -ENOEXEC },
	};
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx info;
	unsigned int i, t;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (t = 0; t < 2; t++) {
		u8 *tensor = f->bytes + f->tensor[t];

		/* 64 wide: the 256-byte rows fit even 4-byte elements. */
		put_unaligned_le32(64, tensor + 0x34);
		for (i = 0; i < ARRAY_SIZE(pairs); i++) {
			put_unaligned_le32(pairs[i].format, tensor + 0x24);
			put_unaligned_le64(pairs[i].element, tensor + 0x68);
			KUNIT_EXPECT_EQ_MSG(test,
					    ane_h16_hwx_validate(f->bytes, f->size, &info),
					    pairs[i].ret, "tensor %u format %u element %#llx",
					    t, pairs[i].format, pairs[i].element);
			if (!pairs[i].ret)
				KUNIT_EXPECT_EQ(test, info.surfaces[t].span, 256ULL);
		}
		/* Leave this surface 8-bit: int8 in with uint8 out loads too. */
		put_unaligned_le32(t ? 3 : 2, tensor + 0x24);
		put_unaligned_le64(1, tensor + 0x68);
	}
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);

	/* A 256-byte row holds 256 int8 elements but only 128 fp16 ones. */
	put_unaligned_le32(256, f->bytes + f->tensor[0] + 0x34);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
	put_unaligned_le32(257, f->bytes + f->tensor[0] + 0x34);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
	put_unaligned_le32(256, f->bytes + f->tensor[0] + 0x34);
	put_unaligned_le32(5, f->bytes + f->tensor[0] + 0x24);
	put_unaligned_le64(2, f->bytes + f->tensor[0] + 0x68);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
	put_unaligned_le32(128, f->bytes + f->tensor[0] + 0x34);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
}

static u8 *fixture_ref(struct hwx_fixture *f, unsigned int index)
{
	size_t pos = 32;
	unsigned int i, seen = 0;

	for (i = 0; i < f->commands; i++) {
		u8 *c = f->bytes + pos;

		if (get_unaligned_le32(c) == 0x40 && seen++ == index)
			return c;
		pos += get_unaligned_le32(c + 4);
	}
	return NULL;
}

static void fixture_surface_kind(struct hwx_fixture *f, unsigned int index, u32 prot)
{
	u8 *c = f->bytes + f->segment[index];

	put_unaligned_le32(prot, c + 56);
	put_unaligned_le32(prot, c + 60);
	memset(c + 72, 0, 16);
	strscpy(c + 72, prot == 1 ? "__const" : "__data", 16);
	put_unaligned_le32(prot == 1 ? 0x21 : 0x23, c + 72 + 64);
}

/*
 * Surfaces stay in segment order whatever order the tensor descriptors use:
 * the add-128 HWX has segments x, y, z (in, out, in) and tensors x, z, y.
 */
static void hwx_surface_order_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx info;
	u8 *tensor = kunit_kzalloc(test, 0xd48, GFP_KERNEL), *ref[2], *symbol[2];
	u64 value;
	u8 ordinal;

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_NOT_NULL(test, tensor);
	/* Tensors listed out of segment order pair by name. */
	memcpy(tensor, f->bytes + f->tensor[0], 0xd48);
	memmove(f->bytes + f->tensor[0], f->bytes + f->tensor[1], 0xd48);
	memcpy(f->bytes + f->tensor[1], tensor, 0xd48);
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
	KUNIT_EXPECT_STREQ(test, info.surfaces[0].name, "in");
	KUNIT_EXPECT_EQ(test, info.surfaces[0].direction, 1U);
	KUNIT_EXPECT_STREQ(test, info.surfaces[1].name, "out");
	KUNIT_EXPECT_EQ(test, info.surfaces[1].direction, 2U);

	/* The output need not be the last segment. */
	f = fixture_create(test, false, false);
	KUNIT_ASSERT_NOT_NULL(test, f);
	ref[0] = fixture_ref(f, 0);
	ref[1] = fixture_ref(f, 1);
	KUNIT_ASSERT_NOT_NULL(test, ref[0]);
	KUNIT_ASSERT_NOT_NULL(test, ref[1]);
	fixture_surface_kind(f, 0, 2);
	fixture_surface_kind(f, 1, 1);
	memset(ref[0] + 0x18, 0, 8);
	memset(ref[1] + 0x18, 0, 8);
	strscpy(ref[0] + 0x18, "out", 8);
	strscpy(ref[1] + 0x18, "in", 8);
	symbol[0] = f->bytes + f->symbols;
	symbol[1] = symbol[0] + 16;
	value = get_unaligned_le64(symbol[0] + 8);
	put_unaligned_le64(get_unaligned_le64(symbol[1] + 8), symbol[0] + 8);
	put_unaligned_le64(value, symbol[1] + 8);
	ordinal = symbol[0][5];
	symbol[0][5] = symbol[1][5];
	symbol[1][5] = ordinal;
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
	KUNIT_EXPECT_STREQ(test, info.surfaces[0].name, "out");
	KUNIT_EXPECT_EQ(test, info.surfaces[0].direction, 2U);
	KUNIT_EXPECT_STREQ(test, info.surfaces[1].name, "in");
	KUNIT_EXPECT_EQ(test, info.surfaces[1].direction, 1U);

	/* A reference naming no tensor refuses. */
	strscpy(ref[1] + 0x18, "zz", 8);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);

	/* Exactly one output: two inputs and no output refuse. */
	f = fixture_create(test, false, false);
	KUNIT_ASSERT_NOT_NULL(test, f);
	fixture_surface_kind(f, 1, 1);
	put_unaligned_le32(1, f->bytes + f->tensor[1] + 0x14);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
}

static void hwx_program_test(struct kunit *test)
{
	static const u32 fields[] = {
		0xc, 0x10, 0x18, 0x20, 0x28, 0x40, 0x48, 0x70, 0x78, 0x80, 0x88,
		0x90, 0xa0, 0xb0, 0xb8, 0xc0, 0x80c, 0x810, 0x818, 0x820, 0x824,
		0x828, 0x830, 0x834, 0x838, 0x83c, 0x840, 0x844, 0x848, 0x84c,
		0x858, 0x85c, 0x860, 0x864, 0x868, 0x870, 0x878, 0x880, 0x890, 0x89c,
	};
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx info;
	u8 *program;
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, f);
	program = f->bytes + f->program;
	for (i = 0; i < ARRAY_SIZE(fields); i++) {
		u32 saved = get_unaligned_le32(program + fields[i]);

		put_unaligned_le32(saved ^ U32_MAX, program + fields[i]);
		KUNIT_EXPECT_EQ_MSG(test, ane_h16_hwx_validate(f->bytes, f->size, &info),
				    -ENOEXEC, "program field %#x", fields[i]);
		put_unaligned_le32(saved, program + fields[i]);
	}
}

/* Build only structural tasks: no model bytes or hardware fixtures needed. */
static void fixture_tasks(struct hwx_fixture *f, u32 tasks, u32 records)
{
	u8 *text = f->bytes + SZ_16K, *program = f->bytes + f->program;
	u8 *section = f->bytes + f->text_command + 72;
	u64 vm = get_unaligned_le64(section + 32);
	u32 pos = 16, task, constants;

	memset(text, 0, SZ_16K);
	put_unaligned_le32(1, text);
	for (task = 0; task < tasks; task++) {
		u32 count = records / tasks + (task < records % tasks);

		put_unaligned_le32((9 + 2 * count) << 16, text + pos);
		/* Coalesce writes: the hint exceeds the encoded record count. */
		put_unaligned_le32(((2 * count - 2) << 15) | (0x4100 / 4),
				   text + pos + 36);
		pos += (9 + 2 * count) * 4;
		if (task + 1 < tasks)
			pos = ALIGN(pos, 16);
	}
	constants = ALIGN(pos, 64);
	put_unaligned_le64(pos, section + 40);
	put_unaligned_le64(vm + constants, section + 80 + 32);
	put_unaligned_le32(SZ_16K + constants, section + 80 + 48);
	put_unaligned_le64(vm + constants, program + 0x20);
	put_unaligned_le32(pos / 4, program + 0x824);
	put_unaligned_le32(tasks, program + 0x830);
	put_unaligned_le32(records, program + 0x860);
}

static void hwx_staged_counts_test(struct kunit *test)
{
	static const struct {
		u32 tasks, records, work;
	} shapes[] = {
		{ 7, 0x58, 1 }, /* Row softmax. */
		{ 4, 0x2e, 0 }, /* Tiled matmul 768. */
		{ 5, 0x3c, 0 }, /* Tiled matmul 896. */
		{ 7, 0x58, 0 }, /* Tiled matmul 1536. */
		{ 25, 0x154, 0 }, /* Tiled matmul 3072. */
		{ 10, 0x75, 1 }, /* Decoded attention. */
		{ 1, 0x10, 61 }, { 1, 0x10, 92 }, /* LUT/ALU 1024. */
		{ 1, 0x10, 245 }, { 1, 0x10, 368 }, /* LUT/ALU 2048. */
	};
	struct ane_h16_hwx info;
	unsigned int form, i;

	for (form = 0; form < 2; form++) {
		for (i = 0; i < ARRAY_SIZE(shapes); i++) {
			struct hwx_fixture *f = fixture_create(test, form, false);
			u32 work_offset = form ? 0x8b0 : 0x890;
			u8 *program;

			KUNIT_ASSERT_NOT_NULL(test, f);
			fixture_tasks(f, shapes[i].tasks, shapes[i].records);
			program = f->bytes + f->program;
			put_unaligned_le32(shapes[i].work, program + work_offset);
			KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
			KUNIT_EXPECT_EQ(test, info.tasks, shapes[i].tasks);
			/* Neither count nor work may exceed its program-derived bound. */
			put_unaligned_le32(shapes[i].records + 1, program + 0x860);
			KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info),
					-ENOEXEC);
			put_unaligned_le32(0, program + 0x860);
			KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info),
					-ENOEXEC);
			put_unaligned_le32(shapes[i].records, program + 0x860);
			put_unaligned_le32(SZ_32K, program + work_offset);
			KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
			put_unaligned_le32(SZ_32K + 1, program + work_offset);
			KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info),
					-ENOEXEC);
			put_unaligned_le32(U32_MAX, program + work_offset);
			KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info),
					-ENOEXEC);
			put_unaligned_le32(shapes[i].work, program + work_offset);
			/* A plausible hint never overrides a corrupt task stream. */
			put_unaligned_le32(0x6000 / 4, f->bytes + SZ_16K + 52);
			KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info),
					-ENOEXEC);
		}
	}
}

/*
 * Trailing zero bytes after the last H16 task leave the engine armed, and the
 * next EXECUTE times out. The final task must end the section exactly.
 */
static void hwx_final_task_end_test(struct kunit *test)
{
	static const u32 tails[] = { 4, 8, 12, 12 + 16 };
	struct ane_h16_hwx info;
	unsigned int form, i;

	for (form = 0; form < 2; form++) {
		struct hwx_fixture *f = fixture_create(test, form, false);
		u8 *section, *program;
		u64 end;

		KUNIT_ASSERT_NOT_NULL(test, f);
		/* One task with two records ends at 68; __const starts at 128. */
		fixture_tasks(f, 1, 2);
		section = f->bytes + f->text_command + 72;
		program = f->bytes + f->program;
		end = get_unaligned_le64(section + 40);
		KUNIT_ASSERT_EQ(test, end, 68ULL);
		KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
		for (i = 0; i < ARRAY_SIZE(tails); i++) {
			put_unaligned_le64(end + tails[i], section + 40);
			put_unaligned_le32((end + tails[i]) / 4, program + 0x824);
			KUNIT_EXPECT_EQ_MSG(test, ane_h16_hwx_validate(f->bytes, f->size, &info),
					    -ENOEXEC, "form %u tail %u", form, tails[i]);
		}
		put_unaligned_le64(end, section + 40);
		put_unaligned_le32(end / 4, program + 0x824);
		KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0);
	}
}

static void hwx_geometry_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx info;
	u8 *segment;
	unsigned int variant;

	KUNIT_ASSERT_NOT_NULL(test, f);
	segment = f->bytes + f->segment[0];
	for (variant = 0; variant < 8; variant++) {
		u32 fields[] = { 24, 32, 40, 48, 56, 60, 64, 72 + 52 };
		u32 saved = get_unaligned_le32(segment + fields[variant]);

		put_unaligned_le32(U32_MAX, segment + fields[variant]);
		KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
		put_unaligned_le32(saved, segment + fields[variant]);
	}
	put_unaligned_le64(U64_MAX - SZ_16K + 1, segment + 24);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
}

static void hwx_symbols_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx info;
	u32 field;

	KUNIT_ASSERT_NOT_NULL(test, f);
	for (field = 8; field < 24; field += 4) {
		u32 saved = get_unaligned_le32(f->bytes + f->symtab + field);

		put_unaligned_le32(U32_MAX, f->bytes + f->symtab + field);
		KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
		put_unaligned_le32(saved, f->bytes + f->symtab + field);
	}
	f->bytes[f->symbols + 5] = 0;
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
	f->bytes[f->symbols + 5] = 1;
	f->bytes[f->strings + 1] = 'X';
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
}

static void hwx_relocations_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, true, true);
	struct ane_h16_hwx info;

	KUNIT_ASSERT_NOT_NULL(test, f);
	/* Scratch changes the section ordinal; a fixed ordinal is invalid. */
	put_unaligned_le32(0x07000005, f->bytes + f->reloc + 4);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
	put_unaligned_le32(0x07000006, f->bytes + f->reloc + 4);
	put_unaligned_le32(64, f->bytes + SZ_16K + 56);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
	put_unaligned_le32(32, f->bytes + SZ_16K + 56);
	put_unaligned_le32(57, f->bytes + f->reloc);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
	/* A small addend in a header or the prefix must not be patched. */
	put_unaligned_le32(20, f->bytes + f->reloc);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
	put_unaligned_le32(4, f->bytes + f->reloc);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC);
}

static void hwx_snapshot_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx_snapshot snapshot = {};

	KUNIT_ASSERT_NOT_NULL(test, f);
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_snapshot_create(f->bytes, f->size, &snapshot), 0);
	f->bytes[f->tensor[0] + 0x24] = 0;
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_snapshot_check(&snapshot), 0);
	/* Even padding and original command bytes are covered by the digest. */
	((u8 *)snapshot.bytes)[f->program + 0x100] = 1;
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_snapshot_check(&snapshot), -ENOEXEC);
	ane_h16_hwx_snapshot_destroy(&snapshot);
	KUNIT_EXPECT_PTR_EQ(test, snapshot.bytes, NULL);
}

static void hwx_snapshot_free(void *snapshot)
{
	ane_h16_hwx_snapshot_destroy(snapshot);
}

static struct ane_h16_hwx_snapshot *fixture_snapshot(struct kunit *test,
						     struct hwx_fixture *f)
{
	struct ane_h16_hwx_snapshot *snapshot;

	if (!f)
		return NULL;
	snapshot = kunit_kzalloc(test, sizeof(*snapshot), GFP_KERNEL);
	if (!snapshot || ane_h16_hwx_snapshot_create(f->bytes, f->size, snapshot))
		return NULL;
	if (kunit_add_action_or_reset(test, hwx_snapshot_free, snapshot))
		return NULL;
	return snapshot;
}

static void hwx_load_plan_test(struct kunit *test)
{
	unsigned int mixed, kernel;

	for (mixed = 0; mixed < 2; mixed++) {
		for (kernel = 0; kernel < 2; kernel++) {
			struct hwx_fixture *f = fixture_create(test, mixed, kernel);
			struct ane_h16_hwx_snapshot *snapshot = fixture_snapshot(test, f);
			struct ane_h16_load_plan plan;
			unsigned int text = 2 + mixed;

			KUNIT_ASSERT_NOT_NULL(test, snapshot);
			KUNIT_ASSERT_EQ(test, ane_h16_load_plan_create(snapshot, &plan), 0);
			KUNIT_EXPECT_EQ(test, plan.count, 3 + mixed + kernel);
			KUNIT_EXPECT_EQ(test, plan.total, (u64)(4 + mixed + kernel) * SZ_16K);
			KUNIT_EXPECT_EQ(test, plan.regions[text].kind, ANE_H16_REGION_TEXT);
			KUNIT_EXPECT_EQ(test, plan.regions[text].permissions, 5);
			KUNIT_EXPECT_EQ(test, plan.regions[text].file_offset, (u64)SZ_16K);
			KUNIT_EXPECT_EQ(test, plan.regions[text].file_size, (u64)SZ_32K);
			/* One TEXT allocation includes task and constant sections. */
			KUNIT_EXPECT_EQ(test, plan.allocation[text], (u64)SZ_32K);
			if (mixed) {
				KUNIT_EXPECT_EQ(test, plan.regions[0].kind, ANE_H16_REGION_SCRATCH);
				KUNIT_EXPECT_EQ(test, plan.regions[0].file_size, 0);
				KUNIT_EXPECT_EQ(test, plan.regions[0].permissions, 3);
			}
		}
	}
}

static void hwx_load_original_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, false, false);
	struct ane_h16_hwx_snapshot *snapshot = fixture_snapshot(test, f);
	struct ane_h16_load_plan plan;

	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	/* Cached metadata cannot replace the original authenticated container. */
	memset(&snapshot->info, 0xff, sizeof(snapshot->info));
	KUNIT_ASSERT_EQ(test, ane_h16_load_plan_create(snapshot, &plan), 0);
	KUNIT_EXPECT_EQ(test, plan.count, 3);
	((u8 *)snapshot->bytes)[f->program] ^= 1;
	plan.count = 0x1234;
	KUNIT_EXPECT_EQ(test, ane_h16_load_plan_create(snapshot, &plan), -ENOEXEC);
	KUNIT_EXPECT_EQ(test, plan.count, 0x1234);
}

static void fixture_mappings(const struct ane_h16_load_plan *plan,
			     struct ane_h16_load_mapping *mappings)
{
	unsigned int i;

	for (i = 0; i < plan->count; i++) {
		mappings[i].dva = 0x20000000000ULL + (u64)i * SZ_1M;
		mappings[i].size = plan->allocation[i];
		mappings[i].permissions = plan->regions[i].permissions;
	}
}

static void hwx_load_resources_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, true, true);
	struct ane_h16_hwx_snapshot *snapshot = fixture_snapshot(test, f);
	struct ane_h16_load_mapping mappings[ANE_H16_REGIONS] = {};
	struct ane_h16_process_binding bindings[2];
	struct ane_h16_resource resources[2] = {};
	struct ane_h16_load_plan plan;
	unsigned int i;
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, snapshot);
	KUNIT_ASSERT_EQ(test, ane_h16_load_plan_create(snapshot, &plan), 0);
	fixture_mappings(&plan, mappings);
	for (i = 0; i < 2; i++) {
		bindings[i].hwx_vm = snapshot->info.surfaces[i].section.vm;
		bindings[i].index = 123 + 11 * i;
		bindings[i].class = i;
	}
	ret = ane_h16_load_resources_create(snapshot, mappings, plan.count,
					    bindings, 2, resources);
	KUNIT_ASSERT_EQ(test, ret, 0);
	KUNIT_EXPECT_EQ(test, resources[0].buffer.mapping_dva, mappings[1].dva);
	KUNIT_EXPECT_EQ(test, resources[1].buffer.index, 134);
	KUNIT_EXPECT_EQ(test, resources[1].permissions, 2);
	for (i = 0; i < 7; i++) {
		struct ane_h16_load_mapping saved = mappings[1];

		switch (i) {
		case 0:
			mappings[1].dva++;
			break;
		case 1:
			mappings[1].size++;
			break;
		case 2:
			mappings[1].permissions = 3;
			break;
		case 3:
			mappings[1].dva = mappings[3].dva;
			break;
		case 4:
			mappings[1].dva = 0x10000000000ULL;
			break;
		case 5:
			mappings[1].dva = U64_MAX - SZ_16K + 1;
			break;
		case 6:
			mappings[1].dva = 0;
			break;
		}
		resources[0].buffer.index = 0x1234;
		ret = ane_h16_load_resources_create(snapshot, mappings, plan.count,
						    bindings, 2, resources);
		KUNIT_EXPECT_EQ(test, ret, -EINVAL);
		KUNIT_EXPECT_EQ(test, resources[0].buffer.index, 0x1234);
		mappings[1] = saved;
	}
	/* Indices are class-local, so input 0 and output 0 may coexist. */
	bindings[1].index = bindings[0].index;
	ret = ane_h16_load_resources_create(snapshot, mappings, plan.count,
					    bindings, 2, resources);
	KUNIT_EXPECT_EQ(test, ret, 0);
	bindings[1].index++;
	bindings[1].class = 8;
	ret = ane_h16_load_resources_create(snapshot, mappings, plan.count,
					    bindings, 2, resources);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	bindings[1].class = 1;
	bindings[1].hwx_vm += SZ_16K;
	ret = ane_h16_load_resources_create(snapshot, mappings, plan.count,
					    bindings, 2, resources);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = ane_h16_load_resources_create(snapshot, mappings, plan.count - 1,
					    bindings, 2, resources);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	ret = ane_h16_load_resources_create(snapshot, mappings, plan.count,
					    bindings, 1, resources);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
}

static void hwx_import_bounds_test(struct kunit *test)
{
	const void __user *invalid = (const void __user *)1UL;
	struct ane_h16_hwx_snapshot snapshot = {};
	struct hwx_fixture *f = fixture_create(test, false, false);
	int ret;

	KUNIT_ASSERT_NOT_NULL(test, f);
	ret = ane_h16_hwx_snapshot_from_user(invalid, U64_MAX, &snapshot);
	KUNIT_EXPECT_EQ(test, ret, -EINVAL);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_snapshot_from_user(invalid, 31, &snapshot), -EINVAL);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_snapshot_from_user(invalid, 32, &snapshot), -EFAULT);
	KUNIT_EXPECT_PTR_EQ(test, snapshot.bytes, NULL);
	KUNIT_ASSERT_EQ(test, ane_h16_hwx_snapshot_create(f->bytes, f->size, &snapshot), 0);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_snapshot_create(f->bytes, f->size, &snapshot), -EBUSY);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_snapshot_from_user(invalid, 32, &snapshot), -EBUSY);
	KUNIT_EXPECT_EQ(test, ane_h16_hwx_snapshot_check(&snapshot), 0);
	ane_h16_hwx_snapshot_destroy(&snapshot);
}

/*
 * Open a gap of @len bytes at @at in a fixture's link-edit data (symbols, strings, relocations
 * after the load commands), moving every file offset that points at or past it.
 */
static void fixture_gap(struct hwx_fixture *f, u32 at, u32 len)
{
	u32 end = f->reloc + 8, off;

	memmove(f->bytes + at + len, f->bytes + at, end - at);
	memset(f->bytes + at, 0, len);
	for (off = 8; off <= 16; off += 8) {
		u32 v = get_unaligned_le32(f->bytes + f->symtab + off);

		if (v >= at)
			put_unaligned_le32(v + len, f->bytes + f->symtab + off);
	}
	if (get_unaligned_le32(f->bytes + f->text_command + 72 + 56) >= at)
		put_unaligned_le32(f->reloc + len, f->bytes + f->text_command + 72 + 56);
	if (f->symbols >= at)
		f->symbols += len;
	if (f->strings >= at)
		f->strings += len;
	f->reloc += len;
}

/*
 * Apple's zin_ane_compiler 10.205 H17 output (this Mac's macOS, camera-denoise's low-light
 * network): __TEXT flags 0 beside __DATA, surfaces + kernel from BAR slot 5, an inline
 * LC_NOTE "rt.version", stabs type records and empty kernel objects at the section end.
 * Each form is H17-only; H16 keeps refusing it.
 */
static void hwx_h17_zin_test(struct kunit *test)
{
	struct hwx_fixture *f = fixture_create(test, true, true);
	struct ane_h16_hwx info;
	u8 *c, *sym, *note;
	u32 i, strsize;
	u64 cell[3];

#define H17_OK() KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0)
#define REFUSED() KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), -ENOEXEC)
#define AS_H16(check)							\
	do {								\
		put_unaligned_le32(ANE_HWX_SUBTYPE_H16, f->bytes + 8);	\
		check;							\
		put_unaligned_le32(ANE_HWX_SUBTYPE_H17, f->bytes + 8);	\
	} while (0)

	KUNIT_ASSERT_NOT_NULL(test, f);
	AS_H16(KUNIT_ASSERT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0));
	H17_OK();

	/* __TEXT carries relocations: flags 0 beside a scratch segment; nothing else. */
	put_unaligned_le32(0, f->bytes + f->text_command + 68);
	H17_OK();
	AS_H16(REFUSED());
	put_unaligned_le32(1, f->bytes + f->text_command + 68);
	REFUSED();
	put_unaligned_le32(4, f->bytes + f->text_command + 68);

	/* The surfaces then the kernel from BAR slot 5 (+0x60), in order, nothing after. */
	c = f->bytes + f->program;
	for (i = 0; i < 3; i++)
		cell[i] = get_unaligned_le64(c + 0x70 + i * 16);
	put_unaligned_le64(0, c + 0x90);
	for (i = 0; i < 3; i++)
		put_unaligned_le64(cell[i], c + 0x60 + i * 16);
	H17_OK();
	AS_H16(REFUSED());
	put_unaligned_le64(cell[2], c + 0x90);
	REFUSED();
	put_unaligned_le64(0, c + 0x90);
	put_unaligned_le64(cell[1], c + 0x60);
	put_unaligned_le64(cell[0], c + 0x70);
	REFUSED();
	put_unaligned_le64(0, c + 0x60);	/* back to the slot 6 form */
	for (i = 0; i < 3; i++)
		put_unaligned_le64(cell[i], c + 0x70 + i * 16);
	AS_H16(KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0));

	/* A stabs N_LSYM record: no section, value 0, a type number in n_desc. */
	strsize = get_unaligned_le32(f->bytes + f->symtab + 20);
	fixture_gap(f, f->strings + strsize, 16);	/* its name */
	memcpy(f->bytes + f->strings + strsize, "void:t1=1", 10);
	fixture_gap(f, f->strings, 16);			/* the record */
	sym = f->bytes + f->symbols + 2 * 16;
	put_unaligned_le32(strsize, sym);
	sym[4] = 0x80;
	put_unaligned_le16(1, sym + 6);
	put_unaligned_le32(3, f->bytes + f->symtab + 12);
	put_unaligned_le32(strsize + 16, f->bytes + f->symtab + 20);
	H17_OK();
	AS_H16(REFUSED());
	put_unaligned_le64(0x1000, sym + 8);
	REFUSED();
	put_unaligned_le64(0, sym + 8);
	sym[5] = 1;
	REFUSED();

	/* The same record as an empty kernel object at the end of __kern_0 (64 bytes). */
	sym[4] = 0xf;
	sym[5] = 6;
	put_unaligned_le16(0, sym + 6);
	put_unaligned_le64(cell[2] + 64, sym + 8);
	H17_OK();
	AS_H16(REFUSED());
	put_unaligned_le64(cell[2] + 65, sym + 8);
	REFUSED();
	put_unaligned_le64(cell[2], sym + 8);	/* inside: every form takes it */
	AS_H16(KUNIT_EXPECT_EQ(test, ane_h16_hwx_validate(f->bytes, f->size, &info), 0));

	/* LC_NOTE "rt.version", its 4 bytes inline at +40, zero padding. */
	fixture_gap(f, f->cursor, 48);
	note = f->bytes + f->cursor;
	put_unaligned_le32(0x31, note);
	put_unaligned_le32(48, note + 4);
	strscpy(note + 8, "rt.version", 16);
	put_unaligned_le64(40, note + 24);
	put_unaligned_le64(4, note + 32);
	put_unaligned_le32(0x10001, note + 40);
	f->cursor += 48;
	put_unaligned_le32(get_unaligned_le32(f->bytes + 16) + 1, f->bytes + 16);
	put_unaligned_le32(f->cursor - 32, f->bytes + 20);
	H17_OK();
	AS_H16(REFUSED());
	put_unaligned_le64(48, note + 24);
	REFUSED();
	put_unaligned_le64(40, note + 24);
	note[44] = 1;
	REFUSED();
	note[44] = 0;
	note[8] = 'R';
	REFUSED();
	note[8] = 'r';

	/* Everything together, as the compiler writes it. */
	put_unaligned_le32(0, f->bytes + f->text_command + 68);
	put_unaligned_le64(0, c + 0x90);
	for (i = 0; i < 3; i++)
		put_unaligned_le64(cell[i], c + 0x60 + i * 16);
	put_unaligned_le64(cell[2] + 64, sym + 8);
	H17_OK();
	KUNIT_EXPECT_EQ(test, info.surface_count, 2U);
	KUNIT_EXPECT_EQ(test, info.subtype, ANE_HWX_SUBTYPE_H17);
#undef H17_OK
#undef REFUSED
#undef AS_H16
}

static struct kunit_case hwx_cases[] = {
	KUNIT_CASE(hwx_forms_test),
	KUNIT_CASE(hwx_header_test),
	KUNIT_CASE(hwx_command_test),
	KUNIT_CASE(hwx_tensor_test),
	KUNIT_CASE(hwx_element_format_test),
	KUNIT_CASE(hwx_surface_order_test),
	KUNIT_CASE(hwx_program_test),
	KUNIT_CASE(hwx_staged_counts_test),
	KUNIT_CASE(hwx_final_task_end_test),
	KUNIT_CASE(hwx_geometry_test),
	KUNIT_CASE(hwx_symbols_test),
	KUNIT_CASE(hwx_relocations_test),
	KUNIT_CASE(hwx_snapshot_test),
	KUNIT_CASE(hwx_load_plan_test),
	KUNIT_CASE(hwx_load_original_test),
	KUNIT_CASE(hwx_load_resources_test),
	KUNIT_CASE(hwx_import_bounds_test),
	KUNIT_CASE(hwx_h17_zin_test),
	{}
};

static struct kunit_suite hwx_suite = {
	.name = "ane-h16-hwx",
	.test_cases = hwx_cases,
};

kunit_test_suite(hwx_suite);
MODULE_LICENSE("Dual MIT/GPL");
