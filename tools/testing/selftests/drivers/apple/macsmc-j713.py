#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the production SMC GPIO callbacks with a mocked SMC transport."""
import os
from pathlib import Path
import re
import shlex
import subprocess
import tempfile


def block(source, marker):
    start = source.index(marker)
    brace = source.index("{", start)
    depth = 1
    end = brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


def main():
    root = Path(__file__).resolve().parents[5]
    source = (root / "drivers/gpio/gpio-macsmc.c").read_text()
    header = (root / "include/linux/mfd/macsmc.h").read_text()
    t8132 = (root / "include/linux/soc/apple/t8132.h").read_text()
    defines = "\n".join(re.findall(
        r"^#define (?:MAX_GPIO|MACSMC_\w+|CMD_\w+|MODE_\w+)\b[^\n]*", source, re.M))
    key_defines = "\n".join(re.findall(r"^#define (?:_SMC_KEY\(s\)|SMC_KEY\(s\))[^\n]*", header, re.M))
    parts = [r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;
typedef uint64_t u64;
typedef uint32_t smc_key;
struct device { int unused; };
struct apple_smc { int unused; };
struct gpio_chip { void *data; };
struct mutex { bool held; };
#define DECLARE_BITMAP(name, bits) unsigned long name[((bits) + 63) / 64]
#define GPIO_LINE_DIRECTION_OUT 0
#define GPIO_LINE_DIRECTION_IN 1
#define dev_err(...) do { } while (0)
#define dev_err_ratelimited(...) do { } while (0)
static void mutex_init(struct mutex *m) { m->held = false; }
static void mutex_lock(struct mutex *m) { assert(!m->held); m->held = true; }
static void mutex_unlock(struct mutex *m) { assert(m->held); m->held = false; }
static void *gpiochip_get_data(struct gpio_chip *gc) { return gc->data; }
static const char hex_asc[] = "0123456789abcdef";
#define hex_asc_hi(n) hex_asc[((n) & 0xf0) >> 4]
#define hex_asc_lo(n) hex_asc[(n) & 0x0f]
static bool test_bit(unsigned int nr, const unsigned long *b)
{ return !!(b[nr / 64] & (1UL << (nr % 64))); }
static void assign_bit(unsigned int nr, unsigned long *b, bool value)
{ if (value) b[nr / 64] |= 1UL << (nr % 64); else b[nr / 64] &= ~(1UL << (nr % 64)); }
''', defines, key_defines, block(source, "struct macsmc_gpio { ".rstrip()) + ";", r'''
static unsigned int checks, rw_count, writes32, writes64;
static int write_result, rw_result;
static smc_key last_key;
static u32 last32, last_cmd;
static u64 last64;
static u32 read_value, read_direction = MODE_OUTPUT;
static struct macsmc_gpio *active;
static bool expect_native;
#define CHECK(expr) do { checks++; assert(expr); } while (0)
static int apple_smc_rw_u32(struct apple_smc *smc, smc_key key, u32 command, u32 *v)
{
    (void)smc;
    assert(!expect_native);
    rw_count++; last_key = key; last_cmd = command;
    *v = command == CMD_PINMODE ? read_direction : read_value;
    return rw_result;
}
static int apple_smc_write_u32(struct apple_smc *smc, smc_key key, u32 value)
{
    (void)smc;
    if (expect_native) assert(active->j713_reg_on_lock.held);
    writes32++; last_key = key; last32 = value;
    return write_result;
}
static int apple_smc_write_u64(struct apple_smc *smc, smc_key key, u64 value)
{
    (void)smc;
    assert(!expect_native);
    writes64++; last_key = key; last64 = value;
    return write_result;
}
''']
    for name in ("macsmc_gpio_is_j700_pcio", "macsmc_gpio_has_output_template",
                 "macsmc_gpio_is_j713_reg_on", "macsmc_gpio_key",
                 "macsmc_gpio_get_direction", "macsmc_gpio_get", "macsmc_gpio_set"):
        marker = re.search(r"^static [^\n]+\b" + name + r"\(", source, re.M).group()
        parts.append(block(source, marker))
    # Compile the actual probe assignments, so the test also checks board selection.
    assignments = re.search(r"\tsmcgp->j713 = [^;]+;\n\tmutex_init\([^;]+;", source).group()
    parts += [r'''
/*
 * No board, then every M4 Mac with the Airs' radio port, then the J716s,
 * whose N1 power line is the same gP13 template.
 */
static const char *const boards[] = {
    NULL, "apple,j713", "apple,j715", "apple,j604", "apple,j623", "apple,j624",
    "apple,j773g", "apple,j716s",
};
#define NUM_BOARDS (sizeof(boards) / sizeof(*boards))
static unsigned int board;
static bool of_machine_is_compatible(const char *name)
{
    bool known = false;
    for (unsigned int i = 1; i < NUM_BOARDS; i++)
        known |= !strcmp(name, boards[i]);
    assert(known);
    return board && !strcmp(name, boards[board]);
}
''', block(t8132, "static inline bool apple_t8132_air("),
        block(t8132, "static inline bool apple_t8132_pro_mini_imac("),
        block(t8132, "static inline bool apple_t8132_managed_radio("), r'''
static void profile_init(struct macsmc_gpio *smcgp)
{
''', assignments, r'''
}
static void reset(struct macsmc_gpio *g, unsigned int which, smc_key base)
{
    memset(g, 0, sizeof(*g));
    g->gc.data = g; g->base_key = base;
    board = which; profile_init(g);
    active = g; expect_native = false; write_result = rw_result = 0;
    writes32 = writes64 = rw_count = 0;
}
int main(void)
{
    struct macsmc_gpio g;
    reset(&g, true, _SMC_KEY("gP\0\0"));
    expect_native = true;
    CHECK(macsmc_gpio_get_direction(&g.gc, 0x13) == GPIO_LINE_DIRECTION_OUT);
    CHECK(macsmc_gpio_get(&g.gc, 0x13) == -EOPNOTSUPP);
    CHECK(rw_count == 0 && writes32 == 0 && writes64 == 0);
    /* A failed first command must not fabricate a cached level. */
    write_result = -EIO;
    CHECK(macsmc_gpio_set(&g.gc, 0x13, 1) == -EIO);
    CHECK(last_key == SMC_KEY(gP13) && last32 == 0x00800001);
    CHECK(!g.j713_reg_on_known && !g.j713_reg_on_level);
    CHECK(macsmc_gpio_get(&g.gc, 0x13) == -EOPNOTSUPP);
    write_result = 0;
    CHECK(macsmc_gpio_set(&g.gc, 0x13, 1) == 0);
    CHECK(last_key == SMC_KEY(gP13) && last32 == 0x00800001);
    CHECK(g.j713_reg_on_known && g.j713_reg_on_level);
    CHECK(macsmc_gpio_get(&g.gc, 0x13) == 1);
    unsigned int saved_writes = writes32;
    CHECK(macsmc_gpio_get_direction(&g.gc, 0x13) == GPIO_LINE_DIRECTION_OUT);
    CHECK(writes32 == saved_writes && rw_count == 0 && writes64 == 0);
    /* An error may affect hardware; this is only the last successful command. */
    write_result = -ETIMEDOUT;
    CHECK(macsmc_gpio_set(&g.gc, 0x13, 0) == -ETIMEDOUT);
    CHECK(last32 == 0x00800000 && g.j713_reg_on_known && g.j713_reg_on_level);
    CHECK(macsmc_gpio_get(&g.gc, 0x13) == 1);
    write_result = 0;
    CHECK(macsmc_gpio_set(&g.gc, 0x13, 0) == 0);
    CHECK(last32 == 0x00800000 && macsmc_gpio_get(&g.gc, 0x13) == 0);
    CHECK(macsmc_gpio_set(&g.gc, 0x13, -9) == 0);
    CHECK(last32 == 0x00800001 && macsmc_gpio_get(&g.gc, 0x13) == 1);
    CHECK(!g.j713_reg_on_lock.held && rw_count == 0 && writes64 == 0);

    /* Every other board/namespace/line retains the generic GPIO operations. */
    for (unsigned int b = 0; b < NUM_BOARDS; b++)
        for (unsigned int lower = 0; lower < 2; lower++)
            for (unsigned int line = 0; line < MAX_GPIO; line++) {
                reset(&g, b, lower ? _SMC_KEY("gp\0\0") : _SMC_KEY("gP\0\0"));
                bool native = b && !lower && line == 0x13;
                CHECK(macsmc_gpio_is_j713_reg_on(&g, line) == native);
                if (native) continue;
                CHECK(macsmc_gpio_get_direction(&g.gc, line) == GPIO_LINE_DIRECTION_OUT);
                CHECK(rw_count == 1 && last_cmd == CMD_PINMODE);
                CHECK(macsmc_gpio_set(&g.gc, line, 1) == 0);
                CHECK(writes32 == 1 && last32 == (CMD_OUTPUT | 1));
                CHECK(last_key == (smc_key)macsmc_gpio_key(g.base_key, line));
                CHECK(!g.j713_reg_on_known && writes64 == 0);
                read_direction = MODE_INPUT; read_value = 1;
                CHECK(macsmc_gpio_get(&g.gc, line) == 1);
                CHECK(last_cmd == CMD_INPUT && rw_count == 3);
                read_direction = MODE_OUTPUT;
            }
    reset(&g, true, _SMC_KEY("gP\0\0"));
    CHECK(!macsmc_gpio_is_j713_reg_on(&g, 0x113));
    /* J700's existing eight-byte pcIO contracts remain unchanged. */
    reset(&g, false, _SMC_KEY("gP\0\0")); g.j700_pcio = true;
    CHECK(macsmc_gpio_set(&g.gc, MACSMC_J700_RADIO_REG_ON_GPIO, 1) == 0);
    CHECK(writes64 == 1 && writes32 == 0 && last_key == SMC_KEY(pcIO));
    CHECK(last64 == 0x0008000000800001ULL);
    CHECK(macsmc_gpio_get(&g.gc, MACSMC_J700_RADIO_REG_ON_GPIO) == 1);
    CHECK(macsmc_gpio_set(&g.gc, MACSMC_J700_AFE_RESET_GPIO, 1) == 0);
    CHECK(last64 == 0x0013000000010001ULL && writes64 == 2);
    CHECK(macsmc_gpio_get_direction(&g.gc, MACSMC_J700_AFE_RESET_GPIO) == GPIO_LINE_DIRECTION_OUT);
    CHECK(rw_count == 0 && !g.j713_reg_on_known);
    printf("ok 1 - %u production SMC GPIO checks passed\n", checks);
    return 0;
}
''']
    with tempfile.TemporaryDirectory(prefix="macsmc-j713-") as temp:
        c = Path(temp) / "test.c"
        binary = Path(temp) / "test"
        c.write_text("\n".join(parts))
        subprocess.run(shlex.split(os.environ.get("HOSTCC", "cc")) + [
            "-std=c11", "-Wall", "-Wextra", "-Werror", str(c), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
