#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Host regression test for the production Apple SPI clock/timeout arithmetic.

Run from the repository root. Requires a C compiler; uses no hardware.
"""
import os
from pathlib import Path
import re
import subprocess
import tempfile

source = Path("drivers/spi/spi-apple.c").read_text()
prep = re.search(r"static bool apple_spi_prep_transfer\(.*?\n}", source, re.S)[0]
timeout = re.search(r"unsigned long ms = APPLE_SPI_TIMEOUT_MS \+.*?;", source, re.S)[0]
program = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef uint32_t u32;
typedef uint64_t u64;
#define DIV_ROUND_UP(n, d) (((n) + (d) - 1) / (d))
#define DIV_ROUND_UP_ULL DIV_ROUND_UP
#define min_t(t, a, b) ((t)(a) < (t)(b) ? (t)(a) : (t)(b))
#define max(a, b) ((a) > (b) ? (a) : (b))
#define FIELD_PREP(a, b) (b)
#define APPLE_SPI_CLKDIV 0x30
#define APPLE_SPI_CLKDIV_MAX 0x7ff
#define APPLE_SPI_SHIFTCFG 0
#define APPLE_SPI_SHIFTCFG_BITS 0
#define APPLE_SPI_FIFO_DEPTH 16
#define APPLE_SPI_TIMEOUT_MS 200
struct apple_spi { unsigned long clk; u32 div; };
struct spi_transfer { u32 speed_hz, bits_per_word, effective_speed_hz; };
#define clk_get_rate(c) (c)
#define reg_write(s, r, v) ((s)->div = (v))
#define reg_mask(s, r, m, v) ((void)0)
""" + prep + r"""
static unsigned long budget(u32 words, struct spi_transfer *t)
{
""" + timeout + r"""
return ms;
}
int main(void)
{
    struct apple_spi s;
    struct spi_transfer t = { .bits_per_word = 8 };
    const struct { unsigned long clk; u32 hz, div, rate, ms; } cases[] = {
        {24000000, 8000000, 3, 8000000, 230},
        {200000000, 8000000, 25, 8000000, 230},
        {24000000, 7000000, 4, 6000000, 239},
        {24000000, 23000000, 2, 12000000, 220},
        {24000000, 1, 2047, 11724, 20115},
    };
    for (unsigned int i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        s.clk = cases[i].clk;
        t.speed_hz = cases[i].hz;
        bool poll = apple_spi_prep_transfer(&s, &t);
        assert(s.div == cases[i].div);
        assert(t.effective_speed_hz == cases[i].rate);
        assert(budget(29184, &t) == cases[i].ms);
        assert(poll == (12800000U <= cases[i].rate));
    }
    /* A rounded divider must not receive the shorter requested-rate budget. */
    s.clk = 24000000; t.speed_hz = 7000000;
    apple_spi_prep_transfer(&s, &t);
    assert(budget(29184, &t) > 200 + DIV_ROUND_UP_ULL(29184ULL * 8000, 7000000));
    puts("PASS: 5 divider/rate/deadline/poll cases and rounding regression");
}
"""
with tempfile.TemporaryDirectory(prefix="apple-spi-clock-") as work:
    c = Path(work) / "test.c"
    exe = Path(work) / "test"
    c.write_text(program)
    subprocess.run([os.environ.get("CC", "cc"), "-Wall", "-Wextra", "-Werror",
                    "-fsanitize=undefined,address", str(c), "-o", str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
