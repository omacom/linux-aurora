#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile the production J613 PMGR predicate and test PMGR fields and unrelated bits."""
from pathlib import Path
import os
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
source = (root / "drivers/gpu/drm/apple/j613_dcp_activate.c").read_text()
start = source.index("static bool j613_power_state_active(")
end = source.index("\n}\n", start) + 3
stub = r"""
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
typedef uint32_t u32;
#define BIT(n) (1U << (n))
#define GENMASK(a,b) (((1U << ((a)-(b)+1))-1) << (b))
#define FIELD_GET(mask,v) (((v) & (mask)) >> __builtin_ctz(mask))
"""
main = r"""
int main(void) {
 /* Only ACTUAL[7:4], TARGET[3:0] and AUTO_ENABLE[28] affect readiness. */
 const u32 unrelated_mask = ~(GENMASK(7, 0) | BIT(28));
 u32 unrelated[32] = {0, unrelated_mask,
                      0xaaaaaaaaU & unrelated_mask,
                      0x55555555U & unrelated_mask};
 unsigned int count = 4;
 for (unsigned int bit = 8; bit < 32; bit++)
  if (bit != 28) unrelated[count++] = BIT(bit);
 for (unsigned int i = 0; i < count; i++)
 for (u32 actual = 0; actual < 16; actual++)
 for (u32 target = 0; target < 16; target++)
 for (u32 automatic = 0; automatic < 2; automatic++) {
  u32 word = unrelated[i] | (actual << 4) | target | (automatic << 28);
  bool expected = actual == 15 || (target == 15 && automatic);
  if (j613_power_state_active(word) != expected) {
   fprintf(stderr, "PMGR state 0x%08x: expected active=%d\n", word, expected);
   return 1;
  }
 }
 if (!j613_power_state_active(0x1404024f)) return 2;
 if (j613_power_state_active(0x0404024f)) return 3;
 if (j613_power_state_active(0x14040240)) return 4;
 puts("PASS: 13824 PMGR combinations and captured-state regression controls");
 return 0;
}
"""
with tempfile.TemporaryDirectory() as temporary:
    path = Path(temporary) / "power.c"
    path.write_text(stub + source[start:end] + main)
    for flags in (["-O0"], ["-O2"]):
        exe = str(path.with_suffix(""))
        subprocess.run([os.environ.get("CC", "cc"), "-Wall", "-Wextra", "-Werror",
                        *flags, str(path), "-o", exe], check=True)
        subprocess.run([exe], check=True)
