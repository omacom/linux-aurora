#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise the actual AVD capability-tail block under ASan/UBSan.

Firmware loading, MMIO and device logging are not exercised. The firmware and
variant structures, kernel BIT helper and bounded strnstr are host stand-ins.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
SOURCE = "drivers/media/platform/apple/avd/avd-drv.c"
HEADER = "drivers/media/platform/apple/avd/avd.h"


def read(path, ref):
    if ref:
        return subprocess.check_output(
            ["git", "-C", str(ROOT), "show", f"{ref}:{path}"], text=True
        )
    return (ROOT / path).read_text()


def build_source(body, defines):
    return r'''
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define BIT(n) (1u << (n))
#define dev_info(...) ((void)0)
''' + defines + r'''
struct firmware { const unsigned char *data; size_t size; };
struct variant { unsigned int capabilities; };
struct avd_dev {
    struct firmware *fw;
    const struct variant *variant;
    unsigned int capabilities;
};
static char *strnstr(const char *s, const char *needle, size_t n)
{
    size_t len = strlen(needle);
    for (size_t i = 0; i + len <= n && s[i]; i++)
        if (!memcmp(s + i, needle, len))
            return (char *)s + i;
    return NULL;
}
static unsigned int probe_tail(struct avd_dev *avd)
{
    const char *codec_info = NULL;
''' + body + r'''
    return avd->capabilities;
}
int main(int argc, char **argv)
{
    if (argc != 5)
        return 2;
    size_t size = strtoul(argv[1], NULL, 0);
    unsigned char *data = calloc(size ? size : 1, 1);
    if (!data)
        return 2;
    struct firmware fw = {data, size};
    struct variant variant = {strtoul(argv[3], NULL, 0)};
    struct avd_dev avd = {&fw, &variant, 0};
    size_t len = strlen(argv[2]);
    if (size >= AVD_CODEC_INFO_MAX_SIZE && len <= AVD_CODEC_INFO_MAX_SIZE)
        memcpy(data + size - AVD_CODEC_INFO_MAX_SIZE, argv[2], len);
    unsigned int actual = probe_tail(&avd);
    unsigned int expected = strtoul(argv[4], NULL, 0);
    printf("size=%zu variant=%u capabilities=%u expected=%u\n",
           size, variant.capabilities, actual, expected);
    free(data);
    return actual == expected ? 0 : 1;
}
'''


def run_variant(out, name, body, defines, cases):
    dest = out / name
    dest.mkdir()
    source = dest / "actual-tail.c"
    source.write_text(build_source(body, defines))
    cmd = shlex.split(os.environ.get("CC", "cc")) + [
        # Match scripts/Makefile.warn for the existing firmware u8*/char* use.
        "-std=gnu11", "-g", "-O1", "-Wall", "-Wextra", "-Werror", "-Wno-pointer-sign",
        "-fsanitize=address,undefined", "-fno-omit-frame-pointer",
        str(source), "-o", str(dest / "actual-tail"),
    ]
    build = subprocess.run(cmd, capture_output=True, text=True)
    (dest / "build.log").write_text(build.stdout + build.stderr)
    rows = []
    if build.returncode == 0:
        for index, (size, tail, mask, expected) in enumerate(cases):
            result = subprocess.run(
                [str(dest / "actual-tail"), str(size), tail, str(mask), str(expected)],
                capture_output=True, text=True,
            )
            (dest / f"case-{index:02d}.log").write_text(result.stdout + result.stderr)
            rows.append({"size": size, "tail": tail, "variant": mask,
                         "expected": expected, "returncode": result.returncode,
                         "sanitizer_diagnostic": bool(re.search(
                             r"AddressSanitizer|runtime error:", result.stderr))})
    return {"name": name, "build_command": cmd,
            "build_returncode": build.returncode, "cases": rows,
            "failed_cases": sum(x["returncode"] != 0 or x["sanitizer_diagnostic"]
                                for x in rows)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ref", help="extract production code from this Git ref")
    parser.add_argument("--output", type=Path, required=True,
                        help="new evidence directory (must not already exist)")
    parser.add_argument("--mutations", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    source, header = read(SOURCE, args.ref), read(HEADER, args.ref)
    start = source.index("\t/* Determine firmware and HW variant capabilities */")
    end = source.index("\n\tret = dma_set_mask_and_coherent", start)
    body = source[start:end]
    wanted = ["AVD_CODEC_INFO_MAX_SIZE", "AVD_CAPABILITY_AV1", "AVD_CAPABILITY_H264",
              "AVD_CAPABILITY_HEVC", "AVD_CAPABILITY_VP9"]
    defines = ""
    for macro in wanted:
        matches = re.findall(r"^#define\s+" + macro + r"\s+[^\n]+$",
                             source + "\n" + header, re.M)
        if len(matches) != 1:
            raise ValueError(f"expected exactly one definition of {macro}")
        defines += matches[0] + "\n"
    # Codec bits are read from the header above; the expected masks below track
    # the current public ABI (HEVC/H264/VP9/AV1 = bits 0/1/2/3).
    all_codecs = "CODECS:av1 h264 h265 vp9"
    cases = [(n, "legacy firmware", 15, 15) for n in (0, 1, 7, 63, 64, 128)]
    cases += [(n, all_codecs, 15, 15) for n in (64, 128)]
    cases += [(64, "CODECS:" + codec, 15, bit)
              for codec, bit in (("av1", 8), ("h264", 2), ("h265", 1), ("vp9", 4))]
    cases += [(128, all_codecs, 3, 3), (128, all_codecs, 0, 0),
              (64, "CODECS:vvc", 15, 0), (64, "CODECS:", 15, 0),
              (64, "CODECS:" + "x" * 57, 15, 0),
              (64, "CODECSx:av1", 5, 5), (128, "legacy", 0, 0),
              (128, "legacy", 5, 5)]
    variants = [("control-before", body)]
    if args.mutations:
        anchor = "\tif (avd->fw->size >= AVD_CODEC_INFO_MAX_SIZE)\n"
        if body.count(anchor) != 1:
            raise ValueError("size-guard mutation did not match exactly once")
        variants += [("no-size-guard", body.replace(anchor, "")),
                     ("no-variant-intersection", body.replace(
                         "\t\tunsigned int variant_caps = avd->variant->capabilities;\n", ""
                     ).replace("caps |= variant_caps & ", "caps |= ")),
                     ("no-legacy-fallback", body.replace(
                         "avd->capabilities = avd->variant->capabilities;",
                         "avd->capabilities = 0;"))]
        variants.append(("control-after", body))
    results = [run_variant(args.output, name, text, defines, cases)
               for name, text in variants]
    ok = True
    for result in results:
        is_control = result["name"].startswith("control-")
        valid = result["build_returncode"] == 0 and len(result["cases"]) == len(cases)
        valid &= (result["failed_cases"] == 0) if is_control else (result["failed_cases"] > 0)
        result["verdict"] = ("PASS" if is_control else "KILLED") if valid else "FAIL"
        ok &= valid
    compiler = subprocess.check_output(
        shlex.split(os.environ.get("CC", "cc")) + ["--version"], text=True
    ).splitlines()[0]
    receipt = {"ref": args.ref, "compiler": compiler,
               "source_sha256": hashlib.sha256(source.encode()).hexdigest(),
               "body_sha256": hashlib.sha256(body.encode()).hexdigest(),
               "defines_sha256": hashlib.sha256(defines.encode()).hexdigest(),
               "cases_per_variant": len(cases), "results": results, "passed": ok}
    (args.output / "receipt.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps({"passed": ok, "cases": len(cases),
                      "variants": [{k: x[k] for k in ("name", "failed_cases", "verdict")}
                                   for x in results]}))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
