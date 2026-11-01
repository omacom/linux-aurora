#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile public C producers and run the production Rust parser offline."""
import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import tempfile


def run(command, **kwargs):
    subprocess.run(command, check=True, **kwargs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesa", type=Path, help="matched public Mesa source")
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[2]
    if sys.byteorder != "little":
        raise SystemExit("C fixtures currently require a little-endian host")
    cc = os.environ.get("CC", "cc")
    rustc = os.environ.get("RUSTC", "rustc")
    fixture_source = root / "tools/asahi/m3-uapi-fixture.c"
    with tempfile.TemporaryDirectory(prefix="asahi-uapi-") as temporary:
        work = Path(temporary)
        headers = work / "headers"
        run(["make", "-s", "-C", str(root), "O=" + str(work / "headers-build"),
             "ARCH=arm64", "headers_install", "INSTALL_HDR_PATH=" + str(headers)])
        sources = [(headers / "include", "drm/asahi_drm.h")]
        if args.mesa:
            sources.append((args.mesa.resolve() / "include", "drm-uapi/asahi_drm.h"))
        fixtures = []
        layouts = []
        prefixes = []
        for index, (includes, header) in enumerate(sources):
            producer = work / ("producer-" + str(index))
            run([cc, "-std=c11", "-Wall", "-Wextra", "-Werror",
                 "-I" + str(includes), '-DASAHI_HEADER="' + header + '"',
                 str(fixture_source), "-o", str(producer)])
            prefix = work / ("prefix-" + str(index))
            with prefix.open("wb") as output:
                run([str(producer)], stdout=output)
            prefixes.append(prefix)
            layout = work / ("layout-" + str(index))
            with layout.open("wb") as output:
                run([str(producer), "--layout"], stdout=output)
            layouts.append(layout)
            fixture = work / ("fixture-" + str(index))
            with fixture.open("wb") as output:
                run([str(producer), "--full"], stdout=output)
            fixtures.append(fixture)
        reference = prefixes[0].read_bytes()
        if any(prefix.read_bytes() != reference for prefix in prefixes[1:]):
            raise SystemExit("kernel/Mesa canonical fixture mismatch")
        print("canonical fixture sha256=" + hashlib.sha256(reference).hexdigest(), flush=True)
        for optimized in (False, True):
            binary = work / ("tests-optimized" if optimized else "tests-debug")
            command = [rustc, "--edition=2021", "--test",
                       str(root / "tools/asahi/m3-uapi-tests.rs"), "-o", str(binary)]
            if optimized:
                command.append("-O")
            run(command)
            for index, fixture in enumerate(prefixes + fixtures):
                env = dict(os.environ, ASAHI_UAPI_FIXTURE=str(fixture),
                           ASAHI_UAPI_LAYOUT=str(layouts[index % len(layouts)]))
                run([str(binary)], env=env)


if __name__ == "__main__":
    main()
