#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the production fdinfo residency helpers with cached and imported backing."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
SOURCE = ROOT / "drivers/gpu/drm/drm_gem_shmem_helper.c"


def function(text, signature):
    start = text.index(signature)
    opening = text.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    original = SOURCE.read_text()
    functions = "\n".join(function(original, signature) for signature in [
        "size_t drm_gem_shmem_object_rss(",
        "enum drm_gem_object_status drm_gem_shmem_object_status(",
    ])
    fixture = r'''
#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#define READ_ONCE(x) (*(volatile __typeof__(x) *)&(x))
#define min_t(type,a,b) ((type)(a) < (type)(b) ? (type)(a) : (type)(b))
#define PAGE_SIZE (1UL << PAGE_SHIFT)
struct address_space { unsigned long nrpages; };
struct file { struct address_space *f_mapping; };
struct drm_gem_object { size_t size; void *import_attach; struct file *filp; };
enum drm_gem_object_status { DRM_GEM_OBJECT_RESIDENT = 1 };
FUNCTIONS
int main(void) {
    struct address_space mapping = {0};
    struct file backing = { .f_mapping = &mapping };
    struct drm_gem_object obj = { .size = 5 * PAGE_SIZE };
    assert(drm_gem_shmem_object_rss(&obj) == 0);
    assert(drm_gem_shmem_object_status(&obj) == 0);
    obj.filp = &backing;
    assert(drm_gem_shmem_object_rss(&obj) == 0);
    mapping.nrpages = 2;
    assert(drm_gem_shmem_object_rss(&obj) == 2 * PAGE_SIZE);
    assert(drm_gem_shmem_object_status(&obj) == DRM_GEM_OBJECT_RESIDENT);
    /* A cached backing remains resident independently of a device page-array pin. */
    mapping.nrpages = 5;
    assert(drm_gem_shmem_object_rss(&obj) == obj.size);
    mapping.nrpages = 9;
    assert(drm_gem_shmem_object_rss(&obj) == obj.size);
    mapping.nrpages = 0;
    assert(drm_gem_shmem_object_rss(&obj) == 0);
    assert(drm_gem_shmem_object_status(&obj) == 0);
    obj.filp = NULL;
    obj.import_attach = &mapping;
    assert(drm_gem_shmem_object_rss(&obj) == obj.size);
    assert(drm_gem_shmem_object_status(&obj) == DRM_GEM_OBJECT_RESIDENT);
    assert(obj.size == 5 * PAGE_SIZE && mapping.nrpages == 0);
    puts("PASS unpopulated, cached, partial, bounded, evicted and imported backing");
}
'''.replace("FUNCTIONS", functions)
    source = args.output / "fdinfo.c"
    source.write_text(fixture)
    results = []
    for shift in [12, 14]:
        binary = args.output / f"fdinfo-{shift}"
        subprocess.run(["cc", "-std=gnu11", "-Wall", "-Werror", "-g",
                        "-fsanitize=address,undefined", "-fno-pie", "-no-pie",
                        f"-DPAGE_SHIFT={shift}", str(source), "-o", str(binary)], check=True)
        result = subprocess.run([str(binary)], capture_output=True, text=True, check=True)
        results.append({"page_shift": shift, "stdout": result.stdout, "stderr": result.stderr})
    record = {"production_sha256": hashlib.sha256(functions.encode()).hexdigest(),
              "results": results,
              "scope": "Actual C helper bodies with typed file/cache boundary stubs; not kernel ABI or hardware execution."}
    (args.output / "result.json").write_text(json.dumps(record, indent=2) + "\n")
    print(json.dumps(record, indent=2))


if __name__ == "__main__":
    main()
