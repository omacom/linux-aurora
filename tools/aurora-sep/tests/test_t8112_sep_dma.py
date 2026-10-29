"""T8112 SEP DMA aperture and DART address admission controls (issue #33).

Set DTC to this kernel's built scripts/dtc/dtc for Apple floating-point cells.
"""

from pathlib import Path
import os
import re
import shutil
import subprocess
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[3]
DTS = ROOT / "arch/arm64/boot/dts/apple"
DTC = os.environ.get("DTC", "dtc")
DART = (ROOT / "drivers/iommu/apple-dart.c").read_text()
PGTABLE = (ROOT / "drivers/iommu/io-pgtable-dart.c").read_text()


def function(source, name):
    match = re.search(r"^static [^;{}]*?\b" + re.escape(name) + r"\(", source, re.M)
    if match is None:
        raise ValueError(f"function not found: {name}")
    start = match.start()
    opening = source.index("{", start)
    depth = 1
    end = opening + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[start:end]


@unittest.skipUnless(all(shutil.which(tool) for tool in ("cpp", DTC, "fdtget", "cc")),
                     "requires cpp, dtc, fdtget and cc")
class T8112SepDmaTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.temp = tempfile.TemporaryDirectory(prefix="aurora-sep-dma-")
        cls.addClassCleanup(cls.temp.cleanup)
        cls.out = Path(cls.temp.name)
        # Compile the actual property parser, including its fail-closed exits.
        probe = DART.split("params_done:\n", 1)[1].split("\n\t/*", 1)[0]
        c_source = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
typedef uint64_t u64;
typedef uint32_t u32;
typedef u64 dma_addr_t;
typedef u64 phys_addr_t;
typedef u64 dart_iopte;
typedef unsigned int gfp_t;
#define DMA_BIT_MASK(n) ((1ULL << (n)) - 1)
#define IS_ALIGNED(x, a) (!((x) & ((a) - 1)))
#define check_add_overflow(a, b, out) __builtin_add_overflow(a, b, out)
#define dev_err(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define dev_info(...) ((void)0)
struct device { void *of_node; };
struct platform_device { struct device dev; };
#define DART_MAX_TTBR 4
#define MAX_DARTS_PER_DEVICE 8
struct apple_dart_fw_root { u64 *entry; u64 **leaf; };
struct apple_dart_hw { u32 tcr, tcr_4level; };
struct apple_dart {
    u32 ias, pgsize, num_streams;
    u64 dma_min, dma_max, dma_offset;
    bool locked, fw_handoff, fw_mirror;
    struct apple_dart_hw *hw;
    int lock;
    const u64 *locked_ttbr[1][DART_MAX_TTBR];
    const u64 *locked_owned[1][DART_MAX_TTBR];
    struct apple_dart_fw_root *locked_fw[1][DART_MAX_TTBR];
};
static u64 supplied[3];
static int of_property_read_u64_array(void *node, const char *name,
                                      u64 *value, size_t count)
{ memcpy(value, supplied, 2 * sizeof(u64)); return 0; }
static int of_property_read_u64(void *node, const char *name, u64 *value)
{ *value = supplied[2]; return 0; }
struct io_pgtable_ops;
struct io_pgtable_ops {
    int (*map_pages)(struct io_pgtable_ops *, unsigned long, phys_addr_t,
                     size_t, size_t, int, gfp_t, size_t *);
    phys_addr_t (*iova_to_phys)(struct io_pgtable_ops *, dma_addr_t);
};
struct iommu_domain { int unused; };
struct apple_dart_atomic_stream_map { struct apple_dart *dart; };
struct apple_dart_domain {
    struct iommu_domain domain;
    struct io_pgtable_ops *pgtbl_ops;
    u64 dma_offset, mask;
    struct apple_dart_atomic_stream_map stream_maps[MAX_DARTS_PER_DEVICE];
};
#define to_dart_domain(domain) ((struct apple_dart_domain *)(domain))
#define for_each_stream_map(i, base, stream_map) \
    for (i = 0, stream_map = &(base)->stream_maps[0]; \
         i < MAX_DARTS_PER_DEVICE && stream_map->dart; \
         stream_map = &(base)->stream_maps[++i])
struct io_pgtable_cfg { struct { int n_levels; } apple_dart_cfg; };
struct io_pgtable { struct io_pgtable_ops ops; struct io_pgtable_cfg cfg; };
#define io_pgtable_ops_to_pgtable(ops) ((struct io_pgtable *)(ops))
#define DART_TCR(dart, sid) ((dart)->hw->tcr + ((sid) << 2))
/* This T8112 SEP domain has no firmware mirror or handoff; firmware-only calls
 * are errors. */
#define spin_lock_irqsave(lock, flags) abort()
#define spin_unlock_irqrestore(lock, flags) abort()
#define READ_ONCE(x) (x)
static unsigned int ilog2(unsigned int value)
{ return 31 - __builtin_clz(value); }
static bool apple_dart_domain_uses_fw(struct apple_dart_domain *domain, int i,
                                    struct apple_dart_atomic_stream_map *map, int sid)
{ abort(); }
static u32 apple_dart_readl(struct apple_dart *dart, u32 reg) { abort(); }
static bool apple_dart_fw_slot_foreign(const u64 *live, const u64 *owned, size_t i)
{ abort(); }
static bool apple_dart_fw_slot_owned(const struct apple_dart_fw_root *fw,
                                    const u64 *live, size_t i)
{ abort(); }
static bool apple_dart_fw_leaf_decode(u64 pte, dma_addr_t iova,
                                     phys_addr_t *phys, int *prot)
{ abort(); }
static bool apple_dart_fw_mapping_matches(phys_addr_t phys, int prot,
                                          phys_addr_t expected, int required)
{ abort(); }
static bool apple_dart_fw_page_matches(const struct apple_dart *dart,
                                       const struct apple_dart_fw_root *fw,
                                       u64 slot, u64 addr, phys_addr_t paddr,
                                       int prot)
{ abort(); }
static bool apple_dart_fw_handoff_page_busy(struct apple_dart *dart,
                                            struct apple_dart_domain *domain, int i,
                                            struct apple_dart_atomic_stream_map *map,
                                            u64 addr)
{ abort(); }
enum { APPLE_DART, APPLE_DART2 };
struct dart_io_pgtable { struct { int fmt; } iop; };
#define GENMASK_ULL(h, l) ((~0ULL >> (63 - (h))) & (~0ULL << (l)))
'''
        c_source += "\n".join(line for line in PGTABLE.splitlines()
                              if line.startswith("#define APPLE_DART") and
                              ("PADDR_MASK" in line or "PADDR_SHIFT" in line))
        c_source += "\n" + "\n".join(function(DART, name) for name in (
            "apple_dart_dma_window", "apple_dart_dma_window_aligned",
            "apple_dart_check_fw_map", "apple_dart_inherited_phys",
            "apple_dart_map_pages", "apple_dart_iova_to_phys"))
        c_source += "\n" + "\n".join(function(PGTABLE, name) for name in (
            "paddr_to_iopte", "iopte_to_paddr"))
        c_source += '\nstatic int admission(struct apple_dart *dart) {\n'
        c_source += 'struct platform_device p = {}; struct platform_device *pdev = &p;\n'
        c_source += 'struct device *dev = &p.dev; u64 dma_range[2]; int ret;\n'
        c_source += probe + '\nreturn 0; err_clk_disable: return ret; }\n'
        c_source += r'''
static u64 seen_iova, seen_phys, encoded;
static struct dart_io_pgtable pgtable = { .iop.fmt = APPLE_DART2 };
static int map(struct io_pgtable_ops *ops, unsigned long iova, phys_addr_t phys,
               size_t pgsize, size_t count, int prot, gfp_t gfp, size_t *mapped)
{
    seen_iova = iova; seen_phys = phys;
    encoded = paddr_to_iopte(phys, &pgtable);
    *mapped = pgsize * count;
    return 0;
}
static phys_addr_t lookup(struct io_pgtable_ops *ops, dma_addr_t iova)
{ return iova == seen_iova ? iopte_to_paddr(encoded, &pgtable) : 0; }
int main(int argc, char **argv)
{
    struct apple_dart dart = { .ias = 32, .pgsize = 0x4000, .num_streams = 1 };
    struct io_pgtable table = {
        .ops = { .map_pages = map, .iova_to_phys = lookup },
        .cfg.apple_dart_cfg.n_levels = 3,
    };
    struct apple_dart_domain domain = {
        .pgtbl_ops = &table.ops, .mask = 0xffffffff,
        .stream_maps = {{ .dart = &dart }},
    };
    size_t mapped;
    assert(argc == 4);
    for (int i = 0; i < 3; ++i) supplied[i] = strtoull(argv[i + 1], NULL, 0);
    int ret = admission(&dart);
    if (ret) { printf("admission=%d\n", ret); return 1; }
    assert(dart.dma_min == 0x4000 && dart.dma_max == 0xffff3fff);
    assert(apple_dart_dma_window(0x4000, 0, &seen_iova) == -EINVAL);
    assert(apple_dart_dma_window(UINT64_MAX, 2, &seen_iova) == -EINVAL);
    assert(apple_dart_dma_window_aligned(0x4000, 0xffff3fff, 0x4000));
    assert(!apple_dart_dma_window_aligned(0x4001, 0xffff3fff, 0x4000));
    assert(!apple_dart_dma_window_aligned(0x4000, 0xffff4000, 0x4000));
    domain.dma_offset = dart.dma_offset;
    assert(!dart.locked && !dart.fw_handoff && !dart.fw_mirror);
    assert(apple_dart_iova_to_phys(&domain.domain, 0x8000) == 0);
    u64 iovas[] = { 0x4000, 0xffff0000 };
    u64 physical[] = { 0x803ffc000, (1ULL << 42) - 0x4000 };
    for (int i = 0; i < 2; ++i) for (int j = 0; j < 2; ++j) {
        assert(apple_dart_map_pages(&domain.domain, iovas[i], physical[j],
                                   0x4000, 1, 3, 0, &mapped) == 0);
        assert(mapped == 0x4000 && seen_iova == iovas[i]);
        assert(seen_phys == physical[j]);
        assert(apple_dart_iova_to_phys(&domain.domain, iovas[i]) == physical[j]);
    }
    puts("admission=0; low IOVA / high physical page round trips pass");
    return 0;
}
'''
        source = cls.out / "admission.c"
        source.write_text(c_source)
        cls.admission = cls.out / "admission"
        subprocess.run(["cc", "-std=gnu11", "-Werror", str(source), "-o", str(cls.admission)],
                       check=True, capture_output=True)

    def compile_board(self, board):
        pp = subprocess.run(["cpp", "-nostdinc", "-undef", "-D__DTS__", "-x",
                             "assembler-with-cpp", "-I", str(ROOT / "include"),
                             "-I", str(DTS), str(DTS / f"t8112-{board}.dts")],
                            check=True, capture_output=True)
        output = self.out / f"{board}.dtb"
        subprocess.run([DTC, "-q", "-I", "dts", "-O", "dtb", "-o", str(output)],
                       input=pp.stdout, check=True, capture_output=True)
        return output

    def prop(self, dtb, node, name, kind="x"):
        return subprocess.run(["fdtget", "-t", kind, str(dtb), node, name],
                              check=True, capture_output=True, text=True).stdout.split()

    def test_compiled_t8112_boards_remain_disabled_with_low_aperture(self):
        for board in ("j413", "j415", "j473", "j493"):
            with self.subTest(board=board):
                dtb = self.compile_board(board)
                dart = "/soc/iommu@25d2c0000"
                self.assertEqual(self.prop(dtb, dart, "status", "s"), ["disabled"])
                self.assertEqual(self.prop(dtb, "/soc/sep@25e400000", "status", "s"),
                                 ["disabled"])
                offset = [int(cell, 16) for cell in self.prop(dtb, dart, "apple,dma-offset")]
                window = [int(cell, 16) for cell in self.prop(dtb, dart, "apple,dma-range")]
                self.assertEqual(offset, [0, 0])
                self.assertEqual(window, [0, 0x4000, 0, 0xffff0000])
                subprocess.run([str(self.admission), "0x4000", "0xffff0000", "0"],
                               check=True, capture_output=True)

    def test_probe_rejects_old_alias_and_malformed_windows(self):
        cases = [(0, 1 << 32, 1 << 40), (0x4000, 0xffff0000, 1 << 40),
                 (0x4001, 0xffff0000, 0), (0x4000, 0xffff0001, 0),
                 (0x4000, 0, 0), (0x4000, 1 << 32, 0),
                 (0x4000, 0xffff0000, 0x10000)]
        for window in cases:
            with self.subTest(window=window):
                proc = subprocess.run([str(self.admission), *map(str, window)],
                                      capture_output=True, text=True)
                self.assertEqual(proc.returncode, 1, proc.stdout + proc.stderr)
                self.assertIn("admission=-22", proc.stdout)

    def test_j493_sensor_and_manifest_handoff(self):
        dtb = self.compile_board("j493")
        self.assertEqual(self.prop(dtb, "/aliases", "sep", "s"),
                         ["/soc/sep@25e400000"])
        bus = "/soc/spi@235108000"
        sensor = bus + "/fingerprint@0"
        self.assertEqual(self.prop(dtb, bus, "status", "s"), ["okay"])
        self.assertEqual(self.prop(dtb, sensor, "status", "s"), ["okay"])
        self.assertEqual(self.prop(dtb, sensor, "firmware-name", "s"),
                         ["apple/mesacal-j493.bin"])
        power = [int(value, 16) for value in self.prop(dtb, sensor, "enable-gpios")]
        gpio = int(self.prop(dtb, "/soc/pinctrl@23c100000", "phandle")[0], 16)
        self.assertEqual(power, [gpio, 178, 0])
        self.assertEqual(self.prop(dtb, sensor, "spi-max-frequency", "u"), ["8000000"])
        self.assertEqual(self.prop(dtb, sensor, "spi-cs-setup-delay-ns", "u"), ["20"])
        self.assertEqual(self.prop(dtb, sensor, "spi-cs-hold-delay-ns", "u"), ["20"])
        properties = subprocess.run(["fdtget", "-p", str(dtb), sensor],
                                    check=True, capture_output=True, text=True).stdout.split()
        self.assertIn("spi-cpha", properties)
        self.assertNotIn("spi-cpol", properties)
        self.assertNotIn("interrupts", properties)
        self.assertNotIn("interrupts-extended", properties)
        bus_properties = subprocess.run(["fdtget", "-p", str(dtb), bus],
                                        check=True, capture_output=True, text=True).stdout.split()
        self.assertNotIn("cs-gpios", bus_properties)


if __name__ == "__main__":
    unittest.main()
