// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The GPU regions, firmware segments and performance states: from the boot loader's ADT
//! export (T6030), or from the GPU device tree node the boot loader fills (T6031).
//!
//! # T6030: the ADT export
//!
//! The GPU node that the PMP/ADT handoff (`drivers/soc/apple/j514s-pmp-handoff.c`) synthesizes
//! has only `compatible = "apple,agx-t6030"`, `reg`/`reg-names` (`asc`, `sgx`), `mboxes`/
//! `mbox-names` (`mbox`), `power-domains` and `apple,pmp`; it has no memory regions, operating
//! points or firmware-compat. Regions, segments and performance states come from the ADT:
//! `/reserved-memory` child `compatible = "phram"`, `label = "adt"` (16 KiB aligned, at most
//! 2 MiB; copied, never parsed in place). Properties: `/arm-io/sgx` `compatible == "gpu,t6030"`,
//! `{gpu-region,gfx-shared-region,gfx-handoff,gfx-shared-l2-region}-{base,size}`,
//! `perf-states`/`perf-states-sram` (14 x {u32 Hz, u32 mV}) and the fixed constants checked by
//! `collect_perf`; `/arm-io/gfx-asc` `segment-ranges` (2 x 32 bytes) and `segment-names`
//! (`__TEXT;__DATA`); `/arm-io/gfx-asc/iop-gfx-nub` `segment-ranges` (identical, length
//! checked too) and `pre-loaded == 1`; `/chosen` `dram-base`/`dram-size`.
//! Admission: every region page-aligned, inside DRAM, disjoint from System RAM and from each
//! other; TEXT at [`super::hw::UPPER`], DATA right after it.
//!
//! # T6031: the device tree node
//!
//! The boot loader fills the kernel's GPU node from the machine's ADT: `memory-region` entries
//! `ttbs`, `pagetables`, `handoff` and `pagetables-l2` (the UAT regions), `fw-text` and
//! `fw-data` (the loaded segments, no-map), `apple,firmware-segment-vas` (their two VAs) and
//! the `operating-points-v2` table (one `opp-hz` per performance state, `opp-microvolt` with
//! one cell per cluster). The same admission applies, without the DRAM bounds, and the four UAT
//! regions may lie in System RAM: they are reserved-memory nodes without `no-map`, which the
//! kernel maps, so the driver reaches them through its own mapping ([`super::regs::Mapping`]).

use core::ops::Range;
use kernel::{c_str, device, prelude::*};

use super::boot::glue;
use super::hw;

/// Number of ADT performance states of the board-image layout.
pub(crate) const PERF_STATES: usize = 14;

/// What the driver takes from the ADT.
pub(crate) struct GpuAdt {
    /// TTBAT page.
    pub(crate) ttbs: Range<u64>,
    /// Firmware kernel root and its tables.
    pub(crate) tables: Range<u64>,
    /// PPL handoff page.
    pub(crate) handoff: Range<u64>,
    /// Shared L2 page: the table of root entry 2 (the host window).
    pub(crate) shared_l2: Range<u64>,
    /// Loaded firmware TEXT and DATA, physical.
    pub(crate) text: Range<u64>,
    pub(crate) data: Range<u64>,
    /// Performance states: MHz, core mV, SRAM mV.
    pub(crate) perf_mhz: [u32; PERF_STATES],
    pub(crate) perf_mv: [u32; PERF_STATES],
    pub(crate) sram_mv: [u32; PERF_STATES],
}

/// A private copy of the ADT.
struct Adt<'a> {
    data: &'a [u8],
}

fn le32(b: &[u8], off: usize) -> Option<u32> {
    Some(u32::from_le_bytes(b.get(off..off + 4)?.try_into().ok()?))
}

fn le64(b: &[u8], off: usize) -> Option<u64> {
    Some(u64::from_le_bytes(b.get(off..off + 8)?.try_into().ok()?))
}

/// `strncmp(a, b, n) == 0` for a byte field `a` and a string `b`, both treated as
/// NUL-terminated (bytes past the end of either count as NUL).
fn strncmp_eq(a: &[u8], b: &str, n: usize) -> bool {
    let b = b.as_bytes();
    for i in 0..n {
        let x = a.get(i).copied().unwrap_or(0);
        let y = b.get(i).copied().unwrap_or(0);
        if x != y {
            return false;
        }
        if x == 0 {
            return true;
        }
    }
    true
}

impl Adt<'_> {
    /// Returns the end of the node at `off`, or `None` on a malformed node.
    fn node_end(&self, mut off: usize, depth: u32) -> Option<usize> {
        let size = self.data.len();
        if depth > 32 || off > size || size - off < 8 {
            return None;
        }
        let props = le32(self.data, off)?;
        let children = le32(self.data, off + 4)?;
        if props > 4096 || children > 1024 {
            return None;
        }
        off += 8;
        for _ in 0..props {
            if size - off < 36 {
                return None;
            }
            let len = (le32(self.data, off + 32)? & 0x7fffffff) as usize;
            off += 36;
            if len > size - off {
                return None;
            }
            off += len.next_multiple_of(4);
            if off > size {
                return None;
            }
        }
        for _ in 0..children {
            off = self.node_end(off, depth + 1)?;
        }
        Some(off)
    }

    /// The value of property `name` of `node`.
    fn prop(&self, node: usize, name: &str) -> Option<&[u8]> {
        let props = le32(self.data, node)?;
        let mut off = node + 8;
        for _ in 0..props {
            let len = (le32(self.data, off + 32)? & 0x7fffffff) as usize;
            if strncmp_eq(self.data.get(off..off + 32)?, name, 32) {
                return self.data.get(off + 36..off + 36 + len);
            }
            off += 36 + len.next_multiple_of(4);
        }
        None
    }

    /// The child named `name` of `node`.
    fn child(&self, node: usize, name: &str) -> Option<usize> {
        let props = le32(self.data, node)?;
        let children = le32(self.data, node + 4)?;
        let mut off = node + 8;
        for _ in 0..props {
            off += 36 + ((le32(self.data, off + 32)? & 0x7fffffff) as usize).next_multiple_of(4);
        }
        for _ in 0..children {
            if let Some(n) = self.prop(off, "name") {
                if !n.is_empty() && strncmp_eq(n, name, n.len()) {
                    return Some(off);
                }
            }
            off = self.node_end(off, 1)?;
        }
        None
    }

    fn u64(&self, node: usize, name: &str) -> Option<u64> {
        let p = self.prop(node, name)?;
        if p.len() != 8 {
            return None;
        }
        le64(p, 0)
    }

    fn u32(&self, node: usize, name: &str) -> Option<u32> {
        let p = self.prop(node, name)?;
        if p.len() != 4 {
            return None;
        }
        le32(p, 0)
    }
}

/// This machine's GPU performance states (per-chip binning): ADT sgx `perf-states` and
/// `perf-states-sram`, (frequency Hz, millivolt) pairs. The board-image layout implements one
/// table shape; another shape is refused. Values the board-image layout takes as given are
/// compared, not trusted.
fn collect_perf(dev: &device::Device, adt: &Adt<'_>, sgx: usize, out: &mut GpuAdt) -> Result {
    const FIXED: [(&str, u32); 5] = [
        ("perf-state-count", PERF_STATES as u32),
        ("gpu-num-perf-states", PERF_STATES as u32 - 1),
        ("perf-state-table-count", 1),
        ("gpu-power-sample-period", 8),
        ("gpu-pwr-sample-period-aic-clks", 200000),
    ];
    for (name, value) in FIXED {
        if adt.u32(sgx, name) != Some(value) {
            dev_err!(dev, "ADT sgx {} is not the supported {}\n", name, value);
            return Err(EINVAL);
        }
    }
    let (Some(core), Some(sram)) = (
        adt.prop(sgx, "perf-states"),
        adt.prop(sgx, "perf-states-sram"),
    ) else {
        dev_err!(dev, "ADT perf-state tables missing or of another shape\n");
        return Err(EINVAL);
    };
    if core.len() != 8 * PERF_STATES || sram.len() != core.len() {
        dev_err!(dev, "ADT perf-state tables missing or of another shape\n");
        return Err(EINVAL);
    }
    for i in 0..PERF_STATES {
        let hz = le32(core, 8 * i).ok_or(EINVAL)?;
        if le32(sram, 8 * i) != Some(hz)
            || hz % 1000000 != 0
            || hz > 3000000000
            || ((i == 0) != (hz == 0))
        {
            dev_err!(dev, "ADT perf-state {} ({} Hz) not usable\n", i, hz);
            return Err(EINVAL);
        }
        out.perf_mhz[i] = hz / 1000000;
        out.perf_mv[i] = le32(core, 8 * i + 4).ok_or(EINVAL)?;
        out.sram_mv[i] = le32(sram, 8 * i + 4).ok_or(EINVAL)?;
        if !(100..=1100).contains(&out.perf_mv[i]) || !(100..=1100).contains(&out.sram_mv[i]) {
            dev_err!(
                dev,
                "ADT perf-state {} voltage {}/{} mV out of range\n",
                i,
                out.perf_mv[i],
                out.sram_mv[i]
            );
            return Err(EINVAL);
        }
    }
    Ok(())
}

/// Region names of the admission messages.
const REGION_NAMES: [&str; 6] = ["ttbs", "tables", "handoff", "shared-l2", "text", "data"];

/// Parses the private ADT copy.
fn parse(dev: &device::Device, adt: &Adt<'_>) -> Result<GpuAdt> {
    const PROPS: [(&str, &str); 4] = [
        ("gpu-region-base", "gpu-region-size"),
        ("gfx-shared-region-base", "gfx-shared-region-size"),
        ("gfx-handoff-base", "gfx-handoff-size"),
        ("gfx-shared-l2-region-base", "gfx-shared-l2-region-size"),
    ];
    if adt.node_end(0, 0).is_none() {
        return Err(EINVAL);
    }
    let armio = adt.child(0, "arm-io").ok_or(EINVAL)?;
    let sgx = adt.child(armio, "sgx").ok_or(EINVAL)?;
    let asc = adt.child(armio, "gfx-asc").ok_or(EINVAL)?;
    let nub = adt.child(asc, "iop-gfx-nub").ok_or(EINVAL)?;
    let compatible = adt.prop(sgx, "compatible").ok_or(EINVAL)?;
    let nul = compatible.iter().position(|&b| b == 0).ok_or(EINVAL)?;
    if &compatible[..nul] != b"gpu,t6030" {
        return Err(EINVAL);
    }
    let mut region: [Range<u64>; 6] = Default::default();
    for (i, (base_key, size_key)) in PROPS.iter().enumerate() {
        let base = adt.u64(sgx, base_key).ok_or(EINVAL)?;
        let size = adt.u64(sgx, size_key).ok_or(EINVAL)?;
        region[i] = base..base.checked_add(size).ok_or(EINVAL)?;
    }
    let ranges = adt.prop(asc, "segment-ranges").ok_or(EINVAL)?;
    let nub_ranges = adt.prop(nub, "segment-ranges").ok_or(EINVAL)?;
    let names = adt.prop(asc, "segment-names").ok_or(EINVAL)?;
    // Both range tables must be exactly 2 x 32 bytes and identical.
    if ranges.len() != 64 || nub_ranges.len() != 64 || ranges != nub_ranges {
        return Err(EINVAL);
    }
    if !strncmp_eq(names, "__TEXT;__DATA", names.len()) {
        return Err(EINVAL);
    }
    let preloaded = adt.prop(nub, "pre-loaded").ok_or(EINVAL)?;
    if preloaded.len() != 4 || le32(preloaded, 0) != Some(1) {
        return Err(EINVAL);
    }
    let mut va = [0u64; 2];
    for i in 0..2 {
        let r = &ranges[32 * i..32 * i + 32];
        let pa = le64(r, 0).ok_or(EINVAL)?;
        va[i] = le64(r, 8).ok_or(EINVAL)?;
        if le64(r, 0x10) != Some(pa) || le32(r, 0x1c) != Some(if i != 0 { 0 } else { 1 }) {
            return Err(EINVAL);
        }
        let size = le32(r, 0x18).ok_or(EINVAL)? as u64;
        region[4 + i] = pa..pa.checked_add(size).ok_or(EINVAL)?;
    }
    // Firmware ABI: TEXT at the base of the upper half, DATA right after it.
    let text_size = region[4].end - region[4].start;
    if va[0] != hw::UPPER || va[1] != va[0] + text_size {
        return Err(EINVAL);
    }
    let chosen = adt.child(0, "chosen");
    let dram = chosen
        .and_then(|chosen| Some((adt.u64(chosen, "dram-base")?, adt.u64(chosen, "dram-size")?)));
    let Some((dram_base, dram_size)) =
        dram.filter(|&(base, size)| size != 0 && base.checked_add(size).is_some())
    else {
        dev_err!(dev, "ADT DRAM bounds unreadable\n");
        return Err(EINVAL);
    };
    // Structural admission: the ADT says where; this checks it is sane.
    // The kernel device tree of this tree also describes the T6030 UAT regions as
    // reserved-memory nodes the boot loader fills; whether it marks them no-map is not fixed
    // here, so a region the kernel maps as System RAM is admitted on this platform too.
    admit_regions(dev, &region, Some(dram_base..dram_base + dram_size), true)?;
    let [ttbs, tables, handoff, shared_l2, text, data] = region;
    let mut out = GpuAdt {
        ttbs,
        tables,
        handoff,
        shared_l2,
        text,
        data,
        perf_mhz: [0; PERF_STATES],
        perf_mv: [0; PERF_STATES],
        sram_mv: [0; PERF_STATES],
    };
    collect_perf(dev, adt, sgx, &mut out)?;
    Ok(out)
}

/// Structural admission of the six regions (same order as [`REGION_NAMES`]): page-aligned,
/// addressable, the fixed UAT sizes, disjoint from each other, and either outside System RAM
/// (the T6030 carveouts) or, with `reserved_ram` and for the four UAT regions only, lying
/// entirely in System RAM as reserved memory the GPU node names (the T6031 device tree, and the
/// T6030 nodes of this tree's device tree). The firmware segments are always outside System RAM.
fn admit_regions(
    dev: &device::Device,
    region: &[Range<u64>; 6],
    dram: Option<Range<u64>>,
    reserved_ram: bool,
) -> Result {
    let fixed = [
        hw::TTBS_SIZE,
        hw::TABLES_SIZE,
        hw::HANDOFF_SIZE,
        hw::SHARED_L2_SIZE,
    ];
    for i in 0..region.len() {
        let r = &region[i];
        let size = r.end - r.start;
        let end = r.end.wrapping_sub(1);
        let outside_dram = match &dram {
            Some(dram) => r.start < dram.start || end > dram.end - 1,
            None => false,
        };
        let placed = if reserved_ram && i < 4 {
            glue::ram_disjoint(r.start, size) || glue::ram_within(r.start, size)
        } else {
            glue::ram_disjoint(r.start, size)
        };
        if size == 0
            || (r.start | size) & (hw::PAGE - 1) != 0
            || end > (hw::PA_MASK | (hw::PAGE - 1))
            || outside_dram
            || (i < 4 && size != fixed[i])
            || !placed
        {
            dev_err!(
                dev,
                "GPU region {} ({}) [mem {:#x}-{:#x}] not admissible\n",
                i,
                REGION_NAMES[i],
                r.start,
                end
            );
            return Err(EINVAL);
        }
        for j in 0..i {
            if r.start < region[j].end && region[j].start < r.end {
                dev_err!(dev, "GPU regions {} and {} overlap\n", j, i);
                return Err(EINVAL);
            }
        }
    }
    Ok(())
}

/// The GPU node's regions, segments and operating points (T6031).
fn from_device_tree(dev: &device::Device) -> Result<GpuAdt> {
    const NAMES: [&CStr; 6] = [
        c_str!("ttbs"),
        c_str!("pagetables"),
        c_str!("handoff"),
        c_str!("pagetables-l2"),
        c_str!("fw-text"),
        c_str!("fw-data"),
    ];
    let node = dev.of_node().ok_or(ENODEV)?;
    let mut region: [Range<u64>; 6] = Default::default();
    for (i, name) in NAMES.iter().enumerate() {
        let r = node
            .reserved_mem_region_to_resource_byname(name)
            .inspect_err(|_| dev_err!(dev, "GPU node has no {} memory region\n", REGION_NAMES[i]))?;
        let start = r.start() as u64;
        let size = r.size() as u64;
        region[i] = start..start.checked_add(size).ok_or(EINVAL)?;
    }
    // Firmware ABI: TEXT at the base of the upper half, DATA right after it.
    let vas: KVec<u64> = node
        .get_property(c_str!("apple,firmware-segment-vas"))
        .inspect_err(|_| dev_err!(dev, "GPU node has no apple,firmware-segment-vas\n"))?;
    let text_size = region[4].end - region[4].start;
    if vas.len() != 2 || vas[0] != hw::UPPER || vas[1] != vas[0] + text_size {
        dev_err!(dev, "GPU firmware segment VAs are not the firmware ABI's\n");
        return Err(EINVAL);
    }
    admit_regions(dev, &region, None, true)?;
    let [ttbs, tables, handoff, shared_l2, text, data] = region;
    let mut out = GpuAdt {
        ttbs,
        tables,
        handoff,
        shared_l2,
        text,
        data,
        perf_mhz: [0; PERF_STATES],
        perf_mv: [0; PERF_STATES],
        sram_mv: [0; PERF_STATES],
    };
    // Operating points: MHz and the highest cluster voltage of each; the SRAM voltage is not in
    // the device tree. The firmware runs the board images' tables; these are informational.
    let opps = node
        .parse_phandle(c_str!("operating-points-v2"), 0)
        .ok_or(EINVAL)
        .inspect_err(|_| dev_err!(dev, "GPU node has no operating points\n"))?;
    let mut count = 0;
    for opp in opps.children() {
        let hz: u64 = opp.get_property(c_str!("opp-hz"))?;
        let uv: KVec<u32> = opp.get_property(c_str!("opp-microvolt"))?;
        let mv = uv.iter().max().copied().unwrap_or(0) / 1000;
        if count < PERF_STATES {
            out.perf_mhz[count] = (hz / 1_000_000) as u32;
            out.perf_mv[count] = mv;
            out.sram_mv[count] = mv;
        }
        count += 1;
    }
    if count != PERF_STATES {
        dev_err!(
            dev,
            "GPU node has {} operating points, not {}\n",
            count,
            PERF_STATES
        );
        return Err(EINVAL);
    }
    Ok(out)
}

/// Reads and admits the GPU's regions, segments and performance states (read-only), from the
/// source the SoC configuration names.
pub(crate) fn read(dev: &device::Device, cfg: &super::Config) -> Result<GpuAdt> {
    if cfg.platform == hw::Platform::DeviceTree {
        return from_device_tree(dev);
    }
    let (start, size) = glue::adt_phram()?;
    if size == 0 || size > 0x200000 || start & 0x3fff != 0 {
        return Err(EINVAL);
    }
    let copy = super::identity::copy_physical(start, usize::try_from(size)?)?;
    parse(dev, &Adt { data: &copy })
}
