// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Board facts for the M3 runtime, read from the hardware and the device tree instead of being
//! fixed to one board. The per-SoC facts are in [`crate::m3_soc`].

use kernel::{bindings, c_str, device, device::Core, io::resource::Resource, of, platform, prelude::*, uapi};

use crate::m3_resources::{
    reserved_resource,
    Region,
    Resources, //
};
use crate::m3_soc::Soc;

/// Whether the root node's compatible list contains `compatible`.
fn board_is(compatible: &[u8]) -> bool {
    let Some(root) = kernel::of::root() else {
        return false;
    };
    let Ok(board) = root.get_property::<KVec<u8>>(c_str!("compatible")) else {
        return false;
    };
    board.split(|b| *b == 0).any(|s| s == compatible)
}

/// Whether this is a board the M3 runtime is validated on (`soc.validated_boards`).
/// `asahi.m3_expose=auto` registers the render node and the probe starts the runtime only on
/// these.
pub(crate) fn runtime_validated_board(soc: &Soc) -> bool {
    soc.validated_boards.iter().any(|board| board_is(board))
}

/// The number of core slots of `soc` (1 x 10 on T8122).
pub(crate) fn core_slots(soc: &Soc) -> u32 {
    soc.cores_per_cluster * soc.clusters
}

/// Whether the core-enable mask describes a usable GPU of `soc`: at least one core, and no core
/// outside its core slots. The mask is the fused core-enable words from SGX+0xe01500, the second
/// word as bits 32..63 when the SoC has more than 32 core slots. It varies with the SKU; absent
/// cores must never be enabled.
pub(crate) fn core_mask_valid(soc: &Soc, mask: u64) -> bool {
    let slots = core_slots(soc);
    mask != 0 && slots <= 64 && (slots == 64 || mask >> slots == 0)
}

/// Per-cluster UAPI core masks for a core-enable mask of `soc`: consecutive fields of
/// `cores_per_cluster` bits, cluster 0 lowest, across the word boundary (the packing the
/// identification code uses for every multi-word mask, `regs.rs`).
pub(crate) fn core_masks(soc: &Soc, mask: u64) -> [u32; uapi::DRM_ASAHI_MAX_CLUSTERS as usize] {
    let mut masks = [0; uapi::DRM_ASAHI_MAX_CLUSTERS as usize];
    let width = soc.cores_per_cluster.min(32);
    let field = (1u64 << width) - 1;
    for (cluster, slot) in masks.iter_mut().enumerate().take(soc.clusters as usize) {
        let shift = cluster as u32 * soc.cores_per_cluster;
        *slot = mask.checked_shr(shift).map_or(0, |m| (m & field) as u32);
    }
    masks
}

/// The highest GPU frequency in the device tree OPP table, in kHz, or None when the GPU node
/// has no populated OPP table.
pub(crate) fn max_frequency_khz(pdev: &platform::Device<Core>) -> Option<u32> {
    let node = pdev.as_ref().of_node()?;
    let opps = node.parse_phandle(c_str!("operating-points-v2"), 0)?;
    let mut max_hz: u64 = 0;
    for opp in opps.children() {
        if let Ok(hz) = opp.get_property::<u64>(c_str!("opp-hz")) {
            max_hz = max_hz.max(hz);
        }
    }
    u32::try_from(max_hz / 1000).ok().filter(|khz| *khz != 0)
}

/// The shape of the performance-state table the device tree's operating points give: states
/// above the off state and the highest frequency in MHz (`t8122_admission::opp_table_shape`).
/// Every cluster of a point must have the same voltage, except on a SoC with per-cluster
/// voltages, where the highest `opp-microvolt` cell orders the point. None without a readable
/// OPP table.
pub(crate) fn opp_table_shape(pdev: &platform::Device<Core>, soc: &Soc) -> Option<(u32, u32)> {
    let node = pdev.as_ref().of_node()?;
    let opps = node.parse_phandle(c_str!("operating-points-v2"), 0)?;
    let mut points = KVec::new();
    for opp in opps.children() {
        let hz = opp.get_property::<u64>(c_str!("opp-hz")).ok()?;
        let microvolt = opp.get_property::<KVec<u32>>(c_str!("opp-microvolt")).ok()?;
        let mv = microvolt.first()?.div_ceil(1000);
        if microvolt.len() != soc.clusters as usize
            || (!soc.per_cluster_voltages && microvolt.iter().any(|uv| uv.div_ceil(1000) != mv))
        {
            return None;
        }
        // With per-cluster voltages a point is ordered by its highest cluster voltage, as the
        // published tables are (`initdata::PStateTables`).
        let uv = if soc.per_cluster_voltages {
            *microvolt.iter().max()?
        } else {
            *microvolt.first()?
        };
        points.push((hz, uv), GFP_KERNEL).ok()?;
    }
    crate::t8122_admission::opp_table_shape(points)
}

/// Register windows the runtime maps on `soc`: name, base, minimum size. The runtime uses the
/// first 16 KiB of `asc` (`m3_device`); the first 16 MiB of `sgx` cover the ID block at
/// +0xd04000, the Fender MMU block at +0xd08000 and the core masks at +0xe01500.
fn reg_windows(soc: &Soc) -> [(&'static CStr, u64, u64); 2] {
    [
        (c_str!("asc"), soc.windows.asc, 0x4000),
        (c_str!("sgx"), soc.windows.sgx, 0x100_0000),
    ]
}

/// The `reg` cells (two address and two size cells) of the mailbox of `soc`: 16 KiB at its base.
fn expected_mailbox_reg(soc: &Soc) -> [u32; 4] {
    [(soc.windows.mailbox >> 32) as u32, soc.windows.mailbox as u32, 0, 0x4000]
}

/// Reserved regions of the firmware handoff, in `Resources::regions` order.
const REGIONS: [&CStr; 6] = [
    c_str!("ttbs"),
    c_str!("pagetables"),
    c_str!("handoff"),
    c_str!("shared-l2"),
    c_str!("fw-text"),
    c_str!("fw-data"),
];

/// Static reserved-memory nodes that describe a handoff region when the GPU node does not list
/// it under its own name: the bootloader fills these nodes (the UAT regions of the firmware's
/// page-table handoff) whether or not the GPU node references them. The second-level table of
/// the firmware's upper page-table tree ("shared-l2") is reserved this way without being in the
/// GPU node's memory-region list.
const REGION_NODES: [(&CStr, &CStr); 4] = [
    (c_str!("ttbs"), c_str!("/reserved-memory/uat-ttbs")),
    (c_str!("pagetables"), c_str!("/reserved-memory/uat-pagetables")),
    (c_str!("handoff"), c_str!("/reserved-memory/uat-handoff")),
    (c_str!("shared-l2"), c_str!("/reserved-memory/uat-pagetables-l2")),
];

/// Properties that describe the loaded firmware's segments on the GPU node.
const SEGMENT_PROPS: [&CStr; 3] = [
    c_str!("apple,m3-handoff-version"),
    c_str!("apple,firmware-segment-vas"),
    c_str!("apple,firmware-segment-flags"),
];

/// Power-management properties of a GPU node whose SoC takes them from the boot loader
/// (`Soc::power_from_boot_loader`), one 32-bit cell each: the ones the T6030 device tree carries
/// as static values, and the shader-engine target. The boot loader copies each from the ADT's GPU
/// node (`apple,X` from `gpu-X`; `apple,se-target` from `gpu-se-tgt`).
///
/// `apple,idleoff-standby-timer` is not among them: the J613 ADT has no
/// `gpu-idleoff-standby-timer`, so the boot loader adds it only when the ADT has one, and the
/// HwConfig's default applies otherwise.
const BOOT_LOADER_POWER: [&CStr; 40] = [
    c_str!("apple,perf-base-pstate"),
    c_str!("apple,min-sram-microvolt"),
    c_str!("apple,avg-power-filter-tc-ms"),
    c_str!("apple,avg-power-ki-only"),
    c_str!("apple,avg-power-kp"),
    c_str!("apple,avg-power-min-duty-cycle"),
    c_str!("apple,avg-power-target-filter-tc"),
    c_str!("apple,fast-die0-integral-gain"),
    c_str!("apple,fast-die0-proportional-gain"),
    c_str!("apple,perf-boost-ce-step"),
    c_str!("apple,perf-boost-min-util"),
    c_str!("apple,perf-filter-drop-threshold"),
    c_str!("apple,perf-filter-time-constant"),
    c_str!("apple,perf-filter-time-constant2"),
    c_str!("apple,perf-integral-gain"),
    c_str!("apple,perf-integral-gain2"),
    c_str!("apple,perf-integral-min-clamp"),
    c_str!("apple,perf-proportional-gain"),
    c_str!("apple,perf-proportional-gain2"),
    c_str!("apple,perf-tgt-utilization"),
    c_str!("apple,power-sample-period"),
    c_str!("apple,ppm-filter-time-constant-ms"),
    c_str!("apple,ppm-ki"),
    c_str!("apple,ppm-kp"),
    c_str!("apple,pwr-filter-time-constant"),
    c_str!("apple,pwr-integral-gain"),
    c_str!("apple,pwr-integral-min-clamp"),
    c_str!("apple,pwr-min-duty-cycle"),
    c_str!("apple,pwr-proportional-gain"),
    c_str!("apple,pwr-sample-period-aic-clks"),
    c_str!("apple,se-engagement-criteria"),
    c_str!("apple,se-filter-time-constant"),
    c_str!("apple,se-filter-time-constant-1"),
    c_str!("apple,se-inactive-threshold"),
    c_str!("apple,se-ki"),
    c_str!("apple,se-ki-1"),
    c_str!("apple,se-kp"),
    c_str!("apple,se-kp-1"),
    c_str!("apple,se-reset-criteria"),
    c_str!("apple,se-target"),
];

/// Check that the boot loader gave a GPU node of `soc` every per-machine property the runtime
/// would otherwise take from a static device tree or a driver default: [`BOOT_LOADER_POWER`], the
/// 64-bit `apple,fast-die0-sensor-mask`, one core and one SRAM leakage coefficient per cluster,
/// the firmware version (three cells, not all zero), and an operating-point table of at least two
/// entries, each with one voltage per cluster and a power, and a frequency after the first. Logs
/// every missing or malformed property.
fn check_boot_loader_power(dev: &device::Device, node: &of::Node, soc: &Soc) -> Result {
    let mut bad = 0u32;
    let mut expect = |name: &CStr, want: usize, nonzero: bool| {
        match node.get_property::<KVec<u8>>(name) {
            Ok(value) if value.len() == want && (!nonzero || value.iter().any(|b| *b != 0)) => {}
            Ok(value) => {
                dev_info!(
                    dev,
                    "M3: not admitted: {:?} is {} bytes, {} expected{}\n",
                    name,
                    value.len(),
                    want,
                    if nonzero { ", not all zero" } else { "" }
                );
                bad += 1;
            }
            Err(_) => {
                dev_info!(dev, "M3: not admitted: the boot loader did not add {:?}\n", name);
                bad += 1;
            }
        }
    };
    for name in BOOT_LOADER_POWER {
        expect(name, 4, false);
    }
    expect(c_str!("apple,fast-die0-sensor-mask"), 8, false);
    expect(c_str!("apple,core-leak-coef"), 4 * soc.clusters as usize, false);
    expect(c_str!("apple,sram-leak-coef"), 4 * soc.clusters as usize, false);
    expect(c_str!("apple,firmware-version"), 12, true);

    let mut opps = 0u32;
    let mut opps_ok = true;
    match node.parse_phandle(c_str!("operating-points-v2"), 0) {
        Some(table) => {
            for opp in table.children() {
                let hz = opp.get_property::<u64>(c_str!("opp-hz")).unwrap_or(0);
                let volts = opp.get_property::<KVec<u32>>(c_str!("opp-microvolt")).map_or(0, |v| v.len());
                let power = opp.get_property::<u32>(c_str!("opp-microwatt")).is_ok();
                opps_ok &= volts == soc.clusters as usize && power && (opps == 0 || hz != 0);
                opps += 1;
            }
        }
        None => opps_ok = false,
    }
    if !opps_ok || opps < 2 {
        dev_info!(
            dev,
            "M3: not admitted: the operating points are not the boot loader's ({} entries; each needs opp-hz, {} opp-microvolt cells and opp-microwatt)\n",
            opps,
            soc.clusters
        );
        bad += 1;
    }
    if bad != 0 {
        return Err(EINVAL);
    }
    dev_info!(dev, "M3: boot loader power configuration accepted ({} operating points)\n", opps);
    Ok(())
}

/// Whether the GPU node lists the memory region `name` in memory-region-names.
fn lists_region(node: &of::Node, name: &CStr) -> bool {
    node.get_property::<KVec<u8>>(c_str!("memory-region-names"))
        .is_ok_and(|names| names.split(|b| *b == 0).any(|n| n == name.to_bytes()))
}

/// The static reserved-memory node that stands in for the handoff region `name`, if the GPU
/// node does not list the region itself and the device tree has such a node.
fn region_node_path(node: &of::Node, name: &CStr) -> Option<&'static CStr> {
    if lists_region(node, name) {
        return None;
    }
    REGION_NODES
        .iter()
        .find(|(region, _)| region.to_bytes() == name.to_bytes())
        .map(|(_, path)| *path)
}

/// A reference to a device tree node found by path, released on drop.
struct NodeRef(*mut bindings::device_node);

impl NodeRef {
    fn find(path: &CStr) -> Option<Self> {
        // SAFETY: `path` is a NUL-terminated string; the lookup takes a node reference or
        // returns NULL.
        let np = unsafe { bindings::of_find_node_opts_by_path(path.as_char_ptr(), core::ptr::null_mut()) };
        (!np.is_null()).then_some(NodeRef(np))
    }
}

impl Drop for NodeRef {
    fn drop(&mut self) {
        // SAFETY: the reference was taken by `find`.
        unsafe { bindings::of_node_put(self.0) };
    }
}

/// Look up the handoff region `name` through its static reserved-memory node (see
/// [`REGION_NODES`]). Returns None when the GPU node lists the region itself or the device tree
/// has no such node; otherwise the region, which must be an enabled reserved-memory node the
/// kernel reserved at boot. Returns whether the node is no-map as well.
pub(crate) fn static_region(node: &of::Node, name: &CStr) -> Option<Result<(Resource, bool)>> {
    let path = region_node_path(node, name)?;
    let np = NodeRef::find(path)?;
    // SAFETY: `np` holds a node reference for the duration of these read-only queries.
    let (available, rmem, nomap) = unsafe {
        (
            bindings::of_device_is_available(np.0),
            bindings::of_reserved_mem_lookup(np.0),
            bindings::of_property_read_bool(np.0, c_str!("no-map").as_char_ptr()),
        )
    };
    if !available || rmem.is_null() {
        return Some(Err(EINVAL));
    }
    // SAFETY: the reserved-memory table entry lives as long as the kernel.
    let (base, size) = unsafe { ((*rmem).base, (*rmem).size) };
    if base == 0 || size == 0 || base.checked_add(size).is_none_or(|end| end > 1 << 42) {
        return Some(Err(EINVAL));
    }
    let raw = bindings::resource {
        start: base,
        end: base + size - 1,
        flags: bindings::IORESOURCE_MEM as _,
        ..Default::default()
    };
    // SAFETY: Resource is repr(transparent) over Opaque<resource>. The descriptor is owned and
    // has no pointers into temporary data; its range is checked above.
    Some(Ok((unsafe { core::mem::transmute::<bindings::resource, Resource>(raw) }, nomap)))
}

/// Admit a GPU of `soc` described by the device tree, either statically or by a runtime overlay.
///
/// The device must be a GPU node of `soc` with the ASC and SGX windows at their addresses
/// (at least as large as the runtime maps), the GPU coprocessor mailbox, the six handoff regions
/// by name (from the reserved-memory registry or no-map overlay nodes), and the firmware segment
/// description. Every refusal is logged.
pub(crate) fn admit(pdev: &platform::Device<Core>, soc: &Soc) -> Result<Resources> {
    let dev = pdev.as_ref();
    let refuse = |what: &str, err: Error| {
        dev_info!(dev, "M3: not admitted: {}\n", what);
        err
    };
    let node = dev.of_node().ok_or(ENODEV)?;

    if !board_is(soc.board.as_bytes()) {
        dev_info!(dev, "M3: not admitted: not a {} board\n", soc.name);
        return Err(ENODEV);
    }
    let compatible: KVec<u8> = node.get_property(c_str!("compatible"))?;
    if !compatible.split(|b| *b == 0).any(|s| s == soc.gpu.as_bytes()) {
        dev_info!(dev, "M3: not admitted: GPU node is not {}\n", soc.gpu);
        return Err(ENODEV);
    }
    // The runtime runs the firmware that the bootloader loaded, in place: without the
    // bootloader's description of its segments there is nothing to admit.
    if !lists_region(&node, c_str!("fw-text"))
        || !lists_region(&node, c_str!("fw-data"))
        || SEGMENT_PROPS.iter().any(|p| node.get_property::<KVec<u8>>(p).is_err())
    {
        return Err(refuse(
            "the device tree does not describe the loaded GPU firmware segments (fw-text/fw-data regions and apple,firmware-segment-* properties); the bootloader predates this",
            ENODEV,
        ));
    }
    dev_info!(dev, "M3: board compatible accepted\n");
    if soc.power_from_boot_loader {
        check_boot_loader_power(dev, &node, soc)?;
    }

    for name in REGIONS {
        if lists_region(&node, name) {
            continue;
        }
        match region_node_path(&node, name) {
            Some(path) if NodeRef::find(path).is_some() => {
                dev_info!(dev, "M3: {:?} region: static node {:?}\n", name, path)
            }
            _ => {
                dev_info!(dev, "M3: not admitted: no {:?} memory region\n", name);
                return Err(EINVAL);
            }
        }
    }
    dev_info!(dev, "M3: memory region list accepted\n");

    for (name, base, min_size) in reg_windows(soc) {
        let res = pdev
            .resource_by_name(name)
            .ok_or_else(|| refuse("missing register window", EINVAL))?;
        if res.start() != base || res.size() < min_size {
            dev_info!(
                dev,
                "M3: not admitted: {:?} window {:#x}+{:#x}, need {:#x}+{:#x} or larger\n",
                name,
                res.start(),
                res.size(),
                base,
                min_size
            );
            return Err(EINVAL);
        }
    }
    dev_info!(dev, "M3: register resources accepted\n");

    let mboxes: KVec<u32> = node.get_property(c_str!("mboxes"))?;
    let mbox = (mboxes.len() == 1)
        .then(|| node.parse_phandle(c_str!("mboxes"), 0))
        .flatten()
        .ok_or_else(|| refuse("expected exactly one mailbox", EINVAL))?;
    let compat: KVec<u8> = mbox.get_property(c_str!("compatible"))?;
    let mbox_reg: KVec<u32> = mbox.get_property(c_str!("reg"))?;
    let irq_names: KVec<u8> = mbox.get_property(c_str!("interrupt-names"))?;
    let irqs: KVec<u32> = mbox.get_property(c_str!("interrupts"))?;
    let cells: u32 = mbox.get_property(c_str!("#mbox-cells"))?;
    if !soc.mailbox_compatibles.iter().any(|c| *c == compat.as_slice())
        || mbox_reg.as_slice() != expected_mailbox_reg(soc)
        || cells != 0
        || irq_names.as_slice() != b"send-empty\0send-not-empty\0recv-empty\0recv-not-empty\0"
        || irqs.as_slice() != soc.mailbox_interrupts
    {
        return Err(refuse("unexpected GPU mailbox description", EINVAL));
    }
    dev_info!(dev, "M3: mailbox resource accepted\n");

    let mut regions = [Region { base: 0, size: 0 }; 6];
    for (i, name) in REGIONS.iter().enumerate() {
        let res = reserved_resource(&node, name).map_err(|e| {
            dev_info!(dev, "M3: not admitted: {:?} region unusable ({:?})\n", name, e);
            e
        })?;
        regions[i] = Region {
            base: res.start(),
            size: res.size(),
        };
    }
    dev_info!(dev, "M3: reserved resources {:?}\n", regions);

    let version: u32 = node
        .get_property(c_str!("apple,m3-handoff-version"))
        .map_err(|e| refuse("no firmware handoff description", e))?;
    let vas: KVec<u64> = node.get_property(c_str!("apple,firmware-segment-vas"))?;
    let flags: KVec<u32> = node.get_property(c_str!("apple,firmware-segment-flags"))?;
    Resources::validate(
        version,
        regions,
        vas.as_slice().try_into().map_err(|_| EINVAL)?,
        flags.as_slice().try_into().map_err(|_| EINVAL)?,
    )
    .map_err(|e| {
        dev_info!(dev, "M3: not admitted: handoff description rejected ({:?})\n", e);
        EINVAL
    })
}

/// Whether the GPU node links to a PMP instance (`apple,pmp`) through which the GPU power vote
/// is cast. Without one, GPU power is left to the power domain.
pub(crate) fn has_pmp_link(pdev: &platform::Device<Core>) -> bool {
    pdev.as_ref()
        .of_node()
        .is_some_and(|node| node.get_property::<u32>(c_str!("apple,pmp")).is_ok())
}

pub(crate) use crate::m3_firmware::{KnownImage, KNOWN_IMAGES_T8122};
use crate::m3_firmware::image_info;

fn sha256(data: &[u8]) -> [u8; 32] {
    let mut digest = [0u8; 32];
    // SAFETY: `data` and `digest` are valid for the duration of the synchronous call.
    unsafe { bindings::sha256(data.as_ptr(), data.len(), digest.as_mut_ptr()) };
    digest
}

/// Lower-case hex of `bytes`, for logs.
fn hex<const N: usize>(bytes: &[u8]) -> [u8; N] {
    const DIGITS: &[u8; 16] = b"0123456789abcdef";
    let mut out = [b'0'; N];
    for (i, b) in bytes.iter().take(N / 2).enumerate() {
        out[2 * i] = DIGITS[(b >> 4) as usize];
        out[2 * i + 1] = DIGITS[(b & 0xf) as usize];
    }
    out
}

pub(crate) fn identify(
    dev: &device::Device,
    text: &[u8],
    stkg_sha256: &[u8; 32],
    images: &'static [KnownImage],
) -> Option<&'static KnownImage> {
    let info = image_info(text);
    let masked_sha256 = info.as_ref().and_then(|info| {
        let mut copy = KVec::new();
        copy.extend_from_slice(text, GFP_KERNEL).ok()?;
        copy.get_mut(info.patchbay.clone())?.fill(0);
        Some(sha256(&copy))
    });
    let uuid = info.as_ref().map(|info| info.uuid);
    let uuid_hex = hex::<32>(&uuid.unwrap_or([0; 16]));
    let stkg_hex = hex::<64>(stkg_sha256);
    let masked_hex = hex::<64>(&masked_sha256.unwrap_or([0; 32]));
    fn as_str(b: &[u8]) -> &str {
        core::str::from_utf8(b).unwrap_or("?")
    }
    dev_info!(
        dev,
        "M3: loaded GPU firmware: uuid {} (header {}), text sha256 {} (STKG zeroed), {} (patchbay zeroed)\n",
        if uuid.is_some() { as_str(&uuid_hex) } else { "none" },
        if info.as_ref().is_some_and(|i| i.text_size == text.len()) { "ok" } else { "missing or unexpected" },
        as_str(&stkg_hex),
        if masked_sha256.is_some() { as_str(&masked_hex) } else { "n/a" }
    );
    let image = images.iter().find(|image| image.accepts_runtime(uuid, stkg_sha256));
    match image {
        Some(image) => dev_info!(dev, "M3: GPU firmware identified as {}\n", image.name),
        None => dev_err!(dev, "M3: loaded GPU firmware is not a known image\n"),
    }
    image
}

/// Whether the reserved-memory node behind the GPU memory region `name` is no-map.
///
/// A region without no-map is part of the kernel's linear map, which maps it write-back; any
/// other CPU mapping of it must use the same memory type, because mismatched aliases of the same
/// memory are not allowed. Only no-map regions may be mapped write-combined. Returns true when
/// the region cannot be looked up, which keeps the caller's own choice.
pub(crate) fn region_is_nomap(node: &of::Node, name: &CStr) -> bool {
    if let Some(found) = static_region(node, name) {
        return found.map_or(true, |(_, nomap)| nomap);
    }
    let Ok(names) = node.get_property::<KVec<u8>>(c_str!("memory-region-names")) else {
        return true;
    };
    let Some(index) = names.split(|b| *b == 0).position(|n| n == name.to_bytes()) else {
        return true;
    };
    let Some(region) = node.parse_phandle(c_str!("memory-region"), index) else {
        return true;
    };
    region.get_property::<KVec<u8>>(c_str!("no-map")).is_ok()
}

/// Whether the M3 render node is registered (`asahi.m3_expose`).
///
/// Userspace drivers pick up any registered render node. Until the M3 runtime is validated on a
/// board, a render node there would let a stock userspace driver submit work the board may not
/// run correctly. `auto` (-1) registers it only on the boards the runtime was validated on.
pub(crate) fn expose_render_node(soc: &Soc) -> bool {
    match *crate::module_parameters::m3_expose.value() {
        0 => false,
        v if v > 0 => true,
        _ => runtime_validated_board(soc),
    }
}
