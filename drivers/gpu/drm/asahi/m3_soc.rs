// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! The per-SoC facts of the M3 runtime backend.
//!
//! Each table describes the register windows, memory layout and configuration of one GPU.
//! Missing configuration is `None`; outstanding runtime requirements are listed in `unported`.
//! Admission rejects either before reading firmware, mapping GPU registers or starting the
//! GPU coprocessor ([`Soc::require_complete`]).

use kernel::{c_str, device, prelude::*};

use crate::{
    hw,
    initdata::G15RuntimeHwDataB,
    m3_compute_layout::RegisterSet,
    m3_board::KnownImage,
    m3_firmware::Layout,
    m3_init_storage::IoMap, //
};

/// The SGX identification words the runtime admits, each compared under its mask.
pub(crate) struct IdWords {
    /// SGX+0xd04000: family [31:24], variant [23:16], revision [15:8].
    pub(crate) version: u32,
    pub(crate) version_mask: u32,
    /// SGX+0xd04010: dies [19:16], clusters per die [15:8].
    pub(crate) counts: u32,
    pub(crate) counts_mask: u32,
}

/// The register windows the runtime admits, as CPU physical addresses (device-tree `reg` values
/// translated through `arm-io` `ranges`).
pub(crate) struct Windows {
    /// Base of the GPU coprocessor window (`asc`, ADT `gfx-asc` reg[0]).
    pub(crate) asc: u64,
    /// Base of the GPU window (`sgx`, ADT `sgx` reg[0]).
    pub(crate) sgx: u64,
    /// Base of the GPU coprocessor mailbox, a 16 KiB window inside `asc`.
    pub(crate) mailbox: u64,
}

/// The performance-state table the runtime accepts: checked on the generated InitData before
/// any GPU register is touched, and again on the uploaded HwDataB.
#[derive(Copy, Clone, Debug)]
pub(crate) enum PstateTable {
    /// The table of the device tree's operating points: one state per distinct voltage above
    /// the off state, the highest at the highest frequency
    /// (`t8122_admission::opp_table_shape`).
    DeviceTree,
}

/// Optional userspace features advertised for a SoC (`DRM_ASAHI_GET_PARAMS`). The runtime
/// implements them the same way on every SoC; a SoC advertises one once it has been validated
/// there. Soft faults are never advertised: the runtime does not apply `asahi.fault_control`.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Features {
    /// `DRM_ASAHI_FEATURE_COMPUTE_WIDE_VISIBILITY`.
    pub(crate) compute_wide_visibility: bool,
    /// `DRM_ASAHI_FEATURE_FRAGMENT_DEPENDENCY`, while `asahi.m3_early_tiling` is set.
    pub(crate) fragment_dependency: bool,
}

/// The MTR temperature-sensor masks of the runtime's HwDataA, where the SoC table sets them
/// instead of the hardware configuration.
#[derive(Copy, Clone, Debug)]
pub(crate) struct MtrMasks {
    /// The fast-die sensor mask, in both HwDataA copies (+0x8ac and +0x1288).
    pub(crate) fast_die: u64,
    /// The mask the firmware matches an MTR alarm against (HwDataA +0x1a98). An alarm from a
    /// sensor outside it, or any alarm while it is 0, is fatal to the firmware.
    pub(crate) alarm: u64,
}

/// Where the boot loader exports a SoC's decoded GPU leakage fuse values, in `/chosen`, and the
/// boot loader switch that makes them this boot's leakage.
#[derive(Copy, Clone, Debug)]
pub(crate) struct LeakFuse {
    /// Two cells, value 1 then value 2, encoded as `apple,core-leak-coef` is.
    pub(crate) values: &'static CStr,
    /// The switch: the boot loader uses the values exactly when it holds the string "1" (the
    /// two bytes `1\0`), the form it copies from its configuration line.
    pub(crate) switch: &'static CStr,
}

/// Where the runtime places the HwData object (`m3_init_storage::HARDWARE_DATA`), when it is not
/// the fixed allocation's address, and how the firmware maps it.
#[derive(Copy, Clone, Debug)]
pub(crate) struct HwDataObject {
    /// Firmware VA of the object.
    pub(crate) address: u64,
    /// Map the object cacheable for the firmware (its atomic updates need cacheable memory)
    /// instead of uncached.
    pub(crate) cached: bool,
}

/// One firmware IO mapping of the runtime's InitData: HwDataB slot, physical address, total
/// size, element size, writable.
pub(crate) type IoMapping = (usize, u64, u32, u32, bool);

/// The M3 runtime's facts for one SoC.
pub(crate) struct Soc {
    /// SoC name, for logs.
    pub(crate) name: &'static str,
    /// GPU name, for logs.
    pub(crate) gpu_name: &'static str,
    pub(crate) chip_id: u32,
    pub(crate) gpu_variant: hw::GpuVariant,
    pub(crate) gpu_revision: hw::GpuRevision,
    pub(crate) gpu_revision_id: hw::GpuRevisionID,
    /// Root node compatible of the SoC.
    pub(crate) board: &'static str,
    /// GPU node compatible.
    pub(crate) gpu: &'static str,
    /// Boards the runtime starts on and registers a render node on. On a T8122 it starts on no
    /// other board (`m3_params::t8122_backend`).
    pub(crate) validated_boards: &'static [&'static [u8]],
    /// GPU clusters, and core slots per cluster.
    pub(crate) clusters: u32,
    pub(crate) cores_per_cluster: u32,
    /// Accepted compatible lists of the GPU coprocessor mailbox.
    pub(crate) mailbox_compatibles: &'static [&'static [u8]],
    /// The mailbox interrupts: send-empty, send-not-empty, recv-empty, recv-not-empty, as the
    /// mailbox node's `interrupts` cells (three per interrupt, or four with the die cell of a
    /// multi-die interrupt controller).
    pub(crate) mailbox_interrupts: &'static [u32],
    /// The register windows of the GPU node and its mailbox.
    pub(crate) windows: Windows,
    /// The identification words the runtime admits.
    pub(crate) id: IdWords,
    /// GPU firmware images the runtime can identify.
    pub(crate) images: &'static [KnownImage],
    /// The layout of the loaded firmware the runtime accepts.
    pub(crate) firmware: Option<&'static Layout>,
    /// The hardware configuration of the InitData.
    pub(crate) hwcfg: Option<&'static hw::HwConfig>,
    /// The firmware IO mappings of the InitData.
    pub(crate) io_mappings: Option<&'static [IoMapping]>,
    /// The runtime's mapping of each of those IO mappings: the CPU physical block and the
    /// firmware VA it is mapped at. One per entry of `io_mappings`, covering it.
    pub(crate) iomaps: Option<&'static [IoMap]>,
    /// The performance-state table the runtime accepts.
    pub(crate) pstates: PstateTable,
    /// The SGX write (offset, value) made after the identity checks, before the firmware starts.
    pub(crate) sgx_setup: Option<(usize, u32)>,
    /// The runtime InitData's HwDataB configuration words and unit masks.
    pub(crate) hwdata_b: Option<&'static G15RuntimeHwDataB>,
    /// The optional userspace features advertised.
    pub(crate) features: Features,
    /// Whether the power-management coefficients, the leakage coefficients and the operating
    /// points are this machine's, added by the boot loader from its ADT, rather than static
    /// device-tree values. Admission then requires every one of them (see
    /// `m3_board::check_boot_loader_power`), with no driver default standing in except the
    /// idle-off standby timer, which an ADT may lack.
    pub(crate) power_from_boot_loader: bool,
    /// Code the runtime runs after admission that still holds another SoC's values.
    pub(crate) unported: &'static [&'static str],
    /// The GPU registers read after every job by default (`asahi.m3_retire_mmio` bits).
    pub(crate) retire_mmio: u64,
    /// The MTR sensor masks, when they are not the hardware configuration's.
    pub(crate) mtr_masks: Option<MtrMasks>,
    /// The HwData object's placement and mapping, when they are not the fixed allocation's.
    pub(crate) hwdata_object: Option<HwDataObject>,
    /// The register lists the GPU's firmware expects in compute and render commands.
    pub(crate) registers: RegisterSet,
    /// Globals +0x7d0, which gates the firmware's frequency-feedback cap: the firmware measures
    /// the GPU clock against the requested performance state and, while this is 1, caps the
    /// state on a shortfall (`asahi.m3_ut_engagement` overrides it).
    pub(crate) ut_engagement: u32,
    /// The highest GPU power target, in mW, while the boot loader's power model is a stand-in:
    /// the operating points' powers are scaled down together so that the highest is this.
    pub(crate) power_target_cap_mw: Option<u32>,
    /// Words of the generated HwData object (offset into the object, value) that this SoC's
    /// firmware takes with other values than the shared builder writes.
    pub(crate) hwdata_words: &'static [(usize, u32)],
    /// The same for the Globals object.
    pub(crate) globals_words: &'static [(usize, u32)],
    /// The boot loader's decoded GPU leakage fuse, when it exports one for this SoC.
    pub(crate) leak_fuse: Option<LeakFuse>,
    /// Whether each operating point may give each cluster its own voltage (per-cluster, binned
    /// voltage tables). Otherwise every cluster of a point must have the same.
    pub(crate) per_cluster_voltages: bool,
}

impl Soc {
    /// Reject incomplete configuration before accessing the GPU.
    pub(crate) fn require_complete(&self, dev: &device::Device) -> Result {
        let mut missing = 0u32;
        let mut note = |what: &str| {
            dev_info!(dev, "M3 {}: missing {}\n", self.gpu_name, what);
            missing += 1;
        };
        if !self.images.iter().any(|image| image.initdata_magic.is_some()) {
            note("the identity of the loaded GPU firmware image and its InitData version");
        }
        if self.firmware.is_none() {
            note("the layout of the loaded GPU firmware (segment sizes and VAs)");
        }
        if self.hwcfg.is_none() {
            note("the hardware configuration of the InitData (HwConfig)");
        }
        if self.io_mappings.is_none() {
            note("the firmware IO mappings of the InitData");
        }
        if self.iomaps.is_none() {
            note("the runtime's mapping of the firmware IO mappings (IO maps)");
        }
        if self.sgx_setup.is_none() {
            note("the SGX setup write made before the firmware starts");
        }
        if self.hwdata_b.is_none() {
            note("the runtime HwDataB configuration words and unit masks");
        }
        for &what in self.unported {
            note(what);
        }
        if missing == 0 {
            return Ok(());
        }
        dev_err!(
            dev,
            "M3 {}: {} configuration requirements missing; GPU startup disabled\n",
            self.gpu_name,
            missing
        );
        Err(ENODEV)
    }
}

/// The HwDataB configuration words and unit masks of the T8122 runtime InitData: the values the
/// runtime was checked with on the T6030 (J514S, J516S), except for four words the G15G firmware
/// takes with other values (+0xa2c, the chip revision's major number, 2; +0xb20, the core slots,
/// 0x0a; +0x17b8, 4; +0x1818, all ones). The unit masks are the T6030's.
pub(crate) static T8122_HWDATA_B: G15RuntimeHwDataB = G15RuntimeHwDataB {
    unk_454: 2,
    unk_464: 1,
    unk_a7c: 0x1_0000_0001,
    unk_a98: 0,
    unk_abc: 4,
    unk_ae4: 0x31,
    unk_b20: 0x0a,
    unk_b24: 3,
    unk_554: 0,
    unk_17b8: 4,
    unit_mask_a: 0x7_0000_0003,
    unit_mask_b: 7,
    unk_1808: 1,
    unk_1818: 0xffff_ffff,
};

/// Every SoC the M3 runtime has a table for. The T6030 and T6031 are the G15 generation's
/// (`g15`), not the runtime's.
pub(crate) static SOCS: [&Soc; 1] = [&T8122];

/// The table of the SoC with chip id `chip_id`, if the M3 runtime has one.
pub(crate) fn by_chip(chip_id: u32) -> Option<&'static Soc> {
    SOCS.iter().copied().find(|soc| soc.chip_id == chip_id)
}

/// T8122 (M3, G15G): one die, one cluster of ten core slots (eight or ten of them active).
///
/// The compatibles, the mailbox and its interrupts are the T8122 device tree's. The identity is
/// the one the T8122 identity gate admits (`t8122_admission`: family 7, variant 2, revision 0x20,
/// core slots in the first core-mask word only), with the die count of the AGX3 identification
/// table (`hw::agx3::T8122`). The hardware configuration is `hw::t8122`; power configuration
/// comes from the boot loader. The rest of the runtime configuration is complete, but until the
/// runtime has passed on an M3 MacBook Air the SoC is refused, before the GPU is accessed, unless
/// `asahi.t8122_start=1` arms the start (`t8122_start`), which uses this table's values and adds a
/// performance-state cap.
pub(crate) static T8122: Soc = Soc {
    name: "T8122",
    gpu_name: "G15G",
    chip_id: 0x8122,
    gpu_variant: hw::GpuVariant::G,
    gpu_revision: hw::GpuRevision::C0,
    gpu_revision_id: hw::GpuRevisionID::C0,
    board: "apple,t8122",
    gpu: "apple,agx-t8122",
    // The M3 MacBook Airs (13" J613, 15" J615). Allowed, not validated: the runtime has not run
    // on either, and starts only behind a boot loader that hands the GPU over.
    validated_boards: &[b"apple,j613", b"apple,j615"],
    clusters: 1,
    cores_per_cluster: 10,
    mailbox_compatibles: &[
        b"apple,t8122-agx-asc-mailbox\0",
        b"apple,t8122-asc-mailbox\0apple,asc-mailbox-v4\0",
    ],
    mailbox_interrupts: &[0, 723, 4, 0, 724, 4, 0, 725, 4, 0, 726, 4],
    // The J613 ADT: sgx reg[0] (child 0x80000000 + arm-io 0x210000000) and gfx-asc reg[0]; the
    // mailbox at asc + 0x8000 (t8122-gpu.dtsi). The same addresses as on T6030.
    windows: Windows {
        asc: 0x2_9240_0000,
        sgx: 0x2_9000_0000,
        mailbox: 0x2_9240_8000,
    },
    id: IdWords {
        version: 0x0702_2000,
        version_mask: 0xffff_ff00,
        // One die, one cluster per die; the other fields are not known.
        counts: 0x0001_0100,
        counts_mask: 0x000f_ff00,
    },
    images: &crate::m3_board::KNOWN_IMAGES_T8122,
    firmware: Some(&crate::m3_firmware::T8122_LAYOUT),
    hwcfg: Some(&hw::t8122::HWCONFIG_T8122),
    io_mappings: Some(&crate::m3_adt_config::T8122_IO_MAPPINGS),
    iomaps: Some(&crate::m3_adt_config::T8122_IOMAPS),
    // The boot loader's ladder from this machine's ADT (J613: eight voltages, up to 1338 MHz).
    pstates: PstateTable::DeviceTree,
    // The same SGX setup write as on T6030.
    sgx_setup: Some((0xd14000, 0x70001)),
    hwdata_b: Some(&T8122_HWDATA_B),
    // Neither is validated on G15G yet; userspace keeps its default ordering and visibility.
    features: Features {
        compute_wide_visibility: false,
        fragment_dependency: false,
    },
    power_from_boot_loader: true,
    unported: &[
        "a pass on an M3 MacBook Air: until then the GPU starts only with asahi.t8122_start=1",
    ],
    // The performance state only. On the single-cluster G15G, reading the engine-busy or the
    // fault-bank registers after every job hangs the SoC within seconds at about 1000 jobs/s;
    // the performance-state read alone does not, and a per-frame readback saw no job retired
    // before its writes were visible.
    retire_mmio: 4,
    // The fast-die controller's sensors (0x4248, both copies), and every sensor the MTR block
    // enables as the alarm mask (sensors 3, 6, 8, 9, 11 and 14: 0x4b48, the upper half of the
    // block's configuration word 0x4b480003). With the second fast-die copy at 0, or an alarm
    // mask without sensor 8, the firmware stopped on its first MTR alarm.
    mtr_masks: Some(MtrMasks {
        fast_die: 0x4248,
        alarm: 0x4b48,
    }),
    // The firmware's MTR alarm handler does a 64-bit atomic update of HwDataA +0x4350. The fixed
    // allocation ends the 0x8a04-byte object at its page end, leaving HwDataA (+0x4580) only
    // 4-byte aligned, and the update took an alignment fault. Starting the object 0x8a80 bytes
    // before the same page end puts HwDataA on a 16-byte boundary. Aligned, the update then
    // faulted as an unsupported atomic on the uncached mapping: map it cacheable.
    hwdata_object: Some(HwDataObject {
        address: T8122_HWDATA_ADDRESS,
        cached: true,
    }),
    registers: RegisterSet::G15G,
    // On a T8122 with the 14.8.3 system firmware the shader clock runs at 3/4 of every requested
    // state, so with the cap engaged the GPU stays at state 2 (462 MHz effective) under any load.
    // With 0 it reaches the requested states (state 8: about 1000 MHz effective).
    ut_engagement: 0,
    // The boot loader's T8122 power model is a stand-in that overstates the power (its highest
    // operating point is about 30 W). Until each Mac's fused leakage gives a real one, scale it
    // to a 22 W target, about the GPU power budget of an M3 MacBook Air.
    power_target_cap_mw: Some(22_000),
    hwdata_words: &T8122_HWDATA_WORDS,
    globals_words: &T8122_GLOBALS_WORDS,
    // Exported on a J613 on every boot; used only with the boot loader's switch, which also makes
    // value 1 its apple,core-leak-coef and the base of its operating points' powers.
    leak_fuse: Some(LeakFuse {
        values: c_str!("asahi,t8122-gpu-leak-fuse"),
        switch: c_str!("asahi,t8122-gpu-fuse-leakage"),
    }),
    per_cluster_voltages: false,
};

/// The T8122 HwData words that differ from the shared builder's (offsets into the object;
/// HwDataA starts at +0x4580):
/// - HwDataB +0xa30 and +0xa34: 0 and 4 (the builder writes 1 and 0);
/// - HwDataB +0x17e0..+0x1860: the G15G firmware's flag words (the builder writes 1 at
///   +0x17e8, +0x1804, +0x1814 and +0x1860 and all-ones at +0x1848);
/// - HwDataA +0x11e0: 30 (the builder writes 40), a word of the shader-engine controller block;
/// - HwDataA +0x1290: 125, and HwDataA +0x424c: 24000000, the 24 MHz reference clock.
pub(crate) static T8122_HWDATA_WORDS: [(usize, u32); 17] = [
    (0xa30, 0),
    (0xa34, 4),
    (0x17e0, 1),
    (0x17e8, 0),
    (0x17f4, 1),
    (0x17f8, 1),
    (0x1804, 0),
    (0x180c, 1),
    (0x1814, 0),
    (0x181c, 0xffff_ffff),
    (0x1848, 0),
    (0x184c, 0),
    (0x1858, 1),
    (0x1860, 0),
    (0x4580 + 0x11e0, 30),
    (0x4580 + 0x1290, 125),
    (0x4580 + 0x424c, 24_000_000),
];

/// The T8122 Globals words that differ from the shared builder's: +0x9bc, the CDM backoff
/// timeout (4) with the three bytes after it 0 (the builder writes 1, 0, 0).
pub(crate) static T8122_GLOBALS_WORDS: [(usize, u32); 1] = [(0x9bc, 4)];

/// The T8122 HwData object's firmware VA: 0x8a80 bytes before the end of the fixed allocation's
/// last page.
pub(crate) const T8122_HWDATA_ADDRESS: u64 = 0xffff_fc20_4070_4000 - 0x8a80;

// The fixed HwData allocation (`m3_init_storage::allocation`): 0x8a04 bytes ending at the page
// end 0xfffffc2040704000. The moved object stays inside the same pages, and HwDataA is 16-byte
// aligned.
const _: () = {
    let (fixed, size) = (0xffff_fc20_406f_b5fc_u64, 0x8a04_u64);
    assert!(T8122_HWDATA_ADDRESS & !0x3fff == fixed & !0x3fff);
    assert!(T8122_HWDATA_ADDRESS + size <= (fixed + size + 0x3fff) & !0x3fff);
    assert!((T8122_HWDATA_ADDRESS + crate::m3_adt_config::HWDATA_A as u64) % 16 == 0);
};
