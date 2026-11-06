// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! T8122 (M3, G15G) board-default startup and bounded experiment values.
//!
//! The T8122 table (`m3_soc::T8122`) is complete. An omitted start parameter starts
//! automatically only on its listed Air boards; explicit `asahi.t8122_start=0`
//! remains the recovery opt-out. Resource and known-firmware admission still run.
//! The same table values can be changed for a controlled experiment by
//! rebooting with a parameter:
//!
//! | Parameter | Default | Others |
//! |---|---|---|
//! | `asahi.t8122_initdata_version` | `0x0c08e21e83800490`, the G15 14.8.3 version | any |
//! | `asahi.t8122_fender` | `0x104000` (`rule`) | `0x12c000` (`adt`) |
//! | `asahi.t8122_clkgen` | `e1c`: SGX+0xe1c000, read-only | `e5c`: SGX+0xe5c000, read-only; `none` |
//! | `asahi.t8122_sgx_setup` | `t6030`: SGX+0xd14000 = 0x70001 | `none` |
//! | `asahi.t8122_unit_mask_a` | `0x700000003` | nonzero, within `0x700000003` |
//! | `asahi.t8122_unit_mask_b` | `0x7` | nonzero, within `0x7` |
//! | `asahi.t8122_pstate_cap` | thermal policy (below) | fixed ceiling 1 to 5 |
//!
//! An omitted cap uses the full table with thermal control, starting/falling back at state 2.
//! With thermal control off it retains a fixed ceiling of 2.
//!
//! The defaults are the T8122 table's values. A parameter given with a value it does not accept
//! refuses the start. Unlisted boards remain refused by the existing backend admission.
//! Nothing here applies to any other SoC.

use core::mem::offset_of;

use kernel::{device, prelude::*};

use crate::{
    m3::g15_raw::raw,
    m3::initdata::G15RuntimeHwDataB,
    m3::m3_adt_config::T8122_IO_MAPPINGS,
    m3::m3_init_storage::{self as storage, IoMap},
    m3::m3_soc::{IoMapping, Soc, T8122_HWDATA_B},
    pgtable::{prot, Prot},
    m3::t8122_knobs::{
        self as knobs, ClockGen, Start, Values, CLOCK_GEN_E1C, CLOCK_GEN_E5C, FENDER_ADT,
        FENDER_RULE,
    },
};

/// The GPU register window (SGX) of T8122.
const SGX: u64 = 0x2_9000_0000;
/// Index of the Fender window and of the GPU clock generator in [`T8122_IO_MAPPINGS`].
const FENDER: usize = 0;
const CLOCK_GEN: usize = 11;
/// The HwDataB slot of the GPU clock generator.
const CLOCK_GEN_SLOT: usize = 29;


/// The firmware IO mappings with the Fender window `fender` and the clock generator at
/// SGX + `clock_gen`.
const fn mappings(fender: u32, clock_gen: u64) -> [IoMapping; 12] {
    let mut m = T8122_IO_MAPPINGS;
    m[FENDER].2 = fender;
    m[FENDER].3 = fender;
    m[CLOCK_GEN].1 = SGX + clock_gen;
    m
}

/// The runtime IO maps, laid out once for the larger Fender window so that no slot's firmware
/// VA depends on the parameters.
const LAYOUT: [IoMap; 12] =
    storage::pack_iomaps(&mappings(FENDER_ADT, CLOCK_GEN_E1C), storage::IOMAP_BASE);

const fn iomaps(fender: u32, clock_gen: u64) -> [IoMap; 12] {
    let mut m = LAYOUT;
    m[FENDER].size = fender as usize;
    m[CLOCK_GEN].physical = SGX + clock_gen;
    m
}

/// Every Fender window and clock-generator address, in [`Experiment::variant`] order.
static IO_MAPPINGS: [[IoMapping; 12]; 4] = [
    mappings(FENDER_RULE, CLOCK_GEN_E5C),
    mappings(FENDER_RULE, CLOCK_GEN_E1C),
    mappings(FENDER_ADT, CLOCK_GEN_E5C),
    mappings(FENDER_ADT, CLOCK_GEN_E1C),
];
static IOMAPS: [[IoMap; 12]; 4] = [
    iomaps(FENDER_RULE, CLOCK_GEN_E5C),
    iomaps(FENDER_RULE, CLOCK_GEN_E1C),
    iomaps(FENDER_ADT, CLOCK_GEN_E5C),
    iomaps(FENDER_ADT, CLOCK_GEN_E1C),
];

/// Whether each IO map holds its IO mapping: same slot, the mapping's 16 KiB-aligned physical
/// base, its subpage offset, and room for its total size.
const fn maps_cover(mappings: &[IoMapping; 12], maps: &[IoMap; 12]) -> bool {
    let mut i = 0;
    while i < 12 {
        let (slot, phys, total, _, _) = mappings[i];
        let io = maps[i];
        if io.slot != slot
            || io.physical + io.offset as u64 != phys
            || io.offset + total as usize > io.size
            || io.size % 0x4000 != 0
        {
            return false;
        }
        i += 1;
    }
    true
}

const _: () = {
    // The default variant's IO mappings are the T8122 table's.
    let defaults = mappings(FENDER_RULE, CLOCK_GEN_E1C);
    let mut j = 0;
    while j < 12 {
        let (a, b) = (defaults[j], T8122_IO_MAPPINGS[j]);
        assert!(a.0 == b.0 && a.1 == b.1 && a.2 == b.2 && a.3 == b.3 && a.4 == b.4);
        j += 1;
    }
    // The T8122 table: the Fender window at the rule size, the clock generator last, at
    // SGX+0xe1c000, 16 KiB, read-only.
    assert!(T8122_IO_MAPPINGS[FENDER].0 == 0 && T8122_IO_MAPPINGS[FENDER].1 == SGX + 0xd0_0000);
    assert!(T8122_IO_MAPPINGS[FENDER].2 == FENDER_RULE);
    assert!(T8122_IO_MAPPINGS[CLOCK_GEN].0 == CLOCK_GEN_SLOT);
    assert!(T8122_IO_MAPPINGS[CLOCK_GEN].1 == SGX + CLOCK_GEN_E1C);
    assert!(T8122_IO_MAPPINGS[CLOCK_GEN].2 == 0x4000 && !T8122_IO_MAPPINGS[CLOCK_GEN].4);
    assert!(CLOCK_GEN + 1 == T8122_IO_MAPPINGS.len());
    // Both windows are whole pages and hold the Fender scratch area (+0x60000..+0x80000).
    assert!(FENDER_RULE % 0x4000 == 0 && FENDER_ADT % 0x4000 == 0 && FENDER_RULE >= 0x8_0000);
    assert!(FENDER_RULE < FENDER_ADT);
    // Both clock-generator blocks are whole pages inside the first 16 MiB of SGX.
    assert!(CLOCK_GEN_E5C % 0x4000 == 0 && CLOCK_GEN_E1C % 0x4000 == 0);
    assert!(CLOCK_GEN_E5C + 0x4000 <= 0x100_0000 && CLOCK_GEN_E1C + 0x4000 <= 0x100_0000);
    let mut i = 0;
    while i < 4 {
        assert!(maps_cover(&IO_MAPPINGS[i], &IOMAPS[i]));
        i += 1;
    }
    // The default unit masks are the T8122 table's, and the limits allow them.
    assert!(T8122_HWDATA_B.unit_mask_a == knobs::UNIT_MASK_A);
    assert!(T8122_HWDATA_B.unit_mask_b == knobs::UNIT_MASK_B);
    assert!(knobs::UNIT_MASK_A & !knobs::UNIT_MASK_A_LIMIT == 0);
    assert!(knobs::UNIT_MASK_B as u64 & !knobs::UNIT_MASK_B_LIMIT == 0);
    assert!(offset_of!(raw::HwDataBG15V14_8_3, unit_mask_a) == 0x17c0);
    assert!(offset_of!(raw::HwDataBG15V14_8_3, unit_mask_b) == 0x17c8);
    // An armed start skips `require_complete`. The only `unported` item of the T8122 table is the
    // opt-in itself; if another is added, this fails the build until the start accounts for it.
    assert!(crate::m3::m3_soc::T8122.unported.len() == 1);
};

/// The values of one armed boot.
#[derive(Copy, Clone, Debug)]
pub(crate) struct Experiment(Values);

/// Whether `soc` is the T8122 table.
pub(crate) fn is_t8122(soc: &Soc) -> bool {
    core::ptr::eq(soc, &crate::m3::m3_soc::T8122)
}

impl Experiment {
    /// Index of this boot's IO-mapping table in [`IO_MAPPINGS`] and [`IOMAPS`].
    fn variant(&self) -> usize {
        let adt = usize::from(self.0.fender == FENDER_ADT);
        let e1c = usize::from(self.0.clock_gen == ClockGen::At(CLOCK_GEN_E1C));
        2 * adt + e1c
    }

    /// How many IO mappings this boot gives the firmware: all, or all but the clock generator.
    fn mapping_count(&self) -> usize {
        match self.0.clock_gen {
            ClockGen::At(_) => CLOCK_GEN + 1,
            ClockGen::Absent => CLOCK_GEN,
        }
    }

    /// The InitData version given to the firmware (`asahi.t8122_initdata_version`).
    pub(crate) fn initdata_version(&self) -> u64 {
        self.0.initdata_version
    }

    /// The firmware IO mappings of the InitData.
    pub(crate) fn io_mappings(&self) -> &'static [IoMapping] {
        &IO_MAPPINGS[self.variant()][..self.mapping_count()]
    }

    /// The runtime's mapping of each of them. With the default Fender window and clock generator
    /// they are the T8122 table's own IO maps.
    pub(crate) fn iomaps(&self) -> &'static [IoMap] {
        if self.0.fender == FENDER_RULE && self.0.clock_gen == ClockGen::At(CLOCK_GEN_E1C) {
            return &crate::m3::m3_adt_config::T8122_IOMAPS;
        }
        &IOMAPS[self.variant()][..self.mapping_count()]
    }

    /// The register block the IO mapping of `slot` must lie in, given the HwConfig's block
    /// `base` for it: the selected clock generator for slot 29.
    pub(crate) fn io_block(&self, slot: usize, base: u64) -> u64 {
        match self.0.clock_gen {
            ClockGen::At(offset) if slot == CLOCK_GEN_SLOT => SGX + offset,
            _ => base,
        }
    }

    /// The HwDataB slots the firmware maps read-only, as a bit mask: the clock generator's.
    pub(crate) fn read_only_slots(&self) -> u32 {
        match self.0.clock_gen {
            ClockGen::At(_) => 1 << CLOCK_GEN_SLOT,
            ClockGen::Absent => 0,
        }
    }

    /// The runtime HwDataB words: the T8122 table's. The unit masks are written over them
    /// ([`Self::unit_masks`]).
    pub(crate) fn hwdata_b(&self) -> &'static G15RuntimeHwDataB {
        &T8122_HWDATA_B
    }

    /// The HwDataB unit masks A (+0x17c0) and B (+0x17c8).
    pub(crate) fn unit_masks(&self) -> (u64, u32) {
        (self.0.unit_mask_a, self.0.unit_mask_b)
    }

    /// The SGX write (offset, value) made before the firmware starts, if any.
    pub(crate) fn sgx_setup(&self) -> Option<(usize, u32)> {
        self.0.sgx_setup
    }

    /// T8122 ceiling and startup cap, retaining an explicit `asahi.t8122_pstate_cap`.
    pub(crate) fn pstate_limits(&self, thermal_enabled: bool) -> knobs::PstateLimits {
        self.0.pstate_limits(thermal_enabled)
    }
}

/// Firmware page protection of the IO map of `slot`, with the read-only slots `read_only`.
pub(crate) fn mmio_prot(read_only: u32, slot: usize) -> Prot {
    if slot < 32 && read_only & (1 << slot) != 0 {
        prot::PROT_FW_MMIO_RO
    } else {
        prot::PROT_FW_MMIO_RW
    }
}

/// Decide, after resource admission, whether `soc` starts with the experiment values.
///
/// Any SoC but T8122: Ok(None). On its listed Air boards, an omitted parameter
/// selects the table values; explicit 0 disables startup, and invalid values fail
/// closed. Existing backend admission still refuses an unlisted board, even
/// with an explicit start request.
pub(crate) fn arm(dev: &device::Device, soc: &Soc) -> Result<Option<Experiment>> {
    if !is_t8122(soc) {
        return Ok(None);
    }
    let raw = crate::m3::m3_params::t8122_params();
    match knobs::start_for_board(&raw, crate::m3::m3_board::runtime_validated_board(soc)) {
        Start::On => {}
        Start::Off => {
            if raw.values_given() {
                dev_info!(
                    dev,
                    "M3 G15G start: disabled by asahi.t8122_start=0; other asahi.t8122_* values ignored\n"
                );
            }
            return Ok(None);
        }
        Start::Invalid => {
            dev_err!(
                dev,
                "M3 G15G start: refused: asahi.t8122_start has a value it does not accept (0 or 1); GPU startup disabled\n"
            );
            return Err(ENODEV);
        }
    }
    // Admission has required the boot loader's handoff: its firmware segments and reserved
    // regions, and on this SoC its power configuration.
    if !soc.power_from_boot_loader || soc.firmware.is_none() || soc.hwcfg.is_none() {
        dev_err!(dev, "M3 G15G start: refused: the T8122 table is not the one this experiment expects\n");
        return Err(ENODEV);
    }
    let v = knobs::resolve(&raw).map_err(|refusal| {
        dev_err!(
            dev,
            "M3 G15G start: refused: asahi.{} has a value it does not accept ({}); GPU startup disabled\n",
            refusal.name,
            refusal.accepts
        );
        ENODEV
    })?;
    let e = Experiment(v);
    dev_warn!(
        dev,
        "M3 G15G start: armed (board default or asahi.t8122_start=1, boot loader handoff admitted): initdata_version={:#018x} fender={:#x} clkgen={} sgx_setup={} unit_mask_a={:#x} unit_mask_b={:#x} pstate_cap={} pstate_cap_explicit={}\n",
        v.initdata_version,
        v.fender,
        clkgen_label(v.clock_gen, v.fender),
        match v.sgx_setup {
            Some(_) => "t6030(sgx+0xd14000=0x70001)",
            None => "none",
        },
        v.unit_mask_a,
        v.unit_mask_b,
        v.pstate_cap,
        v.pstate_cap_explicit
    );
    dev_warn!(
        dev,
        "M3 G15G start: the T8122 table's InitData words, HwData placement and register lists; {} IO mappings at firmware VA {:#x}\n",
        e.mapping_count(),
        storage::IOMAP_BASE
    );
    Ok(Some(e))
}

/// The armed line's clock-generator field. It says `ro` for the firmware's own slot-29 mapping,
/// and adds that the Fender window reaches the block read-write when the chosen window covers it
/// (SGX+0xe1c000 lies under the 0x12c000 window but not the 0x104000 one; SGX+0xe5c000 under
/// neither).
fn clkgen_label(clock_gen: ClockGen, fender: u32) -> &'static str {
    let fender_end = 0xd0_0000u64 + u64::from(fender);
    let reaches = |off: u64| off + 0x4000 <= fender_end;
    match clock_gen {
        ClockGen::At(CLOCK_GEN_E1C) if reaches(CLOCK_GEN_E1C) => {
            "e1c(sgx+0xe1c000,ro; also rw in the fender window)"
        }
        ClockGen::At(CLOCK_GEN_E1C) => "e1c(sgx+0xe1c000,ro)",
        ClockGen::At(_) if reaches(CLOCK_GEN_E5C) => {
            "e5c(sgx+0xe5c000,ro; also rw in the fender window)"
        }
        ClockGen::At(_) => "e5c(sgx+0xe5c000,ro)",
        ClockGen::Absent if reaches(CLOCK_GEN_E1C) => {
            "none (but the fender window reaches sgx+0xe1c000 rw)"
        }
        ClockGen::Absent => "none",
    }
}

/// Log, in an armed start, that the probe stopped at `stage` before the firmware started.
pub(crate) fn refused(dev: &device::Device, experiment: Option<&Experiment>, stage: &str, error: Error) {
    if experiment.is_some() {
        dev_err!(
            dev,
            "M3 G15G start: refused at {} ({:?}); the firmware was not started, see the lines above\n",
            stage,
            error
        );
    }
}

/// Log that resource admission refused a T8122 start selected by board default or flag.
pub(crate) fn not_admitted(dev: &device::Device, soc: &Soc) {
    if is_t8122(soc) && knobs::start_for_board(
        &crate::m3::m3_params::t8122_params(),
        crate::m3::m3_board::runtime_validated_board(soc),
    ) == Start::On {
        dev_err!(
            dev,
            "M3 G15G start: not armed: resource admission refused the GPU (see the lines above); the boot loader did not hand it over completely\n"
        );
    }
}

// One `M3 G15G verdict:` line per outcome of an armed start, so that a tester can tell them
// apart from the log alone. They run on T8122 only (callers check `is_t8122`); nothing here
// changes what the runtime does.

/// The GPU coprocessor's start or the driver's InitData upload failed (`m3_runtime::Runtime::new`).
/// `started`: the coprocessor ran and offered its endpoints.
pub(crate) fn prepare_verdict(dev: &device::Device, soc: &Soc, started: bool, error: Error) {
    if !is_t8122(soc) {
        return;
    }
    if started {
        dev_err!(
            dev,
            "M3 G15G verdict: driver-refused ({:?}): the GPU firmware is up, but the driver did not publish the InitData (UAT, upload checks or thermal setup, see above)\n",
            error
        );
    } else {
        dev_err!(
            dev,
            "M3 G15G verdict: firmware-boot-failed ({:?}): the GPU coprocessor did not start, or its RTKit did not offer endpoints 0x20/0x21; no InitData was published\n",
            error
        );
    }
}

/// The step the firmware boot sequence reached (`m3_runtime::Runtime::boot_inner`). The verdict
/// uses it to name a failure as a publish, a device-control or an InitData-acknowledgement
/// problem, rather than lumping them all under "initdata-rejected".
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub(crate) enum BootStep {
    /// About to send, or sending, the InitData publish message (endpoint 0x20).
    Publish,
    /// A device-control message (0x13 or 9) before the firmware signalled ready.
    DeviceControl,
    /// Waiting (up to 2 s) for the firmware to write its ready words.
    AwaitReady,
    /// The firmware signalled ready; a control or check after that.
    PostReady,
}

/// The firmware's answer to the published InitData (`m3_runtime::Runtime::boot`). `accepted`:
/// the firmware wrote its ready words. `step`: the furthest step the boot reached.
pub(crate) fn boot_verdict(
    dev: &device::Device,
    version: u64,
    step: BootStep,
    accepted: bool,
    crashed: bool,
    result: Result,
) {
    let Err(e) = result else {
        dev_info!(
            dev,
            "M3 G15G verdict: firmware-running: the firmware accepted the InitData (version {:#x}) and the device controls; no job has run yet\n",
            version
        );
        return;
    };
    // The firmware signalled ready, then a later control or check failed: the InitData was
    // accepted, so this is not a rejection.
    if accepted || step == BootStep::PostReady {
        if crashed {
            dev_err!(
                dev,
                "M3 G15G verdict: firmware-running-check-failed ({:?}): the firmware accepted the InitData (version {:#x}) then crashed during a post-boot control or check; see the crash lines above\n",
                e,
                version
            );
        } else {
            dev_err!(
                dev,
                "M3 G15G verdict: firmware-running-check-failed ({:?}): the firmware accepted the InitData (version {:#x}), then a post-boot control or check failed (see above)\n",
                e,
                version
            );
        }
        return;
    }
    match step {
        BootStep::Publish => dev_err!(
            dev,
            "M3 G15G verdict: publish-failed ({:?}): the InitData publish message (endpoint 0x20) could not be sent; the firmware was not told about the InitData (version {:#x})\n",
            e,
            version
        ),
        BootStep::DeviceControl => dev_err!(
            dev,
            "M3 G15G verdict: device-control-failed ({:?}): a device-control queue, mailbox send or acknowledgement failed after the InitData (version {:#x}) was published; see the device-control diagnostic above\n",
            e,
            version
        ),
        // AwaitReady (and PostReady is handled above).
        _ if crashed => dev_err!(
            dev,
            "M3 G15G verdict: initdata-rejected ({:?}): the firmware crashed after the InitData (version {:#x}) was published; see the crash lines above\n",
            e,
            version
        ),
        _ => dev_err!(
            dev,
            "M3 G15G verdict: initdata-rejected ({:?}): the firmware did not acknowledge the InitData (version {:#x}) within 2 s (its ready words stayed unset); see M3 firmware readiness above\n",
            e,
            version
        ),
    }
}

/// A GPU coprocessor setup step failed before the firmware could be started
/// (`m3_runtime::Runtime::new`): the RTKit state, the mailbox transport or the DRM device.
pub(crate) fn coproc_setup_refused(dev: &device::Device, error: Error) {
    dev_err!(
        dev,
        "M3 G15G start: refused at the GPU coprocessor setup (RTKit/mailbox) ({:?}); the firmware was not started\n",
        error
    );
}

/// The firmware ran a job above the performance-state cap (`m3_runtime`, the after-job pstate
/// check). The `M3: firmware ... stays above the cap ...` line precedes this.
pub(crate) fn cap_violated_verdict(dev: &device::Device, error: Error) {
    dev_err!(
        dev,
        "M3 G15G verdict: cap-violated ({:?}): after a job the firmware reported a performance state above the cap; the GPU was marked failed (see the 'stays above the cap' line above)\n",
        error
    );
}

/// A job-path step failed outside the completion wait (`m3_runtime::Runtime::execute`): a submit,
/// a doorbell, or a post-completion read. No dispatch outcome was determined.
pub(crate) fn job_setup_failed_verdict(dev: &device::Device, error: Error) {
    dev_err!(
        dev,
        "M3 G15G verdict: job-failed-before-wait ({:?}): a submit, doorbell or diagnostic step failed, so no dispatch outcome was determined; see the lines above\n",
        error
    );
}

/// A job that failed (`m3_runtime::Inner::fail`). `primary` is ETIMEDOUT when it did not retire
/// within the runtime's per-batch bound; `start_stamp` is the job's first GPU start (CDM
/// dispatch) timestamp, if readable; `pstate` is the GPU performance-state register, if readable.
pub(crate) fn job_failed_verdict(
    dev: &device::Device,
    primary: Error,
    start_stamp: Option<u64>,
    pstate: Option<u32>,
    crashed: bool,
) {
    if primary != ETIMEDOUT || crashed {
        dev_err!(
            dev,
            "M3 G15G verdict: job-faulted ({:?}): the firmware crashed, reported an error, or the GPU reported a fault while the job ran; see the lines around this one\n",
            primary
        );
        return;
    }
    // A nonzero start timestamp means the firmware dispatched the job on the GPU, so this was not
    // a dispatch gap however the power state reads now (the GPU may have idled off since). The
    // stamp and pstate print as `Some(hex)` or `None`.
    if start_stamp.is_some_and(|start| start != 0) {
        dev_err!(
            dev,
            "M3 G15G verdict: job-ran-completion-missed: the job was dispatched (GPU start timestamp {:x?}) but its completion was not seen within the per-batch bound (pstate register {:x?})\n",
            start_stamp,
            pstate
        );
        return;
    }
    // The GPU start timestamp is 0 (or unreadable): the job was not dispatched.
    match pstate {
        Some(p) if p & 0xf == 0 => dev_err!(
            dev,
            "M3 G15G verdict: job-accepted-never-dispatched: the job was not dispatched (GPU start timestamp {:x?}) and the GPU reads powered down (pstate register {:#x})\n",
            start_stamp,
            p
        ),
        Some(p) => dev_err!(
            dev,
            "M3 G15G verdict: job-timed-out-powered: the job did not retire with the GPU powered (pstate register {:#x}, GPU start timestamp {:x?}); see the engine snapshot below\n",
            p,
            start_stamp
        ),
        None => dev_err!(
            dev,
            "M3 G15G verdict: job-timed-out: the job did not retire (pstate register and GPU start timestamp both unreadable)\n"
        ),
    }
}

/// The first job of a kind (0 render, 1 compute) retired (`m3_runtime::Runtime::execute`):
/// `span` is its first start and last end GPU timestamp (24 MHz ticks), if readable.
pub(crate) fn job_completed_verdict(dev: &device::Device, kind: usize, gpu_ns: u64, span: Option<[u64; 2]>) {
    let what = if kind == 0 { "render" } else { "compute" };
    match span {
        Some([start, end]) if start != 0 && end != 0 => dev_info!(
            dev,
            "M3 G15G verdict: job-completed: the first {} job finished, GPU timestamps {:#x}..{:#x}, {} ns of GPU time\n",
            what,
            start,
            end,
            gpu_ns
        ),
        Some([start, end]) => dev_err!(
            dev,
            "M3 G15G verdict: job-retired-without-timestamps: the first {} job retired, but its GPU timestamps are {:#x}..{:#x}\n",
            what,
            start,
            end
        ),
        None => dev_err!(
            dev,
            "M3 G15G verdict: job-retired-without-timestamps: the first {} job retired, but its GPU timestamps are unreadable\n",
            what
        ),
    }
}
