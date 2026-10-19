// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Module parameters of the M3 runtime backend.
//!
//! The `module!` parameter support only parses plain integers. The parameters here are declared
//! with their own `kernel_param` entries (the same layout the `module!` macro emits), so they can
//! take symbolic values on the kernel command line, e.g. `asahi.m3_thermal=hold`, and hex or
//! decimal masks.

use core::ffi::{
    c_char,
    c_int,
};
use core::sync::atomic::{
    AtomicU64,
    Ordering, //
};

use kernel::{
    device::Core,
    platform,
    prelude::*, //
};

/// Parse a decimal or `0x`-prefixed hexadecimal `u64`.
fn parse_u64(text: &str) -> Option<u64> {
    let text = text.trim();
    match text.strip_prefix("0x").or_else(|| text.strip_prefix("0X")) {
        Some(hex) => u64::from_str_radix(hex, 16).ok(),
        None => text.parse::<u64>().ok(),
    }
}

/// Store the parsed value of a parameter string into the `AtomicU64` behind `kp`.
///
/// # Safety
///
/// `val` must be NULL or a NUL-terminated string, and `kp` must be a `kernel_param` declared by
/// [`m3_param!`], whose `arg` points at a static `AtomicU64`.
unsafe fn set_param(
    val: *const c_char,
    kp: *const kernel::bindings::kernel_param,
    parse: fn(&str) -> Option<u64>,
) -> c_int {
    if val.is_null() {
        return EINVAL.to_errno();
    }
    // SAFETY: `val` is a NUL-terminated string per the function contract.
    let Ok(text) = unsafe { core::ffi::CStr::from_ptr(val) }.to_str() else {
        return EINVAL.to_errno();
    };
    let Some(value) = parse(text) else {
        return EINVAL.to_errno();
    };
    // SAFETY: `arg` points at the static `AtomicU64` this entry was declared with.
    let slot = unsafe { &*(*kp).__bindgen_anon_1.arg.cast::<AtomicU64>() };
    slot.store(value, Ordering::Relaxed);
    0
}

/// Declare a module parameter `asahi.<name>` stored in the static `AtomicU64` `$storage` and
/// parsed by `$parse: fn(&str) -> Option<u64>`. Not visible in sysfs.
macro_rules! m3_param {
    ($name:literal, $storage:ident, $parse:path) => {
        m3_param!($name, $storage, $parse, 0, None);
    };
    ($name:literal, $storage:ident, $parse:path, $perm:expr, $get:expr) => {
        const _: () = {
            unsafe extern "C" fn set(
                val: *const c_char,
                kp: *const kernel::bindings::kernel_param,
            ) -> c_int {
                // SAFETY: Called by the parameter parser with a value string (or NULL) and the
                // `kernel_param` entry declared below.
                unsafe { set_param(val, kp, $parse) }
            }

            static OPS: kernel::bindings::kernel_param_ops = kernel::bindings::kernel_param_ops {
                flags: 0,
                set: Some(set),
                get: $get,
                free: None,
            };

            #[link_section = "__param"]
            #[used(compiler)]
            static PARAM: kernel::module_param::KernelParam =
                kernel::module_param::KernelParam::new(kernel::bindings::kernel_param {
                    name: kernel::str::as_char_ptr_in_const_context(if cfg!(MODULE) {
                        kernel::c_str!($name)
                    } else {
                        kernel::c_str!(concat!("asahi.", $name))
                    }),
                    // `__this_module` is constructed by the kernel at load time and is not
                    // freed until the module is unloaded. Take its address without creating a
                    // reference to the mutable static.
                    #[cfg(MODULE)]
                    mod_: (&raw const kernel::bindings::__this_module).cast_mut(),
                    #[cfg(not(MODULE))]
                    mod_: core::ptr::null_mut(),
                    ops: core::ptr::from_ref(&OPS),
                    perm: $perm,
                    level: -1,
                    flags: 0,
                    __bindgen_anon_1: kernel::bindings::kernel_param__bindgen_ty_1 {
                        arg: core::ptr::from_ref(&$storage).cast_mut().cast(),
                    },
                });
        };
    };
}

/// Read an atomic parameter without borrowing storage that a sysfs write may
/// change concurrently. The module-parameter core supplies a PAGE_SIZE buffer.
///
/// # Safety
///
/// `buffer` must be the module-parameter core's writable output buffer and
/// `kp` must be an entry declared by `m3_param!` with AtomicU64 storage.
unsafe extern "C" fn get_atomic_param(
    buffer: *mut c_char,
    kp: *const kernel::bindings::kernel_param,
) -> c_int {
    use core::fmt::Write;

    // SAFETY: The entry points to static AtomicU64 storage by contract. Loads
    // and stores are atomic; module-parameter serialization alone would not
    // protect the scheduler's concurrent reads.
    let value = unsafe { &*((*kp).__bindgen_anon_1.arg.cast::<AtomicU64>()) }
        .load(Ordering::Relaxed);
    // SAFETY: 32 bytes fit in the core's PAGE_SIZE output buffer and hold any
    // decimal u64 plus its NUL. The core appends the sysfs newline itself.
    let bytes = unsafe { core::slice::from_raw_parts_mut(buffer.cast::<u8>(), 32) };
    let mut output = kernel::str::Formatter::new(bytes);
    if write!(output, "{}\0", value).is_err() {
        return EINVAL.to_errno();
    }
    (output.bytes_written() - 1) as c_int
}

fn parse_render_batch_override(text: &str) -> Option<u64> {
    parse_u64(text).filter(|value| *value <= crate::m3_pass_layout::SLOTS as u64)
}

/// Runtime tuning is opt-in: zero keeps the existing boot parameter and its
/// clamping semantics. Only root may write a bounded 1..=SLOTS override; an
/// invalid write leaves the previous value intact. Compute batching is separate.
static M3_RENDER_BATCH_OVERRIDE: AtomicU64 = AtomicU64::new(0);
m3_param!("m3_render_batch_override", M3_RENDER_BATCH_OVERRIDE,
    parse_render_batch_override, 0o644, Some(get_atomic_param));

fn parse_render_stamp_coalesce(text:&str)->Option<u64> {
    parse_u64(text).filter(|value| *value<=1)
}

/// Root-only T6030 render stamp-flush coalescing; zero retains per-pass flushes.
/// Each publication snapshots the setting before encoding any pass.
static M3_RENDER_STAMP_COALESCE: AtomicU64 = AtomicU64::new(0);
m3_param!("m3_render_stamp_coalesce", M3_RENDER_STAMP_COALESCE,
    parse_render_stamp_coalesce, 0o644, Some(get_atomic_param));
pub(crate) fn render_stamp_coalesce()->bool {
    M3_RENDER_STAMP_COALESCE.load(Ordering::Relaxed)!=0
}

/// T6030 cross-engine publications retain GPU ordering with firmware barriers.
/// Zero preserves the existing host retirement between engine groups.
static M3_MIXED_BATCH: AtomicU64 = AtomicU64::new(0);
m3_param!("m3_mixed_batch", M3_MIXED_BATCH,
    parse_render_stamp_coalesce, 0o644, Some(get_atomic_param));
pub(crate) fn mixed_batch()->bool {
    M3_MIXED_BATCH.load(Ordering::Relaxed)!=0
}

/// Boot-only T6030 power-state sampling while GPU stamps are outstanding.
/// Zero preserves the completion path without additional register reads.
static M3_ACTIVE_POWER: AtomicU64 = AtomicU64::new(0);
m3_param!("m3_active_power", M3_ACTIVE_POWER, parse_render_stamp_coalesce);
pub(crate) fn active_power()->bool {
    M3_ACTIVE_POWER.load(Ordering::Relaxed)==1
}

/// Boot-only T6030 Globals debug-word experiment. Zero preserves the runtime word.
static M3_GLOBALS_DEBUG_OFF: AtomicU64 = AtomicU64::new(0);
m3_param!("m3_globals_debug_off",M3_GLOBALS_DEBUG_OFF,parse_render_stamp_coalesce);
pub(crate) fn globals_debug_off()->bool {M3_GLOBALS_DEBUG_OFF.load(Ordering::Relaxed)==1}

/// T6030-only request while work is pending or active. Zero leaves firmware
/// performance selection unchanged; three requests the top published state.
static M3_CONSISTENT_PERF: AtomicU64 = AtomicU64::new(0);
fn parse_consistent_perf(text: &str)->Option<u64> {
    parse_u64(text).filter(|v| matches!(*v,0|3))
}
m3_param!("m3_consistent_perf",M3_CONSISTENT_PERF,parse_consistent_perf,0o644,Some(get_atomic_param));
pub(crate) fn consistent_perf()->u64 {M3_CONSISTENT_PERF.load(Ordering::Relaxed)}

/// One preserves internal render measurements. Application timestamp requests
/// remain recorded when zero omits measurements on T6030.
static M3_RENDER_INTERNAL_TIMESTAMPS: AtomicU64 = AtomicU64::new(1);
m3_param!("m3_render_internal_timestamps", M3_RENDER_INTERNAL_TIMESTAMPS,
    parse_render_stamp_coalesce, 0o644, Some(get_atomic_param));
pub(crate) fn render_internal_timestamps()->bool {
    M3_RENDER_INTERNAL_TIMESTAMPS.load(Ordering::Relaxed)!=0
}

/// Snapshot once per batch. A live change affects later publications, never the
/// storage, ordering or retirement checks of work already on the firmware queues.
pub(crate) fn render_batch_size() -> usize {
    let override_size = M3_RENDER_BATCH_OVERRIDE.load(Ordering::Relaxed);
    let size = if override_size == 0 {
        *crate::module_parameters::m3_render_batch_size.value() as usize
    } else {
        override_size as usize
    };
    size.clamp(1, crate::m3_pass_layout::SLOTS)
}

fn parse_compute_batch_override(text: &str) -> Option<u64> {
    parse_u64(text).filter(|value| *value <= crate::m3_compute_storage::SLOTS as u64)
}

/// Independently tune the existing adjacent-compute path. Zero preserves the
/// boot parameter; invalid writes do not replace the current value. Storage and
/// engine/VM retirement boundaries are unchanged.
static M3_COMPUTE_BATCH_OVERRIDE: AtomicU64 = AtomicU64::new(0);
m3_param!("m3_compute_batch_override", M3_COMPUTE_BATCH_OVERRIDE,
    parse_compute_batch_override, 0o644, Some(get_atomic_param));

/// Snapshot once per packet, so a live write never changes an in-flight batch.
pub(crate) fn compute_batch_size() -> usize {
    let override_size = M3_COMPUTE_BATCH_OVERRIDE.load(Ordering::Relaxed);
    let size = if override_size == 0 {
        *crate::module_parameters::m3_compute_batch_size.value() as usize
    } else {
        override_size as usize
    };
    size.clamp(1, crate::m3_compute_storage::SLOTS)
}

/// `asahi.g15_debug`: M3 runtime debug bits, see [`G15Debug`].
static G15_DEBUG: AtomicU64 = AtomicU64::new(0);
m3_param!("g15_debug", G15_DEBUG, parse_u64);

/// Bits of `asahi.g15_debug`, the debug switches of the M3 runtime backend.
///
/// The bit numbers are unchanged from their earlier `asahi.debug_flags` positions. The other
/// bits of the mask belonged to the G15 GpuManager bring-up backend and are ignored.
#[derive(Copy, Clone)]
#[repr(u32)]
pub(crate) enum G15Debug {
    /// Stop the probe right before the GPU coprocessor would be started. The firmware never
    /// runs; everything before that point (admission, firmware identity, InitData generation,
    /// register identity checks) still does.
    StopBeforeAsc = 53,
    /// When a job fails and the firmware reports that it has halted, clear the halted flag and
    /// set resume, then log for about half a second what the firmware, the engines and the
    /// failed job do. The GPU stays marked failed either way.
    M3ResumeAfterFault = 56,
}

/// Returns whether a `asahi.g15_debug` bit is set.
pub(crate) fn g15_debug(bit: G15Debug) -> bool {
    G15_DEBUG.load(Ordering::Relaxed) & (1u64 << (bit as u32)) != 0
}

/// `asahi.m3_initdata` values.
const INITDATA_AUTO: u64 = 0;
const INITDATA_ADT: u64 = 2;

fn parse_initdata(text: &str) -> Option<u64> {
    match text.trim() {
        "auto" | "0" => Some(INITDATA_AUTO),
        "adt" | "2" => Some(INITDATA_ADT),
        _ => None,
    }
}

/// `asahi.m3_initdata=auto|adt`: the runtime backend's InitData contents are generated from the
/// device tree either way; the value is logged.
static M3_INITDATA: AtomicU64 = AtomicU64::new(INITDATA_AUTO);
m3_param!("m3_initdata", M3_INITDATA, parse_initdata);

/// Requested source of the runtime backend's InitData contents.
#[derive(Copy, Clone, PartialEq, Eq)]
pub(crate) enum InitDataParam {
    /// Not given: contents generated from the device tree.
    Auto,
    /// Contents generated from the device tree.
    Adt,
}

/// The `asahi.m3_initdata` setting.
pub(crate) fn initdata_param() -> InitDataParam {
    match M3_INITDATA.load(Ordering::Relaxed) {
        INITDATA_ADT => InitDataParam::Adt,
        _ => InitDataParam::Auto,
    }
}

/// Default of `asahi.m3_max_pstate` for device-tree InitData: the fifth performance state.
pub(crate) const MAX_PSTATE_DEFAULT: u64 = 5;
/// A performance-state parameter that is not given.
const PSTATE_UNSET: u64 = u64::MAX;

/// `asahi.m3_max_pstate`: the highest GPU performance state the driver lets the firmware use,
/// as an index into the voltage-sorted performance-state table (1 = lowest). It is clamped to
/// the device tree's highest performance state. Unset means [`MAX_PSTATE_DEFAULT`], except
/// while the runtime's thermal limit is on or holding, which keeps the whole table.
static M3_MAX_PSTATE: AtomicU64 = AtomicU64::new(PSTATE_UNSET);
m3_param!("m3_max_pstate", M3_MAX_PSTATE, parse_u64);

/// `asahi.m3_boot_pstate`: the performance state the runtime backend requests at boot, clamped
/// to the cap. Unset means the cap.
static M3_BOOT_PSTATE: AtomicU64 = AtomicU64::new(PSTATE_UNSET);
m3_param!("m3_boot_pstate", M3_BOOT_PSTATE, parse_u64);

/// The `asahi.m3_max_pstate` setting, if given, before clamping to the performance-state table.
pub(crate) fn max_pstate_param() -> Option<u64> {
    Some(M3_MAX_PSTATE.load(Ordering::Relaxed)).filter(|v| *v != PSTATE_UNSET)
}

/// The `asahi.m3_boot_pstate` setting, if given, before clamping.
pub(crate) fn boot_pstate_param() -> Option<u64> {
    Some(M3_BOOT_PSTATE.load(Ordering::Relaxed)).filter(|v| *v != PSTATE_UNSET)
}

/// `asahi.m3_timeout_nohang`: 1 (default) makes a scheduler timeout of a runtime-backend job
/// report "no hang", because the runtime completes every packet itself and enforces its own
/// per-batch completion limit. 0 keeps the previous handling: mark the GPU failed, fail the
/// packet again and report the device as gone.
static M3_TIMEOUT_NOHANG: AtomicU64 = AtomicU64::new(1);
m3_param!("m3_timeout_nohang", M3_TIMEOUT_NOHANG, parse_u64);

/// Whether `asahi.m3_timeout_nohang` is set (the default).
pub(crate) fn timeout_nohang() -> bool {
    M3_TIMEOUT_NOHANG.load(Ordering::Relaxed) != 0
}

/// `asahi.m3_unlocked_wait`: 1 (default) releases the runtime lock while the runtime backend
/// waits for a batch to complete, so VM creation, timestamp mapping and the event worker are
/// not held up by a running job. 0 holds the lock from preparation to retirement, as before.
static M3_UNLOCKED_WAIT: AtomicU64 = AtomicU64::new(1);
m3_param!("m3_unlocked_wait", M3_UNLOCKED_WAIT, parse_u64);

/// Whether `asahi.m3_unlocked_wait` is set (the default).
pub(crate) fn unlocked_wait() -> bool {
    M3_UNLOCKED_WAIT.load(Ordering::Relaxed) != 0
}

/// `asahi.m3_ut_engagement=0|1`: Globals +0x7d0, the gate of the firmware's frequency-feedback
/// cap. Unset: the SoC's default ([`crate::m3_soc::Soc::ut_engagement`]).
static M3_UT_ENGAGEMENT: AtomicU64 = AtomicU64::new(UT_ENGAGEMENT_UNSET);
const UT_ENGAGEMENT_UNSET: u64 = u64::MAX;
fn parse_ut_engagement(text: &str) -> Option<u64> {
    parse_u64(text).filter(|v| *v <= 1)
}
m3_param!("m3_ut_engagement", M3_UT_ENGAGEMENT, parse_ut_engagement);

/// The frequency-feedback gate for `soc`, and whether `asahi.m3_ut_engagement` set it.
pub(crate) fn ut_engagement(soc: &crate::m3_soc::Soc) -> (u32, bool) {
    match M3_UT_ENGAGEMENT.load(Ordering::Relaxed) {
        UT_ENGAGEMENT_UNSET => (soc.ut_engagement, false),
        v => (v as u32, true),
    }
}

/// `asahi.m3_pipeline_depth`: writable at runtime. How many render batches may be in flight on
/// the firmware's render queues. Zero selects the SoC default: 2 on the explicitly enabled
/// experimental T8122 runtime, 1 on T6030. Depth 1 publishes only after the previous batch has
/// fully retired. Larger values publish the next render batch of the same VM while earlier
/// ones still run, in disjoint pass slots, ordered by the existing cross-pass barriers.
static M3_PIPELINE_DEPTH: AtomicU64 = AtomicU64::new(0);
m3_param!("m3_pipeline_depth", M3_PIPELINE_DEPTH, parse_pipeline_depth, 0o644, Some(get_atomic_param));

fn parse_pipeline_depth(text: &str) -> Option<u64> {
    parse_u64(text).filter(|v| *v<=crate::m3_pass_layout::SLOTS as u64)
}

/// Effective depth; changing it never releases already published pass slots.
pub(crate) fn pipeline_depth(soc: &crate::m3_soc::Soc) -> usize {
    match M3_PIPELINE_DEPTH.load(Ordering::Relaxed) {
        0 => crate::m3_runtime::policy::default_depth(crate::t8122_start::is_t8122(soc)),
        v => v.clamp(1,crate::m3_pass_layout::SLOTS as u64) as usize,
    }
}

/// `asahi.m3_render_batch_budget_us`: writable at runtime. A render batch takes in later packets
/// of its VM only while the VM's measured GPU time per pass, times the passes already in the
/// batch, stays below this budget (4000 µs on experimental T8122, disabled on Pro).
/// Coalescing a client's heavy frames
/// otherwise makes every other client's work, the compositor's included, wait for all of them.
/// 0 disables the budget; u64::MAX restores the SoC default.
static M3_RENDER_BATCH_BUDGET_US: AtomicU64 = AtomicU64::new(u64::MAX);
m3_param!("m3_render_batch_budget_us", M3_RENDER_BATCH_BUDGET_US, parse_u64, 0o644, Some(get_atomic_param));

/// The render batch GPU-time budget in ns, or `None` without one.
pub(crate) fn render_batch_budget_ns(soc: &crate::m3_soc::Soc) -> Option<u64> {
    let value=M3_RENDER_BATCH_BUDGET_US.load(Ordering::Relaxed);
    let us=if value==u64::MAX {crate::m3_runtime::policy::default_budget_us(crate::t8122_start::is_t8122(soc))} else {value};
    match us {
        0 => None,
        us => Some(us.saturating_mul(1000)),
    }
}

/// Bits of `asahi.m3_retire_mmio`.
pub(crate) const RETIRE_MMIO_BUSY: u64 = 1 << 0;
pub(crate) const RETIRE_MMIO_FAULTS: u64 = 1 << 1;
pub(crate) const RETIRE_MMIO_PSTATE: u64 = 1 << 2;
const RETIRE_MMIO_UNSET: u64 = u64::MAX;

fn parse_retire_mmio(text: &str) -> Option<u64> {
    parse_u64(text).filter(|v| *v <= 7)
}

/// `asahi.m3_retire_mmio`, writable at runtime: the GPU registers the host reads after every
/// job before retiring it. Bit 0 reads the engine-busy registers (SGX+0xc020/0xc120), bit 1
/// selects and reads each cluster's fault bank (0xd800/0xd8c0), bit 2 reads the performance
/// state (0xe01000). Unset: the SoC's default ([`crate::m3_soc::Soc::retire_mmio`]).
static M3_RETIRE_MMIO: AtomicU64 = AtomicU64::new(RETIRE_MMIO_UNSET);
m3_param!("m3_retire_mmio", M3_RETIRE_MMIO, parse_retire_mmio, 0o644, Some(get_atomic_param));

/// The `asahi.m3_retire_mmio` mask for `soc`.
pub(crate) fn retire_mmio(soc: &crate::m3_soc::Soc) -> u64 {
    match M3_RETIRE_MMIO.load(Ordering::Relaxed) {
        RETIRE_MMIO_UNSET => soc.retire_mmio,
        mask => mask,
    }
}

/// `asahi.m3_thermal` values.
const THERMAL_OFF: u64 = 0;
const THERMAL_HOLD: u64 = 1;
const THERMAL_ON: u64 = 2;
const THERMAL_UNSET: u64 = u64::MAX;

fn parse_thermal(text: &str) -> Option<u64> {
    match text.trim() {
        "off" | "0" => Some(THERMAL_OFF),
        "hold" => Some(THERMAL_HOLD),
        "on" | "1" => Some(THERMAL_ON),
        _ => None,
    }
}

static M3_THERMAL: AtomicU64 = AtomicU64::new(THERMAL_UNSET);
m3_param!("m3_thermal", M3_THERMAL, parse_thermal);

/// How the runtime backend limits the GPU performance state by temperature.
#[derive(Copy, Clone, PartialEq, Eq, Debug)]
pub(crate) enum ThermalMode {
    /// Today's fixed cap: the firmware is given the cap as its highest state, and nothing
    /// changes it at runtime.
    Off,
    /// The firmware is given the whole table as its highest state, and the runtime cap words
    /// hold the fixed cap for the whole boot. The same performance as `Off`, if the firmware
    /// follows the runtime cap: this checks that it does.
    Hold,
    /// As `Hold`, and the runtime cap follows the SoC die temperature, up to the whole table
    /// while the die is cool.
    On,
}

/// The `asahi.m3_thermal` setting, if given.
pub(crate) fn thermal_param() -> Option<ThermalMode> {
    match M3_THERMAL.load(Ordering::Relaxed) {
        THERMAL_OFF => Some(ThermalMode::Off),
        THERMAL_HOLD => Some(ThermalMode::Hold),
        THERMAL_ON => Some(ThermalMode::On),
        _ => None,
    }
}

/// Default and bounds of `asahi.m3_thermal_hot`, in degrees Celsius.
pub(crate) const THERMAL_HOT_DEFAULT: u64 = 80;
pub(crate) const THERMAL_HOT_MIN: u64 = 40;
pub(crate) const THERMAL_HOT_MAX: u64 = 85;

/// `asahi.m3_thermal_hot`: the SoC die temperature (degrees Celsius) at or above which the thermal
/// limit lowers the runtime cap, clamped to [`THERMAL_HOT_MIN`]..=[`THERMAL_HOT_MAX`].
static M3_THERMAL_HOT: AtomicU64 = AtomicU64::new(THERMAL_HOT_DEFAULT);
m3_param!("m3_thermal_hot", M3_THERMAL_HOT, parse_u64);

/// The `asahi.m3_thermal_hot` setting and whether it was clamped.
pub(crate) fn thermal_hot_param() -> (u64, bool) {
    let value = M3_THERMAL_HOT.load(Ordering::Relaxed);
    let clamped = value.clamp(THERMAL_HOT_MIN, THERMAL_HOT_MAX);
    (clamped, clamped != value)
}

/// Whether to start the runtime backend on this T8122 board, and say why not.
///
/// The runtime starts only on the boards the T8122 table allows (the M3 MacBook Airs): it has
/// not run on any T8122, so nothing forces it onto another board.
pub(crate) fn t8122_backend(pdev: &platform::Device<Core>) -> bool {
    if crate::m3_board::runtime_validated_board(&crate::m3_soc::T8122) {
        return true;
    }
    dev_info!(
        pdev.as_ref(),
        "M3: no GPU backend on this T8122 board: the runtime backend may start only on the M3 MacBook Airs (J613, J615)\n"
    );
    false
}

fn parse_t8122_number(text: &str) -> Option<u64> {
    Some(crate::t8122_knobs::parse_number(text))
}

fn parse_t8122_fender(text: &str) -> Option<u64> {
    Some(crate::t8122_knobs::parse_fender(text))
}

fn parse_t8122_clkgen(text: &str) -> Option<u64> {
    Some(crate::t8122_knobs::parse_clkgen(text))
}

fn parse_t8122_sgx_setup(text: &str) -> Option<u64> {
    Some(crate::t8122_knobs::parse_sgx_setup(text))
}

// The T8122 start experiment (`t8122_start`). Each parameter is stored as given, unset or with a
// value its parser does not accept (`t8122_knobs`), so that the experiment can refuse rather
// than fall back to a default.
use crate::t8122_knobs::UNSET as T8122_UNSET;

/// `asahi.t8122_start=1`: start the M3 runtime on a T8122 with the start experiment's values.
/// Only with a boot loader that handed the GPU over.
static T8122_START: AtomicU64 = AtomicU64::new(T8122_UNSET);
m3_param!("t8122_start", T8122_START, parse_t8122_number);
/// `asahi.t8122_initdata_version=<u64>`: the InitData version given to the T8122 firmware.
static T8122_INITDATA_VERSION: AtomicU64 = AtomicU64::new(T8122_UNSET);
m3_param!("t8122_initdata_version", T8122_INITDATA_VERSION, parse_t8122_number);
/// `asahi.t8122_fender=0x104000|0x12c000` (`rule`, `adt`): the Fender window size.
static T8122_FENDER: AtomicU64 = AtomicU64::new(T8122_UNSET);
m3_param!("t8122_fender", T8122_FENDER, parse_t8122_fender);
/// `asahi.t8122_clkgen=e1c|e5c|none`: the GPU clock-generator IO mapping.
static T8122_CLKGEN: AtomicU64 = AtomicU64::new(T8122_UNSET);
m3_param!("t8122_clkgen", T8122_CLKGEN, parse_t8122_clkgen);
/// `asahi.t8122_sgx_setup=none|t6030`: the SGX write made before the firmware starts.
static T8122_SGX_SETUP: AtomicU64 = AtomicU64::new(T8122_UNSET);
m3_param!("t8122_sgx_setup", T8122_SGX_SETUP, parse_t8122_sgx_setup);
/// `asahi.t8122_unit_mask_a=<u64>`: HwDataB unit mask A (+0x17c0).
static T8122_UNIT_MASK_A: AtomicU64 = AtomicU64::new(T8122_UNSET);
m3_param!("t8122_unit_mask_a", T8122_UNIT_MASK_A, parse_t8122_number);
/// `asahi.t8122_unit_mask_b=<u32>`: HwDataB unit mask B (+0x17c8).
static T8122_UNIT_MASK_B: AtomicU64 = AtomicU64::new(T8122_UNSET);
m3_param!("t8122_unit_mask_b", T8122_UNIT_MASK_B, parse_t8122_number);
/// `asahi.t8122_pstate_cap=N`: the highest performance state the T8122 firmware may use.
static T8122_PSTATE_CAP: AtomicU64 = AtomicU64::new(T8122_UNSET);
m3_param!("t8122_pstate_cap", T8122_PSTATE_CAP, parse_t8122_number);

/// `asahi.t8122_hwdata_418c=0|1`: on a T8122 with the G15 14.8.3 firmware, 1 (the default)
/// writes the 20 words at HwDataA +0x418c ([`crate::t8122_hwdata`]); 0 leaves the
/// range zero, for an A/B test. Readable in sysfs.
static T8122_HWDATA_418C: AtomicU64 = AtomicU64::new(crate::t8122_hwdata::DEFAULT);
m3_param!("t8122_hwdata_418c", T8122_HWDATA_418C, crate::t8122_hwdata::parse, 0o444,
    Some(get_atomic_param));

/// Whether `asahi.t8122_hwdata_418c` asks for the HwDataA +0x418c words (the default).
pub(crate) fn t8122_hwdata_418c() -> bool {
    crate::t8122_hwdata::enabled(T8122_HWDATA_418C.load(Ordering::Relaxed))
}

/// The T8122 start-experiment parameters of this boot, as given.
pub(crate) fn t8122_params() -> crate::t8122_knobs::Raw {
    let get = |p: &AtomicU64| p.load(Ordering::Relaxed);
    crate::t8122_knobs::Raw {
        start: get(&T8122_START),
        initdata_version: get(&T8122_INITDATA_VERSION),
        fender: get(&T8122_FENDER),
        clkgen: get(&T8122_CLKGEN),
        sgx_setup: get(&T8122_SGX_SETUP),
        unit_mask_a: get(&T8122_UNIT_MASK_A),
        unit_mask_b: get(&T8122_UNIT_MASK_B),
        pstate_cap: get(&T8122_PSTATE_CAP),
    }
}
