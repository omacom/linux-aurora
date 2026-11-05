// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! GPU power budget governor for M5 (J714).
//!
//! The firmware limits the GPU to the host power budget word in Region C within about a second
//! (the calibrated maximum is written there at initialization). Left at that maximum under
//! sustained load, the SoC hotspot temperature computed by the SMC (about 6 C above the hottest
//! GPU die sensor while the GPU dominates) spends time above roughly 97 C, and the firmware's
//! emergency path then pins the GPU to its base performance state for 20-40 s, repeatedly. This
//! governor reads the hotspot temperature through the thermal framework once per second while the
//! GPU is busy and, once it reaches the engage temperature, lowers the budget with a PI controller
//! (plus a feed-forward on the temperature's rise) so it settles at the target, below that region.
//!
//! The controller works below the GPU's actual draw (the calibrated maximum is far above it). It
//! keeps the integral it converged to across releases, so after a release or a firmware clamp it
//! resumes from that budget; after a long period without control the memory decays to the
//! cold-start budget, never to the ceiling. It holds while the firmware's emergency clamp is
//! active and never releases then. The default target keeps the hotspot below the level at which
//! the firmware clamps even with a heat-soaked chassis. As a safety net, each emergency clamp the
//! governor sees lowers its effective target by 1 C (at most 6 C, not below 84 C), and every 10
//! minutes without one raises it again by 1 C. It publishes the budget like the effort input, only when it changes, and
//! restores the calibrated maximum on release, when the GPU goes idle, when no sensor can be read
//! and at shutdown. Below the engage temperature nothing is written.

use core::ffi::{c_char, c_int, c_uint, c_void, CStr};
use core::sync::atomic::{AtomicI32, Ordering};
use kernel::{bindings, prelude::*};

// fans.c: the SMC hwmon cell's fan requests and thermal zone reads, which have stubs where
// their drivers are not built.
extern "C" {
    fn asahi_zone_temp(name: *const c_char, temp: *mut c_int) -> c_int;
    fn asahi_fans_request(dev: *mut c_void, permille: c_uint, min_step_rpm: c_uint) -> c_int;
}

/// SoC hotspot temperature computed by the SMC (registered as a thermal zone by the SMC driver):
/// the input of the firmware's emergency clamp.
const J714_HOTSPOT_ZONE: &CStr = c"smc-TCMb";
/// The hottest of the 42 SMC GPU die sensors of J714 near the limit, and their offset to the
/// hotspot while the GPU dominates: the fallback input when the hotspot cannot be read.
const J714_GPU_DIE_ZONES: [&CStr; 8] = [
    c"smc-Tg08", c"smc-Tg12", c"smc-Tg0O", c"smc-Tg0X", c"smc-Tg29", c"smc-Tg2P", c"smc-Tg1U",
    c"smc-Tg43",
];
const GPU_DIE_TO_HOTSPOT_MC: i32 = 6000;
/// Where the SMC driver discovers the temperatures and the device tree names the hotspot key
/// (TCMb) as the SoC die zone's only key (J775D), that zone reads the hotspot.
const SOC_DIE_ZONE: &CStr = c"macsmc_soc_die";

/// Region C word holding the host GPU power budget (mW).
pub(crate) const REGION_C_POWER_BUDGET: usize = 0x88;
/// Shared power-policy word holding the GPU's current power draw (mW), written by the firmware.
pub(crate) const POLICY_GPU_POWER: usize = 0xad4;
/// Shared power-policy words of the firmware's primary performance limiter: the mask of active
/// limit reasons and the limit. The emergency reason with the limit at its floor is the clamp.
pub(crate) const POLICY_LIMITER_MASK: usize = 0x44;
pub(crate) const POLICY_LIMITER_LIMIT: usize = 0x48;
const LIMITER_EMERGENCY: u32 = 0x4;
const LIMITER_FLOOR: u32 = 100;
/// Control period while the GPU is busy.
pub(crate) const PERIOD_MS: u32 = 1000;
/// Restore the maximum after this long without a submission.
const IDLE_RESTORE_NS: u64 = 2_000_000_000;
/// Engage when the hotspot reaches this; release below it minus the hysteresis once the
/// controller has returned to its ceiling.
const ENGAGE_BELOW_TARGET_MC: i32 = 4000;
const RELEASE_HYSTERESIS_MC: i32 = 3000;
/// Lowest budget the governor sets (mW).
const FLOOR_MW: i64 = 10_000;
/// Lowest ceiling of the controller (mW): the GPU's draw at engage, but at least this.
const CEILING_MIN_MW: i64 = 40_000;
/// Proportional gain (mW per C) and integral gain (mW per C per second). Power follows the
/// budget within a second; the die settles over 30-60 s at ~1.2-1.5 C per W.
const KP_MW_PER_C: i64 = 1500;
const KI_MW_PER_C_S: i64 = 50;
/// Feed-forward on the hotspot's rise (mW per C/s), applied only while it rises, to curb the
/// overshoot when the GPU restarts from cool.
const KF_MW_PER_C_PER_S: i64 = 2000;
/// Without a converged integral, start at this fraction of the ceiling: the budget that holds
/// the target is about half the GPU's unconstrained draw.
const INITIAL_NUM: i64 = 3;
const INITIAL_DEN: i64 = 5;
/// The converged integral is kept for this long after the controller was last active, then
/// decays linearly to the cold-start integral over the decay time.
const MEMORY_HOLD_NS: u64 = 60_000_000_000;
const MEMORY_DECAY_NS: u64 = 300_000_000_000;
/// Effective target reduction per emergency clamp seen while governing, its limit, the lowest
/// effective target it leads to, and the time without a clamp after which it is reduced by one
/// step again.
const CLAMP_TARGET_STEP_MC: i32 = 1000;
const CLAMP_TARGET_MAX_MC: i32 = 6000;
const CLAMP_TARGET_FLOOR_MC: i32 = 84_000;
const CLAMP_TARGET_RECOVER_NS: u64 = 600_000_000_000;
/// Budget changes smaller than this are not published.
const STEP_MW: i64 = 250;

/// CPU ceiling: one step lowers every cpufreq policy's maximum by this share of its hardware
/// maximum, down to the floor share.
const CPU_STEP_PERCENT: u32 = 4;
const CPU_FLOOR_PERCENT: u32 = 60;
/// Minimum time between CPU ceiling steps; the hotspot and the GPU draw follow within it.
const CPU_DWELL_NS: u64 = 2_000_000_000;
/// The GPU uses its budget when it draws at least this share of it, and leaves headroom below
/// the lower share (it then waits for the CPU).
const GPU_BOUND_PERCENT: i64 = 90;
const GPU_SLACK_PERCENT: i64 = 70;
/// The CPU's own heat: the hotspot this far above the target while the GPU budget is within the
/// slack of its floor.
const CPU_HEAT_MARGIN_MC: i32 = 2000;
const CPU_HEAT_FLOOR_SLACK_MW: i64 = 1000;

fn enabled() -> bool {
    crate::module_parameters::m5_thermal_governor.value() & 1 != 0
}
fn cpu_balance_enabled() -> bool {
    crate::module_parameters::m5_thermal_governor.value() & 4 != 0
}
fn fan_boost_enabled() -> bool {
    crate::module_parameters::m5_thermal_governor.value() & 8 != 0
}
/// The proportional fan curve is on unless bit 4 (16) of m5_thermal_governor opts out.
fn fan_curve_enabled() -> bool {
    crate::module_parameters::m5_thermal_governor.value() & 16 == 0
}
pub(super) fn requested() -> bool {
    enabled()
}
fn target_mc() -> i32 {
    let parameter = crate::module_parameters::m5_thermal_target_mc.value();
    i32::try_from(parameter).unwrap_or(90_000).clamp(70_000, 104_000)
}

/// Whether the firmware's primary limiter words describe the emergency clamp.
pub(crate) fn is_emergency_clamp(mask: u32, limit: u32) -> bool {
    mask & LIMITER_EMERGENCY != 0 && limit <= LIMITER_FLOOR
}

/// Pins the sensor provider and serializes reads with its probe and removal.
/// Its thermal zones are managed by the hwmon device registered during probe;
/// removal releases that device's resources while holding the provider lock.
struct SensorProvider(*mut bindings::device);

impl SensorProvider {
    fn acquire() -> Option<Self> {
        // SAFETY: the bus and name have static lifetimes. The bus iterator
        // returns an owned device reference on success.
        let device = unsafe {
            bindings::bus_find_device(
                &bindings::platform_bus_type,
                core::ptr::null_mut(),
                c"macsmc-hwmon".as_ptr().cast(),
                Some(bindings::device_match_name),
            )
        };
        if device.is_null() {
            return None;
        }
        // SAFETY: the owned reference keeps the device and mutex alive. No
        // GPU driver lock is held during acquisition or sensor reads.
        unsafe { bindings::mutex_lock(&raw mut (*device).mutex) };
        let provider = Self(device);
        // SAFETY: the provider's lock protects binding and unbinding.
        if unsafe { (*device).driver.is_null() } {
            return None;
        }
        Some(provider)
    }
}

impl Drop for SensorProvider {
    fn drop(&mut self) {
        // SAFETY: acquire obtained this reference and locked this mutex on
        // the current thread; the guard never leaves the sensor-read call.
        unsafe {
            bindings::mutex_unlock(&raw mut (*self.0).mutex);
            bindings::put_device(self.0);
        }
    }
}

/// The provider guard prevents its managed zones from disappearing during
/// lookup and read. Do not retain a zone pointer after dropping the guard.
fn zone_temperature_mc(_provider: &SensorProvider, name: &CStr) -> Option<i32> {
    let mut temp: c_int = 0;
    // SAFETY: name is a NUL-terminated string that outlives the call, and temp is a valid
    // destination; the zone is looked up and read under the provider guard.
    (unsafe { asahi_zone_temp(name.as_ptr(), &mut temp) } == 0)
        .then_some(temp)
        .filter(|temp| (0..=125_000).contains(temp))
}

/// SoC hotspot temperature in milli-C (the hottest GPU die sensor plus its offset when the hotspot
/// cannot be read), or None when no sensor can be read. Called without driver locks held: zone
/// lookups and sensor reads may sleep.
pub(crate) fn hotspot_temperature_mc() -> Option<i32> {
    let provider = SensorProvider::acquire()?;
    zone_temperature_mc(&provider, J714_HOTSPOT_ZONE)
        .or_else(|| zone_temperature_mc(&provider, SOC_DIE_ZONE))
        .or_else(|| {
            J714_GPU_DIE_ZONES.iter().filter_map(|name| zone_temperature_mc(&provider, name)).max()
                .map(|temp| temp + GPU_DIE_TO_HOTSPOT_MC)
        })
}

/// Keeps the fans boosted at least this long once boosted, so a short release or idle period
/// (a loading screen) does not cycle them.
const FAN_HOLD_NS: u64 = 30_000_000_000;

/// Proportional fan curve on the hotspot while the GPU is busy: on J714 the SMC's automatic mode
/// keeps the fans near their minimum under sustained GPU load on Linux. From the board's curve
/// start ([`super::fw::m5::initdata::Board::fan_curve_start_mc`]) to the governor target the fans
/// ramp from the minimum to two thirds of the range, and on to FAN_CURVE_TOP_PERMILLE at
/// FAN_CURVE_TOP_MC.
static FAN_CURVE_START_MC: AtomicI32 =
    AtomicI32::new(super::fw::m5::initdata::J714.fan_curve_start_mc);
const FAN_CURVE_TOP_MC: i32 = 98_000;
const FAN_CURVE_TARGET_PERMILLE: i32 = 660;
const FAN_CURVE_TOP_PERMILLE: i32 = 800;
/// Below this the curve releases the fans to automatic mode (after FAN_HOLD_NS).
const FAN_CURVE_RELEASE_MC: i32 = 82_000;
/// A new target is written only when it moves at least this far.
const FAN_CURVE_STEP_RPM: u32 = 150;

/// Selects the board's thermal settings at probe; J714's until then.
pub(crate) fn select_board(board: &super::fw::m5::initdata::Board) {
    FAN_CURVE_START_MC.store(board.fan_curve_start_mc, Ordering::Relaxed);
}

/// The permille of the fan range the curve asks for at hotspot `temp_mc`.
fn fan_curve_permille(temp_mc: i32, target_mc: i32) -> i32 {
    let start_mc = FAN_CURVE_START_MC.load(Ordering::Relaxed);
    let target_mc = target_mc.max(start_mc + 1000);
    if temp_mc <= start_mc {
        0
    } else if temp_mc <= target_mc {
        (temp_mc - start_mc) * FAN_CURVE_TARGET_PERMILLE / (target_mc - start_mc)
    } else {
        let top = FAN_CURVE_TOP_MC.max(target_mc + 1000);
        (FAN_CURVE_TARGET_PERMILLE + (temp_mc.min(top) - target_mc)
            * (FAN_CURVE_TOP_PERMILLE - FAN_CURVE_TARGET_PERMILLE) / (top - target_mc))
            .min(FAN_CURVE_TOP_PERMILLE)
    }
}

/// Sets every fan the SMC hwmon cell lets a kernel policy set to `permille` of its
/// minimum-to-maximum range, or hands them back to the SMC for 0, under the provider lock (which
/// serializes with its unbinding). A fan whose target is within `min_step_rpm` of the new one
/// keeps it. Returns the first fan's target rpm, 0 once handed back.
fn request_fans(permille: u32, min_step_rpm: u32) -> Option<u32> {
    let provider = SensorProvider::acquire()?;
    // SAFETY: the provider lock keeps the device bound, as the request requires.
    let rpm = unsafe { asahi_fans_request(provider.0.cast(), permille, min_step_rpm) };
    u32::try_from(rpm).ok()
}

/// Fan boost: while the governor limits the GPU, every fan runs at its hardware maximum instead of
/// the SMC's automatic curve, which settles near two thirds of the maximum at the governor's
/// target.
/// Released (automatic mode) when the governor releases or the GPU goes idle, after a hold.
pub(crate) struct FanBoost {
    boosted: bool,
    since_ns: u64,
    /// The fan target the curve last wrote (rpm), 0 while the fans are automatic.
    curve_rpm: u32,
}

impl FanBoost {
    const fn new() -> Self {
        Self { boosted: false, since_ns: 0, curve_rpm: 0 }
    }

    /// Puts every fan in manual mode at `permille` of its minimum-to-maximum range, or back in
    /// automatic mode for `None`. Returns the first fan's target rpm, 0 for automatic.
    fn set_curve(&mut self, permille: Option<i32>) -> Option<u32> {
        request_fans(permille.map_or(0, |permille| permille.clamp(1, 1000) as u32), 0)
    }

    /// One governor step of the proportional curve: `busy` while the governor runs a busy GPU.
    fn apply_curve(&mut self, busy: bool, temp_mc: Option<i32>, target_mc: i32, now_ns: u64) {
        let permille = temp_mc.filter(|_| busy).map(|t| fan_curve_permille(t, target_mc)).unwrap_or(0);
        if permille > 0 {
            self.since_ns = now_ns;
            // The target rpm moves with the range only when it moved enough.
            if let Some(rpm) = self.set_curve_if_moved(permille) {
                self.curve_rpm = rpm;
            }
        } else if self.curve_rpm != 0
            && (!busy || temp_mc.is_none_or(|t| t < FAN_CURVE_RELEASE_MC))
            && now_ns.saturating_sub(self.since_ns) >= FAN_HOLD_NS
            && self.set_curve(None).is_some()
        {
            self.curve_rpm = 0;
        }
    }

    fn set_curve_if_moved(&mut self, permille: i32) -> Option<u32> {
        // A fan already on the curve keeps its target while the new one is within a step.
        let step = if self.curve_rpm != 0 { FAN_CURVE_STEP_RPM } else { 0 };
        request_fans(permille.clamp(1, 1000) as u32, step)
    }

    fn set(&mut self, boost: bool) -> bool {
        request_fans(if boost { 1000 } else { 0 }, 0).is_some()
    }

    /// Applies one governor step's request. Called from the governor work without driver locks:
    /// SMC accesses sleep.
    fn apply(&mut self, want: bool, temp_mc: Option<i32>, now_ns: u64) {
        if !fan_boost_enabled() {
            if fan_curve_enabled() {
                self.apply_curve(want, temp_mc, target_mc(), now_ns);
            } else if self.curve_rpm != 0 && self.set_curve(None).is_some() {
                self.curve_rpm = 0;
            }
        }
        let want = want && fan_boost_enabled();
        if want {
            self.since_ns = now_ns;
        }
        if want && !self.boosted {
            self.boosted = self.set(true);
        } else if !want && self.boosted
            && (!fan_boost_enabled() || now_ns.saturating_sub(self.since_ns) >= FAN_HOLD_NS)
        {
            if self.set(false) {
                self.boosted = false;
            }
        }
    }

    pub(crate) fn release(&mut self) {
        if self.boosted && self.set(false) {
            self.boosted = false;
        }
        if self.curve_rpm != 0 && self.set_curve(None).is_some() {
            self.curve_rpm = 0;
        }
    }

    pub(crate) fn boosted(&self) -> bool {
        self.boosted || self.curve_rpm != 0
    }
}

impl Drop for FanBoost {
    fn drop(&mut self) {
        self.release();
    }
}

/// The host's thermal actuators besides the GPU budget, applied outside the device lock.
pub(crate) struct Actuators {
    pub(crate) cpu: CpuCeiling,
    pub(crate) fans: FanBoost,
}

impl Actuators {
    pub(crate) const fn new() -> Self {
        Self { cpu: CpuCeiling::new(), fans: FanBoost::new() }
    }

    pub(crate) fn apply(&mut self, advice: CpuAdvice, fans: bool, temp_mc: Option<i32>, now_ns: u64) {
        self.cpu.apply(advice, now_ns);
        self.fans.apply(fans, temp_mc, now_ns);
    }

    pub(crate) fn release(&mut self) {
        self.cpu.release();
        self.fans.release();
    }
}

/// What the CPU ceiling should do after a governor step.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum CpuAdvice {
    /// Keep the current ceiling.
    Hold,
    /// The GPU draws its whole thermally limited budget: give it the CPU's share of the heat.
    Lower,
    /// The GPU is not limited by its budget (or waits for the CPU): give the CPU a step back.
    Raise,
    /// Not governing: drop the ceiling.
    Release,
}

/// One cpufreq policy's maximum-frequency request.
struct CpuRequest {
    policy: *mut bindings::cpufreq_policy,
    request: KBox<bindings::freq_qos_request>,
    max_khz: u32,
}

// SAFETY: the policy reference and the request are only used from the governor work, under the
// ceiling's lock.
unsafe impl Send for CpuRequest {}

/// Shares the SoC's heat between the CPU and the GPU while the governor limits the GPU: when the
/// GPU is budget-bound it lowers the maximum
/// frequency of every CPU cluster one step at a time, and gives the steps back once the GPU no
/// longer uses its budget (it then waits for the CPU) or is no longer limited. The requests are
/// ordinary frequency QoS maxima, combined with any other limit on the policies.
pub(crate) struct CpuCeiling {
    requests: KVec<CpuRequest>,
    level: u32,
    last_change_ns: u64,
}

impl CpuCeiling {
    pub(crate) const fn new() -> Self {
        Self { requests: KVec::new(), level: 0, last_change_ns: 0 }
    }

    fn max_level() -> u32 {
        (100 - CPU_FLOOR_PERCENT) / CPU_STEP_PERCENT
    }

    fn limit_khz(max_khz: u32, level: u32) -> i32 {
        let share = 100 - level.min(Self::max_level()) * CPU_STEP_PERCENT;
        (u64::from(max_khz) * u64::from(share) / 100).min(i32::MAX as u64) as i32
    }

    /// Takes a request on every policy (each policy once, through its first CPU).
    fn acquire(&mut self) -> bool {
        if !self.requests.is_empty() {
            return true;
        }
        // SAFETY: nr_cpu_ids is set once during early boot.
        let cpus = unsafe { bindings::nr_cpu_ids };
        for cpu in 0..cpus {
            // SAFETY: cpufreq_cpu_get accepts any CPU number and returns a counted reference or
            // null.
            let policy = unsafe { bindings::cpufreq_cpu_get(cpu) };
            if policy.is_null() {
                continue;
            }
            // SAFETY: the reference keeps the policy alive.
            let (first, max_khz) = unsafe { ((*policy).cpu, (*policy).cpuinfo.max_freq) };
            let request = if first == cpu && max_khz != 0 {
                // The zeroed (default) request is the unused state freq_qos_add_request expects.
                KBox::new(bindings::freq_qos_request::default(), GFP_KERNEL).ok()
            } else {
                None
            };
            let Some(mut request) = request.filter(|_| self.requests.reserve(1, GFP_KERNEL).is_ok())
            else {
                // SAFETY: drops the reference taken above.
                unsafe { bindings::cpufreq_cpu_put(policy) };
                continue;
            };
            // SAFETY: the policy outlives the request (the reference is held until removal) and
            // the request is in its unused state, at a stable address in its box.
            let added = unsafe {
                bindings::freq_qos_add_request(
                    &raw mut (*policy).constraints,
                    &mut *request,
                    bindings::freq_qos_req_type_FREQ_QOS_MAX,
                    Self::limit_khz(max_khz, 0),
                )
            };
            if added < 0 {
                // SAFETY: drops the reference taken above.
                unsafe { bindings::cpufreq_cpu_put(policy) };
                continue;
            }
            // Capacity was reserved above, so this cannot fail.
            let _ = self.requests.push(CpuRequest { policy, request, max_khz }, GFP_KERNEL);
        }
        !self.requests.is_empty()
    }

    /// Removes every request and the policy references.
    pub(crate) fn release(&mut self) {
        for mut entry in self.requests.drain_all() {
            // SAFETY: the request was added in acquire and the policy reference is still held.
            unsafe {
                bindings::freq_qos_remove_request(&mut *entry.request);
                bindings::cpufreq_cpu_put(entry.policy);
            }
        }
        self.level = 0;
    }

    fn publish(&mut self) {
        for entry in self.requests.iter_mut() {
            let limit = Self::limit_khz(entry.max_khz, self.level);
            // SAFETY: the request was added in acquire and stays valid until release.
            unsafe { bindings::freq_qos_update_request(&mut *entry.request, limit) };
        }
    }

    /// Applies one governor step's advice. Called from the governor work without driver locks:
    /// frequency QoS updates may sleep.
    pub(crate) fn apply(&mut self, advice: CpuAdvice, now_ns: u64) {
        if !cpu_balance_enabled() || advice == CpuAdvice::Release {
            if !self.requests.is_empty() {
                self.release();
            }
            return;
        }
        if now_ns.saturating_sub(self.last_change_ns) < CPU_DWELL_NS {
            return;
        }
        match advice {
            CpuAdvice::Lower if self.level < Self::max_level() => {
                if !self.acquire() {
                    return;
                }
                self.level += 1;
            }
            CpuAdvice::Raise if self.level > 0 => {
                self.level -= 1;
                if self.level == 0 {
                    self.release();
                    self.last_change_ns = now_ns;
                    return;
                }
            }
            _ => return,
        }
        self.last_change_ns = now_ns;
        self.publish();
    }
}

impl Drop for CpuCeiling {
    fn drop(&mut self) {
        self.release();
    }
}

pub(crate) struct Governor {
    maximum_mw: u32,
    budget_mw: u32,
    integral_mw: i64,
    /// Upper limit of the controller while active: the GPU's draw at engage.
    ceiling_mw: i64,
    /// Integral the controller last converged to (output inside its range), kept across releases.
    converged_mw: i64,
    active: bool,
    last_tick_ns: u64,
    last_activity_ns: u64,
    /// Last tick at which the controller was active, for the memory decay.
    last_active_ns: u64,
    last_temp_mc: Option<i32>,
    /// Effective target reduction learnt from emergency clamps, and when it last changed.
    target_offset_mc: i32,
    target_offset_ns: u64,
    was_clamped: bool,
}

impl Governor {
    pub(crate) const fn new() -> Self {
        Self { maximum_mw: 0, budget_mw: 0, integral_mw: 0, ceiling_mw: 0, converged_mw: 0, active: false,
            last_tick_ns: 0, last_activity_ns: 0, last_active_ns: 0, last_temp_mc: None, target_offset_mc: 0,
            target_offset_ns: 0, was_clamped: false }
    }

    /// The firmware accepted a submission; returns whether the control loop should run.
    pub(crate) fn note_activity(&mut self, now_ns: u64) -> bool {
        self.last_activity_ns = now_ns;
        requested() || self.budget_mw != self.maximum_mw
    }

    /// Learn the calibrated maximum from the budget word before the first change.
    pub(crate) fn set_maximum(&mut self, current_mw: u32) {
        if self.maximum_mw == 0 && i64::from(current_mw) >= FLOOR_MW {
            self.maximum_mw = current_mw;
            self.budget_mw = current_mw;
        }
    }

    /// The budget to publish while stopping, if it differs from the maximum.
    pub(crate) fn restore(&mut self) -> Option<u32> {
        self.active = false;
        self.integral_mw = 0;
        self.converged_mw = 0;
        self.target_offset_mc = 0;
        if self.maximum_mw == 0 || self.budget_mw == self.maximum_mw {
            return None;
        }
        self.budget_mw = self.maximum_mw;
        Some(self.budget_mw)
    }

    /// Integral to engage with: the converged one while it is fresh, decaying to the cold-start
    /// integral (a fraction of the ceiling) after a long period without control.
    fn resume_integral(&self, now_ns: u64) -> i64 {
        let initial = self.ceiling_mw * INITIAL_NUM / INITIAL_DEN;
        if self.converged_mw == 0 {
            return initial;
        }
        let kept = self.converged_mw.min(self.ceiling_mw);
        let cool_ns = now_ns.saturating_sub(self.last_active_ns);
        if cool_ns <= MEMORY_HOLD_NS {
            return kept;
        }
        let decay_ns = (cool_ns - MEMORY_HOLD_NS).min(MEMORY_DECAY_NS);
        kept + (initial - kept) * (decay_ns / 1_000_000) as i64 / (MEMORY_DECAY_NS / 1_000_000) as i64
    }

    /// Learn from the firmware's emergency clamp: lower the effective target by a step on each
    /// clamp seen while governing, raise it again after a long period without one.
    fn adapt_target(&mut self, clamped: bool, now_ns: u64) {
        if clamped && !self.was_clamped && self.active {
            let limit = (target_mc() - CLAMP_TARGET_FLOOR_MC).clamp(0, CLAMP_TARGET_MAX_MC);
            self.target_offset_mc = (self.target_offset_mc + CLAMP_TARGET_STEP_MC).min(limit);
            self.target_offset_ns = now_ns;
        } else if !clamped && self.target_offset_mc > 0
            && now_ns.saturating_sub(self.target_offset_ns) >= CLAMP_TARGET_RECOVER_NS
        {
            self.target_offset_mc -= CLAMP_TARGET_STEP_MC;
            self.target_offset_ns = now_ns;
        }
        self.was_clamped = clamped;
    }

    /// How the CPU ceiling should follow the last step, from the GPU's draw (mW) against the
    /// budget it was given. While governing, a GPU that draws its whole budget below its ceiling
    /// is limited by heat the CPU shares; one that leaves headroom waits for the CPU. When the
    /// hotspot stays above the target with the GPU budget at its floor, the heat is the CPU's
    /// and only the CPU can keep the firmware from clamping the GPU.
    pub(crate) fn cpu_advice(&self, power_mw: u32, now_ns: u64) -> CpuAdvice {
        if self.maximum_mw == 0 || now_ns.saturating_sub(self.last_activity_ns) >= IDLE_RESTORE_NS {
            return CpuAdvice::Release;
        }
        if !self.active {
            // Released by the controller: hand the steps back one at a time, so the CPU's heat
            // returns gradually and does not re-engage the governor at once.
            return CpuAdvice::Raise;
        }
        let budget = i64::from(self.budget_mw);
        let power = i64::from(power_mw);
        let target = target_mc() - self.target_offset_mc;
        let temp = self.last_temp_mc.unwrap_or(target);
        if temp >= target + CPU_HEAT_MARGIN_MC
            && budget <= FLOOR_MW.min(self.ceiling_mw) + CPU_HEAT_FLOOR_SLACK_MW
        {
            CpuAdvice::Lower
        } else if budget < self.ceiling_mw && power * 100 >= budget * GPU_BOUND_PERCENT {
            CpuAdvice::Lower
        } else if budget >= self.ceiling_mw
            || (power * 100 < budget * GPU_SLACK_PERCENT && temp < target)
        {
            // A GPU waiting for the CPU gets it back only below the target: above it, the
            // controller is still taking heat out.
            CpuAdvice::Raise
        } else {
            CpuAdvice::Hold
        }
    }

    /// Whether the fans should be boosted: while the controller governs a busy GPU.
    pub(crate) fn fans_wanted(&self, now_ns: u64) -> bool {
        self.maximum_mw != 0 && self.active
            && now_ns.saturating_sub(self.last_activity_ns) < IDLE_RESTORE_NS
    }

    /// One control step with the hotspot temperature, the GPU's current draw (mW) and whether the
    /// firmware's emergency clamp is active. Returns the budget to publish (if it changed) and
    /// whether to run again.
    pub(crate) fn tick(&mut self, temp_mc: Option<i32>, power_mw: u32, clamped: bool, now_ns: u64)
        -> (Option<u32>, bool)
    {
        let maximum = i64::from(self.maximum_mw);
        let idle = now_ns.saturating_sub(self.last_activity_ns) >= IDLE_RESTORE_NS;
        let dt_ns = now_ns.saturating_sub(self.last_tick_ns).min(2 * IDLE_RESTORE_NS);
        self.last_tick_ns = now_ns;
        let previous_mc = core::mem::replace(&mut self.last_temp_mc, temp_mc);
        if enabled() {
            self.adapt_target(clamped, now_ns);
        }
        let target = target_mc() - self.target_offset_mc;
        let desired = match temp_mc {
            _ if maximum == 0 => return (None, false),
            None => {
                self.active = false;
                maximum
            }
            _ if idle || !enabled() => {
                self.active = false;
                maximum
            }
            Some(temp) => {
                let error_mc = i64::from(target - temp);
                let proportional = KP_MW_PER_C * error_mc / 1000;
                let rise_mc_per_s = match previous_mc {
                    Some(previous) if temp > previous && dt_ns >= 1_000_000 =>
                        i64::from(temp - previous) * 1000 / (dt_ns / 1_000_000) as i64,
                    _ => 0,
                };
                let feed_forward = KF_MW_PER_C_PER_S * rise_mc_per_s / 1000;
                if !self.active && temp >= target - ENGAGE_BELOW_TARGET_MC {
                    self.active = true;
                    self.ceiling_mw = i64::from(power_mw).max(CEILING_MIN_MW).clamp(FLOOR_MW, maximum);
                    self.integral_mw = self.resume_integral(now_ns);
                }
                if self.active && clamped {
                    // The firmware's emergency clamp: hold the budget and the integral until it
                    // ends, so the GPU resumes at the converged budget.
                    self.last_active_ns = now_ns;
                    i64::from(self.budget_mw)
                } else if self.active {
                    self.last_active_ns = now_ns;
                    let floor = FLOOR_MW.min(self.ceiling_mw);
                    let candidate = self.integral_mw + KI_MW_PER_C_S * error_mc * (dt_ns / 1_000_000) as i64 / 1_000_000;
                    let output = proportional + candidate - feed_forward;
                    // Anti-windup: integrate only while the output is inside its range; at a
                    // limit, back-calculate the integral so the output sits on the limit.
                    let output = if output > self.ceiling_mw {
                        self.integral_mw = self.ceiling_mw - proportional + feed_forward;
                        self.ceiling_mw
                    } else if output < floor {
                        self.integral_mw = floor - proportional + feed_forward;
                        floor
                    } else {
                        self.integral_mw = candidate;
                        self.converged_mw = candidate;
                        output
                    };
                    if output >= self.ceiling_mw
                        && temp < target - ENGAGE_BELOW_TARGET_MC - RELEASE_HYSTERESIS_MC
                    {
                        // Released: back to the calibrated maximum.
                        self.active = false;
                        maximum
                    } else {
                        output
                    }
                } else {
                    maximum
                }
            }
        };
        let desired = if desired >= maximum { maximum } else { desired - desired % STEP_MW };
        let publish = (desired != i64::from(self.budget_mw)).then(|| {
            self.budget_mw = desired as u32;
            self.budget_mw
        });
        let again = (!idle && requested()) || self.budget_mw != self.maximum_mw;
        (publish, again)
    }
}
