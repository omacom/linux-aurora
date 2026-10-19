// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Thermal limit of the M3 runtime backend's GPU performance cap (`asahi.m3_thermal`).
//!
//! Without it (`off`, the default with the built-in InitData image) the firmware is given a
//! fixed highest performance state (5 of 8 with device-tree InitData: 1056 of 1380 MHz on
//! J516S) and nothing watches temperatures. The firmware's own die temperature controller is
//! off in the InitData this driver builds. With device-tree InitData the default is `on`;
//! `asahi.m3_thermal=off` restores the fixed cap exactly.
//!
//! With it, the InitData gives the firmware the whole table as its highest state (the
//! "ceiling"), and what bounds the firmware is the runtime cap: the Globals performance-state
//! cap and power-interface state targets, which the firmware keeps reading while it runs (a
//! host moves the cap with a single store, with no message). The runtime cap starts at the
//! board-specific fallback (the "safe cap") and never goes below it. The T8122 experiment
//! retains state 2 as that fallback even when its ceiling is raised; other SoCs retain
//! the cap used without the limit.
//!
//! - `hold`: the runtime cap stays at the safe cap for the whole boot, provided the firmware
//!   follows the runtime cap; the per-job check below proves or disproves that before `on`
//!   is used.
//! - `on`: the runtime cap follows the hottest SoC die temperature, read by a work item every
//!   500 ms while the GPU is busy (every 2 s when idle) from the thermal zones in [`ZONES`]
//!   that exist: the SMC's die sensors and the PMP's CPU cluster sensors (see [`ZONES`]):
//!   * at or above `critical` (`hot` + 8 C): the safe cap at once;
//!   * at or above `hot` (`asahi.m3_thermal_hot`, default 80 C): one state lower, at most every
//!     500 ms and once per reading;
//!   * at or below `cool` (`hot` - 10 C): one state higher, at most every 2 s and once per
//!     reading, up to the ceiling;
//!   * no zone, a zone that cannot be read, a reading outside 10..=125 C, one older than 3 s
//!     or one that has not changed at all for 60 s: the safe cap at once.
//!
//! The defaults leave a wide margin below the limits the boot firmware's device tree gives this
//! machine (a 94 C GPU die target and a 111 C SoC-hot temperature), because the sensors may
//! read cooler than the GPU's hottest spot (the PMP's are CPU cluster sensors), and this limit
//! reacts in steps of hundreds of milliseconds while the firmware changes state within the cap
//! much faster.
//!
//! The cap is changed only by the job thread, after a job has retired, right before the state
//! the firmware reports is checked. After a lowering the previous cap is tolerated for 1 s. A
//! reported state above the runtime cap in 20 consecutive checks spanning at least 3 s means
//! the firmware does not follow the runtime cap, so this limit cannot work: the GPU is then
//! marked failed, as for a state above the ceiling.
//!
//! The decisions are in `m3_thermal_policy`, which host tests drive; this module reads the
//! sensors, publishes the cap and logs.

use core::sync::atomic::{AtomicBool, Ordering};

use kernel::{
    bindings, c_str, device,
    error::to_result,
    new_spinlock,
    prelude::*,
    str::CStr,
    sync::{Arc, SpinLock},
    time::{msecs_to_jiffies, Instant, Monotonic},
    workqueue::{self, impl_has_delayed_work, new_delayed_work, DelayedWork, WorkItem},
};

use crate::{
    m3_adt_config::PstatePolicy,
    m3_params::{self, ThermalMode},
    m3_thermal_policy::{Change, Policy, Reading, Source, Why, S},
};

/// The thermal zones read, each the hottest of a set of SoC die sensors: the SMC's die sensors
/// (registered by the SMC hwmon driver) and the PMP's CPU cluster sensors (registered by the
/// PMP report driver once the PMP runs). Zones that do not exist are skipped; a zone that
/// exists but cannot be read fails the reading. The M3 Pro boards (J514S and J516S) need
/// both zones: on a J516S the SMC die sensors alone stayed below 67 C while the PMP hotspot
/// exceeded 85 C and the cap stepped down. Without the PMP running, the cap stays safe.
/// Every T8122 board requires the SMC die zone; T8122 has no PMP hotspot zone.
const ZONES: [&CStr; 2] = [c_str!("macsmc_soc_die"), c_str!("apple_pmp_hotspot")];
// Do not raise its cap on a partial reconstruction of that sensor set.
const J516_REQUIRED_ZONES: u32 = 3;
fn required_zones() -> u32 {
    let Some(root) = kernel::of::root() else { return J516_REQUIRED_ZONES; };
    let Ok(compatible) = root.get_property::<KVec<u8>>(c_str!("compatible")) else {
        return J516_REQUIRED_ZONES;
    };
    crate::m3_thermal_policy::required_zones(&compatible)
}

/// Sensor polling period while the GPU is busy and while it is idle, in ms.
const POLL_BUSY_MS: u32 = 500;
const POLL_IDLE_MS: u32 = 2000;
/// The GPU counts as busy for this long after a job has retired, in ns.
const BUSY_WINDOW_NS: i64 = 5 * S;
/// Lines logging cap changes per boot, and the period of the summary line while busy.
const LOG_LINES: u32 = 256;
const SUMMARY_EVERY_NS: i64 = 30 * S;

type Time = Instant<Monotonic>;

/// Nanoseconds from `base` to `t`.
fn ns(base: Time, t: Time) -> i64 {
    (t - base).as_nanos()
}

struct SensorState {
    reading: Reading,
    /// When the GPU last retired a job, in ns from the sensor's base.
    busy: i64,
}

/// Reads [`ZONES`] from a self-rearming delayed work item, so the job thread never waits for the
/// SMC. The work item holds a reference, so it stays valid after the governor is dropped; it
/// then stops at its next run.
#[pin_data]
struct Sensor {
    required_zones: u32,
    /// The origin of the sensor's and the governor's times.
    base: Time,
    stopped: AtomicBool,
    #[pin]
    state: SpinLock<SensorState>,
    #[pin]
    work: DelayedWork<Sensor>,
}

impl_has_delayed_work! {
    impl HasDelayedWork<Self> for Sensor { self.work }
}

/// One reading of the zone `name`, in millidegrees Celsius; ENODEV if there is no such zone.
fn read_zone(name: &CStr) -> Result<i32> {
    let mut temp: i32 = 0;
    // SAFETY: name is NUL-terminated and temp is a valid output pointer. The
    // thermal core acquires a device reference under its registry lock and
    // keeps it through sampling, so zone unregistration cannot free it here.
    to_result(unsafe { bindings::thermal_zone_get_temp_by_name(name.as_char_ptr(), &mut temp) })?;
    Ok(temp)
}

/// The hottest reading of the zones in [`ZONES`] that exist, and which ones were read.
fn read_zones(required: u32) -> Result<(i32, u32)> {
    let mut hottest: Option<i32> = None;
    let mut zones = 0;
    for (i, name) in ZONES.iter().enumerate() {
        match read_zone(name) {
            Ok(temp) => {
                hottest = Some(hottest.map_or(temp, |h| h.max(temp)));
                zones |= 1 << i;
            }
            Err(e) if e == ENODEV => {}
            Err(e) => return Err(e),
        }
    }
    if zones & required != required {
        return Err(ENODEV);
    }
    hottest.map(|t| (t, zones)).ok_or(ENODEV)
}

impl WorkItem for Sensor {
    type Pointer = Arc<Sensor>;

    fn run(this: Arc<Sensor>) {
        if this.stopped.load(Ordering::Acquire) {
            return;
        }
        let result = read_zones(this.required_zones);
        let at = ns(this.base, Time::now());
        let reading = match result {
            Ok((mc, zones)) => Reading::Temp { mc, at, zones },
            Err(e) => Reading::Failed {
                errno: e.to_errno(),
                at,
            },
        };
        let busy = {
            let mut state = this.state.lock();
            state.reading = reading;
            at - state.busy < BUSY_WINDOW_NS
        };
        let delay = if busy { POLL_BUSY_MS } else { POLL_IDLE_MS };
        let _ = workqueue::system().enqueue_delayed(this, msecs_to_jiffies(delay));
    }
}

fn describe(source: Source) -> &'static str {
    match source {
        Source::Unknown | Source::NoReading => "no reading yet",
        Source::Ok(_) => "reading",
        Source::Failed(e) if e == ENODEV.to_errno() => "no SoC die temperature zone",
        Source::Failed(_) => "a SoC die temperature zone cannot be read",
        Source::Stale => "the reading is older than 3 s",
        Source::Implausible => "the reading is outside 10..=125 C",
        Source::Stuck => "the reading has not changed for 60 s",
    }
}

fn describe_why(why: Why) -> &'static str {
    match why {
        Why::Critical => "critical",
        Why::Hot => "hot",
        Why::Cool => "cool",
        Why::NoTemperature(source) => describe(source),
    }
}

/// Degrees Celsius with one decimal, for the log.
struct Celsius(i32);

impl kernel::fmt::Display for Celsius {
    fn fmt(&self, f: &mut kernel::fmt::Formatter<'_>) -> kernel::fmt::Result {
        let sign = if self.0 < 0 { "-" } else { "" };
        let v = self.0.unsigned_abs();
        write!(f, "{}{}.{} C", sign, v / 1000, (v % 1000) / 100)
    }
}

/// The runtime cap and the thermal limit that moves it.
pub(crate) struct Governor {
    mode: ThermalMode,
    /// Without the limit (`off`): the cap, which never changes.
    fixed: u32,
    policy: Policy,
    freqs: [u32; 16],
    base: Time,
    sensor: Option<Arc<Sensor>>,
    source: Source,
    last_summary: Option<i64>,
    /// The highest state the firmware reported since the last summary.
    peak: u32,
    lines: u32,
}

impl Governor {
    /// The governor for `policy`. Starts the sensor work item unless the limit is off.
    pub(crate) fn new(dev: &device::Device, pstates: &PstatePolicy) -> Result<Self> {
        let (hot_c, clamped) = m3_params::thermal_hot_param();
        let base = Time::now();
        let mut governor = Governor {
            mode: pstates.thermal,
            fixed: pstates.max,
            policy: Policy::new(
                pstates.thermal == ThermalMode::On,
                pstates.safe,
                pstates.max,
                hot_c as i32 * 1000,
            ),
            freqs: pstates.freqs,
            base,
            sensor: None,
            source: Source::Unknown,
            last_summary: None,
            peak: 0,
            lines: 0,
        };
        if governor.mode == ThermalMode::Off {
            return Ok(governor);
        }
        if clamped {
            dev_info!(
                dev,
                "M3: asahi.m3_thermal_hot clamped to {} C ({}..={} C)\n",
                hot_c,
                m3_params::THERMAL_HOT_MIN,
                m3_params::THERMAL_HOT_MAX
            );
        }
        let sensor = Arc::pin_init(
            pin_init!(Sensor {
                required_zones: required_zones(),
                base,
                stopped: AtomicBool::new(false),
                state <- new_spinlock!(SensorState {
                    reading: Reading::None,
                    busy: 0,
                }),
                work <- new_delayed_work!("asahi::M3ThermalSensor"),
            }),
            GFP_KERNEL,
        )?;
        let _ = workqueue::system().enqueue_delayed(sensor.clone(), msecs_to_jiffies(POLL_BUSY_MS));
        governor.sensor = Some(sensor);
        let p = &governor.policy;
        let (cool, hot, critical) = p.thresholds();
        match governor.mode {
            ThermalMode::Hold => dev_info!(
                dev,
                "M3: thermal limit holding (asahi.m3_thermal=hold): the firmware may use states 1..={} ({} MHz) and the runtime cap stays at {} ({} MHz) for this boot; the SoC die temperature is logged only\n",
                p.ceiling(),
                governor.mhz(p.ceiling()),
                p.safe(),
                governor.mhz(p.safe())
            ),
            _ => dev_info!(
                dev,
                "M3: thermal limit on (asahi.m3_thermal=on): runtime cap {}..={} ({}..={} MHz) from the hottest SoC die temperature of zones macsmc_soc_die and apple_pmp_hotspot: up while at or below {}, down at or above {}, cap {} at or above {} or without a fresh reading\n",
                p.safe(),
                p.ceiling(),
                governor.mhz(p.safe()),
                governor.mhz(p.ceiling()),
                Celsius(cool),
                Celsius(hot),
                p.safe(),
                Celsius(critical)
            ),
        }
        Ok(governor)
    }

    fn mhz(&self, state: u32) -> u32 {
        self.freqs.get(state as usize).copied().unwrap_or(0)
    }

    fn cap(&self) -> u32 {
        match self.mode {
            ThermalMode::Off => self.fixed,
            ThermalMode::Hold | ThermalMode::On => self.policy.cap(),
        }
    }

    fn note_source(&mut self, dev: &device::Device, source: Source, temp: Option<i32>) {
        if source == self.source {
            return;
        }
        self.source = source;
        let safe = self.policy.safe();
        match (source, temp) {
            (Source::Ok(zones), Some(t)) => {
                let smc = if zones & 1 != 0 {
                    " macsmc_soc_die (SMC die sensors)"
                } else {
                    ""
                };
                let pmp = if zones & 2 != 0 {
                    " apple_pmp_hotspot (PMP CPU cluster sensors)"
                } else {
                    ""
                };
                dev_info!(
                    dev,
                    "M3: thermal: SoC die temperature {} from zones:{}{}\n",
                    Celsius(t),
                    smc,
                    pmp
                )
            }
            (Source::Failed(e), _) => dev_info!(
                dev,
                "M3: thermal: {} (error {}); runtime cap at most {} ({} MHz)\n",
                describe(source),
                e,
                safe,
                self.mhz(safe)
            ),
            _ => dev_info!(
                dev,
                "M3: thermal: {}; runtime cap at most {} ({} MHz)\n",
                describe(source),
                safe,
                self.mhz(safe)
            ),
        }
    }

    fn summary(&mut self, dev: &device::Device, now: i64, temp: Option<i32>) {
        if self
            .last_summary
            .is_some_and(|t| now - t < SUMMARY_EVERY_NS)
        {
            return;
        }
        self.last_summary = Some(now);
        let peak = core::mem::take(&mut self.peak);
        let cap = self.policy.cap();
        let (raised, lowered) = self.policy.counts();
        let (safe, ceiling) = (self.policy.safe(), self.policy.ceiling());
        match temp {
            Some(t) => dev_info!(
                dev,
                "M3: thermal: SoC die {}, runtime cap {} ({} MHz) of {}..={}, firmware peak state {} since the last summary, raised {} and lowered {} times so far\n",
                Celsius(t),
                cap,
                self.mhz(cap),
                safe,
                ceiling,
                peak,
                raised,
                lowered
            ),
            None => dev_info!(
                dev,
                "M3: thermal: no SoC die temperature ({}), runtime cap {} ({} MHz) of {}..={}, firmware peak state {} since the last summary, raised {} and lowered {} times so far\n",
                describe(self.source),
                cap,
                self.mhz(cap),
                safe,
                ceiling,
                peak,
                raised,
                lowered
            ),
        }
    }

    fn log_change(&mut self, dev: &device::Device, change: Change, temp: Option<i32>) {
        if self.lines < LOG_LINES {
            match temp {
                Some(t) => dev_info!(
                    dev,
                    "M3: thermal: runtime cap {} -> {} ({} MHz): SoC die {} ({})\n",
                    change.from,
                    change.to,
                    self.mhz(change.to),
                    Celsius(t),
                    describe_why(change.why)
                ),
                None => dev_info!(
                    dev,
                    "M3: thermal: runtime cap {} -> {} ({} MHz): {}\n",
                    change.from,
                    change.to,
                    self.mhz(change.to),
                    describe_why(change.why)
                ),
            }
        } else if self.lines == LOG_LINES {
            dev_info!(
                dev,
                "M3: thermal: further runtime cap changes are not logged\n"
            );
        }
        self.lines = self.lines.saturating_add(1);
    }

    /// Recompute the runtime cap after a job. Returns the new cap if it changed; the caller
    /// publishes it.
    pub(crate) fn update(&mut self, dev: &device::Device, now: Time) -> Option<u32> {
        let Some(sensor) = &self.sensor else {
            return None;
        };
        let reading = sensor.state.lock().reading;
        let now = ns(self.base, now);
        let (usable, change) = self.policy.update(now, reading);
        let temp = usable.ok().map(|(t, _, _)| t);
        match usable {
            Err(source) => self.note_source(dev, source, None),
            Ok((t, _, zones)) => self.note_source(dev, Source::Ok(zones), Some(t)),
        }
        self.summary(dev, now, temp);
        let change = change?;
        self.log_change(dev, change, temp);
        Some(change.to)
    }

    /// The highest state the firmware may report now: the runtime cap, or the highest cap of
    /// the grace period after a lowering.
    pub(crate) fn allowed(&self, now: Time) -> u32 {
        match self.mode {
            ThermalMode::Off => self.fixed,
            ThermalMode::Hold | ThermalMode::On => self.policy.allowed(ns(self.base, now)),
        }
    }

    /// Account one check of the reported state; `over` says whether it stayed above
    /// [`Self::allowed`], `peak` is the highest state it reported. Fails with ERANGE when the
    /// firmware does not follow the runtime cap.
    pub(crate) fn note_check(
        &mut self,
        dev: &device::Device,
        now: Time,
        over: bool,
        peak: u32,
    ) -> Result {
        self.peak = self.peak.max(peak);
        if let Some((count, span)) = self.policy.note_check(ns(self.base, now), over) {
            let cap = self.cap();
            dev_err!(
                dev,
                "M3: thermal: the firmware reported a state above the runtime cap {} ({} MHz) in {} consecutive checks over {} ms: it does not follow the runtime cap; marking the GPU failed\n",
                cap,
                self.mhz(cap),
                count,
                span / 1_000_000
            );
            return Err(ERANGE);
        }
        Ok(())
    }

    /// Note that the GPU retired a job, so the sensor is read at the busy rate.
    pub(crate) fn busy(&self, now: Time) {
        if let Some(sensor) = &self.sensor {
            sensor.state.lock().busy = ns(self.base, now);
        }
    }
}

impl Drop for Governor {
    fn drop(&mut self) {
        if let Some(sensor) = &self.sensor {
            sensor.stopped.store(true, Ordering::Release);
        }
    }
}
