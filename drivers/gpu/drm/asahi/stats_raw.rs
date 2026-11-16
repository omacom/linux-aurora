// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Raw firmware statistics snapshot and its read-only debugfs export.
//!
//! The AGX firmware sends statistics messages on the stats channel
//! (`channel::StatsChannel`). They are recorded here exactly as they arrive:
//! every field keeps the value the firmware sent, no unit is assumed, nothing
//! is scaled and no metric is derived from it. `FwBusy` is the firmware's own
//! busy flag and is *not* a GPU utilisation figure; the `Utilization` counters
//! are raw per-subqueue counters whose unit this driver does not assert.
//!
//! This complements the interpreted `stats` snapshot behind `agx_stats`: the
//! snapshot here keeps every message kind the firmware sends, together with
//! per-kind sample counters and capture times, and it never derives a value
//! from the firmware words.
//!
//! Every message kind keeps its own sample counter and the monotonic time at
//! which the sample was captured, so a reader can tell a missing sample
//! (count 0) from a stale one (old capture time), and can compute rates and
//! deltas between two reads.
//!
//! The snapshot is fixed size: recording a sample never allocates. It is
//! guarded by a spin lock because the critical section is short: a sample
//! is stored under the lock and a reader copies the snapshot out, so the lock
//! is never held across formatting.

use core::fmt::Write;

use kernel::{
    bindings,
    prelude::*,
    seq_file::SeqFile,
    sync::{
        Arc,
        SpinLock, //
    },
};

use crate::driver::AsahiDrmDevice;

/// Format tag of the export. Bumped whenever the field set changes.
pub(crate) const STATS_FORMAT: &str = "asahi_stats_v1";

/// The firmware statistics message kinds that are recorded.
///
/// The discriminants index the per-kind counters of [`RawStatsSnapshot`]; the
/// firmware message tag of each kind is given in its documentation, so every
/// exported line can be mapped back to the message it came from.
#[derive(Copy, Clone, PartialEq, Eq)]
#[repr(usize)]
pub(crate) enum StatsKind {
    /// 0x00 `Power`.
    Power = 0,
    /// 0x02 `PowerOn`.
    PowerOn = 1,
    /// 0x03 `PowerOff`.
    PowerOff = 2,
    /// 0x04 `Utilization`.
    Utilization = 3,
    /// 0x09 `AvgPower`.
    AvgPower = 4,
    /// 0x0a `Temperature`.
    Temperature = 5,
    /// 0x0b `PowerState`.
    PowerState = 6,
    /// 0x0c `FwBusy`.
    FwBusy = 7,
    /// 0x0d `PState`.
    PState = 8,
    /// 0x0e `TempSensor`.
    TempSensor = 9,
}

impl StatsKind {
    /// Number of recorded kinds.
    pub(crate) const COUNT: usize = 10;

    /// All kinds, in export order.
    const ALL: [StatsKind; StatsKind::COUNT] = [
        StatsKind::Power,
        StatsKind::PowerOn,
        StatsKind::PowerOff,
        StatsKind::Utilization,
        StatsKind::AvgPower,
        StatsKind::Temperature,
        StatsKind::PowerState,
        StatsKind::FwBusy,
        StatsKind::PState,
        StatsKind::TempSensor,
    ];

    /// Stable name of the kind in the export.
    fn name(self) -> &'static str {
        match self {
            StatsKind::Power => "power",
            StatsKind::PowerOn => "power_on",
            StatsKind::PowerOff => "power_off",
            StatsKind::Utilization => "utilization",
            StatsKind::AvgPower => "avg_power",
            StatsKind::Temperature => "temperature",
            StatsKind::PowerState => "power_state",
            StatsKind::FwBusy => "fw_busy",
            StatsKind::PState => "pstate",
            StatsKind::TempSensor => "temp_sensor",
        }
    }
}

/// Last received firmware statistics of one device.
///
/// All values are the raw firmware words. The driver knows the unit of none
/// of them and derives nothing from them.
#[derive(Copy, Clone, Default)]
pub(crate) struct RawStatsSnapshot {
    /// Number of samples received per kind (0 means: never received).
    count: [u64; StatsKind::COUNT],
    /// Monotonic capture time (ns) of the last sample per kind.
    captured_ns: [u64; StatsKind::COUNT],
    /// Number of statistics messages received on the channel.
    messages: u64,

    /// 0x00 `Power`: raw power word, unit unknown.
    power: u64,
    /// 0x02 `PowerOn`: raw off-time word, unit unknown.
    power_off_time: u64,
    /// 0x03 `PowerOff`: raw on-time word, unit unknown.
    power_on_time: u64,
    /// 0x04 `Utilization`: raw firmware timestamp and four raw counters.
    util_timestamp: u64,
    /// 0x04 `Utilization`: the four raw counters.
    util: [u32; 4],
    /// 0x09 `AvgPower`: raw active time word, unit unknown.
    avg_power_active_cs: u64,
    /// 0x09 `AvgPower`: raw undecoded word.
    avg_power_unk2: u32,
    /// 0x09 `AvgPower`: raw undecoded word.
    avg_power_unk3: u32,
    /// 0x09 `AvgPower`: raw undecoded word.
    avg_power_unk4: u32,
    /// 0x09 `AvgPower`: raw average power word, unit unknown.
    avg_power: u32,
    /// 0x0a `Temperature`: raw sensor word.
    temp_raw_value: u32,
    /// 0x0a `Temperature`: raw scale word, travels with the sample.
    temp_scale: u32,
    /// 0x0a `Temperature`: raw minimum word.
    temp_tmin: u32,
    /// 0x0a `Temperature`: raw maximum word.
    temp_tmax: u32,
    /// 0x0b `PowerState`: raw firmware timestamp.
    power_state_timestamp: u64,
    /// 0x0b `PowerState`: raw firmware timestamp of the last busy period.
    power_state_last_busy_ts: u64,
    /// 0x0b `PowerState`: raw active word.
    power_state_active: u32,
    /// 0x0b `PowerState`: raw poweroff word.
    power_state_poweroff: u32,
    /// 0x0b `PowerState`: raw undecoded word.
    power_state_unk1: u32,
    /// 0x0b `PowerState`: raw p-state index.
    power_state_pstate: u32,
    /// 0x0b `PowerState`: raw undecoded word.
    power_state_unk2: u32,
    /// 0x0b `PowerState`: raw undecoded word.
    power_state_unk3: u32,
    /// 0x0c `FwBusy`: raw firmware timestamp.
    fw_busy_timestamp: u64,
    /// 0x0c `FwBusy`: raw firmware busy flag.
    fw_busy: u32,
    /// 0x0d `PState`: raw lowest p-state bound.
    pstate_min: u32,
    /// 0x0d `PState`: raw undecoded word.
    pstate_unk1: u32,
    /// 0x0d `PState`: raw highest p-state bound.
    pstate_max: u32,
    /// 0x0d `PState`: raw undecoded word.
    pstate_unk2: u32,
    /// 0x0e `TempSensor`: raw sensor identifier.
    sensor_id: u32,
    /// 0x0e `TempSensor`: raw sensor word.
    sensor_raw_value: u32,
    /// 0x0e `TempSensor`: raw scale word, travels with the sample.
    sensor_scale: u32,
    /// 0x0e `TempSensor`: raw minimum word.
    sensor_tmin: u32,
    /// 0x0e `TempSensor`: raw maximum word.
    sensor_tmax: u32,
}

impl RawStatsSnapshot {
    /// Note one statistics message on the channel.
    pub(crate) fn note_message(&mut self) {
        self.messages += 1;
    }

    /// Record one sample of `kind` captured at `stamp` (monotonic ns).
    fn record(&mut self, kind: StatsKind, stamp: u64) {
        self.count[kind as usize] += 1;
        self.captured_ns[kind as usize] = stamp;
    }

    /// Record the 0x00 `Power` sample.
    pub(crate) fn set_power(&mut self, power: u64, stamp: u64) {
        self.power = power;
        self.record(StatsKind::Power, stamp);
    }

    /// Record the 0x02 `PowerOn` sample.
    pub(crate) fn set_power_on(&mut self, off_time: u64, stamp: u64) {
        self.power_off_time = off_time;
        self.record(StatsKind::PowerOn, stamp);
    }

    /// Record the 0x03 `PowerOff` sample.
    pub(crate) fn set_power_off(&mut self, on_time: u64, stamp: u64) {
        self.power_on_time = on_time;
        self.record(StatsKind::PowerOff, stamp);
    }

    /// Record the 0x04 `Utilization` sample.
    pub(crate) fn set_utilization(&mut self, timestamp: u64, util: [u32; 4], stamp: u64) {
        self.util_timestamp = timestamp;
        self.util = util;
        self.record(StatsKind::Utilization, stamp);
    }

    /// Record the 0x09 `AvgPower` sample.
    pub(crate) fn set_avg_power(
        &mut self,
        active_cs: u64,
        unk: [u32; 3],
        avg_power: u32,
        stamp: u64,
    ) {
        self.avg_power_active_cs = active_cs;
        self.avg_power_unk2 = unk[0];
        self.avg_power_unk3 = unk[1];
        self.avg_power_unk4 = unk[2];
        self.avg_power = avg_power;
        self.record(StatsKind::AvgPower, stamp);
    }

    /// Record the 0x0a `Temperature` sample.
    pub(crate) fn set_temperature(
        &mut self,
        raw_value: u32,
        scale: u32,
        tmin: u32,
        tmax: u32,
        stamp: u64,
    ) {
        self.temp_raw_value = raw_value;
        self.temp_scale = scale;
        self.temp_tmin = tmin;
        self.temp_tmax = tmax;
        self.record(StatsKind::Temperature, stamp);
    }

    /// Record the 0x0b `PowerState` sample.
    #[allow(clippy::too_many_arguments)]
    pub(crate) fn set_power_state(
        &mut self,
        timestamp: u64,
        last_busy_ts: u64,
        active: u32,
        poweroff: u32,
        unk1: u32,
        pstate: u32,
        unk2: u32,
        unk3: u32,
        stamp: u64,
    ) {
        self.power_state_timestamp = timestamp;
        self.power_state_last_busy_ts = last_busy_ts;
        self.power_state_active = active;
        self.power_state_poweroff = poweroff;
        self.power_state_unk1 = unk1;
        self.power_state_pstate = pstate;
        self.power_state_unk2 = unk2;
        self.power_state_unk3 = unk3;
        self.record(StatsKind::PowerState, stamp);
    }

    /// Record the 0x0c `FwBusy` sample.
    pub(crate) fn set_fw_busy(&mut self, timestamp: u64, busy: u32, stamp: u64) {
        self.fw_busy_timestamp = timestamp;
        self.fw_busy = busy;
        self.record(StatsKind::FwBusy, stamp);
    }

    /// Record the 0x0d `PState` sample.
    pub(crate) fn set_pstate(
        &mut self,
        ps_min: u32,
        unk1: u32,
        ps_max: u32,
        unk2: u32,
        stamp: u64,
    ) {
        self.pstate_min = ps_min;
        self.pstate_unk1 = unk1;
        self.pstate_max = ps_max;
        self.pstate_unk2 = unk2;
        self.record(StatsKind::PState, stamp);
    }

    /// Record the 0x0e `TempSensor` sample.
    pub(crate) fn set_temp_sensor(
        &mut self,
        sensor_id: u32,
        raw_value: u32,
        scale: u32,
        tmin: u32,
        tmax: u32,
        stamp: u64,
    ) {
        self.sensor_id = sensor_id;
        self.sensor_raw_value = raw_value;
        self.sensor_scale = scale;
        self.sensor_tmin = tmin;
        self.sensor_tmax = tmax;
        self.record(StatsKind::TempSensor, stamp);
    }

    /// Write the raw payload fields of one kind.
    fn write_payload(&self, kind: StatsKind, out: &mut impl Write) -> core::fmt::Result {
        match kind {
            StatsKind::Power => write!(out, " power={}", self.power),
            StatsKind::PowerOn => write!(out, " off_time={}", self.power_off_time),
            StatsKind::PowerOff => write!(out, " on_time={}", self.power_on_time),
            StatsKind::Utilization => write!(
                out,
                " timestamp={} util1={} util2={} util3={} util4={}",
                self.util_timestamp, self.util[0], self.util[1], self.util[2], self.util[3]
            ),
            StatsKind::AvgPower => write!(
                out,
                " active_cs={} unk2={} unk3={} unk4={} avg_power={}",
                self.avg_power_active_cs,
                self.avg_power_unk2,
                self.avg_power_unk3,
                self.avg_power_unk4,
                self.avg_power
            ),
            StatsKind::Temperature => write!(
                out,
                " raw_value={} scale={} tmin={} tmax={}",
                self.temp_raw_value, self.temp_scale, self.temp_tmin, self.temp_tmax
            ),
            StatsKind::PowerState => write!(
                out,
                " timestamp={} last_busy_ts={} active={} poweroff={} unk1={} pstate={} unk2={} unk3={}",
                self.power_state_timestamp,
                self.power_state_last_busy_ts,
                self.power_state_active,
                self.power_state_poweroff,
                self.power_state_unk1,
                self.power_state_pstate,
                self.power_state_unk2,
                self.power_state_unk3
            ),
            StatsKind::FwBusy => {
                write!(out, " timestamp={} busy={}", self.fw_busy_timestamp, self.fw_busy)
            }
            StatsKind::PState => write!(
                out,
                " ps_min={} unk1={} ps_max={} unk2={}",
                self.pstate_min, self.pstate_unk1, self.pstate_max, self.pstate_unk2
            ),
            StatsKind::TempSensor => write!(
                out,
                " sensor_id={} raw_value={} scale={} tmin={} tmax={}",
                self.sensor_id, self.sensor_raw_value, self.sensor_scale, self.sensor_tmin, self.sensor_tmax
            ),
        }
    }
}

/// Write the snapshot in the documented text format.
///
/// One line per record, `key=value` pairs, LF terminated. A kind that was
/// never received is printed with `count=0` and without payload, so a missing
/// sample is never confused with a zero value. `now_ns` and `captured_ns` are
/// monotonic (CLOCK_MONOTONIC) nanoseconds taken by the driver; every other
/// value is a raw firmware word whose unit this driver does not assert.
pub(crate) fn format_snapshot(
    snap: &RawStatsSnapshot,
    now_ns: u64,
    device: &str,
    out: &mut impl Write,
) -> core::fmt::Result {
    writeln!(out, "{}", STATS_FORMAT)?;
    writeln!(out, "device={}", device)?;
    writeln!(out, "now_ns={}", now_ns)?;
    writeln!(out, "messages count={}", snap.messages)?;
    for kind in StatsKind::ALL {
        let count = snap.count[kind as usize];
        write!(out, "{} count={}", kind.name(), count)?;
        if count != 0 {
            write!(out, " captured_ns={}", snap.captured_ns[kind as usize])?;
            snap.write_payload(kind, out)?;
        }
        writeln!(out)?;
    }
    Ok(())
}

/// Current monotonic time in nanoseconds.
pub(crate) fn monotonic_ns() -> u64 {
    // SAFETY: `ktime_get()` may be called from any context.
    unsafe { bindings::ktime_get() as u64 }
}

/// Publish the firmware statistics of one device as a read-only debugfs file.
///
/// The file is created in the DRM device's own debugfs directory
/// (`/sys/kernel/debug/dri/<card>/asahi_stats`), so a card only ever exposes
/// its own statistics. The DRM core removes it when the device is
/// unregistered, and the caller keeps the snapshot alive until after that
/// registration is dropped, so the pointer stays valid for as long as the
/// file exists.
pub(crate) fn register(drm: &AsahiDrmDevice, stats: &Arc<SpinLock<RawStatsSnapshot>>) {
    // `drm::Device` is `repr(C)` with the `drm_device` as its first field.
    let raw = (drm as *const AsahiDrmDevice)
        .cast_mut()
        .cast::<bindings::drm_device>();
    // `drm_debugfs_add_file()` is a static inline stub without CONFIG_DEBUG_FS, so
    // bindgen has no binding for it and there is no file to create.
    #[cfg(CONFIG_DEBUG_FS)]
    // SAFETY: `raw` is the live DRM device. It is registered at this point,
    // so its debugfs directory exists and the file is created in it.
    unsafe {
        bindings::drm_debugfs_add_file(
            raw,
            c"asahi_stats".as_char_ptr(),
            Some(show),
            Arc::as_ptr(stats).cast_mut().cast::<core::ffi::c_void>(),
        );
    }
    #[cfg(not(CONFIG_DEBUG_FS))]
    let _ = (raw, stats);
}

/// `drm_debugfs_add_file()` show callback.
///
/// The DRM core opens the file with `single_open(file, node->show, entry)`, so
/// the seq file's private data is the `struct drm_debugfs_entry` and the
/// per-device payload is in `entry->file.data` (see `drm_debugfs_entry_open()`
/// in `drivers/gpu/drm/drm_debugfs.c`). The `data` argument of the callback is
/// the seq file start token, not the payload.
#[cfg_attr(not(CONFIG_DEBUG_FS), allow(dead_code))]
unsafe extern "C" fn show(
    m: *mut bindings::seq_file,
    _data: *mut core::ffi::c_void,
) -> core::ffi::c_int {
    // SAFETY: The DRM core passes a valid seq file to the show callback.
    let seq = unsafe { SeqFile::from_raw(m) };
    // SAFETY: The seq file was opened with the DRM entry as private data.
    let entry = unsafe { (*m).private.cast::<bindings::drm_debugfs_entry>() };
    // SAFETY: `entry->file.data` is the payload pointer we registered.
    let stats = unsafe { (*entry).file.data.cast::<SpinLock<RawStatsSnapshot>>() };

    let mut out = SeqWriter { seq };
    // SAFETY: The device-owned snapshot outlives its debugfs entry.
    let snapshot = *unsafe { (*stats).lock() };
    let device = device_name(entry.cast_const());
    let _ = format_snapshot(&snapshot, monotonic_ns(), device, &mut out);
    0
}

/// Name of the DRM device the statistics belong to.
///
/// The name is owned by the DRM device, which outlives the read.
fn device_name<'a>(entry: *const bindings::drm_debugfs_entry) -> &'a str {
    // SAFETY: The DRM core passes the entry it allocated for this file.
    let drm = unsafe { (*entry).dev };
    if drm.is_null() {
        return "unknown";
    }
    // SAFETY: `drm` is the live DRM device of this entry.
    let unique = unsafe { (*drm).unique };
    if unique.is_null() {
        return "unknown";
    }
    // SAFETY: `unique` is a NUL terminated string owned by the DRM device,
    // which outlives this read.
    let name = unsafe { core::ffi::CStr::from_ptr(unique) };
    name.to_str().unwrap_or("unknown")
}

/// [`Write`] adapter that forwards to a seq file.
struct SeqWriter<'a> {
    seq: &'a SeqFile,
}

impl Write for SeqWriter<'_> {
    fn write_str(&mut self, value: &str) -> core::fmt::Result {
        self.seq.call_printf(fmt!("{}", value));
        Ok(())
    }
}
