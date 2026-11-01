#!/usr/bin/env python3
"""Exercise the raw firmware statistics snapshot and its export format.

The snapshot keeps the firmware words verbatim: no unit is assumed, nothing is
scaled and no metric is derived. This harness pins that contract, because a
consumer parses the exported text.
"""

from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[1]
stats = (ROOT / "stats_raw.rs").read_text()


def block(marker):
    """Extract the item starting at `marker`, up to its matching brace."""
    start = stats.index(marker)
    index = stats.index("{", start) + 1
    depth = 1
    while depth:
        depth += (stats[index] == "{") - (stats[index] == "}")
        index += 1
    return stats[start:index]

format_tag = re.search(r"^pub\(crate\) const STATS_FORMAT: &str = \".*\";$", stats, re.MULTILINE)
assert format_tag is not None


source = "\n".join([
    "#![allow(dead_code, unused_variables)]",
    "use std::fmt::Write;",
    format_tag.group(),
    block("#[derive(Copy, Clone, PartialEq, Eq)]"),
    block("impl StatsKind"),
    block("#[derive(Copy, Clone, Default)]"),
    block("impl RawStatsSnapshot"),
    block("pub(crate) fn format_snapshot"),
    r'''
fn render(snap: &RawStatsSnapshot) -> String {
    let mut out = String::new();
    format_snapshot(snap, 1000, "testdev", &mut out).unwrap();
    out
}

const EMPTY: &str = "\
asahi_stats_v1
device=testdev
now_ns=1000
messages count=0
power count=0
power_on count=0
power_off count=0
utilization count=0
avg_power count=0
temperature count=0
power_state count=0
fw_busy count=0
pstate count=0
temp_sensor count=0
";

#[test]
fn missing_samples_are_explicit() {
    let snap = RawStatsSnapshot::default();
    assert_eq!(render(&snap), EMPTY);
    // A kind that was never received carries no payload at all, so a missing
    // sample can never be read as a zero value.
    for line in EMPTY.lines() {
        if line.ends_with("count=0") {
            assert_eq!(line.split_whitespace().count(), 2, "{}", line);
        }
    }
}

#[test]
fn raw_samples_are_preserved_verbatim() {
    let mut snap = RawStatsSnapshot::default();
    snap.note_message();
    snap.set_power(0x1234, 11);
    snap.note_message();
    snap.set_power_on(0x2222, 12);
    snap.note_message();
    snap.set_power_off(0x3333, 13);
    snap.note_message();
    snap.set_utilization(0x4444, [1, 2, 3, 4], 14);
    snap.note_message();
    snap.set_avg_power(0x5555, [6, 7, 8], 9, 15);
    snap.note_message();
    snap.set_temperature(30000, 1000, 0, 100000, 16);
    snap.note_message();
    snap.set_power_state(0x6666, 0x7777, 1, 0, 2, 5, 3, 4, 17);
    snap.note_message();
    snap.set_fw_busy(0x8888, 1, 18);
    snap.note_message();
    snap.set_pstate(0, 1, 7, 2, 19);
    snap.note_message();
    snap.set_temp_sensor(2, 30000, 1000, 0, 100000, 20);

    assert_eq!(render(&snap), "\
asahi_stats_v1
device=testdev
now_ns=1000
messages count=10
power count=1 captured_ns=11 power=4660
power_on count=1 captured_ns=12 off_time=8738
power_off count=1 captured_ns=13 on_time=13107
utilization count=1 captured_ns=14 timestamp=17476 util1=1 util2=2 util3=3 util4=4
avg_power count=1 captured_ns=15 active_cs=21845 unk2=6 unk3=7 unk4=8 avg_power=9
temperature count=1 captured_ns=16 raw_value=30000 scale=1000 tmin=0 tmax=100000
power_state count=1 captured_ns=17 timestamp=26214 last_busy_ts=30583 active=1 poweroff=0 unk1=2 pstate=5 unk2=3 unk3=4
fw_busy count=1 captured_ns=18 timestamp=34952 busy=1
pstate count=1 captured_ns=19 ps_min=0 unk1=1 ps_max=7 unk2=2
temp_sensor count=1 captured_ns=20 sensor_id=2 raw_value=30000 scale=1000 tmin=0 tmax=100000
");
}

#[test]
fn counters_and_capture_times_track_samples() {
    let mut snap = RawStatsSnapshot::default();
    snap.set_fw_busy(1, 1, 100);
    snap.set_fw_busy(2, 0, 200);
    let text = render(&snap);
    assert!(text.contains("fw_busy count=2 captured_ns=200 timestamp=2 busy=0\n"), "{}", text);
    // The sample count of a kind that was never received stays at zero.
    assert!(text.contains("power count=0\n"), "{}", text);
    assert!(text.contains("messages count=0\n"), "{}", text);
}

#[test]
fn no_metric_semantics_are_invented() {
    let mut snap = RawStatsSnapshot::default();
    snap.set_fw_busy(1, 1, 100);
    snap.set_utilization(1, [10, 20, 30, 40], 100);
    let text = render(&snap);
    // The firmware busy flag is not a GPU utilisation figure and the raw
    // counters keep their own names; no scaled or renamed metric appears.
    for forbidden in ["gpu_busy", "util_pct", "utilization_pct", "power_mw", "temp_c", "freq_mhz"] {
        assert!(!text.contains(forbidden), "{} in {}", forbidden, text);
    }
    assert!(text.contains("fw_busy count=1 captured_ns=100 timestamp=1 busy=1\n"), "{}", text);
}
''',
])

with tempfile.TemporaryDirectory(prefix="asahi-stats-export-") as tmp:
    source_path = Path(tmp) / "test.rs"
    binary = Path(tmp) / "test"
    source_path.write_text(source)
    subprocess.run(["rustc", "--edition=2021", "--test", str(source_path),
                    "-o", str(binary)], check=True)
    subprocess.run([str(binary)], check=True)
