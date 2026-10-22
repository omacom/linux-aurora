// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Tests the production decoder against power transitions, inter-ring ordering and overlap.
#[path = "../../drivers/gpu/drm/asahi/g17/load.rs"]
mod load;
use load::Load;
fn record(tag: u32, timestamp: u64) -> [u32; 18] {
    let mut w = [0; 18];
    w[0] = tag;
    w[1] = timestamp as u32;
    w[2] = (timestamp >> 32) as u32;
    w
}
fn util(timestamp: u64, values: [u32; 4]) -> [u32; 18] {
    let mut w = record(0x13, timestamp);
    w[3..7].copy_from_slice(&values);
    w
}
fn power(timestamp: u64, off: bool) -> [u32; 18] {
    let mut w = record(0x19, timestamp);
    w[5] = u32::from(!off);
    w[6] = u32::from(off);
    w
}
#[test]
fn aggregate_is_firmware_union_not_engine_sum() {
    let s = Load::new();
    assert!(s.snapshot().unwrap().current().is_none());
    // Captured mixed rendering/compute: engine activity overlaps.
    s.record(0, 1000, &util(100, [69, 93, 74, 95]));
    assert_eq!(s.snapshot().unwrap().current(), Some([69, 93, 74, 95]));
}
#[test]
fn power_off_clears_retained_load_and_wake_requires_a_fresh_sample() {
    let s = Load::new();
    s.record(0, 1, &util(100, [10, 90, 0, 90]));
    s.record(1, 2, &power(110, true));
    assert_eq!(s.snapshot().unwrap().current(), Some([0; 4]));
    s.record(1, 3, &power(120, false));
    assert!(s.snapshot().unwrap().current().is_none());
    s.record(0, 4, &util(130, [0, 0, 94, 94]));
    assert_eq!(s.snapshot().unwrap().current(), Some([0, 0, 94, 94]));
}
#[test]
fn late_old_ring_records_do_not_clear_newer_work() {
    let s = Load::new();
    s.record(0, 10, &util(200, [0, 0, 90, 90]));
    s.record(1, 11, &power(100, true));
    assert_eq!(s.snapshot().unwrap().current(), Some([0, 0, 90, 90]));
    s.record(1, 12, &power(190, false));
    assert_eq!(s.snapshot().unwrap().current(), Some([0, 0, 90, 90]));
    s.record(1, 13, &power(150, true));
    assert_eq!(s.snapshot().unwrap().power_ticks, 190);
}
#[test]
fn invalid_samples_and_wrong_sources_cannot_replace_a_valid_sample() {
    let s = Load::new();
    s.record(0, 1, &util(100, [1, 20, 0, 20]));
    s.record(1, 2, &util(200, [2, 100, 0, 100]));
    s.record(0, 3, &power(200, true));
    s.record(0, 4, &util(300, [1, 101, 0, 20]));
    s.record(0, 5, &util(90, [1, 90, 0, 90]));
    let r = s.snapshot().unwrap();
    assert_eq!(r.samples, 1);
    assert_eq!(r.invalid, 1);
    assert_eq!(r.current(), Some([1, 20, 0, 20]));
}
#[test]
fn unwritten_tails_and_coprocessor_busy_are_not_gpu_utilization() {
    let s = Load::new();
    let mut w = util(100, [1, 20, 0, 20]);
    w[7..].fill(u32::MAX);
    s.record(0, 1, &w);
    let mut busy = record(0x1a, 200);
    busy[3] = 0;
    s.record(0, 2, &busy);
    s.record(1, 3, &busy);
    assert_eq!(s.snapshot().unwrap().current(), Some([1, 20, 0, 20]));
}
#[test]
fn timestamps_keep_all_64_bits_and_power_residency_is_separate_from_busy() {
    let s = Load::new();
    let timestamp = (1u64 << 32) + 100;
    s.record(0, 1, &util(timestamp, [0, 0, 25, 25]));
    s.record(1, 2, &record(0x11, 48_000_000));
    s.record(1, 3, &record(0x12, 24_000_000));
    let r = s.snapshot().unwrap();
    assert_eq!(r.ticks, timestamp);
    assert_eq!(r.powered_ticks, load::CLOCK_HZ);
    assert_eq!(r.unpowered_ticks, 2 * load::CLOCK_HZ);
    assert_eq!(r.current(), Some([0, 0, 25, 25]));
    s.record(1, 4, &record(0x12, u64::MAX));
    assert_eq!(s.snapshot().unwrap().powered_ticks, u64::MAX);
}
#[test]
fn concurrent_snapshots_are_consistent() {
    let s = Load::new();
    std::thread::scope(|scope| {
        scope.spawn(|| {
            for i in 1..=50_000 {
                let v = i % 101;
                s.record(0, i as u64, &util(i as u64, [v; 4]));
            }
        });
        for _ in 0..50_000 {
            if let Some(r) = s.snapshot() {
                if r.samples != 0 {
                    assert_eq!(r.received_ns, r.ticks);
                    assert_eq!(r.utilization, [(r.ticks % 101) as u32; 4]);
                }
            }
        }
    });
    assert_eq!(s.snapshot().unwrap().samples, 50_000);
}

#[test]
fn device_loss_stays_unavailable_even_when_queued_records_arrive() {
    let s = Load::new();
    s.record(0, 1, &util(100, [10, 90, 0, 90]));
    s.invalidate();
    assert!(s.snapshot().unwrap().current().is_none());
    s.record(1, 2, &power(200, true));
    s.record(0, 3, &util(300, [0, 0, 95, 95]));
    assert!(s.snapshot().unwrap().current().is_none());
}

#[test]
fn same_state_reports_refresh_the_witness_without_resetting_the_wake_epoch() {
    let s = Load::new();
    s.record(0, 1, &util(110, [0, 90, 0, 90]));
    s.record(1, 2, &power(100, true));
    assert_eq!(s.snapshot().unwrap().current(), Some([0, 90, 0, 90]));
    s.record(1, 3, &power(120, true));
    assert_eq!(s.snapshot().unwrap().current(), Some([0; 4]));
    s.record(1, 4, &power(130, false));
    s.record(1, 5, &power(150, false));
    s.record(0, 6, &util(140, [0, 20, 0, 20]));
    let r = s.snapshot().unwrap();
    assert_eq!(r.on_ticks, 130);
    assert_eq!(r.power_ticks, 150);
    assert_eq!(r.current(), Some([0, 20, 0, 20]));
}
#[test]
fn concurrent_repeated_power_reports_preserve_state_and_epoch_consistency() {
    let s = Load::new();
    std::thread::scope(|scope| {
        scope.spawn(|| {
            for i in 1..=20_000u64 {
                let base = i * 20;
                s.record(1, base, &power(base, false));
                s.record(0, base + 1, &util(base + 1, [1, 90, 0, 90]));
                for j in 2..10 {
                    s.record(1, base + j, &power(base + j, false));
                }
                s.record(1, base + 10, &power(base + 10, true));
                for j in 11..20 {
                    s.record(1, base + j, &power(base + j, true));
                }
            }
        });
        for _ in 0..100_000 {
            if let Some(r) = s.snapshot() {
                if r.power_known {
                    assert!(r.power_ticks >= r.on_ticks);
                }
                if r.power_known && !r.powered {
                    assert_eq!(r.current(), Some([0; 4]));
                }
                if let Some(v) = r.current() {
                    assert!(v == [0; 4] || v == [1, 90, 0, 90]);
                }
            }
        }
    });
}

#[test]
fn unrelated_power_payload_words_do_not_change_decoding() {
    let s = Load::new();
    let mut w = power(100, true);
    w[5] = u32::MAX;
    s.record(1, 1, &w);
    assert_eq!(s.snapshot().unwrap().current(), Some([0; 4]));
    assert_eq!(s.snapshot().unwrap().invalid, 0);
}
