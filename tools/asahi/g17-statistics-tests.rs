// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Host tests for the production firmware-report snapshot, including concurrent readers.

#[path = "../../drivers/gpu/drm/asahi/g17/load.rs"]
mod load;

#[path = "../../drivers/gpu/drm/asahi/g17/statistics.rs"]
mod statistics;

use statistics::{Statistics, WORDS};

#[test]
fn retains_full_payload_and_separates_sources() {
    let stats = Statistics::new();
    assert_eq!(stats.snapshot(0, 0x14).unwrap().count, 0);
    let mut words = core::array::from_fn(|i| i as u32);
    words[0] = 0x14;
    stats.record(0, 1234, &words);
    let report = stats.snapshot(0, 0x14).unwrap();
    assert_eq!(report.words, words);
    assert_eq!(report.received_ns, 1234);
    assert_eq!(report.count, 1);
    assert_eq!(stats.snapshot(1, 0x14).unwrap().count, 0);
    assert_eq!(stats.snapshot(0, 0x19).unwrap().count, 0);
    words[17] = u32::MAX;
    stats.record(0, 4567, &words);
    assert_eq!(stats.snapshot(0, 0x14).unwrap().count, 2);
    assert_eq!(stats.snapshot(0, 0x14).unwrap().words, words);
}

#[test]
fn rejects_out_of_range_sources_without_aliasing() {
    let stats = Statistics::new();
    for tag in [0, 15, 31, u32::MAX] {
        let mut words = [0; WORDS];
        words[0] = tag;
        stats.record(0, 1, &words);
        assert!(stats.snapshot(0, tag).is_none());
    }
    let mut words = [0; WORDS];
    words[0] = 0x14;
    stats.record(2, 1, &words);
    assert!(stats.snapshot(2, 0x14).is_none());
    assert_eq!(stats.snapshot(0, 0x14).unwrap().count, 0);
}

#[test]
fn readers_never_observe_a_torn_report() {
    let stats = Statistics::new();
    std::thread::scope(|scope| {
        scope.spawn(|| {
            for sequence in 1..=50_000 {
                let mut words = [sequence; WORDS];
                words[0] = 0x14;
                stats.record(0, sequence as u64, &words);
            }
        });
        for _ in 0..50_000 {
            if let Some(report) = stats.snapshot(0, 0x14) {
                if report.count != 0 {
                    assert_eq!(report.received_ns, report.count);
                    assert_eq!(report.words[0], 0x14);
                    assert!(report.words[1..]
                        .iter()
                        .all(|&word| word as u64 == report.count));
                }
            }
        }
    });
    assert_eq!(stats.snapshot(0, 0x14).unwrap().count, 50_000);
}

#[test]
fn full_payload_capture_is_requested_explicitly_and_expires() {
    let stats = Statistics::new();
    assert!(!stats.capture_pending());
    assert!(!stats.capture_enabled(1));
    stats.request_capture(100);
    assert!(stats.capture_pending());
    assert!(stats.capture_enabled(101));
    stats.request_capture(200);
    assert!(stats.capture_enabled(100 + statistics::CAPTURE_NS));
    assert!(!stats.capture_enabled(200 + statistics::CAPTURE_NS));
    assert!(!stats.capture_pending());
}
