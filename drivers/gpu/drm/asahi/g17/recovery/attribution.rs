// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Resolve ambiguous recovery blame using slot identity and render execution evidence.

pub(super) const SLOTS: usize = 16;
const STAMP_MASK: u64 = (1 << 40) - 1;
/// Firmware recovery reason for a GPU MMU fault.
pub(crate) const PAGE_FAULT: u32 = 3;
/// M5 firmware recovery reason for a scheduling pause the GPU did not acknowledge in time. It was
/// read from every recovery of a ten-minute mixed client load on a J716s with the 27.3 beta 1
/// firmware.
pub(crate) const SCHEDULING_PAUSE_TIMEOUT: u32 = 5;
/// M5 firmware recovery reason for a GPU that did not reach forced idle after a sleep request.
/// Not yet observed.
pub(crate) const SLEEP_FORCED_IDLE_TIMEOUT: u32 = 6;

/// What a firmware recovery reason means: 3 a GPU page fault (as on G17P), 5 a scheduling pause
/// that timed out, 6 a forced idle that timed out after a sleep request. Reasons 0 to 4 are not
/// named here. Only 5 has been read from a running M5 firmware; 3 and 6 are
/// not yet confirmed from captured InitData.
pub(crate) fn cause(reason: u32) -> &'static str {
    match reason {
        PAGE_FAULT => "GPU MMU page fault",
        SCHEDULING_PAUSE_TIMEOUT => {
            "scheduling pause timeout: the GPU did not pause scheduling in time"
        }
        SLEEP_FORCED_IDLE_TIMEOUT => "sleep forced-idle timeout",
        0..=4 => "unnamed firmware reason",
        _ => "unknown firmware reason",
    }
}

/// The M5 firmware's recovery record in the primary status B (`M5_RECOVERY_REASON` and its
/// neighbours), read while the firmware waits in recovery state 1.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct Record {
    pub(crate) reason: u32,
    pub(crate) slots_walked: u32,
    pub(crate) slot_word: u32,
    pub(crate) detail: u64,
}

#[derive(Clone, Copy, Default)]
pub(super) struct Source {
    slot: u8,
    qid: Option<u8>,
    stamp: u64,
    monitor: u32,
    started: Option<bool>,
}

pub(crate) struct Sources {
    /// The reason attribution uses. M5 leaves it unknown unless an MMU fault is latched; its
    /// firmware's own word is in `record`, which only the log reads.
    pub(crate) reason: Option<u32>,
    /// The M5 firmware's recovery record, for the log.
    pub(crate) record: Option<Record>,
    mask: u32,
    records: [Source; SLOTS],
    count: usize,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Attribution {
    Firmware,
    Queues(u128),
    NoCulprit,
}

impl Sources {
    /// Bounded evidence from the existing halted-firmware snapshot. No guessed
    /// MMIO/key addresses or additional firmware reads are used for diagnostics.
    pub(super) fn report(&self, generation: u64, stampless_total: u64, blamed: Option<u8>) {
        kernel::pr_warn!(
            "G17: recovery snapshot gen {} reason {:?} slot_mask {:#x} sampled {} known_keys {}; stampless lifetime total {} before restart\n",
            generation, self.reason, self.mask, self.count,
            self.records[..self.count].iter().filter(|source| source.qid.is_some()).count(),
            stampless_total
        );
        match self.record {
            Some(record) if record == Record::default() && self.mask == 0 => kernel::pr_warn!(
                "G17: recovery cause gen {}: none recorded (the record reads all zero); firmware blames queue {:?}\n",
                generation, blamed
            ),
            Some(record) => kernel::pr_warn!(
                "G17: recovery cause gen {}: {} (firmware reason {}); firmware blames queue {:?}; slots walked {}, slot word {:#x}, detail {:#x}\n",
                generation, cause(record.reason), record.reason, blamed, record.slots_walked,
                record.slot_word, record.detail
            ),
            None => {}
        }
        for source in &self.records[..self.count] {
            kernel::pr_warn!(
                "G17: recovery slot {} queue {:?} kick {:#x} monitor {:#x}\n",
                source.slot, source.qid, source.stamp, source.monitor
            );
        }
    }

    /// Firmware is halted for every selected key and progress-record read.
    /// Failed reads stay unknown; incomplete evidence cannot override firmware blame.
    pub(crate) fn sample(mask: u32, mut read: impl FnMut(usize) -> Option<(u64, u32)>) -> Self {
        let mut sources = Self {
            reason: None,
            record: None,
            mask,
            records: [Source::default(); SLOTS],
            count: 0,
        };
        for slot in 0..SLOTS {
            if mask & (1 << slot) == 0 {
                continue;
            }
            let Some((key, monitor)) = read(slot) else {
                continue;
            };
            let qid = (key >> 40) & 0xff;
            sources.records[sources.count] = Source {
                slot: slot as u8,
                qid: (key != 0 && qid < 128).then_some(qid as u8),
                stamp: key & STAMP_MASK,
                monitor,
                started: None,
            };
            sources.count += 1;
        }
        sources
    }

    /// The firmware's reason: the one attribution uses, else the M5 record's.
    pub(super) fn firmware_reason(&self) -> Option<u32> {
        self.reason.or(self.record.map(|record| record.reason))
    }

    pub(super) fn attribute(
        &mut self,
        blamed: Option<u8>,
        mut started: impl FnMut(u8, u64) -> Option<bool>,
    ) -> Attribution {
        if self.reason.is_none()
            || self.count as u32 != self.mask.count_ones()
            || self.records[..self.count].iter().any(|source| source.qid.is_none())
        {
            return Attribution::Firmware;
        }
        for source in &mut self.records[..self.count] {
            if let Some(qid) = source.qid {
                source.started = started(qid, source.stamp);
            }
        }
        if self.reason == Some(PAGE_FAULT) || self.mask.count_ones() < 2 {
            return Attribution::Firmware;
        }
        let Some(blamed) = blamed else {
            return Attribution::Firmware;
        };
        let Some(source) = self.records[..self.count]
            .iter()
            .find(|source| source.qid == Some(blamed))
        else {
            return Attribution::Firmware;
        };
        if source.started != Some(false) || source.monitor != 0 {
            return Attribution::Firmware;
        }
        let slot = source.slot;
        let mut guilty = 0;
        for source in &self.records[..self.count] {
            if source.slot != slot && source.started == Some(true) {
                if let Some(qid) = source.qid {
                    guilty |= 1u128 << qid;
                }
            }
        }
        if guilty == 0 {
            Attribution::NoCulprit
        } else {
            Attribution::Queues(guilty)
        }
    }

    /// When slot identity cannot check a timeout's blame (25G83 reads every key as 0, and a
    /// single-slot mask has no peer), the host's own pass records still can: a blamed render
    /// that never started was starved, not hung. `culprit` names the queues of the one VM whose
    /// render started and did not finish; without one the firmware's blame stands.
    pub(super) fn host_fallback(
        &self,
        attribution: Attribution,
        culprit: impl FnOnce() -> Option<u128>,
    ) -> Attribution {
        if attribution != Attribution::Firmware || self.reason.is_none_or(|reason| reason == PAGE_FAULT) {
            return attribution;
        }
        culprit().map_or(Attribution::Firmware, Attribution::Queues)
    }
}

#[kernel::macros::kunit_tests(asahi_g17_recovery_sources)]
mod tests {
    use super::*;

    #[test]
    fn progress_without_keys_preserves_firmware_blame() {
        let mut reads = 0;
        let mut sources = Sources::sample(0b101, |slot| {
            reads += 1;
            Some((0, (slot + 1) as u32))
        });
        sources.reason = Some(2);
        assert_eq!(reads, 2);
        assert_eq!(sources.count, 2);
        assert_eq!(sources.records[1].slot, 2);
        assert_eq!(sources.records[1].monitor, 3);
        assert!(sources.records[..sources.count].iter().all(|source| source.qid.is_none()));
        assert_eq!(sources.attribute(Some(38), |_, _| panic!("unknown key")),
            Attribution::Firmware);
    }

    #[test]
    fn incomplete_progress_cannot_clear_firmware_blame() {
        let mut sources = Sources::sample(0b11, |slot| {
            (slot == 0).then_some(((38 << 40) | 1, 0))
        });
        sources.reason = Some(2);
        assert_eq!(sources.attribute(Some(38), |_, _| Some(false)), Attribution::Firmware);
    }

    #[test]
    fn a_missing_peer_key_cannot_clear_a_known_queues_blame() {
        let mut sources = Sources::sample(0b11, |slot| {
            Some((if slot == 0 { (38 << 40) | 1 } else { 0 }, 0))
        });
        sources.reason = Some(2);
        assert_eq!(sources.count, 2);
        assert_eq!(sources.attribute(Some(38), |_, _| Some(false)), Attribution::Firmware);
    }

    #[test]
    fn page_fault_keeps_firmware_blame_with_complete_keys() {
        let mut sources = Sources::sample(0b11, |slot| {
            Some(((((38 + slot) as u64) << 40) | 1, 0))
        });
        sources.reason = Some(PAGE_FAULT);
        assert_eq!(sources.attribute(Some(38), |qid, _| Some(qid == 39)),
            Attribution::Firmware);
    }

    #[test]
    fn host_records_move_a_keyless_timeout_blame_to_the_running_vm() {
        let mut sources = Sources::sample(0b10, |_| Some((0, 2)));
        sources.reason = Some(0);
        let attribution = sources.attribute(Some(36), |_, _| panic!("unknown key"));
        assert_eq!(attribution, Attribution::Firmware);
        assert_eq!(sources.host_fallback(attribution, || Some(3 << 60)), Attribution::Queues(3 << 60));
        assert_eq!(sources.host_fallback(attribution, || None), Attribution::Firmware);
    }

    #[test]
    fn m5_record_names_the_cause_without_steering_blame() {
        let mut sources = Sources::sample(0b101, |_| None);
        sources.record = Some(Record {
            reason: SCHEDULING_PAUSE_TIMEOUT,
            ..Record::default()
        });
        assert_eq!(sources.firmware_reason(), Some(SCHEDULING_PAUSE_TIMEOUT));
        assert_eq!(
            cause(SCHEDULING_PAUSE_TIMEOUT),
            "scheduling pause timeout: the GPU did not pause scheduling in time"
        );
        assert_eq!(
            sources.attribute(Some(38), |_, _| panic!("no keys")),
            Attribution::Firmware
        );
        assert_eq!(
            sources.host_fallback(Attribution::Firmware, || panic!("reason unused")),
            Attribution::Firmware
        );
    }

    #[test]
    fn host_records_never_override_a_page_fault_or_slot_evidence() {
        let mut sources = Sources::sample(0b10, |_| Some((0, 2)));
        sources.reason = Some(PAGE_FAULT);
        assert_eq!(sources.host_fallback(Attribution::Firmware, || panic!("page fault")),
            Attribution::Firmware);
        sources.reason = None;
        assert_eq!(sources.host_fallback(Attribution::Firmware, || panic!("no reason")),
            Attribution::Firmware);
        sources.reason = Some(0);
        assert_eq!(sources.host_fallback(Attribution::NoCulprit, || panic!("slot evidence")),
            Attribution::NoCulprit);
    }
}
