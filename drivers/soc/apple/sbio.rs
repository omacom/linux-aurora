// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj
// Copyright 2026 Ryan Murray
//! Touch ID: the biometric endpoint (SBIO) and its `/dev/sep-bio` interface.
//! The enclave matches; no biometric image ever crosses to userspace.

use super::*;
use crate::proto::*;
use crate::sks::SKS_AUTH_TOKEN_LEN;
use kernel::prelude::*;
use kernel::soc::apple::mailbox::Message;

enum MatchStep {
    Answer(KVec<u8>),
    // 0x1: the match credential no longer holds; see `enter_lockout`.
    LockedOut,
    Refused,
}

/// The Touch ID lockout, kept across a restart. Labelled with the driver's
/// other state files (aurora_sep_state_t).
const LOCKOUT_RECORD: &CStr = c"/var/lib/aurora-sep-touchid-lockout.bin";
const LOCKOUT_MAGIC: [u8; 8] = *b"SEPLOCK1";
const LOCKOUT_RECORD_LEN: usize = 16;
const LOCKOUT_LOCKED: u32 = 1;

/// Message tag of the enclave's unsolicited biometric lock-state notification.
const SBIO_NOTIFY_LOCK_STATE: u8 = 0x01;
/// Bits of the user's biometric lock state, as the enclave announces it.
/// The user's password (passcode) is required before biometry.
const LOCK_STATE_PASSCODE_LOCKOUT: u16 = 0x0002;
/// Biometry is locked for the user; set with the identification lockout.
const LOCK_STATE_BIO_LOCKOUT: u16 = 0x0004;
/// The user holds a biometric unlock token.
const LOCK_STATE_UNLOCK_TOKEN: u16 = 0x0008;
/// The user has not unlocked with the password since the enclave started.
const LOCK_STATE_BEFORE_FIRST_UNLOCK: u16 = 0x0010;
/// A password was validated recently.
const LOCK_STATE_PASSCODE_VALIDATED: u16 = 0x0020;
/// Identification is locked: five completed no-matches in a row.
const LOCK_STATE_LOCKOUT: u16 = 0x0040;
/// The user's catacomb is damaged; matching cannot work until it is repaired.
const LOCK_STATE_CATACOMB_CORRUPTED: u16 = 0x0080;
/// The states in which the enclave will not match until the user's password
/// has been verified.
const LOCK_STATE_NEEDS_PASSWORD: u16 = LOCK_STATE_PASSCODE_LOCKOUT
    | LOCK_STATE_BIO_LOCKOUT
    | LOCK_STATE_BEFORE_FIRST_UNLOCK
    | LOCK_STATE_LOCKOUT;
/// Message tag of the enclave's notice that a user's enrolled-identity
/// count changed (the count in bits 16-31, the user in bits 32-63).
const SBIO_NOTIFY_ENROLLED_COUNT: u8 = 0x02;
/// Message tag of the enclave's notice that its device list changed.
const SBIO_NOTIFY_DEVICE_LIST: u8 = 0x04;

/// The named bits of a lock state, for the log.
fn lock_state_names(state: u16) -> [&'static str; 7] {
    let name = |bit: u16, what: &'static str| if state & bit != 0 { what } else { "" };
    [
        name(LOCK_STATE_PASSCODE_LOCKOUT, " password-lockout"),
        name(LOCK_STATE_BIO_LOCKOUT, " biometry-lockout"),
        name(LOCK_STATE_UNLOCK_TOKEN, " unlock-token"),
        name(LOCK_STATE_BEFORE_FIRST_UNLOCK, " before-first-unlock"),
        name(LOCK_STATE_PASSCODE_VALIDATED, " password-validated"),
        name(LOCK_STATE_LOCKOUT, " identification-lockout"),
        name(LOCK_STATE_CATACOMB_CORRUPTED, " catacomb-corrupted"),
    ]
}
/// Failed matches in one prompt after which the prompt's match operation
/// ends: macOS LocalAuthentication stops a Touch ID prompt after three NoMatch
/// and offers the password, while the enclave stays unlocked and the next
/// prompt offers Touch ID again.
const PROMPT_RETRY_LIMIT: u32 = 3;
/// Failed matches since the last match or password re-arm after which Touch
/// ID is locked until a password authentication: the enclave's own limit
/// (a macOS lock screen got five completed no-matches, then the user's lock
/// state gained the lockout bit) and the documented "five unsuccessful
/// attempts to match".
const VERIFY_RETRY_LIMIT: u32 = 5;

/// What a verify's outcome does to the prompt's match operation.
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum PromptStep {
    /// The prompt continues: its match operation stays open for the next touch.
    KeepOpen,
    /// The prompt is over (a match, an error, or its retry limit): end the
    /// match operation.
    End,
}

/// Decides the prompt's next step. `no_match` and `capture_retry` describe
/// this touch (a compared no-match; a touch that gave no usable frame);
/// `prompt_misses` and `total_misses` already count it.
fn prompt_step(
    no_match: bool,
    capture_retry: bool,
    prompt_misses: u32,
    total_misses: u32,
) -> PromptStep {
    if capture_retry {
        // Nothing was compared: macOS keeps imaging in the same operation.
        return PromptStep::KeepOpen;
    }
    if no_match && prompt_misses < PROMPT_RETRY_LIMIT && total_misses < VERIFY_RETRY_LIMIT {
        PromptStep::KeepOpen
    } else {
        PromptStep::End
    }
}
/// Further frames of a no-match touch relayed before the retry pass ends.
const NOMATCH_EXTRA_FRAMES: u32 = 3;
/// How long the no-match tail waits for each extra frame of the same touch.
const NOMATCH_FRAME_WAIT_MS: u64 = 500;
/// How long the tail waits on a sensor armed for a finger: one still down is
/// imaged within a frame period or two (~80 ms each, ~125 ms measured from a
/// START_CAPTURE on Linux).
const NOMATCH_ARMED_WAIT_MS: u64 = 250;
/// Message tag of the enclave's "background operation result ready" notice.
/// The host answers it with BG_OPS_RESULT (0x21); until it does, the result
/// stays queued in the enclave.
const SBIO_NOTIFY_BG_OPS_READY: u8 = 0x03;
/// How long a verify waits after its result for that notice.
const BG_OPS_WAIT_MS: u32 = 1000;

/// What collecting an announced background result led to.
enum BgOps {
    Nothing,
    NotUpdated,
    TemplatesUpdated,
    Failed,
}
/// How long to wait after a background result for a component to need saving.
const BG_OPS_SAVE_WAIT_MS: u32 = 10_000;
const BG_OPS_SAVE_POLL_MS: u32 = 50;

/// How long a re-arm waits for the enclave to announce the cleared state.
const LOCK_STATE_WAIT_MS: u32 = 1000;
const LOCK_STATE_POLL_MS: u32 = 10;

/// The record: magic, the lock (`LOCKOUT_LOCKED` or 0), and the failed
/// matches since the last match or password. A restart does not clear
/// failed attempts on macOS (the enclave reloads its saved count), so the
/// host keeps its count across one too.
fn lockout_record(locked: bool, failures: u32) -> [u8; LOCKOUT_RECORD_LEN] {
    let mut raw = [0u8; LOCKOUT_RECORD_LEN];
    raw[..LOCKOUT_MAGIC.len()].copy_from_slice(&LOCKOUT_MAGIC);
    let state = if locked { LOCKOUT_LOCKED } else { 0 };
    raw[8..12].copy_from_slice(&state.to_le_bytes());
    raw[12..16].copy_from_slice(&failures.to_le_bytes());
    raw
}

/// The failed matches a record holds, at most `VERIFY_RETRY_LIMIT`; 0 for a
/// foreign record.
fn lockout_record_failures(raw: &[u8; LOCKOUT_RECORD_LEN]) -> u32 {
    if raw[..LOCKOUT_MAGIC.len()] != LOCKOUT_MAGIC {
        return 0;
    }
    u32::from_le_bytes([raw[12], raw[13], raw[14], raw[15]]).min(VERIFY_RETRY_LIMIT)
}

/// Whether a lockout follows too many failed matches: `VERIFY_RETRY_LIMIT`
/// of them since the last match or password, or the enclave's own lockout
/// after failed matches (which macOS can have caused). Otherwise the lockout
/// needs the password for another reason, such as a lapsed match credential.
fn lockout_after_failures(failures: u32, enclave_lockout: bool) -> bool {
    failures >= VERIFY_RETRY_LIMIT || enclave_lockout
}

fn lockout_record_locked(raw: &[u8; LOCKOUT_RECORD_LEN]) -> bool {
    raw[..LOCKOUT_MAGIC.len()] == LOCKOUT_MAGIC
        && u32::from_le_bytes([raw[8], raw[9], raw[10], raw[11]]) == LOCKOUT_LOCKED
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_touchid_lockout)]
mod lockout_tests {
    use super::*;

    #[test]
    fn prompt_limit_ends_the_operation_not_touch_id() {
        assert_eq!(prompt_step(true, false, 1, 1), PromptStep::KeepOpen);
        assert_eq!(prompt_step(true, false, 2, 2), PromptStep::KeepOpen);
        assert_eq!(prompt_step(true, false, 3, 3), PromptStep::End);
        // A new prompt after three misses offers Touch ID again ...
        assert_eq!(prompt_step(true, false, 1, 4), PromptStep::KeepOpen);
        // ... until the fifth miss since the last match or password.
        assert_eq!(prompt_step(true, false, 2, 5), PromptStep::End);
        assert!(VERIFY_RETRY_LIMIT > PROMPT_RETRY_LIMIT);
    }

    #[test]
    fn unusable_touch_keeps_the_prompt() {
        assert_eq!(prompt_step(false, true, 0, 0), PromptStep::KeepOpen);
        assert_eq!(prompt_step(false, true, 2, 4), PromptStep::KeepOpen);
        // A match or an error ends it.
        assert_eq!(prompt_step(false, false, 0, 0), PromptStep::End);
    }

    #[test]
    fn after_boot_wait_reports_locked() {
        use crate::bio::{Lockout, LOCKOUT_FLAG_AFTER_BOOT, LOCKOUT_FLAG_EVERY_PASSWORD};
        let plain = Lockout::new(false, false, false, 2);
        assert_eq!(plain.locked, 0);
        assert_eq!(plain.failures, 2);
        assert_eq!(plain.flags, LOCKOUT_FLAG_EVERY_PASSWORD);
        let waiting = Lockout::new(false, true, false, 0);
        assert_eq!(waiting.locked, 1);
        assert_eq!(waiting.flags, LOCKOUT_FLAG_EVERY_PASSWORD | LOCKOUT_FLAG_AFTER_BOOT);
        // A lockout and the wait together: one password ends both.
        let both = Lockout::new(true, true, true, 5);
        assert_eq!(both.locked, 1);
        assert_ne!(both.flags & LOCKOUT_FLAG_AFTER_BOOT, 0);
        let locked = Lockout::new(true, false, true, 5);
        assert_eq!(locked.locked, 1);
        assert_eq!(locked.flags & LOCKOUT_FLAG_AFTER_BOOT, 0);
    }

    #[test]
    fn lockout_names_failed_attempts() {
        use crate::bio::{Lockout, LOCKOUT_FLAG_FAILED_ATTEMPTS};
        // Five misses, on the host or in the enclave, are the real lockout.
        assert!(lockout_after_failures(VERIFY_RETRY_LIMIT, false));
        assert!(lockout_after_failures(0, true));
        // Fewer misses and no lockout in the enclave: the password is needed
        // for another reason (a lapsed credential, a refused match context).
        assert!(!lockout_after_failures(0, false));
        assert!(!lockout_after_failures(VERIFY_RETRY_LIMIT - 1, false));
        let locked = Lockout::new(true, false, true, VERIFY_RETRY_LIMIT);
        assert_ne!(locked.flags & LOCKOUT_FLAG_FAILED_ATTEMPTS, 0);
        let lapsed = Lockout::new(true, false, false, 0);
        assert_eq!(lapsed.locked, 1);
        assert_eq!(lapsed.flags & LOCKOUT_FLAG_FAILED_ATTEMPTS, 0);
        // Only a lockout carries the flag; failed matches alone do not.
        let open = Lockout::new(false, false, true, VERIFY_RETRY_LIMIT - 1);
        assert_eq!(open.flags & LOCKOUT_FLAG_FAILED_ATTEMPTS, 0);
        // The wait after boot is not a lockout after failed matches.
        let waiting = Lockout::new(false, true, false, 0);
        assert_eq!(waiting.flags & LOCKOUT_FLAG_FAILED_ATTEMPTS, 0);
    }

    #[test]
    fn record_round_trips() {
        assert!(lockout_record_locked(&lockout_record(true, 5)));
        assert!(!lockout_record_locked(&lockout_record(false, 0)));
        assert_eq!(lockout_record_failures(&lockout_record(false, 3)), 3);
        assert_eq!(lockout_record_failures(&lockout_record(true, 5)), 5);
        // A count beyond the limit is clamped.
        assert_eq!(lockout_record_failures(&lockout_record(false, 1000)), VERIFY_RETRY_LIMIT);
        // A record written before the count existed holds zero there.
        let mut old = lockout_record(true, 0);
        old[12..16].fill(0);
        assert_eq!(lockout_record_failures(&old), 0);
        assert!(lockout_record_locked(&old));
    }

    #[test]
    fn assessment_reasons() {
        use crate::sbio::assessment_reason;
        let mut a = [0u8; 238];
        assert_eq!(assessment_reason(&a).0, "too little of the finger, or too light a touch");
        assert!(assessment_reason(&a).1 == crate::bio::Guidance::Center);
        a[14..18].copy_from_slice(&205u32.to_le_bytes());
        assert_eq!(assessment_reason(&a).0, "the finger moved (hold it still)");
        assert!(assessment_reason(&a).1 == crate::bio::Guidance::TooFast);
        a[40..44].copy_from_slice(&1u32.to_le_bytes());
        assert_eq!(assessment_reason(&a).0, "a wet finger (dry it)");
        a[64] = 1;
        assert_eq!(assessment_reason(&a).0, "dirt on the sensor (clean it)");
        assert!(assessment_reason(&a).1 == crate::bio::Guidance::Clean);
        let short = assessment_reason(&[0u8; 8]).0;
        assert_eq!(short, "too little of the finger, or too light a touch");
    }

    #[test]
    fn lock_state_bits() {
        // 0x0045: identification lockout, biometry lockout, device locked.
        assert_ne!(0x0045 & LOCK_STATE_NEEDS_PASSWORD, 0);
        // 0x0228 / 0x0208 / 0x0209: usable (unlock token, no lockout).
        for usable in [0x0228u16, 0x0208, 0x0209, 0x0229] {
            assert_eq!(usable & LOCK_STATE_NEEDS_PASSWORD, 0);
            assert_eq!(usable & LOCK_STATE_CATACOMB_CORRUPTED, 0);
        }
        // 0x0055 and 0x0007 need the password too.
        assert_ne!(0x0055 & LOCK_STATE_NEEDS_PASSWORD, 0);
        assert_ne!(0x0007 & LOCK_STATE_NEEDS_PASSWORD, 0);
        assert_eq!(lock_state_names(0x0228)[2], " unlock-token");
    }

    #[test]
    fn foreign_bytes_are_no_lockout() {
        let mut raw = lockout_record(true, 5);
        raw[0] ^= 0xff;
        assert!(!lockout_record_locked(&raw));
        assert_eq!(lockout_record_failures(&raw), 0);
        assert!(!lockout_record_locked(&[0u8; LOCKOUT_RECORD_LEN]));
        let mut other = lockout_record(true, 5);
        other[8] = 2;
        assert!(!lockout_record_locked(&other));
    }

    #[test]
    fn verify_password_reply_shape() {
        let mut body = [0u8; crate::sks::SKS_VERIFY_SECRET_REPLY_LEN];
        body[0] = 1;
        assert_eq!(crate::sks::verify_secret_reply_flags(&body), Some(0));
        body[4] = 3;
        assert_eq!(crate::sks::verify_secret_reply_flags(&body), Some(3));
        let mut wrong_variant = body;
        wrong_variant[0] = 0;
        assert_eq!(crate::sks::verify_secret_reply_flags(&wrong_variant), None);
        assert_eq!(crate::sks::verify_secret_reply_flags(&body[..8]), None);
        assert_eq!(crate::sks::verify_secret_reply_flags(&[1u8; 16]), None);
    }

    #[test]
    fn unlock_evaluation_wire() {
        use crate::scrd::*;
        let handle = [0x5au8; SCRD_ACM_HANDLE_LEN];
        let pre = scrd_verify_unlock_policy(&handle, 1000, true);
        let p = pre.payload();
        assert_eq!(p.len(), 0x55);
        assert_eq!(&p[0..4], b"DRCS");
        assert_eq!(p[4], 3);
        assert_eq!(&p[8..24], &handle);
        assert_eq!(&p[24..63], b"DeviceOwnerAuthenticationWithBiometrics");
        assert_eq!(p[63], 0);
        assert_eq!(p[64], 1);
        assert_eq!(&p[65..69], &[0u8; 4]);
        assert_eq!(&p[69..73], &1u32.to_le_bytes());
        assert_eq!(&p[73..77], &4u32.to_le_bytes());
        assert_eq!(&p[77..81], &4u32.to_le_bytes());
        assert_eq!(&p[81..85], &1000u32.to_le_bytes());
        let fin = scrd_verify_unlock_policy(&handle, 1000, false);
        assert_eq!(fin.payload()[64], 0);
        assert_eq!(fin.payload().len(), 0x55);
        assert_eq!(scrd_verify_touchid_enrollment(&handle).payload().len(), 0x33);
        let del = scrd_context_delete(&handle);
        assert_eq!(del.payload().len(), 24);
        assert_eq!(del.payload()[4], 2);

        let mut created = [0u8; SCRD_TRACKED_CREATE_REPLY_LEN];
        created[..16].copy_from_slice(&handle);
        created[20] = 1;
        assert_eq!(tracked_create_reference(&created), Some(handle));
        created[20] = 2;
        assert_eq!(tracked_create_reference(&created), None);
        assert_eq!(tracked_create_reference(&created[..16]), None);

        let mut answer = [0u8; 20];
        answer[4..8].copy_from_slice(&3u32.to_le_bytes());
        answer[8..12].copy_from_slice(&1u32.to_le_bytes());
        assert_eq!(
            parse_policy_answer(&answer),
            Some(PolicyAnswer { satisfied: false, requirement: Some((3, 1, 0)) })
        );
        assert_eq!(
            parse_policy_answer(&1u32.to_le_bytes()),
            Some(PolicyAnswer { satisfied: true, requirement: None })
        );
        assert_eq!(parse_policy_answer(&2u32.to_le_bytes()), None);
        answer[16] = 1;
        assert_eq!(parse_policy_answer(&answer), None);
    }

    #[test]
    fn bg_ops_result_discriminators() {
        let mut r = [0u8; SBIO_BG_OPS_RESULT_LEN];
        assert_eq!(bg_ops_result_kind(&r), BgOpsResult::NoResult);
        r[0] = 1;
        assert_eq!(bg_ops_result_kind(&r), BgOpsResult::TemplateListNotUpdated);
        r[2] = 1;
        assert_eq!(bg_ops_result_kind(&r), BgOpsResult::TemplatesUpdated);
        r[0] = 0;
        assert_eq!(bg_ops_result_kind(&r), BgOpsResult::NoResult);
        assert_eq!(bg_ops_result_kind(&[1u8]), BgOpsResult::NoResult);
    }

    #[test]
    fn operation_device_list_is_empty_and_checked() {
        let op = sbio_operation_device_list();
        assert_eq!(op.opcode(), 0x7d);
        assert_eq!(op.payload().len(), 0);
        assert_eq!(operation_device_count(&[0, 0, 0, 0]), Some(0));
        let mut one = [0u8; 28];
        one[0] = 1;
        assert_eq!(operation_device_count(&one), Some(1));
        assert_eq!(operation_device_count(&one[..27]), None);
        assert_eq!(operation_device_count(&[2, 0, 0, 0]), None);
        assert_eq!(operation_device_count(&[0, 0, 0]), None);
        assert_eq!(operation_device_count(&[0xff, 0xff, 0xff, 0xff]), None);
    }

    #[test]
    fn no_match_tail_requests_are_empty() {
        assert_eq!(sbio_retry_capture_info().opcode(), 0x3e);
        assert!(sbio_retry_capture_info().payload().is_empty());
        assert_eq!(sbio_match_retry_finalize().opcode(), 0x0c);
        assert!(sbio_match_retry_finalize().payload().is_empty());
    }

    #[test]
    fn match_cancel_is_empty() {
        let op = sbio_match_cancel();
        assert_eq!(op.opcode(), 0x0b);
        assert_eq!(op.payload().len(), 0);
        assert_eq!(sbio_match_policy().opcode(), 0x1f);
    }

    #[test]
    fn protected_config_unlock_words() {
        let mut raw = [0u8; SBIO_PROTECTED_CONFIG_LEN];
        raw[0..4].copy_from_slice(&1i32.to_le_bytes());
        raw[0x10..0x14].copy_from_slice(&(-1i32).to_le_bytes());
        let config = ProtectedConfig::parse(&raw).unwrap();
        assert_eq!(config.unlock_set, 1);
        assert_eq!(config.unlock_effective, -1);
        assert!(!config.allows_unlock());
        raw[0x10..0x14].copy_from_slice(&1i32.to_le_bytes());
        assert!(ProtectedConfig::parse(&raw).unwrap().allows_unlock());
        assert!(ProtectedConfig::parse(&raw[..31]).is_none());
        assert_eq!(ProtectedConfig::describe(-1), "unspecified");
        assert_eq!(ProtectedConfig::describe(0), "disabled");
    }

    #[test]
    fn authenticated_match_carries_mode_and_context() {
        let user = UserId::new(1000).unwrap();
        let handle = [0xa5u8; crate::scrd::SCRD_ACM_HANDLE_LEN];
        let op = sbio_image_processing_init(ImagePurpose::Matching, true, true, 0, user, 0, Some(&handle));
        assert_eq!(&op.payload[IPI_MATCH_MODE..IPI_MATCH_MODE + 4], &SBIO_MATCH_MODE_ACM.to_le_bytes());
        // The LocalAuthentication class: credential set, no keybag-unlock bit.
        assert_eq!(SBIO_MATCH_MODE_ACM, 0x88);
        assert_eq!(&op.payload[IPI_USER_ID..IPI_USER_ID + 4], &1000i32.to_le_bytes());
        assert_eq!(&op.payload[IPI_CREDENTIAL_LEN..IPI_CREDENTIAL_LEN + 4], &16u32.to_le_bytes());
        assert_eq!(&op.payload[0x97..0xa7], &handle);
        assert_eq!(op.payload().len(), 0xa7);
        // The fixed token slot belongs to another mode and stays empty.
        assert!(op.payload[0x83..0x93].iter().all(|b| *b == 0));
        let plain = sbio_image_processing_init(ImagePurpose::Enrolment, true, false, 0, user, 0, None);
        assert_eq!(&plain.payload[IPI_MATCH_MODE..IPI_MATCH_MODE + 4], &[0u8; 4]);
        assert_eq!(&plain.payload[IPI_CREDENTIAL_LEN..IPI_CREDENTIAL_LEN + 4], &[0u8; 4]);
        assert_eq!(plain.payload().len(), 0x97);
    }
}

struct EnrolPrerequisites {
    lane: bool,
    live_count: Option<usize>,
    bag_bound: bool,
    catacomb_readable: [bool; 3],
    catacomb_writable: [bool; 3],
    keybag_writable: bool,
    host_store_capacity: bool,
}

fn admit_enrol(p: &EnrolPrerequisites) -> Result<()> {
    if !p.lane { return Err(ENXIO); }
    let count = p.live_count.ok_or(EIO)?;
    if count >= 3 { return Err(ENOSPC); }
    if !p.bag_bound || p.catacomb_readable.contains(&false) {
        return Err(ENODATA);
    }
    if p.catacomb_writable.contains(&false) || !p.keybag_writable {
        return Err(EROFS);
    }
    if !p.host_store_capacity { return Err(ENOSPC); }
    Ok(())
}

fn catacomb_needed_for_enrol(kind: u8, live_count: Option<usize>) -> bool {
    live_count != Some(0) || kind == PRIVATE_TYPE_CATACOMB_OWNER
}

fn admissible_orphan_target(
    listed_linux: &[[u8; bio::UUID_LEN]],
    uuid: &[u8; bio::UUID_LEN],
) -> Result<()> {
    if *uuid == [0; bio::UUID_LEN] { return Err(EINVAL); }
    if !listed_linux.contains(uuid) { return Err(ENOENT); }
    Ok(())
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_enrol_admission)]
mod admission_tests {
    use super::*;

    fn ready() -> EnrolPrerequisites {
        EnrolPrerequisites {
            lane: true, live_count: Some(1), bag_bound: true,
            catacomb_readable: [true; 3], catacomb_writable: [true; 3],
            keybag_writable: true, host_store_capacity: true,
        }
    }

    fn refused(p: &EnrolPrerequisites, expected: Error) {
        assert_eq!(admit_enrol(p).unwrap_err().to_errno(), expected.to_errno());
    }

    #[test]
    fn missing_os_identity() {
        let mut p = ready(); p.lane = false; refused(&p, ENXIO);
    }

    #[test]
    fn unavailable_enclave_list() {
        let mut p = ready(); p.live_count = None; refused(&p, EIO);
    }

    #[test]
    fn enclave_full() {
        let mut p = ready(); p.live_count = Some(3); refused(&p, ENOSPC);
    }

    #[test]
    fn missing_identity_bag_binding() {
        let mut p = ready(); p.bag_bound = false; refused(&p, ENODATA);
    }

    #[test]
    fn missing_catacomb() {
        for i in 0..3 {
            let mut p = ready(); p.catacomb_readable[i] = false; refused(&p, ENODATA);
        }
    }

    #[test]
    fn unwritable_catacomb_or_keybag() {
        for i in 0..3 {
            let mut p = ready(); p.catacomb_writable[i] = false; refused(&p, EROFS);
        }
        let mut p = ready(); p.keybag_writable = false; refused(&p, EROFS);
    }

    #[test]
    fn host_store_full() {
        let mut p = ready(); p.host_store_capacity = false; refused(&p, ENOSPC);
    }

    #[test]
    fn ready_binding_is_admitted() {
        assert!(admit_enrol(&ready()).is_ok());
    }

    #[test]
    fn clean_lane_requires_prepared_owner_before_first_capture() {
        assert!(catacomb_needed_for_enrol(PRIVATE_TYPE_CATACOMB_OWNER, Some(0)));
        assert!(!catacomb_needed_for_enrol(PRIVATE_TYPE_CATACOMB_USER, Some(0)));
        assert!(!catacomb_needed_for_enrol(PRIVATE_TYPE_CATACOMB_MASTER, Some(0)));
        assert!(catacomb_needed_for_enrol(PRIVATE_TYPE_CATACOMB_USER, Some(1)));
    }

    #[test]
    fn orphan_delete_requires_exact_linux_identity() {
        let linux = [1u8; bio::UUID_LEN];
        let other_lane = [2u8; bio::UUID_LEN];
        assert!(admissible_orphan_target(&[linux], &linux).is_ok());
        refused_orphan(&[linux], &other_lane, ENOENT);
        refused_orphan(&[linux], &[0; bio::UUID_LEN], EINVAL);
    }

    fn refused_orphan(list: &[[u8; bio::UUID_LEN]], uuid: &[u8; bio::UUID_LEN], expected: Error) {
        assert_eq!(admissible_orphan_target(list, uuid).unwrap_err().to_errno(),
                   expected.to_errno());
    }
}

impl SepData {
    fn sbio_enrol_step(&self, op: &crate::sbio::SbioOp) -> Option<KVec<u8>> {
        match self.sbio_call(op) {
            SbioOutcome::Ok(payload) => Some(payload),
            SbioOutcome::PrerequisiteMissing => {
                dev_err_ratelimited!(self.dev, "enrol: {} prerequisite missing (status 0x01)\n", op.name());
                None
            }
            SbioOutcome::Status16 => {
                dev_err_ratelimited!(self.dev, "enrol: {} refused (status 0x16)\n", op.name());
                None
            }
            SbioOutcome::Other => {
                dev_err_ratelimited!(self.dev, "enrol: {} got no usable answer\n", op.name());
                None
            }
        }
    }

    /// The Linux enclave user's catacomb id and enrolled-set hash, which a
    /// key's biometric access-control list names (`pbioc`, `pbioh`). Read
    /// only.
    pub(crate) fn catacomb_binding(
        &self,
    ) -> Result<([u8; crate::sbio::SBIO_CATACOMB_ID_LEN], [u8; crate::sbio::SBIO_CATACOMB_HASH_LEN])> {
        let user = crate::sbio::UserId::new(SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
        let id_reply = self.sbio_expect_ok(&crate::sbio::sbio_catacomb_id(user)).ok_or(EIO)?;
        let hash_reply = self.sbio_expect_ok(&crate::sbio::sbio_catacomb_hash(user)).ok_or(EIO)?;
        dev_dbg!(
            self.dev,
            "sep-key: catacomb id {} bytes, hash {} bytes\n",
            id_reply.len(),
            hash_reply.len()
        );
        if id_reply.len() != crate::sbio::SBIO_CATACOMB_ID_LEN {
            return Err(EIO);
        }
        let mut id = [0u8; crate::sbio::SBIO_CATACOMB_ID_LEN];
        id.copy_from_slice(&id_reply);
        let hash = crate::sbio::catacomb_hash_from_reply(&hash_reply).ok_or(EIO)?;
        Ok((id, hash))
    }

    fn sbio_expect_ok(&self, op: &crate::sbio::SbioOp) -> Option<KVec<u8>> {
        match self.sbio_call(op) {
            SbioOutcome::Ok(payload) => Some(payload),
            _ => None,
        }
    }

    // A match step whose refusal ends the verify. libfprint reports every
    // failed verify as the same unspecified error, so the enclave's exact
    // answer has to be recorded here or nowhere. Success is decided as in
    // `sbio_call`.
    fn sbio_verify_step(&self, op: &crate::sbio::SbioOp) -> Option<KVec<u8>> {
        let done = match self.sbio_transfer(op) {
            Ok(done) => done,
            Err(e) => {
                dev_err_ratelimited!(
                    self.dev,
                    "verify: {} transfer failed ({:?})\n",
                    op.name(),
                    e
                );
                return None;
            }
        };
        match done.status.answered() {
            Some(err) if err as u16 == crate::sbio::SBIO_STATUS_OK => Some(done.payload),
            _ => {
                dev_err_ratelimited!(
                    self.dev,
                    "verify: {} answered status {}\n",
                    op.name(),
                    done.status
                );
                None
            }
        }
    }

    pub(crate) fn sbio_call(&self, op: &crate::sbio::SbioOp) -> SbioOutcome {
        let done = match self.sbio_transfer(op) {
            Ok(done) => done,
            Err(e) => {
                if op.opcode() == OP_SBIO_BEGIN_ENROL {
                    dev_err_ratelimited!(self.dev, "enrol: BEGIN_ENROL transfer failed ({:?})\n", e);
                }
                return SbioOutcome::Other;
            }
        };

        let Some(err) = done.status.answered() else {
            if op.opcode() == OP_SBIO_BEGIN_ENROL {
                dev_err_ratelimited!(self.dev, "enrol: BEGIN_ENROL reply had no status: {}\n", done.status);
            }
            return SbioOutcome::Other;
        };

        match err as u16 {
            crate::sbio::SBIO_STATUS_OK => SbioOutcome::Ok(done.payload),
            crate::sbio::SBIO_STATUS_PREREQUISITE => SbioOutcome::PrerequisiteMissing,
            crate::sbio::SBIO_STATUS_16 => SbioOutcome::Status16,
            _ => {
                if op.opcode() == OP_SBIO_BEGIN_ENROL {
                    dev_err_ratelimited!(self.dev, "enrol: BEGIN_ENROL replied with status 0x{:x}\n", err);
                }
                SbioOutcome::Other
            }
        }
    }

    fn sbio_relay(&self, relay: &crate::sbio::SbioRelay<'_>) -> Option<KVec<u8>> {
        match self.sbio_transfer_raw(relay.opcode(), relay.name(), relay.payload()) {
            Ok(done) if done.status.is_ok() => Some(done.payload),
            Ok(done) => {
                dev_err_ratelimited!(self.dev, "enrol: {} relay ({} bytes) refused with status {}\n", relay.name(), relay.payload().len(), done.status);
                None
            }
            Err(e) => {
                dev_err_ratelimited!(self.dev, "enrol: {} relay ({} bytes) failed ({:?})\n", relay.name(), relay.payload().len(), e);
                None
            }
        }
    }

    fn sync_device_view(&self) -> bool {
        let synced = matches!(
            self.sbio_call(&crate::sbio::sbio_update_device_list()),
            SbioOutcome::Ok(_)
        );

        let policy_ok = match self.sbio_call(&crate::sbio::sbio_match_policy()) {
            SbioOutcome::Ok(policy) if policy.len() == crate::sbio::SBIO_MATCH_POLICY_LEN => true,
            SbioOutcome::Ok(_) => false,
            _ => false,
        };

        synced && policy_ok
    }

    /// Ends a prompt's match operation and unlock evaluation that are still
    /// open (a prompt that ended without a match or the retry limit).
    fn end_match_scope(&self, why: &CStr) {
        self.match_scope_gen.store(0, Relaxed);
        let Some(evaluation) = self.match_context.lock().take() else {
            return;
        };
        match self.sbio_call(&crate::sbio::sbio_match_cancel()) {
            SbioOutcome::Ok(_) => {
                dev_info_ratelimited!(self.dev, "verify: match operation ended (MATCH_CANCEL) at {}\n", why)
            }
            _ => dev_warn_ratelimited!(self.dev, "verify: MATCH_CANCEL at {} failed\n", why),
        }
        self.close_unlock_evaluation(&evaluation);
    }

    /// The tail macOS sends after a no-match result: further frames of the
    /// same touch, each relayed (0x65) and followed by RETRY_CAPTURE_INFO,
    /// then MATCH_RETRY_FINALIZE, then MATCH_POLICY. macOS relays the frames
    /// the sensor queued while the finger stayed down (three in the captured
    /// prompts); the whole touch is one result.
    fn no_match_tail(&self) {
        let mut frames: u32 = 0;
        let mut streamed: u32 = 0;
        while frames < NOMATCH_EXTRA_FRAMES {
            let Some((advertised, from_stream)) = self.next_tail_frame() else {
                break;
            };
            if from_stream {
                streamed += 1;
            }
            let Ok(capture) = sensor::read_capture(advertised) else {
                break;
            };
            let relay = crate::sbio::SbioRelay::new(&capture);
            if self.sbio_relay(&relay).is_none() {
                break;
            }
            drop(capture);
            let op = crate::sbio::sbio_retry_capture_info();
            match self.sbio_transfer(&op) {
                Ok(done) if done.status.is_ok() => frames += 1,
                Ok(done) => {
                    dev_warn_ratelimited!(
                        self.dev,
                        "verify: no-match tail: {} answered status {} ({} bytes)\n",
                        op.name(),
                        done.status,
                        done.payload.len()
                    );
                    break;
                }
                Err(e) => {
                    dev_warn_ratelimited!(self.dev, "verify: no-match tail: {} failed ({:?})\n", op.name(), e);
                    break;
                }
            }
        }
        let op = crate::sbio::sbio_match_retry_finalize();
        match self.sbio_transfer(&op) {
            Ok(done) => dev_info_ratelimited!(
                self.dev,
                "verify: no-match tail: {} further frame(s) relayed ({} still streaming from the touch); {} answered status {}, {} bytes\n",
                frames,
                streamed,
                op.name(),
                done.status,
                done.payload.len()
            ),
            Err(e) => dev_warn_ratelimited!(self.dev, "verify: no-match tail: {} failed ({:?})\n", op.name(), e),
        }
    }

    /// The next frame of the touch for the no-match tail, and whether the
    /// sensor was still imaging it. One START_CAPTURE on macOS yields a frame
    /// every ~80 ms while the finger stays down, and the tail relays the
    /// frames taken while MATCH_RESULT ran; take those first. Otherwise the
    /// capture is started again: a sensor armed for a finger finds one still
    /// down within a frame period or two, so it waits less for a lifted one.
    fn next_tail_frame(&self) -> Option<(u32, bool)> {
        let st = sensor::status().ok()?;
        if let Some(n) = st.available().filter(|n| *n > 0) {
            return Some((n, true));
        }
        let (wait_ms, streaming) = match st.state {
            sensor::STATE_READING => (NOMATCH_FRAME_WAIT_MS, true),
            sensor::STATE_ARMED => {
                sensor::start_capture().ok()?;
                (NOMATCH_ARMED_WAIT_MS, false)
            }
            _ => {
                sensor::start_capture().ok()?;
                (NOMATCH_FRAME_WAIT_MS, false)
            }
        };
        match self.await_capture_within(wait_ms) {
            CaptureWait::Ready(n) => Some((n, streaming)),
            _ => None,
        }
    }

    /// Starts the enclave's operation for a prompt or an enrolment
    /// (`OPERATION_DEVICE_LIST`, `0x7d`) and checks its device list: a count
    /// and one 24-byte record per device, at least one. `what` names the
    /// caller in the log.
    fn open_operation(&self, what: &CStr, kind: &CStr) -> bool {
        match self.sbio_call(&crate::sbio::sbio_operation_device_list()) {
            SbioOutcome::Ok(reply) => match crate::sbio::operation_device_count(&reply) {
                Some(count) if count > 0 => {
                    dev_info_ratelimited!(
                        self.dev,
                        "{}: {} operation opened (OPERATION_DEVICE_LIST, {} device(s))\n",
                        what,
                        kind,
                        count
                    );
                    true
                }
                Some(_) => {
                    dev_err_ratelimited!(self.dev, "{}: OPERATION_DEVICE_LIST listed no device\n", what);
                    false
                }
                None => {
                    dev_err_ratelimited!(
                        self.dev,
                        "{}: OPERATION_DEVICE_LIST answered a malformed list ({} bytes)\n",
                        what,
                        reply.len()
                    );
                    false
                }
            },
            SbioOutcome::PrerequisiteMissing => {
                dev_err_ratelimited!(
                    self.dev,
                    "{}: OPERATION_DEVICE_LIST refused: prerequisite missing\n",
                    what
                );
                false
            }
            _ => {
                dev_err_ratelimited!(self.dev, "{}: OPERATION_DEVICE_LIST failed\n", what);
                false
            }
        }
    }

    /// Starts a match operation (`0x7d`) and checks its device list.
    fn open_match_operation(&self) -> bool {
        self.open_operation(c"verify", c"match")
    }

    /// Ends the imaging of a touch the way macOS does: PAUSE_CAPTURE stops
    /// the frames but keeps the finger detector running, so the sensor falls
    /// to idle at the moment the finger is lifted. The next capture is armed
    /// then ([`Self::await_finger_lift`]), not after a fixed pause.
    fn pause_after_touch(&self, what: &CStr) {
        match sensor::pause_capture() {
            Ok(()) => self.paused_at_ns.store(shim::boottime_ns().max(1), Relaxed),
            Err(e) => {
                dev_warn_ratelimited!(
                    self.dev,
                    "{}: PAUSE_CAPTURE failed ({:?}); idling the sensor\n",
                    what,
                    e
                );
                self.paused_at_ns.store(0, Relaxed);
                let _ = sensor::idle();
            }
        }
    }

    /// Forgets a pending finger lift: the sensor was idled, powered off or is
    /// brought up afresh, so it no longer reports one.
    fn forget_finger_lift(&self) {
        self.paused_at_ns.store(0, Relaxed);
    }

    /// Names why a verify capture did not count, in the poll's `guidance`
    /// that comes with its `SEP_BIO_STATUS_RETRY`: PLACE when no finger came,
    /// CENTER for partial contact, LIFT_AND_MOVE for a bad frame.
    /// The failed result that follows wakes the poller.
    fn retry_cause(&self, cause: bio::Guidance) {
        let epoch = self.capture_epoch.load(Relaxed);
        let _ = bio::capture_guide(&mut self.bio_session.lock(), epoch, cause);
    }

    /// Waits until the finger of the last touch is off the sensor, so that the
    /// next capture takes a new touch and not the resting finger. macOS arms
    /// the next capture within 2 ms of the lift; a finger lifted before the
    /// result came back costs nothing here. While the finger stays down the
    /// person is told to lift it. Frames the sensor took before the pause took
    /// effect are read and dropped, as macOS does. Returns false if the
    /// operation was cancelled meanwhile.
    fn await_finger_lift(&self, what: &CStr) -> bool {
        let paused_at = self.paused_at_ns.load(Relaxed);
        if paused_at == 0 {
            return true;
        }
        let use_irq = sensor::irq_available();
        if use_irq {
            sensor::irq_arm();
        }
        let started = shim::boottime_ns();
        let deadline = started.saturating_add(u64::from(LIFT_WAIT_MS) * 1_000_000);
        let mut dropped: u32 = 0;
        let mut told = false;
        let mut first: Option<[u8; sensor::STATUS_LEN]> = None;
        loop {
            if !self.capture_live() {
                return false;
            }
            let Ok(st) = sensor::status() else {
                // The capture that follows reports an unreadable sensor.
                self.forget_finger_lift();
                return true;
            };
            // Lifted: the sensor stopped imaging (idle, or armed for a new
            // finger) and its finger detector is clear. macOS reads the same
            // status (state 0, flags 0x0c) at the lift interrupt.
            let lifted = st.finger_off();
            if first.is_none() {
                first = Some(st.raw);
            }
            if lifted {
                self.forget_finger_lift();
                let now = shim::boottime_ns();
                dev_info_ratelimited!(
                    self.dev,
                    "{}: finger off the sensor {} ms after the last touch ended (waited {} ms, {} late frame(s) dropped; status {} then {}); arming the capture\n",
                    what,
                    now.saturating_sub(paused_at) / 1_000_000,
                    now.saturating_sub(started) / 1_000_000,
                    dropped,
                    Hex(&first.unwrap_or(st.raw)),
                    Hex(&st.raw)
                );
                return true;
            }
            if let Some(n) = st.available().filter(|n| *n > 0) {
                if sensor::read_capture(n).is_ok() {
                    dropped = dropped.saturating_add(1);
                }
            }
            if !told {
                told = true;
                if bio::capture_guide(
                    &mut self.bio_session.lock(),
                    self.capture_epoch.load(Relaxed),
                    bio::Guidance::Lift,
                ) {
                    self.bio_wake();
                }
            }
            if shim::boottime_ns() >= deadline {
                dev_warn_ratelimited!(
                    self.dev,
                    "{}: the finger stayed on the sensor for {} ms; arming the capture anyway\n",
                    what,
                    LIFT_WAIT_MS
                );
                self.forget_finger_lift();
                let _ = sensor::idle();
                return true;
            }
            if use_irq {
                let _ = sensor::irq_wait(LIFT_POLL_MS);
                sensor::irq_arm();
            } else {
                let poll = kernel::time::Delta::from_millis(i64::from(LIFT_POLL_MS));
                kernel::time::delay::fsleep(poll);
            }
        }
    }

    fn stash_enrol_identity(&self, record: &[u8]) {
        let mut found: KVec<[u8; bio::UUID_LEN]> = KVec::new();
        crate::sbio::enrol_identity_candidates(record, SBIO_PROBE_USER_ID, |_at, uuid| {
            let _ = found.push(uuid, GFP_KERNEL);
        });
        *self.enrol_identity_candidates.lock() = found;
    }

    fn identity_just_enrolled(&self) -> Option<[u8; bio::UUID_LEN]> {
        let candidates = core::mem::take(&mut *self.enrol_identity_candidates.lock());
        let listed = self.enclave_identities_for(SBIO_PROBE_USER_ID)?;

        let mut confirmed: KVec<[u8; bio::UUID_LEN]> = KVec::new();
        for uuid in candidates.iter() {
            if listed.iter().any(|u| u == uuid) && !confirmed.iter().any(|u| u == uuid) {
                if confirmed.push(*uuid, GFP_KERNEL).is_err() {
                    break;
                }
            }
        }

        if confirmed.len() == 1 {
            return Some(confirmed[0]);
        }

        let index = self.bio_index.lock();
        let mut new_identity = None;
        for uuid in listed.iter() {
            if index.contains_uuid(uuid) {
                continue;
            }
            if new_identity.is_some() {
                return None;
            }
            new_identity = Some(*uuid);
        }
        new_identity
    }

    fn reconcile_identities(&self) {
        let Some(listed) = self.enclave_identities_for(SBIO_PROBE_USER_ID) else {
            return;
        };

        if listed.is_empty() {
            return;
        }

        let dropped = {
            let mut index = self.bio_index.lock();
            match index.reconcile_to(&listed) {
                Ok(dropped) => dropped,
                Err(_) => {
                    return;
                }
            }
        };

        let held = {
            let index = self.bio_index.lock();
            index.total()
        };
        if dropped.is_empty() && held == listed.len() {
            return;
        }
        let persisted = {
            let index = self.bio_index.lock();
            self.with_host_store(|store| index.persist(store))
        };
        match persisted {
            Some(Ok(())) => {},
            _ => dev_warn_ratelimited!(
                self.dev,
                "sbio: the reconciled index could not be written; it is correct in memory for this boot and will be rebuilt at the next attach\n"
            ),
        }
    }

    fn enclave_identities_for(&self, user_id: i32) -> Option<KVec<[u8; bio::UUID_LEN]>> {
        let op = crate::sbio::sbio_list_identities();
        let SbioOutcome::Ok(reply) = self.sbio_call(&op) else {
            return None;
        };

        let records = crate::sbio::IdentityRecords::new(&reply)?;

        let mut out = KVec::new();
        let mut failed = false;
        records.each_built_in_for(user_id, |identity| {
            if out.push(*identity.uuid(), GFP_KERNEL).is_err() {
                failed = true;
            }
        });
        if failed {
            return None;
        }
        Some(out)
    }

    fn finish_enrolment(&self, outcome: core::result::Result<[u8; bio::UUID_LEN], u32>) {
        let filed = {
            let mut session = self.bio_session.lock();
            let mut index = self.bio_index.lock();
            bio::enrol_finish(
                &mut session, self.capture_epoch.load(Relaxed), &mut index, outcome,
            )
        };
        if !filed {
            return;
        }
        // lock ordering: store lock outside the session lock (UAF otherwise)
        if outcome.is_ok() {
            let index = self.bio_index.lock();
            let saved = self.with_host_store(|store| index.persist(store));
            match saved {
                Some(Ok(())) => {},
                Some(Err(e)) => dev_err_ratelimited!(
                    self.dev,
                    "enrol: enclave holds a new template but the host index could not be written ({:?})\n",
                    e
                ),
                None => {}
            }
        }
        self.bio_wake();
    }

    fn bio_wake(&self) {
        if let Some(dev) = self.bio_dev.lock().as_ref() {
            dev.wake();
        }
    }

    pub(crate) fn attach_sensor(&self) {
        if let Err(e) = sensor::register_driver() {
            dev_err_ratelimited!(
                self.dev,
                "sensor: could not register the SPI driver: {:?}\n",
                e
            );
            return;
        }

        match dt::enable_spi_sensor(sensor::CONTROLLER_BASE, sensor::CHIP_SELECT) {
            Ok(true) => {},
            Ok(false) => {
                dev_info_ratelimited!(
                    self.dev,
                    "sensor: SPI controller withheld by the J700 firmware handoff; Touch ID unavailable\n"
                );
                return;
            }
            Err(e) => {
                dev_warn_ratelimited!(
                    self.dev,
                    "sensor: could not enable the SPI bus at 0x{:x} or create the sensor node ({:?}); no sensor\n",
                    sensor::CONTROLLER_BASE,
                    e
                );
                return;
            }
        }

        let bound = sensor::is_bound();
        if bound && sensor::power_source() == sensor::PowerSource::None {
            dev_warn_ratelimited!(
                self.dev,
                "sensor: no power line ({}); an unpowered sensor answers sixteen zero bytes like a dead bus\n",
                sensor::power_source().name()
            );
        } else if bound {
            dev_info_ratelimited!(self.dev, "sensor: power via {}\n", sensor::power_source().name());
        }
    }

    fn open_enrolment_context(&self, user: crate::sbio::UserId) -> bool {
        let listed = match self.enclave_identities_for(SBIO_PROBE_USER_ID) {
            Some(list) => list.len(),
            None => {
                dev_warn_ratelimited!(self.dev, "enrol: identity enumeration failed; trying the existing context\n");
                return self.enrol_into_existing_context(user);
            }
        };

        let Some(proof) = crate::sbio::NoExistingCatacomb::from_zero_identities(listed) else {
            return self.enrol_into_existing_context(user);
        };

        // PREPARE_ENROL may already have selected the empty Linux context in
        // this boot. A file left from an earlier boot is not that proof.
        if self.empty_context_prepared.xchg(false, Relaxed) {
            return self.enrol_into_existing_context(user);
        }

        self.open_fresh_context(user, &proof)
    }

    fn enrol_into_existing_context(&self, user: crate::sbio::UserId) -> bool {
        if !self.activate_protected_config(user) {
            return false;
        }
        true
    }

    fn open_fresh_context(
        &self,
        user: crate::sbio::UserId,
        proof: &crate::sbio::NoExistingCatacomb,
    ) -> bool {
        let system = crate::sbio::sbio_select_context(crate::sbio::ContextScope::SYSTEM, proof);
        if self.sbio_enrol_step(&system).is_none() {
            return false;
        }

        let Some(reply) = self.read_component_states() else {
            dev_warn_ratelimited!(self.dev, "enrol: could not read component state after system SELECT_CONTEXT\n");
            return false;
        };
        let states = crate::sbio::ComponentStates::new(&reply);
        let owner_initial_state = states.state_for(crate::sbio::CatacombUser::OWNER.value());

        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"master catacomb",
            ),
            (
                crate::sbio::CatacombUser::OWNER,
                PRIVATE_TYPE_CATACOMB_OWNER,
                c"owner catacomb",
            ),
        ] {
            let Some(state) = states.state_for(who.value()) else {
                dev_err_ratelimited!(self.dev, "enrol: {} is absent after system SELECT_CONTEXT\n", what);
                return false;
            };
            dev_info_ratelimited!(self.dev, "enrol: after system SELECT_CONTEXT {} state 0x{:x}\n", what, state);
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                dev_err_ratelimited!(self.dev, "enrol: {} is not active after system SELECT_CONTEXT\n", what);
                return false;
            }
            // On m2, state 0x3 refused SAVE_CATACOMB with status 0x6. The
            // 0x04 save-pending bit is also used by the T2 Catacomb protocol;
            // only persist a fresh component when SEP reports that bit.
            if state & crate::sbio::COMPONENT_STATE_SAVE_PENDING == 0 {
                continue;
            }
            if !self.save_catacomb(who, kind, what) {
                dev_err_ratelimited!(self.dev, "enrol: could not save {} after selecting the system context\n", what);
                return false;
            }
        }

        // The owner Catacomb must be exported even though state 0x3 has no
        // save-pending bit. J313, J414s and J314s each restored a fresh
        // enrollment after this export and user-before-master completion.
        // Every profile now does this.
        if self.profile.persistent_enrol || module_parameters::j414s_persistent_enrol.value() != 0 {
            if owner_initial_state == Some(0x3)
                && !self.save_catacomb(
                    crate::sbio::CatacombUser::OWNER,
                    PRIVATE_TYPE_CATACOMB_OWNER,
                    c"fresh-context owner catacomb (user 501)",
                )
            {
                dev_err_ratelimited!(self.dev, "enrol: opt-in fresh-context owner export failed\n");
                return false;
            }
            if !crate::catacomb::is_saved(PRIVATE_TYPE_CATACOMB_OWNER) {
                dev_err_ratelimited!(self.dev, "enrol: opt-in fresh context has no owner file\n");
                return false;
            }
        }

        let per_user =
            crate::sbio::sbio_select_context(crate::sbio::ContextScope::user(user), proof);
        if self.sbio_enrol_step(&per_user).is_none() {
            return false;
        }

        if !self.activate_protected_config(user) {
            return false;
        }

        true
    }

    /// Opens and closes one unlock evaluation at boot, before any touch, so
    /// the log shows whether its preflight leaves a biometric match open.
    fn probe_unlock_evaluation(&self) {
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return;
        };
        match self.open_unlock_evaluation(user) {
            Some(handle) => {
                dev_info_ratelimited!(self.dev, "Touch ID: unlock preflight gate passed for user {}\n", user.value());
                self.close_unlock_evaluation(&handle);
            }
            None => dev_err_ratelimited!(
                self.dev,
                "Touch ID: unlock preflight gate failed for user {}; verifies will be refused\n",
                user.value()
            ),
        }
    }

    /// Reads the Linux user's protected configuration once and logs whether
    /// it allows unlock. Only the two symbolic states are logged.
    fn log_protected_unlock(&self) {
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return;
        };
        let Some(raw) = self.sbio_expect_ok(&crate::sbio::sbio_protected_config(user)) else {
            dev_warn_ratelimited!(self.dev, "sbio: PROTECTED_CONFIG for user {} did not answer\n", user.value());
            return;
        };
        match crate::sbio::ProtectedConfig::parse(&raw) {
            Some(config) => dev_info_ratelimited!(
                self.dev,
                "sbio: protected unlock for user {}: set={}, effective={}, admits an unlock match {}\n",
                user.value(),
                crate::sbio::ProtectedConfig::describe(config.unlock_set),
                crate::sbio::ProtectedConfig::describe(config.unlock_effective),
                config.allows_unlock()
            ),
            None => dev_warn_ratelimited!(
                self.dev,
                "sbio: PROTECTED_CONFIG for user {} answered {} bytes, expected {}\n",
                user.value(),
                raw.len(),
                crate::sbio::SBIO_PROTECTED_CONFIG_LEN
            ),
        }
    }

    fn activate_protected_config(&self, user: crate::sbio::UserId) -> bool {
        let op = crate::sbio::sbio_protected_config(user);
        let Some(config) = self.sbio_enrol_step(&op) else {
            return false;
        };
        if config.len() != crate::sbio::SBIO_PROTECTED_CONFIG_LEN {
            dev_err_ratelimited!(self.dev, "enrol: protected configuration has {} bytes, expected {}\n", config.len(), crate::sbio::SBIO_PROTECTED_CONFIG_LEN);
            return false;
        }
        true
    }

    fn begin_enrolment_on_enclave(&self) -> Option<OpenEnrolment<'_>> {
        if self.enrol_open.load(Relaxed) {
            self.enrol_open.store(false, Relaxed);
            let _ = self.sbio_call(&crate::sbio::sbio_cancel_operation());
        }

        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            dev_err_ratelimited!(self.dev, "enrol: invalid probe user id\n");
            return None;
        };

        // 0x03 answers -3 until a user key bag is designated
        if !self.keybag_designated.load(Relaxed) {
            dev_err_ratelimited!(self.dev, "enrol: no user keybag was designated\n");
            return None;
        }

        if self.enrol_material.lock().is_none() {
            dev_err_ratelimited!(self.dev, "enrol: no enrolment material is held\n");
            return None;
        }

        // 0x03 answers 0x1 until an enrolment context exists
        if !self.open_enrolment_context(user) {
            dev_err_ratelimited!(self.dev, "enrol: could not open the enclave enrolment context\n");
            return None;
        }

        let Some(acm_handle) = self.establish_passcode_validated_context(user) else {
            dev_err_ratelimited!(
                self.dev,
                "enrol: could not establish the PasscodeValidated SCRD credential; not falling back to a transient SKS token (a catacomb sealed under one does not survive a reboot)\n"
            );
            return None;
        };
        let op =
            crate::sbio::sbio_begin_enrol(user, crate::sbio::BE_AUTH_TYPE_ACM_CONTEXT, &acm_handle);

        if self.sbio_enrol_step(&op).is_none() {
            return None;
        }
        self.enrol_open.store(true, Relaxed);
        let open = OpenEnrolment {
            sep: self,
            armed: true,
        };
        // macOS sends OPERATION_DEVICE_LIST right after BEGIN_ENROL, before
        // the first capture, as it does at the start of a match operation.
        // Dropping `open` on a failure cancels the enrolment.
        if !self.open_operation(c"enrol", c"enrolment") {
            return None;
        }
        Some(open)
    }

    fn probe_owner_export(&self) {
        if module_parameters::probe_owner_export.value() == 0 {
            return;
        }
        if self.templates_restored.load(Relaxed)
            || !crate::catacomb::is_absent(PRIVATE_TYPE_CATACOMB_OWNER)
            || !crate::catacomb::is_saved(PRIVATE_TYPE_CATACOMB_MASTER)
            || !crate::catacomb::is_saved(PRIVATE_TYPE_CATACOMB_USER)
        {
            dev_warn_ratelimited!(self.dev, "owner-probe: preconditions not met; no context selected\n");
            return;
        }
        let Some(identities) = self.enclave_identities_for(SBIO_PROBE_USER_ID) else {
            dev_warn_ratelimited!(self.dev, "owner-probe: identity count unavailable; no context selected\n");
            return;
        };
        let Some(proof) = crate::sbio::NoExistingCatacomb::from_zero_identities(identities.len()) else {
            dev_warn_ratelimited!(self.dev, "owner-probe: enclave identity exists; no context selected\n");
            return;
        };

        // This is destructive to the enclave's *volatile* current context.
        // It is opt-in and only reached after proving zero live identities.
        let system = crate::sbio::sbio_select_context(crate::sbio::ContextScope::SYSTEM, &proof);
        if self.sbio_expect_ok(&system).is_none() {
            dev_warn_ratelimited!(self.dev, "owner-probe: system context selection refused\n");
            return;
        }
        let state = self.read_component_states().and_then(|reply| {
            crate::sbio::ComponentStates::new(&reply)
                .state_for(crate::sbio::CatacombUser::OWNER.value())
        });
        dev_info_ratelimited!(self.dev, "owner-probe: selected owner component state {:?}\n", state);
        if !matches!(state, Some(value) if value & crate::sbio::COMPONENT_STATE_ACTIVE != 0) {
            return;
        }
        let saved = self.save_catacomb(
            crate::sbio::CatacombUser::OWNER,
            PRIVATE_TYPE_CATACOMB_OWNER,
            c"owner catacomb (user 501) diagnostic",
        );
        dev_warn_ratelimited!(self.dev, "owner-probe: owner SAVE_CATACOMB and CONFIRM_SAVE completed {}\n", saved);
    }

    fn restore_all_components(&self) -> bool {
        // 0x6b is re-read before every component, not once up front.
        let user_id = SBIO_PROBE_USER_ID;
        let mut any = false;
        let mut missing_files = false;
        let mut user_outcome = RestoreOutcome::NoStoredFile;

        // A 0x8002 reply is not sufficient evidence of restoration. Re-read
        // the component state for every load, including the user component.
        for (id, kind, what) in [
            (
                crate::sbio::CatacombUser::MASTER.value(),
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"master catacomb",
            ),
            (
                crate::sbio::CatacombUser::OWNER.value(),
                PRIVATE_TYPE_CATACOMB_OWNER,
                c"owner catacomb",
            ),
            (user_id, PRIVATE_TYPE_CATACOMB_USER, c"user catacomb"),
        ] {
            let Some(reply) = self.read_component_states() else {
                return false;
            };
            let states = crate::sbio::ComponentStates::new(&reply);
            let state = states.state_for(id);
            let outcome = self.restore_catacomb(state, id, kind, what);
            dev_info_ratelimited!(
                self.dev,
                "sbio: restore {} state {:?} outcome {:?}\n",
                what,
                state,
                outcome
            );
            match outcome {
                RestoreOutcome::Restored | RestoreOutcome::AlreadyActive => any = true,
                RestoreOutcome::NoStoredFile => missing_files = true,
                RestoreOutcome::Failed | RestoreOutcome::EmptyTolerated => {}
            }
            if id == user_id {
                user_outcome = outcome;
            }
        }

        let user_ok = matches!(
            user_outcome,
            RestoreOutcome::Restored | RestoreOutcome::AlreadyActive
        );
        // The enclave's lockout record goes back after the first password
        // (see `lockout_import_pending`); here only whether one is stored.
        let lockout_outcome = self.defer_lockout_import();
        dev_info_ratelimited!(self.dev, "sbio: restore lockout outcome {:?}\n", lockout_outcome);
        let lockout = match lockout_outcome {
            RestoreOutcome::Restored | RestoreOutcome::AlreadyActive => true,
            RestoreOutcome::EmptyTolerated => true,
            RestoreOutcome::NoStoredFile => {
                missing_files = true;
                false
            }
            RestoreOutcome::Failed => false,
        };

        if missing_files {
            dev_warn_ratelimited!(
                self.dev,
                "sbio: at least one catacomb/lockout file is absent; match readiness depends on the user restore and device-view proof\n"
            );
        }

        let _ = any;
        user_ok && lockout
    }

    fn read_component_states(&self) -> Option<KVec<u8>> {
        let op = crate::sbio::sbio_context_state();
        let SbioOutcome::Ok(reply) = self.sbio_call(&op) else {
            return None;
        };

        let states = crate::sbio::ComponentStates::new(&reply);
        if states.count() == 0 {
            return None;
        }
        Some(reply)
    }

    fn restore_catacomb(
        &self,
        state: Option<u32>,
        id: i32,
        kind: u8,
        what: &CStr,
    ) -> RestoreOutcome {
        let Some(state) = state else {
            return RestoreOutcome::Failed;
        };
        match crate::sbio::component_action(state) {
            crate::sbio::ComponentAction::AlreadyActive => RestoreOutcome::AlreadyActive,
            crate::sbio::ComponentAction::Unsupported => RestoreOutcome::Failed,
            crate::sbio::ComponentAction::Load => {
                let blob = match self.read_stored(kind, what) {
                    Ok(Some(blob)) => blob,
                    Ok(None) => return RestoreOutcome::NoStoredFile,
                    Err(_) => return RestoreOutcome::Failed,
                };

                let at = crate::sbio::SBIO_SAVED_USER_ID_AT;
                if blob.len() < at + 4 {
                    return RestoreOutcome::Failed;
                }
                let carried =
                    i32::from_le_bytes([blob[at], blob[at + 1], blob[at + 2], blob[at + 3]]);
                if carried != id {
                    return RestoreOutcome::Failed;
                }

                let Some(request) = crate::sbio::SbioLoadCatacomb::new(&blob) else {
                    return RestoreOutcome::Failed;
                };

                match self.sbio_transfer_raw(request.opcode(), request.name(), request.payload()) {
                    Ok(done) if done.status.is_ok() => {
                        self.confirm_active(id, what, LoadAnswer::StatusZero)
                    }
                    Ok(done)
                        if done.status.answered()
                            == Some(crate::sbio::SBIO_STATUS_COLD_TRANSITION) =>
                    {
                        let active = self.confirm_active(id, what, LoadAnswer::ColdTransition);
                        if id != SBIO_PROBE_USER_ID
                            || !matches!(&active, RestoreOutcome::Restored)
                        {
                            return active;
                        }
                        match self.log_identity_count(c"after first user cold-transition load") {
                            Some(n) if n > 0 => return active,
                            Some(0) => {}
                            _ => return RestoreOutcome::Failed,
                        }

                        // Diagnostic only: one idempotent retry of the same
                        // host blob, with no context reset or save. An active
                        // state by itself did not prove a live identity on J414s.
                        let second = self.sbio_transfer_raw(
                            request.opcode(), request.name(), request.payload()
                        );
                        let answer = match second {
                            Ok(done) if done.status.is_ok() => LoadAnswer::StatusZero,
                            Ok(done)
                                if done.status.answered()
                                    == Some(crate::sbio::SBIO_STATUS_COLD_TRANSITION) =>
                            {
                                LoadAnswer::ColdTransition
                            }
                            Ok(done)
                                if done.status.answered()
                                    == Some(crate::sbio::SBIO_STATUS_ALREADY_ACTIVE) =>
                            {
                                LoadAnswer::AlreadyActive
                            }
                            Ok(done) => {
                                dev_warn_ratelimited!(self.dev, "sbio: second user LOAD_CATACOMB refused with status {}\n", done.status);
                                return RestoreOutcome::Failed;
                            }
                            Err(e) => {
                                dev_warn_ratelimited!(self.dev, "sbio: second user LOAD_CATACOMB transfer failed ({:?})\n", e);
                                return RestoreOutcome::Failed;
                            }
                        };
                        let active = self.confirm_active(id, what, answer);
                        match (active, self.log_identity_count(c"after second user cold-transition load")) {
                            (RestoreOutcome::Restored, Some(n)) if n > 0 => RestoreOutcome::Restored,
                            _ => RestoreOutcome::Failed,
                        }
                    }
                    // 0x101 ALREADY_ACTIVE is a success
                    Ok(done)
                        if done.status.answered()
                            == Some(crate::sbio::SBIO_STATUS_ALREADY_ACTIVE) =>
                    {
                        self.confirm_active(id, what, LoadAnswer::AlreadyActive)
                    }
                    Ok(done) => {
                        dev_warn_ratelimited!(self.dev, "sbio: {} LOAD_CATACOMB refused with status {}\n", what, done.status);
                        RestoreOutcome::Failed
                    }
                    Err(e) => {
                        dev_warn_ratelimited!(self.dev, "sbio: {} LOAD_CATACOMB transfer failed ({:?})\n", what, e);
                        RestoreOutcome::Failed
                    }
                }
            }
        }
    }

    fn confirm_active(&self, id: i32, what: &CStr, answer: LoadAnswer) -> RestoreOutcome {
        let Some(reply) = self.read_component_states() else {
            dev_err_ratelimited!(
                self.dev,
                "sbio: the {} answered {} and 0x6b could not be re-read; treating unknown as failure\n",
                what,
                answer.describe()
            );
            return RestoreOutcome::Failed;
        };
        let states = crate::sbio::ComponentStates::new(&reply);

        let state = states.state_for(id);
        dev_info_ratelimited!(self.dev, "sbio: {} LOAD_CATACOMB answered {} with post-load state {:?}\n", what, answer.describe(), state);
        match state {
            Some(state) if state & crate::sbio::COMPONENT_STATE_ACTIVE != 0 => {
                RestoreOutcome::Restored
            }
            Some(state) if state & crate::sbio::COMPONENT_STATE_COLD != 0 => RestoreOutcome::Failed,
            Some(_) => RestoreOutcome::Failed,
            None => RestoreOutcome::Failed,
        }
    }

    /// Notes whether a lockout record is stored, for `import_lockout_after_password`.
    /// A stored record counts as present for match readiness: until it is
    /// loaded the host record carries the failed-attempt count, and Touch ID
    /// waits for the first password anyway.
    fn defer_lockout_import(&self) -> RestoreOutcome {
        match self.read_stored(PRIVATE_TYPE_LOCKOUT, c"bio lockout record") {
            Ok(Some(_)) => {
                self.lockout_import_pending.store(true, Relaxed);
                RestoreOutcome::EmptyTolerated
            }
            Ok(None) => RestoreOutcome::NoStoredFile,
            Err(_) => RestoreOutcome::Failed,
        }
    }

    /// Hands the saved lockout record back to the enclave once, after the
    /// first password authentication of the boot, as macOS does after the
    /// first unlock. Before anything else the re-arm does, so the enclave's
    /// own count is in place before a password clears it.
    fn import_lockout_after_password(&self) {
        if !self.touchid_started.load(Relaxed) || !self.lockout_import_pending.xchg(false, Relaxed) {
            return;
        }
        let Some(_writer) = self.catacomb_writer.try_lock() else {
            // A capture holds the sensor lock; try again at the next password.
            self.lockout_import_pending.store(true, Relaxed);
            return;
        };
        if !self.capture_begin() {
            self.lockout_import_pending.store(true, Relaxed);
            return;
        }
        let outcome = self.restore_lockout();
        self.capture_end();
        dev_info_ratelimited!(
            self.dev,
            "sbio: lockout record loaded after the first password: {:?}\n",
            outcome
        );
    }

    fn restore_lockout(&self) -> RestoreOutcome {
        let blob = match self.read_stored(PRIVATE_TYPE_LOCKOUT, c"bio lockout record") {
            Ok(Some(blob)) => blob,
            Ok(None) => return RestoreOutcome::NoStoredFile,
            Err(_) => return RestoreOutcome::Failed,
        };
        let Some(request) = crate::sbio::SbioLoadLockout::new(&blob) else {
            return RestoreOutcome::Failed;
        };
        match self.sbio_transfer_raw(request.opcode(), request.name(), request.payload()) {
            Ok(done) if done.status.is_ok() => RestoreOutcome::Restored,
            Ok(done)
                if matches!(
                    done.status,
                    transfer::DeviceStatus::Answered(crate::sbio::SBIO_STATUS_COLD_TRANSITION)
                ) =>
            {
                // Not a restore: the enclave did not take the record (macOS
                // discards such a record). The host record keeps the count,
                // and the next SAVE_LOCKOUT replaces the stored one.
                dev_info_ratelimited!(
                    self.dev,
                    "sbio: LOAD_LOCKOUT refused the saved lockout record (0x8002); the failed-attempt count comes from the host record\n"
                );
                RestoreOutcome::Failed
            }
            Ok(_) => RestoreOutcome::Failed,
            Err(_) => RestoreOutcome::Failed,
        }
    }

    /// `Ok(None)` means provably not stored; any read failure is an error so
    /// it cannot be mistaken for a missing file.
    fn read_stored(&self, kind: u8, what: &CStr) -> Result<Option<KVec<u8>>> {
        if crate::catacomb::is_kind(kind) {
            match crate::catacomb::read(kind) {
                Ok(Some(blob)) => return Ok(Some(blob)),
                Ok(None) => {}
                Err(e) => {
                    dev_err_ratelimited!(self.dev, "restore: cannot read {}: {:?}\n", what, e);
                    return Err(e);
                }
            }
        }
        match self.with_host_store(|store| store.read(&store::Key::root(kind))) {
            Some(Ok(found)) => Ok(found),
            Some(Err(e)) => {
                dev_err_ratelimited!(self.dev, "restore: cannot read {}: {:?}\n", what, e);
                Err(e)
            }
            None => Ok(None),
        }
    }

    fn save_all_components(&self, user: crate::sbio::UserId) -> bool {
        if self.profile.persistent_enrol || module_parameters::j414s_persistent_enrol.value() != 0 {
            return self.save_all_components_user_first(user);
        }
        // On m2fix11, a completed enrollment marked both master and user 0x7,
        // but saving and confirming the user first was followed by master
        // SAVE_CATACOMB status 0x6. Try the master before the user at this
        // completion gate; post-match persistence keeps its separate order.
        // Re-read state before each save and never send an unconditional one.
        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"master catacomb (user -1)",
            ),
            (
                crate::sbio::CatacombUser::OWNER,
                PRIVATE_TYPE_CATACOMB_OWNER,
                c"owner catacomb (user 501)",
            ),
            (
                crate::sbio::CatacombUser::enrolling(user),
                PRIVATE_TYPE_CATACOMB_USER,
                c"user catacomb",
            ),
        ] {
            let Some(reply) = self.read_component_states() else {
                dev_err_ratelimited!(self.dev, "enrol: cannot read component state before saving {}\n", what);
                return false;
            };
            let states = crate::sbio::ComponentStates::new(&reply);
            let Some(state) = states.state_for(who.value()) else {
                dev_err_ratelimited!(self.dev, "enrol: {} is absent at enrollment completion\n", what);
                return false;
            };
            dev_info_ratelimited!(self.dev, "enrol: completed {} state 0x{:x}\n", what, state);
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                dev_err_ratelimited!(self.dev, "enrol: {} is not active at enrollment completion\n", what);
                return false;
            }
            if state & crate::sbio::COMPONENT_STATE_SAVE_PENDING != 0 {
                if !self.save_catacomb(who, kind, what) {
                    return false;
                }
            } else if who.value() == user.value() {
                dev_err_ratelimited!(self.dev, "enrol: completed user catacomb is not marked for saving\n");
                return false;
            }
            if (who.value() == user.value()
                || who.value() == crate::sbio::CatacombUser::MASTER.value())
                && !crate::catacomb::is_saved(kind)
            {
                dev_err_ratelimited!(self.dev, "enrol: {} has no durable host file\n", what);
                return false;
            }
        }
        if !self.save_lockout() {
            dev_err_ratelimited!(self.dev, "enrol: lockout persistence failed\n");
            return false;
        }
        true
    }

    fn save_all_components_user_first(&self, user: crate::sbio::UserId) -> bool {
        if !crate::catacomb::is_saved(PRIVATE_TYPE_CATACOMB_OWNER) {
            dev_err_ratelimited!(self.dev, "enrol: opt-in completion has no owner Catacomb file\n");
            return false;
        }
        // Saving the user can change system material. Persist it before the
        // master, as in the reboot-tested J313/J414s enrollment paths.
        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::enrolling(user),
                PRIVATE_TYPE_CATACOMB_USER,
                c"opt-in completed user catacomb",
            ),
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"opt-in post-user master catacomb",
            ),
        ] {
            let Some(reply) = self.read_component_states() else {
                dev_err_ratelimited!(self.dev, "enrol: opt-in cannot read state before saving {}\n", what);
                return false;
            };
            let state = crate::sbio::ComponentStates::new(&reply).state_for(who.value());
            dev_info_ratelimited!(self.dev, "enrol: opt-in {} state {:?}\n", what, state);
            let Some(state) = state else {
                return false;
            };
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                return false;
            }
            if who.value() == user.value()
                && state & crate::sbio::COMPONENT_STATE_SAVE_PENDING == 0
            {
                dev_err_ratelimited!(self.dev, "enrol: opt-in user catacomb is not save-pending\n");
                return false;
            }
            if !self.save_catacomb(who, kind, what) {
                return false;
            }
            if !crate::catacomb::is_saved(kind) {
                dev_err_ratelimited!(self.dev, "enrol: opt-in {} has no durable host file\n", what);
                return false;
            }
        }
        if !self.save_lockout() {
            dev_err_ratelimited!(self.dev, "enrol: opt-in lockout persistence failed\n");
            return false;
        }
        true
    }

    fn save_after_match(&self, user: crate::sbio::UserId, lockout_saved: bool) -> bool {
        if !lockout_saved && !self.save_lockout() {
            dev_err_ratelimited!(
                self.dev,
                "verify: post-match lockout persistence failed; refusing to claim the AP and SEP anti-replay state are synchronized\n"
            );
            return false;
        }

        // Matching can update a template without making every component
        // saveable. The user must be considered before the master, and 0x6b
        // must be re-read after each save because saving the user can change
        // the master's state. Never claim persistence without host copies.
        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::enrolling(user),
                PRIVATE_TYPE_CATACOMB_USER,
                c"post-match user catacomb",
            ),
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"post-match master catacomb",
            ),
        ] {
            let Some(reply) = self.read_component_states() else {
                dev_err_ratelimited!(self.dev, "verify: cannot read state before saving {}\n", what);
                return false;
            };
            let states = crate::sbio::ComponentStates::new(&reply);
            let Some(state) = states.state_for(who.value()) else {
                dev_err_ratelimited!(self.dev, "verify: {} is absent after matching\n", what);
                return false;
            };
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                dev_err_ratelimited!(self.dev, "verify: {} is not active after matching (state 0x{:x})\n", what, state);
                return false;
            }
            if state & crate::sbio::COMPONENT_STATE_SAVE_PENDING != 0
                && !self.save_catacomb(who, kind, what)
            {
                dev_err_ratelimited!(self.dev, "verify: {} persistence failed\n", what);
                return false;
            }
            if !crate::catacomb::is_saved(kind) {
                dev_err_ratelimited!(self.dev, "verify: {} has no durable host file\n", what);
                return false;
            }
        }

        if !self.resnapshot_identity_keybag() {
            dev_err_ratelimited!(
                self.dev,
                "verify: post-match identity-bag snapshot failed; the newly confirmed catacomb has no matching durable bag image\n"
            );
            return false;
        }

        true
    }

    fn save_catacomb(&self, who: crate::sbio::CatacombUser, kind: u8, what: &CStr) -> bool {
        let selector = crate::sbio::SaveSelector::new(who);

        let save = match self.sbio_transfer(&crate::sbio::sbio_save_catacomb(&selector)) {
            Ok(done) if done.status.is_ok() => done,
            Ok(done) => {
                dev_err_ratelimited!(self.dev, "enrol: {} SAVE_CATACOMB refused with status {}\n", what, done.status);
                return false;
            }
            Err(e) => {
                dev_err_ratelimited!(self.dev, "enrol: {} SAVE_CATACOMB transfer failed ({:?})\n", what, e);
                return false;
            }
        };
        let blob = save.payload;
        if blob.len() < crate::sbio::SBIO_SAVED_MIN || blob.len() > crate::sbio::SBIO_SAVED_MAX {
            dev_err_ratelimited!(self.dev, "enrol: {} SAVE_CATACOMB returned invalid length {}\n", what, blob.len());
            return false;
        }

        let at = crate::sbio::SBIO_SAVED_USER_ID_AT;
        let carried = i32::from_le_bytes([blob[at], blob[at + 1], blob[at + 2], blob[at + 3]]);
        if carried != who.value() {
            dev_err_ratelimited!(self.dev, "enrol: {} SAVE_CATACOMB returned user id {}, expected {}\n", what, carried, who.value());
            return false;
        }

        if crate::catacomb::write(kind, &blob).is_err() {
            dev_err_ratelimited!(self.dev, "enrol: {} SAVE_CATACOMB host persistence failed\n", what);
            return false;
        }

        match self.sbio_transfer(&crate::sbio::sbio_confirm_save(&selector)) {
            Ok(done) if done.status.is_ok() => {
                // The master is saved last in every catacomb transaction. The
                // enclave checks a reloaded lockout record against the
                // catacombs, so one saved before this generation is refused
                // at the next start (LOAD_LOCKOUT 0x8002): save it again.
                let master = who.value() == crate::sbio::CatacombUser::MASTER.value();
                if master && !self.save_lockout() {
                    dev_warn_ratelimited!(
                        self.dev,
                        "sbio: the lockout record was not saved again after the catacombs; the next start may refuse it\n"
                    );
                }
                true
            }
            Ok(done) => {
                dev_err_ratelimited!(self.dev, "enrol: {} CONFIRM_SAVE refused with status {}\n", what, done.status);
                false
            }
            Err(e) => {
                dev_err_ratelimited!(self.dev, "enrol: {} CONFIRM_SAVE transfer failed ({:?})\n", what, e);
                false
            }
        }
    }

    fn save_lockout(&self) -> bool {
        let Some(blob) = self.sbio_expect_ok(&crate::sbio::sbio_save_lockout()) else {
            return false;
        };
        if blob.is_empty() {
            return false;
        }
        match self
            .with_host_store(|store| store.write(&store::Key::root(PRIVATE_TYPE_LOCKOUT), &blob))
        {
            Some(Ok(())) => true,
            Some(Err(_)) => false,
            None => false,
        }
    }

    fn enable_sbio(&self) -> Result<()> {
        if self.sbio_ready.load(Relaxed) {
            return Ok(());
        }

        self.register_ool(&self.ool_sbio)?;

        self.sbio_ready.store(true, Relaxed);
        let init_word = if self.profile.nonce_session {
            Some(6u32)
        } else if matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. }) {
            Some(1u32)
        } else {
            None
        };
        if let Some(word) = init_word {
            let result = self.sbio_transfer_raw(
                0x73, c"INIT_SBIO_COMMUNICATION", &word.to_le_bytes(),
            );
            match result {
                Ok(done) if done.status.is_ok()
                    && (self.profile.nonce_session || done.payload.is_empty()) => {
                    dev_info_ratelimited!(self.dev, "sbio: communication initialization accepted\n");
                }
                Ok(done) => {
                    self.sbio_ready.store(false, Relaxed);
                    dev_err_ratelimited!(self.dev, "sbio: communication init refused: status {}, {} bytes\n",
                        done.status, done.payload.len());
                    return Err(EIO);
                }
                Err(e) => {
                    self.sbio_ready.store(false, Relaxed);
                    dev_err_ratelimited!(self.dev, "sbio: communication init transport failed {:?}; aborting Touch ID setup\n", e);
                    return Err(e);
                }
            }
        }
        Ok(())
    }

    /// Brings the sensor up, starting again after a failed attempt as
    /// `retry` allows and for as long as `wanted` says the result is still
    /// needed.
    ///
    /// A bring-up that fails on the bus leaves nothing a new attempt cannot
    /// undo: each attempt power-cycles the sensor and, after a failed one,
    /// starts the registration again as [`registration::retry_stage`] says,
    /// clearing the enclave's registration and session as every verify does
    /// and, on a profile without a nonce session, every second time from a
    /// fresh registration instead.
    fn bring_sensor_online_retrying(
        &self,
        purpose: &CStr,
        kind: Bringup,
        retry: &registration::Retry,
        wanted: impl Fn() -> bool,
    ) -> registration::Outcome {
        let mut attempts = BringupAttempts {
            sep: self,
            purpose,
            wanted,
            began_ns: shim::boottime_ns(),
            attach: kind == Bringup::Attach,
            // The late round follows the failed attach-time one.
            retry: u32::from(kind == Bringup::Late),
        };
        registration::bring_up(retry, &mut attempts)
    }

    /// One attach-time bring-up. A profile without a nonce session only
    /// wakes and patches the sensor here; its first verify calibrates it.
    fn attach_bring_sensor_online(&self) -> bool {
        if self.profile.nonce_session {
            return self.bring_sensor_online();
        }
        let bind = diag::sensor_bind();
        let Some(patch) = self.wake_sensor() else {
            dev_warn_ratelimited!(self.dev, "sbio: the sensor did not wake at attach\n");
            diag::sensor_brought_up(bind, false);
            return false;
        };
        if !self.bringup_may_continue(c"the sensor's completion") {
            diag::sensor_brought_up(bind, false);
            return false;
        }
        let online = self.complete_bringup(patch);
        diag::sensor_brought_up(bind, online);
        online
    }

    /// Checked between the protocol stages of one bring-up attempt: once the
    /// system has begun going to sleep or the driver is going away, the
    /// attempt stops before `next`. The attempt then counts as failed, and a
    /// bring-up whose result is no longer wanted makes no other.
    fn bringup_may_continue(&self, next: &CStr) -> bool {
        if registration::may_continue(
            self.shutting_down.load(Relaxed),
            self.suspending.load(Relaxed),
        ) {
            return true;
        }
        dev_info_ratelimited!(
            self.dev,
            "sensor: bring-up stopped before {}: the system is going to sleep or the driver is going away\n",
            next
        );
        false
    }

    /// Whether a bring-up that came online leaves the sensor attached: on a
    /// profile with a nonce session only once the enclave's identity view
    /// is proven too.
    fn attach_complete(&self, online: bool) -> bool {
        online && (!self.profile.nonce_session || self.device_view_synced.load(Relaxed))
    }

    /// Brings the sensor up as Touch ID is activated. Returns whether the
    /// attach fell short and a late retry should follow.
    fn attach_bringup(&self) -> bool {
        if !self.profile.nonce_session && !sensor::is_bound() {
            dev_warn_ratelimited!(
                self.dev,
                "sbio: no sensor bound at attach; catacomb restore ran but capture is unavailable\n"
            );
            diag::sensor_attach(registration::AttachState::NotReady);
            return false;
        }
        let outcome = self.bring_sensor_online_retrying(
            c"attach-time",
            Bringup::Attach,
            &registration::ATTACH,
            || {
                registration::may_continue(
                    self.shutting_down.load(Relaxed),
                    self.suspending.load(Relaxed),
                )
            },
        );
        let complete = self.attach_complete(outcome.online());
        diag::sensor_attach(registration::after_round(complete, true));
        if !complete {
            if self.profile.nonce_session {
                dev_warn_ratelimited!(
                    self.dev,
                    "sensor: authenticated attach incomplete; capture unavailable; retrying in {} s\n",
                    LATE_ATTACH_DELAY_MS / 1000
                );
            } else {
                dev_warn_ratelimited!(
                    self.dev,
                    "sbio: attach-time sensor bring-up did not complete; matching unavailable; retrying in {} s\n",
                    LATE_ATTACH_DELAY_MS / 1000
                );
            }
        }
        !complete
    }

    /// One more round of sensor bring-up, some seconds after an attach-time
    /// one fell short and once the boot-time bring-up is over. Without it
    /// the sensor stayed offline until the first verify, which the login
    /// screen does not make. It runs at most once a boot, waits for neither
    /// a capture nor a suspend, and ends within the attach's attempts.
    pub(crate) fn run_late_attach(this: &Arc<SepData>) {
        if this.shutting_down.load(Relaxed) {
            return;
        }
        if this.phase.load(Relaxed) != PHASE_READY || this.suspending.load(Relaxed) {
            Self::defer_late_attach(this);
            return;
        }
        // Enrol, verify and deletion drive the sensor and the enclave under
        // the same lock, and `capture_begin` lets a suspend wait for us.
        let _writer = this.catacomb_writer.lock();
        if !this.capture_begin() {
            Self::defer_late_attach(this);
            return;
        }
        if this.attach_complete(this.device_view_synced.load(Relaxed)) {
            // An enrol or a verify got there first.
            diag::sensor_attach(registration::AttachState::Attached);
            this.capture_end();
            return;
        }
        dev_info_ratelimited!(
            this.dev,
            "sensor: retrying the attach-time bring-up after the ready phase\n"
        );
        let outcome = this.bring_sensor_online_retrying(
            c"late attach",
            Bringup::Late,
            &registration::ATTACH,
            || !this.shutting_down.load(Relaxed) && !this.suspending.load(Relaxed),
        );
        if let registration::Outcome::Abandoned(_) = outcome {
            this.capture_end();
            Self::defer_late_attach(this);
            return;
        }
        if outcome.online() {
            this.refresh_match_credential();
        }
        let complete = this.attach_complete(outcome.online());
        diag::sensor_attach(registration::after_round(complete, false));
        if complete {
            dev_info_ratelimited!(this.dev, "sensor: attached after the ready phase\n");
        } else {
            dev_warn_ratelimited!(
                this.dev,
                "sensor: authenticated attach incomplete after the late retry; capture unavailable until an enrol or verify brings the sensor up\n"
            );
        }
        this.capture_end();
    }

    /// The late attach could not run yet; try again later, a few times.
    fn defer_late_attach(this: &Arc<SepData>) {
        if this.shutting_down.load(Relaxed) {
            return;
        }
        if this.late_attach_defers.fetch_add(1, Relaxed) < LATE_ATTACH_DEFERS {
            Self::queue_late_attach(this.clone());
            return;
        }
        dev_warn_ratelimited!(
            this.dev,
            "sensor: the late attach could not run; capture unavailable until an enrol or verify brings the sensor up\n"
        );
        diag::sensor_attach(registration::AttachState::NotReady);
    }

    pub(crate) fn queue_late_attach(this: Arc<SepData>) {
        let delay = time::msecs_to_jiffies(LATE_ATTACH_DELAY_MS);
        let _ = workqueue::system_dfl()
            .enqueue_delayed::<Arc<SepData>, LATE_ATTACH_WORK_ID>(this, delay);
    }

    /// Whether the activation that just ended wants a late attach. True at
    /// most once.
    pub(crate) fn take_late_attach(&self) -> bool {
        self.late_attach_wanted.xchg(false, Relaxed)
    }

    fn bring_sensor_online(&self) -> bool {
        let bind = diag::sensor_bind();
        let Some(mut patch) = self.wake_sensor() else {
            diag::sensor_brought_up(bind, false);
            return false;
        };

        // Newer sensors require the encrypted parameters and transparent
        // channel before the calibration challenge can be serviced.
        let parameters = if self.profile.nonce_session {
            match self.apply_sensor_parameters() {
                Some(parameters) => Some(parameters),
                None => return false,
            }
        } else {
            None
        };

        if !self.sensor_calibrated.load(Relaxed) {
            if !self.calibrate_sensor() {
                diag::sensor_brought_up(bind, false);
                return false;
            }
            self.sensor_calibrated.store(true, Relaxed);
            if parameters.is_none() {
                let Some(reloaded_patch) = self.wake_sensor() else {
                    diag::sensor_brought_up(bind, false);
                    return false;
                };
                patch = reloaded_patch;
            }
        }

        if !self.bringup_may_continue(c"the sensor's completion") {
            diag::sensor_brought_up(bind, false);
            return false;
        }
        let ok = match parameters {
            Some(parameters) => self.finish_sensor_setup(patch, parameters),
            None => self.complete_bringup(patch),
        };
        diag::sensor_brought_up(bind, ok);
        // The sensor is patched and idle here -- the only safe moment to
        // configure the data-ready interrupt, which is then left alone for the
        // driver's life. Interrupt capture is the default; a machine that does
        // not describe a data-ready line falls back to SPI polling.
        if ok && !sensor::irq_available() {
            if sensor::irq_setup() {
                dev_info_ratelimited!(self.dev, "sensor: interrupt-driven capture\n");
            } else {
                dev_warn_ratelimited!(
                    self.dev,
                    "sensor: no data-ready interrupt (describe it in DT); capture falls back to SPI polling\n"
                );
            }
        }
        ok
    }

    pub(crate) fn run_bringup(&self) {
        if self.bringup_started.xchg(true, Relaxed) {
            return;
        }
        if !self.run_keystore_bringup() {
            return;
        }
        self.finish_keystore_bringup();
    }

    pub(crate) fn run_keystore_bringup(&self) -> bool {
        if let Err(e) = self.enable_sks() {
            dev_warn_ratelimited!(self.dev, "bringup: could not enable the key store ({:?}); keybag and ref-key operations are unavailable\n", e);
            return false;
        }
        if !self.sks_init_endpoint() {
            dev_err_ratelimited!(
                self.dev,
                "bringup: key-store endpoint init failed; the key store stays closed\n"
            );
            return false;
        }
        if self.keystore_user_bags {
            // keystore_only=2: a volume sealed to a user's password opens
            // here, before the real root. rearm_ioctl limits the node to the
            // unlock requests until complete_attach.
            if let Err(e) = self.register_rearm() {
                dev_err_ratelimited!(
                    self.dev,
                    "bringup: could not publish /dev/{} for the initrd password unlock: {:?}\n",
                    bio::REARM_DEVICE_NAME,
                    e
                );
            }
        }
        diag::keystore_open();
        true
    }

    pub(crate) fn finish_keystore_bringup(&self) {
        if module_parameters::provision_keybag.value() != 0 {
            if module_parameters::xart_writes.value() == 0 {
                dev_err_ratelimited!(
                    self.dev,
                    "bringup: provision_keybag=1 requires xart_writes=1\n"
                );
                return;
            }
            if !self.sks_provision_identity_keybag() {
                dev_err_ratelimited!(
                    self.dev,
                    "bringup: identity-keybag provisioning failed; no retry this boot\n"
                );
                diag::keybag(diag::Keybag::Failed);
                return;
            }
            // Either one was already persisted or one has just been.
            diag::keybag(diag::Keybag::Present);
        }
        if let Err(e) = self.register_bio() {
            dev_err_ratelimited!(
                self.dev,
                "bringup: could not publish /dev/{} after SKS became ready: {:?}\n",
                bio::DEVICE_NAME,
                e
            );
            // No activation can follow, so a pinning first-setup window ends.
            self.with_store(|store| store.close_first_setup());
        }
        if let Err(e) = self.register_rearm() {
            dev_err_ratelimited!(
                self.dev,
                "bringup: could not publish /dev/{}; a Touch ID lockout will last until a restart: {:?}\n",
                bio::REARM_DEVICE_NAME,
                e
            );
        }
        if let Err(e) = self.register_key() {
            dev_err_ratelimited!(
                self.dev,
                "bringup: could not publish /dev/{}: {:?}\n",
                crate::key::DEVICE_NAME,
                e
            );
        }
    }

    fn activate_touchid(&self) -> Result<()> {
        if self.touchid_started.load(Relaxed) {
            return Ok(());
        }
        if self.touchid_failed.load(Relaxed) {
            return Err(EIO);
        }
        // A pinning first-setup window ends when this first activation
        // returns, on every path: on a J313 the enclave rewrites the third
        // shared locker about 280 ms after the LOAD_KEYBAG reply, during the
        // requests that follow it.
        let _window = crate::sks::SetupWindow::for_activation(self);
        let mut store = store::Store::open().inspect_err(|e| {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: opening the host state failed: {:?}\n",
                e
            );
        })?;
        let index = bio::IdentityIndex::load(&mut store).inspect_err(|e| {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: loading the host identity index failed: {:?}\n",
                e
            );
        })?;
        *self.host_store.lock() = Some(store);
        *self.bio_index.lock() = index;

        self.attach_sensor();
        self.enable_sbio().inspect_err(|e| {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: registering the biometric buffers failed: {:?}\n",
                e
            );
        })?;
        let stored = match keybag::read(keybag::Slot::Identity) {
            Ok(keybag::State::Present(stored)) => stored,
            Ok(keybag::State::Absent(_)) => {
                dev_err_ratelimited!(self.dev, "Touch ID: no persisted identity keybag exists\n");
                diag::keybag(diag::Keybag::Missing);
                return Err(ENOENT);
            }
            Err(e) => {
                dev_err_ratelimited!(
                    self.dev,
                    "Touch ID: reading the persisted identity keybag failed: {:?}\n",
                    e
                );
                diag::keybag(diag::Keybag::Failed);
                return Err(e);
            }
        };
        if !self.sks_ready() {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: the key-store endpoint is unavailable\n"
            );
            return Err(ENODEV);
        }
        let (handle, uuid) = self.sks_recover(&stored).ok_or_else(|| {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: the persisted identity keybag did not recover\n"
            );
            // Only the 13.5 key store records the UUID it generated. An M2
            // Pro/Max uses it before system firmware 26.3 and variant 5 after.
            if self.profile.key_store == profile::KeyStore::Variant5
                && stored.uuid_provenance() == crate::keybag::UuidProvenance::AsGenerated
            {
                dev_err_ratelimited!(
                    self.dev,
                    "Touch ID: the keybag was made by the 13.5 key store, but this enclave speaks variant 5; if macOS was updated past 26.2, start Touch ID over (install-aurora-sep.sh --reset-touchid) and enrol again\n"
                );
            }
            // Only the variant-5 key store records the UUID read back from
            // the bag on a warm-registration Mac. Firmware that selects the
            // 13.5 key store for a bag variant 5 made is worth a report.
            if matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. })
                && self.profile.bootstrap == profile::Bootstrap::WarmRegister
                && stored.uuid_provenance() == crate::keybag::UuidProvenance::ReadBackFromBag
            {
                dev_err_ratelimited!(
                    self.dev,
                    "Touch ID: the keybag was made by the variant-5 key store, but this firmware selects the 13.5 one; please report this with the \"M2 Pro/Max on system firmware\" line\n"
                );
            }
            diag::keybag(diag::Keybag::Failed);
            EIO
        })?;
        diag::keybag(diag::Keybag::Present);
        // When Touch ID comes up before a trusted key is unsealed, this is
        // the keybag's first use this boot, and after a boot policy change
        // the enclave rewrites one session record here; see
        // crate::xart_guard.
        crate::xart_guard::policy_window(|| {
            self.sks_designate_user_keybag(handle, stored.secret())
        });
        self.sks_machine_refkey(handle, stored.secret());
        let prepared = self.cold_match_continue(handle, uuid, stored.uuid_provenance());
        self.ensure_restored_after(prepared);
        if self.attach_bringup() {
            self.late_attach_wanted.store(true, Relaxed);
        }
        self.probe_owner_export();
        self.refresh_match_credential();
        self.restore_lockout_record();
        self.touchid_started.store(true, Relaxed);
        diag::touchid(true);
        Ok(())
    }

    // The SCRD credential set up before the Catacomb restore does not survive
    // the rest of activation: J414c (variant-5 key store) and J314s (13.5 key
    // store) answered every cold-boot MATCH_RESULT with 0x1 on a usable image,
    // and a J313 showed the same symptom. Setting it up once more here was
    // enough for every match that boot, including verifies that re-register
    // the sensor with CLEAR_STATE. It runs on every profile.
    fn refresh_match_credential(&self) {
        if !self.templates_restored.load(Relaxed) {
            return;
        }
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return;
        };
        if !self.establish_scrd_match_context(user) {
            dev_warn_ratelimited!(
                self.dev,
                "Touch ID: the match credential did not re-establish after the restore; the enclave may refuse matches this boot\n"
            );
        }
    }

    fn prepare_bio_open(&self) -> Result<()> {
        if let Err(e) = self.activate_touchid() {
            self.touchid_failed.store(true, Relaxed);
            diag::touchid(false);
            dev_err_ratelimited!(
                self.dev,
                "Touch ID activation failed: {:?}; refusing retries this boot because a partial keybag load is not safely repeatable\n",
                e
            );
            return Err(e);
        }
        Ok(())
    }

    // The attach-time bring-up can fail after the Catacomb restore (the
    // sensor patch step has no retry), which leaves the device-view proof
    // undone and every verify refused until reboot. The restore itself is
    // still a valid candidate, so bring the sensor up once more and let
    // complete_bringup() finish the proof before refusing.
    fn retry_restore_proof(&self) {
        if self.templates_restored.load(Relaxed)
            || !self.cold_restore_candidate.load(Relaxed)
            || self.device_view_synced.load(Relaxed)
        {
            return;
        }
        dev_warn_ratelimited!(
            self.dev,
            "verify: attach-time bring-up left the device-view proof undone; retrying the sensor bring-up\n"
        );
        if !self.bring_sensor_online() {
            // After a failed first patch, the enclave can refuse the serial
            // re-registration that follows CLEAR_STATE, while a reboot's
            // fresh registration works. Retry once from the fresh stage.
            dev_warn_ratelimited!(
                self.dev,
                "verify: retrying the sensor bring-up from a fresh registration\n"
            );
            self.bringup.store(BRINGUP_FRESH, Relaxed);
            if !self.bring_sensor_online() {
                dev_err_ratelimited!(self.dev, "verify: the retried sensor bring-up failed\n");
            }
        }
        let _ = sensor::idle();
        self.refresh_match_credential();
    }

    /// Whether the verify this work item runs is still wanted: not cancelled,
    /// not ended by a suspend and not closed.
    fn verify_wanted(&self) -> bool {
        !self.shutting_down.load(Relaxed)
            && ((self.key_touch_active.load(Relaxed) && !self.key_touch_cancel.load(Relaxed))
                || bio::verify_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed)))
    }

    // Every verify registers the sensor again. On a profile with a nonce
    // session that registration can fail as the attach-time one does, most
    // often the first one after a system suspend, and a new attempt recovers
    // it in the same way. Without one, the failure ends the user's
    // authentication even though the sensor works.
    fn bring_sensor_online_for_verify(&self) -> bool {
        if !self.profile.nonce_session {
            return self.bring_sensor_online();
        }
        self.bring_sensor_online_retrying(c"verify", Bringup::Verify, &registration::VERIFY, || {
            self.verify_wanted()
        })
        .online()
    }

    pub(crate) fn run_verify(&self) {
        // Asked to wait for the first password since boot, as macOS does
        // after a restart: nothing is wrong with the finger or the enclave,
        // so refuse at once, before any capture.
        if self.bio_after_boot.load(Relaxed) {
            if !self.bio_lockout_logged.xchg(true, Relaxed) {
                dev_info_refusal!(
                    self.dev,
                    "verify: Touch ID waits for the first password since this boot; refusing without a capture until a password authentication\n"
                );
            }
            self.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_LOCKOUT),
                [0u8; bio::TOKEN_LEN],
            );
            return;
        }
        // A capture now would only be refused at MATCH_RESULT again, so do
        // not ask the user to touch the sensor for it.
        if self.bio_lockout.load(Relaxed) {
            if !self.bio_lockout_logged.xchg(true, Relaxed) {
                dev_info_refusal!(
                    self.dev,
                    "verify: Touch ID is locked after too many failed attempts; refusing without a capture until a password authentication re-arms it\n"
                );
            }
            self.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_LOCKOUT),
                [0u8; bio::TOKEN_LEN],
            );
            return;
        }
        if self.enclave_reports_lockout() {
            let outcome = self.enter_lockout();
            self.finish_verify(outcome, [0u8; bio::TOKEN_LEN]);
            return;
        }
        if self.enclave_reports_state(LOCK_STATE_CATACOMB_CORRUPTED) {
            dev_err_ratelimited!(
                self.dev,
                "verify: the enclave reports the Touch ID catacomb of user {} damaged; refusing before the capture. This is not a non-matching finger: the enrolment needs repair\n",
                SBIO_PROBE_USER_ID
            );
            self.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE),
                [0u8; bio::TOKEN_LEN],
            );
            return;
        }
        self.retry_restore_proof();
        if !self.templates_restored.load(Relaxed) {
            dev_err_ratelimited!(
                self.dev,
                "verify: refusing before touching the sensor; cold-match prep, restore, or device-view identity proof did not complete. This is not a non-matching finger\n"
            );
            self.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE),
                [0u8; bio::TOKEN_LEN],
            );
            return;
        }

        if self.match_scope_gen.load(Relaxed) != self.bio_open_gen.load(Relaxed) {
            if let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) {
                self.settle_owed_enclave_failures(user);
            }
        }

        let Some(token_bytes) = self.mint_token_bytes() else {
            dev_err_ratelimited!(self.dev, "verify: could not draw a result token\n");
            self.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE),
                [0u8; bio::TOKEN_LEN],
            );
            return;
        };

        // Reuse the kept sensor only for the next touch of the authentication
        // that is still open. A new authentication brings it up afresh: on
        // J815 a sensor kept idle for 57 s between two sudo prompts had reset
        // itself and lost its patch (state 9 in the next capture, boot
        // 0f4c4bbd), while a fresh bring-up takes about 40 ms.
        if self.sensor_kept.load(Relaxed)
            && (self.key_touch_active.load(Relaxed)
                || self.match_scope_gen.load(Relaxed) != self.bio_open_gen.load(Relaxed)
                || self.match_context.lock().is_none())
        {
            dev_info_ratelimited!(self.dev, "verify: a new authentication; bringing the sensor up afresh instead of reusing it\n");
            let _ = sensor::idle();
            self.sensor_kept.store(false, Relaxed);
            self.forget_finger_lift();
            sensor::power(false);
        }

        // Arm the capture once the last touch's finger is off the sensor, as
        // macOS does; a sensor brought up afresh has no touch to wait out.
        if !self.sensor_kept.load(Relaxed) {
            self.forget_finger_lift();
        } else if !self.await_finger_lift(c"verify") {
            self.finish_verify(bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR), token_bytes);
            return;
        }

        // macOS registers the sensor once and keeps it for later matches: no
        // power cycle, CLEAR_STATE or re-registration per touch. Do the same
        // while the sensor stayed up since the last verify.
        if self.sensor_kept.load(Relaxed) {
            dev_info_ratelimited!(self.dev, "verify: sensor still registered from the last verify; no new bring-up\n");
        } else if !self.bring_sensor_online_for_verify() {
            // The enclave refuses a capture from an uncalibrated sensor (0x65 answers 1).
            dev_err_ratelimited!(self.dev, "verify: the sensor did not come back online\n");
            self.finish_verify(bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR), token_bytes);
            let _ = sensor::idle();
            return;
        } else {
            self.sensor_kept.store(true, Relaxed);
        }

        // Each verify matches under its own unlock evaluation: a context whose
        // preflight leaves a biometric match outstanding, carried in the image
        // init and completed after an enrolled match.
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            self.finish_verify(bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE), token_bytes);
            return;
        };
        // An unfinished template persistence blocks matching until it is done.
        if !self.handle_bg_ops(c"before a verify") || !self.finish_owed_persistence() {
            dev_err_ratelimited!(self.dev, "verify: the catacombs are not persisted after a template update; refusing before the capture\n");
            // Not a rejected finger: ask for another try.
            self.finish_verify(bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY), token_bytes);
            return;
        }
        // macOS keeps one match operation and one unlock evaluation for every
        // touch of a Touch ID prompt and ends them when the prompt ends: on a
        // match, when it stops after the retry limit, or when the prompt goes
        // away (a captured prompt: one OPERATION_DEVICE_LIST for three touches,
        // MATCH_CANCEL only after the third). Here a prompt is one open of the
        // Touch ID device: fprintd claims it for one PAM authentication.
        let open_gen = self.bio_open_gen.load(Relaxed);
        let key_mode = self.key_touch_active.load(Relaxed);
        // A /dev/sep-key signature is its own prompt: a fresh evaluation that
        // never outlives this run, whatever prompt /dev/sep-bio has open.
        let current = if !key_mode && self.match_scope_gen.load(Relaxed) == open_gen {
            *self.match_context.lock()
        } else {
            None
        };
        let evaluation = match current {
            Some(evaluation) => {
                let touch = self.match_scope_touches.fetch_add(1, Relaxed).saturating_add(1);
                dev_info_ratelimited!(
                    self.dev,
                    "verify: touch {} of this authentication; its match operation and unlock evaluation stay open\n",
                    touch
                );
                evaluation
            }
            None => {
                self.end_match_scope(c"the start of an authentication");
                let Some(evaluation) = self.open_unlock_evaluation(user) else {
                    dev_err_ratelimited!(
                        self.dev,
                        "verify: the unlock evaluation did not open; refusing before the capture. This is not a non-matching finger\n"
                    );
                    let _ = sensor::idle();
                    self.sensor_kept.store(false, Relaxed);
                    sensor::power(false);
                    self.finish_verify(bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE), token_bytes);
                    return;
                };
                // Open the enclave's match operation before the first image, as
                // macOS does at the start of a prompt; MATCH_CANCEL ends it.
                if !self.open_match_operation() {
                    dev_err_ratelimited!(
                        self.dev,
                        "verify: the match operation did not open; refusing before the capture. This is not a non-matching finger\n"
                    );
                    self.close_unlock_evaluation(&evaluation);
                    let _ = sensor::idle();
                    self.finish_verify(bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE), token_bytes);
                    return;
                }
                *self.match_context.lock() = Some(evaluation);
                if key_mode {
                    self.key_touch_preflight(&evaluation);
                }
                self.match_scope_gen.store(open_gen, Relaxed);
                self.match_scope_touches.store(1, Relaxed);
                self.match_scope_misses.store(0, Relaxed);
                evaluation
            }
        };
        // A /dev/sep-key call owns its evaluation outright: it is deleted
        // when this goes, on every path out of this run, never kept open
        // for a later touch, and never shared with /dev/sep-bio.
        let key_context = key_mode.then(|| crate::key::OwnedContext::new(self, evaluation));

        let mut outcome = self.verify_one_image();
        // A retry straight from the capture (no finger in time, partial
        // contact, a damaged frame) compared nothing and needs no new
        // bring-up; the touch is simply taken again in the same prompt.
        // A sensor that reset under the capture is a retry for the user but
        // not a capture retry here: the prompt's match operation ends and the
        // sensor is dropped (below), so the next try starts from a bring-up.
        let sensor_reset = self.sensor_reset_in_capture.xchg(false, Relaxed);
        let capture_retry =
            matches!(outcome, bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY)) && !sensor_reset;
        let touched_at = shim::boottime_ns();
        // The touch's frames are taken: stop imaging but keep watching the
        // finger, as macOS does right after the image cleanup. The next
        // capture is armed when it is lifted.
        if matches!(outcome, bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR)) || sensor_reset {
            self.forget_finger_lift();
            let _ = sensor::idle();
        } else {
            self.pause_after_touch(c"verify");
        }
        let answered = matches!(
            outcome,
            bio::VerifyOutcome::Matched(_) | bio::VerifyOutcome::NoMatch
        );
        // A no-match below the prompt's and the overall retry limits, or a
        // touch that gave no usable frame, keeps the prompt open for its next
        // touch, as macOS keeps the match operation until LocalAuthentication
        // ends the prompt; anything else ends the prompt's match operation.
        let no_match = matches!(outcome, bio::VerifyOutcome::NoMatch);
        let miss = u32::from(no_match);
        let prompt_misses = self.match_scope_misses.load(Relaxed).saturating_add(miss);
        let total_misses = self.bio_nomatch_streak.load(Relaxed).saturating_add(miss);
        self.match_scope_misses.store(prompt_misses, Relaxed);
        let step = prompt_step(no_match, capture_retry, prompt_misses, total_misses);
        let keep_open = !key_mode && !self.enclave_reports_lockout() && step == PromptStep::KeepOpen;
        if no_match
            && !keep_open
            && prompt_misses >= PROMPT_RETRY_LIMIT
            && total_misses < VERIFY_RETRY_LIMIT
        {
            dev_info_ratelimited!(
                self.dev,
                "verify: {} failed matches in this authentication; ending its match operation as macOS ends a Touch ID prompt. Touch ID stays available ({} of {} failed attempts before a password is required)\n",
                prompt_misses,
                total_misses,
                VERIFY_RETRY_LIMIT
            );
        }
        // macOS hands a no-match to LocalAuthentication as soon as the touch
        // is finished (MATCH_POLICY and the image cleanup done), and saves the
        // lockout record and collects the background result after it, while
        // the person lifts the finger. Do the same: the next verify waits for
        // this worker, so the bookkeeping below still precedes its capture.
        let reported_early = matches!(outcome, bio::VerifyOutcome::NoMatch);
        if reported_early {
            self.finish_verify(bio::VerifyOutcome::NoMatch, token_bytes);
        }
        if keep_open {
            dev_info_ratelimited!(
                self.dev,
                "verify: {}; the match operation stays open for the next touch of this authentication\n",
                if no_match { "no match" } else { "no usable frame" }
            );
        } else {
            match self.sbio_call(&crate::sbio::sbio_match_cancel()) {
                SbioOutcome::Ok(_) => dev_info_ratelimited!(self.dev, "verify: match operation ended (MATCH_CANCEL)\n"),
                _ => {
                    dev_err_ratelimited!(self.dev, "verify: MATCH_CANCEL failed; not reporting this verify as a match\n");
                    if matches!(outcome, bio::VerifyOutcome::Matched(_)) {
                        outcome = bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
                    }
                }
            }
            let matched = matches!(outcome, bio::VerifyOutcome::Matched(_));
            // The signature runs in the evaluation the match just satisfied,
            // before it is completed (as macOS signs after MATCH_CANCEL) and,
            // if the enclave wants the completed evaluation, once more after.
            if key_mode && matched {
                self.key_touch_sign(&evaluation, c"after the match");
            }
            if matched && !self.complete_unlock_evaluation(&evaluation, user) {
                dev_err_ratelimited!(self.dev, "verify: the enclave matched but the unlock policy was not satisfied\n");
                outcome = bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
            }
            if key_mode && matched {
                self.key_touch_sign(&evaluation, c"after the completed evaluation");
            }
            *self.match_context.lock() = None;
            self.match_scope_gen.store(0, Relaxed);
            match key_context {
                Some(context) => drop(context),
                None => self.close_unlock_evaluation(&evaluation),
            }
            if key_mode && matched {
                self.key_touch_after_close(&evaluation);
            }
        }
        // macOS saves the enclave's lockout record once after every answered
        // result (SAVE_LOCKOUT, kept on disk for the boot's LOAD_LOCKOUT),
        // after MATCH_CANCEL when the prompt ends there and before it collects
        // the background result.
        let lockout_saved = answered && self.save_lockout();
        if answered {
            if lockout_saved {
                dev_info_ratelimited!(
                    self.dev,
                    "verify: lockout record saved after the result (SAVE_LOCKOUT)\n"
                );
            } else {
                dev_warn_ratelimited!(self.dev, "verify: the lockout record was not saved after the result\n");
            }
        }
        if matches!(
            outcome,
            bio::VerifyOutcome::Matched(_) | bio::VerifyOutcome::NoMatch | bio::VerifyOutcome::Failed(ENROL_STATUS_LOCKOUT)
        ) {
            // A template update must be persisted (user, then master) before a
            // match is reported.
            if !self.await_and_handle_bg_ops(c"after a match result", BG_OPS_WAIT_MS)
                && matches!(outcome, bio::VerifyOutcome::Matched(_))
            {
                dev_err_ratelimited!(self.dev, "verify: the background result was not handled; not reporting this verify as a match\n");
                outcome = bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
            }
        }
        // Keep the sensor up for the next verify unless the sensor itself
        // failed or the retry came after the capture; then the next verify
        // brings it up from a power cycle. A capture retry keeps it: the
        // prompt's match operation is still open, and a new registration
        // would clear the sensor state under it.
        if matches!(outcome, bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR))
            || (matches!(outcome, bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY)) && !capture_retry)
        {
            self.sensor_kept.store(false, Relaxed);
            self.forget_finger_lift();
            sensor::power(false);
        }
        let streak_before = self.bio_nomatch_streak.load(Relaxed);
        match &outcome {
            bio::VerifyOutcome::Matched(_) => {
                self.bio_nomatch_streak.store(0, Relaxed);
                self.enclave_failures_owed.store(false, Relaxed);
            }
            bio::VerifyOutcome::NoMatch => {
                self.bio_nomatch_streak.fetch_add(1, Relaxed);
            }
            bio::VerifyOutcome::Failed(_) => {}
        }
        // Keep the count across a restart, as the enclave keeps its own; a
        // lockout below writes the record itself.
        let streak = self.bio_nomatch_streak.load(Relaxed);
        if streak != streak_before
            && streak < VERIFY_RETRY_LIMIT
            && !self.bio_lockout.load(Relaxed)
            && !self.write_lockout_record(false)
        {
            dev_warn_ratelimited!(
                self.dev,
                "verify: could not record {} failed attempt(s) in {}; a restart forgets them\n",
                streak,
                LOCKOUT_RECORD
            );
        }
        // The enclave announces its lockout with the no-match that causes it;
        // the next verify must not ask for another touch.
        if matches!(outcome, bio::VerifyOutcome::NoMatch)
            && self.enclave_reports_lockout()
            && !self.bio_lockout.load(Relaxed)
        {
            let _ = self.enter_lockout();
        }
        // Five failed matches since the last match or password lock Touch ID
        // until a password authentication, as the enclave itself does on
        // macOS. Recorded on the host too, so the lockout holds even if the
        // enclave's own announcement is missed, and across a restart.
        if matches!(outcome, bio::VerifyOutcome::NoMatch)
            && !self.bio_lockout.load(Relaxed)
            && self.bio_nomatch_streak.load(Relaxed) >= VERIFY_RETRY_LIMIT
        {
            self.enter_retry_limit();
        }

        let definitive = matches!(
            &outcome,
            bio::VerifyOutcome::Matched(_) | bio::VerifyOutcome::NoMatch
        );
        if definitive {
            let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
                if !reported_early {
                    let failed = bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
                    self.finish_verify(failed, token_bytes);
                }
                return;
            };
            if !self.save_after_match(user, lockout_saved) {
                if reported_early {
                    dev_err_ratelimited!(
                        self.dev,
                        "verify: the state after the reported no-match was not persisted; the next verify persists it first\n"
                    );
                    self.persistence_owed.store(true, Relaxed);
                } else {
                    let failed = bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
                    self.finish_verify(failed, token_bytes);
                }
                return;
            }
        }
        dev_info_ratelimited!(
            self.dev,
            "verify: post-result steps done {} ms after the touch was finished\n",
            shim::boottime_ns().saturating_sub(touched_at) / 1_000_000
        );
        if !reported_early {
            self.finish_verify(outcome, token_bytes);
        }
    }

    // Every exit that fails a live verify logs why: userspace sees only a bare
    // "unspecified error", which cannot tell a sensor fault from a refused
    // match step.
    fn verify_one_image(&self) -> bio::VerifyOutcome {
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            dev_err_ratelimited!(self.dev, "verify: invalid probe user id\n");
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        // Never fall back to an unauthenticated match scope.
        let Some(scope) = *self.match_context.lock() else {
            dev_err_ratelimited!(
                self.dev,
                "verify: no authenticated match context; refusing before the capture. This is not a non-matching finger\n"
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        dev_info_ratelimited!(
            self.dev,
            "verify: match mode {:#x}, user {}, credential length {}, image init length {:#x}, match context generation {}\n",
            crate::sbio::SBIO_MATCH_MODE_ACM,
            user.value(),
            scope.len(),
            crate::sbio::SBIO_IMAGE_INIT_LEN + scope.len(),
            self.match_context_generation.load(Relaxed)
        );

        if let Err(e) = sensor::start_capture() {
            dev_err_ratelimited!(self.dev, "verify: could not start the capture ({:?})\n", e);
            return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
        }
        let advertised = match self.await_capture() {
            CaptureWait::Ready(n) => n,
            CaptureWait::Timeout => {
                dev_info_ratelimited!(self.dev, "verify: the capture ended without a frame\n");
                // No finger came: nothing was compared, so ask again rather
                // than report an error. PLACE names the cause.
                self.retry_cause(bio::Guidance::Place);
                return bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
            }
            CaptureWait::Fault(state) => {
                dev_err_ratelimited!(
                    self.dev,
                    "verify: the sensor fell to state {} during the capture\n",
                    state
                );
                if state == sensor::STATE_NEEDS_PATCH {
                    // The sensor reset itself: nothing was compared, and a
                    // fresh bring-up recovers it. Ask for another touch rather
                    // than fail the authentication.
                    dev_info_ratelimited!(self.dev, "verify: the sensor reset under the capture; asking for another touch after a fresh bring-up\n");
                    self.sensor_reset_in_capture.store(true, Relaxed);
                    self.retry_cause(bio::Guidance::Place);
                    return bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
                }
                return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
            }
            CaptureWait::Abandon => {
                // A cancelled verify also abandons; only a live one is a fault.
                if self.key_touch_active.load(Relaxed)
                    || bio::verify_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed))
                {
                    dev_err_ratelimited!(
                        self.dev,
                        "verify: the sensor status became unreadable during the capture\n"
                    );
                }
                return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
            }
        };

        let capture = match sensor::read_capture(advertised) {
            Ok(capture) => capture,
            Err(sensor::CaptureError::Checksum {
                advertised: sent,
                computed,
            }) => {
                dev_warn_ratelimited!(
                    self.dev,
                    "verify: frame checksum mismatch (sensor sent {:#06x}, computed {:#06x})\n",
                    sent,
                    computed
                );
                self.retry_cause(bio::Guidance::LiftAndMove);
                return bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
            }
            Err(sensor::CaptureError::Length(n)) => {
                dev_warn_ratelimited!(
                    self.dev,
                    "verify: the sensor advertised an unusable frame length {}\n",
                    n
                );
                self.retry_cause(bio::Guidance::LiftAndMove);
                return bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
            }
            Err(sensor::CaptureError::Bus(e)) => {
                dev_err_ratelimited!(self.dev, "verify: reading the frame failed ({:?})\n", e);
                return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
            }
            Err(sensor::CaptureError::NoMemory) => {
                dev_err_ratelimited!(self.dev, "verify: no memory for the frame\n");
                return bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR);
            }
        };

        if self
            .sbio_verify_step(&crate::sbio::sbio_prepare_image_processing())
            .is_none()
        {
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        let _image = ImageContext { sep: self };

        // MATCHING: byte 0x12 set, 0x13 clear; last-image flag at byte 9 (enrol byte 8)
        let init = crate::sbio::sbio_image_processing_init(
            crate::sbio::ImagePurpose::Matching,
            true,
            true,
            0,
            user,
            crate::shim::monotonic_ns(),
            Some(&scope),
        );
        if self.sbio_verify_step(&init).is_none() {
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }

        let relay = crate::sbio::SbioRelay::new(&capture);
        if self.sbio_relay(&relay).is_none() {
            dev_err_ratelimited!(
                self.dev,
                "verify: enclave refused the capture (status above); no comparison performed, not a non-match. Reported as a failure so userspace does not tell the user their finger was rejected\n"
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        drop(capture);

        let Some(assessment) = self.sbio_verify_step(&crate::sbio::sbio_image_assessment()) else {
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        if assessment.len() <= crate::sbio::ASSESS_USABLE_MATCH {
            dev_err_ratelimited!(
                self.dev,
                "verify: image assessment is {} bytes, too short to read\n",
                assessment.len()
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }

        if assessment[crate::sbio::ASSESS_USABLE_MATCH] == 0 {
            let (reason, cause) = crate::sbio::assessment_reason(&assessment);
            dev_warn_ratelimited!(self.dev, "verify: the image is not usable for matching: {}\n", reason);
            self.retry_cause(cause);
            return bio::VerifyOutcome::Failed(ENROL_STATUS_RETRY);
        }

        let result = match self.sbio_match_step() {
            MatchStep::Answer(result) => result,
            MatchStep::LockedOut => return self.enter_lockout(),
            MatchStep::Refused => return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE),
        };
        // On a no-match macOS finishes the touch before MATCH_POLICY.
        if crate::sbio::MatchResult::parse(&result).is_some_and(|p| !p.matches(user)) {
            self.no_match_tail();
        }
        // macOS reads the match policy after every answered result, before
        // the image cleanup; neither a match nor a no-match counts without it.
        match self.sbio_call(&crate::sbio::sbio_match_policy()) {
            SbioOutcome::Ok(policy) if policy.len() == crate::sbio::SBIO_MATCH_POLICY_LEN => {}
            _ => {
                dev_err_ratelimited!(self.dev, "verify: MATCH_POLICY after the match result failed\n");
                return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
            }
        }
        let Some(parsed) = crate::sbio::MatchResult::parse(&result) else {
            dev_err_ratelimited!(
                self.dev,
                "verify: the {}-byte match result did not parse\n",
                result.len()
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };

        if !parsed.matches(user) {
            // The enclave compared a usable capture and did not answer the
            // Linux user. Usually no template matched (a different finger).
            // Log the user it did answer, so a match in another user context
            // can be told apart from a plain non-match.
            dev_info_ratelimited!(
                self.dev,
                "verify: no enrolled finger matched the capture (enclave answered user {}, Linux enrolments use user {})\n",
                parsed.user_id(),
                user.value()
            );
            return bio::VerifyOutcome::NoMatch;
        }

        // match result: user id groups templates, UUID at offset 0x04 identifies one
        let identity = *parsed.identity_uuid();
        let known = self.bio_index.lock().contains_uuid(&identity);

        dev_info_ratelimited!(
            self.dev,
            "verify: enclave matched identity {} (host index {})\n",
            Hex(&identity), if known { "present" } else { "absent" }
        );

        if !known {
            dev_err_ratelimited!(
                self.dev,
                "verify: the enclave matched identity {}, which the host index does not hold\n",
                Hex(&identity)
            );
            return bio::VerifyOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }

        bio::VerifyOutcome::Matched(bio::MatchEvidence::from_enclave_reply(identity))
    }

    pub(crate) fn run_enrolment(&self) {
        self.end_match_scope(c"an enrolment");
        self.enrol_frames_accepted.store(0, Relaxed);
        self.sensor_kept.store(false, Relaxed);

        if !self.bring_sensor_online() {
            dev_err_ratelimited!(self.dev, "enrol: the sensor did not come back online\n");
            self.finish_enrolment(Err(ENROL_STATUS_SENSOR));
            let _ = sensor::idle();
            return;
        }

        let Some(open) = self.begin_enrolment_on_enclave() else {
            dev_err_ratelimited!(self.dev, "enrol: the enclave would not start an enrolment; see the preceding step\n");
            self.finish_enrolment(Err(ENROL_STATUS_ENCLAVE));
            let _ = sensor::idle();
            return;
        };

        let mut counter: u32 = 0;
        let mut enrolment_completed = false;
        let mut enrolment_persisted = false;
        let mut identity: Option<[u8; bio::UUID_LEN]> = None;
        let mut last_percent: u32 = 0;

        let outcome = loop {
            if !bio::enrol_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed)) {
                break None;
            }
            if counter >= ENROL_MAX_CAPTURES {
                let accepted = self.enrol_frames_accepted.load(Relaxed);
                dev_warn_ratelimited!(
                    self.dev,
                    "enrol: capture budget exhausted ({} captures: {} accepted, reaching {}%, {} retried); aborting\n",
                    counter,
                    accepted,
                    last_percent,
                    counter.saturating_sub(accepted)
                );
                break Some(Err(ENROL_STATUS_TOO_MANY));
            }

            match self.enrol_one_image(counter) {
                ImageOutcome::Progress {
                    stage,
                    percent,
                    complete,
                    has_template,
                } => {
                    counter = counter.saturating_add(1);
                    last_percent = last_percent.max(percent);
                    let woke = bio::enrol_advance(
                        &mut self.bio_session.lock(),
                        self.capture_epoch.load(Relaxed),
                        stage,
                        percent,
                        bio::Guidance::LiftAndMove,
                    );
                    if woke {
                        self.bio_wake();
                    }
                    // completion is the flag at offset 0xbfe, not a derived stage count
                    if complete {
                        if !has_template {
                            dev_err_ratelimited!(
                                self.dev,
                                "enrol: enclave flags completion after {} capture(s) (stage {}, {}%) but reports no template\n",
                                counter,
                                stage,
                                percent
                            );
                            break Some(Err(ENROL_STATUS_ENCLAVE));
                        }
                        enrolment_completed = true;
                        break Some(Ok(()));
                    }
                }
                ImageOutcome::Retry(cause) => {
                    counter = counter.saturating_add(1);
                    // The capture did not count. Report it once with its cause
                    // (SEP_BIO_STATUS_RETRY in the poll), so userspace can say
                    // why instead of re-capturing with no feedback.
                    if bio::enrol_retry(
                        &mut self.bio_session.lock(),
                        self.capture_epoch.load(Relaxed),
                        cause,
                    ) {
                        self.bio_wake();
                    }
                }
                ImageOutcome::NoFinger => {
                    break Some(Err(ENROL_STATUS_TIMEOUT));
                }
                ImageOutcome::Failed(status) => break Some(Err(status)),
            }

            // Each enrolment touch is a new placement: stop imaging and arm
            // the next capture when the finger is lifted, as macOS does, not
            // after a fixed pause. A spent budget ends the run at the top of
            // the loop without waiting for the lift.
            if counter < ENROL_MAX_CAPTURES {
                if self.paused_at_ns.load(Relaxed) == 0 {
                    self.pause_after_touch(c"enrol");
                }
                if !self.await_finger_lift(c"enrol") {
                    break None;
                }
            }
        };

        if enrolment_completed {
            open.completed();

            if let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) {
                if !self.save_all_components(user) {
                    dev_err_ratelimited!(
                        self.dev,
                        "enrol: enclave completed but Catacomb persistence failed; not reporting enrollment success\n"
                    );
                } else if !self.resnapshot_identity_keybag() {
                    dev_err_ratelimited!(
                        self.dev,
                        "enrol: Catacombs saved but identity-bag snapshot failed; not reporting enrollment success\n"
                    );
                } else {
                    self.templates_restored.store(true, Relaxed);
                    enrolment_persisted = true;
                }
            }

            identity = self.identity_just_enrolled();

            self.reconcile_identities();
        }

        let _ = sensor::idle();
        self.sensor_kept.store(false, Relaxed);
        self.forget_finger_lift();
        sensor::power(false);

        if let Some(result) = outcome {
            let filed = match (result, identity, enrolment_persisted) {
                (Ok(()), Some(uuid), true) => Ok(uuid),
                (Ok(()), _, _) => Err(ENROL_STATUS_UNFILED),
                (Err(status), _, _) => Err(status),
            };
            self.finish_enrolment(filed);
        }
    }

    fn sensor_ms(&self) -> u64 {
        shim::boottime_ns().saturating_sub(self.sensor_power_ns.load(Relaxed)) / 1_000_000
    }

    fn sensor_trace(&self, step: &CStr) {
        if module_parameters::sensor_trace.value() != 0 {
            dev_info_ratelimited!(self.dev, "sensor: {} done at {} ms\n", step, self.sensor_ms());
        }
    }

    /// Reports frames the sensor exchange had to read again or discard.
    fn note_sensor_recoveries(&self) {
        let (reread, drained) = sensor::take_recoveries();
        if reread != 0 || drained != 0 {
            dev_warn_ratelimited!(
                self.dev,
                "sensor: re-read {} frame(s) the sensor still offered after a garbled read, discarded {} stale frame(s)\n",
                reread,
                drained
            );
        }
    }

    /// Reports a failed registration step with what the sensor shows
    /// afterwards: its state, patch acknowledgment, whether it presents an
    /// identifier and the length of any frame it holds. Never the
    /// identifier, the serial or any key material.
    fn sensor_step_failed(&self, step: &CStr) {
        self.note_sensor_recoveries();
        let ms = self.sensor_ms();
        match sensor::status() {
            Ok(st) => dev_warn_ratelimited!(
                self.dev,
                "sensor: {} failed {} ms after power-on; sensor now state {} ack {:#04x} identifier {} holding {} bytes{}\n",
                step,
                ms,
                st.state,
                st.patch_ack(),
                st.identifier_to_register().is_some(),
                st.available().unwrap_or(0),
                if st.is_silent() { c" (silent)" } else { c"" }
            ),
            Err(e) => dev_warn_ratelimited!(
                self.dev,
                "sensor: {} failed {} ms after power-on; its status read failed too ({:?})\n",
                step,
                ms,
                e
            ),
        }
    }

    /// One enclave request of the registration. A refusal is named.
    fn sensor_enclave_step(&self, op: &crate::sbio::SbioOp) -> Option<KVec<u8>> {
        match self.sbio_transfer(op) {
            Ok(done) if done.status.is_ok() => {
                self.sensor_trace(op.name());
                Some(done.payload)
            }
            Ok(done) => {
                dev_warn_ratelimited!(self.dev, "sensor: {} refused with status {}\n", op.name(), done.status);
                self.sensor_step_failed(op.name());
                None
            }
            Err(e) => {
                dev_warn_ratelimited!(self.dev, "sensor: {} transfer failed: {:?}\n", op.name(), e);
                self.sensor_step_failed(op.name());
                None
            }
        }
    }

    fn register_sensor(&self, id: &sensor::Identifier) -> bool {
        let stage = self.bringup.load(Relaxed);
        let plan = registration::plan(
            stage,
            self.sensor_held.load(Relaxed),
            self.profile.nonce_session,
            matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. }),
        );
        dev_info_ratelimited!(
            self.dev,
            "sbio: registering sensor at bring-up stage {}{}\n",
            stage,
            if plan.clear_state && stage < BRINGUP_ESTABLISHED {
                c", clearing the enclave's partial registration first"
            } else {
                c""
            }
        );

        if plan.identify_first {
            // J700 needs identification before CLEAR_STATE on every reset.
            let platform = match Self::sensor_platform_identity() {
                Ok(platform) => platform,
                Err(error) => {
                    dev_err_ratelimited!(self.dev, "sensor: platform identity unavailable: {:?}\n", error);
                    return false;
                }
            };
            if !self.sensor_session_request(0x48, c"PLATFORM_IDENTITY", &platform) {
                self.sensor_step_failed(c"PLATFORM_IDENTITY");
                return false;
            }
            self.sensor_trace(c"PLATFORM_IDENTITY");
            // Counted as held from submission on: a reply that is lost or
            // late does not mean the enclave did not act on the request.
            self.sensor_held.store(true, Relaxed);
            if self.sensor_enclave_step(&crate::sbio::sbio_register_sensor(id)).is_none() {
                return false;
            }
        }

        if plan.clear_state {
            let _ = self.log_identity_count(c"before CLEAR_STATE for sensor re-registration");
            if self
                .sensor_enclave_step(&crate::sbio::sbio_clear_state())
                .is_none()
            {
                return false;
            }
            let _ = self.log_identity_count(c"after CLEAR_STATE for sensor re-registration");
        }

        if stage == BRINGUP_FRESH
            && matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. })
        {
            // The 13.5 J313 flow begins with REGISTER_SENSOR_SERIAL (0x17).
            // The newer REGISTER_SENSOR (0x80) returned 0x16 on this firmware.
            self.bringup.store(BRINGUP_IDENTIFIED, Relaxed);
        } else if plan.register_sensor {
            let op = crate::sbio::sbio_register_sensor(id);
            match self.sbio_call(&op) {
                SbioOutcome::Ok(_payload) => {
                    self.bringup.store(BRINGUP_IDENTIFIED, Relaxed);
                }
                _ => {
                    return false;
                }
            }
        }

        if self.profile.nonce_session {
            if !self.authenticate_sensor() {
                return false;
            }
        } else {
            let Some(serial) = self.read_serial_logged() else {
                return false;
            };
            let op = crate::sbio::sbio_register_sensor_serial(&serial);
            if self.sensor_enclave_step(&op).is_none() {
                return false;
            }
        }
        self.bringup.store(BRINGUP_ESTABLISHED, Relaxed);
        self.note_sensor_recoveries();
        let _ = self.log_identity_count(c"after sensor serial registration");
        true
    }

    fn sensor_platform_identity() -> Result<[u8; 12]> {
        let node = kernel::of::root().ok_or(ENODEV)?;
        let property: KVec<u8> = node.get_property(c"serial-number")?;
        let length = property.iter().position(|byte| *byte == 0).ok_or(EINVAL)?;
        let mut identity = [0; 12];
        if length == 0 || length > identity.len() || property.len() != length + 1 {
            return Err(EINVAL);
        }
        identity[..length].copy_from_slice(&property[..length]);
        Ok(identity)
    }

    fn sensor_session_request(&self, opcode: u16, name: &'static CStr, bytes: &[u8]) -> bool {
        match self.sbio_transfer_raw(opcode, name, bytes) {
            Ok(reply) if reply.status.is_ok() => true,
            Ok(reply) => {
                dev_err_ratelimited!(self.dev, "sensor: {} refused with status {}\n", name, reply.status);
                false
            }
            Err(error) => {
                dev_err_ratelimited!(self.dev, "sensor: {} transfer failed: {:?}\n", name, error);
                false
            }
        }
    }

    fn read_serial_logged(&self) -> Option<sensor::SensorSerial> {
        match sensor::read_sensor_serial() {
            Ok(serial) => {
                self.sensor_trace(c"serial read");
                Some(serial)
            }
            Err(e) => {
                dev_warn_ratelimited!(self.dev, "sensor: serial read failed: {:?}\n", e);
                self.sensor_step_failed(c"serial read");
                None
            }
        }
    }

    /// Authenticates the sensor on a profile with a nonce session, in the
    /// order [`registration::authenticate`] gives.
    fn authenticate_sensor(&self) -> bool {
        let mut steps = SensorAuthentication {
            sep: self,
            refused16: false,
        };
        let mut unfinished = self.serial_unfinished.load(Relaxed);
        let result = registration::authenticate(&mut steps, &mut unfinished);
        self.serial_unfinished.store(unfinished, Relaxed);
        match result {
            Ok(registration::Authenticated::Registered) => true,
            Ok(registration::Authenticated::Resumed) => {
                dev_info_ratelimited!(
                    self.dev,
                    "sensor: REGISTER_SERIAL answered 0x16 while the enclave held the unfinished registration of an earlier attempt; the session opened on that one\n"
                );
                true
            }
            Err(registration::Stopped::RegisterSerial) if steps.refused16 => {
                dev_warn_ratelimited!(self.dev, "sensor: REGISTER_SERIAL refused with status 0x16\n");
                self.sensor_step_failed(c"REGISTER_SERIAL");
                false
            }
            // Each step has said why it failed.
            Err(_) => false,
        }
    }

    /// SENSOR_NONCE, the signed root key once a boot, and the session share.
    fn sensor_nonce_session(&self, nonce: &[u8; 32]) -> bool {
        if module_parameters::sensor_test_fault.value() != 0
            && !self.sensor_test_fault_done.xchg(true, Relaxed)
        {
            dev_warn_ratelimited!(
                self.dev,
                "sensor: sensor_test_fault: stopping before SENSOR_NONCE, so that the enclave holds an unfinished registration\n"
            );
            return false;
        }
        if !self.sensor_session_request(0x81, c"SENSOR_NONCE", nonce) {
            self.sensor_step_failed(c"SENSOR_NONCE");
            return false;
        }
        self.sensor_trace(c"SENSOR_NONCE");

        if !self.sensor_key_loaded.load(Relaxed) {
            // Per-device data, so the board names it; J700 predates that.
            let name = sensor::root_key_firmware_name().unwrap_or(ROOT_KEY_FIRMWARE);
            let firmware = match kernel::firmware::Firmware::request(name, &self.dev) {
                Ok(firmware) => firmware,
                Err(error) => {
                    dev_err_ratelimited!(
                        self.dev,
                        "sensor: signed root-key container {} unavailable: {:?}\n",
                        name,
                        error
                    );
                    return false;
                }
            };
            if firmware.data().is_empty()
                || !self.sensor_session_request(0x86, c"SENSOR_ROOT_KEY", firmware.data())
            {
                self.sensor_step_failed(c"SENSOR_ROOT_KEY");
                return false;
            }
            self.sensor_key_loaded.store(true, Relaxed);
            self.sensor_trace(c"SENSOR_ROOT_KEY");
        }
        // Each identification starts a new sensor-side exchange, even though
        // the signed key remains loaded for the lifetime of the enclave.
        self.establish_session()
    }

    fn wake_sensor(&self) -> Option<PatchLoaded> {
        self.sensor_kept.store(false, Relaxed);
        let _source = sensor::power_source();
        self.sensor_power_ns.store(shim::boottime_ns(), Relaxed);
        let cycled = sensor::power_cycle();
        if !cycled && self.profile.nonce_session {
            // A sensor that was not reset may still hold the session of an
            // earlier attempt; registering it again cannot be trusted.
            dev_err_ratelimited!(self.dev, "sensor: the power cycle failed; not registering the sensor\n");
            return None;
        }
        if !sensor::cs_timing().is_hardware() {
            dev_warn_ratelimited!(
                self.dev,
                "sensor: chip-select timing not reaching hardware; software emulation cannot time this sensor, silent status expected (mode 2 is the other half)\n"
            );
        }

        let mut answered_without_identifier = false;

        for delay in sensor::POWER_ON_READ_DELAYS_MS {
            if delay > 0 {
                kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(delay)));
            }
            let st = match sensor::status() {
                Ok(st) => st,
                Err(_) => {
                    dev_warn_ratelimited!(self.dev, "sensor: status read failed on wake\n");
                    return None;
                }
            };
            if st.is_silent() {
                continue;
            }

            if let sensor::Offset12::Identifier(id) = st.offset12() {
                if id != 0 && !sensor::KNOWN_IDENTIFIERS.contains(&id) {
                    dev_err_ratelimited!(
                        self.dev,
                        "sensor: unknown Mesa identifier {:#06x} at status offset 12; not registering it with the enclave\n",
                        id
                    );
                    return None;
                }
            }

            let Some(id) = st.identifier_to_register() else {
                answered_without_identifier = true;
                continue;
            };
            if module_parameters::sensor_trace.value() != 0 {
                dev_info_ratelimited!(
                    self.dev,
                    "sensor: identifier present {} ms after power-on\n",
                    self.sensor_ms()
                );
            }
            if !self.register_sensor(&id) {
                dev_warn_ratelimited!(self.dev, "sensor: the enclave did not register the sensor\n");
                return None;
            }
            if !self.bringup_may_continue(c"the post-registration status read and the patch") {
                return None;
            }

            // Read the status again before anything else goes to the sensor,
            // on every profile. Registration can reset an authenticated
            // session, so the acknowledgement sampled before it is stale; and
            // on a T6000 the session share that followed a registration
            // without this read was dropped or answered wrongly on 4 boots in
            // 15, while the kernels that made it never failed. See
            // registration::patch_loaded for what decides.
            let st = match sensor::status() {
                Ok(st) if !st.is_silent() => st,
                Ok(_) => {
                    dev_warn_ratelimited!(
                        self.dev,
                        "sensor: status unavailable after registration: silent {} ms after power-on\n",
                        self.sensor_ms()
                    );
                    return None;
                }
                Err(e) => {
                    dev_warn_ratelimited!(
                        self.dev,
                        "sensor: status unavailable after registration: read failed ({:?}) {} ms after power-on\n",
                        e,
                        self.sensor_ms()
                    );
                    return None;
                }
            };
            if registration::patch_loaded(
                self.profile.nonce_session,
                st.patch_ack() == sensor::PATCH_ACCEPTED,
                st.state == sensor::STATE_NEEDS_PATCH,
            ) {
                return Some(PatchLoaded(()));
            }
            return self.apply_sensor_patch(st.patch_ack());
        }

        dev_err_ratelimited!(
            self.dev,
            "sensor gated: all {} wake reads returned sixteen zero bytes. Power {}. CS timing: {}. Mode 2 (CPOL=1 CPHA=0)\n",
            sensor::POWER_ON_READ_DELAYS_MS.len(),
            if cycled { c"was cycled" } else { c"was NOT cycled" },
            sensor::cs_timing().name()
        );
        if answered_without_identifier {
            return None;
        }
        None
    }

    fn calibrate_sensor(&self) -> bool {
        // The blob is per-device factory data, so its name is board data: take
        // it from the sensor node's firmware-name, falling back to the default.
        let name = sensor::firmware_name().unwrap_or(CALIBRATION_FIRMWARE);
        let blob = match kernel::firmware::Firmware::request(name, &self.dev) {
            Ok(fw) => CalibrationBlob::new(fw),
            Err(e) => {
                dev_err_ratelimited!(
                    self.dev,
                    "sensor: calibration blob {} could not be loaded ({:?}); per-device factory data, cannot be synthesised, stopping\n",
                    name,
                    e
                );
                return false;
            }
        };

        let Some(challenge) = self.sbio_expect_ok(&crate::sbio::sbio_module_challenge()) else {
            return false;
        };
        if challenge.len() != crate::sbio::SBIO_MODULE_CHALLENGE_LEN {
            return false;
        }

        let challenge = sensor::Params::new(challenge);
        if sensor::send_module_challenge(&challenge).is_err() {
            return false;
        }

        let reply = match sensor::read_module_reply() {
            Ok(reply) => reply,
            Err(_) => {
                return false;
            }
        };

        // commit the sensor's reply, not the challenge; both are 64 bytes at count 0x4b
        let Some(commit) = crate::sbio::sbio_module_commit(&reply) else {
            return false;
        };
        let Some(_serial) = self.sbio_expect_ok(&commit) else {
            return false;
        };

        let source = if self.profile.nonce_session { 6 } else { SBIO_CALIBRATION_SOURCE };
        let request = match crate::sbio::SbioCalibration::new(&blob, source) {
            Ok(request) => request,
            Err(_) => {
                return false;
            }
        };
        if !self.sensor_session_request(request.opcode(), request.name(), request.payload()) {
            dev_err_ratelimited!(self.dev, "sensor: calibration refused (source {}, {} bytes)\n",
                source, request.payload().len());
            return false;
        }

        let _ = self.sbio_call(&crate::sbio::sbio_diagnostics());

        true
    }

    fn complete_bringup(&self, patch: PatchLoaded) -> bool {
        let Some(params) = self.apply_sensor_parameters() else {
            return false;
        };

        self.finish_sensor_setup(patch, params)
    }

    fn finish_sensor_setup(&self, patch: PatchLoaded, params: ParametersApplied) -> bool {
        let op = crate::sbio::sbio_complete_init(patch, params);
        if self.sbio_expect_ok(&op).is_none() {
            dev_err_ratelimited!(
                self.dev,
                "sensor: 0x01 COMPLETE_INIT failed — status above\n"
            );
            return false;
        }

        let listed = self.sync_device_view();

        if listed && !self.device_view_synced.xchg(true, Relaxed) {
            let count = self.log_identity_count(c"after device-view synchronisation");
            dev_info_ratelimited!(self.dev, "sbio: device-view identity count {:?}, restore-ready before proof {}\n", count, self.templates_restored.load(Relaxed));
            match count {
                Some(0) => {
                    self.templates_restored.store(false, Relaxed);
                    diag::restore(diag::Restore::Empty);
                }
                Some(n) if (self.templates_restored.load(Relaxed)
                    || self.cold_restore_candidate.load(Relaxed)) && self.prove_restore() => {
                    self.templates_restored.store(true, Relaxed);
                    diag::restore(diag::Restore::Restored);
                    dev_info_ratelimited!(
                        self.dev,
                        "Touch ID: restored {} enrolled identity/identities; enrolment survived reboot\n",
                        n
                    );
                }
                Some(_) => {
                    self.templates_restored.store(false, Relaxed);
                    diag::restore(diag::Restore::Unproven);
                }
                None => {
                    self.templates_restored.store(false, Relaxed);
                    diag::restore(diag::Restore::Unproven);
                }
            }
            dev_info_ratelimited!(self.dev, "sbio: restore-ready after device-view proof {}\n", self.templates_restored.load(Relaxed));
            if self.templates_restored.load(Relaxed) {
                self.reconcile_identities();
                self.log_protected_unlock();
                self.probe_unlock_evaluation();
                let _ = self.handle_bg_ops(c"after the restore");
            }
        } else if !listed {
            dev_warn_ratelimited!(self.dev, "sbio: device-view synchronisation or match policy unavailable\n");
        }

        true
    }

    fn apply_sensor_parameters(&self) -> Option<ParametersApplied> {
        // 0x5d and 0x5c: an empty blob is a failure.
        self.relay_parameters(
            &crate::sbio::sbio_coverage_params(),
            &sensor::Geometry::COVERAGE,
            false,
        )?;
        self.relay_parameters(
            &crate::sbio::sbio_operation_params(),
            &sensor::Geometry::OPERATION,
            false,
        )?;
        // 0x6a: an empty blob is success, and relaying nothing is correct.
        self.relay_parameters(
            &crate::sbio::sbio_transparent_channel(),
            &sensor::Geometry::TRANSPARENT,
            true,
        )?;

        Some(ParametersApplied(()))
    }

    fn relay_parameters(
        &self,
        op: &crate::sbio::SbioOp,
        geom: &sensor::Geometry,
        empty_is_success: bool,
    ) -> Option<usize> {
        let blob = match self.sbio_call(op) {
            SbioOutcome::Ok(b) => sensor::Params::new(b),
            _ => {
                return None;
            }
        };

        if blob.is_empty() {
            if empty_is_success {
                return Some(0);
            }
            return None;
        }

        let relayed = blob.len();
        match sensor::send_encrypted_parameters(&blob, geom) {
            Ok(()) => Some(relayed),
            Err(sensor::ParamsError::Empty) => None,
            Err(sensor::ParamsError::TooLong(n, capacity)) => {
                dev_err_ratelimited!(
                    self.dev,
                    "sensor: {}'s blob is {} bytes and this frame holds {} (declared 0x{:x} less the 9 bytes of header and CRC)\n",
                    op.name(),
                    n,
                    capacity,
                    geom.declared()
                );
                None
            }
            Err(sensor::ParamsError::Transfer(_e)) => None,
        }
    }

    fn establish_session(&self) -> bool {
        let Some(share) = self.sensor_enclave_step(&crate::sbio::sbio_request_session_share())
        else {
            return false;
        };

        // 0x30 reserved for this reply; length is a floor, take the first 40
        if share.len() < sensor::SESSION_SHARE_LEN {
            dev_warn_ratelimited!(self.dev, "sensor: short session share ({} bytes)\n", share.len());
            return false;
        }
        let mut out = [0u8; sensor::SESSION_SHARE_LEN];
        out.copy_from_slice(&share[..sensor::SESSION_SHARE_LEN]);

        // relay it: class 0x72, no CRC, no padding
        if let Err(e) = sensor::send_session_share(&out) {
            dev_warn_ratelimited!(self.dev, "sensor: sending the session share failed: {:?}\n", e);
            self.sensor_step_failed(c"session share send");
            return false;
        }

        let reply = match sensor::read_session_reply() {
            Ok(r) => r,
            Err(e) => {
                dev_warn_ratelimited!(self.dev, "sensor: session reply read failed: {:?}\n", e);
                self.sensor_step_failed(c"session reply read");
                return false;
            }
        };

        self.sensor_enclave_step(&crate::sbio::sbio_commit_session_share(&reply))
            .is_some()
    }

    // Each failure names its step: a report that only says "sequence counter
    // init failed" cannot tell the enclave side from the sensor side.
    fn init_sequence_counter(&self) -> bool {
        let challenge = match self.sbio_call(&crate::sbio::sbio_request_challenge()) {
            SbioOutcome::Ok(c) => c,
            SbioOutcome::PrerequisiteMissing => {
                if !self.establish_session() {
                    dev_warn_ratelimited!(self.dev, "sequence counter: session setup failed\n");
                    return false;
                }
                match self.sbio_call(&crate::sbio::sbio_request_challenge()) {
                    SbioOutcome::Ok(c) => c,
                    _ => {
                        dev_warn_ratelimited!(self.dev, "sequence counter: challenge request failed after session setup\n");
                        return false;
                    }
                }
            }
            SbioOutcome::Status16 => {
                dev_warn_ratelimited!(self.dev, "sequence counter: challenge request answered status 16\n");
                return false;
            }
            SbioOutcome::Other => {
                dev_warn_ratelimited!(self.dev, "sequence counter: challenge request failed\n");
                return false;
            }
        };
        if challenge.len() < sensor::CHALLENGE_LEN {
            dev_warn_ratelimited!(self.dev, "sequence counter: short challenge ({} bytes)\n", challenge.len());
            return false;
        }
        let mut out = [0u8; sensor::CHALLENGE_LEN];
        out.copy_from_slice(&challenge[..sensor::CHALLENGE_LEN]);

        if sensor::send_challenge(&out).is_err() {
            dev_warn_ratelimited!(self.dev, "sequence counter: sending the challenge to the sensor failed\n");
            return false;
        }

        let reply = match sensor::read_challenge_reply() {
            Ok(r) => r,
            Err(_) => {
                dev_warn_ratelimited!(self.dev, "sequence counter: the sensor did not answer the challenge\n");
                return false;
            }
        };

        match self.sbio_transfer(&crate::sbio::sbio_commit_challenge(&reply)) {
            Ok(done) if done.status.is_ok() => true,
            Ok(done) => {
                dev_warn_ratelimited!(self.dev, "sequence counter: the enclave refused the sensor's reply, status {}\n", done.status);
                self.sensor_step_failed(c"challenge commit");
                false
            }
            Err(e) => {
                dev_warn_ratelimited!(self.dev, "sequence counter: committing the reply failed {:?}\n", e);
                false
            }
        }
    }

    fn apply_sensor_patch(&self, _ack_before: u8) -> Option<PatchLoaded> {
        let blob = match self.sbio_call(&crate::sbio::sbio_fetch_patch()) {
            SbioOutcome::Ok(b) => b,
            SbioOutcome::PrerequisiteMissing => {
                if !self.init_sequence_counter() {
                    dev_warn_ratelimited!(
                        self.dev,
                        "sensor patch: sequence counter init failed before the patch fetch\n"
                    );
                    return None;
                }
                match self.sbio_call(&crate::sbio::sbio_fetch_patch()) {
                    SbioOutcome::Ok(b) => b,
                    _ => {
                        dev_warn_ratelimited!(
                            self.dev,
                            "sensor patch: patch fetch failed after the sequence counter init\n"
                        );
                        return None;
                    }
                }
            }
            SbioOutcome::Status16 => {
                dev_warn_ratelimited!(self.dev, "sensor patch: patch fetch answered status 16\n");
                return None;
            }
            SbioOutcome::Other => {
                dev_warn_ratelimited!(self.dev, "sensor patch: patch fetch failed\n");
                return None;
            }
        };
        if blob.is_empty() {
            dev_warn_ratelimited!(self.dev, "sensor patch: the enclave returned an empty patch\n");
            return None;
        }

        // enable command is mandatory; skipping it leaves the sensor in state 9
        if sensor::setup_patch_enable().is_err() {
            dev_warn_ratelimited!(self.dev, "sensor patch: the patch enable command failed on the bus\n");
            return None;
        }

        if !self.await_sensor_state(sensor::STATE_IDLE, c"idle, before sending the patch") {
            dev_warn_ratelimited!(self.dev, "sensor patch: the sensor did not reach idle before the patch\n");
            return None;
        }

        if sensor::send_patch(&blob).is_err() {
            dev_warn_ratelimited!(self.dev, "sensor patch: sending the patch failed on the bus\n");
            return None;
        }

        // acceptance is at status[8], not at the state
        for _attempt in 0..PATCH_POLL_ATTEMPTS {
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(PATCH_POLL_MS)));
            let st = match sensor::status() {
                Ok(st) => st,
                Err(_) => {
                    dev_warn_ratelimited!(
                        self.dev,
                        "sensor patch: status read failed while waiting for the acknowledgment\n"
                    );
                    return None;
                }
            };
            if st.patch_ack() == sensor::PATCH_ACCEPTED {
                if let Ok(after) = sensor::status() {
                    if after.state == sensor::STATE_NEEDS_PATCH {
                        dev_warn_ratelimited!(
                            self.dev,
                            "sensor patch: acknowledged, but the sensor still asks for a patch\n"
                        );
                        return None;
                    }
                }
                return Some(PatchLoaded(()));
            }
        }

        let _ = sensor::status();
        dev_warn_ratelimited!(
            self.dev,
            "sensor patch: no acknowledgment after {} polls\n",
            PATCH_POLL_ATTEMPTS
        );
        None
    }

    fn await_sensor_state(&self, want: u8, _why: &CStr) -> bool {
        for _attempt in 0..PATCH_POLL_ATTEMPTS {
            match sensor::status() {
                Ok(st) if st.state == want => {
                    return true;
                }
                Ok(_) => {}
                Err(_) => {
                    return false;
                }
            }
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(PATCH_POLL_MS)));
        }
        false
    }

    // Like verify, every exit that ends an enrolment says why: userspace sees
    // only "driver status 0x1", which cannot tell these apart.
    fn enrol_one_image(&self, counter: u32) -> ImageOutcome {
        if let Err(e) = sensor::start_capture() {
            dev_err_ratelimited!(self.dev, "enrol: could not start capture {} ({:?})\n", counter, e);
            return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
        }

        let available = match self.await_capture() {
            CaptureWait::Ready(n) => n,
            CaptureWait::Timeout => {
                dev_info_ratelimited!(self.dev, "enrol: capture {} ended without a frame\n", counter);
                return ImageOutcome::NoFinger;
            }
            CaptureWait::Fault(state) => {
                dev_err_ratelimited!(
                    self.dev,
                    "enrol: the sensor fell to state {} during capture {}\n",
                    state,
                    counter
                );
                return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
            }
            CaptureWait::Abandon => {
                // A cancelled enrolment also abandons; only a live one is a fault.
                if bio::enrol_is_live(&self.bio_session.lock(), self.capture_epoch.load(Relaxed)) {
                    dev_err_ratelimited!(
                        self.dev,
                        "enrol: the sensor status became unreadable during capture {}\n",
                        counter
                    );
                }
                return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
            }
        };

        let capture = match sensor::read_capture(available) {
            Ok(c) => c,
            Err(sensor::CaptureError::Checksum {
                advertised: sent,
                computed,
            }) => {
                dev_warn_ratelimited!(
                    self.dev,
                    "enrol: capture {} frame checksum mismatch (sensor sent {:#06x}, computed {:#06x}); retrying\n",
                    counter,
                    sent,
                    computed
                );
                return ImageOutcome::Retry(bio::Guidance::LiftAndMove);
            }
            Err(sensor::CaptureError::Length(n)) => {
                dev_warn_ratelimited!(
                    self.dev,
                    "enrol: capture {} advertised an unusable frame length {}; retrying\n",
                    counter,
                    n
                );
                return ImageOutcome::Retry(bio::Guidance::LiftAndMove);
            }
            Err(sensor::CaptureError::Bus(e)) => {
                dev_err_ratelimited!(self.dev, "enrol: reading the frame of capture {} failed ({:?})\n", counter, e);
                return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
            }
            Err(sensor::CaptureError::NoMemory) => {
                dev_err_ratelimited!(self.dev, "enrol: no memory for the frame of capture {}\n", counter);
                return ImageOutcome::Failed(ENROL_STATUS_SENSOR);
            }
        };
        // An enrolment touch gives one frame: macOS pauses the sensor as soon
        // as it has it, while the enclave processes the frame.
        self.pause_after_touch(c"enrol");

        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            dev_err_ratelimited!(self.dev, "enrol: invalid user id before image processing\n");
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        if self
            .sbio_enrol_step(&crate::sbio::sbio_prepare_image_processing())
            .is_none()
        {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        let _image = ImageContext { sep: self };

        let init = crate::sbio::sbio_image_processing_init(
            crate::sbio::ImagePurpose::Enrolment,
            counter == 0,
            false,
            counter,
            user,
            crate::shim::monotonic_ns(),
            None,
        );
        if self.sbio_enrol_step(&init).is_none() {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }

        let relay = crate::sbio::SbioRelay::new(&capture);
        if self.sbio_relay(&relay).is_none() {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        drop(capture);

        let Some(assessment) = self.sbio_enrol_step(&crate::sbio::sbio_image_assessment()) else {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        if assessment.len() < crate::sbio::ASSESS_MIN_LEN {
            dev_err_ratelimited!(self.dev, "enrol: image assessment short ({} bytes)\n", assessment.len());
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        }
        let usable = assessment[crate::sbio::ASSESS_USABLE_ENROL] != 0;
        if !usable {
            let (reason, cause) = crate::sbio::assessment_reason(&assessment);
            dev_info_ratelimited!(
                self.dev,
                "enrol: capture {} is not usable for enrolment: {}\n",
                counter,
                reason
            );
            return ImageOutcome::Retry(cause);
        }

        let Some(result) = self.sbio_enrol_step(&crate::sbio::sbio_enrolment_result()) else {
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };
        let Some(parsed) = crate::sbio::EnrolmentResult::parse(&result) else {
            dev_err_ratelimited!(self.dev, "enrol: malformed enrolment result ({} bytes)\n", result.len());
            return ImageOutcome::Failed(ENROL_STATUS_ENCLAVE);
        };

        if parsed.complete() {
            self.stash_enrol_identity(&result);
        }

        let percent = parsed.progress_percent();
        // Coverage can stop changing while SEP continues accepting frames.
        // Count accepted captures so each one remains visible to libfprint.
        let accepted = self.enrol_frames_accepted.fetch_add(1, Relaxed) + 1;
        let stage = if parsed.complete() {
            bio::ENROL_STAGES
        } else {
            accepted.min(bio::ENROL_STAGES - 1)
        };

        ImageOutcome::Progress {
            stage,
            percent,
            complete: parsed.complete(),
            has_template: parsed.has_template(),
        }
    }

    fn await_capture(&self) -> CaptureWait {
        self.await_capture_within(u64::from(ENROL_POLL_ATTEMPTS) * u64::from(ENROL_POLL_MS))
    }

    fn await_capture_within(&self, wait_ms: u64) -> CaptureWait {
        let mut previous: Option<[u8; sensor::STATUS_LEN]> = None;
        let mut armed_reported = false;

        // Interrupt-driven capture blocks on the data-ready line instead of
        // polling; the SPI status read below still gates every frame, so a
        // missed or spurious interrupt costs at most one backstop interval. A
        // machine with no data-ready line falls back to SPI polling.
        let use_irq = sensor::irq_available();
        if use_irq {
            sensor::irq_arm();
        }

        // Bound the wait by wall-clock time, not a fixed iteration count. The
        // data-ready interrupt can wake early on a stray edge (the sensor
        // toggles the line as the finger moves between enrol frames); a fixed
        // attempt budget would then be spent in milliseconds and time the
        // capture out before the frame lands. Time-bounding degrades, at worst,
        // to the same duration as the poll path.
        let deadline_ns = crate::shim::monotonic_ns().saturating_add(wait_ms.saturating_mul(1_000_000));
        let mut attempt: u32 = 0;
        loop {
            if !self.capture_live() {
                return CaptureWait::Abandon;
            }

            let st = match sensor::status() {
                Ok(st) => st,
                Err(_) => {
                    return CaptureWait::Abandon;
                }
            };

            if attempt == 0 || previous != Some(st.raw) {
                previous = Some(st.raw);
            }

            if st.state == sensor::STATE_NEEDS_PATCH {
                return CaptureWait::Fault(st.state);
            }

            let guide = match st.state {
                sensor::STATE_ARMED => Some(bio::Guidance::Place),
                sensor::STATE_READING => Some(bio::Guidance::HoldStill),
                _ => None,
            };
            if let Some(guide) = guide {
                if bio::capture_guide(
                    &mut self.bio_session.lock(), self.capture_epoch.load(Relaxed), guide,
                ) {
                    self.bio_wake();
                }
            }

            if st.state == sensor::STATE_ARMED && !armed_reported {
                armed_reported = true;
            }

            if let sensor::Offset12::Available(count) = st.offset12() {
                if count == 0 {
                    return CaptureWait::Timeout;
                }
                return CaptureWait::Ready(count);
            }

            if crate::shim::monotonic_ns() >= deadline_ns {
                break;
            }

            if use_irq {
                // Block on the data-ready line; the interrupt wakes this the
                // instant a frame is ready, and the backstop bounds a missed
                // edge. Re-arm for the next frame.
                let _ = sensor::irq_wait(ENROL_IRQ_WAIT_MS);
                sensor::irq_arm();
            } else {
                kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(ENROL_POLL_MS)));
            }
            attempt = attempt.saturating_add(1);
        }
        let _ = sensor::status();
        CaptureWait::Timeout
    }

    pub(crate) fn on_sbio(&self, msg: Message) {
        let f = proto::decode(&msg);
        let marker = f.tag;

        // An unsolicited lock-state notification carries everything in the
        // message word and nothing in the out-of-line buffer.
        if marker == SBIO_NOTIFY_LOCK_STATE {
            self.on_lock_state_notification(msg.msg0);
            return;
        }
        if marker == SBIO_NOTIFY_BG_OPS_READY {
            // Collected from process context; this runs in the receive path.
            dev_info_ratelimited!(self.dev, "sbio: the enclave has a background operation result ready\n");
            self.bg_ops_ready.store(true, Relaxed);
            self.sbio_wq.notify_all();
            return;
        }
        if marker == SBIO_NOTIFY_ENROLLED_COUNT {
            let user = (msg.msg0 >> proto::MSG_DATA_SHIFT) as u32;
            let count = (msg.msg0 >> proto::MSG_TYPE_SHIFT) as u16;
            let plural = if count == 1 { "y" } else { "ies" };
            dev_info_ratelimited!(
                self.dev,
                "sbio: user {} now has {} enrolled identit{} in the enclave\n",
                user,
                count,
                plural
            );
        } else if marker == SBIO_NOTIFY_DEVICE_LIST {
            dev_info_ratelimited!(self.dev, "sbio: the enclave's biometric device list changed\n");
        } else if marker < transfer::MARKER_FIRST {
            dev_info_ratelimited!(self.dev, "sbio: unhandled notification, tag {:#04x}\n", marker);
        }

        if !self.sbio_ready.load(Relaxed) {
            return;
        }
        // Before reading the reply: a requester that times out meanwhile and
        // begins a new exchange must not receive it.
        let generation = self.sbio_rx.lock().generation();

        // notification and the enclave's DMA are not ordered; buffer may still be poison
        let packet = match transfer::take_header(
            &SbioHeader(self),
            marker,
            OOL_POISON_OUTBOUND,
            OOL_WRITE_POLL_ATTEMPTS,
        ) {
            Ok(p) => p,
            Err(status) => {
                self.sbio_fail_with(status);
                return;
            }
        };

        let chunk = if transfer::carries_payload(marker) {
            packet.chunk as usize
        } else {
            0
        };
        let payload = if chunk == 0 {
            KVec::new()
        } else if chunk > OOL_SIZE_SBIO - transfer::HEADER_LEN {
            self.sbio_fail_with(transfer::DeviceStatus::Malformed);
            return;
        } else if !self.ool_await_written(&self.ool_sbio, transfer::HEADER_LEN, chunk) {
            self.sbio_fail_with(transfer::DeviceStatus::BufferNeverWritten);
            return;
        } else {
            match self.ool_read(&self.ool_sbio, transfer::HEADER_LEN, chunk) {
                Ok(p) => p,
                Err(_) => {
                    self.sbio_fail();
                    return;
                }
            }
        };

        let progress = self.sbio_rx.lock().on_chunk(generation, marker, &packet, &payload);

        match progress {
            transfer::Progress::NeedMore(cont) => {
                self.sbio_continue(&cont);
            }
            transfer::Progress::Complete => {
                // 0xFE requests the peer's next packet and acks the final one; nothing to send
                self.sbio_wq.notify_all();
            }
            transfer::Progress::Ignored => {}
            transfer::Progress::Grant => {
                self.sbio_wq.notify_all();
            }
            transfer::Progress::Notification => {}
            transfer::Progress::Failed => {
                self.sbio_wq.notify_all();
            }
        }
    }

    // The enclave announces each change of a user's biometric lock state:
    // the state in bits 16-31, the user in bits 32-63. Only this metadata is
    // logged; `LOCK_STATE_NEEDS_PASSWORD` are the lockout bits.
    fn on_lock_state_notification(&self, word: u64) {
        let state = (word >> proto::MSG_TYPE_SHIFT) as u16;
        let user = (word >> proto::MSG_DATA_SHIFT) as u32;
        let word = u64::from(state) | (u64::from(user) << 16);
        self.bio_lock_state.store(word, Relaxed);
        self.bio_lock_state_seq.fetch_add(1, Relaxed);
        let lockout = if state & LOCK_STATE_NEEDS_PASSWORD != 0 { "set" } else { "clear" };
        let n = lock_state_names(state);
        // The enclave repeats a user's state after each query; log each
        // user's state when it changes, and a repeat only at debug level.
        if self.lock_state_changed(word) {
            dev_info_ratelimited!(
                self.dev,
                "sbio: lock state of user {} is {:#06x} (lockout {}:{}{}{}{}{}{}{})\n",
                user,
                state,
                lockout,
                n[0],
                n[1],
                n[2],
                n[3],
                n[4],
                n[5],
                n[6]
            );
        } else {
            dev_dbg!(
                self.dev,
                "sbio: lock state of user {} is still {:#06x}\n",
                user,
                state
            );
        }
        self.sbio_wq.notify_all();
    }

    /// Records `word` (state and user, as in `bio_lock_state`) as the last
    /// lock state logged for its user. Returns false if it is that user's
    /// last logged state already. A user not in the table takes the slot of
    /// one that is, in turn, so a new user is always logged.
    fn lock_state_changed(&self, word: u64) -> bool {
        let user = word >> 16;
        let slots = &self.bio_lock_logged;
        let mut free = None;
        for (i, slot) in slots.iter().enumerate() {
            let seen = slot.load(Relaxed);
            if seen == 0 {
                free.get_or_insert(i);
            } else if seen >> 16 == user {
                if seen == word {
                    return false;
                }
                slot.store(word, Relaxed);
                return true;
            }
        }
        let i = free.unwrap_or_else(|| (self.bio_lock_state_seq.load(Relaxed) as usize) % slots.len());
        slots[i].store(word, Relaxed);
        true
    }

    /// Waits up to `timeout_ms` for a lock-state notification for `user`
    /// newer than `since` (a `bio_lock_state_seq` value). Returns its state.
    fn await_lock_state(&self, user: u32, since: u32, timeout_ms: u32) -> Option<u16> {
        let mut waited = 0;
        loop {
            if self.bio_lock_state_seq.load(Relaxed) != since {
                let last = self.bio_lock_state.load(Relaxed);
                if (last >> 16) as u32 == user {
                    return Some(last as u16);
                }
            }
            if waited >= timeout_ms {
                return None;
            }
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(
                LOCK_STATE_POLL_MS,
            )));
            waited += LOCK_STATE_POLL_MS;
        }
    }

    /// Collects a background operation result the enclave announced, if
    /// any, as macOS does after every match. Logs only status and length.
    /// Collects one announced background operation result with an empty
    /// 0x21. The reply must be status 0 and exactly
    /// `SBIO_BG_OPS_RESULT_LEN` bytes; otherwise the result stays pending.
    fn collect_bg_ops_result(&self, why: &CStr) -> BgOps {
        if !self.bg_ops_ready.xchg(false, Relaxed) {
            return BgOps::Nothing;
        }
        let done = match self.sbio_transfer(&crate::sbio::sbio_bg_ops_result()) {
            Ok(done) if done.status.is_ok() && done.payload.len() == crate::sbio::SBIO_BG_OPS_RESULT_LEN => done,
            Ok(done) => {
                dev_err_ratelimited!(
                    self.dev,
                    "sbio: background result ({}): status {}, {} bytes; left pending\n",
                    why,
                    done.status,
                    done.payload.len()
                );
                self.bg_ops_ready.store(true, Relaxed);
                return BgOps::Failed;
            }
            Err(e) => {
                dev_err_ratelimited!(self.dev, "sbio: background result ({}) transfer failed: {:?}; left pending\n", why, e);
                self.bg_ops_ready.store(true, Relaxed);
                return BgOps::Failed;
            }
        };
        let kind = crate::sbio::bg_ops_result_kind(&done.payload);
        dev_info_ratelimited!(self.dev, "sbio: background result ({}): {:?}\n", why, kind);
        match kind {
            crate::sbio::BgOpsResult::TemplatesUpdated => BgOps::TemplatesUpdated,
            _ => BgOps::NotUpdated,
        }
    }

    /// Collects an announced result and, when it updated the templates,
    /// persists them. Returns false when matching must stop.
    fn handle_bg_ops(&self, why: &CStr) -> bool {
        match self.collect_bg_ops_result(why) {
            BgOps::Nothing | BgOps::NotUpdated => true,
            BgOps::Failed => false,
            BgOps::TemplatesUpdated => self.persist_template_update(),
        }
    }

    fn await_and_handle_bg_ops(&self, why: &CStr, timeout_ms: u32) -> bool {
        let mut waited = 0;
        while !self.bg_ops_ready.load(Relaxed) && waited < timeout_ms {
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(LOCK_STATE_POLL_MS)));
            waited += LOCK_STATE_POLL_MS;
        }
        self.handle_bg_ops(why)
    }

    /// The state bits of one component, from a fresh 0x6b.
    fn component_state(&self, who: crate::sbio::CatacombUser) -> Option<u32> {
        let reply = self.read_component_states()?;
        crate::sbio::ComponentStates::new(&reply).state_for(who.value())
    }

    /// Waits until `who` is active and save-pending.
    fn await_save_pending(&self, who: crate::sbio::CatacombUser) -> bool {
        let mut waited = 0;
        loop {
            if self.component_state(who).is_some_and(|s| {
                s & crate::sbio::COMPONENT_STATE_ACTIVE != 0
                    && s & crate::sbio::COMPONENT_STATE_SAVE_PENDING != 0
            }) {
                return true;
            }
            if waited >= BG_OPS_SAVE_WAIT_MS {
                return false;
            }
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(BG_OPS_SAVE_POLL_MS)));
            waited += BG_OPS_SAVE_POLL_MS;
        }
    }

    /// A template update makes the user and then the master component
    /// save-pending; macOS saves them as a pair, user first, master last.
    /// A pair saved across generations makes the next boot's user load cold.
    /// Each side is awaited, saved durably and confirmed in turn, then both
    /// must show no pending save. Until that holds the obligation stays owed
    /// and matching stays off.
    fn persist_template_update(&self) -> bool {
        self.persistence_owed.store(true, Relaxed);
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return false;
        };
        let user_c = crate::sbio::CatacombUser::enrolling(user);
        if !self.await_save_pending(user_c) {
            dev_err_ratelimited!(self.dev, "sbio: template update: the user catacomb never became save-pending\n");
            return false;
        }
        if !self.save_catacomb(user_c, PRIVATE_TYPE_CATACOMB_USER, c"user catacomb after a template update") {
            return false;
        }
        self.finish_owed_persistence()
    }

    /// Saves the master last once it is pending, then proves both sides
    /// clean. Retried before each verify while an obligation is owed.
    fn finish_owed_persistence(&self) -> bool {
        if !self.persistence_owed.load(Relaxed) {
            return true;
        }
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return false;
        };
        let user_c = crate::sbio::CatacombUser::enrolling(user);
        // A user save left pending by an earlier failure goes first.
        if self.component_state(user_c).is_some_and(|s| s & crate::sbio::COMPONENT_STATE_SAVE_PENDING != 0)
            && !self.save_catacomb(user_c, PRIVATE_TYPE_CATACOMB_USER, c"user catacomb after a template update")
        {
            return false;
        }
        // macOS appends the master to every template-update save, last, and
        // does not wait for it to report a pending save of its own: after the
        // user save it usually reports none. A pair saved without it makes the
        // next boot's user load cold.
        let master = crate::sbio::CatacombUser::MASTER;
        if !self.component_state(master).is_some_and(|s| s & crate::sbio::COMPONENT_STATE_ACTIVE != 0) {
            dev_err_ratelimited!(self.dev, "sbio: template update: the master catacomb is not active\n");
            return false;
        }
        if !self.save_catacomb(master, PRIVATE_TYPE_CATACOMB_MASTER, c"master catacomb after a template update") {
            return false;
        }
        let clean = |s: Option<u32>| {
            s.is_some_and(|s| {
                s & crate::sbio::COMPONENT_STATE_ACTIVE != 0 && s & crate::sbio::COMPONENT_STATE_SAVE_PENDING == 0
            })
        };
        if !clean(self.component_state(user_c)) || !clean(self.component_state(master)) {
            dev_err_ratelimited!(self.dev, "sbio: template update: a component is still save-pending after the pair was saved\n");
            return false;
        }
        if !self.resnapshot_identity_keybag() {
            dev_err_ratelimited!(self.dev, "sbio: template update: identity-bag snapshot failed\n");
            return false;
        }
        self.persistence_owed.store(false, Relaxed);
        dev_info_ratelimited!(self.dev, "sbio: template update persisted (user, then master)\n");
        true
    }

    fn sbio_continue(&self, cont: &transfer::Continuation) {
        let mut header = [0u8; transfer::HEADER_LEN];
        if cont.packet().encode(&mut header).is_err() {
            self.sbio_fail();
            return;
        }
        if self.ool_write(&self.ool_sbio, 0, &header).is_err() {
            self.sbio_fail();
            return;
        }
        if self.send(crate::sbio::encode_sbio_continue(cont)).is_err() {
            self.sbio_fail();
        }
    }

    fn sbio_fail(&self) {
        self.sbio_rx.lock().abort();
        self.sbio_wq.notify_all();
    }

    fn sbio_fail_with(&self, status: transfer::DeviceStatus) {
        self.sbio_rx.lock().abort_with(status);
        self.sbio_wq.notify_all();
    }

    fn sbio_transfer(&self, op: &crate::sbio::SbioOp) -> Result<transfer::Completed> {
        self.sbio_transfer_raw(op.opcode(), op.name(), op.payload())
    }

    fn sbio_transfer_raw(
        &self,
        opcode: u16,
        name: &'static CStr,
        payload: &[u8],
    ) -> Result<transfer::Completed> {
        // The capture work item and the ioctl path both reach this endpoint.
        // Without this, the second caller is refused with EBUSY, or clears a
        // reply the first has not collected yet and takes the next one. Taken
        // before the shutdown check, so a waiter never starts after remove().
        let _exchange = self.sbio_exchange_lock.lock();
        if self.shutting_down.load(Relaxed) || !self.sbio_ready.load(Relaxed) {
            return Err(ENODEV);
        }

        self.sbio_rx.lock().begin(u32::from(opcode))?;

        if let Err(e) = self.sbio_start(opcode, name, payload) {
            let mut guard = self.sbio_rx.lock();
            guard.abort();
            let _ = guard.take_done();
            return Err(e);
        }

        let mut remaining = time::msecs_to_jiffies(SBIO_TIMEOUT_MS);
        let mut guard = self.sbio_rx.lock();
        loop {
            if self.shutting_down.load(Relaxed) {
                guard.abort();
                let _ = guard.take_done();
                return Err(ENODEV);
            }
            if let Some(done) = guard.take_done() {
                return Ok(done);
            }
            if remaining == 0 {
                guard.abort();
                let _ = guard.take_done();
                drop(guard);
                return Err(ETIMEDOUT);
            }
            // A reply wait sleeps uninterruptibly: with a signal pending an
            // interruptible wait returns at once with the time left unchanged,
            // so this loop would spin until the reply came, or forever.
            match self
                .sbio_wq
                .wait_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { .. } => remaining = 0,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }

    fn sbio_start(&self, opcode: u16, name: &'static CStr, payload: &[u8]) -> Result<()> {
        if crate::sbio::encode_sbio_raw(opcode, 0).is_none() {
            return Err(EPERM);
        }

        let total = payload.len();
        if total > transfer::MAX_TRANSACTION as usize {
            return Err(EINVAL);
        }

        let capacity = OOL_SIZE_SBIO - transfer::HEADER_LEN;

        self.sbio_rx.lock().begin_send(u32::from(opcode));
        let result = self.sbio_send_chunks(opcode, name, payload, capacity);
        self.sbio_rx.lock().end_send();
        result
    }

    fn sbio_send_chunks(
        &self,
        opcode: u16,
        name: &'static CStr,
        payload: &[u8],
        capacity: usize,
    ) -> Result<()> {
        let total = payload.len();
        let mut offset = 0usize;
        let mut first = true;
        let mut seq: u16 = 0;
        let mut request = KVec::new();

        loop {
            let chunk = core::cmp::min(total - offset, capacity);

            let packet = transfer::Packet {
                version: 1,
                total: total as u32,
                offset: offset as u32,
                flags: 0,
                err: 0,
                opcode: u32::from(opcode),
                chunk: chunk as u32,
            };

            request.clear();
            request.resize(transfer::HEADER_LEN + chunk, 0u8, GFP_KERNEL)?;
            packet.encode(&mut request)?;
            request[transfer::HEADER_LEN..].copy_from_slice(&payload[offset..offset + chunk]);

            self.ool_write(&self.ool_sbio, 0, &request)?;

            let msg = if first {
                crate::sbio::encode_sbio_raw(opcode, seq)
            } else {
                crate::sbio::encode_sbio_next(opcode, seq)
            }
            .ok_or(EPERM)?;
            // The last grant's header may sit in the outbound buffer; the
            // reply to this chunk must not take it.
            transfer::send_for_reply(&SbioHeader(self), || self.send(msg))?;

            offset += chunk;
            first = false;
            seq = seq.wrapping_add(1);
            if offset >= total {
                return Ok(());
            }

            self.sbio_await_grant(name, offset, total)?;
        }
    }

    fn sbio_await_grant(&self, _name: &'static CStr, _sent: usize, _total: usize) -> Result<()> {
        let mut remaining = time::msecs_to_jiffies(SBIO_TIMEOUT_MS);
        let mut guard = self.sbio_rx.lock();
        loop {
            if self.shutting_down.load(Relaxed) {
                return Err(ENODEV);
            }
            if guard.take_grant() {
                return Ok(());
            }
            if remaining == 0 {
                drop(guard);
                return Err(ETIMEDOUT);
            }
            // A reply wait sleeps uninterruptibly: with a signal pending an
            // interruptible wait returns at once with the time left unchanged,
            // so this loop would spin until the reply came, or forever.
            match self
                .sbio_wq
                .wait_timeout(&mut guard, remaining)
            {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Signal { .. } => remaining = 0,
                CondVarTimeoutResult::Timeout => remaining = 0,
            }
        }
    }

    /// Publishes a chardev into `slot` (`bio_dev`, `rearm_dev`, `key_dev`).
    ///
    /// The slot's lock is not held across `misc_register()`. `misc_open()`
    /// calls the shim's open under `misc_mtx`, the shim runs every callback
    /// under its per-device lock, and the release and ioctl callbacks take the
    /// slot lock to wake pollers. Holding the slot lock while registering
    /// would close the cycle slot -> `misc_mtx` -> device lock -> slot.
    ///
    /// Only the settle work registers. A device that lost the slot, or that
    /// came up after `remove()` emptied the slots, is dropped (unregistered)
    /// again, outside the slot lock. Returns whether `dev` was published.
    pub(crate) fn publish_chardev(
        &self,
        slot: &Mutex<Option<shim::BioChardev>>,
        register: impl FnOnce() -> Result<shim::BioChardev>,
    ) -> Result<bool> {
        if slot.lock().is_some() {
            return Ok(false);
        }
        let dev = register()?;
        let mut guard = slot.lock();
        if guard.is_some() || self.shutting_down.load(Relaxed) {
            drop(guard);
            drop(dev);
            return Ok(false);
        }
        *guard = Some(dev);
        Ok(true)
    }

    pub(crate) fn register_bio(&self) -> Result<()> {
        let ctx = core::ptr::from_ref(self).cast_mut().cast::<c_void>();

        let published = self.publish_chardev(&self.bio_dev, || {
            // SAFETY: `ctx` is this `SepData`, kept alive by the `Arc` in the
            // driver's private data. `remove()` drops the registration before that
            // `Arc` can go, and the shim's unregister clears its stored context
            // under the same lock every callback takes, so once it returns, no
            // callback can observe this pointer again.
            unsafe {
                shim::BioChardev::register(
                    bio::DEVICE_NAME,
                    bio::DEVICE_MODE,
                    ctx,
                    bio_open_trampoline,
                    bio_release_trampoline,
                    bio_ioctl_trampoline,
                    bio_ready_trampoline,
                )
            }
        })?;
        if published {
            diag::bio_published();
        }
        Ok(())
    }

    pub(crate) fn bio_open(&self) -> Result<()> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        let mut session = self.bio_session.lock();
        self.wait_for_worker_exit(&mut session, true)?;
        bio::open(&mut session)?;
        drop(session);
        if let Err(e) = self.prepare_bio_open() {
            bio::release(&mut self.bio_session.lock());
            return Err(e);
        }
        self.bio_open_gen.fetch_add(1, Relaxed);
        Ok(())
    }

    pub(crate) fn bio_release(&self) {
        bio::release(&mut self.bio_session.lock());
        if let Some(dev) = self.bio_dev.lock().as_ref() {
            dev.wake();
        }
    }

    /// Called by the enrol and verify work items before they touch the sensor,
    /// and by a deletion before it touches the enclave. Returns false if the
    /// system is going to sleep; the session has then been ended and the
    /// capture or deletion must not start. A true return must be paired with
    /// [`Self::capture_end`].
    pub(crate) fn capture_begin(&self) -> bool {
        let mut session = self.bio_session.lock();
        if self.suspending.load(Relaxed) {
            let woke = bio::abort_capture(&mut session);
            drop(session);
            if woke {
                self.bio_wake();
            }
            return false;
        }
        self.captures_running.fetch_add(1, Relaxed);
        true
    }

    pub(crate) fn capture_end(&self) {
        self.captures_running.fetch_sub(1, Relaxed);
    }

    /// The enrol or verify work item `epoch` has stopped.
    pub(crate) fn capture_worker_stopped(&self, epoch: u64) {
        bio::worker_stopped(&mut self.bio_session.lock(), epoch);
        self.worker_gone.notify_all();
    }

    /// Holds an open or a capture start while the last enrol or verify work
    /// item is still on its way out. It reports its result before it stops,
    /// and after a cancel or a close it stops only once the capture notices,
    /// so an open or a start straight after either would be refused with
    /// EBUSY, which the caller cannot tell from a device in use. If it has
    /// not stopped in time, the caller refuses as before.
    fn wait_for_worker_exit(
        &self,
        session: &mut MutexGuard<'_, bio::Session>,
        opening: bool,
    ) -> Result<()> {
        let mut remaining = time::msecs_to_jiffies(WORKER_EXIT_WAIT_MS);
        while bio::waiting_on_worker(session, opening) {
            if self.shutting_down.load(Relaxed) {
                return Err(ENODEV);
            }
            if remaining == 0 {
                dev_warn_ratelimited!(
                    self.dev,
                    "Touch ID: the last capture had not stopped after {} ms\n",
                    WORKER_EXIT_WAIT_MS
                );
                return Ok(());
            }
            match self.worker_gone.wait_interruptible_timeout(session, remaining) {
                CondVarTimeoutResult::Woken { jiffies } => remaining = jiffies,
                CondVarTimeoutResult::Timeout => remaining = 0,
                CondVarTimeoutResult::Signal { .. } => return Err(ERESTARTSYS),
            }
        }
        Ok(())
    }

    /// The system is about to suspend. End any capture in progress, hold new
    /// ones, and wait for the enrol or verify work to stop touching the sensor
    /// before devices go down.
    pub(crate) fn sleep_prepare(&self) {
        // Power may be removed across the sleep; register afresh after it.
        self.sensor_kept.store(false, Relaxed);
        // A prompt does not carry across a sleep: the next verify ends it.
        self.match_scope_gen.store(0, Relaxed);
        let woke = {
            let mut session = self.bio_session.lock();
            self.suspending.store(true, Relaxed);
            bio::abort_capture(&mut session)
        };
        if woke {
            dev_info_ratelimited!(
                self.dev,
                "Touch ID: ending the capture in progress before sleep\n"
            );
            self.bio_wake();
        }

        let mut waited: u32 = 0;
        while self.captures_running.load(Relaxed) != 0 {
            if waited >= SLEEP_DRAIN_MS {
                // Suspend anyway, as before: failing here would leave a
                // laptop awake with its lid shut. The work item has been told
                // to stop, and the next capture power-cycles and registers
                // the sensor again.
                dev_warn_ratelimited!(
                    self.dev,
                    "Touch ID: a capture was still running {} ms after the system started to sleep; continuing\n",
                    SLEEP_DRAIN_MS
                );
                return;
            }
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(10));
            waited += 10;
        }
    }

    /// The system has resumed, or the suspend was aborted.
    pub(crate) fn sleep_finished(&self) {
        let _session = self.bio_session.lock();
        self.suspending.store(false, Relaxed);
        self.sleep_over.notify_all();
    }

    /// Holds a capture start until the system is awake. Userspace is thawed
    /// before the driver hears of the resume, so a lock screen that retries at
    /// once would otherwise be refused. A start made just before the system
    /// sleeps is interrupted by the freezer and restarted after resume.
    fn wait_until_awake(&self) -> Result<()> {
        let mut session = self.bio_session.lock();
        while self.suspending.load(Relaxed) {
            if self.sleep_over.wait_interruptible(&mut session) {
                return Err(ERESTARTSYS);
            }
        }
        Ok(())
    }

    pub(crate) fn queue_enrolment(this: Arc<SepData>) {
        let Some(epoch) = bio::queued_epoch(&this.bio_session.lock()) else {
            return;
        };
        this.capture_epoch.store(epoch, Relaxed);
        if workqueue::system_dfl()
            .enqueue::<Arc<SepData>, ENROL_WORK_ID>(this.clone())
            .is_err()
        {
            this.finish_enrolment(Err(ENROL_STATUS_SENSOR));
            this.capture_worker_stopped(epoch);
        }
    }

    pub(crate) fn queue_verify(this: Arc<SepData>) {
        let Some(epoch) = bio::queued_epoch(&this.bio_session.lock()) else {
            return;
        };
        this.capture_epoch.store(epoch, Relaxed);
        if workqueue::system_dfl()
            .enqueue::<Arc<SepData>, VERIFY_WORK_ID>(this.clone())
            .is_err()
        {
            this.finish_verify(
                bio::VerifyOutcome::Failed(ENROL_STATUS_SENSOR),
                [0u8; bio::TOKEN_LEN],
            );
            this.capture_worker_stopped(epoch);
        }
    }

    fn finish_verify(&self, outcome: bio::VerifyOutcome, token_bytes: [u8; bio::TOKEN_LEN]) {
        // A touch-gated /dev/sep-key signature owns this verify: its result
        // goes to that ioctl, never to /dev/sep-bio's session.
        if self.key_touch_active.load(Relaxed) {
            if let Some(job) = &mut *self.key_touch.lock() {
                job.set_outcome(&outcome);
            }
            return;
        }
        let filed = bio::verify_finish(
            &mut self.bio_session.lock(), self.capture_epoch.load(Relaxed), outcome, token_bytes,
        );
        if filed {
            self.bio_wake();
        }
    }

    /// MATCH_RESULT, with the lockout told apart from any other refusal.
    fn sbio_match_step(&self) -> MatchStep {
        let op = crate::sbio::sbio_match_result();
        let done = match self.sbio_transfer(&op) {
            Ok(done) => done,
            Err(e) => {
                dev_err_ratelimited!(self.dev, "verify: {} transfer failed ({:?})\n", op.name(), e);
                return MatchStep::Refused;
            }
        };
        match done.status.answered() {
            Some(err) if err as u16 == crate::sbio::SBIO_STATUS_OK => MatchStep::Answer(done.payload),
            Some(err) if err == u32::from(crate::sbio::SBIO_STATUS_PREREQUISITE) => {
                MatchStep::LockedOut
            }
            _ => {
                dev_err_ratelimited!(self.dev, "verify: {} answered status {}\n", op.name(), done.status);
                MatchStep::Refused
            }
        }
    }

    // MATCH_RESULT answers 0x1 when the match credential no longer holds.
    // After failed matches that is the enclave's lockout, which on macOS the
    // passcode clears. The credential is set up again only after a password
    // authentication, in `rearm_after_password`, never here: doing it on the
    // enclave's refusal would undo the lockout.
    /// Whether the enclave's latest lock-state notification for the Linux
    /// user says matching needs the password first (a lockout, a password
    /// lockout, or no password unlock yet).
    /// Whether the enclave's latest lock-state notification for the Linux
    /// user has any of `bits` set.
    fn enclave_reports_state(&self, bits: u16) -> bool {
        let last = self.bio_lock_state.load(Relaxed);
        self.bio_lock_state_seq.load(Relaxed) != 0
            && (last >> 16) as u32 == SBIO_PROBE_USER_ID as u32
            && (last as u16) & bits != 0
    }

    fn enclave_reports_lockout(&self) -> bool {
        self.enclave_reports_state(LOCK_STATE_NEEDS_PASSWORD)
    }

    /// Locks Touch ID on the host after `VERIFY_RETRY_LIMIT` failed matches
    /// since the last match or re-arm. A password authentication re-arms it,
    /// and the record keeps the lock across a restart, as for the enclave's
    /// own lockout.
    fn enter_retry_limit(&self) {
        dev_info_ratelimited!(
            self.dev,
            "verify: {} failed attempts since the last match or password; Touch ID is locked until a password authentication (the retry limit)\n",
            self.bio_nomatch_streak.load(Relaxed)
        );
        self.bio_lockout.store(true, Relaxed);
        self.bio_lockout_logged.store(true, Relaxed);
        if !self.write_lockout_record(true) {
            dev_warn_ratelimited!(
                self.dev,
                "verify: could not record the lockout in {}; it ends at the next restart\n",
                LOCKOUT_RECORD
            );
        }
    }

    fn enter_lockout(&self) -> bio::VerifyOutcome {
        let failures = self.bio_nomatch_streak.load(Relaxed);
        if !self.enclave_reports_lockout() {
            // A refusal with the user's lockout bit clear is the match
            // scope's own limit, not the user's biometric lockout: it lasts
            // until a new authenticated context, and a restart ends it, so it
            // is not recorded across restarts.
            dev_warn_ratelimited!(
                self.dev,
                "verify: MATCH_RESULT refused after {} failed attempts while the enclave reports user {} unlocked: a match-scope refusal; Touch ID needs a password authentication for a new match context\n",
                failures,
                SBIO_PROBE_USER_ID
            );
            self.bio_lockout.store(true, Relaxed);
            self.bio_lockout_logged.store(true, Relaxed);
            return bio::VerifyOutcome::Failed(ENROL_STATUS_LOCKOUT);
        }
        if failures > 0 {
            dev_warn_ratelimited!(
                self.dev,
                "verify: the enclave stopped matching after {} failed attempts (MATCH_RESULT answered status 0x1); Touch ID is locked until a password authentication\n",
                failures
            );
        } else {
            dev_warn_ratelimited!(
                self.dev,
                "verify: MATCH_RESULT answered status 0x1 with no failed attempt since the last match; the match credential lapsed, and Touch ID is locked until a password authentication sets it up again\n"
            );
        }
        self.bio_lockout.store(true, Relaxed);
        self.bio_lockout_logged.store(true, Relaxed);
        if !self.write_lockout_record(true) {
            dev_warn_ratelimited!(
                self.dev,
                "verify: could not record the lockout in {}; it ends at the next restart\n",
                LOCKOUT_RECORD
            );
        }
        bio::VerifyOutcome::Failed(ENROL_STATUS_LOCKOUT)
    }

    /// Whether the record says Touch ID was locked. A missing or unreadable
    /// record is no lockout: only root can remove it, and root can restart.
    fn read_lockout_record(&self) -> bool {
        let Ok(file) = shim::StoreFile::open_readonly(LOCKOUT_RECORD) else {
            return false;
        };
        if file.size().ok() != Some(LOCKOUT_RECORD_LEN as u64) {
            return false;
        }
        let mut raw = [0u8; LOCKOUT_RECORD_LEN];
        if file.read_exact(0, &mut raw).is_err() {
            return false;
        }
        lockout_record_locked(&raw)
    }

    /// The failed matches the record holds; 0 when it is missing or foreign.
    fn read_recorded_failures(&self) -> u32 {
        let Ok(file) = shim::StoreFile::open_readonly(LOCKOUT_RECORD) else {
            return 0;
        };
        let mut raw = [0u8; LOCKOUT_RECORD_LEN];
        if file.size().ok() != Some(LOCKOUT_RECORD_LEN as u64)
            || file.read_exact(0, &mut raw).is_err()
        {
            return 0;
        }
        lockout_record_failures(&raw)
    }

    fn write_lockout_record(&self, locked: bool) -> bool {
        let Ok(file) = shim::StoreFile::open_trunc(LOCKOUT_RECORD) else {
            return false;
        };
        let failures = self.bio_nomatch_streak.load(Relaxed);
        file.write_all(0, &lockout_record(locked, failures)).is_ok() && file.sync().is_ok()
    }

    // macOS asks for the passcode after a restart before Touch ID works
    // again. Here Touch ID may unlock the login screen, so a lockout has to
    // outlast the restart instead, or restarting would reset the attempts.
    fn restore_lockout_record(&self) {
        let failures = self.read_recorded_failures();
        if failures > 0 {
            self.bio_nomatch_streak.store(failures, Relaxed);
            dev_info_ratelimited!(
                self.dev,
                "Touch ID: {} failed attempt(s) from before the restart still count ({} lock Touch ID until a password)\n",
                failures,
                VERIFY_RETRY_LIMIT
            );
        }
        if self.read_lockout_record() {
            self.bio_lockout.store(true, Relaxed);
            dev_info_ratelimited!(
                self.dev,
                "Touch ID: the lockout from before the restart still holds; a password authentication re-arms it\n"
            );
        }
    }

    pub(crate) fn register_rearm(&self) -> Result<()> {
        let ctx = core::ptr::from_ref(self).cast_mut().cast::<c_void>();
        self.publish_chardev(&self.rearm_dev, || {
            // SAFETY: as in `register_bio`: `remove()` drops the registration
            // before the `Arc` that keeps `ctx` alive, and the shim's unregister
            // clears the stored context under the lock every callback takes.
            unsafe {
                shim::BioChardev::register(
                    bio::REARM_DEVICE_NAME,
                    bio::REARM_DEVICE_MODE,
                    ctx,
                    rearm_open_trampoline,
                    rearm_release_trampoline,
                    rearm_ioctl_trampoline,
                    rearm_ready_trampoline,
                )
            }
        })?;
        Ok(())
    }

    /// Reports the lockout, and why Touch ID needs the password, so that
    /// login screens can say which. Read-only, on both nodes.
    fn write_lockout_state(&self, arg: usize) -> Result<()> {
        let (locked, failures) = if self.touchid_started.load(Relaxed) {
            (
                self.bio_lockout.load(Relaxed),
                self.bio_nomatch_streak.load(Relaxed),
            )
        } else {
            (self.read_lockout_record(), self.read_recorded_failures())
        };
        let failed_attempts = locked
            && lockout_after_failures(
                failures,
                self.enclave_reports_state(LOCK_STATE_LOCKOUT | LOCK_STATE_BIO_LOCKOUT),
            );
        let state = bio::Lockout::new(
            locked,
            self.bio_after_boot.load(Relaxed),
            failed_attempts,
            failures,
        );
        kernel::uaccess::UserSlice::new(
            kernel::uaccess::UserPtr::from_addr(arg),
            core::mem::size_of::<bio::Lockout>(),
        )
        .writer()
        .write(&state)
    }

    pub(crate) fn rearm_ioctl(&self, cmd: u32, arg: usize) -> Result<()> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        // In the initrd phase the node serves only a password unlock; Touch
        // ID and the host state are not attached yet.
        if self.keystore_only.load(Relaxed) && !crate::userbag::is_initrd_ioctl(cmd) {
            return Err(EAGAIN);
        }
        // The per-user key bag carries login passwords: root only, on this
        // node, never on /dev/sep-bio, which fprintd holds.
        if crate::userbag::is_ioctl(cmd) {
            return self.userbag_ioctl(cmd, arg).map(|_| ());
        }
        match cmd {
            bio::IOC_LOCKOUT_STATE => self.write_lockout_state(arg),
            bio::IOC_REARM => {
                bio::require_admin()?;
                self.rearm_after_password(bio::read_rearm(arg)?)
            }
            bio::IOC_REQUIRE_PASSWORD => {
                bio::require_admin()?;
                if !self.bio_after_boot.xchg(true, Relaxed) {
                    self.bio_lockout_logged.store(false, Relaxed);
                    dev_info_ratelimited!(
                        self.dev,
                        "Touch ID: waits for the first password since this boot (as macOS after a restart); every verify is refused until then\n"
                    );
                }
                Ok(())
            }
            _ => Err(ENOTTY),
        }
    }

    /// Userspace calls this only after the user's password was verified. The
    /// node is root's alone and its SELinux label confines it to the re-arm
    /// helper. The verified password authorizes a password-mode verification
    /// of the designated user's key bag with its stored secret, which is what
    /// ends the enclave's lockout. A new SCRD match context is then set up and
    /// the enclave's lockout record is saved.
    fn rearm_after_password(&self, _password: crate::Secret) -> Result<()> {
        // The first password since boot is all the after-boot wait needs;
        // a lockout or failed matches are handled below as for any password.
        if self.bio_after_boot.xchg(false, Relaxed) {
            self.bio_lockout_logged.store(false, Relaxed);
            dev_info_ratelimited!(
                self.dev,
                "Touch ID: armed by the first password since this boot\n"
            );
        }
        self.import_lockout_after_password();
        if !self.touchid_started.load(Relaxed) {
            // Activation sets up a fresh match credential; only the record of
            // a lockout from before the restart is left to clear.
            if self.read_lockout_record() {
                if !self.write_lockout_record(false) {
                    dev_err_ratelimited!(self.dev, "Touch ID: could not clear the recorded lockout\n");
                    return Err(EIO);
                }
                dev_info_ratelimited!(
                    self.dev,
                    "Touch ID: re-armed after a password authentication; cleared the lockout recorded before the restart\n"
                );
            }
            return Ok(());
        }
        let user = crate::sbio::UserId::new(SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
        if !self.bio_lockout.load(Relaxed) {
            self.clear_failures_after_password(user);
            return Ok(());
        }
        // Under the same lock as verify, enrolment and deletion, so the
        // credential never changes under a match.
        let rearmed = loop {
            self.wait_until_awake()?;
            let _writer = self.catacomb_writer.lock();
            if !self.bio_lockout.load(Relaxed) {
                // Another re-arm got there first.
                return Ok(());
            }
            if !self.capture_begin() {
                continue;
            }
            let outcome = self.clear_enclave_lockout(user);
            if outcome.is_some() {
                self.bio_nomatch_streak.store(0, Relaxed);
                self.bio_lockout_logged.store(false, Relaxed);
                self.bio_lockout.store(false, Relaxed);
            }
            self.capture_end();
            break outcome;
        };
        let Some(confirmed) = rearmed else {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: re-arming after a password authentication failed; Touch ID stays locked\n"
            );
            return Err(EIO);
        };
        if !self.write_lockout_record(false) {
            dev_warn_ratelimited!(
                self.dev,
                "Touch ID: could not clear the recorded lockout; the next restart will ask for the password again\n"
            );
        }
        if confirmed {
            dev_info_ratelimited!(
                self.dev,
                "Touch ID: re-armed after a password authentication; the enclave reported the lockout cleared\n"
            );
        } else {
            dev_warn_ratelimited!(
                self.dev,
                "Touch ID: re-armed after a password authentication, but the enclave announced no lock state; whether it matches again is unconfirmed\n"
            );
        }
        Ok(())
    }

    /// A password authentication while Touch ID is not locked. On macOS the
    /// password's verification of the user's key bag clears the enclave's
    /// failed-attempt count, not only the prompt's: after a missed prompt
    /// and a password, the next prompt again has five attempts. Clear the
    /// host's count and record at once, then the enclave's with the same
    /// key bag verification a re-arm does. A verify holds the sensor lock
    /// for its whole wait for a touch, and a lock screen takes the password
    /// while one waits, so never hold the password authentication up behind
    /// it: when the lock is taken, the enclave's count is cleared at the
    /// start of the next authentication instead.
    fn clear_failures_after_password(&self, user: crate::sbio::UserId) {
        let failures = self.bio_nomatch_streak.xchg(0, Relaxed);
        if failures == 0 {
            return;
        }
        if !self.write_lockout_record(false) {
            dev_warn_ratelimited!(
                self.dev,
                "Touch ID: could not record the cleared failed attempts in {}\n",
                LOCKOUT_RECORD
            );
        }
        let Some(_writer) = self.catacomb_writer.try_lock() else {
            self.enclave_failures_owed.store(true, Relaxed);
            dev_info_ratelimited!(
                self.dev,
                "Touch ID: a password authentication cleared {} failed attempt(s); a capture is running, so the enclave's count is cleared at the start of the next authentication\n",
                failures
            );
            return;
        };
        if !self.capture_begin() {
            self.enclave_failures_owed.store(true, Relaxed);
            dev_info_ratelimited!(
                self.dev,
                "Touch ID: a password authentication cleared {} failed attempt(s); suspending, so the enclave's count is cleared at the start of the next authentication\n",
                failures
            );
            return;
        }
        self.enclave_failures_owed.store(false, Relaxed);
        let cleared = self.clear_enclave_lockout(user).is_some();
        self.capture_end();
        self.report_enclave_failures_cleared(failures, cleared);
    }

    /// The enclave's count still owed a clear by a password that came while
    /// a capture ran. Called at the start of an authentication, with the
    /// sensor lock held.
    fn settle_owed_enclave_failures(&self, user: crate::sbio::UserId) {
        if !self.enclave_failures_owed.xchg(false, Relaxed) {
            return;
        }
        let cleared = self.clear_enclave_lockout(user).is_some();
        self.report_enclave_failures_cleared(0, cleared);
    }

    fn report_enclave_failures_cleared(&self, failures: u32, cleared: bool) {
        if cleared {
            if failures > 0 {
                dev_info_ratelimited!(
                    self.dev,
                    "Touch ID: a password authentication cleared {} failed attempt(s), in the enclave too\n",
                    failures
                );
            } else {
                dev_info_ratelimited!(
                    self.dev,
                    "Touch ID: the enclave's failed attempts were cleared after the password that came during a capture\n"
                );
            }
        } else {
            dev_warn_ratelimited!(
                self.dev,
                "Touch ID: a password authentication cleared the host's failed attempts, but the enclave's verification did not complete; the enclave may still lock at its own count, and the next password then re-arms in full\n"
            );
        }
    }

    /// The enclave side of a re-arm. `Some(true)` when the enclave announced
    /// the user's lock state with the lockout bit clear, `Some(false)` when
    /// it accepted the password verification but announced nothing, `None`
    /// on any failure or when it still reports the lockout.
    fn clear_enclave_lockout(&self, user: crate::sbio::UserId) -> Option<bool> {
        let du = crate::sks::DesignateUser::new(SBIO_PROBE_USER_ID)?;
        if !self.keybag_designated.load(Relaxed) || self.enrol_material.lock().is_none() {
            dev_err_ratelimited!(self.dev, "Touch ID: re-arm: no user key bag is designated\n");
            return None;
        }
        let stored = match keybag::read(keybag::Slot::Identity) {
            Ok(keybag::State::Present(stored)) => stored,
            Ok(keybag::State::Absent(_)) => {
                dev_err_ratelimited!(self.dev, "Touch ID: re-arm: there is no stored identity key bag secret\n");
                return None;
            }
            Err(e) => {
                dev_err_ratelimited!(
                    self.dev,
                    "Touch ID: re-arm: the identity key bag record is unreadable: {:?}\n",
                    e
                );
                return None;
            }
        };
        let notify_user = user.value() as u32;
        let since = self.bio_lock_state_seq.load(Relaxed);
        let Some(out) = self.sks_send(self.sks_req_verify_password(du.special_handle(), stored.secret()))
        else {
            dev_err_ratelimited!(self.dev, "Touch ID: re-arm: the password verification did not answer\n");
            return None;
        };
        if out.reply.status != 0 {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: re-arm: the password verification answered SKS status {}\n",
                out.reply.status
            );
            return None;
        }
        let body = self.sks_report_response(crate::sks::SKS_VERIFY_SECRET_NAME, &out);
        let Some(flags) = body.and_then(crate::sks::verify_secret_reply_flags) else {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: re-arm: the password verification reply is malformed ({} body bytes)\n",
                body.map_or(0, |b| b.len())
            );
            return None;
        };
        dev_info_ratelimited!(
            self.dev,
            "Touch ID: re-arm: password verification of the designated bag answered status 0, returned flags {:#x}\n",
            flags
        );
        let confirmed = match self.await_lock_state(notify_user, since, LOCK_STATE_WAIT_MS) {
            Some(state) if state & LOCK_STATE_NEEDS_PASSWORD != 0 => {
                dev_err_ratelimited!(
                    self.dev,
                    "Touch ID: re-arm: the enclave still reports the lockout (state {:#06x})\n",
                    state
                );
                return None;
            }
            Some(_) => true,
            None => false,
        };
        // The next verify opens a fresh unlock evaluation; nothing of the old
        // one is carried over.
        self.end_match_scope(c"a password re-arm");
        let _ = self.handle_bg_ops(c"at a password re-arm");
        // The password verification alone leaves matching refused: the old
        // cold-match credentials stay live until a restart of the enclave,
        // and a restart is the only thing that has ever ended the refusal.
        // Redo what a restart's cold-match preparation does, without the
        // Catacomb loads: retire every cold-match credential, unlock the
        // designated bag with its secret, and set up one new credential.
        let (deleted, refused) = self.retire_cold_match_contexts();
        let unlocked = self.unlock_special_bag();
        if !self.establish_scrd_match_context(user) {
            dev_err_ratelimited!(
                self.dev,
                "Touch ID: re-arm: no new cold-match credential after retiring {} ({} refused); Touch ID stays locked\n",
                deleted,
                refused
            );
            return None;
        }
        dev_info_ratelimited!(
            self.dev,
            "Touch ID: re-arm: cold-match preparation redone (retired {}, refused {}, special unlock {}, new credential true)\n",
            deleted,
            refused,
            unlocked
        );
        // Bring the sensor up from a power cycle for the next verify, as
        // after a restart.
        let _ = sensor::idle();
        self.sensor_kept.store(false, Relaxed);
        sensor::power(false);
        if !self.save_lockout() {
            dev_warn_ratelimited!(self.dev, "Touch ID: re-arm: the enclave's lockout record was not saved; a restart restores the older one\n");
        }
        Some(confirmed)
    }

    fn mint_token_bytes(&self) -> Option<[u8; bio::TOKEN_LEN]> {
        let mut bytes = [0u8; bio::TOKEN_LEN];
        // This is a host-issued, per-operation capability, not SEP match
        // evidence. Use the initialized kernel CSPRNG, as for other host
        // secrets, rather than trusting firmware control replies as entropy.
        // J313/13.5 answered those draws with zero and caused valid matches
        // to carry an invalid all-zero token to userspace.
        if shim::random_bytes(&mut bytes).is_err() || bytes.iter().all(|b| *b == 0) {
            return None;
        }
        Some(bytes)
    }

    fn require_linux_bio_lane(&self) -> Result<()> {
        if !self.touchid_started.load(Relaxed) || self.xarm.lock().os_uuid.is_none() {
            return Err(ENXIO);
        }
        if !self.keybag_designated.load(Relaxed)
            || self.enrol_material.lock().is_none()
            || !matches!(keybag::read(keybag::Slot::Identity),
                         Ok(keybag::State::Present(_))) {
            return Err(ENODATA);
        }
        Ok(())
    }

    /// Refuse before ENROL_START schedules work or sends BEGIN_ENROL. A
    /// userspace passwd lookup proves the host account; these are the durable
    /// Linux-lane bindings that only the kernel can inspect.
    fn preflight_enrol(&self, live_count: Option<usize>) -> Result<()> {
        let lane = self.touchid_started.load(Relaxed) && self.xarm.lock().os_uuid.is_some();
        let bag_bound = self.keybag_designated.load(Relaxed) && self.enrol_material.lock()
            .as_ref().is_some_and(|material| material.snapshot_identity.is_some());
        let kinds = [PRIVATE_TYPE_CATACOMB_OWNER, PRIVATE_TYPE_CATACOMB_USER,
                     PRIVATE_TYPE_CATACOMB_MASTER];
        let readable = kinds.map(|kind| !catacomb_needed_for_enrol(kind, live_count)
            || crate::catacomb::is_saved(kind));
        let writable = kinds.map(|kind| !catacomb_needed_for_enrol(kind, live_count)
            || crate::catacomb::writable(kind));
        let index = self.bio_index.lock();
        let preconditions = EnrolPrerequisites {
            lane,
            live_count,
            bag_bound,
            catacomb_readable: readable,
            catacomb_writable: writable,
            keybag_writable: shim::StoreFile::open_writable_existing(
                keybag::Slot::Identity.path()).is_ok(),
            host_store_capacity: self.with_host_store(|store| index.can_persist_next(store))
                .unwrap_or(false),
        };
        let result = admit_enrol(&preconditions);
        if let Err(e) = &result {
            dev_warn_ratelimited!(self.dev, "enrol: prerequisites failed before enclave capture: {:?}\n", e);
        }
        result
    }

    fn bio_list_enclave(&self, arg: usize) -> Result<bio::Handled> {
        bio::require_admin()?;
        self.require_linux_bio_lane()?;
        let ids = self.enclave_identities_for(SBIO_PROBE_USER_ID).ok_or(EIO)?;
        let mut out = bio::EnclaveList::new();
        for uuid in ids.iter() {
            out.push(*uuid)?;
        }
        kernel::uaccess::UserSlice::new(
            kernel::uaccess::UserPtr::from_addr(arg), core::mem::size_of::<bio::EnclaveList>()
        ).writer().write(&out)?;
        bio::ok()
    }

    fn bio_delete_orphan(&self, arg: usize) -> Result<bio::Handled> {
        bio::require_admin()?;
        self.require_linux_bio_lane()?;
        if !bio::idle_for_mutation(&self.bio_session.lock()) { return Err(EBUSY); }
        let request: bio::Delete = kernel::uaccess::UserSlice::new(
            kernel::uaccess::UserPtr::from_addr(arg), core::mem::size_of::<bio::Delete>()
        ).reader().read()?;
        // The exact SEP user and built-in selector are checked by the parser.
        // The host index may still hold an orphan after fprintd lost its file,
        // so that index cannot decide whether this exact-ID recovery is valid.
        let ids = self.enclave_identities_for(SBIO_PROBE_USER_ID).ok_or(EIO)?;
        admissible_orphan_target(&ids, &request.uuid)?;
        bio::orphan_delete(crate::sbio::IdentityV1::from_index(
            SBIO_PROBE_USER_ID, request.uuid))
    }

    fn bio_prepare_enrol(&self, arg: usize) -> Result<bio::Handled> {
        bio::require_admin()?;
        bio::validate_enrol_start_arg(arg)?;
        self.require_linux_bio_lane()?;
        if !bio::idle_for_mutation(&self.bio_session.lock()) { return Err(EBUSY); }
        if !sensor::is_bound() { return Err(ENODEV); }
        if !self.xart_writable() { return Err(EROFS); }
        let ids = self.enclave_identities_for(SBIO_PROBE_USER_ID).ok_or(EIO)?;
        if !ids.is_empty() {
            // An existing identity must already have a durable owner file;
            // selecting SYSTEM here could disturb its live context.
            return if crate::catacomb::is_saved(PRIVATE_TYPE_CATACOMB_OWNER) {
                bio::ok()
            } else { Err(ENODATA) };
        }
        if crate::catacomb::is_saved(PRIVATE_TYPE_CATACOMB_OWNER) {
            return bio::ok();
        }
        // An unreadable owner record is not permission to select a new context.
        if !crate::catacomb::is_absent(PRIVATE_TYPE_CATACOMB_OWNER) {
            return Err(EIO);
        }
        if !self.keybag_designated.load(Relaxed)
            || !self.enrol_material.lock().as_ref()
                .is_some_and(|material| material.snapshot_identity.is_some()) {
            return Err(ENODATA);
        }
        if shim::StoreFile::open_writable_existing(keybag::Slot::Identity.path()).is_err() {
            return Err(EROFS);
        }
        let index = self.bio_index.lock();
        if !self.with_host_store(|store| index.can_persist_next(store)).unwrap_or(false) {
            return Err(ENOSPC);
        }
        drop(index);
        let user = crate::sbio::UserId::new(SBIO_PROBE_USER_ID).ok_or(EINVAL)?;
        // The host must be able to create and sync the owner file before a
        // system-context write. A pre-existing invalid file is left intact.
        crate::catacomb::prepare_missing_owner()?;
        loop {
            self.wait_until_awake()?;
            let _writer = self.catacomb_writer.lock();
            if !self.capture_begin() { continue; }
            let prepared = self.open_enrolment_context(user);
            self.capture_end();
            if !prepared { return Err(EIO); }
            break;
        }
        if !crate::catacomb::is_saved(PRIVATE_TYPE_CATACOMB_OWNER)
            || !crate::catacomb::writable(PRIVATE_TYPE_CATACOMB_OWNER) {
            return Err(EROFS);
        }
        self.empty_context_prepared.store(true, Relaxed);
        bio::ok()
    }

    pub(crate) fn bio_ioctl(&self, cmd: u32, arg: usize) -> Result<bio::Handled> {
        if self.shutting_down.load(Relaxed) {
            return Err(ENODEV);
        }
        if cmd == bio::IOC_ATTEST {
            return self.bio_attest(arg);
        }
        if cmd == bio::IOC_LIST_ENCLAVE {
            return self.bio_list_enclave(arg);
        }
        if cmd == bio::IOC_PREPARE_ENROL {
            return self.bio_prepare_enrol(arg);
        }
        // fprintd holds this node, not the re-arm one: it reads here why a
        // verify was refused, to tell the user.
        if cmd == bio::IOC_LOCKOUT_STATE {
            self.write_lockout_state(arg)?;
            return bio::ok();
        }
        // With xART writes off the enclave cannot persist what an enrolment or
        // a match changes. It saves anti-replay state after every match, and a
        // refused save has left it unresponsive until reboot. Refuse the
        // capture instead; attach and the read-only ioctls still work.
        if bio::starts_capture(cmd) && !self.xart_writable() {
            if !self.capture_refusal_logged().xchg(true, Relaxed) {
                dev_warn_ratelimited!(
                    self.dev,
                    "Touch ID: enrol and verify need xart_writes=1; the enclave saves anti-replay state after every match and stops responding when that save is refused\n"
                );
            }
            return Err(EROFS);
        }
        // A capture started while the system goes to sleep would run into the
        // suspend; `capture_begin` repeats this check under the session lock.
        if bio::starts_capture(cmd) {
            self.wait_until_awake()?;
        }
        // Query SEP outside the bio session/index locks. A host index can
        // outlive its SEP identity after a failed cold restore; userspace
        // must not mistake that index for proof of a live template.
        let live_identity_count = if (cmd == bio::IOC_GET_INFO || cmd == bio::IOC_ENROL_START)
            && self.touchid_started.load(Relaxed)
        {
            self.enclave_identities_for(SBIO_PROBE_USER_ID).map(|ids| ids.len())
        } else {
            None
        };
        if cmd == bio::IOC_ENROL_START {
            bio::require_admin()?;
            bio::validate_enrol_start_arg(arg)?;
            self.preflight_enrol(live_identity_count)?;
        }
        // The SPI probe may have deferred until its SMC power supplier bound.
        // A saved attach-time snapshot would incorrectly reject later ioctls.
        let sensor_present = sensor::is_bound();
        let handled = if cmd == bio::IOC_DELETE_ORPHAN {
            self.bio_delete_orphan(arg)?
        } else {
            let mut session = self.bio_session.lock();
            if bio::starts_capture(cmd) {
                self.wait_for_worker_exit(&mut session, false)?;
            }
            let index = self.bio_index.lock();
            let mut ctx = bio::Context {
                session: &mut session,
                index: &index,
                sensor_present,
                live_identity_count,
            };
            bio::ioctl(&mut ctx, cmd, arg)?
        };

        if handled.delete_identity.is_some() || !handled.delete_identities.is_empty() {
            // A deletion is made durable by saving the Catacombs, and with
            // xART writes off that save would be refused. Refuse before 0x57
            // rather than report a deletion that the next boot undoes.
            if !self.xart_writable() {
                dev_warn_ratelimited!(
                    self.dev,
                    "Touch ID: deleting a fingerprint needs xart_writes=1; with writes off the deletion would not survive a reboot\n"
                );
                return Err(EROFS);
            }
            // 0x57 and the saves are sent with the session lock released
            // (they wait on mailbox replies). The system must not sleep
            // partway through the saves, and a verify that finishes after a
            // cancel saves too.
            let deleted = loop {
                self.wait_until_awake()?;
                let _writer = self.catacomb_writer.lock();
                // A suspend began while this waited for the lock: wait it out.
                if !self.capture_begin() {
                    continue;
                }
                let deleted = self.delete_and_persist(&handled, cmd == bio::IOC_DELETE_ORPHAN);
                self.capture_end();
                break deleted;
            };
            deleted?;
        }

        if handled.wake {
            self.bio_wake();
        }
        Ok(handled)
    }

    fn bio_attest(&self, arg: usize) -> Result<bio::Handled> {
        let user = kernel::uaccess::UserPtr::from_addr(arg);
        let req: bio::Attest =
            kernel::uaccess::UserSlice::new(user, core::mem::size_of::<bio::Attest>())
                .reader()
                .read()?;
        let (sig, pubk) = self.refkey_attest_sign(&req.challenge)?;
        if sig.len() > bio::ATTEST_SIG_MAX || pubk.len() != bio::ATTEST_PUB_LEN {
            return Err(EIO);
        }
        let mut out = bio::Attest {
            sig_len: sig.len() as u32,
            challenge: req.challenge,
            public_key: [0u8; bio::ATTEST_PUB_LEN],
            signature: [0u8; bio::ATTEST_SIG_MAX],
            reserved: [0u8; 3],
        };
        out.public_key.copy_from_slice(&pubk);
        out.signature[..sig.len()].copy_from_slice(&sig);
        kernel::uaccess::UserSlice::new(user, core::mem::size_of::<bio::Attest>())
            .writer()
            .write(&out)?;
        Ok(bio::Handled {
            ret: 0,
            wake: false,
            start_enrol: false,
            start_verify: false,
            delete_identity: None,
            delete_identities: KVec::new(),
        })
    }

    /// Unlocks the designated user's key bag with the stored secret, as
    /// macOS unlocks the user's bag with the passcode. The enclave ends a
    /// biometric lockout when the bag is unlocked with its secret, not
    /// when a match credential is set up.
    fn unlock_special_bag(&self) -> bool {
        if !self.keybag_designated.load(Relaxed) {
            return false;
        }
        let Some(user) = crate::sks::DesignateUser::new(SBIO_PROBE_USER_ID) else {
            return false;
        };
        let special = user.special_handle();
        let Ok(keybag::State::Present(stored)) = keybag::read(keybag::Slot::Identity) else {
            return false;
        };
        let label = match self.profile.key_store {
            profile::KeyStore::Sepos13 { .. } => c"DEVICE_STATE_TRANSITION",
            profile::KeyStore::Variant5 => crate::sks::SKS_LOCK_STATE_NAME,
        };
        self.sks_step(label, |healthy| {
            self.sks_req_unlock_special(special, stored.secret(), healthy)
        })
        .is_some()
    }

    fn ensure_restored_after(&self, prepared: bool) {
        // Probe credential ordering without assuming it fixes cold restore.
        // J414s still answered 0x8002 and listed zero identities with this
        // credential established, so the identity proof below remains required.
        let unlocked = self.unlock_special_bag();
        let match_context = crate::sbio::UserId::new(SBIO_PROBE_USER_ID)
            .map(|user| self.establish_scrd_match_context(user))
            .unwrap_or(false);
        dev_info_ratelimited!(self.dev, "sbio: pre-restore special unlock {}, SCRD match context {}\n", unlocked, match_context);

        let restored = self.restore_all_components();
        dev_info_ratelimited!(self.dev, "sbio: cold-match preparation {}, component restore {}\n", prepared, restored);
        // Restore can replace the credential and the device's identity view.
        // Refresh both after loading J700's persistent components, before
        // admitting a verify operation.
        let match_context = if self.profile.nonce_session {
            let synced = restored && self.sync_device_view();
            self.device_view_synced.store(synced, Relaxed);
            synced && crate::sbio::UserId::new(SBIO_PROBE_USER_ID)
                .is_some_and(|user| self.establish_scrd_match_context(user))
        } else {
            match_context
        };
        let identities = self.log_identity_count(c"after Catacomb restore with sensor registered");
        let identity_ready = matches!(identities, Some(n) if n > 0);
        let candidate = prepared && match_context && restored && identity_ready;
        self.cold_restore_candidate.store(candidate, Relaxed);
        // COMPLETE_INIT has not synchronized the device view yet. Never make
        // verification available on the preliminary Catacomb proof alone;
        // complete_bringup finishes the proof and sets templates_restored.
        self.templates_restored.store(false, Relaxed);
        // The nonce path synced the view before COMPLETE_INIT to refresh SCRD.
        // Re-arm the final proof and its bounded retry on every cold restore.
        if self.profile.nonce_session {
            self.device_view_synced.store(false, Relaxed);
        }
        if !candidate {
            dev_err_ratelimited!(
                self.dev,
                "matching unavailable this boot: cold-match preparation, SCRD credential, Catacomb restore, or identity proof did not complete; template state is unproven\n"
            );
            return;
        }
        dev_info_ratelimited!(self.dev, "sbio: cold restore candidate ready; waiting for device-view proof\n");

    }

    fn cold_match_continue(
        &self,
        source: crate::sks::KeyBagHandle,
        uuid: [u8; keybag::UUID_LEN],
        provenance: keybag::UuidProvenance,
    ) -> bool {
        if !self.keybag_designated.load(Relaxed) {
            return false;
        }

        let Some(user) = crate::sks::DesignateUser::new(SBIO_PROBE_USER_ID) else {
            return false;
        };
        let special = user.special_handle();
        self.log_identity_count(c"before the cold-match preparation");

        let Some(source_uuid) = self.sks_read_uuid(source) else {
            return false;
        };
        let generated_lookup = matches!(self.profile.key_store, profile::KeyStore::Sepos13 { .. })
            && provenance == keybag::UuidProvenance::AsGenerated;
        let Some(identity) = keybag::SnapshotIdentity::from_loaded(
            uuid, provenance, source_uuid, generated_lookup,
        ) else {
            dev_err_ratelimited!(self.dev, "sks: recovered source bag does not match its stored identity\n");
            return false;
        };
        let uuid_ok = self
            .sks_send(self.sks_req_copy_uuid_special(special))
            .and_then(|out| self.sks_uuid_from_reply(&out));
        if !uuid_ok.is_some_and(|got| identity.accepts(&uuid, provenance, &got)) {
            dev_err_ratelimited!(self.dev, "sks: designated bag does not match the recovered source bag\n");
            return false;
        }

        if self.sks_send(self.sks_req_unload_keybag(source)).is_none() {
            return false;
        }

        let mut material = self.enrol_material.lock();
        let Some(material) = &mut *material else {
            return false;
        };
        material.snapshot_identity = Some(identity);
        dev_info_ratelimited!(self.dev, "sks: snapshot identity bound to recovered and designated bag; generated lookup {}\n", generated_lookup);

        true
    }

    pub(crate) fn sep_random(&self, buf: &mut [u8]) -> Result<()> {
        // The trusted-key framework asks for a fresh plaintext key before
        // passing it to SEP for sealing. The host CSPRNG is appropriate for
        // that key; J313/13.5's unverified SEP control entropy replies have
        // returned all zeros and must not be used as key material.
        shim::random_bytes(buf)?;
        if !buf.is_empty() && buf.iter().all(|b| *b == 0) {
            return Err(EIO);
        }
        Ok(())
    }

    pub(crate) fn wrapped_from_copy_reply(
        &self,
        out: &SksOutcome,
        _from: &CStr,
    ) -> Option<KVec<u8>> {
        let body = self.sks_report_response(c"COPY_KEYBAG", out)?;
        if out.reply.status != 0 || image::operation_status(body).unwrap_or(-1) != 0 {
            return None;
        }
        let (blob, _) = image::read_blob(body, 4)?;
        if blob.is_empty() {
            return None;
        }
        let mut copy = KVec::new();
        if copy.extend_from_slice(blob, GFP_KERNEL).is_err() {
            return None;
        }
        Some(copy)
    }

    fn prove_restore(&self) -> bool {
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return false;
        };
        let op = crate::sbio::sbio_free_identity_count(user);
        let Some(reply) = self.sbio_expect_ok(&op) else {
            return false;
        };
        if reply.len() != crate::sbio::SBIO_FREE_COUNT_REPLY_LEN {
            return false;
        }
        let count = u32::from_le_bytes([reply[0], reply[1], reply[2], reply[3]]);
        if count > crate::sbio::SBIO_FREE_COUNT_MAX {
            return false;
        }
        if count == crate::sbio::SBIO_FREE_COUNT_MAX {
            return true;
        }
        true
    }

    fn enclave_identity_count(&self) -> Option<usize> {
        let op = crate::sbio::sbio_list_identities();
        let SbioOutcome::Ok(reply) = self.sbio_call(&op) else {
            return None;
        };
        let records = crate::sbio::IdentityRecords::new(&reply)?;
        Some(records.count())
    }

    fn log_identity_count(&self, when: &CStr) -> Option<usize> {
        let Some(count) = self.enclave_identity_count() else {
            dev_warn_ratelimited!(self.dev, "sbio: identity list unavailable {}\n", when);
            return None;
        };
        dev_info_ratelimited!(self.dev, "sbio: identity count {} {}\n", count, when);
        Some(count)
    }

    fn enclave_lists(&self, uuid: &[u8; bio::UUID_LEN]) -> Option<bool> {
        let op = crate::sbio::sbio_list_identities();
        let SbioOutcome::Ok(reply) = self.sbio_call(&op) else {
            return None;
        };
        let records = crate::sbio::IdentityRecords::new(&reply)?;
        Some(records.lists_uuid(uuid))
    }

    /// Called with `catacomb_writer` held. A host index entry goes only once
    /// its deletion is saved: if the save fails, the entry stays, userspace
    /// still lists the fingerprint, and a retried deletion saves again.
    fn delete_and_persist(&self, handled: &bio::Handled, recovery: bool) -> Result<()> {
        let retry = self.deletion_unsaved.load(Relaxed);
        let wanted =
            Iterator::chain(handled.delete_identity.iter(), handled.delete_identities.iter());
        // Allocated before any 0x57, so recording a removal cannot fail.
        let mut gone: KVec<[u8; bio::UUID_LEN]> =
            KVec::with_capacity(wanted.clone().count(), GFP_KERNEL)?;
        let mut delete_err: Option<Error> = None;
        let mut orphan_removed = false;
        for identity in wanted {
            let orphan = recovery || !self.bio_index.lock().contains_uuid(identity.uuid());
            match self.delete_identity(identity, orphan) {
                Ok(true) => {
                    let _ = gone.push_within_capacity(*identity.uuid());
                    orphan_removed |= orphan;
                }
                Ok(false) => {}
                Err(e) => delete_err = delete_err.or(Some(e)),
            }
        }
        // Save what was removed even if another deletion failed.
        if !gone.is_empty() {
            match self.persist_deletion(retry, orphan_removed) {
                Ok(()) => {
                    self.deletion_unsaved.store(false, Relaxed);
                    for uuid in gone.iter() {
                        self.forget_identity(uuid, c"removed from the enclave");
                    }
                }
                Err(e) => {
                    dev_err_ratelimited!(
                        self.dev,
                        "bio: removed from the enclave but not saved; the fingerprint stays listed, returns at the next boot, and deleting it again retries the save\n"
                    );
                    self.deletion_unsaved.store(true, Relaxed);
                    delete_err = delete_err.or(Some(e));
                }
            }
        }
        match delete_err {
            Some(e) => Err(e),
            None => Ok(()),
        }
    }

    /// Returns whether the enclave no longer holds the identity and the host
    /// entry must wait for [`Self::persist_deletion`].
    fn delete_identity(&self, identity: &crate::sbio::IdentityV1, orphan: bool) -> Result<bool> {
        if self.enclave_lists(identity.uuid()) == Some(false) {
            // Unless an earlier deletion's save failed, which leaves the
            // enclave without an identity the saved Catacomb still holds.
            if self.deletion_unsaved.load(Relaxed) {
                return Ok(true);
            }
            self.forget_identity(identity.uuid(), c"a host entry the enclave never had");
            return Ok(false);
        }

        // 0x57 cannot be undone. Ensure the host can save the affected user
        // and master Catacombs before sending it. For an explicit recovery the
        // owner Catacomb provides the recovery path even when the bag snapshot
        // has no binding; export that owner first if this install lacks a file.
        for kind in [PRIVATE_TYPE_CATACOMB_USER, PRIVATE_TYPE_CATACOMB_MASTER] {
            if !crate::catacomb::is_saved(kind) || !crate::catacomb::writable(kind) {
                dev_err_ratelimited!(
                    self.dev,
                    "bio: not deleting: user/master Catacomb is unavailable or unwritable\n"
                );
                return Err(EIO);
            }
        }
        if orphan && crate::catacomb::is_absent(PRIVATE_TYPE_CATACOMB_OWNER) {
            let Some(reply) = self.read_component_states() else { return Err(EIO); };
            let state = crate::sbio::ComponentStates::new(&reply)
                .state_for(crate::sbio::CatacombUser::OWNER.value());
            if !state.is_some_and(|s| s & crate::sbio::COMPONENT_STATE_ACTIVE != 0)
                || !self.save_catacomb(crate::sbio::CatacombUser::OWNER,
                                      PRIVATE_TYPE_CATACOMB_OWNER,
                                      c"orphan-recovery owner Catacomb") {
                dev_err_ratelimited!(self.dev, "bio: not deleting: owner Catacomb export failed\n");
                return Err(EIO);
            }
        }
        if !crate::catacomb::is_saved(PRIVATE_TYPE_CATACOMB_OWNER)
            || !crate::catacomb::writable(PRIVATE_TYPE_CATACOMB_OWNER)
        {
            return Err(EIO);
        }
        if !orphan && !self.enrol_material.lock().as_ref()
            .is_some_and(|material| material.snapshot_identity.is_some()) {
            dev_err_ratelimited!(self.dev, "bio: not deleting: indexed identity has no identity-bag binding\n");
            return Err(EIO);
        }
        if orphan && !matches!(keybag::read(keybag::Slot::Identity),
                               Ok(keybag::State::Present(_))) {
            dev_err_ratelimited!(self.dev, "bio: not deleting: Linux identity bag file is unavailable\n");
            return Err(EIO);
        }

        let op = crate::sbio::sbio_delete_identity(identity);

        if self.sbio_expect_ok(&op).is_none() {
            dev_err_ratelimited!(
                self.dev,
                "bio: 0x57 failed, status above; host index left unchanged (forgetting a template the enclave still holds would report a deletion that did not happen)\n"
            );
            return Err(EIO);
        }

        Ok(true)
    }

    /// 0x57 removes an identity from the enclave's memory only. Until the user
    /// Catacomb is saved, the next attach restores the saved one and the
    /// deleted fingerprint with it. Saved like a match: only the components
    /// the enclave marks save-pending (a SAVE_CATACOMB of one that is not has
    /// been refused), the user before the master because saving the user can
    /// change the master, state re-read before each, then the lockout and the
    /// identity bag when a verified binding exists. An exact-ID orphan
    /// recovery without that binding saves the Linux owner Catacomb instead.
    /// Safe to repeat after a failure partway: a `retry` accepts a user
    /// Catacomb an earlier attempt already saved.
    fn persist_deletion(&self, retry: bool, orphan: bool) -> Result<()> {
        let Some(user) = crate::sbio::UserId::new(SBIO_PROBE_USER_ID) else {
            return Err(EIO);
        };
        for (who, kind, what) in [
            (
                crate::sbio::CatacombUser::enrolling(user),
                PRIVATE_TYPE_CATACOMB_USER,
                c"user Catacomb after deletion",
            ),
            (
                crate::sbio::CatacombUser::MASTER,
                PRIVATE_TYPE_CATACOMB_MASTER,
                c"master Catacomb after deletion",
            ),
            (
                crate::sbio::CatacombUser::OWNER,
                PRIVATE_TYPE_CATACOMB_OWNER,
                c"owner Catacomb after deletion",
            ),
        ] {
            let Some(reply) = self.read_component_states() else {
                dev_err_ratelimited!(self.dev, "bio: cannot read component state before saving the {}\n", what);
                return Err(EIO);
            };
            let state = crate::sbio::ComponentStates::new(&reply).state_for(who.value());
            dev_info_ratelimited!(self.dev, "bio: {} state {:?}\n", what, state);
            let Some(state) = state else {
                dev_err_ratelimited!(self.dev, "bio: the {} is absent; the deletion cannot be saved\n", what);
                return Err(EIO);
            };
            if state & crate::sbio::COMPONENT_STATE_ACTIVE == 0 {
                dev_err_ratelimited!(self.dev, "bio: the {} is not active; the deletion cannot be saved\n", what);
                return Err(EIO);
            }
            if state & crate::sbio::COMPONENT_STATE_SAVE_PENDING != 0 ||
                (orphan && who.value() == crate::sbio::CatacombUser::OWNER.value()) {
                if !self.save_catacomb(who, kind, what) {
                    dev_err_ratelimited!(self.dev, "bio: saving the {} failed\n", what);
                    return Err(EIO);
                }
            } else if who.value() == user.value() && !retry {
                dev_err_ratelimited!(
                    self.dev,
                    "bio: the enclave did not mark the user Catacomb for saving after 0x57; the deletion cannot be saved\n"
                );
                return Err(EIO);
            }
            if !crate::catacomb::is_saved(kind) {
                dev_err_ratelimited!(self.dev, "bio: the {} has no durable host file\n", what);
                return Err(EIO);
            }
        }
        if !self.save_lockout() {
            dev_err_ratelimited!(self.dev, "bio: lockout persistence failed after the deletion\n");
            return Err(EIO);
        }
        let bag_bound = self.enrol_material.lock().as_ref()
            .is_some_and(|material| material.snapshot_identity.is_some());
        if bag_bound {
            if !self.resnapshot_identity_keybag() {
                dev_err_ratelimited!(
                    self.dev,
                    "bio: Catacombs saved after the deletion but the identity-bag snapshot failed\n"
                );
                return Err(EIO);
            }
        } else if orphan {
            dev_info_ratelimited!(
                self.dev,
                "bio: orphan deletion saved via Linux user/master/owner Catacombs without a bag snapshot binding\n"
            );
        } else {
            return Err(EIO);
        }
        Ok(())
    }

    fn forget_identity(&self, uuid: &[u8; bio::UUID_LEN], why: &CStr) {
        let persisted = {
            let mut index = self.bio_index.lock();
            index.remove(uuid);
            self.with_host_store(|store| index.persist(store))
        };
        match persisted {
            Some(Ok(())) => {},
            _ => dev_warn_ratelimited!(
                self.dev,
                "bio: {} ({}) but the host index could not be written; correct in memory this boot, rebuilt at next attach\n",
                Hex(uuid),
                why
            ),
        }
    }

    pub(crate) fn bio_ready(&self) -> bool {
        self.bio_session.lock().ready()
    }
}

/// Which bring-up a round of attempts makes.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Bringup {
    /// As Touch ID is activated.
    Attach,
    /// The late round after an attach that fell short.
    Late,
    /// Before a verify.
    Verify,
}

/// A sensor bring-up with retries, as [`registration::bring_up`] drives it.
struct BringupAttempts<'a, W: Fn() -> bool> {
    sep: &'a SepData,
    purpose: &'a CStr,
    wanted: W,
    began_ns: u64,
    /// Each attempt is the attach-time one rather than a verify's.
    attach: bool,
    /// How many failed attempts the next one follows, in this round or the
    /// one before it; [`registration::retry_stage`] restarts from it.
    retry: u32,
}

impl<W: Fn() -> bool> registration::Attempts for BringupAttempts<'_, W> {
    fn attempt(&mut self) -> bool {
        if self.retry > 0 {
            let stage = registration::retry_stage(
                self.sep.bringup.load(Relaxed),
                self.sep.profile.nonce_session,
                self.retry,
            );
            self.sep.bringup.store(stage, Relaxed);
        }
        self.retry += 1;
        if self.attach {
            self.sep.attach_bring_sensor_online()
        } else {
            self.sep.bring_sensor_online()
        }
    }

    fn wanted(&self) -> bool {
        (self.wanted)()
    }

    fn elapsed_ms(&self) -> u64 {
        shim::boottime_ns().saturating_sub(self.began_ns) / 1_000_000
    }

    fn failed(&mut self, attempt: u32, of: u32, pause_ms: u32) {
        if pause_ms == 0 {
            dev_warn_ratelimited!(
                self.sep.dev,
                "sensor: {} bring-up attempt {} of {} failed; starting again\n",
                self.purpose,
                attempt,
                of
            );
        } else {
            dev_warn_ratelimited!(
                self.sep.dev,
                "sensor: {} bring-up attempt {} of {} failed; starting again in {} ms\n",
                self.purpose,
                attempt,
                of,
                pause_ms
            );
        }
    }

    // In short steps, so that a suspend or an unbind is not kept waiting.
    fn pause(&mut self, ms: u32) -> bool {
        let mut left = ms;
        while left > 0 {
            if !(self.wanted)() {
                return false;
            }
            let step = left.min(BRINGUP_PAUSE_STEP_MS);
            kernel::time::delay::fsleep(kernel::time::Delta::from_millis(i64::from(step)));
            left -= step;
        }
        (self.wanted)()
    }
}

/// The requests of a nonce-session authentication, as
/// [`registration::authenticate`] orders them. Every failure is logged where
/// it happens, except a 0x16 refusal of REGISTER_SERIAL, which may be the
/// enclave pointing at a registration it already holds.
struct SensorAuthentication<'a> {
    sep: &'a SepData,
    refused16: bool,
}

impl registration::Authentication for SensorAuthentication<'_> {
    type Serial = sensor::SensorSerial;
    type Nonce = [u8; 32];

    fn read_serial(&mut self) -> Option<sensor::SensorSerial> {
        self.sep.read_serial_logged()
    }

    fn read_nonce(&mut self) -> Option<[u8; 32]> {
        match sensor::session_nonce() {
            Ok(nonce) => {
                self.sep.sensor_trace(c"nonce read");
                Some(nonce)
            }
            Err(error) => {
                dev_err_ratelimited!(self.sep.dev, "sensor: nonce read failed: {:?}\n", error);
                self.sep.sensor_step_failed(c"nonce read");
                None
            }
        }
    }

    fn register_serial(&mut self, serial: &sensor::SensorSerial) -> registration::SerialAnswer {
        let op = crate::sbio::sbio_register_sensor_serial(serial);
        match self.sep.sbio_transfer(&op) {
            Ok(done) if done.status.is_ok() => {
                self.sep.sensor_trace(op.name());
                registration::SerialAnswer::Accepted
            }
            Ok(done)
                if matches!(
                    done.status.answered(),
                    Some(err) if err as u16 == crate::sbio::SBIO_STATUS_16
                ) =>
            {
                self.refused16 = true;
                registration::SerialAnswer::Refused16
            }
            Ok(done) => {
                dev_warn_ratelimited!(
                    self.sep.dev,
                    "sensor: {} refused with status {}\n",
                    op.name(),
                    done.status
                );
                self.sep.sensor_step_failed(op.name());
                registration::SerialAnswer::Failed
            }
            Err(e) => {
                dev_warn_ratelimited!(self.sep.dev, "sensor: {} transfer failed: {:?}\n", op.name(), e);
                self.sep.sensor_step_failed(op.name());
                registration::SerialAnswer::Failed
            }
        }
    }

    fn open_session(&mut self, nonce: &[u8; 32]) -> bool {
        self.sep.sensor_nonce_session(nonce)
    }
}

/// The SBIO reply header, as [`transfer::take_header`] samples it.
struct SbioHeader<'a>(&'a SepData);

impl transfer::HeaderSlot for SbioHeader<'_> {
    fn sample(&self, header: &mut [u8; transfer::HEADER_LEN]) -> bool {
        self.0.ool_peek(&self.0.ool_sbio, 0, header)
    }

    fn repoison(&self) {
        self.0
            .ool_repoison(&self.0.ool_sbio, 0, transfer::HEADER_LEN);
    }

    fn pause(&self) {
        let poll = kernel::time::Delta::from_millis(i64::from(OOL_WRITE_POLL_MS));
        kernel::time::delay::fsleep(poll);
    }
}

pub(crate) struct SbioOp {
    opcode: u16,
    payload: [u8; SBIO_MAX_PAYLOAD],
    payload_len: usize,
    name: &'static CStr,
}

/// The largest request: an image init with its credential-context blob.
const SBIO_MAX_PAYLOAD: usize = SBIO_IMAGE_INIT_LEN + crate::scrd::SCRD_ACM_HANDLE_LEN;

/// The image init without a credential context.
pub(crate) const SBIO_IMAGE_INIT_LEN: usize = 0x97;
static_assert!(SBIO_IMAGE_INIT_LEN <= SBIO_MAX_PAYLOAD);

const fn pad_payload(src: &[u8]) -> [u8; SBIO_MAX_PAYLOAD] {
    let mut out = [0u8; SBIO_MAX_PAYLOAD];
    let mut i = 0;
    while i < src.len() {
        out[i] = src[i];
        i += 1;
    }
    out
}

impl SbioOp {
    pub(crate) fn opcode(&self) -> u16 {
        self.opcode
    }
    pub(crate) fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }
    pub(crate) fn name(&self) -> &'static CStr {
        self.name
    }
}

pub(crate) const SBIO_PROTOCOL_GENERATION: u32 = 1;

const OP_SBIO_REGISTER_SENSOR: u16 = 0x80;
pub(crate) fn sbio_register_sensor(id: &crate::sensor::Identifier) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_REGISTER_SENSOR,
        payload: pad_payload(&id.value().to_le_bytes()),
        payload_len: 2,
        name: c"REGISTER_SENSOR",
    }
}

pub(crate) const SBIO_SIGNAL_QUALITY: u32 = 0;

const OP_SBIO_COVERAGE_PARAMS: u16 = 0x5d;
pub(crate) fn sbio_coverage_params() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_COVERAGE_PARAMS,
        payload: pad_payload(&SBIO_SIGNAL_QUALITY.to_le_bytes()),
        payload_len: 4,
        name: c"COVERAGE_PARAMS",
    }
}

const OP_SBIO_OPERATION_PARAMS: u16 = 0x5c;

pub(crate) const SBIO_OPERATION_PARAMS_LEN: usize = match SBIO_PROTOCOL_GENERATION {
    1 => 4,
    6 => 8,
    _ => 0,
};
static_assert!(SBIO_OPERATION_PARAMS_LEN != 0);
static_assert!(SBIO_OPERATION_PARAMS_LEN <= SBIO_MAX_PAYLOAD);

pub(crate) fn sbio_operation_params() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_OPERATION_PARAMS,
        payload: pad_payload(&[]),
        payload_len: SBIO_OPERATION_PARAMS_LEN,
        name: c"OPERATION_PARAMS",
    }
}

const OP_SBIO_TRANSPARENT_CHANNEL: u16 = 0x6a;
pub(crate) fn sbio_transparent_channel() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_TRANSPARENT_CHANNEL,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"TRANSPARENT_CHANNEL",
    }
}

const OP_SBIO_COMPLETE_INIT: u16 = 0x01;
pub(crate) fn sbio_complete_init(
    patch: crate::PatchLoaded,
    params: crate::ParametersApplied,
) -> SbioOp {
    let (_, _) = (patch, params);
    let mut body = [0u8; 8];
    body[0..4].copy_from_slice(&1u32.to_le_bytes());
    body[4..8].copy_from_slice(&1u32.to_le_bytes());
    SbioOp {
        opcode: OP_SBIO_COMPLETE_INIT,
        payload: pad_payload(&body),
        payload_len: 8,
        name: c"COMPLETE_INIT",
    }
}

const CONTEXT_SCOPE_SYSTEM: i32 = -1;
static_assert!(CONTEXT_SCOPE_SYSTEM != 0);
static_assert!(CONTEXT_SCOPE_SYSTEM < 0);

#[derive(Clone, Copy)]
pub(crate) struct ContextScope(i32);

impl ContextScope {
    pub(crate) const SYSTEM: ContextScope = ContextScope(CONTEXT_SCOPE_SYSTEM);

    pub(crate) fn user(id: UserId) -> ContextScope {
        ContextScope(id.value())
    }

    pub(crate) fn value(&self) -> i32 {
        self.0
    }
}

const OP_SBIO_CONTEXT_STATE: u16 = 0x6b;
pub(crate) fn sbio_context_state() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CONTEXT_STATE,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"CONTEXT_STATE",
    }
}

// 0x2d destroys the catacomb; only send with zero existing identities
pub(crate) struct NoExistingCatacomb {
    _seal: (),
}

impl NoExistingCatacomb {
    pub(crate) fn from_zero_identities(count: usize) -> Option<NoExistingCatacomb> {
        match count {
            0 => Some(NoExistingCatacomb { _seal: () }),
            _ => None,
        }
    }
}

const OP_SBIO_SELECT_CONTEXT: u16 = 0x2d;
pub(crate) fn sbio_select_context(scope: ContextScope, proof: &NoExistingCatacomb) -> SbioOp {
    let NoExistingCatacomb { _seal: () } = proof;
    SbioOp {
        opcode: OP_SBIO_SELECT_CONTEXT,
        payload: pad_payload(&scope.value().to_le_bytes()),
        payload_len: 4,
        name: c"SELECT_CONTEXT",
    }
}

pub(crate) const SBIO_PROTECTED_CONFIG_LEN: usize = 32;

/// The PROTECTED_CONFIG answer: eight signed words, the set unlock,
/// identification, login and payment settings, then the effective ones.
/// Each is 0 (disabled), 1 (enabled) or -1 (unspecified).
pub(crate) struct ProtectedConfig {
    pub(crate) unlock_set: i32,
    pub(crate) unlock_effective: i32,
}

const PC_UNLOCK_SET: usize = 0x00;
const PC_UNLOCK_EFFECTIVE: usize = 0x10;
static_assert!(PC_UNLOCK_EFFECTIVE + 4 <= SBIO_PROTECTED_CONFIG_LEN);

impl ProtectedConfig {
    pub(crate) fn parse(raw: &[u8]) -> Option<ProtectedConfig> {
        if raw.len() != SBIO_PROTECTED_CONFIG_LEN {
            return None;
        }
        let word = |at: usize| i32::from_le_bytes([raw[at], raw[at + 1], raw[at + 2], raw[at + 3]]);
        Some(ProtectedConfig {
            unlock_set: word(PC_UNLOCK_SET),
            unlock_effective: word(PC_UNLOCK_EFFECTIVE),
        })
    }

    pub(crate) fn describe(value: i32) -> &'static str {
        match value {
            0 => "disabled",
            1 => "enabled",
            -1 => "unspecified",
            _ => "unknown",
        }
    }

    /// The macOS admission rule for an unlock match.
    pub(crate) fn allows_unlock(&self) -> bool {
        self.unlock_set == 1 && self.unlock_effective != -1
    }
}

const OP_SBIO_PROTECTED_CONFIG: u16 = 0x2b;
pub(crate) fn sbio_protected_config(id: UserId) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_PROTECTED_CONFIG,
        payload: pad_payload(&id.value().to_le_bytes()),
        payload_len: 4,
        name: c"PROTECTED_CONFIG",
    }
}

/// The 16-byte id of `id`'s Touch ID catacomb (the enrolled-template set),
/// as a key access-control list names it. Read only.
const OP_SBIO_CATACOMB_ID: u16 = 0x33;
pub(crate) const SBIO_CATACOMB_ID_LEN: usize = 16;
pub(crate) fn sbio_catacomb_id(id: UserId) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CATACOMB_ID,
        payload: pad_payload(&id.value().to_le_bytes()),
        payload_len: 4,
        name: c"CATACOMB_ID",
    }
}

/// The 32-byte hash of `id`'s enrolled set, which changes when a finger is
/// added or removed. Read only. The reply may lead with one version byte.
const OP_SBIO_CATACOMB_HASH: u16 = 0x36;
pub(crate) const SBIO_CATACOMB_HASH_LEN: usize = 32;
pub(crate) fn sbio_catacomb_hash(id: UserId) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CATACOMB_HASH,
        payload: pad_payload(&id.value().to_le_bytes()),
        payload_len: 4,
        name: c"CATACOMB_HASH",
    }
}

/// The 32-byte hash in a CATACOMB_HASH reply: the bare hash, or one leading
/// version byte and the hash.
pub(crate) fn catacomb_hash_from_reply(reply: &[u8]) -> Option<[u8; SBIO_CATACOMB_HASH_LEN]> {
    let raw = match reply.len() {
        SBIO_CATACOMB_HASH_LEN => reply,
        n if n == SBIO_CATACOMB_HASH_LEN + 1 => &reply[1..],
        _ => return None,
    };
    let mut out = [0u8; SBIO_CATACOMB_HASH_LEN];
    out.copy_from_slice(raw);
    Some(out)
}

const OP_SBIO_BEGIN_ENROL: u16 = 0x03;

pub(crate) const BE_AUTH_TYPE_ACM_CONTEXT: u32 = 0;

pub(crate) const SBIO_BEGIN_ENROL_LEN: usize = 0x44;
static_assert!(SBIO_BEGIN_ENROL_LEN <= SBIO_MAX_PAYLOAD);

const BE_FLAGS: usize = 0;
const BE_USER_ID: usize = 4;
const BE_AUTH_TYPE: usize = 8;
const BE_TOKEN_LEN: usize = 12;
const BE_TOKEN: usize = 16;
static_assert!(BE_TOKEN + SKS_AUTH_TOKEN_LEN <= SBIO_BEGIN_ENROL_LEN);

pub(crate) fn sbio_begin_enrol(
    user: UserId,
    auth_type: u32,
    token: &[u8; SKS_AUTH_TOKEN_LEN],
) -> SbioOp {
    let mut body = [0u8; SBIO_BEGIN_ENROL_LEN];
    body[BE_FLAGS..BE_FLAGS + 4].copy_from_slice(&0u32.to_le_bytes());
    body[BE_USER_ID..BE_USER_ID + 4].copy_from_slice(&user.value().to_le_bytes());
    body[BE_AUTH_TYPE..BE_AUTH_TYPE + 4].copy_from_slice(&auth_type.to_le_bytes());
    body[BE_TOKEN_LEN..BE_TOKEN_LEN + 4]
        .copy_from_slice(&(SKS_AUTH_TOKEN_LEN as u32).to_le_bytes());
    body[BE_TOKEN..BE_TOKEN + SKS_AUTH_TOKEN_LEN].copy_from_slice(token);
    SbioOp {
        opcode: OP_SBIO_BEGIN_ENROL,
        payload: pad_payload(&body),
        payload_len: SBIO_BEGIN_ENROL_LEN,
        name: c"BEGIN_ENROL",
    }
}

const OP_SBIO_MODULE_CHALLENGE: u16 = 0x31;
pub(crate) fn sbio_module_challenge() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_MODULE_CHALLENGE,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"MODULE_CHALLENGE",
    }
}

pub(crate) const SBIO_MODULE_CHALLENGE_LEN: usize = 0x40;

const OP_SBIO_MODULE_COMMIT: u16 = 0x32;
pub(crate) fn sbio_module_commit(reply: &[u8]) -> Option<SbioOp> {
    if reply.is_empty() || reply.len() > SBIO_MAX_PAYLOAD {
        return None;
    }
    Some(SbioOp {
        opcode: OP_SBIO_MODULE_COMMIT,
        payload: pad_payload(reply),
        payload_len: reply.len(),
        name: c"MODULE_COMMIT",
    })
}

const OP_SBIO_LOAD_CALIBRATION: u16 = 0x5b;

pub(crate) const SBIO_CALIBRATION_SOURCE: u32 = 3;

pub(crate) struct SbioCalibration(KVec<u8>);

impl SbioCalibration {
    pub(crate) fn new(blob: &crate::CalibrationBlob, source: u32) -> Result<SbioCalibration> {
        let bytes = blob.bytes();
        let mut body = KVec::new();
        body.extend_from_slice(&source.to_le_bytes(), GFP_KERNEL)?;
        body.extend_from_slice(bytes, GFP_KERNEL)?;
        Ok(SbioCalibration(body))
    }

    pub(crate) fn opcode(&self) -> u16 {
        OP_SBIO_LOAD_CALIBRATION
    }

    pub(crate) fn payload(&self) -> &[u8] {
        &self.0
    }

    pub(crate) fn name(&self) -> &'static CStr {
        c"LOAD_CALIBRATION"
    }
}

const OP_SBIO_UPDATE_DEVICE_LIST: u16 = 0x7b;
pub(crate) fn sbio_update_device_list() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_UPDATE_DEVICE_LIST,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"UPDATE_DEVICE_LIST",
    }
}

/// Opens a match operation: the enclave answers the operation's device list,
/// a four-byte device count and one 24-byte record per device. Empty. Sent at
/// the start of every logical match, before its first image; the operation
/// ends with `MATCH_CANCEL`. Distinct from `UPDATE_DEVICE_LIST` (`0x7b`),
/// which only syncs the device view.
const OP_SBIO_OPERATION_DEVICE_LIST: u16 = 0x7d;
pub(crate) fn sbio_operation_device_list() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_OPERATION_DEVICE_LIST,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"OPERATION_DEVICE_LIST",
    }
}

const OP_DEVICE_LIST_HEADER_LEN: usize = 4;
const OP_DEVICE_RECORD_LEN: usize = 24;

/// The device count of an operation device list, if the reply is exactly a
/// count followed by that many records.
pub(crate) fn operation_device_count(reply: &[u8]) -> Option<u32> {
    let header = reply.get(..OP_DEVICE_LIST_HEADER_LEN)?;
    let count = u32::from_le_bytes([header[0], header[1], header[2], header[3]]);
    let records = (count as usize).checked_mul(OP_DEVICE_RECORD_LEN)?;
    (reply.len() == OP_DEVICE_LIST_HEADER_LEN.checked_add(records)?).then_some(count)
}

pub(crate) const SBIO_MATCH_POLICY_LEN: usize = 2;

const OP_SBIO_MATCH_POLICY: u16 = 0x1f;
pub(crate) fn sbio_match_policy() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_MATCH_POLICY,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"MATCH_POLICY",
    }
}

pub(crate) const SBIO_ENROL_RESULT_LEN: usize = 0xc98;

const ER_PROGRESS: usize = 0x004;
const ER_HAS_TEMPLATE: usize = 0x006;
const ER_COMPLETE: usize = 0xbfe;

static_assert!(ER_PROGRESS < SBIO_ENROL_RESULT_LEN);
static_assert!(ER_HAS_TEMPLATE + 4 <= SBIO_ENROL_RESULT_LEN);
static_assert!(ER_COMPLETE + 4 <= SBIO_ENROL_RESULT_LEN);
static_assert!(ER_COMPLETE > ER_HAS_TEMPLATE + 4);

pub(crate) const ER_IDENTITY_LEN: usize = 4 + IDENTITY_UUID_LEN;
static_assert!(ER_IDENTITY_LEN == 20);
static_assert!(ER_IDENTITY_LEN <= SBIO_ENROL_RESULT_LEN);

pub(crate) fn enrol_identity_candidates(
    bytes: &[u8],
    user_id: i32,
    mut visit: impl FnMut(usize, [u8; IDENTITY_UUID_LEN]),
) {
    if bytes.len() < ER_IDENTITY_LEN {
        return;
    }
    let wanted = user_id.to_le_bytes();
    for at in 0..=bytes.len() - ER_IDENTITY_LEN {
        if bytes[at..at + 4] != wanted {
            continue;
        }
        let mut uuid = [0u8; IDENTITY_UUID_LEN];
        uuid.copy_from_slice(&bytes[at + 4..at + ER_IDENTITY_LEN]);
        if uuid.iter().all(|&b| b == 0) {
            continue;
        }
        visit(at, uuid);
    }
}

pub(crate) struct EnrolmentResult {
    progress_raw: u8,
    has_template: u32,
    complete: u32,
}

impl EnrolmentResult {
    pub(crate) fn parse(bytes: &[u8]) -> Option<EnrolmentResult> {
        if bytes.len() < SBIO_ENROL_RESULT_LEN {
            return None;
        }
        Some(EnrolmentResult {
            progress_raw: bytes[ER_PROGRESS],
            has_template: u32::from_le_bytes([
                bytes[ER_HAS_TEMPLATE],
                bytes[ER_HAS_TEMPLATE + 1],
                bytes[ER_HAS_TEMPLATE + 2],
                bytes[ER_HAS_TEMPLATE + 3],
            ]),
            complete: u32::from_le_bytes([
                bytes[ER_COMPLETE],
                bytes[ER_COMPLETE + 1],
                bytes[ER_COMPLETE + 2],
                bytes[ER_COMPLETE + 3],
            ]),
        })
    }

    pub(crate) fn progress_percent(&self) -> u32 {
        (u32::from(self.progress_raw) * 100 + 127) / 255
    }

    pub(crate) fn has_template(&self) -> bool {
        self.has_template != 0
    }

    pub(crate) fn complete(&self) -> bool {
        self.complete != 0
    }
}

#[derive(Clone, Copy)]
pub(crate) struct CatacombUser(i32);

impl CatacombUser {
    pub(crate) const MASTER: CatacombUser = CatacombUser(-1);
    pub(crate) const OWNER: CatacombUser = CatacombUser(501);

    pub(crate) fn enrolling(user: UserId) -> CatacombUser {
        CatacombUser(user.value())
    }

    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}
static_assert!(CatacombUser::MASTER.value() == -1);
static_assert!(CatacombUser::OWNER.value() == 501);
static_assert!(CatacombUser::MASTER.value() != CatacombUser::OWNER.value());
static_assert!(CatacombUser::MASTER.value() < 0);

pub(crate) const SBIO_SAVED_USER_ID_AT: usize = 8;
static_assert!(SBIO_SAVED_USER_ID_AT + 4 <= SBIO_SAVED_MIN);

pub(crate) const SBIO_SAVE_SELECTOR_LEN: usize = 24;
const SS_USER_ID: usize = 0;
static_assert!(SBIO_SAVE_SELECTOR_LEN <= SBIO_MAX_PAYLOAD);

pub(crate) struct SaveSelector([u8; SBIO_SAVE_SELECTOR_LEN]);

impl SaveSelector {
    pub(crate) fn new(user: CatacombUser) -> SaveSelector {
        let mut out = [0u8; SBIO_SAVE_SELECTOR_LEN];
        out[SS_USER_ID..SS_USER_ID + 4].copy_from_slice(&user.value().to_le_bytes());
        SaveSelector(out)
    }

    pub(crate) fn bytes(&self) -> &[u8; SBIO_SAVE_SELECTOR_LEN] {
        &self.0
    }
}

pub(crate) const SBIO_STATUS_COLD_TRANSITION: u32 = 0x8002;
static_assert!(SBIO_STATUS_COLD_TRANSITION != 0);

// 0x101 from a 0x6d load = already active; a success answer, not an error
pub(crate) const SBIO_STATUS_ALREADY_ACTIVE: u32 = 0x101;
static_assert!(SBIO_STATUS_ALREADY_ACTIVE != 0);
static_assert!(SBIO_STATUS_ALREADY_ACTIVE != SBIO_STATUS_COLD_TRANSITION);

pub(crate) const COMPONENT_STATE_COLD: u32 = 0x1;
pub(crate) const COMPONENT_STATE_ACTIVE: u32 = 0x2;
pub(crate) const COMPONENT_STATE_SAVE_PENDING: u32 = 0x4;
static_assert!(COMPONENT_STATE_COLD != COMPONENT_STATE_ACTIVE);
static_assert!(COMPONENT_STATE_SAVE_PENDING & (COMPONENT_STATE_COLD | COMPONENT_STATE_ACTIVE) == 0);

pub(crate) const COMPONENT_PAIR_LEN: usize = 8;

pub(crate) struct ComponentStates<'a>(&'a [u8]);

impl<'a> ComponentStates<'a> {
    pub(crate) fn new(reply: &'a [u8]) -> ComponentStates<'a> {
        ComponentStates(reply)
    }

    pub(crate) fn count(&self) -> usize {
        self.0.len() / COMPONENT_PAIR_LEN
    }

    pub(crate) fn pair(&self, i: usize) -> Option<(i32, u32)> {
        let at = i.checked_mul(COMPONENT_PAIR_LEN)?;
        let bytes = self.0.get(at..at + COMPONENT_PAIR_LEN)?;
        Some((
            i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]),
            u32::from_le_bytes([bytes[4], bytes[5], bytes[6], bytes[7]]),
        ))
    }

    pub(crate) fn state_for(&self, id: i32) -> Option<u32> {
        (0..self.count())
            .filter_map(|i| self.pair(i))
            .find(|(pair_id, _)| *pair_id == id)
            .map(|(_, state)| state)
    }
}

pub(crate) enum ComponentAction {
    AlreadyActive,
    Load,
    Unsupported,
}

pub(crate) fn component_action(state: u32) -> ComponentAction {
    if state & COMPONENT_STATE_ACTIVE != 0 {
        ComponentAction::AlreadyActive
    } else if state & COMPONENT_STATE_COLD != 0 {
        ComponentAction::Load
    } else {
        ComponentAction::Unsupported
    }
}

const OP_SBIO_SAVE_LOCKOUT: u16 = 0x70;
pub(crate) fn sbio_save_lockout() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_SAVE_LOCKOUT,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"SAVE_LOCKOUT",
    }
}

pub(crate) struct SbioLoadLockout<'a>(&'a [u8]);

impl<'a> SbioLoadLockout<'a> {
    pub(crate) fn new(blob: &'a [u8]) -> Option<SbioLoadLockout<'a>> {
        if blob.is_empty() || blob.len() > SBIO_SAVED_MAX {
            return None;
        }
        Some(SbioLoadLockout(blob))
    }

    pub(crate) fn opcode(&self) -> u16 {
        OP_SBIO_LOAD_LOCKOUT
    }

    pub(crate) fn payload(&self) -> &[u8] {
        self.0
    }

    pub(crate) fn name(&self) -> &'static CStr {
        c"LOAD_LOCKOUT"
    }
}
const OP_SBIO_LOAD_LOCKOUT: u16 = 0x71;

const OP_SBIO_SAVE_CATACOMB: u16 = 0x6c;
pub(crate) fn sbio_save_catacomb(selector: &SaveSelector) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_SAVE_CATACOMB,
        payload: pad_payload(selector.bytes()),
        payload_len: SBIO_SAVE_SELECTOR_LEN,
        name: c"SAVE_CATACOMB",
    }
}

const OP_SBIO_CONFIRM_SAVE: u16 = 0x37;
pub(crate) fn sbio_confirm_save(selector: &SaveSelector) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CONFIRM_SAVE,
        payload: pad_payload(selector.bytes()),
        payload_len: SBIO_SAVE_SELECTOR_LEN,
        name: c"CONFIRM_SAVE",
    }
}

pub(crate) const SBIO_SAVED_MIN: usize = 0x22;
pub(crate) const SBIO_SAVED_MAX: usize = 0x4b000;
static_assert!(SBIO_SAVED_MIN < SBIO_SAVED_MAX);
static_assert!(SBIO_SAVED_MAX as u32 <= transfer::MAX_TRANSACTION);

pub(crate) struct SbioLoadCatacomb<'a>(&'a [u8]);

impl<'a> SbioLoadCatacomb<'a> {
    pub(crate) fn new(blob: &'a [u8]) -> Option<SbioLoadCatacomb<'a>> {
        if blob.len() < SBIO_SAVED_MIN || blob.len() > SBIO_SAVED_MAX {
            return None;
        }
        Some(SbioLoadCatacomb(blob))
    }

    pub(crate) fn opcode(&self) -> u16 {
        OP_SBIO_LOAD_CATACOMB
    }

    pub(crate) fn payload(&self) -> &[u8] {
        self.0
    }

    pub(crate) fn name(&self) -> &'static CStr {
        c"LOAD_CATACOMB"
    }
}
const OP_SBIO_LOAD_CATACOMB: u16 = 0x6d;

pub(crate) const IDENTITY_RECORD_LEN: usize = 40;
const IR_SELECTOR: usize = 20;
const IR_SELECTOR_BUILTIN: u32 = 1;
static_assert!(IR_SELECTOR == IDENTITY_V1_LEN);
static_assert!(IR_SELECTOR + 4 <= IDENTITY_RECORD_LEN);

const OP_SBIO_LIST_IDENTITIES: u16 = 0x7a;
pub(crate) fn sbio_list_identities() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_LIST_IDENTITIES,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"LIST_IDENTITIES",
    }
}

pub(crate) struct IdentityRecords<'a>(&'a [u8]);

impl<'a> IdentityRecords<'a> {
    pub(crate) fn new(reply: &'a [u8]) -> Option<IdentityRecords<'a>> {
        if reply.len() % IDENTITY_RECORD_LEN != 0 {
            return None;
        }
        Some(IdentityRecords(reply))
    }

    pub(crate) fn count(&self) -> usize {
        self.0.len() / IDENTITY_RECORD_LEN
    }

    pub(crate) fn record(&self, i: usize) -> Option<(IdentityV1, bool)> {
        let at = i.checked_mul(IDENTITY_RECORD_LEN)?;
        let bytes = self.0.get(at..at + IDENTITY_RECORD_LEN)?;
        let user_id = i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
        let mut uuid = [0u8; IDENTITY_UUID_LEN];
        uuid.copy_from_slice(&bytes[4..IDENTITY_V1_LEN]);
        let selector = u32::from_le_bytes([
            bytes[IR_SELECTOR],
            bytes[IR_SELECTOR + 1],
            bytes[IR_SELECTOR + 2],
            bytes[IR_SELECTOR + 3],
        ]);
        Some((
            IdentityV1 { user_id, uuid },
            selector == IR_SELECTOR_BUILTIN,
        ))
    }

    pub(crate) fn lists_uuid(&self, uuid: &[u8; IDENTITY_UUID_LEN]) -> bool {
        (0..self.count())
            .filter_map(|i| self.record(i))
            .any(|(identity, _)| identity.uuid == *uuid)
    }

    pub(crate) fn each_built_in_for(&self, user_id: i32, mut visit: impl FnMut(IdentityV1)) {
        for i in 0..self.count() {
            if let Some((identity, built_in)) = self.record(i) {
                if built_in && identity.user_id == user_id {
                    visit(identity);
                }
            }
        }
    }
}

#[cfg(CONFIG_APPLE_SEP_KUNIT_TEST)]
#[::kernel::macros::kunit_tests(apple_sep_enclave_identity_scope)]
mod identity_scope_tests {
    use super::*;

    #[test]
    fn list_excludes_other_users_and_selectors() {
        let mut reply = [0u8; 3 * IDENTITY_RECORD_LEN];
        for (i, (user, selector, byte)) in [
            (SBIO_PROBE_USER_ID, IR_SELECTOR_BUILTIN, 1u8),
            (501, IR_SELECTOR_BUILTIN, 2u8),
            (SBIO_PROBE_USER_ID, 2, 3u8),
        ].iter().enumerate() {
            let at = i * IDENTITY_RECORD_LEN;
            reply[at..at + 4].copy_from_slice(&user.to_le_bytes());
            reply[at + 4..at + IDENTITY_V1_LEN].fill(*byte);
            reply[at + IR_SELECTOR..at + IR_SELECTOR + 4]
                .copy_from_slice(&selector.to_le_bytes());
        }
        let records = IdentityRecords::new(&reply).unwrap();
        let mut seen = 0;
        records.each_built_in_for(SBIO_PROBE_USER_ID, |identity| {
            seen += 1;
            assert!(*identity.uuid() == [1u8; IDENTITY_UUID_LEN]);
        });
        assert_eq!(seen, 1);
    }

    #[test]
    fn match_result_carries_the_exact_enclave_identity() {
        let mut reply = [0u8; SBIO_MATCH_RESULT_LEN];
        reply[MR_USER_ID..MR_USER_ID + 4]
            .copy_from_slice(&SBIO_PROBE_USER_ID.to_le_bytes());
        reply[MR_IDENTITY..MR_IDENTITY + IDENTITY_UUID_LEN].fill(0xa5);
        let result = MatchResult::parse(&reply).unwrap();
        let user = UserId::new(SBIO_PROBE_USER_ID).unwrap();
        assert!(result.matches(user));
        assert!(*result.identity_uuid() == [0xa5; IDENTITY_UUID_LEN]);
    }
}

const FIC_DEVICE_BUILTIN: u32 = 1;
pub(crate) const SBIO_FREE_COUNT_LEN: usize = 4 + 20;
static_assert!(SBIO_FREE_COUNT_LEN == SBIO_SAVE_SELECTOR_LEN);
pub(crate) const SBIO_FREE_COUNT_REPLY_LEN: usize = 4;
pub(crate) const SBIO_FREE_COUNT_MAX: u32 = 3;

const OP_SBIO_FREE_IDENTITY_COUNT: u16 = 0x38;
pub(crate) fn sbio_free_identity_count(user: UserId) -> SbioOp {
    let mut body = [0u8; SBIO_FREE_COUNT_LEN];
    body[0..4].copy_from_slice(&user.value().to_le_bytes());
    body[4..8].copy_from_slice(&FIC_DEVICE_BUILTIN.to_le_bytes());
    SbioOp {
        opcode: OP_SBIO_FREE_IDENTITY_COUNT,
        payload: pad_payload(&body),
        payload_len: SBIO_FREE_COUNT_LEN,
        name: c"FREE_IDENTITY_COUNT",
    }
}

const OP_SBIO_DELETE_IDENTITY: u16 = 0x57;
pub(crate) fn sbio_delete_identity(identity: &IdentityV1) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_DELETE_IDENTITY,
        payload: pad_payload(&identity.wire()),
        payload_len: IDENTITY_V1_LEN,
        name: c"DELETE_IDENTITY",
    }
}

const OP_SBIO_MATCH_RESULT: u16 = 0x09;
pub(crate) fn sbio_match_result() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_MATCH_RESULT,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"MATCH_RESULT",
    }
}

pub(crate) const SBIO_MATCH_RESULT_LEN: usize = 0xca2;

const MR_USER_ID: usize = 0x000;
const MR_IDENTITY: usize = 0x004;
static_assert!(MR_USER_ID + 4 <= SBIO_MATCH_RESULT_LEN);
static_assert!(MR_IDENTITY + IDENTITY_UUID_LEN <= SBIO_MATCH_RESULT_LEN);
static_assert!(MR_IDENTITY == MR_USER_ID + 4);
static_assert!(SBIO_MATCH_RESULT_LEN != SBIO_ENROL_RESULT_LEN);

pub(crate) const IDENTITY_UUID_LEN: usize = 16;
pub(crate) const IDENTITY_V1_LEN: usize = 4 + IDENTITY_UUID_LEN;
static_assert!(IDENTITY_V1_LEN == 0x14);

#[derive(Clone, Copy)]
pub(crate) struct IdentityV1 {
    user_id: i32,
    uuid: [u8; IDENTITY_UUID_LEN],
}

impl IdentityV1 {
    pub(crate) fn from_index(user_id: i32, uuid: [u8; IDENTITY_UUID_LEN]) -> IdentityV1 {
        IdentityV1 { user_id, uuid }
    }

    pub(crate) fn uuid(&self) -> &[u8; IDENTITY_UUID_LEN] {
        &self.uuid
    }

    pub(crate) fn wire(&self) -> [u8; IDENTITY_V1_LEN] {
        let mut out = [0u8; IDENTITY_V1_LEN];
        out[0..4].copy_from_slice(&self.user_id.to_le_bytes());
        out[4..IDENTITY_V1_LEN].copy_from_slice(&self.uuid);
        out
    }
}

pub(crate) struct MatchResult {
    user_id: i32,
    identity: [u8; IDENTITY_UUID_LEN],
}

impl MatchResult {
    pub(crate) fn parse(bytes: &[u8]) -> Option<MatchResult> {
        if bytes.len() < SBIO_MATCH_RESULT_LEN {
            return None;
        }
        let mut identity = [0u8; IDENTITY_UUID_LEN];
        identity.copy_from_slice(&bytes[MR_IDENTITY..MR_IDENTITY + IDENTITY_UUID_LEN]);
        Some(MatchResult {
            user_id: i32::from_le_bytes([
                bytes[MR_USER_ID],
                bytes[MR_USER_ID + 1],
                bytes[MR_USER_ID + 2],
                bytes[MR_USER_ID + 3],
            ]),
            identity,
        })
    }

    pub(crate) fn identity_uuid(&self) -> &[u8; IDENTITY_UUID_LEN] {
        &self.identity
    }

    pub(crate) fn user_id(&self) -> i32 {
        self.user_id
    }

    pub(crate) fn matches(&self, user: UserId) -> bool {
        self.user_id == user.value()
    }
}

/// Retry capture info: sent after each further frame of a no-match touch has
/// been relayed (0x65). Empty; the enclave answers six bytes of capture
/// metadata (button state, drive voltage), which Linux does not use. It does
/// not count as another attempt. The name is descriptive.
const OP_SBIO_RETRY_CAPTURE_INFO: u16 = 0x3e;
pub(crate) fn sbio_retry_capture_info() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_RETRY_CAPTURE_INFO,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"RETRY_CAPTURE_INFO",
    }
}

/// Match retry finalize: ends the same-touch retry pass of a no-match touch,
/// so the touch counts as one result, before MATCH_POLICY. Empty request and
/// reply. The name is descriptive.
const OP_SBIO_MATCH_RETRY_FINALIZE: u16 = 0x0c;
pub(crate) fn sbio_match_retry_finalize() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_MATCH_RETRY_FINALIZE,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"MATCH_RETRY_FINALIZE",
    }
}

const OP_SBIO_MATCH_CANCEL: u16 = 0x0b;
/// Ends the enclave's match mode when a match operation ends. Empty.
pub(crate) fn sbio_match_cancel() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_MATCH_CANCEL,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"MATCH_CANCEL",
    }
}

/// The background operation result: a u16 "result present" word, a u16
/// "template list updated" word, then the update record.
pub(crate) const SBIO_BG_OPS_RESULT_LEN: usize = 0xc8e;
const BG_OPS_PRESENT: usize = 0x000;
const BG_OPS_UPDATED: usize = 0x002;

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(crate) enum BgOpsResult {
    NoResult,
    TemplateListNotUpdated,
    TemplatesUpdated,
}

pub(crate) fn bg_ops_result_kind(reply: &[u8]) -> BgOpsResult {
    let word = |at: usize| u16::from_le_bytes([reply[at], reply[at + 1]]);
    if reply.len() < BG_OPS_UPDATED + 2 || word(BG_OPS_PRESENT) == 0 {
        BgOpsResult::NoResult
    } else if word(BG_OPS_UPDATED) == 0 {
        BgOpsResult::TemplateListNotUpdated
    } else {
        BgOpsResult::TemplatesUpdated
    }
}

const OP_SBIO_BG_OPS_RESULT: u16 = 0x21;
/// Collects the background operation result the enclave announced with a
/// tag-0x03 notice. The request is empty.
pub(crate) fn sbio_bg_ops_result() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_BG_OPS_RESULT,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"BG_OPS_RESULT",
    }
}

const OP_SBIO_IMAGE_CLEANUP: u16 = 0x22;
pub(crate) fn sbio_image_cleanup() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_IMAGE_CLEANUP,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"IMAGE_CLEANUP",
    }
}

const OP_SBIO_CANCEL: u16 = 0x05;
pub(crate) fn sbio_cancel_operation() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CANCEL,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"CANCEL_OPERATION",
    }
}

const OP_SBIO_CLEAR_STATE: u16 = 0x19;
pub(crate) fn sbio_clear_state() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CLEAR_STATE,
        payload: pad_payload(&[]),
        payload_len: 0,
        name: c"CLEAR_STATE",
    }
}

const OP_SBIO_REGISTER_SERIAL: u16 = 0x17;
pub(crate) fn sbio_register_sensor_serial(serial: &crate::sensor::SensorSerial) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_REGISTER_SERIAL,
        payload: pad_payload(serial.bytes()),
        payload_len: crate::sensor::SENSOR_SERIAL_LEN,
        name: c"REGISTER_SERIAL",
    }
}

const OP_SBIO_DIAGNOSTICS: u16 = 0x63;
pub(crate) const fn sbio_diagnostics() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_DIAGNOSTICS,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"DIAGNOSTICS",
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum ImagePurpose {
    Enrolment,
    Matching,
}

impl ImagePurpose {
    const fn offset(self) -> usize {
        match self {
            ImagePurpose::Matching => 0x12,
            ImagePurpose::Enrolment => 0x13,
        }
    }

    const fn last_image_offset(self) -> usize {
        match self {
            ImagePurpose::Enrolment => 0x08,
            ImagePurpose::Matching => 0x09,
        }
    }
}

static_assert!(ImagePurpose::Enrolment.offset() != ImagePurpose::Matching.offset());
static_assert!(ImagePurpose::Matching.offset() == 0x12);
static_assert!(ImagePurpose::Enrolment.offset() == 0x13);
static_assert!(ImagePurpose::Enrolment.last_image_offset() == 0x08);
static_assert!(ImagePurpose::Matching.last_image_offset() == 0x09);
static_assert!(
    ImagePurpose::Enrolment.last_image_offset() != ImagePurpose::Matching.last_image_offset()
);
static_assert!(ImagePurpose::Matching.last_image_offset() < ImagePurpose::Matching.offset());

#[derive(Clone, Copy)]
pub(crate) struct UserId(i32);

impl UserId {
    pub(crate) fn new(value: i32) -> Option<UserId> {
        if value > 0 {
            Some(UserId(value))
        } else {
            None
        }
    }

    pub(crate) const fn value(&self) -> i32 {
        self.0
    }
}

const IPI_FIRST_IMAGE: usize = 0x07;
const IPI_TIMESTAMP: usize = 0x40;
const IPI_CAPTURE_COUNTER: usize = 0x48;
const IPI_DEVICE_KIND: usize = 0x4f;
const IPI_USER_ID: usize = 0x67;

const IPI_DEVICE_BUILTIN: u32 = 1;
/// The match mode word of an authenticated match.
const IPI_MATCH_MODE: usize = 0x00;
/// Match mode of an authenticated match in the LocalAuthentication class:
/// macOS 26 opens a Touch ID prompt's matches with match flags 0x88
/// (MatchModeACM). It includes the credential-set bit 0x08, which makes the
/// request carry the ACM context as a length-prefixed blob after the base
/// structure. The login window's keybag-unlock match adds bit 0x01 (0x89);
/// that class has its own attempt budget in the enclave, which a password
/// does not reset, and Linux has no keybag-unlock caller. Every verify here
/// uses the LocalAuthentication class.
pub(crate) const SBIO_MATCH_MODE_ACM: u32 = 0x88;
/// The length word of the credential-set blob; its bytes follow the base.
const IPI_CREDENTIAL_LEN: usize = 0x93;
const IPI_CREDENTIAL: usize = SBIO_IMAGE_INIT_LEN;

static_assert!(IPI_MATCH_MODE + 4 <= IPI_FIRST_IMAGE);
static_assert!(IPI_CREDENTIAL_LEN > IPI_USER_ID + 4);
static_assert!(IPI_CREDENTIAL_LEN + 4 == SBIO_IMAGE_INIT_LEN);
static_assert!(IPI_CREDENTIAL + crate::scrd::SCRD_ACM_HANDLE_LEN <= SBIO_MAX_PAYLOAD);

static_assert!(IPI_USER_ID + 4 <= SBIO_MAX_PAYLOAD);
static_assert!(IPI_DEVICE_KIND + 4 <= SBIO_MAX_PAYLOAD);
static_assert!(IPI_TIMESTAMP + 8 <= SBIO_MAX_PAYLOAD);
static_assert!(IPI_CAPTURE_COUNTER + 4 <= SBIO_MAX_PAYLOAD);
static_assert!(IPI_FIRST_IMAGE < ImagePurpose::Matching.offset());
static_assert!(ImagePurpose::Enrolment.offset() < IPI_TIMESTAMP);

const OP_SBIO_PREPARE_IMAGE: u16 = 0x23;
pub(crate) fn sbio_prepare_image_processing() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_PREPARE_IMAGE,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"PREPARE_IMAGE_PROCESSING",
    }
}

const OP_SBIO_IMAGE_INIT: u16 = 0x06;
pub(crate) fn sbio_image_processing_init(
    purpose: ImagePurpose,
    first_image: bool,
    last_image: bool,
    capture_counter: u32,
    user: UserId,
    monotonic_ns: u64,
    scope: Option<&[u8; crate::scrd::SCRD_ACM_HANDLE_LEN]>,
) -> SbioOp {
    let mut payload = [0u8; SBIO_MAX_PAYLOAD];

    // An authenticated match names its mode and appends the live ACM handle
    // as a length-prefixed blob; without it the enclave matches in a scope
    // with its own small limit.
    let mut payload_len = SBIO_IMAGE_INIT_LEN;
    if let Some(handle) = scope {
        payload[IPI_MATCH_MODE..IPI_MATCH_MODE + 4]
            .copy_from_slice(&SBIO_MATCH_MODE_ACM.to_le_bytes());
        payload[IPI_CREDENTIAL_LEN..IPI_CREDENTIAL_LEN + 4]
            .copy_from_slice(&(handle.len() as u32).to_le_bytes());
        payload[IPI_CREDENTIAL..IPI_CREDENTIAL + handle.len()].copy_from_slice(handle);
        payload_len += handle.len();
    }

    if first_image {
        payload[IPI_FIRST_IMAGE] = 1;
    }
    if last_image {
        payload[purpose.last_image_offset()] = 1;
    }
    payload[purpose.offset()] = 1;

    // ns*24/1000 split to dodge u64 overflow and the kernel's absent __udivti3
    let scaled = (monotonic_ns / 1000) * 24 + ((monotonic_ns % 1000) * 24) / 1000;
    payload[IPI_TIMESTAMP..IPI_TIMESTAMP + 8].copy_from_slice(&scaled.to_le_bytes());
    payload[IPI_CAPTURE_COUNTER..IPI_CAPTURE_COUNTER + 4]
        .copy_from_slice(&capture_counter.to_le_bytes());
    payload[IPI_DEVICE_KIND..IPI_DEVICE_KIND + 4]
        .copy_from_slice(&IPI_DEVICE_BUILTIN.to_le_bytes());
    payload[IPI_USER_ID..IPI_USER_ID + 4].copy_from_slice(&user.value().to_le_bytes());

    SbioOp {
        opcode: OP_SBIO_IMAGE_INIT,
        payload,
        payload_len,
        name: c"IMAGE_PROCESSING_INIT",
    }
}

const OP_SBIO_RELAY_CAPTURE: u16 = 0x65;

pub(crate) struct SbioRelay<'a> {
    capture: &'a crate::sensor::Capture,
}

impl<'a> SbioRelay<'a> {
    pub(crate) fn new(capture: &'a crate::sensor::Capture) -> SbioRelay<'a> {
        SbioRelay { capture }
    }

    pub(crate) fn opcode(&self) -> u16 {
        OP_SBIO_RELAY_CAPTURE
    }

    pub(crate) fn payload(&self) -> &[u8] {
        self.capture.bytes()
    }

    pub(crate) fn name(&self) -> &'static CStr {
        c"RELAY_CAPTURE"
    }
}

const OP_SBIO_ASSESSMENT: u16 = 0x07;
pub(crate) fn sbio_image_assessment() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_ASSESSMENT,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"IMAGE_ASSESSMENT",
    }
}

const OP_SBIO_ENROL_RESULT: u16 = 0x08;
pub(crate) fn sbio_enrolment_result() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_ENROL_RESULT,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"ENROLMENT_RESULT",
    }
}

pub(crate) const ASSESS_MIN_LEN: usize = 0x89;
pub(crate) const ASSESS_USABLE_MATCH: usize = 0x06;
pub(crate) const ASSESS_USABLE_ENROL: usize = 0x07;

static_assert!(ASSESS_USABLE_MATCH < ASSESS_USABLE_ENROL);

/// The assessment's feedback word, its finger-problem class and its dirt
/// byte: why the enclave did not use an image.
const ASSESS_FEEDBACK: usize = 14;
const ASSESS_PROBLEM: usize = 40;
const ASSESS_DIRT: usize = 64;
const FEEDBACK_BAD_IMAGE: u32 = 200;
const FEEDBACK_BAD_BLOCKS: u32 = 201;
const FEEDBACK_CHANNEL_NOISE: u32 = 202;
const FEEDBACK_ARRAY_NOISE: u32 = 204;
const FEEDBACK_MOVEMENT: u32 = 205;
const PROBLEM_WET: u32 = 1;
static_assert!(ASSESS_DIRT < ASSESS_MIN_LEN);

/// Why an image assessment rejected the image, in words for the log. The
/// reasons are those macOS turns into its status messages (finger moved,
/// clean the sensor, bad image); with none set the image was simply not
/// good enough (too little of the finger, too light a touch).
pub(crate) fn assessment_reason(a: &[u8]) -> (&'static str, crate::bio::Guidance) {
    use crate::bio::Guidance;
    let word = |at: usize| {
        a.get(at..at + 4)
            .map_or(0, |b| u32::from_le_bytes([b[0], b[1], b[2], b[3]]))
    };
    if a.get(ASSESS_DIRT).is_some_and(|b| *b != 0) {
        return ("dirt on the sensor (clean it)", Guidance::Clean);
    }
    if word(ASSESS_PROBLEM) == PROBLEM_WET {
        return ("a wet finger (dry it)", Guidance::Clean);
    }
    match word(ASSESS_FEEDBACK) {
        FEEDBACK_MOVEMENT => ("the finger moved (hold it still)", Guidance::TooFast),
        FEEDBACK_BAD_IMAGE => ("a bad image", Guidance::LiftAndMove),
        FEEDBACK_BAD_BLOCKS => ("a bad image (damaged blocks)", Guidance::LiftAndMove),
        FEEDBACK_CHANNEL_NOISE | FEEDBACK_ARRAY_NOISE => {
            ("a bad image (sensor noise)", Guidance::LiftAndMove)
        }
        _ => ("too little of the finger, or too light a touch", Guidance::Center),
    }
}

pub(crate) const SBIO_SESSION_SHARE_LEN: usize = 40;

pub(crate) const SBIO_STATUS_OK: u16 = 0x00;
pub(crate) const SBIO_STATUS_PREREQUISITE: u16 = 0x01;
pub(crate) const SBIO_STATUS_16: u16 = 0x16;
static_assert!(SBIO_STATUS_PREREQUISITE != SBIO_STATUS_16);
pub(crate) const SBIO_SESSION_MODE: u32 = 1;
static_assert!(SBIO_SESSION_SHARE_LEN <= SBIO_MAX_PAYLOAD);

const OP_SBIO_SESSION_SHARE: u16 = 0x15;
pub(crate) fn sbio_request_session_share() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_SESSION_SHARE,
        payload: pad_payload(&SBIO_SESSION_MODE.to_le_bytes()),
        payload_len: 4,
        name: c"SESSION_SHARE",
    }
}

const OP_SBIO_COMMIT_SESSION: u16 = 0x16;
pub(crate) fn sbio_commit_session_share(share: &[u8; SBIO_SESSION_SHARE_LEN]) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_COMMIT_SESSION,
        payload: pad_payload(share),
        payload_len: SBIO_SESSION_SHARE_LEN,
        name: c"SESSION_COMMIT",
    }
}
pub(crate) const SBIO_CHALLENGE_LEN: usize = 64;
static_assert!(SBIO_CHALLENGE_LEN <= SBIO_MAX_PAYLOAD);

const OP_SBIO_CHALLENGE: u16 = 0x42;
pub(crate) fn sbio_request_challenge() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_CHALLENGE,
        payload: [0; SBIO_MAX_PAYLOAD],
        payload_len: 0,
        name: c"SEQUENCE_CHALLENGE",
    }
}

const OP_SBIO_COMMIT_CHALLENGE: u16 = 0x18;
pub(crate) fn sbio_commit_challenge(reply: &[u8; SBIO_CHALLENGE_LEN]) -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_COMMIT_CHALLENGE,
        payload: pad_payload(reply),
        payload_len: SBIO_CHALLENGE_LEN,
        name: c"SEQUENCE_COMMIT",
    }
}
const OP_SBIO_FETCH_PATCH: u16 = 0x5f;

const SBIO_PATCH_SUBTYPE: u16 = 2;

pub(crate) fn sbio_fetch_patch() -> SbioOp {
    SbioOp {
        opcode: OP_SBIO_FETCH_PATCH,
        payload: pad_payload(&SBIO_PATCH_SUBTYPE.to_le_bytes()),
        payload_len: 2,
        name: c"FETCH_SENSOR_PATCH",
    }
}

const fn encode_sbio(opcode: u16, marker: u8, seq: u16) -> Message {
    Message {
        msg0: (EP_SBIO as u64)
            | ((marker as u64) << MSG_TAG_SHIFT)
            | ((opcode as u64) << MSG_TYPE_SHIFT)
            | ((seq as u64) << 48),
        msg1: 0,
    }
}

static_assert!(encode_sbio(0x73, transfer::MARKER_FIRST, 0).msg0 == 0x0000_0073_fc08);

pub(crate) fn encode_sbio_raw(opcode: u16, seq: u16) -> Option<Message> {
    Some(encode_sbio(opcode, transfer::MARKER_FIRST, seq))
}

pub(crate) fn encode_sbio_next(opcode: u16, seq: u16) -> Option<Message> {
    Some(encode_sbio(opcode, transfer::MARKER_NEXT, seq))
}

pub(crate) fn encode_sbio_continue(cont: &transfer::Continuation) -> Message {
    encode_sbio(cont.opcode() as u16, transfer::MARKER_REQUEST, cont.seq())
}
