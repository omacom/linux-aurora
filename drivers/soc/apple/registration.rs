// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Ryan Murray

//! What a Touch ID sensor registration has to send to the enclave.
//!
//! A registration identifies the sensor (REGISTER_SENSOR), registers its
//! serial and, on profiles with a nonce session, authenticates it. Any of
//! those steps can fail after the enclave has already accepted an earlier
//! one, and then the enclave holds a partial registration. On boot
//! d1962b3a every retry that registered the sensor again without clearing
//! it failed, 42 of them over 800 s. Re-registration that identifies the
//! sensor and then clears the enclave's sensor state, the order every
//! verify uses, has not failed on an idle machine. So the next attempt
//! clears whenever the enclave may hold any registration, complete or not.
//!
//! Clearing does not end everything, though: see [`authenticate`]. This
//! file also holds how often, and how far apart, a bring-up is attempted.

/// Nothing is registered with the enclave yet.
pub(crate) const FRESH: u32 = 0;
/// The sensor is identified; profiles without a nonce session register the
/// serial next.
pub(crate) const IDENTIFIED: u32 = 1;
/// The registration completed.
pub(crate) const ESTABLISHED: u32 = 2;

/// The enclave requests one registration attempt makes, in order.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Plan {
    /// PLATFORM_IDENTITY and REGISTER_SENSOR, before anything else. A
    /// nonce-session profile identifies the sensor on every reset.
    pub(crate) identify_first: bool,
    /// CLEAR_STATE, after any identification.
    pub(crate) clear_state: bool,
    /// REGISTER_SENSOR after the clear, on a profile without a nonce session.
    pub(crate) register_sensor: bool,
}

/// Plans one registration attempt.
///
/// `stage` is the bring-up stage and `holds_sensor` whether an
/// identification of the sensor was submitted to the enclave since it
/// booted, answered or not: a lost reply does not mean the enclave did not
/// act on it. `sepos13` says whether the profile starts from the serial, as
/// the 13.5 key store does.
pub(crate) fn plan(stage: u32, holds_sensor: bool, nonce_session: bool, sepos13: bool) -> Plan {
    Plan {
        identify_first: nonce_session,
        clear_state: stage >= ESTABLISHED || (nonce_session && holds_sensor),
        register_sensor: stage == FRESH && !nonce_session && !sepos13,
    }
}

/// Whether the sensor's patch is in place, from the status read after a
/// registration: `accepted` is its patch acknowledgement (status[8]) reading
/// accepted and `needs_patch` its state reading 9. On a profile with a nonce
/// session state 9 overrides an accepted acknowledgement; on one without,
/// the acknowledgement alone decides, as it always has there.
pub(crate) fn patch_loaded(nonce_session: bool, accepted: bool, needs_patch: bool) -> bool {
    accepted && !(nonce_session && needs_patch)
}

/// Whether a completed registration can be left in place.
///
/// Stricter than [`patch_loaded`] on a 13.5 profile: state 9 forces a reload
/// even when the acknowledgement still reads accepted, because a power cycle
/// is what clears a stuck NEEDS_PATCH. `left_powered` is this driver's record
/// that it has not switched the sensor off since that registration.
pub(crate) fn registration_still_held(
    established: bool,
    left_powered: bool,
    silent: bool,
    accepted: bool,
    needs_patch: bool,
) -> bool {
    established && left_powered && !silent && accepted && !needs_patch
}

/// Whether calibration dropped the patch, so a profile without a nonce
/// session has to power-cycle and register again. A nonce-session profile
/// never takes that second wake: its parameter path does not.
pub(crate) fn needs_reload_after_calibration(
    nonce_session: bool,
    silent: bool,
    accepted: bool,
    needs_patch: bool,
) -> bool {
    if nonce_session {
        return false;
    }
    !registration_still_held(true, true, silent, accepted, needs_patch)
}

/// The bring-up stage retry `retry` (counted from 1) starts from.
///
/// The stage an attempt left stands: once the serial is registered, [`plan`]
/// sends CLEAR_STATE before the next registration, as every verify does, and
/// the enclave then asks for the session share and commit again. A retry
/// without it finds the enclave's session from the failed attempt still
/// open: on an M1 Pro the enclave skipped the session share and sent its
/// challenge to a sensor that, power-cycled, no longer held the session,
/// and every such retry failed the same way.
///
/// A profile with a nonce session clears on every attempt anyway. On one
/// without, every second retry starts from a fresh registration instead, as
/// a reboot does and as a verify's second try does: after a failed sequence
/// counter init a J314s refused the CLEAR_STATE re-registration for the
/// rest of the boot, while a fresh one recovered.
pub(crate) fn retry_stage(stage: u32, nonce_session: bool, retry: u32) -> u32 {
    if nonce_session || retry % 2 == 1 {
        stage
    } else {
        FRESH
    }
}

/// Whether a bring-up may go on to its next protocol stage (registration,
/// session and sequence counter, patch, completion). Not once the system
/// has begun going to sleep or the driver is going away: the attempt then
/// stops where it is, and the next one after resume clears what it left
/// with the enclave, as any retry does (see [`retry_stage`]).
pub(crate) fn may_continue(shutting_down: bool, suspending: bool) -> bool {
    !shutting_down && !suspending
}

/// How the enclave answered REGISTER_SERIAL.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum SerialAnswer {
    Accepted,
    /// Status 0x16.
    Refused16,
    /// Any other status, or no answer at all.
    Failed,
}

/// The step at which a nonce-session authentication stopped.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Stopped {
    Serial,
    Nonce,
    RegisterSerial,
    Session,
}

/// How a nonce-session authentication that completed began on the enclave.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Authenticated {
    /// The enclave accepted REGISTER_SERIAL.
    Registered,
    /// The enclave still held the serial of an earlier attempt whose
    /// session never opened, and the session was opened on it.
    Resumed,
}

/// The requests of a nonce-session authentication, which follows the
/// identification of the sensor and any CLEAR_STATE.
pub(crate) trait Authentication {
    type Serial;
    type Nonce;
    /// Reads the serial from the sensor.
    fn read_serial(&mut self) -> Option<Self::Serial>;
    /// Reads a session nonce from the sensor.
    fn read_nonce(&mut self) -> Option<Self::Nonce>;
    /// Sends REGISTER_SERIAL to the enclave.
    fn register_serial(&mut self, serial: &Self::Serial) -> SerialAnswer;
    /// Sends SENSOR_NONCE, the signed root key if it is not loaded yet, and
    /// relays the session share between the enclave and the sensor.
    fn open_session(&mut self, nonce: &Self::Nonce) -> bool;
}

/// Authenticates the sensor on a profile with a nonce session.
///
/// `unfinished` says whether the enclave may hold a serial registration
/// whose session never opened. It is kept up to date here and survives
/// between attempts.
///
/// On boot 0ab402d0 the first attempt had REGISTER_SERIAL accepted and then
/// found the sensor silent when it read the nonce. The next two attempts
/// identified the sensor, cleared the enclave and were refused at
/// REGISTER_SERIAL with 0x16, as a J314s was refused after a failed sequence
/// counter init: CLEAR_STATE does not end a serial registration whose
/// session never opened. So both sensor reads come first, and a sensor that
/// falls silent then leaves the enclave as it was. If the enclave does hold
/// an unfinished registration, because a later step failed, a 0x16 refusal
/// of the new one is the enclave saying so, and the session is opened on
/// the registration it holds.
pub(crate) fn authenticate<A: Authentication>(
    steps: &mut A,
    unfinished: &mut bool,
) -> Result<Authenticated, Stopped> {
    let serial = steps.read_serial().ok_or(Stopped::Serial)?;
    let nonce = steps.read_nonce().ok_or(Stopped::Nonce)?;
    let how = match steps.register_serial(&serial) {
        SerialAnswer::Accepted => Authenticated::Registered,
        SerialAnswer::Refused16 if *unfinished => Authenticated::Resumed,
        SerialAnswer::Refused16 => return Err(Stopped::RegisterSerial),
        SerialAnswer::Failed => {
            // A lost reply does not mean the enclave did not act on it.
            *unfinished = true;
            return Err(Stopped::RegisterSerial);
        }
    };
    *unfinished = true;
    if !steps.open_session(&nonce) {
        return Err(Stopped::Session);
    }
    *unfinished = false;
    Ok(how)
}

/// How many times a sensor bring-up is attempted, and how far apart.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Retry {
    attempts: u32,
    /// The pause before the second attempt; each later one doubles it.
    first_pause_ms: u32,
    /// No attempt starts later than this after the first one began.
    budget_ms: u64,
}

/// At attach, on a profile with a nonce session. Across 29 J700 boots, 7
/// needed a second attempt and one failed three attempts made back to back.
/// Pauses of 0.5, 1, 2 and 4 s give a sensor or an enclave that is still
/// settling time to do so, and five attempts end within about 10 s.
pub(crate) const ATTACH: Retry = Retry {
    attempts: 5,
    first_pause_ms: 500,
    budget_ms: 10_000,
};

/// Before a verify, which someone is waiting on: three attempts, back to
/// back.
pub(crate) const VERIFY: Retry = Retry {
    attempts: 3,
    first_pause_ms: 0,
    budget_ms: u64::MAX,
};

impl Retry {
    /// The pause before attempt `next`, counted from 1, when the first
    /// attempt began `elapsed_ms` ago; `None` if there is no such attempt.
    pub(crate) fn pause_before(&self, next: u32, elapsed_ms: u64) -> Option<u32> {
        if next < 2 || next > self.attempts {
            return None;
        }
        let pause = self.first_pause_ms.saturating_mul(1 << (next - 2).min(31));
        if elapsed_ms.saturating_add(u64::from(pause)) > self.budget_ms {
            return None;
        }
        Some(pause)
    }
}

/// What a sensor bring-up with retries came to.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Outcome {
    /// This attempt brought the sensor online.
    Online(u32),
    /// This many attempts failed, all that were allowed.
    GaveUp(u32),
    /// This many attempts failed, and the result was no longer wanted.
    Abandoned(u32),
}

impl Outcome {
    pub(crate) fn online(&self) -> bool {
        matches!(self, Outcome::Online(_))
    }
}

/// What a bring-up with retries needs from the driver.
pub(crate) trait Attempts {
    /// One bring-up, from a power-cycled sensor.
    fn attempt(&mut self) -> bool;
    /// Whether the result is still wanted.
    fn wanted(&self) -> bool;
    /// Milliseconds since the first attempt began.
    fn elapsed_ms(&self) -> u64;
    /// Attempt `attempt` of `of` failed, and the next one starts `pause_ms`
    /// from now.
    fn failed(&mut self, attempt: u32, of: u32, pause_ms: u32);
    /// Waits `ms`. False if it stopped early because the result is no
    /// longer wanted.
    fn pause(&mut self, ms: u32) -> bool;
}

/// Brings the sensor up, starting again after a failed attempt as `retry`
/// allows. Every path ends within the attempts and the budget of `retry`.
pub(crate) fn bring_up(retry: &Retry, steps: &mut impl Attempts) -> Outcome {
    let mut attempt = 1;
    loop {
        if steps.attempt() {
            return Outcome::Online(attempt);
        }
        let Some(pause) = retry.pause_before(attempt + 1, steps.elapsed_ms()) else {
            return Outcome::GaveUp(attempt);
        };
        if !steps.wanted() {
            return Outcome::Abandoned(attempt);
        }
        steps.failed(attempt, retry.attempts, pause);
        if pause > 0 && !steps.pause(pause) {
            return Outcome::Abandoned(attempt);
        }
        attempt += 1;
    }
}

/// The attach of the sensor, as `diag/sensor_attach` reports it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum AttachState {
    /// The sensor came online and the enclave's identity view was proven.
    Attached = 1,
    /// The attach-time bring-up fell short and a later one is still to run.
    Retrying = 2,
    /// Every automatic bring-up fell short. An enrol or verify still tries.
    NotReady = 3,
}

/// The attach after a round of bring-up attempts. `complete` says whether
/// the sensor came online and the identity view was proven, and
/// `round_left` whether another round will follow.
pub(crate) fn after_round(complete: bool, round_left: bool) -> AttachState {
    if complete {
        AttachState::Attached
    } else if round_left {
        AttachState::Retrying
    } else {
        AttachState::NotReady
    }
}

#[kernel::macros::kunit_tests(apple_sep_registration)]
mod tests {
    use super::*;

    #[test]
    fn first_registration_after_boot_does_not_clear() {
        assert_eq!(
            plan(FRESH, false, true, false),
            Plan {
                identify_first: true,
                clear_state: false,
                register_sensor: false
            }
        );
    }

    #[test]
    fn retry_after_a_partial_registration_clears() {
        // REGISTER_SENSOR was accepted, then the serial, nonce or session
        // step failed: the bring-up stage is still FRESH.
        let retry = plan(FRESH, true, true, false);
        assert!(retry.identify_first);
        assert!(retry.clear_state);
    }

    #[test]
    fn re_registration_after_a_complete_one_clears() {
        assert!(plan(ESTABLISHED, true, true, false).clear_state);
        assert!(plan(ESTABLISHED, false, false, false).clear_state);
    }

    #[test]
    fn a_fresh_restart_still_clears_what_the_enclave_holds() {
        // Restarting the bring-up from FRESH does not erase the enclave's
        // registration; only CLEAR_STATE does.
        assert!(plan(FRESH, true, true, false).clear_state);
    }

    #[test]
    fn profiles_without_a_nonce_session_are_unchanged() {
        assert_eq!(
            plan(FRESH, true, false, false),
            Plan {
                identify_first: false,
                clear_state: false,
                register_sensor: true
            }
        );
        assert_eq!(
            plan(IDENTIFIED, true, false, false),
            Plan {
                identify_first: false,
                clear_state: false,
                register_sensor: false
            }
        );
        assert_eq!(
            plan(FRESH, false, false, true),
            Plan {
                identify_first: false,
                clear_state: false,
                register_sensor: false
            }
        );
    }

    #[test]
    fn a_held_registration_needs_every_condition() {
        assert!(registration_still_held(true, true, false, true, false));
        assert!(!registration_still_held(false, true, false, true, false));
        assert!(!registration_still_held(true, false, false, true, false));
        assert!(!registration_still_held(true, true, true, true, false));
        assert!(!registration_still_held(true, true, false, false, false));
        assert!(!registration_still_held(true, true, false, true, true));
    }

    #[test]
    fn calibration_skips_the_second_wake_while_the_patch_remains() {
        assert!(!needs_reload_after_calibration(false, false, true, false));
    }

    #[test]
    fn calibration_reloads_when_the_patch_is_gone() {
        assert!(needs_reload_after_calibration(false, false, true, true));
        assert!(needs_reload_after_calibration(false, true, true, false));
        assert!(needs_reload_after_calibration(false, false, false, false));
    }

    #[test]
    fn a_nonce_session_never_takes_the_post_calibration_wake() {
        assert!(!needs_reload_after_calibration(true, true, false, true));
        assert!(!needs_reload_after_calibration(true, false, true, false));
    }

    /// The enclave's sensor registration, as the boots on record show it.
    #[derive(Clone, Copy, Debug, PartialEq, Eq)]
    enum Enclave {
        Empty,
        /// REGISTER_SERIAL was accepted and the session never opened.
        /// CLEAR_STATE does not end this, and REGISTER_SERIAL is refused
        /// with 0x16 while it lasts.
        SerialHeld,
        Established,
    }

    const ATTEMPT_MS: u64 = 100;

    /// A J700 sensor and enclave, scripted per attempt of a round.
    struct Model {
        enclave: Enclave,
        /// Whether the sensor answers the nonce read.
        nonce: [bool; 8],
        /// Whether the session exchange completes.
        session: [bool; 8],
        /// Whether REGISTER_SERIAL is refused with 0x16 whatever the
        /// enclave holds, as by an enclave that is still settling.
        busy: [bool; 8],
        attempt: usize,
        identified: bool,
        unfinished: bool,
        registered: u32,
        sessions: u32,
        resumed: bool,
        stopped: [Option<Stopped>; 8],
        wanted_for: usize,
        elapsed_ms: u64,
        pauses: [u32; 8],
        paused: usize,
        failed: u32,
    }

    impl Model {
        fn new() -> Model {
            Model {
                enclave: Enclave::Empty,
                nonce: [true; 8],
                session: [true; 8],
                busy: [false; 8],
                attempt: 0,
                identified: false,
                unfinished: false,
                registered: 0,
                sessions: 0,
                resumed: false,
                stopped: [None; 8],
                wanted_for: usize::MAX,
                elapsed_ms: 0,
                pauses: [0; 8],
                paused: 0,
                failed: 0,
            }
        }

        /// Starts another round, as the late retry does.
        fn next_round(&mut self) {
            self.attempt = 0;
            self.elapsed_ms = 0;
            self.paused = 0;
            self.stopped = [None; 8];
        }
    }

    impl Authentication for Model {
        type Serial = ();
        type Nonce = ();

        fn read_serial(&mut self) -> Option<()> {
            Some(())
        }

        fn read_nonce(&mut self) -> Option<()> {
            self.nonce[self.attempt].then_some(())
        }

        fn register_serial(&mut self, _serial: &()) -> SerialAnswer {
            self.registered += 1;
            if self.busy[self.attempt] || self.enclave == Enclave::SerialHeld {
                return SerialAnswer::Refused16;
            }
            self.enclave = Enclave::SerialHeld;
            SerialAnswer::Accepted
        }

        fn open_session(&mut self, _nonce: &()) -> bool {
            self.sessions += 1;
            if self.enclave != Enclave::SerialHeld || !self.session[self.attempt] {
                return false;
            }
            self.enclave = Enclave::Established;
            true
        }
    }

    impl Attempts for Model {
        fn attempt(&mut self) -> bool {
            if plan(FRESH, self.identified, true, false).clear_state
                && self.enclave == Enclave::Established
            {
                self.enclave = Enclave::Empty;
            }
            self.identified = true;
            let mut unfinished = self.unfinished;
            let result = authenticate(self, &mut unfinished);
            self.unfinished = unfinished;
            match result {
                Ok(how) => self.resumed |= how == Authenticated::Resumed,
                Err(step) => self.stopped[self.attempt] = Some(step),
            }
            self.attempt += 1;
            self.elapsed_ms += ATTEMPT_MS;
            result.is_ok()
        }

        fn wanted(&self) -> bool {
            self.attempt < self.wanted_for
        }

        fn elapsed_ms(&self) -> u64 {
            self.elapsed_ms
        }

        fn failed(&mut self, _attempt: u32, _of: u32, _pause_ms: u32) {
            self.failed += 1;
        }

        fn pause(&mut self, ms: u32) -> bool {
            self.pauses[self.paused] = ms;
            self.paused += 1;
            self.elapsed_ms += u64::from(ms);
            true
        }
    }

    #[test]
    fn a_silent_first_attempt_is_retried_after_a_pause() {
        // Boot 0ab402d0's first attempt: the sensor fell silent at the
        // nonce read.
        let mut m = Model::new();
        m.nonce[0] = false;
        assert_eq!(bring_up(&ATTACH, &mut m), Outcome::Online(2));
        assert_eq!(m.stopped[0], Some(Stopped::Nonce));
        // The enclave never heard of the serial the silent attempt read, so
        // the next registration was not refused.
        assert_eq!(m.registered, 1);
        assert!(!m.resumed);
        assert_eq!(&m.pauses[..m.paused], &[500]);
        assert_eq!(m.enclave, Enclave::Established);
        assert!(!m.unfinished);
    }

    #[test]
    fn repeated_0x16_refusals_then_success() {
        let mut m = Model::new();
        m.busy[0] = true;
        m.busy[1] = true;
        assert_eq!(bring_up(&ATTACH, &mut m), Outcome::Online(3));
        assert_eq!(m.stopped[0], Some(Stopped::RegisterSerial));
        assert_eq!(m.stopped[1], Some(Stopped::RegisterSerial));
        // A refusal the enclave holds nothing for is not resumed.
        assert_eq!(m.sessions, 1);
        assert!(!m.resumed);
        assert_eq!(&m.pauses[..m.paused], &[500, 1000]);
        assert_eq!(m.failed, 2);
    }

    #[test]
    fn a_0x16_refusal_of_an_unfinished_registration_resumes_it() {
        // The session of the first attempt failed after REGISTER_SERIAL was
        // accepted. Clearing does not end that registration, so the second
        // REGISTER_SERIAL is refused; the session opens on the held one.
        let mut m = Model::new();
        m.session[0] = false;
        assert_eq!(bring_up(&ATTACH, &mut m), Outcome::Online(2));
        assert_eq!(m.stopped[0], Some(Stopped::Session));
        assert!(m.resumed);
        assert_eq!(m.enclave, Enclave::Established);
        assert!(!m.unfinished);
    }

    #[test]
    fn a_registration_without_a_reply_counts_as_unfinished() {
        struct Lost {
            opened: bool,
        }
        impl Authentication for Lost {
            type Serial = ();
            type Nonce = ();
            fn read_serial(&mut self) -> Option<()> {
                Some(())
            }
            fn read_nonce(&mut self) -> Option<()> {
                Some(())
            }
            fn register_serial(&mut self, _serial: &()) -> SerialAnswer {
                SerialAnswer::Failed
            }
            fn open_session(&mut self, _nonce: &()) -> bool {
                self.opened = true;
                true
            }
        }
        let mut lost = Lost { opened: false };
        let mut unfinished = false;
        assert_eq!(
            authenticate(&mut lost, &mut unfinished),
            Err(Stopped::RegisterSerial)
        );
        assert!(unfinished);
        assert!(!lost.opened);
    }

    #[test]
    fn a_bring_up_that_never_succeeds_gives_up_and_reports_not_ready() {
        let mut m = Model::new();
        m.nonce = [false; 8];
        let outcome = bring_up(&ATTACH, &mut m);
        assert_eq!(outcome, Outcome::GaveUp(5));
        assert_eq!(&m.pauses[..m.paused], &[500, 1000, 2000, 4000]);
        assert!(m.elapsed_ms <= ATTACH.budget_ms + ATTEMPT_MS);
        // A late round is still to come.
        assert_eq!(after_round(outcome.online(), true), AttachState::Retrying);

        m.next_round();
        let outcome = bring_up(&ATTACH, &mut m);
        assert_eq!(outcome, Outcome::GaveUp(5));
        assert_eq!(after_round(outcome.online(), false), AttachState::NotReady);
        // Nothing reached the enclave, so nothing is left half open.
        assert_eq!(m.registered, 0);
        assert_eq!(m.enclave, Enclave::Empty);
        assert!(!m.unfinished);
    }

    #[test]
    fn a_completed_attach_is_reported_attached() {
        assert_eq!(after_round(true, true), AttachState::Attached);
        assert_eq!(after_round(true, false), AttachState::Attached);
    }

    #[test]
    fn no_attempt_starts_past_the_budget() {
        assert_eq!(ATTACH.pause_before(1, 0), None);
        assert_eq!(ATTACH.pause_before(2, 0), Some(500));
        assert_eq!(ATTACH.pause_before(5, 6_000), Some(4000));
        assert_eq!(ATTACH.pause_before(5, 6_001), None);
        assert_eq!(ATTACH.pause_before(6, 0), None);

        // Attempts that each take as long as an enclave timeout end early.
        let mut m = Model::new();
        m.nonce = [false; 8];
        struct Slow(Model);
        impl Attempts for Slow {
            fn attempt(&mut self) -> bool {
                self.0.elapsed_ms += 5_000;
                self.0.attempt()
            }
            fn wanted(&self) -> bool {
                self.0.wanted()
            }
            fn elapsed_ms(&self) -> u64 {
                self.0.elapsed_ms()
            }
            fn failed(&mut self, attempt: u32, of: u32, pause_ms: u32) {
                self.0.failed(attempt, of, pause_ms)
            }
            fn pause(&mut self, ms: u32) -> bool {
                self.0.pause(ms)
            }
        }
        let mut slow = Slow(m);
        assert_eq!(bring_up(&ATTACH, &mut slow), Outcome::GaveUp(2));
    }

    #[test]
    fn verify_attempts_follow_each_other_at_once() {
        assert_eq!(VERIFY.pause_before(2, 0), Some(0));
        assert_eq!(VERIFY.pause_before(3, u64::MAX - 1), Some(0));
        assert_eq!(VERIFY.pause_before(4, 0), None);

        let mut m = Model::new();
        m.nonce = [false; 8];
        assert_eq!(bring_up(&VERIFY, &mut m), Outcome::GaveUp(3));
        assert_eq!(m.paused, 0);
        assert_eq!(m.failed, 2);
    }

    #[test]
    fn a_retry_clears_first_then_tries_a_fresh_registration() {
        // Without a nonce session: the first retry keeps the stage, so a
        // registered serial is cleared; the second starts fresh.
        assert_eq!(retry_stage(ESTABLISHED, false, 1), ESTABLISHED);
        assert!(plan(retry_stage(ESTABLISHED, false, 1), true, false, true).clear_state);
        assert_eq!(retry_stage(ESTABLISHED, false, 2), FRESH);
        assert_eq!(
            plan(retry_stage(ESTABLISHED, false, 2), true, false, false),
            Plan {
                identify_first: false,
                clear_state: false,
                register_sensor: true,
            }
        );
        assert_eq!(retry_stage(ESTABLISHED, false, 3), ESTABLISHED);
        assert_eq!(retry_stage(IDENTIFIED, false, 1), IDENTIFIED);
        // A nonce-session profile keeps its stage and clears every time.
        for retry in 1..=4 {
            assert_eq!(retry_stage(ESTABLISHED, true, retry), ESTABLISHED);
            assert!(plan(retry_stage(FRESH, true, retry), true, true, false).clear_state);
        }
    }

    /// The M1 Pro (13.5 key store, no nonce session) enclave's view of a
    /// bring-up: the session it shared survives a sensor power cycle and a
    /// fresh registration, and only CLEAR_STATE ends it. A bring-up asks
    /// for a session share only while it has none.
    struct SessionEnclave {
        stage: u32,
        session: bool,
        shares: [bool; 8],
        attempt: usize,
        /// The first `fail` attempts fail after the session commit, as at
        /// the sequence counter's challenge.
        fail: usize,
        /// As the J314s did after a failed sequence counter init: the
        /// enclave refuses the serial re-registration that follows
        /// CLEAR_STATE, and a fresh registration starts a new session.
        clear_refused: bool,
        /// Whether the system begins to sleep during this attempt, once the
        /// serial is registered and before the session share.
        suspend_at: Option<usize>,
        suspending: bool,
    }

    impl SessionEnclave {
        fn new(fail: usize) -> SessionEnclave {
            SessionEnclave {
                stage: FRESH,
                session: false,
                shares: [false; 8],
                attempt: 0,
                fail,
                clear_refused: false,
                suspend_at: None,
                suspending: false,
            }
        }

        fn attempt(&mut self, retry: u32) -> bool {
            let this = self.attempt;
            self.attempt += 1;
            if retry > 0 {
                self.stage = retry_stage(self.stage, false, retry);
            }
            let clear = plan(self.stage, true, false, true).clear_state;
            if clear && self.clear_refused {
                // REGISTER_SERIAL after CLEAR_STATE: 0x16.
                return false;
            }
            if clear || (self.clear_refused && self.stage == FRESH) {
                self.session = false;
            }
            self.stage = ESTABLISHED;
            if self.suspend_at == Some(this) {
                self.suspending = true;
            }
            if !may_continue(false, self.suspending) {
                // Stopped between registration and the session share.
                return false;
            }
            self.shares[this] = !self.session;
            self.session = true;
            // The sensor was power-cycled: it holds a session only if it
            // took part in this attempt's share.
            self.shares[this] && this >= self.fail
        }
    }

    /// A round of attempts on [`SessionEnclave`], as the driver makes them.
    struct Round<'a> {
        enclave: &'a mut SessionEnclave,
        retry: u32,
    }

    impl Attempts for Round<'_> {
        fn attempt(&mut self) -> bool {
            let retry = self.retry;
            self.retry += 1;
            self.enclave.attempt(retry)
        }
        fn wanted(&self) -> bool {
            may_continue(false, self.enclave.suspending)
        }
        fn elapsed_ms(&self) -> u64 {
            0
        }
        fn failed(&mut self, _attempt: u32, _of: u32, _pause_ms: u32) {}
        fn pause(&mut self, _ms: u32) -> bool {
            self.wanted()
        }
    }

    #[test]
    fn a_retry_after_a_post_commit_failure_shares_the_session_again() {
        let mut e = SessionEnclave::new(1);
        assert!(!e.attempt(0));
        // The first retry clears the enclave, so it shares and commits again.
        assert!(e.attempt(1));
        assert_eq!(&e.shares[..2], &[true, true]);

        // Had it started fresh, the enclave would have skipped the share.
        let mut fresh = SessionEnclave::new(1);
        assert!(!fresh.attempt(0));
        assert!(!fresh.attempt(2));
        assert_eq!(&fresh.shares[..2], &[true, false]);
        // The retry after it clears, and recovers.
        assert!(fresh.attempt(3));
        assert!(fresh.shares[2]);
    }

    #[test]
    fn a_refused_clear_is_followed_by_a_fresh_registration_that_recovers() {
        // The J314s: after a failed sequence counter init, the CLEAR_STATE
        // re-registration is refused for the rest of the boot.
        let mut e = SessionEnclave::new(1);
        e.clear_refused = true;
        let mut round = Round {
            enclave: &mut e,
            retry: 0,
        };
        assert_eq!(bring_up(&ATTACH, &mut round), Outcome::Online(3));
        // Attempt 1 shared and failed later, attempt 2 was refused at the
        // re-registration, attempt 3 started fresh and shared again.
        assert_eq!(&e.shares[..3], &[true, false, true]);
        assert_eq!(e.stage, ESTABLISHED);
    }

    #[test]
    fn a_suspend_mid_attempt_stops_cleanly_and_resume_recovers() {
        // The system begins to sleep after the first attempt registered the
        // serial and before its session share.
        let mut e = SessionEnclave::new(0);
        e.suspend_at = Some(0);
        let mut round = Round {
            enclave: &mut e,
            retry: 0,
        };
        // No further stage and no further attempt.
        assert_eq!(bring_up(&ATTACH, &mut round), Outcome::Abandoned(1));
        assert_eq!(e.attempt, 1);
        assert_eq!(e.shares[0], false);
        // What it left: a registered serial, which the next attempt clears.
        assert_eq!(e.stage, ESTABLISHED);
        assert!(plan(retry_stage(e.stage, false, 1), true, false, true).clear_state);

        // Resumed: the late round counts on from the round before it.
        e.suspending = false;
        let mut round = Round {
            enclave: &mut e,
            retry: 1,
        };
        assert_eq!(bring_up(&ATTACH, &mut round), Outcome::Online(1));
        assert!(e.shares[1]);
        assert!(e.session);
    }

    #[test]
    fn a_bring_up_continues_only_while_wanted() {
        assert!(may_continue(false, false));
        assert!(!may_continue(false, true));
        assert!(!may_continue(true, false));
        assert!(!may_continue(true, true));
    }

    /// A sensor that falls silent at the sequence counter init, as the
    /// M1 Pro's did about 1.3 s after power-on, for its first `fail` attempts.
    struct Silent {
        fail: u32,
        tries: u32,
        elapsed_ms: u64,
        pauses: [u32; 8],
        paused: usize,
    }

    impl Silent {
        fn new(fail: u32) -> Silent {
            Silent {
                fail,
                tries: 0,
                elapsed_ms: 0,
                pauses: [0; 8],
                paused: 0,
            }
        }
    }

    impl Attempts for Silent {
        fn attempt(&mut self) -> bool {
            self.tries += 1;
            self.elapsed_ms += 1_300;
            self.tries > self.fail
        }
        fn wanted(&self) -> bool {
            true
        }
        fn elapsed_ms(&self) -> u64 {
            self.elapsed_ms
        }
        fn failed(&mut self, _attempt: u32, _of: u32, _pause_ms: u32) {}
        fn pause(&mut self, ms: u32) -> bool {
            self.pauses[self.paused] = ms;
            self.paused += 1;
            self.elapsed_ms += u64::from(ms);
            true
        }
    }

    #[test]
    fn a_sensor_that_did_not_wake_at_attach_is_retried_after_pauses() {
        let mut s = Silent::new(2);
        assert_eq!(bring_up(&ATTACH, &mut s), Outcome::Online(3));
        assert_eq!(&s.pauses[..s.paused], &[500, 1000]);

        // Never: four attempts of 1.3 s fit in the budget, then the late
        // round is asked for.
        let mut s = Silent::new(u32::MAX);
        let outcome = bring_up(&ATTACH, &mut s);
        assert_eq!(outcome, Outcome::GaveUp(4));
        assert_eq!(&s.pauses[..s.paused], &[500, 1000, 2000]);
        assert_eq!(after_round(outcome.online(), true), AttachState::Retrying);
    }

    #[test]
    fn the_status_after_registration_decides_the_patch() {
        // Nonce session: state 9 asks for the patch even when acknowledged.
        assert!(patch_loaded(true, true, false));
        assert!(!patch_loaded(true, true, true));
        assert!(!patch_loaded(true, false, false));
        // Without one, the acknowledgement alone decides.
        assert!(patch_loaded(false, true, false));
        assert!(patch_loaded(false, true, true));
        assert!(!patch_loaded(false, false, false));
        assert!(!patch_loaded(false, false, true));
    }

    #[test]
    fn an_unwanted_bring_up_stops_without_another_attempt() {
        let mut m = Model::new();
        m.nonce = [false; 8];
        m.wanted_for = 1;
        assert_eq!(bring_up(&ATTACH, &mut m), Outcome::Abandoned(1));
        assert_eq!(m.attempt, 1);
        assert_eq!(m.paused, 0);
    }
}
