# SPDX-License-Identifier: MIT
"""Execute the actual verify and attach sensor bring-up code with mocks.

The first verify after a system suspend re-registers the sensor, and on a
nonce-session or T6000 profile that registration can fail in a way a new attempt from
a power-cycled sensor recovers. A verify makes up to three attempts back to
back and stops as soon as it is no longer wanted. Attach makes up to five,
spaced 0.5, 1, 2 and 4 s apart, on every profile, and asks for a late retry
when all fail. A retry keeps the stage the failed attempt left, so a
registered serial is cleared first; without a nonce session every second
retry starts from a fresh registration instead.
"""
from pathlib import Path
import re
import subprocess
import tempfile
import unittest

APPLE = Path(__file__).resolve().parents[3] / 'drivers/soc/apple'


def item(source, start):
    """The text of the item starting with `start` in `source`, through its
    closing brace."""
    begin = source.index(start)
    opening = source.index('{', begin)
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[begin:end]


def registration_module():
    """registration.rs without its KUnit tests, as a module."""
    text = (APPLE / 'registration.rs').read_text()
    text = text[:text.index('#[kernel::macros::kunit_tests')]
    return 'mod registration {\n' + text + '\n}\n'


def method(source, name):
    """The text of method `name` in `source`, or '' if there is none."""
    match = re.search(r'\n    (pub\(crate\) )?fn ' + name + r'\b', source)
    if not match:
        return ''
    opening = source.index('{', match.end())
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


PRELUDE = r'''
#![allow(dead_code, unused_variables, unused_imports)]
use std::cell::RefCell;
use std::ffi::CStr;
use std::sync::atomic::{AtomicBool, AtomicU32, AtomicU64, Ordering::Relaxed};
macro_rules! dev_err { ($($args:tt)*) => {} }
macro_rules! dev_warn { ($($args:tt)*) => {} }
macro_rules! dev_info { ($($args:tt)*) => {} }

const ENROL_STATUS_SENSOR: u32 = 1;
const ENROL_STATUS_ENCLAVE: u32 = 2;
const SBIO_PROBE_USER_ID: u32 = 1;
const BRINGUP_FRESH: u32 = registration::FRESH;
const BRINGUP_IDENTIFIED: u32 = registration::IDENTIFIED;
const BRINGUP_ESTABLISHED: u32 = registration::ESTABLISHED;
const MATCH_SETTLE_MS: u32 = 1800;

const LATE_ATTACH_DELAY_MS: u32 = 10_000;
const BRINGUP_PAUSE_STEP_MS: u32 = 100;

#[derive(Default)]
struct Mock {
    /// Outcomes of successive bring-up attempts; missing ones fail.
    script: Vec<bool>,
    attempts: usize,
    /// The verify stops being wanted once this many attempts have run.
    cancel_after: Option<usize>,
    captured: bool,
    cleanup: Vec<&'static str>,
    last_capture_end_ns: u64,
    finished: Option<bio::VerifyOutcome>,
    /// What attach reported to diag, and whether it asked for a late retry.
    attach: Option<registration::AttachState>,
    late: bool,
    /// Milliseconds slept.
    clock_ms: u64,
    /// The bring-up stage each attempt started from.
    stages: Vec<u32>,
    /// Exercise the T6000 registration method against scripted SBIO replies.
    serial_refusal: bool,
    replies: Vec<(&'static CStr, u16)>,
}
thread_local! { static MOCK: RefCell<Mock> = RefCell::new(Mock::default()); }

mod shim {
    pub fn boottime_ns() -> u64 {
        // Zero is the driver's sentinel for a sensor that has never captured.
        super::MOCK.with(|m| (1 + m.borrow().clock_ms) * 1_000_000)
    }
}
mod kernel {
    pub mod time {
        pub struct Delta(pub i64);
        impl Delta {
            pub fn from_millis(ms: i64) -> Self { Delta(ms) }
        }
        pub mod delay {
            pub fn fsleep(d: super::Delta) {
                crate::MOCK.with(|m| m.borrow_mut().clock_ms += d.0 as u64);
            }
        }
    }
}

mod bio {
    pub const TOKEN_LEN: usize = 4;
    #[derive(Clone, Copy, Debug, PartialEq)]
    pub enum VerifyOutcome { Matched(u8), NoMatch, Failed(u32) }
    pub struct Session;
    pub fn verify_is_live(_: &Session, _: u64) -> bool {
        super::MOCK.with(|m| {
            let m = m.borrow();
            m.cancel_after.map_or(true, |n| m.attempts < n)
        })
    }
}
mod sbio {
    pub struct UserId;
    impl UserId { pub fn new(_: u32) -> Option<Self> { Some(UserId) } }
    pub struct SbioOp(&'static super::CStr);
    impl SbioOp { pub fn name(&self) -> &'static super::CStr { self.0 } }
    pub fn sbio_register_sensor(_: &super::sensor::Identifier) -> SbioOp { SbioOp(c"REGISTER_SENSOR") }
    pub fn sbio_clear_state() -> SbioOp { SbioOp(c"CLEAR_STATE") }
    pub fn sbio_register_sensor_serial(_: &()) -> SbioOp { SbioOp(c"REGISTER_SERIAL") }
}
type KVec<T> = Vec<T>;
enum SbioOutcome { Ok(Vec<u8>), Other }
struct Status(u16);
impl Status { fn is_ok(&self) -> bool { self.0 == 0 } }
struct Reply { status: Status, payload: Vec<u8> }
mod sensor {
    pub struct Identifier;
    pub fn idle() -> Result<(), ()> {
        super::MOCK.with(|m| m.borrow_mut().cleanup.push("idle"));
        Ok(())
    }
    pub fn power(on: bool) -> bool {
        if !on { super::MOCK.with(|m| m.borrow_mut().cleanup.push("off")); }
        true
    }
    pub fn is_bound() -> bool { true }
}
mod diag {
    pub fn sensor_bind() -> u32 { 0 }
    pub fn sensor_brought_up(_: u32, _: bool) {}
    pub fn sensor_attach(state: super::registration::AttachState) {
        super::MOCK.with(|m| m.borrow_mut().attach = Some(state));
    }
}
struct PatchLoaded(());
mod profile {
    #[derive(Clone, Copy)]
    pub enum KeyStore { Sepos13 {}, Variant5 }
}
#[derive(Clone, Copy)]
struct Profile { nonce_session: bool, verify_registration: registration::Retry, key_store: profile::KeyStore }
const NONCE: Profile = Profile {
    nonce_session: true, verify_registration: registration::VERIFY, key_store: profile::KeyStore::Variant5,
};
const T6000: Profile = Profile {
    nonce_session: false, verify_registration: registration::VERIFY, key_store: profile::KeyStore::Sepos13 {},
};
const SINGLE: Profile = Profile { verify_registration: registration::ONCE, ..T6000 };
// The bring-up adapter names the driver by its real type.
type SepData = Driver;
struct SessionLock(bio::Session);
impl SessionLock { fn lock(&self) -> &bio::Session { &self.0 } }

struct Driver {
    dev: (),
    profile: Profile,
    templates_restored: AtomicBool,
    cold_restore_candidate: AtomicBool,
    device_view_synced: AtomicBool,
    shutting_down: AtomicBool,
    suspending: AtomicBool,
    capture_epoch: AtomicU64,
    last_capture_end_ns: AtomicU64,
    bio_session: SessionLock,
    bringup: AtomicU32,
    sensor_held: AtomicBool,
}

impl Driver {
    fn new(profile: Profile) -> Self {
        Driver {
            dev: (),
            profile,
            templates_restored: AtomicBool::new(true),
            cold_restore_candidate: AtomicBool::new(false),
            device_view_synced: AtomicBool::new(false),
            shutting_down: AtomicBool::new(false),
            suspending: AtomicBool::new(false),
            capture_epoch: AtomicU64::new(1),
            last_capture_end_ns: AtomicU64::new(0),
            bio_session: SessionLock(bio::Session),
            bringup: AtomicU32::new(registration::FRESH),
            sensor_held: AtomicBool::new(false),
        }
    }
    /// The next scripted attempt. Its registration gets through and a
    /// later step fails, as on the M1 Pro, leaving the stage established.
    fn scripted_attempt(&self) -> bool {
        if MOCK.with(|m| m.borrow().serial_refusal) {
            MOCK.with(|m| {
                let mut m = m.borrow_mut();
                m.attempts += 1;
                m.stages.push(self.bringup.load(Relaxed));
            });
            return self.register_sensor(&sensor::Identifier);
        }
        let stage = self.bringup.swap(registration::ESTABLISHED, Relaxed);
        MOCK.with(|m| {
            let mut m = m.borrow_mut();
            m.stages.push(stage);
            let ok = m.script.get(m.attempts).copied().unwrap_or(false);
            m.attempts += 1;
            ok
        })
    }
    fn bring_sensor_online(&self) -> bool {
        let ok = self.scripted_attempt();
        // A bring-up that completes proves the identity view.
        self.device_view_synced.fetch_or(ok, Relaxed);
        if ok && self.cold_restore_candidate.load(Relaxed) {
            self.templates_restored.store(true, Relaxed);
        }
        ok
    }
    fn refresh_match_credential(&self) {}
    fn read_serial_logged(&self) -> Option<()> { Some(()) }
    fn sbio_transfer(&self, op: &sbio::SbioOp) -> Result<Reply, ()> {
        let status = match op.name() {
            name if name == c"CLEAR_STATE" => 0,
            name if name == c"REGISTER_SERIAL" && self.bringup.load(Relaxed) == BRINGUP_ESTABLISHED => 0x16,
            name if name == c"REGISTER_SERIAL" => 0,
            other => panic!("unexpected SBIO request {other:?}"),
        };
        MOCK.with(|m| m.borrow_mut().replies.push((op.name(), status)));
        Ok(Reply { status: Status(status), payload: Vec::new() })
    }
    fn sensor_trace(&self, _: &CStr) {}
    fn sensor_step_failed(&self, _: &CStr) {}
    fn log_identity_count(&self, _: &CStr) {}
    fn note_sensor_recoveries(&self) {}
    fn sensor_platform_identity() -> Result<(), ()> { unreachable!() }
    fn sensor_session_request(&self, _: u16, _: &CStr, _: &()) -> bool { unreachable!() }
    fn authenticate_sensor(&self) -> bool { unreachable!() }
    fn sbio_call(&self, _: &sbio::SbioOp) -> SbioOutcome { unreachable!() }
    fn mint_token_bytes(&self) -> Option<[u8; bio::TOKEN_LEN]> { Some([1; bio::TOKEN_LEN]) }
    fn verify_one_image(&self) -> bio::VerifyOutcome {
        MOCK.with(|m| m.borrow_mut().captured = true);
        bio::VerifyOutcome::Matched(1)
    }
    fn save_after_match(&self, _: sbio::UserId) -> bool { true }
    fn finish_verify(&self, outcome: bio::VerifyOutcome, _: [u8; bio::TOKEN_LEN]) {
        MOCK.with(|m| {
            let mut m = m.borrow_mut();
            assert!(m.finished.is_none(), "the verify reported twice");
            m.finished = Some(outcome);
        });
    }
    fn wake_sensor(&self) -> Option<PatchLoaded> {
        self.scripted_attempt().then_some(PatchLoaded(()))
    }
    fn complete_bringup(&self, _: PatchLoaded) -> bool { true }
'''

MAIN = r'''
}

fn result(driver: &Driver) -> Mock {
    let mut m = MOCK.with(|m| m.take());
    m.last_capture_end_ns = driver.last_capture_end_ns.load(Relaxed);
    m
}

fn run(script: &[bool], cancel_after: Option<usize>, profile: Profile, verify: bool) -> Mock {
    MOCK.with(|m| *m.borrow_mut() = Mock { script: script.to_vec(), cancel_after, ..Mock::default() });
    let driver = Driver::new(profile);
    if verify {
        driver.run_verify()
    } else {
        let late = driver.attach_bringup();
        MOCK.with(|m| m.borrow_mut().late = late);
    }
    result(&driver)
}

fn cold_verify(script: &[bool], cancel_after: Option<usize>, previous_capture_ms_ago: Option<u64>) -> Mock {
    MOCK.with(|m| *m.borrow_mut() = Mock { script: script.to_vec(), cancel_after, ..Mock::default() });
    let driver = Driver::new(T6000);
    driver.templates_restored.store(false, Relaxed);
    driver.cold_restore_candidate.store(true, Relaxed);
    if let Some(age) = previous_capture_ms_ago {
        driver.note_capture_end();
        MOCK.with(|m| m.borrow_mut().clock_ms += age);
    }
    driver.run_verify();
    result(&driver)
}

fn main() {
    use bio::VerifyOutcome::*;

    // A cancelled cold verify must not wake the sensor for a pending restore proof.
    let m = cold_verify(&[true], Some(0), None);
    assert_eq!((m.attempts, m.captured, m.finished), (0, false, None));
    assert!(m.cleanup.is_empty());

    // Cancellation in a successful or failed proof leaves no powered reader.
    for proof in [true, false] {
        let m = cold_verify(&[proof, true], Some(1), None);
        assert_eq!((m.attempts, m.captured, m.finished), (1, false, None));
        assert_eq!(m.cleanup, ["idle", "off"]);
        assert_eq!(m.last_capture_end_ns, 0);
    }
    // Finishing the proof powers off the reader without delaying its first capture.
    let m = cold_verify(&[false, true, true], None, None);
    assert_eq!((m.attempts, m.captured, m.finished), (3, true, Some(Matched(1))));
    assert_eq!(m.cleanup, ["idle", "off", "idle", "off"]);
    assert_eq!(m.clock_ms, 0);

    // An earlier capture still gets the rest of its original settling interval.
    let age = u64::from(MATCH_SETTLE_MS) / 2;
    let m = cold_verify(&[true, true], None, Some(age));
    assert_eq!((m.captured, m.clock_ms), (true, u64::from(MATCH_SETTLE_MS)));

    // The case seen after a suspend: the first attempt fails, the next works.
    let m = run(&[false, true], None, NONCE, true);
    assert!(m.captured && m.finished == Some(Matched(1)) && m.attempts == 2,
            "a verify gave up after one failed sensor bring-up (attempts {}, finished {:?})",
            m.attempts, m.finished);

    // A sensor that never comes up: three attempts, then a sensor failure.
    let m = run(&[], None, NONCE, true);
    assert_eq!((m.attempts, m.captured, m.finished), (3, false, Some(Failed(ENROL_STATUS_SENSOR))));
    assert_eq!(m.cleanup, ["idle", "off"]);
    assert!(m.last_capture_end_ns > 0);

    // Cancellation before bring-up leaves the sensor alone.
    let m = run(&[true], Some(0), T6000, true);
    assert_eq!((m.attempts, m.captured, m.finished), (0, false, None));
    assert!(m.cleanup.is_empty());

    // Cancelled during the first attempt: power down without another attempt.
    let m = run(&[false, true], Some(1), NONCE, true);
    assert_eq!((m.attempts, m.captured, m.finished), (1, false, None));
    assert_eq!(m.cleanup, ["idle", "off"]);
    assert!(m.last_capture_end_ns > 0);

    // A first attempt that works is the only one.
    let m = run(&[true], None, NONCE, true);
    assert_eq!((m.attempts, m.captured, m.finished), (1, true, Some(Matched(1))));

    // The successful retry may return after the original verify was cancelled.
    let m = run(&[false, true], Some(2), T6000, true);
    assert_eq!((m.attempts, m.captured, m.finished), (2, false, None));
    assert_eq!(m.cleanup, ["idle", "off"]);
    assert!(m.last_capture_end_ns > 0);

    // The M1 Max accepted CLEAR_STATE and refused REGISTER_SERIAL. Exercise the
    // production registration and SBIO reply handling until a fresh try succeeds.
    MOCK.with(|m| *m.borrow_mut() = Mock { serial_refusal: true, ..Mock::default() });
    let driver = Driver::new(T6000);
    driver.bringup.store(BRINGUP_ESTABLISHED, Relaxed);
    driver.run_verify();
    let m = result(&driver);
    assert_eq!((m.attempts, m.captured, m.finished), (3, true, Some(Matched(1))));
    assert_eq!(m.stages, [BRINGUP_ESTABLISHED, BRINGUP_ESTABLISHED, BRINGUP_FRESH]);
    assert_eq!(m.replies, [(c"CLEAR_STATE", 0), (c"REGISTER_SERIAL", 0x16),
                           (c"CLEAR_STATE", 0), (c"REGISTER_SERIAL", 0x16),
                           (c"REGISTER_SERIAL", 0)]);

    // Other single-attempt profiles keep their policy.
    let m = run(&[false, true], None, SINGLE, true);
    assert_eq!((m.attempts, m.captured, m.finished), (1, false, Some(Failed(ENROL_STATUS_SENSOR))));

    // Verify attempts follow each other at once.
    assert_eq!(run(&[], None, NONCE, true).clock_ms, 0);

    // Attach spaces its attempts and recovers without a late retry.
    let m = run(&[false, false, true], None, NONCE, false);
    assert_eq!((m.attempts, m.clock_ms, m.late), (3, 1500, false));
    assert_eq!(m.attach, Some(registration::AttachState::Attached));
    // A nonce-session profile clears instead of starting afresh.
    assert_eq!(m.stages, [registration::FRESH, registration::ESTABLISHED, registration::ESTABLISHED]);

    // An attach that never comes up: five attempts within the budget, then
    // a late retry is asked for and diag says so.
    let m = run(&[], None, NONCE, false);
    assert_eq!((m.attempts, m.clock_ms, m.late), (5, 7500, true));
    assert_eq!(m.attach, Some(registration::AttachState::Retrying));

    let m = run(&[true], None, NONCE, false);
    assert_eq!((m.attempts, m.clock_ms, m.late), (1, 0, false));
    assert_eq!(m.attach, Some(registration::AttachState::Attached));

    // Without a nonce session: the M1 Pro's sensor did not wake at attach.
    // The same spacing; the first retry clears the registered serial (and
    // with it the enclave's session), the second starts fresh.
    let m = run(&[false, false, true], None, T6000, false);
    assert_eq!((m.attempts, m.clock_ms, m.late), (3, 1500, false));
    assert_eq!(m.stages, [registration::FRESH, registration::ESTABLISHED, registration::FRESH]);
    assert_eq!(m.attach, Some(registration::AttachState::Attached));

    let m = run(&[], None, T6000, false);
    assert_eq!((m.attempts, m.clock_ms, m.late), (5, 7500, true));
    assert_eq!(m.attach, Some(registration::AttachState::Retrying));

    let m = run(&[true], None, T6000, false);
    assert_eq!((m.attempts, m.clock_ms, m.late), (1, 0, false));
    assert_eq!(m.attach, Some(registration::AttachState::Attached));
}
'''


class VerifyBringupTest(unittest.TestCase):
    def test_verify_retries_a_failed_bring_up_like_attach(self):
        source = (APPLE / 'sbio.rs').read_text()
        methods = [method(source, name) for name in (
            'run_verify', 'retry_restore_proof', 'attach_bringup',
            'bring_sensor_online_retrying', 'bring_sensor_online_for_verify',
            'attach_bring_sensor_online', 'attach_complete', 'bringup_may_continue',
            'verify_wanted', 'power_down_sensor', 'power_down_capture_sensor',
            'note_capture_end', 'settle_before_capture', 'register_sensor', 'sensor_enclave_step')]
        self.assertTrue(all(methods))
        attempts = (item(source, '#[derive(Clone, Copy, PartialEq, Eq)]\nenum Bringup {')
                    + '\n' + item(source, 'struct BringupAttempts<')
                    + '\n' + item(source, 'impl<W: Fn() -> bool> registration::Attempts'))
        program = (PRELUDE + '\n'.join(methods) + MAIN
                   + registration_module() + attempts)
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'test.rs').write_text(program)
            subprocess.run(['rustc', '--edition=2021', str(root / 'test.rs'),
                            '-o', str(root / 'test')], check=True)
            subprocess.run([str(root / 'test')], check=True)


if __name__ == '__main__':
    unittest.main()
