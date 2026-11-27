# SPDX-License-Identifier: MIT
"""Run the driver's pre-enrolment bring-up against a scripted sensor."""
from pathlib import Path
import subprocess
import tempfile
import unittest

from test_verify_bringup import APPLE, method, registration_module


class EnrolBringupTest(unittest.TestCase):
    def test_registration_before_opening_an_enrolment(self):
        source = (APPLE / 'sbio.rs').read_text()
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / 'test.rs').write_text(
                registration_module() + HARNESS + method(source, 'power_down_sensor')
                + method(source, 'power_down_capture_sensor')
                + method(source, 'bring_sensor_online_for_enrol') + CASES)
            subprocess.run(['rustc', '--edition=2021', str(root / 'test.rs'), '-o', str(root / 'test')], check=True)
            subprocess.run([str(root / 'test')], check=True)


HARNESS = r"""
use std::cell::Cell;
use std::ffi::CStr;
use std::sync::atomic::{AtomicBool, Ordering::Relaxed};
macro_rules! dev_err { ($($args:tt)*) => {} }
const ENROL_STATUS_SENSOR: u32 = 1;
thread_local! { static EVENTS: std::cell::RefCell<Vec<&'static str>> = Default::default(); }
fn event(name: &'static str) { EVENTS.with(|events| events.borrow_mut().push(name)); }
mod sensor {
    pub fn idle() -> bool { super::event("idle"); true }
    pub fn power(on: bool) { assert!(!on); super::event("off"); }
}
struct Profile { enrol_registration: registration::Retry }
enum Bringup { Enrol }
struct Session { epoch: u64, attempts: Cell<u32>, cancel_after: u32 }
impl Session { fn lock(&self) -> &Self { self } }
mod bio {
    pub fn enrol_is_live(s: &super::Session, epoch: u64) -> bool {
        s.epoch == epoch && s.attempts.get() < s.cancel_after
    }
}
struct Sensor {
    profile: Profile, bio_session: Session, shutting_down: AtomicBool,
    suspending: AtomicBool, ready_after: u32,
}
struct Round<'a, W>(&'a Sensor, W);
impl<W: Fn() -> bool> registration::Attempts for Round<'_, W> {
    fn attempt(&mut self) -> bool { self.0.bring_sensor_online() }
    fn wanted(&self) -> bool { (self.1)() }
    fn elapsed_ms(&self) -> u64 { 0 }
    fn failed(&mut self, _: u32, _: u32, _: u32) {}
    fn pause(&mut self, _: u32) -> bool { true }
}
impl Sensor {
    fn note_capture_end(&self) { event("settle"); }
    fn finish_enrolment(&self, result: Result<(), u32>) {
        assert_eq!(result, Err(ENROL_STATUS_SENSOR)); event("fault");
    }
    fn bring_sensor_online(&self) -> bool {
        let count = &self.bio_session.attempts;
        count.set(count.get() + 1);
        count.get() == self.ready_after
    }
    fn bring_sensor_online_retrying(&self, _: &CStr, _: Bringup, policy: &registration::Retry,
                                    wanted: impl Fn() -> bool) -> registration::Outcome {
        registration::bring_up(policy, &mut Round(self, wanted))
    }
"""

CASES = r"""
}
fn run(policy: registration::Retry, ready_after: u32, cancel_after: u32, epoch: u64) -> (bool, u32, Vec<&'static str>) {
    let sensor = Sensor {
        profile: Profile { enrol_registration: policy },
        bio_session: Session { epoch: 10, attempts: Cell::new(0), cancel_after },
        suspending: false.into(), shutting_down: false.into(), ready_after,
    };
    (sensor.bring_sensor_online_for_enrol(epoch), sensor.bio_session.attempts.get(), EVENTS.with(|e| e.take()))
}
fn main() {
    use registration::{ONCE, VERIFY};
    assert_eq!(run(VERIFY, 1, 99, 10), (true, 1, vec![]));
    assert_eq!(run(VERIFY, 3, 99, 10), (true, 3, vec![]));
    assert_eq!(run(VERIFY, 4, 99, 10), (false, 3, vec!["idle", "off", "settle", "fault"]));
    assert_eq!(run(ONCE, 2, 99, 10), (false, 1, vec!["idle", "off", "settle", "fault"]));
    assert_eq!(run(VERIFY, 2, 1, 10), (false, 1, vec!["idle", "off", "settle"]));
    assert_eq!(run(VERIFY, 1, 1, 10), (false, 1, vec!["idle", "off", "settle"])); // cancelled during a successful attempt
    assert_eq!(run(VERIFY, 1, 99, 9), (false, 0, vec![])); // obsolete worker
}
"""
