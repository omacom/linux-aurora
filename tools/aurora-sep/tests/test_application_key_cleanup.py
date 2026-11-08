# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run actual credential and application-key paths with scripted firmware replies.

This complements the lifecycle/UAPI tests: allocation, transport, policy and
capture are adapters, not a second implementation of cleanup/publication. No
SEP device, secret material or live fault injection is involved.
"""
from pathlib import Path
import re
import shutil
import subprocess
import tempfile
import unittest

APPLE = Path(__file__).resolve().parents[3] / 'drivers/soc/apple'


def function(source, name):
    match = re.search(r'^ *(?:pub\(crate\) )?fn ' + name + r'\(', source, re.M)
    if match is None:
        raise AssertionError(f'Missing production function {name}')
    opening = source.index('{', match.start())
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


HARNESS = r'''
#![allow(dead_code, unused_imports, unused_variables)]
use std::ffi::CStr;
use std::cell::Cell;
use std::sync::atomic::{AtomicBool, AtomicUsize, Ordering::Relaxed};

type Result<T = ()> = std::result::Result<T, Error>;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Error(i32);
impl Error { fn from_errno(n: i32) -> Self { Self(n) } }
const EIO: Error = Error(-5);
const EINVAL: Error = Error(-22);
const ENOENT: Error = Error(-2);
const EACCES: Error = Error(-13);
const ECANCELED: Error = Error(-125);
const ETIMEDOUT: Error = Error(-110);
const EROFS: Error = Error(-30);
const EBUSY: Error = Error(-16);
const EOPNOTSUPP: Error = Error(-95);
const GFP_KERNEL: () = ();
const SBIO_PROBE_USER_ID: i32 = 1000;
const ENROL_STATUS_TIMEOUT: u32 = 1;
static DELETES: AtomicUsize = AtomicUsize::new(0);
static WIPES: AtomicUsize = AtomicUsize::new(0);
macro_rules! dev_err { ($($t:tt)*) => { () }; }
macro_rules! static_assert { ($e:expr) => { const _: () = assert!($e); }; }
struct KVec<T>(Vec<T>);
impl<T> KVec<T> {
    fn new() -> Self { Self(Vec::new()) }
    fn as_slice(&self) -> &[T] { &self.0 }
    fn push(&mut self, value: T, _: ()) -> Result { self.0.push(value); Ok(()) }
}
impl<T: Clone> KVec<T> {
    fn extend_from_slice(&mut self, value: &[T], _: ()) -> Result {
        self.0.extend_from_slice(value); Ok(())
    }
}
impl<T> std::ops::Deref for KVec<T> { type Target = [T]; fn deref(&self) -> &[T] { &self.0 } }
impl<T> std::ops::DerefMut for KVec<T> { fn deref_mut(&mut self) -> &mut [T] { &mut self.0 } }
fn bytes(value: &[u8]) -> KVec<u8> { KVec(value.to_vec()) }
struct Mutex<T>(std::sync::Mutex<T>);
impl<T> Mutex<T> {
    fn new(value: T) -> Self { Self(std::sync::Mutex::new(value)) }
    fn lock(&self) -> std::sync::MutexGuard<'_, T> { self.0.lock().unwrap() }
}
mod kernel {
    pub mod bindings { pub const EOPNOTSUPP: u32 = 95; }
    pub mod time { pub fn msecs_to_jiffies(n: u32) -> u32 { n } }
    pub mod sync { pub enum CondVarTimeoutResult { Signal {}, Timeout } }
}
struct CondVar;
impl CondVar {
    fn wait_interruptible_timeout<T>(&self, _: &mut T, _: u32) -> kernel::sync::CondVarTimeoutResult {
        panic!("cleanup tests must not enter a plaintext delivery wait")
    }
}
mod shim { pub fn monotonic_ns() -> u64 { 1 } }
mod sks { pub const SKS_AUTH_TOKEN_LEN: usize = 16; pub const SKS_PERFORM_OP_NAME: &str = "perform"; }
mod profile {
    pub enum KeyStore { Sepos13 {}, Other }
    pub struct PlatformProfile { pub key_store: KeyStore }
}
mod der {
    pub enum RefKeyValue<'a> { Utf8(&'a [u8]), Octets(&'a [u8]), Der(&'a [u8]), Integer(u32) }
    pub fn encode_refkey_set(items: &[(&[u8], RefKeyValue<'_>)]) -> super::Result<super::KVec<u8>> {
        for (name, value) in items {
            if *name == b"o" { if let RefKeyValue::Utf8(op) = value { return Ok(super::bytes(op)); } }
        }
        Err(super::EINVAL)
    }
    pub fn octet_string_body(_: &[u8]) -> Option<&[u8]> { None }
}
mod refkey_policy {
    pub fn biometric_signing_policy(_: &[u8; 16]) -> super::Result<super::KVec<u8>> { Ok(super::bytes(b"policy")) }
    pub use biometric_signing_policy as biometric_combined_policy;
}
mod proto {
    pub struct FieldCursor<'a>(&'a [u8]);
    impl<'a> FieldCursor<'a> {
        pub fn new(b: &'a [u8]) -> Self { Self(b) }
        pub fn i32(&mut self) -> Option<i32> { Some(0) }
        pub fn blob(&mut self) -> Option<&'a [u8]> { Some(self.0) }
    }
}
mod keybag {
    pub enum Slot { Identity }
    pub enum State { Present(Stored), Absent }
    pub struct Stored;
    impl Stored { pub fn secret(&self) -> &[u8] { b"synthetic bag secret" } }
    pub fn read(_: Slot) -> super::Result<State> { Ok(State::Present(Stored)) }
}
mod sbio {
    pub struct UserId(i32);
    impl UserId { pub fn new(n: i32) -> Option<Self> { Some(Self(n)) } pub fn value(&self) -> i32 { self.0 } }
    pub fn sbio_catacomb_id(user: UserId) -> UserId { user }
}
enum SbioOutcome { Ok(KVec<u8>), Failed }
mod bio {
    pub const ATTEST_PUB_LEN: usize = 65;
    pub const ATTEST_SIG_MAX: usize = 72;
    pub enum VerifyOutcome { Matched(()), NoMatch, Failed(u32) }
    pub struct Session {
        pub checks: super::Cell<u32>,
        pub cancel_at_check: u32,
        pub result: Option<super::Result<super::keys::Output>>,
        pub deletes_at_publish: usize,
        pub wipes_at_publish: usize,
    }
    impl Session { pub fn available(&self) -> bool { true } }
    pub mod key {
        use super::super::*;
        pub fn key_is_live(s: &super::Session, _: u64) -> bool {
            let n = s.checks.get() + 1; s.checks.set(n);
            s.cancel_at_check == 0 || n < s.cancel_at_check
        }
        pub fn finish_key(s: &mut super::Session, _: u64, result: Result<keys::Output>) -> bool {
            s.deletes_at_publish = DELETES.load(Relaxed);
            s.wipes_at_publish = WIPES.load(Relaxed);
            s.result = Some(result); true
        }
        pub fn key_plaintext_wait_ns(_: &mut super::Session, _: u64, _: u64) -> Option<u64> { None }
    }
    pub fn abort_capture(_: &mut Session) {}
}
mod key_request {
    use super::*;
    pub struct Request<'a> { pub data: keys::KeyRequest, pub sep: &'a SepData }
    impl Request<'_> {
        pub fn is_live(&self) -> bool { bio::key::key_is_live(&self.sep.bio_session.lock(), 1) }
        pub fn complete(&self, result: Result<keys::Output>) {
            bio::key::finish_key(&mut self.sep.bio_session.lock(), 1, result);
        }
    }
}
#[derive(Clone, Copy, PartialEq)]
enum Fault { None, Externalize, Unload, Binding, Operation }
struct Outcome { reply: Reply, body: KVec<u8> }
struct Reply { status: i32 }
enum Request { Unload, Verify, Refkey(KVec<u8>) }
struct Material { special: i32, secret: KVec<u8> }
struct SepData {
    profile: profile::PlatformProfile,
    key_api_failed: AtomicBool,
    suspending: AtomicBool,
    bio_session: Mutex<bio::Session>,
    key_result_changed: CondVar,
    enrol_material: Mutex<Option<Material>>,
    delete_mode: u8,
    fault: Fault,
    calls: Cell<usize>,
    private_uses: Cell<usize>,
}
struct Prepared<'a>(&'a SepData);
impl Prepared<'_> {
    fn verify(self, _: Option<&[u8; 16]>) -> bio::VerifyOutcome { bio::VerifyOutcome::Matched(()) }
}
impl SepData {
    fn new(delete_mode: u8, fault: Fault, cancel_at_check: u32) -> Self {
        DELETES.store(0, Relaxed); WIPES.store(0, Relaxed);
        Self { profile: profile::PlatformProfile { key_store: profile::KeyStore::Sepos13 {} },
            key_api_failed: AtomicBool::new(false), suspending: AtomicBool::new(false),
            bio_session: Mutex::new(bio::Session { checks: Cell::new(0), cancel_at_check,
                result: None, deletes_at_publish: 0, wipes_at_publish: 0 }), key_result_changed: CondVar,
            enrol_material: Mutex::new(Some(Material { special: 1, secret: bytes(b"synthetic") })),
            delete_mode, fault, calls: Cell::new(0), private_uses: Cell::new(0) }
    }
    fn called(&self) { self.calls.set(self.calls.get() + 1); }
    fn scrd_ready(&self) -> bool { true }
    fn scrd_command(&self, c: &scrd::ScrdCommand) -> Option<(i32, KVec<u8>)> {
        self.called();
        match c.payload()[4] {
            0x02 => { DELETES.fetch_add(1, Relaxed); match self.delete_mode {
                0 => Some((0, bytes(b""))), 1 => Some((1, bytes(b""))), _ => None } },
            0x24 => Some((0, bytes(&[1; 16]))),
            0x13 if self.fault == Fault::Externalize => None,
            0x03 => Some((0, bytes(&1u32.to_le_bytes()))),
            _ => Some((0, bytes(b""))),
        }
    }
    fn sks_recover(&self, _: &keybag::Stored) -> Option<(i32, ())> { self.called(); Some((1, ())) }
    fn sks_prepare_identity_refkey(&self, _: i32, _: &[u8]) -> bool { self.called(); true }
    fn sks_req_unload_keybag(&self, _: i32) -> Result<Request> { Ok(Request::Unload) }
    fn sks_req_verify_secret(&self, _: i32, _: &[u8], _: &[u8]) -> Result<Request> { Ok(Request::Verify) }
    fn sks_req_refkey(&self, _: i32, body: &[u8]) -> Result<Request> { Ok(Request::Refkey(bytes(body))) }
    fn sks_send(&self, request: Result<Request>) -> Option<Outcome> {
        self.called();
        let request = request.ok()?;
        if matches!(request, Request::Unload) && self.fault == Fault::Unload { return None; }
        if let Request::Refkey(op) = request {
            if &*op != b"oc" { self.private_uses.set(self.private_uses.get() + 1); }
            if self.fault == Fault::Operation { return Some(Outcome { reply: Reply { status: -8 }, body: bytes(b"") }); }
            let body = match &*op { b"oc" => bytes(b"wrapped"), b"osgn" => bytes(&[0x30, 0]), b"oecd" => bytes(&[0x7a; 32]), _ => panic!("unexpected op") };
            return Some(Outcome { reply: Reply { status: 0 }, body });
        }
        Some(Outcome { reply: Reply { status: 0 }, body: bytes(b"") })
    }
    fn sks_report_response<'a>(&self, _: &str, out: &'a Outcome) -> Option<&'a [u8]> { Some(&out.body) }
    fn sbio_call(&self, _: &sbio::UserId) -> SbioOutcome {
        self.called(); SbioOutcome::Ok(bytes(if self.fault == Fault::Binding { &[1; 15] } else { &[1; 16] }))
    }
    fn refkey_pub(_: &[u8]) -> Option<&'static [u8]> { Some(&[4; 65]) }
    fn prepare_verify_capture(&self) -> std::result::Result<Prepared<'_>, u32> { self.called(); Ok(Prepared(self)) }
    fn bio_wake(&self) {}
    fn xart_writable(&self) -> bool { true }
}
'''

TESTS = r'''
#[test]
fn cleanup_failure_suppresses_success_and_latches() {
    for operation in 0..3 {
        for delete_mode in 0..3 {
            let sep = SepData::new(delete_mode, Fault::None, 0);
            let input = match operation {
                0 => keys::Input::Create(0),
                1 => keys::Input::Sign([0; 32]),
                _ => keys::Input::Decrypt(bytes(b"sealed")),
            };
            let success = {
                sep.run_key_operation(&key_request::Request { sep: &sep, data: keys::KeyRequest { blob: bytes(b"wrapped"),
                    input, deadline_ns: u64::MAX } });
                let mut session = sep.bio_session.lock();
                assert_eq!(session.deletes_at_publish, 1, "published before disposal");
                if delete_mode != 0 {
                    assert!(matches!(session.result, Some(Err(EIO))));
                    if operation == 2 { assert_eq!(session.wipes_at_publish, 1, "plaintext not wiped before error publication"); }
                }
                session.result.take().unwrap().is_ok()
            };
            assert_eq!(DELETES.load(Relaxed), 1, "context must be deleted exactly once");
            assert_eq!(success, delete_mode == 0);
            if operation == 2 { assert_eq!(WIPES.load(Relaxed), 1); }
            if delete_mode != 0 {
                assert!(!sep.biometric_keys_supported());
                let calls = sep.calls.get();
                assert!(matches!(sep.start_gate(), Err(Error(-95))));
                assert!(matches!(sep.create_biometric_key(0), Err(Error(-95))));
                assert!(matches!(sep.key_credential(), Err(Error(-95))));
                assert_eq!(sep.calls.get(), calls, "poisoned operations reached firmware");
            }
        }
    }
}

#[test]
fn early_exits_keep_fallback_cleanup() {
    for fault in [Fault::Externalize, Fault::Unload, Fault::Binding, Fault::Operation] {
        let sep = SepData::new(0, fault, 0);
        assert!(sep.create_biometric_key(0).is_err());
        assert_eq!(DELETES.load(Relaxed), 1);
    }
    for decrypt in [false, true] {
        let sep = SepData::new(0, Fault::Operation, 0);
        sep.run_key_operation(&key_request::Request { sep: &sep, data: keys::KeyRequest { blob: bytes(b"wrapped"), input: if decrypt {
            keys::Input::Decrypt(bytes(b"sealed"))
        } else { keys::Input::Sign([0; 32]) }, deadline_ns: u64::MAX } });
        assert_eq!(DELETES.load(Relaxed), 1);
        assert!(matches!(sep.bio_session.lock().result, Some(Err(EACCES))));
    }
    // Cancel after context creation and after matching, before private-key use.
    for cancel_at_check in [3, 4] {
        for delete_mode in 0..3 {
            let sep = SepData::new(delete_mode, Fault::None, cancel_at_check);
            sep.run_key_operation(&key_request::Request { sep: &sep, data: keys::KeyRequest { blob: bytes(b"wrapped"), input: keys::Input::Sign([0; 32]), deadline_ns: u64::MAX } });
            assert_eq!(DELETES.load(Relaxed), 1);
            // At this worker boundary cleanup failure may take precedence as
            // EIO. The real session-generation cancellation tests are separate.
            let session = sep.bio_session.lock();
            assert!(matches!(session.result, Some(Err(ECANCELED | EIO))));
            assert_eq!(sep.private_uses.get(), 0, "private-key operation after cancellation");
            assert_eq!(sep.key_api_failed.load(Relaxed), delete_mode != 0);
        }
    }
}
'''


class CredentialCleanup(unittest.TestCase):
    @unittest.skipUnless(shutil.which('rustc'), 'requires rustc')
    def test_driver_cleanup_paths(self):
        keys = (APPLE / 'keys.rs').read_text()
        sep = (APPLE / 'sep.rs').read_text()
        scrd = (APPLE / 'scrd.rs').read_text()
        image = (APPLE / 'image.rs').read_text()
        code = HARNESS
        code += '\nmod scrd { use super::*; use crate::sks::SKS_AUTH_TOKEN_LEN;\n'
        code += scrd[scrd.index('pub(crate) const SCRD_ACM_HANDLE_LEN'):scrd.index('pub(crate) fn decode_scrd')]
        code += '\n}\nmod image { use super::*;\n'
        code += function(image, 'wipe').replace('fn wipe(', 'fn production_wipe(', 1)
        code += '''
        pub fn wipe(bytes: &mut [u8]) {
            let plaintext = bytes == [0x7a; 32];
            production_wipe(bytes);
            if plaintext { assert!(bytes.iter().all(|b| *b == 0)); WIPES.fetch_add(1, Relaxed); }
        }
        }\n'''
        code += sep[sep.index('pub(crate) struct Secret'):sep.index('#[derive(Clone, Copy)]\nstruct Abandoned')]
        code += '\nmod keys { use super::*; use crate::der::RefKeyValue as V;\n'
        code += keys[keys.index('pub(crate) struct Credential'):keys.index('impl SepData {')]
        code += keys[keys.index('pub(crate) struct Signature'):keys.index('impl SepData {', keys.index('pub(crate) struct Signature'))]
        code += 'pub const MAX_BLOB: usize = 4096;\nimpl SepData {\n'
        for name in ('biometric_keys_supported', 'key_credential', 'application_refkey_op',
                     'create_biometric_key', 'sign_biometric_key', 'decrypt_biometric_key', 'run_key_operation'):
            code += function(keys, name) + '\n'
        code += function(sep, 'establish_passcode_validated_context')
        # Exercise the new node's actual failure-latch gate too.
        device = (APPLE / 'key_device.rs').read_text()
        code += '\npub(crate) fn start_gate(&self) -> Result { key_device::File { sep: self }.check_api() }\n}\n}\n'
        code += "mod key_device {use super::*; pub struct File<'a>{pub sep:&'a SepData} impl File<'_>{\n"
        code += function(device, 'check_api').replace('fn check_api', 'pub fn check_api', 1)
        code += '} }\n'
        code += TESTS
        with tempfile.TemporaryDirectory(prefix='sep-cleanup-') as directory:
            source = Path(directory) / 'cleanup.rs'
            source.write_text(code)
            binary = Path(directory) / 'cleanup'
            subprocess.run(['rustc', '--edition=2021', '--test', str(source), '-o', str(binary)], check=True)
            # Shared counters observe destruction before publication; serialize cases.
            subprocess.run([str(binary), '--test-threads=1'], check=True)


if __name__ == '__main__':
    unittest.main()
