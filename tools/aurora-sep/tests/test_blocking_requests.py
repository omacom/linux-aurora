# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise production blocking request/file/lease transitions without SEP.

Pin allocation, condvar signals and uaccess are host adapters. Request, file,
capture ownership, result-copy branches and volatile wiping use actual source.
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
    opening = source.index('{', match.start())
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


ADAPTER = r'''
#![allow(dead_code, unused_imports)]
use std::sync::{Arc, atomic::{AtomicBool, AtomicUsize, AtomicU64, Ordering::Relaxed}};
use std::ops::{Deref, DerefMut};
use crate::key_request::Request;
type Error = i32;
type Result<T=()> = std::result::Result<T, Error>;
const EBUSY:i32=-16; const EINVAL:i32=-22; const EIO:i32=-5;
const ECANCELED:i32=-125; const ETIMEDOUT:i32=-110; const EACCES:i32=-13;
const EALREADY:i32=-114; const ENOENT:i32=-2; const ENODATA:i32=-61;
const EOVERFLOW:i32=-75; const ENODEV:i32=-19; const EINTR:i32=-4;
const EFAULT:i32=-14; const ENOSPC:i32=-28;
const ERESTARTSYS:i32=-512; const ERESTARTNOINTR:i32=-513;
const ERESTARTNOHAND:i32=-514; const ERESTART_RESTARTBLOCK:i32=-516;
const GFP_KERNEL:()=();
static CLOCK:AtomicU64=AtomicU64::new(1);
static WIPES:AtomicUsize=AtomicUsize::new(0);
static LOCKED:AtomicUsize=AtomicUsize::new(0);
static COPIES:AtomicUsize=AtomicUsize::new(0);
static FAULT:AtomicUsize=AtomicUsize::new(0);
static ENQUEUED:AtomicUsize=AtomicUsize::new(0);
static QUEUE_FAIL:AtomicBool=AtomicBool::new(false);
const KEY_WORK_ID:u64=5;
struct KVec<T>(Vec<T>);
impl<T> KVec<T>{fn new()->Self{Self(Vec::new())}}
impl<T> Deref for KVec<T>{type Target=[T];fn deref(&self)->&[T]{&self.0}}
impl<T> DerefMut for KVec<T>{fn deref_mut(&mut self)->&mut[T]{&mut self.0}}
struct Mutex<T>(std::sync::Mutex<T>);
struct Guard<'a,T>(Option<std::sync::MutexGuard<'a,T>>);
impl<T> Mutex<T>{
 fn new(value:T)->Self{Self(std::sync::Mutex::new(value))}
 fn lock(&self)->Guard<'_,T>{let guard=self.0.lock().unwrap();LOCKED.fetch_add(1,Relaxed);Guard(Some(guard))}
}
impl<T> Deref for Guard<'_,T>{type Target=T;fn deref(&self)->&T{self.0.as_ref().unwrap()}}
impl<T> DerefMut for Guard<'_,T>{fn deref_mut(&mut self)->&mut T{self.0.as_mut().unwrap()}}
impl<T> Drop for Guard<'_,T>{fn drop(&mut self){LOCKED.fetch_sub(1,Relaxed);}}
enum CondVarTimeoutResult{Signal{},Timeout,Woken{}}
struct CondVar{cv:std::sync::Condvar, signal:AtomicUsize}
impl CondVar{
 fn new()->Self{Self{cv:std::sync::Condvar::new(),signal:AtomicUsize::new(0)}}
 fn notify_all(&self){self.cv.notify_all();}
 fn wait_interruptible_timeout(&self,guard:&mut Guard<'_,key_request::Phase>,ms:u32)->CondVarTimeoutResult{
  match self.signal.swap(0,Relaxed){
   1=>return CondVarTimeoutResult::Signal{},
   2=>{**guard=key_request::Phase::Complete(Ok(keys::Output::Plaintext(Secret(KVec(vec![42])))));return CondVarTimeoutResult::Signal{}},
   _=>{},
  }
  LOCKED.fetch_sub(1,Relaxed);
  let (inner,timed)=self.cv.wait_timeout(guard.0.take().unwrap(),std::time::Duration::from_millis(u64::from(ms.min(100)))).unwrap();
  guard.0=Some(inner);LOCKED.fetch_add(1,Relaxed);
  if timed.timed_out(){CondVarTimeoutResult::Timeout}else{CondVarTimeoutResult::Woken{}}
 }
}
mod shim{pub fn monotonic_ns()->u64{super::CLOCK.load(super::Relaxed)}}
mod kernel{
 pub mod time{pub fn msecs_to_jiffies(ms:u32)->u32{ms}}
 pub mod workqueue{
  pub struct Queue;
  pub fn system_dfl()->Queue{Queue}
  impl Queue{pub fn enqueue<W,const ID:u64>(&self,_:W)->std::result::Result<(),()>{
   use super::super::*;
   assert!(LOCKED.load(Relaxed)>0,"enqueue escaped the removal-serialization lock");
   ENQUEUED.fetch_add(1,Relaxed);
   if QUEUE_FAIL.load(Relaxed){Err(())}else{Ok(())}
  }}
 }
 pub mod uaccess{
  #[derive(Clone,Copy)] pub struct UserPtr;
  impl UserPtr{pub fn from_addr(_:usize)->Self{Self}}
  pub struct UserSlice; pub struct Writer;
  impl UserSlice{pub fn new(_:UserPtr,_:usize)->Self{Self}pub fn writer(&self)->Writer{Writer}}
  impl Writer{
   fn copy(&self)->super::super::Result{use super::super::*;assert_eq!(LOCKED.load(Relaxed),0,"user copy under a state lock");let n=COPIES.fetch_add(1,Relaxed)+1;if FAULT.load(Relaxed)==n{Err(EFAULT)}else{Ok(())}}
   pub fn write<T>(&mut self,_:&T)->super::super::Result{self.copy()}
   pub fn write_slice(&mut self,_:&[u8])->super::super::Result{self.copy()}
  }
 }
}
mod bio{pub const ATTEST_SIG_MAX:usize=72;pub const ATTEST_PUB_LEN:usize=65;}
'''

TESTS = r'''
fn data(deadline_ns:u64)->keys::KeyRequest{
 keys::KeyRequest{blob:KVec::new(),input:keys::Input::Decrypt(KVec::new()),deadline_ns}
}
fn request(id:u64)->Arc<key_request::Request>{key_request::Request::new(id,data(u64::MAX)).unwrap()}
fn plain()->Result<keys::Output>{Ok(keys::Output::Plaintext(Secret(KVec(vec![42]))))}

#[test]
fn cancellation_expiry_interruption_and_cleanup_failures_suppress_delivery(){
 for mode in 0..6{
  CLOCK.store(1,Relaxed);WIPES.store(0,Relaxed);
  let request=request(1);
  match mode{
   0=>{request.complete(plain());request.cancel(ECANCELED).unwrap();},
   1=>{request.cancel(ENODEV).unwrap();request.complete(plain());},
   2=>{request.complete(plain());CLOCK.store(u64::MAX,Relaxed);},
   3=>{request.inject_signal(2);},
   4=>{request.complete(Err(EIO));assert!(matches!(request.cancel(ECANCELED),Err(ENOENT)));CLOCK.store(u64::MAX,Relaxed);},
   5=>{request.inject_signal(1);},
   _=>unreachable!(),
  }
  let expected=match mode{0=>ECANCELED,1=>ENODEV,2=>ETIMEDOUT,3|5=>EINTR,_=>EIO};
  assert!(matches!(request.wait(),Err(e) if e==expected));
  if mode<4{assert_eq!(WIPES.load(Relaxed),1);}
  assert!(!request.is_live());
  request.complete(plain());
  assert!(matches!(request.wait(),Err(e) if e==expected));
 }
 CLOCK.store(1,Relaxed);
}

#[test]
fn concurrent_cancel_is_request_scoped_and_covers_preacceptance(){
 let sep=SepData::new();
 let first=Arc::new(file::new(sep.clone()));let other=file::new(sep);
 // Cancel before descriptor/input copying has registered the request.
 first.cancel(1).unwrap();
 assert!(matches!(first.begin(Some(request(1))),Err(ECANCELED)));
 let req=request(2);let active=first.begin(Some(req.clone())).unwrap();
 assert!(matches!(first.begin(Some(request(3))),Err(EBUSY)));
 assert!(other.cancel(2).is_ok()); // retires only the other file's future ID
 assert!(req.is_live());
 std::thread::scope(|scope|{
  let waiter=scope.spawn(||req.wait());
  first.cancel(2).unwrap();
  assert!(matches!(waiter.join().unwrap(),Err(ECANCELED)));
 });
 drop(active);
 let next=request(3);let _active=first.begin(Some(next.clone())).unwrap();
 assert!(matches!(first.cancel(2),Err(ENOENT)));
 req.complete(plain());assert!(next.is_live());
 next.complete(plain());let result=next.wait().unwrap();
 assert!(matches!(first.cancel(3),Err(EALREADY)));drop(result);
}

#[test]
fn capture_lease_survives_cancel_until_worker_drain(){
 let sep=SepData::new();
 let req=request(1);sep.capture.lock().start_key(req.clone()).unwrap();
 req.cancel(ECANCELED).unwrap();assert!(matches!(capture::Lease::acquire(&sep),Err(EBUSY)));
 let unrelated=request(2);sep.capture.lock().finish_key(&unrelated);
 assert!(matches!(capture::Lease::acquire(&sep),Err(EBUSY)));
 sep.capture.lock().finish_key(&req);
 let lease=capture::Lease::acquire(&sep).unwrap();assert_eq!(sep.running.load(Relaxed),1);
 lease.into_bio(8);assert_eq!(sep.running.load(Relaxed),0);
 sep.capture.lock().finish_bio(7);assert!(matches!(capture::Lease::acquire(&sep),Err(EBUSY)));
 sep.capture.lock().finish_bio(8);
 drop(capture::Lease::acquire(&sep).unwrap());assert_eq!(sep.running.load(Relaxed),0);
 sep.suspending.store(true,Relaxed);assert!(matches!(capture::Lease::acquire(&sep),Err(EBUSY)));
 sep.shutting_down.store(true,Relaxed);assert!(matches!(capture::Lease::acquire(&sep),Err(ENODEV)));
}

#[test]
fn enqueue_failure_and_removal_revoke_without_losing_ownership(){
 CLOCK.store(1,Relaxed);ENQUEUED.store(0,Relaxed);QUEUE_FAIL.store(true,Relaxed);
 let sep=SepData::new();let failed=request(1);
 assert_eq!(SepData::queue_key(sep.clone(),failed.clone()),Err(EIO));
 assert!(matches!(failed.wait(),Err(EIO)));
 drop(capture::Lease::acquire(&sep).unwrap());
 QUEUE_FAIL.store(false,Relaxed);
 let active=request(2);SepData::queue_key(sep.clone(),active.clone()).unwrap();
 sep.shutting_down.store(true,Relaxed);sep.cancel_key(ENODEV);
 assert!(matches!(active.wait(),Err(ENODEV)));
 assert!(sep.capture.lock().key().is_some()); // worker still owns its lease
 assert_eq!(SepData::queue_key(sep.clone(),request(3)),Err(ENODEV));
 assert_eq!(ENQUEUED.load(Relaxed),2);
 sep.capture.lock().finish_key(&active);
}

#[test]
fn faulted_copy_consumes_claim_and_wipes_without_holding_locks(){
 for fault in 0..3{
  WIPES.store(0,Relaxed);COPIES.store(0,Relaxed);FAULT.store(fault,Relaxed);
  let request=request(1);request.complete(plain());let result=request.wait().unwrap();
  let destination=file::decrypt_destination();
  let delivery=file::deliver(result,destination,kernel::uaccess::UserPtr);
  assert_eq!(delivery,if fault==0{Ok(())}else{Err(EFAULT)});
  assert_eq!(WIPES.load(Relaxed),1);
  assert!(matches!(request.wait(),Err(ENODATA)));
 }
}

#[test]
fn worker_errors_cannot_restart_or_look_like_presubmission_contention(){
 CLOCK.store(1,Relaxed);
 for error in [ERESTARTSYS,ERESTARTNOINTR,ERESTARTNOHAND,ERESTART_RESTARTBLOCK,EBUSY]{
  let request=request(1);request.complete(Err(error));
  let expected=if error==EBUSY{EIO}else{EINTR};
  assert!(matches!(request.wait(),Err(e) if e==expected));
  assert!(matches!(request.wait(),Err(e) if e==expected));
 }
}
'''


class BlockingRequests(unittest.TestCase):
    @unittest.skipUnless(shutil.which('rustc'), 'requires rustc')
    def test_request_file_capture_and_delivery(self):
        keys = (APPLE / 'keys.rs').read_text()
        device = (APPLE / 'key_device.rs').read_text()
        request = (APPLE / 'key_request.rs').read_text()
        capture = (APPLE / 'capture.rs').read_text()
        sep = (APPLE / 'sep.rs').read_text()
        code = ADAPTER + '\nmod image {use super::*;\n'
        code += function((APPLE / 'image.rs').read_text(), 'wipe').replace('fn wipe(', 'fn production_wipe(', 1)
        code += '\npub fn wipe(bytes:&mut[u8]){let plain=bytes==[42];production_wipe(bytes);if plain{assert_eq!(bytes,[0]);WIPES.fetch_add(1,Relaxed);}}}\n'
        code += sep[sep.index('pub(crate) struct Secret'):sep.index('#[derive(Clone, Copy)]\nstruct Abandoned')]
        code += '\nmod keys{use super::*;\n'
        code += keys[keys.index('pub(crate) struct Signature'):keys.index('impl SepData {', keys.index('pub(crate) struct Signature'))] + '}\n'
        body = request[request.index('enum Phase'):].replace('enum Phase', 'pub(crate) enum Phase', 1)
        body = body.replace(function(body, 'new'), '''pub(crate) fn new(id:u64,data:KeyRequest)->Result<Arc<Self>>{
            Ok(Arc::new(Self{id,data,phase:Mutex::new(Phase::Pending),changed:CondVar::new()}))
        }
        pub(crate) fn inject_signal(&self,n:usize){self.changed.signal.store(n,Relaxed);}''')
        code += '\nmod key_request{use super::*;use crate::keys::{KeyRequest,Output};\n' + body + '}\n'
        code += '\nmod capture{use super::*;use crate::key_request::Request;\n' + capture[capture.index('enum Owner'):] + '}\n'
        code += '''struct SepData{capture:Mutex<capture::State>,shutting_down:AtomicBool,suspending:AtomicBool,running:AtomicUsize}
        impl SepData{
         fn new()->Arc<Self>{Arc::new(Self{capture:Mutex::new(capture::State::new()),shutting_down:AtomicBool::new(false),suspending:AtomicBool::new(false),running:AtomicUsize::new(0)})}
         fn capture_begin(&self)->bool{if self.suspending.load(Relaxed){false}else{self.running.fetch_add(1,Relaxed);true}}
         fn capture_end(&self){assert!(self.running.fetch_sub(1,Relaxed)>0);}
        }\n'''
        code += 'impl SepData {\n' + function(device, 'queue_key') + '\n' + function(device, 'cancel_key') + '\n}\n'
        code += 'mod file{use super::*;use crate::key_request::Request;use kernel::uaccess::{UserPtr,UserSlice};\n'
        code += device[device.index('#[repr(C)]'):device.index('static_assert!')]
        code += device[device.index('enum Operation'):device.index('impl File {')]
        code += 'impl File {\n' + function(device, 'begin') + '\n' + function(device, 'cancel') + '\n}\n'
        code += '''pub(crate) fn new(sep:Arc<SepData>)->File{File{sep,state:Mutex::new(State{last_id:0,cancelled_through:0,operation:Operation::Idle})}}
        pub(crate) fn decrypt_destination()->Destination{Destination::Decrypt(Decrypt{request_id:1,flags:0,timeout_ms:1,blob_len:1,sealed_len:81,blob_ptr:0,sealed_ptr:0,plain_len:4096,reserved:0,plain_ptr:0})}
        pub(crate) fn deliver(result:keys::Output,destination:Destination,user:UserPtr)->Result{
        '''
        code += device[device.index('        match (result, destination)'):device.index('\nextern "C"')]
        # The source's ioctl/impl closing braces also close this adapter module.
        code = code.replace('struct File {', 'pub(crate) struct File {')
        code = code.replace("struct Active<'a>", "pub(crate) struct Active<'a>")
        code = code.replace('enum Destination {', 'pub(crate) enum Destination {')
        code = re.sub(r'(?m)^struct (Sign|Decrypt) \{', r'pub(crate) struct \1 {', code)
        code = re.sub(r'(?m)^    fn (begin|cancel)\(', r'    pub(crate) fn \1(', code)
        code = re.sub(r'#\[pin_data\]|#\[pin\]', '', code)
        code += TESTS
        with tempfile.TemporaryDirectory(prefix='sep-blocking-') as directory:
            source, binary = Path(directory) / 'request.rs', Path(directory) / 'request'
            source.write_text(code)
            subprocess.run(['rustc', '--edition=2021', '--test', str(source), '-o', str(binary)], check=True)
            subprocess.run([str(binary), '--test-threads=1'], check=True, timeout=10)


if __name__ == '__main__':
    unittest.main()
