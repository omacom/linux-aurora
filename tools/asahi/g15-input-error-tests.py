#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Actual G15 job dependency completion and G17 status/policy bodies on host services.

DMA fence lookup, engine submission, queue-fence ordering, VM slot allocation and
kernel mutexes are models. Production job binding/completion and prefix admission
bodies run against them. No kernel scheduling, GPU access or firmware execution
is exercised.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]


def extract(text, marker):
    start = text.index(marker)
    opening = text.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


PRELUDE = r'''
#![allow(dead_code)]
use std::sync::{Arc,Mutex as StdMutex,MutexGuard,atomic::{AtomicBool,AtomicI32,AtomicU32,Ordering}};
#[derive(Copy,Clone,Debug,PartialEq,Eq)] struct Error(i32);
impl Error { fn from_errno(n:i32)->Self {assert!(n<0);Self(n)} fn to_errno(self)->i32{self.0} }
type Result<T=()> = std::result::Result<T,Error>;
const EIO:Error=Error(-5);const ENODEV:Error=Error(-19);const ETIMEDOUT:Error=Error(-110);
const ECANCELED:Error=Error(-125);const EINVAL:Error=Error(-22);const ENOMEM:Error=Error(-12);
const ENOSPC:Error=Error(-28);const EAGAIN:Error=Error(-11);const ENOENT:Error=Error(-2);
macro_rules! pr_warn { ($($arg:tt)*)=>{{ let _=format_args!($($arg)*); }} }
macro_rules! mod_pr_debug { ($($arg:tt)*)=>{{ let _=format_args!($($arg)*); }} }
struct Mutex<T>(StdMutex<T>);impl<T> Mutex<T>{fn lock(&self)->MutexGuard<'_,T>{self.0.lock().unwrap()}}
struct Mirror{word:*mut i32}
struct VmStatus{error:AtomicI32,reported_error:AtomicI32,client_closed:AtomicBool,warned:AtomicBool,mirror:Mutex<Option<Mirror>>}
impl VmStatus {
 // The client's VM handle stays open here, as for every job these checks run.
 fn new(word:*mut i32)->Self{Self{error:AtomicI32::new(0),reported_error:AtomicI32::new(0),client_closed:AtomicBool::new(false),warned:AtomicBool::new(false),mirror:Mutex(StdMutex::new(Some(Mirror{word})))}}
__STATUS__
}
mod g17 {use super::*;__POLICY__}
mod bindings {pub unsafe fn dma_fence_get_status(p:*const i32)->i32{unsafe{*p}}}
struct Fence(i32);impl Fence{fn raw(&self)->*const i32{&self.0}}
struct Binding{slot:u32,drops:Arc<AtomicU32>}
impl Binding{fn slot(&self)->u32{self.slot}}
impl Drop for Binding{fn drop(&mut self){self.drops.fetch_add(1,Ordering::SeqCst);}}
struct Vm{status:VmStatus,refuse_binding:AtomicBool,next_slot:AtomicU32,allocations:AtomicU32,binding_drops:Arc<AtomicU32>}
impl Vm{fn try_bind(&self)->Result<Binding>{
 if self.refuse_binding.load(Ordering::SeqCst){return Err(ENOSPC);}
 self.allocations.fetch_add(1,Ordering::SeqCst);
 Ok(Binding{slot:self.next_slot.load(Ordering::SeqCst),drops:self.binding_drops.clone()})
}}
struct Progress{signaled:AtomicBool,next:AtomicU32,sent:AtomicU32}
impl Progress{fn next(&self)->u32{self.next.load(Ordering::SeqCst)}fn sent(&self)->u32{self.sent.load(Ordering::SeqCst)}}
struct Guard{quarantines:AtomicU32}impl Guard{fn quarantine(&self){self.quarantines.fetch_add(1,Ordering::SeqCst);}}
struct JobFence{vm:Arc<Vm>,signals:AtomicU32,result:AtomicI32,saw_sticky:AtomicI32,saw_report:AtomicI32,saw_mirror:AtomicI32}
impl JobFence{
 fn signal(&self,result:Result){
  self.saw_sticky.store(self.vm.status.get(),Ordering::SeqCst);
  self.saw_report.store(self.vm.status.reported_error(),Ordering::SeqCst);
  let mirror=self.vm.status.mirror.lock();
  self.saw_mirror.store(unsafe{AtomicI32::from_ptr(mirror.as_ref().unwrap().word)}.load(Ordering::SeqCst),Ordering::SeqCst);
  self.result.store(result.err().map_or(0,|e|e.to_errno()),Ordering::SeqCst);
  self.signals.fetch_add(1,Ordering::SeqCst);
 }
}
struct Shared{crashed:AtomicBool,submits:AtomicU32,submit_result:Result}
impl Shared{fn submit(&self,_:Arc<Job>)->Result{self.submits.fetch_add(1,Ordering::SeqCst);self.submit_result}}
struct Job{started:AtomicBool,vm:Arc<Vm>,progress:Progress,_guard:Guard,fence:JobFence,context:AtomicU32,binding:Mutex<Option<Binding>>}
impl Job {__JOB__}
type Completions=Vec<Result>;
fn complete(done:&mut Completions,_job:&Arc<Job>,result:Result){done.push(result);}
struct Flight{claims:(),group:Vec<Arc<Job>>}
impl Flight{fn new(_job:Arc<Job>)->Self{Self{claims:(),group:vec![]}}}
struct Slots{releases:u32}
impl Slots{fn release(&mut self,_claims:&mut ()){self.releases+=1;}}
struct Engine{slot_wait:bool}
struct HostFirmware{engine:Engine,slots:Slots,publish_result:Result,publishes:u32}
impl HostFirmware{
 fn new(result:Result)->Self{Self{engine:Engine{slot_wait:false},slots:Slots{releases:0},publish_result:result,publishes:0}}
 fn publish_prefix_into(&mut self,_job:&Arc<Job>,_flight:&mut Flight)->Result{self.publishes+=1;self.publish_result}
 __PREFIX__
}
fn make_job(own:Option<Error>)->(Arc<Job>,Box<AtomicI32>){
 let mut mirror=Box::new(AtomicI32::new(0));let vm=Arc::new(Vm{status:VmStatus::new((&mut *mirror as *mut AtomicI32).cast()),refuse_binding:AtomicBool::new(false),next_slot:AtomicU32::new(7),allocations:AtomicU32::new(0),binding_drops:Arc::new(AtomicU32::new(0))});
 if let Some(error)=own{vm.status.record(error);}
 let job=Arc::new(Job{started:AtomicBool::new(false),vm:vm.clone(),progress:Progress{signaled:AtomicBool::new(false),next:AtomicU32::new(0),sent:AtomicU32::new(0)},_guard:Guard{quarantines:AtomicU32::new(0)},fence:JobFence{vm,signals:AtomicU32::new(0),result:AtomicI32::new(0),saw_sticky:AtomicI32::new(0),saw_report:AtomicI32::new(0),saw_mirror:AtomicI32::new(0)},context:AtomicU32::new(0),binding:Mutex(StdMutex::new(None))});
 (job,mirror)
}
fn shared(crashed:bool,result:Result)->Arc<Shared>{Arc::new(Shared{crashed:AtomicBool::new(crashed),submits:AtomicU32::new(0),submit_result:result})}
fn n(n:&AtomicI32)->i32{n.load(Ordering::SeqCst)}
fn main(){
 let (unrelated,other_mirror)=make_job(None);
 for input in [ECANCELED,EIO,ENODEV,ETIMEDOUT,EINVAL,ENOMEM]{
  let (job,mirror)=make_job(None);let device=shared(false,Ok(()));
  job.start(&device,&[Fence(1),Fence(input.to_errno()),Fence(-1)]);
  let expected=g17::inherited_error(input).to_errno();
  assert_eq!(device.submits.load(Ordering::SeqCst),0,"errored input must never submit");
  assert_eq!(job.fence.signals.load(Ordering::SeqCst),1,"dependency completion must signal once");
  assert_eq!(n(&job.fence.result),expected,"inherited error mapping");
  assert_eq!(n(&job.fence.saw_report),expected,"report must precede fence signal");
  assert_eq!(n(&job.fence.saw_mirror),expected,"mirror must precede fence signal");
  assert_eq!(n(&job.fence.saw_sticky),0,"healthy dependency consumer admission stays open");
  assert_eq!(job.vm.status.get(),0,"healthy dependency consumer admission stays open");
  assert_eq!(mirror.load(Ordering::SeqCst),expected);
  assert_eq!(job._guard.quarantines.load(Ordering::SeqCst),0,"unpublished job must not quarantine");
  job.finish_dependency(EIO);job.complete(Err(EIO));
  assert_eq!(job.fence.signals.load(Ordering::SeqCst),1,"dependency completion must signal once");
  assert_eq!(job.vm.status.get(),0,"duplicate completion must not poison VM");
  // A later job on the same VM is admitted despite the reported client error.
  let later=Arc::new(Job{started:AtomicBool::new(false),vm:job.vm.clone(),progress:Progress{signaled:AtomicBool::new(false),next:AtomicU32::new(0),sent:AtomicU32::new(0)},_guard:Guard{quarantines:AtomicU32::new(0)},fence:JobFence{vm:job.vm.clone(),signals:AtomicU32::new(0),result:AtomicI32::new(0),saw_sticky:AtomicI32::new(0),saw_report:AtomicI32::new(0),saw_mirror:AtomicI32::new(0)},context:AtomicU32::new(0),binding:Mutex(StdMutex::new(None))});
  later.start(&device,&[Fence(1)]);assert_eq!(device.submits.load(Ordering::SeqCst),1,"successful dependencies after report must submit");
 }
 for own in [EIO,EINVAL]{for input in [ECANCELED,EIO,ENODEV,ETIMEDOUT,EINVAL]{
  let(job,_mirror)=make_job(Some(own));let device=shared(false,Ok(()));job.start(&device,&[Fence(input.to_errno())]);
  assert_eq!(device.submits.load(Ordering::SeqCst),0);assert_eq!(n(&job.fence.result),input.to_errno(),"own-failed VM must retain original dependency errno");
  assert_eq!(job.vm.status.get(),own.to_errno(),"own-failed VM sticky error must remain");
  assert_eq!(n(&job.fence.saw_report),own.to_errno());assert_eq!(n(&job.fence.saw_mirror),own.to_errno());
  assert_eq!(job._guard.quarantines.load(Ordering::SeqCst),0);
 }}
 for inputs in [vec![],vec![Fence(1)],vec![Fence(1),Fence(1)]]{
  let(job,_mirror)=make_job(None);let device=shared(false,Ok(()));job.start(&device,&inputs);
  assert_eq!(device.submits.load(Ordering::SeqCst),1,"successful dependencies must submit");assert_eq!(job.fence.signals.load(Ordering::SeqCst),0);assert_eq!(job.vm.status.get(),0);
 }
 // Preserve device-crash and submit-refusal sticky semantics.
 for (crashed,result) in [(true,Ok(())),(false,Err(ENODEV)),(false,Err(EINVAL))]{
  let(job,_mirror)=make_job(None);let device=shared(crashed,result);job.start(&device,&[Fence(1)]);
  let expected=if crashed||result==Err(ENODEV){EIO}else{EINVAL};assert_eq!(job.vm.status.get(),expected.to_errno());assert_eq!(n(&job.fence.saw_mirror),expected.to_errno());
 }
 // An engine-owned fault still quarantines before completion.
 let(job,_mirror)=make_job(None);job.complete(Err(EIO));assert_eq!(job._guard.quarantines.load(Ordering::SeqCst),1);assert_eq!(job.vm.status.get(),EIO.to_errno());
 // Actual bind/release bodies: a job keeps its slot until explicitly released.
 {
  let(job,_mirror)=make_job(None);
  job.vm.refuse_binding.store(true,Ordering::SeqCst);
  assert_eq!(job.bind(),Err(ENOSPC));assert!(job.binding.lock().is_none());
  assert_eq!(job.vm.allocations.load(Ordering::SeqCst),0);
  job.vm.refuse_binding.store(false,Ordering::SeqCst);
  job.bind().unwrap();assert_eq!(job.slot(),7,"binding must publish its slot");
  job.vm.next_slot.store(9,Ordering::SeqCst);
  job.bind().unwrap();assert_eq!(job.slot(),7,"repeated bind must keep existing binding");
  assert_eq!(job.vm.allocations.load(Ordering::SeqCst),1);
  job.release_binding();job.release_binding();
  assert_eq!(job.vm.binding_drops.load(Ordering::SeqCst),1,"release_binding must drop exactly once");
  job.bind().unwrap();assert_eq!(job.slot(),9,"released job may acquire a different slot");
  assert_eq!(job.vm.allocations.load(Ordering::SeqCst),2);
 }
 // An engine failure must not recycle an ASID while retained job references live.
 for result in [Ok(()),Err(EIO),Err(ETIMEDOUT),Err(ECANCELED)]{
  let(job,_mirror)=make_job(None);job.bind().unwrap();let drops=job.vm.binding_drops.clone();
  job.complete(result);
  if result.is_err(){
   assert!(job.binding.lock().is_some(),"failed engine completion must retain binding");
   assert_eq!(drops.load(Ordering::SeqCst),0);
   job.complete(Ok(()));assert!(job.binding.lock().is_some(),"duplicate completion must retain failed binding");
   let retained=job.clone();drop(job);
   assert_eq!(drops.load(Ordering::SeqCst),0,"a retained reference must keep the binding");
   drop(retained);assert_eq!(drops.load(Ordering::SeqCst),1,"last reference releases failed binding");
  }else{
   assert!(job.binding.lock().is_none(),"successful engine completion releases binding");
   assert_eq!(drops.load(Ordering::SeqCst),1);drop(job);assert_eq!(drops.load(Ordering::SeqCst),1);
  }
 }
 // Run the real prefix admission wrapper; only firmware publication is modeled.
 {
  let(job,_mirror)=make_job(None);let mut fw=HostFirmware::new(Ok(()));let mut done=vec![];
  job.vm.refuse_binding.store(true,Ordering::SeqCst);
  assert_eq!(fw.publish_prefix(&job,&mut done),Err(EAGAIN));
  assert!(fw.engine.slot_wait,"slot exhaustion must request retry");
  assert_eq!(fw.publishes,0);assert!(done.is_empty());assert_eq!(job.vm.status.get(),0);
  job.vm.refuse_binding.store(false,Ordering::SeqCst);
  assert_eq!(fw.publish_prefix(&job,&mut done),Ok(()));assert_eq!(fw.publishes,1);
 }
 for published in [false,true]{
  let(job,_mirror)=make_job(None);job.bind().unwrap();
  job.progress.sent.store(u32::from(published),Ordering::SeqCst);
  let mut fw=HostFirmware::new(Err(ENOMEM));let mut done=vec![];
  assert_eq!(fw.publish_prefix(&job,&mut done),Err(ENOMEM));
  assert_eq!(fw.slots.releases,1);assert!(done.is_empty());
  if published{
   assert!(job.binding.lock().is_some(),"published prefix must retain binding on append failure");
   assert_eq!(job.vm.binding_drops.load(Ordering::SeqCst),0);
  }else{
   assert!(job.binding.lock().is_none(),"unpublished failure must release binding");
   assert_eq!(job.vm.binding_drops.load(Ordering::SeqCst),1);
  }
 }
 {
  let(job,_mirror)=make_job(None);let mut fw=HostFirmware::new(Err(EIO));let mut done=vec![];
  assert_eq!(fw.publish_prefix(&job,&mut done),Err(EIO));
  assert!(job.binding.lock().is_some(),"firmware-visible failure must retain binding");
  assert_eq!(fw.slots.releases,0);
 }
 assert_eq!(unrelated.vm.status.get(),0,"unrelated VM untouched");assert_eq!(unrelated.vm.status.reported_error(),0);assert_eq!(other_mirror.load(Ordering::SeqCst),0);assert_eq!(unrelated.fence.signals.load(Ordering::SeqCst),0);
 println!("PASS: production job/prefix/status bodies: input errors, binding reuse/release/retention, exhaustion retry, published-prefix lifetime and isolation");
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mutations", action="store_true")
    parser.add_argument("--ref", help="Read job.rs at a public git revision for a removed-fix control")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    job_path = "drivers/gpu/drm/asahi/g15/job.rs"
    job = (subprocess.check_output(["git", "show", args.ref + ":" + job_path], cwd=ROOT, text=True)
           if args.ref else (ROOT / job_path).read_text())
    current = (ROOT / job_path).read_text()
    status = (ROOT / "drivers/gpu/drm/asahi/g17/status.rs").read_text()
    policy = (ROOT / "drivers/gpu/drm/asahi/g17/dependency.rs").read_text()
    engine = (ROOT / "drivers/gpu/drm/asahi/g15/engine.rs").read_text()
    methods = [extract(job, "    pub(crate) fn complete("), extract(job, "    fn finish("),
               extract(job, "    fn start(self: &Arc<Self>")]
    # The old control needs the current helper for the duplicate-completion test;
    # it still runs its actual unmodified start, which reproduces the lost input error.
    methods.append(extract(job if "fn finish_dependency(" in job else current, "    fn finish_dependency("))
    # Historical removed-fix controls predate publication bindings. Keep their
    # actual start/completion bodies and use current binding helpers alongside.
    binding_source = job if "fn release_binding(" in job else current
    methods.extend(extract(binding_source, marker) for marker in [
        "    pub(crate) fn slot(", "    pub(crate) fn bind(",
        "    pub(crate) fn release_binding("])
    bodies = "\n".join(methods)
    code = PRELUDE.replace("__JOB__", bodies).replace("__STATUS__", "\n".join(
        extract(status, marker) for marker in ["    pub(crate) fn get(", "    pub(crate) fn record(",
        "    pub(crate) fn reported_error(", "    pub(crate) fn report_failure(",
        "    fn unobserved_teardown("])).replace(
        "__POLICY__", extract(policy, "pub(crate) fn inherited_error(")).replace(
        "__PREFIX__", extract(engine, "    fn publish_prefix("))
    variants = {"control": code}
    expected = {}
    if args.mutations:
        changes = {
            "healthy-errors-submit": ("if let Some(status) = failed {", "if let Some(status) = failed.filter(|_| self.vm.status.get() != 0) {", "errored input must never submit"),
            "foreign-device-error-unmapped": ("crate::g17::inherited_error(error)", "error", "inherited error mapping"),
            "dependency-records-sticky": ("self.finish_dependency(crate::g17::inherited_error(error));", "self.finish(Err(crate::g17::inherited_error(error)), false);", "healthy dependency consumer admission stays open"),
            "report-after-signal": ("self.vm.status.report_failure(error);\n        self.fence.signal(Err(error));", "self.fence.signal(Err(error));\n        self.vm.status.report_failure(error);", "report must precede fence signal"),
            "dependency-quarantines": ("    fn finish_dependency(&self, error: Error) {", "    fn finish_dependency(&self, error: Error) {\n        self._guard.quarantine();", "unpublished job must not quarantine"),
            "own-error-remapped": ("self.finish(Err(error), false);\n            } else", "self.finish(Err(crate::g17::inherited_error(error)), false);\n            } else", "own-failed VM must retain original dependency errno"),
            "success-refused": ("if let Some(status) = failed {", "if let Some(status) = failed.or(Some(-125)) {", "successful dependencies after report must submit"),
            "dependency-completes-twice": ("    fn finish_dependency(&self, error: Error) {\n        if self.progress.signaled.swap(true, Ordering::AcqRel) {\n            return;\n        }", "    fn finish_dependency(&self, error: Error) {", "dependency completion must signal once"),
            "mirror-not-published": ("unsafe { AtomicI32::from_ptr(mirror.word) }.store(error, Ordering::SeqCst);", "let _ = mirror;", "mirror must precede fence signal"),
            "failed-engine-releases-binding": ("if result.is_ok() || !engine {", "if true {", "failed engine completion must retain binding"),
            "binding-release-leaks": ("drop(self.binding.lock().take());", "let _ = self;", "release_binding must drop exactly once"),
            "bound-job-rebinds": ("if slot.is_none() {", "if true {", "repeated bind must keep existing binding"),
            "slot-exhaustion-not-retried": ("self.engine.slot_wait = true;", "self.engine.slot_wait = false;", "slot exhaustion must request retry"),
            "published-prefix-releases-binding": ("if job.progress.next() == job.progress.sent() {", "if true {", "published prefix must retain binding on append failure"),
        }
        for name, (old, new, message) in changes.items():
            if code.count(old) != 1:
                raise RuntimeError(f"{name}: unique mutation anchor count {code.count(old)}")
            variants[name] = code.replace(old, new)
            expected[name] = message
    variants["control-after"] = code
    receipt = {"scope": __doc__, "ref": args.ref, "sources": {name: hashlib.sha256(text.encode()).hexdigest()
        for name, text in [(job_path, job), ("g15/engine.rs", engine),
                           ("g17/status.rs", status), ("g17/dependency.rs", policy)]}, "variants": {}}
    for name, source in variants.items():
        path = args.output / (name + ".rs"); path.write_text(source)
        binary = args.output / name
        built = subprocess.run([os.environ.get("RUSTC", "rustc"), "--edition=2021", str(path), "-o", str(binary)], capture_output=True, text=True)
        (args.output / (name + ".build.log")).write_text(built.stdout + built.stderr)
        if built.returncode:
            raise RuntimeError(f"{name}: NOBUILD, not a runtime kill: {built.stderr}")
        ran = subprocess.run([str(binary.resolve())], capture_output=True, text=True, timeout=30)
        receipt["variants"][name] = {"exit": ran.returncode, "stdout": ran.stdout, "stderr": ran.stderr,
            "intended_failure": name in expected and expected[name] in ran.stderr}
        if name.startswith("control") and not args.ref and ran.returncode:
            raise RuntimeError(ran.stdout + ran.stderr)
        if name in expected and (ran.returncode == 0 or expected[name] not in ran.stderr):
            raise RuntimeError(f"{name}: did not die at intended runtime assertion: {ran.stderr}")
    (args.output / "result.json").write_text(json.dumps(receipt, indent=2) + "\n")
    if args.ref:
        for name in ["control", "control-after"]:
            result = receipt["variants"][name]
            if result["exit"] == 0 or "errored input must never submit" not in result["stderr"]:
                raise RuntimeError("removed-fix control did not reproduce the intended lost-input failure")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
