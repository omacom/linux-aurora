#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile the production bound-device gate with host locking/refcount adapters."""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
HEADER = r'''
#![allow(dead_code)]
extern crate self as kernel;
mod prelude {
    pub type Result<T=()> = std::result::Result<T,i32>;
    pub const ENODEV:i32=-19;
    pub const EOVERFLOW:i32=-75;
    pub const GFP_KERNEL:()=();
    pub struct KVec<T>(Vec<T>);
    impl<T> KVec<T> { pub const fn new()->Self {Self(Vec::new())} pub fn push(&mut self,v:T,_:())->Result {self.0.push(v);Ok(())} }
    impl<T> std::ops::Deref for KVec<T>{type Target=Vec<T>;fn deref(&self)->&Vec<T>{&self.0}}
    impl<T> std::ops::DerefMut for KVec<T>{fn deref_mut(&mut self)->&mut Vec<T>{&mut self.0}}
}
#[macro_export]
macro_rules! new_mutex {($v:expr $(,$name:expr)?)=>{$crate::sync::Mutex::new($v)};}
#[macro_export]
macro_rules! global_lock { (unsafe(uninit) static $name:ident: Mutex<$ty:ty> = $value:expr;) => { static $name:$crate::sync::Mutex<$ty>=$crate::sync::Mutex::new($value); }; }
#[macro_export]
macro_rules! new_condvar {($name:expr)=>{$crate::sync::CondVar(std::sync::Condvar::new())};}
macro_rules! pin_init {($($t:tt)*)=>{$($t)*};}
mod sync {
    pub use crate::global_lock;
    pub struct Arc<T>(std::sync::Arc<T>);
    impl<T> Clone for Arc<T>{fn clone(&self)->Self{Self(self.0.clone())}}
    impl<T> std::ops::Deref for Arc<T>{type Target=T;fn deref(&self)->&T{&self.0}}
    impl<T> Arc<T>{pub fn pin_init(v:T,_:())->crate::prelude::Result<Self>{Ok(Self(std::sync::Arc::new(v)))} pub fn ptr_eq(a:&Self,b:&Self)->bool{std::sync::Arc::ptr_eq(&a.0,&b.0)}}
    pub struct Mutex<T>(std::sync::Mutex<T>);
    impl<T> Mutex<T>{pub const fn new(v:T)->Self{Self(std::sync::Mutex::new(v))} pub unsafe fn init(&self){} pub fn lock(&self)->Guard<'_,T>{Guard(Some(self.0.lock().unwrap()))}}
    pub struct Guard<'a,T>(Option<std::sync::MutexGuard<'a,T>>);
    impl<T> std::ops::Deref for Guard<'_,T>{type Target=T;fn deref(&self)->&T{self.0.as_ref().unwrap()}}
    impl<T> std::ops::DerefMut for Guard<'_,T>{fn deref_mut(&mut self)->&mut T{self.0.as_mut().unwrap()}}
    pub struct CondVar(pub std::sync::Condvar);
    impl CondVar {pub fn wait<T>(&self,g:&mut Guard<'_,T>){let old=g.0.take().unwrap();g.0=Some(self.0.wait(old).unwrap());} pub fn notify_all(&self){self.0.notify_all();}}
    pub mod aref {
        pub struct ARef<T:'static>(&'static T);
        impl<T> std::ops::Deref for ARef<T>{type Target=T;fn deref(&self)->&T{self.0}}
        impl From<&crate::device::Device<crate::device::Bound>> for ARef<crate::device::Device>{fn from(d:&crate::device::Device<crate::device::Bound>)->Self {
            // The host fixtures leak their devices for the test process.
            Self(unsafe{&*(d as *const _ as *const crate::device::Device)})
        }}
    }
}
mod device {
    pub struct Normal;
    pub struct Bound;
    #[repr(C)]
    pub struct Device<T=Normal>{pub bound:std::sync::atomic::AtomicBool,pub marker:std::marker::PhantomData<T>}
    impl<T> Device<T>{pub fn new()->Self{Self{bound:std::sync::atomic::AtomicBool::new(true),marker:std::marker::PhantomData}}}
    impl Device {pub unsafe fn as_bound(&self)->&Device<Bound>{assert!(self.bound.load(std::sync::atomic::Ordering::SeqCst));unsafe{&*(self as *const _ as *const Device<Bound>)}}}
}
mod driver {pub struct AsahiDrmDevice(pub u64);}
mod binding;
'''
TESTS = r'''
use {binding::*,device::*,driver::AsahiDrmDevice};
use std::{sync::{mpsc,atomic::Ordering},time::Duration};
fn device()->&'static Device<Bound>{Box::leak(Box::new(Device::new()))}
#[test]
fn close_joins_existing_access_and_rejects_future_access() {
    let dev=device();let drm=Box::new(AsahiDrmDevice(1));
    let owner=unsafe{Owner::new(dev,&drm)}.unwrap();
    let gate=lookup(&drm).unwrap();let access=gate.enter().unwrap();let _=access.device();
    let (started_rx,started)= {let (a,b)=mpsc::channel();(b,a)};
    let (done,finished)=mpsc::channel();
    let thread=std::thread::spawn(move||{started.send(()).unwrap();owner.close();done.send(()).unwrap();});
    started_rx.recv().unwrap();
    assert!(finished.recv_timeout(Duration::from_millis(50)).is_err(),"unbind returned with a live device borrow");
    drop(access);finished.recv_timeout(Duration::from_secs(2)).unwrap();thread.join().unwrap();
    dev.bound.store(false,Ordering::SeqCst);
    assert!(gate.enter().is_err());assert!(lookup(&drm).is_err());
}
#[test]
fn rebind_cannot_reopen_an_old_vm_gate() {
    let dev=device();let a=Box::new(AsahiDrmDevice(2));let b=Box::new(AsahiDrmDevice(3));
    let first=unsafe{Owner::new(dev,&a)}.unwrap();let old=lookup(&a).unwrap();
    first.close();let second=unsafe{Owner::new(dev,&b)}.unwrap();
    assert!(old.enter().is_err());assert!(lookup(&a).is_err());
    let new=lookup(&b).unwrap();let access=new.enter().unwrap();let _=access.device();drop(access);second.close();
}
#[test]
fn failed_probe_owner_drop_closes_retained_vm_handles() {
    let dev=device();let drm=Box::new(AsahiDrmDevice(4));
    let owner=unsafe{Owner::new(dev,&drm)}.unwrap();let gate=lookup(&drm).unwrap();drop(owner);
    dev.bound.store(false,Ordering::SeqCst);
    assert!(gate.enter().is_err());assert!(lookup(&drm).is_err());
}
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--mutation', choices=['no-close', 'no-join'])
    args = ap.parse_args()
    work = (args.out or Path(tempfile.mkdtemp(prefix='asahi-binding-'))).resolve()
    work.mkdir(parents=True, exist_ok=True)
    source = (ROOT / 'drivers/gpu/drm/asahi/binding.rs').read_text()
    source = '\n'.join(line for line in source.splitlines()
                       if line.strip() not in ('#[pin_data]', '#[pin]'))
    source = source.replace(' <- ', ': ')
    source = source.replace('// SAFETY: Module initialization initializes this lock before registering adapters.', '')
    if args.mutation == 'no-close':
        source = source.replace('state.live = false;', 'state.live = true;')
    if args.mutation == 'no-join':
        source = source.replace('while state.users != 0 {', 'while false {')
    (work / 'binding.rs').write_text(source)
    (work / 'test.rs').write_text(HEADER + TESTS)
    subprocess.run(['rustc', '--edition=2021', '--test', str(work / 'test.rs'), '-o', str(work / 'test')], check=True)
    subprocess.run([str(work / 'test')], check=True)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as error:
        raise SystemExit(error.returncode if error.returncode > 0 else 128 - error.returncode)
