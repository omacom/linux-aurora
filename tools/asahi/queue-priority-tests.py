#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile the production queue admission gate with controlled credentials/backends."""
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
s = (root / 'drivers/gpu/drm/asahi/file.rs').read_text()
a = s.index('        if data.flags != 0', s.index('pub(crate) fn queue_create('))
b = s.index('        let queues_xa', a)
gate = s[a:b]
predicate = (root / 'drivers/gpu/drm/asahi/m3/m3_drm.rs').read_text().split('pub(crate) fn supports_high_priority(&self) -> bool {', 1)[1].split('\n    }', 1)[0]
submit = (root / 'drivers/gpu/drm/asahi/m3/m3_submit.rs').read_text()
ordering = submit.split('        let urgency=', 1)[1].split('        Ok(Self', 1)[0]
ordering = 'let urgency=' + ordering
assert 'uapi::drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME - data.priority' in s
unit = r'''
#![allow(dead_code, non_upper_case_globals)]
use std::any::Any;
const EINVAL:i32=22; const EACCES:i32=13;
macro_rules! cls_pr_debug { ($($arg:tt)*) => {} }
mod uapi {
 pub const drm_asahi_priority_DRM_ASAHI_PRIORITY_REALTIME:u32=3;
 pub const drm_asahi_priority_DRM_ASAHI_PRIORITY_HIGH:u32=2;
}
mod bindings { pub const CAP_SYS_NICE:u32=23;
 pub unsafe fn capable(_:u32)->bool { CAP.with(|c|c.get()) }
 thread_local! {pub static CAP:std::cell::Cell<bool>=const {std::cell::Cell::new(false)};}
}
mod g17 {pub struct Gpu;}
mod m3 {pub mod m3_drm {
 #[derive(PartialEq)] pub enum Profile {Legacy, Extended}
 pub struct Soc {pub chip_id:u32}
 pub struct Backend {pub soc:Soc,pub profile:Profile}
 impl Backend {pub fn supports_high_priority(&self)->bool { PREDICATE }}
}}
struct Gpu(Box<dyn Any>); impl Gpu {fn as_any(&self)->&dyn Any {&*self.0}}
struct Reg {gpu:Gpu}
struct Data {flags:u32,priority:u32}
struct File(bool); impl File {fn is_current_master(&self)->bool {self.0}}
fn admit(reg_data:&Reg,data:&Data,file:&File)->Result<(),i32> { GATE Ok(()) }
mod sched {
 #[derive(Debug,PartialEq)] pub enum Priority {Low,Normal,High,Kernel}
}
fn ordering(priority:u32)->Result<(u8,sched::Priority),i32> {
 ORDERING
 Ok((urgency,priority))
}
fn main() {
 use m3::m3_drm::*;
 let mut n=0;
 for backend in 0..5 { for cap in [false,true] { for master in [false,true] {
  let gpu:Box<dyn Any>=match backend {
   0=>Box::new(g17::Gpu),
   1=>Box::new(Backend{soc:Soc{chip_id:0x8122},profile:Profile::Extended}),
   2=>Box::new(Backend{soc:Soc{chip_id:0x8122},profile:Profile::Legacy}),
   3=>Box::new(Backend{soc:Soc{chip_id:0x6030},profile:Profile::Extended}),
   _=>Box::new(())};
  let reg=Reg{gpu:Gpu(gpu)};
  bindings::CAP.with(|c|c.set(cap));
  for flags in [0,1,u32::MAX] { for priority in [0,1,2,3,4,u32::MAX] {
   let expected=if flags!=0 || priority>=3 {Err(EINVAL)}
    else if priority!=2 {Ok(())}
    else if backend!=0 && !(backend==1 && cfg!(CONFIG_DRM_ASAHI_M3_T8122)) {Err(EINVAL)}
    else if cap||master {Ok(())} else {Err(EACCES)};
   assert_eq!(admit(&reg,&Data{flags,priority},&File(master)),expected,
    "backend={backend} cap={cap} master={master} flags={flags} priority={priority}"); n+=1;
  }}
 }}}
 for (uapi,expected) in [(0,sched::Priority::Low),(1,sched::Priority::Normal),(2,sched::Priority::High)] {
  assert_eq!(ordering(3-uapi),Ok((uapi as u8,expected)));
 }
 assert_eq!(ordering(4),Err(EINVAL));
 println!("{n} actual admission cases and LOW/MEDIUM/HIGH host mappings passed");
}
'''.replace('PREDICATE', predicate.replace('super::m3_profile::Profile', 'Profile')).replace('GATE', gate).replace('ORDERING', ordering)
with tempfile.TemporaryDirectory(prefix='queue-priority-') as d:
    p = Path(d) / 'gate.rs'
    p.write_text(unit)
    for enabled in [False, True]:
        subprocess.run(['rustc', '--edition=2021', *(['--cfg', 'CONFIG_DRM_ASAHI_M3_T8122'] if enabled else []), str(p), '-o', str(p.with_suffix(''))], check=True)
        # Without M3 compiled only the G17 backend is admitted.
        subprocess.run([str(p.with_suffix(''))], check=True)
print('M3 enabled/disabled gate compilation passed')
