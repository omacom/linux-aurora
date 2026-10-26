#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Compile actual G17 admission, queue-ID, and unused-graph eligibility methods.

Run with Python and rustc; temporary host binaries are removed after the checks.
"""
from pathlib import Path
import os
import re
import subprocess
import tempfile

DRIVER = Path(__file__).resolve().parents[2] / 'drivers/gpu/drm/asahi'

def admission_tests(root, source):
    s=(source/'g17/queue/admission.rs').read_text();cfg=(source/'hw/t8140.rs').read_text()
    def item(start):
     at=s.index(start);begin=s.index('{',at);depth=1;end=begin+1
     while depth:
      depth+=(s[end]=='{')-(s[end]=='}');end+=1
     return s[at:end]
    constants='\n'.join(re.search(r'pub\(crate\) const '+name+r': [^;]+;',cfg).group(0) for name in ['RENDER_SLOTS','RENDER_PAIRS_PER_OWNER','RENDER_DEPTH'])
    mmu=(source/'mmu.rs').read_text()
    limits='\n'.join(re.search(r'const '+name+r': [^;]+;',mmu).group(0) for name in ['EXECUTION_CONTEXT_MASK','MAX_EXECUTION_CONTEXTS'])
    header='''#![allow(dead_code, unused_macros)]
    type Result<T=()> = std::result::Result<T,i32>;
    mod g17 { pub mod capacity {
        pub enum Site { RenderSlots, LogicalQueues, ComputeReserve, PhysicalQids }
        pub fn refused(_: Site) -> i32 { -28 }
    }}

    const ENOSPC:i32=-28; const EIO:i32=-5;
    macro_rules! resource_exhausted { ($s:literal)=>{ENOSPC}; }
    type KVec<T> = Vec<T>; struct InFlight;
    mod cfg {\n'''+constants+'}\n'+limits+'\n'
    methods=['const fn new','fn exhausted','fn refusal','fn advance','fn take_idle','fn acquire','fn claim_sibling','fn reassign_unused_spare','fn warmable_sibling','fn borrow_warm_sibling','fn cancel_empty_slot','fn release']
    code=header+item('struct State')+'\nimpl State {\n'+re.search(r'    const MASK: u64 = .*;',s).group(0)+'\n'+'\n'.join(item(n) for n in methods)+'\n}\n'+'''
    fn main(){
     if cfg::RENDER_PAIRS_PER_OWNER == 1 {
      let mut state=State::new();
      let mut slots=Vec::new();
      for owner in 1..=u64::from(MAX_EXECUTION_CONTEXTS) {
       let slot=state.acquire(owner).expect("every logical owner must fit");slots.push(slot);
       assert_eq!(state.owners.iter().filter(|id|**id==owner).count(),1);
      }
      for slot in slots {state.release(slot);}
      let mut state=State::new();let first=state.acquire(1).unwrap();
      for _ in 1..cfg::RENDER_DEPTH {assert_eq!(state.acquire(1),Some(first));}
      assert_eq!(state.acquire(1),None);state.release(first);assert_eq!(state.acquire(1),Some(first));
      println!("shared render admission: all58 owners fit; depth16 waits for exact retirement without consuming a spare");
      return;
     }
     let mut state=State::new();
     for owner in 1..=32 {
      assert!(state.refusal(owner).is_ok());
      let slot=state.acquire(owner).expect("existing owner fits");
      state.release(slot);
     }
     assert_eq!(state.active,0,"all submitted work retired");
     let retained=state.owners.iter().filter(|o|**o!=0).count();
     assert_eq!(retained,63);
     assert_eq!(state.refusal(33),Err(ENOSPC));
     assert!(state.acquire(33).is_none());
     println!("confirmed: 32 open idle owners retain all 63 usable pairs; owner33 receives ENOSPC");
     // Yield one unused spare for each additional logical owner. The runtime
     // separately proves that these physical graphs have never been published.
     for owner in 33..=u64::from(MAX_EXECUTION_CONTEXTS) {
      let slot=(1..cfg::RENDER_SLOTS).find(|slot|state.reassign_unused_spare(*slot as u8,None,Some(owner))).unwrap();
      assert_eq!(state.acquire(owner),Some(slot as u8));state.release(slot as u8);
     }
     for owner in 1..=u64::from(MAX_EXECUTION_CONTEXTS) {let slot=state.acquire(owner).unwrap();state.release(slot);}
     assert!(state.owners.iter().all(|owner|*owner<=u64::from(MAX_EXECUTION_CONTEXTS)));
     println!("fixed: all{} logical owners render and resume without reducing slot count or depth",MAX_EXECUTION_CONTEXTS);
     let mut guarded=State::new();let first=guarded.acquire(1).unwrap();guarded.release(first);
     let spare=guarded.owners.iter().enumerate().find(|(s,o)|**o==1&&*s!=first as usize).unwrap().0;
     guarded.take_idle(spare,1);
     assert!(!guarded.reassign_unused_spare(spare as u8,Some(1),Some(2)),"warm borrower excludes reclamation");
     guarded.release(spare as u8);
     assert!(!guarded.reassign_unused_spare(spare as u8,Some(3),Some(2)),"owner mismatch");
     guarded.quarantined |= 1u64<<first;
     assert!(!guarded.reassign_unused_spare(spare as u8,Some(1),Some(2)),"keep another usable pair");
     guarded.quarantined=0;
     assert!(guarded.reassign_unused_spare(spare as u8,Some(1),Some(2)));
     assert!(!guarded.reassign_unused_spare(first,None,Some(2)),"never take an owner's last pair");
     assert!(!guarded.reassign_unused_spare(0,None,Some(3)));
     assert!(!guarded.reassign_unused_spare(64,None,Some(3)));
     guarded.released |= 1u64<<first;
     assert!(guarded.reassign_unused_spare(first,Some(1),None),"closed owner's unused last slot can be freed for compute");
     let second=guarded.acquire(3).unwrap();guarded.release(second);
     let spare3=guarded.owners.iter().enumerate().find(|(s,o)|**o==3&&*s!=second as usize).unwrap().0;
     assert!(guarded.reassign_unused_spare(spare3 as u8,Some(3),Some(2)));
     let fourth=guarded.acquire(4).unwrap();guarded.release(fourth);
     let spare4=guarded.owners.iter().enumerate().find(|(s,o)|**o==4&&*s!=fourth as usize).unwrap().0;
     assert!(!guarded.reassign_unused_spare(spare4 as u8,Some(4),Some(2)),"destination retains at most two channels");
     let mut recency=State::new();recency.owners[9]=1;recency.owners[2]=1;
     recency.take_idle(9,1);let epoch=recency.epochs[9];
     assert_eq!(recency.borrow_warm_sibling(1,9),Some(2));
     assert_eq!(recency.epochs[2],0);assert_eq!(recency.epochs[9],epoch);
     recency.release(2);recency.release(9);
     for _ in 0..32 {let slot=recency.acquire(1).unwrap();assert_eq!(slot,9);recency.release(slot);}
     let a=recency.acquire(1).unwrap();let b=recency.acquire(1).unwrap();assert_ne!(a,b);
     recency.release(b);recency.released|=1u64<<b;
     assert_eq!(recency.borrow_warm_sibling(1,a),None,"never reopen a closed sibling while warming");
     recency.release(a);
     let mut empty=State::new();let slot=empty.acquire(1).unwrap();
     assert!(!empty.cancel_empty_slot(slot,2));
     empty.refs[slot as usize]+=1;assert!(!empty.cancel_empty_slot(slot,1));empty.release(slot);
     assert!(empty.cancel_empty_slot(slot,1));empty.release(slot);
     assert_eq!(empty.owners[slot as usize],0);assert_eq!(empty.active&(1u64<<slot),0);
     println!("recency: serial work keeps its used pair; overlapping work retains both channels; warm close and empty-reservation races protected");
     println!("guards: live warming, expected owner, remaining usable pair, existing destination, and reserved slots");
     let mut state=State::new();
     for owner in 1..=32 {let slot=state.acquire(owner).unwrap();state.release(slot);}
     // Every existing owner still resumes, so this is held capacity, not live GPU work.
     for owner in 1..=32 {let slot=state.acquire(owner).unwrap();state.release(slot);}
     // Logical close makes the retained pair eligible; no firmware behavior modeled.
     let slot=state.owners.iter().position(|o|*o==1).unwrap();
     state.released |= 1u64<<slot;
     assert!(state.refusal(33).is_ok());
     assert_eq!(state.acquire(33),Some(slot as u8));
     println!("control: explicitly released idle ownership admits a new owner");
    }
    '''
    fixture=root/'admission-capacity-fixed-controls.rs';fixture.write_text(code)
    subprocess.run([os.environ.get('RUSTC', 'rustc'),'--edition=2021',str(fixture),'-o',str(root/'admission-capacity-fixed-controls')],check=True)
    subprocess.run([str(root/'admission-capacity-fixed-controls')],check=True)

def queue_tests(root, source):
    source = source / "g17"
    def item(s,start):
     at=s.index(start);begin=s.index('{',at);depth=1;end=begin+1
     while depth:
      depth+=(s[end]=='{')-(s[end]=='}');end+=1
     return s[at:end]
    s=(source/'kick.rs').read_text()
    fw=(source/'fw/kick.rs').read_text()
    constants=re.search(r'const QID_MAX: [^;]+;',fw).group(0)+'\n'+'\n'.join(re.search(r'const '+name+r': [^;]+;',s).group(0) for name in ['QID_COUNT','RENDER_HEADROOM','FULL_RENDER_RESERVE'])
    header='''#![allow(dead_code, unused_macros)]
    type Result<T=()> = std::result::Result<T,i32>;
    mod g17 { pub mod capacity {
        pub enum Site { RenderSlots, LogicalQueues, ComputeReserve, PhysicalQids }
        pub fn refused(_: Site) -> i32 { -28 }
    }}

    mod mmu { pub const MAX_EXECUTION_CONTEXTS:u32=58; }
    const ENOSPC:i32=-28; const EINVAL:i32=-22; const EOVERFLOW:i32=-75;
    macro_rules! resource_exhausted { ($s:literal)=>{ENOSPC}; }
    #[derive(Clone,Copy,PartialEq,Eq,Debug)] enum DataMaster {Compute,Tiling,Fragment}
    #[derive(Clone,Copy,PartialEq,Eq,Debug)] struct Id {qid:u8,owner:u64,engine:DataMaster}
    #[derive(Clone,Copy)] struct Identity {id:Id,published:bool}
    '''+constants+'\n'+item(s,'const fn render_reserve(')+'\n'+item(s,'pub(crate) struct QueueIds')+'\nimpl QueueIds {\n'+''.join(item(s,'pub(crate) fn '+name+'(')+'\n' for name in ['reserve_initial_render','reserve','reserve_for','publish','can_cancel_unpublished','cancel_unpublished'])+''.join(item(s, sig)+'\n' for sig in ['fn render_ids(', 'fn compute_room('])+'}\n'
    test='''
    fn fresh()->QueueIds {QueueIds {entries:[None;QID_COUNT],initial_render:[None;2],leased:0,floor:0}}
    fn main(){
     for floor in [0,32] {
     for render in [0,2,16,32,80,96,112,126].into_iter().filter(|render|*render<=QID_COUNT-floor) {
      let mut q=fresh();q.floor=floor;
      for i in 0..render {q.reserve(i as u64+1,if i%2==0 {DataMaster::Tiling}else{DataMaster::Fragment},None).unwrap();}
      let mut compute=0;
      while q.reserve(1000+compute,DataMaster::Compute,None).is_ok(){compute+=1;}
      assert_eq!(compute as usize,QID_COUNT-floor-render.max(FULL_RENDER_RESERVE-floor));
      assert!(q.entries[..floor].iter().all(Option::is_none));
     }
     }
     let mut q=fresh();let first=q.reserve(1,DataMaster::Tiling,None).unwrap();
     assert!(q.can_cancel_unpublished(first));q.publish(first).unwrap();
     assert!(!q.can_cancel_unpublished(first));assert_eq!(q.cancel_unpublished(first),Err(EINVAL));assert_eq!(q.leased,1);
     let unused=q.reserve(2,DataMaster::Fragment,None).unwrap();q.cancel_unpublished(unused).unwrap();assert_eq!(q.leased,1);
     let next=q.reserve(3,DataMaster::Fragment,None).unwrap();assert_eq!(next.qid,unused.qid);
     assert!(!q.can_cancel_unpublished(unused));assert_eq!(q.cancel_unpublished(unused),Err(EINVAL));
     println!("queue IDs: render/compute capacity boundaries at zero and reserved floors; compute-only render reserve retained; published and stale identities protected");
     // Reproduce the hardware snapshot: 124 installed render identities, two
     // installed compute identities, and two initial reservations fill128 IDs.
     let mut q=fresh();q.reserve_initial_render().unwrap();
     assert_eq!(q.reserve_initial_render(),Err(EINVAL));
     let initial=q.initial_render;
     for owner in 1000..1002 {let id=q.reserve(owner,DataMaster::Compute,Some(4)).unwrap();q.publish(id).unwrap();}
     let mut installed=Vec::new();
     for owner in 3..127 {let id=q.reserve(owner,if owner%2==0 {DataMaster::Tiling}else{DataMaster::Fragment},None).unwrap();q.publish(id).unwrap();installed.push(id);}
     assert_eq!(q.leased,QID_COUNT);
     for (n,old) in initial.into_iter().enumerate() {
      let old=old.unwrap();let id=q.reserve(2000+n as u64,DataMaster::Compute,None).unwrap();
      assert_eq!(id.qid,old.qid);assert_eq!(q.leased,QID_COUNT);
      assert!(!q.can_cancel_unpublished(old));assert_eq!(q.cancel_unpublished(old),Err(EINVAL));
      q.publish(id).unwrap();
     }
     assert_eq!(q.reserve(3000,DataMaster::Compute,None),Err(ENOSPC));
     for id in installed {assert!(!q.can_cancel_unpublished(id));assert_eq!(q.entries[id.qid as usize].unwrap().id,id);}
     // A bootstrap token that became published is never replaced.
     let mut q=fresh();q.reserve_initial_render().unwrap();let protected=q.initial_render[0].unwrap();q.publish(protected).unwrap();
     for owner in 3..129 {q.reserve(owner,DataMaster::Tiling,None).unwrap();}
     assert_eq!(q.leased,QID_COUNT);
     assert_eq!(q.reserve(1000,DataMaster::Compute,None).unwrap().qid,1);
     assert_eq!(q.reserve(1001,DataMaster::Compute,None),Err(ENOSPC));
     assert_eq!(q.entries[0].unwrap().id,protected);
     // Replacement must not consume the protected render minimum.
     let mut q=fresh();q.reserve_initial_render().unwrap();
     for owner in 3..=FULL_RENDER_RESERVE as u64 {q.reserve(owner,DataMaster::Tiling,None).unwrap();}
     for owner in 1000..1000+(QID_COUNT-FULL_RENDER_RESERVE) as u64 {q.reserve(owner,DataMaster::Compute,None).unwrap();}
     assert_eq!(q.leased,QID_COUNT);assert_eq!(q.reserve(2000,DataMaster::Compute,None),Err(ENOSPC));
     assert!(q.initial_render.iter().all(Option::is_some));
     // Ordinary free capacity leaves the initialization reservations untouched.
     let mut q=fresh();q.reserve_initial_render().unwrap();let initial=q.initial_render;
     let id=q.reserve(1000,DataMaster::Compute,Some(4)).unwrap();assert_eq!(id.qid,4);assert_eq!(q.initial_render,initial);
     println!("initial reservations: full-namespace compute admission restored; physical count unchanged; published and stale identities plus render minimum protected");

    }
    '''
    fixture=root/'queue-id-capacity-controls.rs';fixture.write_text(header+test)
    subprocess.run([os.environ.get('RUSTC', 'rustc'),'--edition=2021',str(fixture),'-o',str(root/'queue-id-capacity-controls')],check=True)
    subprocess.run([str(root/'queue-id-capacity-controls')],check=True)
    s=(source/'queue/render/parked.rs').read_text();guard=item(s,'pub(crate) fn can_yield_unused')
    fixture=root/'unused-render-guard-controls.rs';fixture.write_text('''#![allow(dead_code, unused_macros)]
    type Result<T> = std::result::Result<T,i32>;
    struct Context;
    struct Pair {owner:Option<u64>,ordinal:u64,registered:Option<u8>,outer_started:u8,previous:Option<()>,graph_started:bool,backing:Result<bool>}
    impl Pair {
     fn context(&self)->&Context {&Context}
     fn can_park_backing(&self,_:&Context)->Result<bool>{self.backing}
    '''+guard+'''
    }
    fn fresh()->Pair {Pair {owner:Some(1),ordinal:0,registered:None,outer_started:0,previous:None,graph_started:false,backing:Ok(true)}}
    fn main(){
     assert_eq!(fresh().can_yield_unused(),Ok(true));
     let mut p=fresh();p.owner=None;assert_eq!(p.can_yield_unused(),Ok(true));
     let mut p=fresh();p.ordinal=1;assert_eq!(p.can_yield_unused(),Ok(true));
     let mut p=fresh();p.registered=Some(2);assert_eq!(p.can_yield_unused(),Ok(true));
     let mut p=fresh();p.outer_started=4;assert_eq!(p.can_yield_unused(),Ok(true));
     let mut p=fresh();p.graph_started=true;assert_eq!(p.can_yield_unused(),Ok(false));
     let mut p=fresh();p.backing=Ok(false);assert_eq!(p.can_yield_unused(),Ok(false));
     let mut p=fresh();p.backing=Err(-5);assert_eq!(p.can_yield_unused(),Err(-5));
     println!("unused graph: eight eligibility/error controls protect every publication boundary");
    }
    ''')
    subprocess.run([os.environ.get('RUSTC', 'rustc'),'--edition=2021',str(fixture),'-o',str(root/'unused-render-guard-controls')],check=True)
    subprocess.run([str(root/'unused-render-guard-controls')],check=True)

def lower_window_gate_tests(root, source):
    s=(source/'g17/freelist.rs').read_text()
    start=s.index('static LOWER_FREED');end=s.index('pub(crate) struct Pages {')
    gate=s[start:end]
    fixture=root/'lower-window-gate-controls.rs'
    fixture.write_text('''#![allow(dead_code)]
use std::sync::atomic::{AtomicU64, Ordering};
type Result<T=()> = std::result::Result<T,i32>;
const ENOSPC:i32=-28;const ENOMEM:i32=-12;
static CLOCK: AtomicU64 = AtomicU64::new(1_000_000_000);
fn now_ns()->u64 {CLOCK.load(Ordering::Relaxed)}
mod freelist {
use super::*;
'''+gate+'''
pub(crate) fn freed() {LOWER_FREED.fetch_add(1, Ordering::AcqRel);}
pub(crate) fn place<T>(size: usize, f: impl FnOnce() -> Result<T>) -> Result<T> {place_lower(size, f)}
}
use freelist::*;
fn attempt(size:usize, result:Result<u32>, calls:&mut u32)->Result<u32> {place(size,||{*calls+=1;result})}
fn main(){
 let mut calls=0;
 assert_eq!(attempt(100,Ok(1),&mut calls),Ok(1));assert_eq!(calls,1);assert!(!lower_pressure());
 assert_eq!(attempt(100,Err(ENOSPC),&mut calls),Err(ENOSPC));assert_eq!(calls,2);assert!(lower_pressure());
 for _ in 0..1000 {assert_eq!(attempt(100,Ok(1),&mut calls),Err(ENOSPC));}
 assert_eq!(attempt(200,Ok(1),&mut calls),Err(ENOSPC));
 assert_eq!(calls,2,"a refused size is not placed again while nothing was freed");
 assert_eq!(attempt(50,Err(ENOSPC),&mut calls),Err(ENOSPC));assert_eq!(calls,3,"a smaller size still probes");
 assert_eq!(attempt(60,Ok(1),&mut calls),Err(ENOSPC));assert_eq!(calls,3,"the smaller refusal now gates larger sizes");
 freed();assert!(!lower_pressure());
 assert_eq!(attempt(100,Err(ENOSPC),&mut calls),Err(ENOSPC));assert_eq!(calls,4,"freed backing lets a waiter retry");
 // One probe per interval across all callers, then the gate closes again.
 CLOCK.fetch_add(100_000_000,Ordering::Relaxed);
 assert_eq!(attempt(100,Err(ENOSPC),&mut calls),Err(ENOSPC));assert_eq!(calls,5);
 assert_eq!(attempt(100,Ok(1),&mut calls),Err(ENOSPC));assert_eq!(calls,5);
 CLOCK.fetch_add(100_000_000,Ordering::Relaxed);
 assert_eq!(attempt(100,Ok(7),&mut calls),Ok(7));assert_eq!(calls,6);assert!(!lower_pressure(),"a fitting placement clears pressure");
 assert_eq!(attempt(100,Err(ENOMEM),&mut calls),Err(ENOMEM));assert!(!lower_pressure(),"only window refusals gate");
 println!("lower window: refused free-list placements are not repeated until backing is freed or one probe per 100 ms");
}
''')
    subprocess.run([os.environ.get('RUSTC','rustc'),'--edition=2021',str(fixture),'-o',str(root/'lower-window-gate-controls')],check=True)
    subprocess.run([str(root/'lower-window-gate-controls')],check=True)

def watchdog_publication_guard_tests(source):
    # A packet still in publication (capacity or graph waits) must never reach
    # pair lookups, the timeout storm count or a collector allocation from its
    # watchdog: each of them can stop the runtime for every client.
    text = (source / 'g17/runtime/backend.rs').read_text()
    at = text.index('    fn timed_out(')
    body = text[at:text.index('\n    }\n', at)]
    guard = body.index('if !packet.is_published() {\n            return sched::Status::NoHang;')
    assert guard < body.index('self.release_backlog(&packet);') < body.index('self.shared.watchdog_timeout()'), 'timed_out'
    for path, name in [('g17/runtime/render/settle.rs', 'pub(in crate::g17) fn timeout_render('),
                       ('g17/runtime/compute_control.rs', 'pub(super) fn timeout_compute(')]:
        text = (source / path).read_text()
        at = text.index(name)
        body = text[at:text.index('\n    }\n', at)]
        assert 'let Ok(mut deferred) = DeferredBatch::worker() else {' in body, name
        assert 'DeferredBatch::worker()?' not in body, name
    print('watchdog: unpublished packets and collector allocation failure never stop the runtime')

def shared_alias_tests(root, source):
    s=(source/'g17/object.rs').read_text()
    at=s.index('    pub(crate) fn lower_with_aliases(');begin=s.index('{',at);end=begin+1;depth=1
    while depth:
        depth+=(s[end]=='{')-(s[end]=='}');end+=1
    method=s[at:end]
    fixture=root/'shared-alias-controls.rs'
    fixture.write_text('''#![allow(dead_code)]
use std::{cell::RefCell,rc::Rc,ops::Range};
type Result<T> = std::result::Result<T,i32>;type Error=i32;
const EINVAL:i32=-22;const EOVERFLOW:i32=-75;const ENOSPC:i32=-28;const EIO:i32=-5;
mod g17 {pub mod capacity {pub enum Site {LowerVa, AliasVa} pub fn refused(_: Site)->i32 {-28}}}
mod hw {pub mod t8140 {pub mod dynamic {pub const LOWER:std::ops::Range<u64>=0..256;}}}
mod mmu {pub const UAT_PGSZ:usize=16;pub type Prot=u8;pub type Vm=super::Vm;pub type KernelMapping=super::Mapping;}
#[derive(Clone,Copy)] enum CpuMap {WriteCombined}
#[derive(Clone)] struct Region {vm:usize,start:u64,size:u64,object:u64}
#[derive(Default)] struct State {regions:Vec<Region>,allocations:u64,attempts:usize,fault:Option<usize>}
struct Vm {id:usize,state:Rc<RefCell<State>>}
struct Uat {lower:Vm}
impl Uat {fn kernel_lower_vm(&self)->&Vm {&self.lower}}
struct Device {state:Rc<RefCell<State>>}
struct Mapping {vm:usize,address:u64,size:u64,object:u64,state:Rc<RefCell<State>>}
impl Mapping {fn iova(&self)->u64 {self.address}}
impl Drop for Mapping {fn drop(&mut self) {
 let mut state=self.state.borrow_mut();
 if self.vm==0 {assert!(!state.regions.iter().any(|r|r.vm!=0&&r.object==self.object&&r.start==self.address),"aliases must retire before canonical reservation");}
 let index=state.regions.iter().position(|r|r.vm==self.vm&&r.start==self.address&&r.object==self.object).unwrap();
 state.regions.remove(index);
}}
fn overlaps(s:&State,vm:usize,start:u64,size:u64)->bool {s.regions.iter().any(|r|r.vm==vm&&r.start<start+size&&start<r.start+r.size)}
fn claim(vm:&Vm,address:u64,size:u64,object:u64)->Result<Mapping> {
 let mut state=vm.state.borrow_mut();
 if state.fault==Some(vm.id) {return Err(EIO)}
 if overlaps(&state,vm.id,address,size) {return Err(ENOSPC)}
 state.regions.push(Region {vm:vm.id,start:address,size,object});
 Ok(Mapping {vm:vm.id,address,size,object,state:vm.state.clone()})
}
struct ObjectRef {id:u64,size:usize}
impl ObjectRef {
 fn map_mode(&self,vm:&Vm,range:Range<u64>,align:u64,guard:bool,lowest:bool)->Result<Mapping> {
  if vm.id==0 {assert!(guard,"canonical guard must be preserved");vm.state.borrow_mut().attempts+=1;}
  let length=self.size as u64+if guard {16}else{0};
  let mut used:Vec<_>=vm.state.borrow().regions.iter().filter(|r|r.vm==vm.id).cloned().collect();used.sort_by_key(|r|r.start);
  let mut cursor=range.start;let mut holes=Vec::new();
  for r in used {
   if r.start>cursor {holes.push(cursor..r.start.min(range.end));}
   cursor=cursor.max(r.start+r.size);
  }
  if cursor<range.end {holes.push(cursor..range.end);}
  let choice=holes.into_iter().filter_map(|h| {
   let address=h.start.div_ceil(align)*align;
   (address+length<=h.end).then_some((if lowest {address}else{h.end-h.start},address))
  }).min();
  match choice {Some((_,address))=>claim(vm,address,length,self.id),None=>Err(ENOSPC)}
 }
 fn map_into_range(&self,vm:&Vm,range:Range<u64>,align:u64,_prot:u8,guard:bool)->Result<Mapping> {
  self.map_mode(vm,range,align,guard,false)
 }
}
struct KernelObject {obj:ObjectRef,mapping:Option<Mapping>}
impl KernelObject {
 fn backing(dev:&Device,size:usize,_cpu:CpuMap)->Result<Self> {let mut s=dev.state.borrow_mut();s.allocations+=1;Ok(Self {obj:ObjectRef {id:s.allocations,size},mapping:None})}
 fn map_alias(&self,vm:&Vm,address:u64,_prot:u8)->Result<Mapping> {claim(vm,address,self.obj.size as u64,self.obj.id)}
 fn alias_in_lowest(&self,vm:&Vm,range:Range<u64>,align:u64,_prot:u8,guard:usize)->Result<Mapping> {self.obj.map_mode(vm,range,align,guard!=0,true)}
}
struct Allocator<'a> {dev:&'a Device,uat:&'a Uat}
impl Allocator<'_> {
'''+method+'''
}
fn case(obstacles:&[(usize,u64,u64)],fault:Option<usize>,size:usize,expected:std::result::Result<u64,i32>,attempts:usize) {
 let state=Rc::new(RefCell::new(State::default()));
 for (vm,start,len) in obstacles {state.borrow_mut().regions.push(Region {vm:*vm,start:*start,size:*len,object:0});}
 state.borrow_mut().fault=fault;
 let dev=Device {state:state.clone()};let uat=Uat {lower:Vm {id:0,state:state.clone()}};
 let first=Vm {id:1,state:state.clone()};let second=Vm {id:2,state:state.clone()};
 let alloc=Allocator {dev:&dev,uat:&uat};
 let result=alloc.lower_with_aliases(size,16,3,CpuMap::WriteCombined,[&first,&second]);
 match (expected,result) {
  (Ok(address),Ok((object,aliases)))=>{
   assert_eq!(object.mapping.as_ref().unwrap().iova(),address);
   assert!(aliases.iter().all(|m|m.iova()==address&&m.object==object.obj.id));
   assert_eq!(state.borrow().regions.iter().filter(|r|r.object!=0).count(),3);
   drop(aliases);drop(object);
  },
  (Err(expected),Err(actual))=>assert_eq!(actual,expected),
  _=>panic!("unexpected common-address placement result"),
 }
 assert_eq!(state.borrow().regions.len(),obstacles.len(),"failed attempts leak no reservations or aliases");
 assert_eq!(state.borrow().allocations,if size==0||size%16!=0 {0}else{1},"all retries reuse one backing");
 assert_eq!(state.borrow().attempts,attempts);
}
fn exhaustive_common_holes() {
 for canonical in 0u32..64 {for global in 0u32..64 {for client in 0u32..64 {
  let masks=[canonical,global,client];
  let available=(0..4).any(|page|canonical&(7<<page)==0&&global&(3<<page)==0&&client&(3<<page)==0);
  let state=Rc::new(RefCell::new(State::default()));
  for (vm,mask) in masks.into_iter().enumerate() {
   state.borrow_mut().regions.push(Region {vm,start:96,size:160,object:0});
   for page in 0..6 {if mask&(1<<page)!=0 {state.borrow_mut().regions.push(Region {vm,start:page*16,size:16,object:0});}}
  }
  let original=state.borrow().regions.len();
  let dev=Device {state:state.clone()};let uat=Uat {lower:Vm {id:0,state:state.clone()}};
  let first=Vm {id:1,state:state.clone()};let second=Vm {id:2,state:state.clone()};
  let alloc=Allocator {dev:&dev,uat:&uat};
  match alloc.lower_with_aliases(32,16,3,CpuMap::WriteCombined,[&first,&second]) {
   Ok((object,aliases))=>{
    assert!(available,"allocator found a false common hole");
    let address=object.mapping.as_ref().unwrap().iova();assert_eq!(address%16,0);let page=address/16;assert!(page<4);
    assert!(canonical&(7<<page)==0&&global&(3<<page)==0&&client&(3<<page)==0);
    assert!(aliases.iter().all(|m|m.iova()==address&&m.object==object.obj.id));
    drop(aliases);drop(object);
   },
   Err(error)=>{assert_eq!(error,ENOSPC);assert!(!available,"allocator skipped a common hole");},
  }
  assert_eq!(state.borrow().allocations,1);assert_eq!(state.borrow().regions.len(),original);
 }}}
 println!("shared aliases: all262144 three-VM occupancy combinations match the independent common-hole oracle");
}
fn main() {
 case(&[],None,64,Ok(0),1);
 case(&[(1,0,16)],None,64,Ok(16),2);
 case(&[(2,0,16)],None,64,Ok(16),2);
 case(&[(1,0,16),(2,64,16)],None,64,Ok(80),3);
 case(&[(0,0,16)],None,64,Ok(16),1);
 case(&[(2,0,256)],None,64,Err(ENOSPC),1);
 case(&[(2,0,176)],None,64,Ok(176),2);
 case(&[],Some(1),64,Err(EIO),1);
 case(&[],Some(2),64,Err(EIO),1);
 case(&[],Some(0),64,Err(EIO),1);
 case(&[],None,256,Err(ENOSPC),1);
 case(&[],None,0,Err(EINVAL),0);
 case(&[],None,15,Err(EINVAL),0);
 // Best-fit initially selects the small late hole, occupied in the client.
 // The larger earlier canonical hole must remain eligible after that collision.
 case(&[(0,128,16),(2,144,112)],None,64,Ok(0),2);
 case(&[(1,0,256)],None,64,Err(ENOSPC),1);
 case(&[(0,128,128),(2,0,128)],None,64,Err(ENOSPC),2);
 println!("shared aliases: 16 best-fit, collision, rollback, lifetime, backing-reuse, guard and error controls pass");
 exhaustive_common_holes();
}
''')
    subprocess.run([os.environ.get('RUSTC','rustc'),'--edition=2021',str(fixture),'-o',str(root/'shared-alias-controls')],check=True)
    subprocess.run([str(root/'shared-alias-controls')],check=True)

if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='asahi-resource-tests-') as temporary:
        admission_tests(Path(temporary), DRIVER)
        queue_tests(Path(temporary), DRIVER)
        lower_window_gate_tests(Path(temporary), DRIVER)
        watchdog_publication_guard_tests(DRIVER)
        shared_alias_tests(Path(temporary), DRIVER)
