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
    const ENOSPC:i32=-28; const EIO:i32=-5;
    macro_rules! resource_exhausted { ($s:literal)=>{ENOSPC}; }
    type KVec<T> = Vec<T>; struct InFlight;
    mod cfg {\n'''+constants+'}\n'+limits+'\n'
    methods=['const fn new','fn exhausted','fn refusal','fn advance','fn take_idle','fn acquire','fn claim_sibling','fn reassign_unused_spare','fn forfeit_sibling','fn warmable_sibling','fn borrow_warm_sibling','fn cancel_empty_slot','fn release']
    code=header+item('struct State')+'\nimpl State {\n'+re.search(r'    const MASK: u64 = .*;',s).group(0)+'\n'+'\n'.join(item(n) for n in methods)+'\n}\n'+'''
    fn main(){
     let mut state=State::new();
     for owner in 1..=32 {
      assert!(state.refusal(owner).is_ok());
      let slot=state.acquire(owner,None).expect("existing owner fits");
      state.release(slot);
     }
     assert_eq!(state.active,0,"all submitted work retired");
     let retained=state.owners.iter().filter(|o|**o!=0).count();
     assert_eq!(retained,63);
     assert_eq!(state.refusal(33),Err(ENOSPC));
     assert!(state.acquire(33,None).is_none());
     println!("confirmed: 32 open idle owners retain all 63 usable pairs; owner33 receives ENOSPC");
     // Yield one unused spare for each additional logical owner. The runtime
     // separately proves that these physical graphs have never been published.
     for owner in 33..=u64::from(MAX_EXECUTION_CONTEXTS) {
      let slot=(1..cfg::RENDER_SLOTS).find(|slot|state.reassign_unused_spare(*slot as u8,None,Some(owner))).unwrap();
      assert_eq!(state.acquire(owner,None),Some(slot as u8));state.release(slot as u8);
     }
     for owner in 1..=u64::from(MAX_EXECUTION_CONTEXTS) {let slot=state.acquire(owner,None).unwrap();state.release(slot);}
     assert!(state.owners.iter().all(|owner|*owner<=u64::from(MAX_EXECUTION_CONTEXTS)));
     println!("fixed: all{} logical owners render and resume without reducing slot count or depth",MAX_EXECUTION_CONTEXTS);
     let mut guarded=State::new();let first=guarded.acquire(1,None).unwrap();guarded.release(first);
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
     let second=guarded.acquire(3,None).unwrap();guarded.release(second);
     let spare3=guarded.owners.iter().enumerate().find(|(s,o)|**o==3&&*s!=second as usize).unwrap().0;
     assert!(guarded.reassign_unused_spare(spare3 as u8,Some(3),Some(2)));
     let fourth=guarded.acquire(4,None).unwrap();guarded.release(fourth);
     let spare4=guarded.owners.iter().enumerate().find(|(s,o)|**o==4&&*s!=fourth as usize).unwrap().0;
     assert!(!guarded.reassign_unused_spare(spare4 as u8,Some(4),Some(2)),"destination retains at most two channels");
     let mut recency=State::new();recency.owners[9]=1;recency.owners[2]=1;
     recency.take_idle(9,1);let epoch=recency.epochs[9];
     assert_eq!(recency.borrow_warm_sibling(1,9),Some(2));
     assert_eq!(recency.epochs[2],0);assert_eq!(recency.epochs[9],epoch);
     recency.release(2);recency.release(9);
     for _ in 0..32 {let slot=recency.acquire(1,None).unwrap();assert_eq!(slot,9);recency.release(slot);}
     let a=recency.acquire(1,None).unwrap();let b=recency.acquire(1,None).unwrap();assert_ne!(a,b);
     // A command ordered after the owner's live render follows it onto its pair.
     let c=recency.acquire(1,Some(a)).unwrap();assert_eq!(c,a,"a dependent render stays on its predecessor's pair");
     recency.release(c);
     recency.release(b);recency.released|=1u64<<b;
     assert_eq!(recency.borrow_warm_sibling(1,a),None,"never reopen a closed sibling while warming");
     recency.release(a);
     let mut empty=State::new();let slot=empty.acquire(1,None).unwrap();
     assert!(!empty.cancel_empty_slot(slot,2));
     empty.refs[slot as usize]+=1;assert!(!empty.cancel_empty_slot(slot,1));empty.release(slot);
     assert!(empty.cancel_empty_slot(slot,1));empty.release(slot);
     assert_eq!(empty.owners[slot as usize],0);assert_eq!(empty.active&(1u64<<slot),0);
     println!("recency: serial work keeps its used pair; overlapping work retains both channels; warm close and empty-reservation races protected");
     println!("guards: live warming, expected owner, remaining usable pair, existing destination, and reserved slots");
     let mut state=State::new();
     for owner in 1..=32 {let slot=state.acquire(owner,None).unwrap();state.release(slot);}
     // Every existing owner still resumes, so this is held capacity, not live GPU work.
     for owner in 1..=32 {let slot=state.acquire(owner,None).unwrap();state.release(slot);}
     // Logical close makes the retained pair eligible; no firmware behavior modeled.
     let slot=state.owners.iter().position(|o|*o==1).unwrap();
     state.released |= 1u64<<slot;
     assert!(state.refusal(33).is_ok());
     assert_eq!(state.acquire(33,None),Some(slot as u8));
     println!("control: explicitly released idle ownership admits a new owner");
     // Live owners whose siblings were published hold every slot: no spare
     // can yield. The runtime proves each candidate pair idle and installed.
     let candidates=|s:&State|->Vec<(u8,u64)> {(1..cfg::RENDER_SLOTS).filter(|slot|s.owners[*slot]!=0).map(|slot|(slot as u8,s.owners[slot])).collect()};
     let mut live=State::new();
     for owner in 1..=32 {let slot=live.acquire(owner,None).unwrap();live.release(slot);}
     assert_eq!(live.refusal(33),Err(ENOSPC));
     let mut gifts=Vec::new();
     for owner in 33..=u64::from(MAX_EXECUTION_CONTEXTS) {
      assert_eq!(live.refusal(owner),Err(ENOSPC));
      let (slot,donor)=live.forfeit_sibling(&candidates(&live),0,owner).expect("an owner with two pairs donates one");
      assert_eq!(live.owners[slot as usize],owner);assert_eq!(live.epochs[slot as usize],0);
      assert_eq!(live.owners.iter().filter(|o|**o==donor).count(),1,"donor keeps exactly one pair");
      assert_eq!(live.acquire(owner,None),Some(slot));live.release(slot);
      gifts.push((slot as usize,donor));
     }
     for owner in 1..=u64::from(MAX_EXECUTION_CONTEXTS) {
      assert!(live.refusal(owner).is_ok());let slot=live.acquire(owner,None).unwrap();live.release(slot);
     }
     println!("forfeit: all{} live logical owners render while idle siblings move; every donor keeps a pair",MAX_EXECUTION_CONTEXTS);
     // A donor never receives another pair while open: neither free capacity
     // that appears later nor the pair it gave away once its recipient closes.
     let (given,donor)=gifts[0];
     let kept=live.owners.iter().position(|o|*o==donor).unwrap();
     let closed=live.owners.iter().position(|o|*o==u64::from(MAX_EXECUTION_CONTEXTS)).unwrap();
     assert_ne!(given,closed,"control: the released pair was not this donor's");
     live.released|=1u64<<closed;
     let slot=live.acquire(donor,None).unwrap();assert_eq!(slot as usize,kept);live.release(slot);
     assert_eq!(live.owners.iter().filter(|o|**o==donor).count(),1,"no sibling after forfeiting");
     live.released|=1u64<<given;
     let slot=live.acquire(donor,None).unwrap();assert_eq!(slot as usize,kept);live.release(slot);
     assert_ne!(live.owners[given],donor,"a donor never takes back the pair it gave away");
     assert!(live.forfeited.contains(&donor),"the record lasts until the donor closes");
     let undonated=(1..=32).find(|o|!live.forfeited.contains(o)&&*o!=donor&&live.owners.iter().filter(|v|**v==*o).count()==1).unwrap();
     let slot=live.acquire(undonated,None).unwrap();live.release(slot);
     assert_eq!(live.owners.iter().filter(|o|**o==undonated).count(),2,"control: an owner that never donated claims the free sibling");
     // Guards: borrowed, active, quarantined, held, released, last pair, mismatch, recipient state.
     let mut g=State::new();
     for owner in 1..=2 {let slot=g.acquire(owner,None).unwrap();g.release(slot);}
     let pairs=|g:&State,o:u64|->Vec<usize> {(0..cfg::RENDER_SLOTS).filter(|s|g.owners[*s]==o).collect()};
     let (a,b)=(pairs(&g,1)[0],pairs(&g,1)[1]);
     g.refs[b]=1;assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,9),None,"borrowed");g.refs[b]=0;
     g.active|=1u64<<b;assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,9),None,"active");g.active=0;
     g.quarantined|=1u64<<b;assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,9),None,"quarantined");g.quarantined=0;
     g.kill_hold|=1u64<<b;assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,9),None,"exit-kill hold");g.kill_hold=0;
     g.released|=1u64<<b;assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,9),None,"closed owners use release");g.released=0;
     g.quarantined|=1u64<<a;assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,9),None,"donor must keep a usable pair");g.quarantined=0;
     assert_eq!(g.forfeit_sibling(&[(b as u8,2)],0,9),None,"pair owner mismatch");
     assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,1),None,"donor cannot receive its own pair");
     assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,0),None);
     let held=pairs(&g,2)[0];
     assert_eq!(g.forfeit_sibling(&[(b as u8,1)],0,2),None,"recipient holding identities");
     let empty:u64=pairs(&g,2).iter().map(|s|1u64<<s).sum();
     assert_eq!(g.forfeit_sibling(&[(b as u8,1)],empty&!(1u64<<held),2),None,"one reservation still holds identities");
     g.refs[held]=1;assert_eq!(g.forfeit_sibling(&[(b as u8,1)],empty,2),None,"recipient reservation borrowed");g.refs[held]=0;
     assert_eq!(g.forfeit_sibling(&[(b as u8,1)],empty,2).map(|(s,_)|s as usize),Some(b),"recipient returns empty reservations");
     assert_eq!(pairs(&g,2),vec![b]);assert_eq!(g.owners[held],0);
     assert_eq!(g.forfeit_sibling(&[(a as u8,1)],0,3),None,"never take a donor's last pair");
     let mut lru=State::new();
     for owner in 1..=2 {let slot=lru.acquire(owner,None).unwrap();lru.release(slot);}
     let one=pairs(&lru,1);let two=pairs(&lru,2);
     lru.epochs[one[0]]=7;lru.epochs[one[1]]=9;lru.epochs[two[0]]=8;lru.epochs[two[1]]=3;
     let all=candidates(&lru);
     assert_eq!(lru.forfeit_sibling(&all,0,5),Some((two[1] as u8,2)),"least recently used idle pair");
     assert_eq!(lru.forfeit_sibling(&candidates(&lru),0,6),Some((one[0] as u8,1)));
     assert_eq!(lru.forfeit_sibling(&candidates(&lru),0,7),None,"donors keep their last pair");
     println!("forfeit guards: borrowed, active, quarantined, held, released, last-pair, mismatch, recipient and recency controls");
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
    constants=re.search(r'const QID_MAX: [^;]+;',fw).group(0)+'\n'+'\n'.join(re.search(r'const '+name+r': [^;]+;',s).group(0) for name in ['QID_COUNT','MIN_RENDER_IDENTITIES'])
    constants+='\n'+item(s,'fn render_floor(')
    header='''#![allow(dead_code, unused_macros)]
    type Result<T=()> = std::result::Result<T,i32>;
    const ENOSPC:i32=-28; const EINVAL:i32=-22; const EOVERFLOW:i32=-75;
    macro_rules! resource_exhausted { ($s:literal)=>{ENOSPC}; }
    #[derive(Clone,Copy,PartialEq,Eq,Debug)] enum DataMaster {Compute,Tiling,Fragment}
    #[derive(Clone,Copy,PartialEq,Eq,Debug)] struct Id {qid:u8,owner:u64,engine:DataMaster}
    #[derive(Clone,Copy)] struct Identity {id:Id,published:bool}
    '''+constants+'\n'+item(s,'pub(crate) struct QueueIds')+'\nimpl QueueIds {\n'+''.join(item(s,name+'(')+'\n' for name in ['pub(crate) fn reserve_initial_render','pub(crate) fn reserve','pub(crate) fn reserve_compute','fn reserve_for','fn pair_identities','pub(crate) fn render_surplus','pub(crate) fn compute_available','pub(crate) fn publish','pub(crate) fn can_cancel_unpublished','pub(crate) fn cancel_unpublished'])+'}\n'
    test='''
    fn fresh()->QueueIds {QueueIds {entries:[None;QID_COUNT],initial_render:[None;2],compute_publication:0,leased:0}}
    fn main(){
     let floor=|queues:usize| MIN_RENDER_IDENTITIES.max(2*queues).min(QID_COUNT-2);
     assert_eq!(render_floor(0),16);assert_eq!(render_floor(26),52);assert_eq!(render_floor(58),116);assert_eq!(render_floor(1000),QID_COUNT-2);
     let mut cases=0;
     for queues in [0usize,8,20,26,54,58] {
      for render in [0usize,2,12,16,32,52,108,116,126] {
       let mut q=fresh();
       for i in 0..render {q.reserve(i as u64+1,if i%2==0 {DataMaster::Tiling}else{DataMaster::Fragment},None).unwrap();}
       assert_eq!(q.render_surplus(queues),render.saturating_sub(floor(queues)));
       let mut compute=0;
       loop {
        let available=q.compute_available(queues);
        let result=q.reserve_compute(1000+compute,None,queues);
        assert_eq!(available,result.is_ok(),"availability predicts admission");
        if result.is_err() {assert_eq!(result,Err(ENOSPC));break;}
        compute+=1;
       }
       assert_eq!(compute as usize,QID_COUNT-render.max(floor(queues)));
       // Render keeps a pair for every open logical queue.
       let mut pairs=render;
       while q.reserve(5000+pairs as u64,DataMaster::Tiling,None).is_ok() {pairs+=1;}
       assert!(pairs>=floor(queues),"render capacity {} below {} open queues",pairs,queues);
       cases+=1;
      }
     }
     // compute20: 20 compute clients beside a desktop of six owners with pairs.
     let mut q=fresh();q.reserve_initial_render().unwrap();
     for owner in 3..15 {q.reserve(owner,DataMaster::Tiling,None).unwrap();}
     for owner in 100..126 {q.reserve_compute(owner,Some(4),26).unwrap();}
     assert!(q.compute_available(26),"26 compute queues while QIDs are free: no sharing");
     println!("queue IDs: {} open-queue/render/compute capacity boundaries; a render pair for every open queue; contention-free compute keeps one QID per owner",cases);
     let mut q=fresh();let first=q.reserve(1,DataMaster::Tiling,None).unwrap();
     assert!(q.can_cancel_unpublished(first));q.publish(first).unwrap();
     assert!(!q.can_cancel_unpublished(first));assert_eq!(q.cancel_unpublished(first),Err(EINVAL));assert_eq!(q.leased,1);
     let unused=q.reserve(2,DataMaster::Fragment,None).unwrap();q.cancel_unpublished(unused).unwrap();assert_eq!(q.leased,1);
     let next=q.reserve(3,DataMaster::Fragment,None).unwrap();assert_eq!(next.qid,unused.qid);
     assert!(!q.can_cancel_unpublished(unused));assert_eq!(q.cancel_unpublished(unused),Err(EINVAL));
     println!("queue IDs: published and stale identities protected");
     // Reproduce the hardware snapshot: 124 installed render identities, two
     // installed compute identities, and two initial reservations fill128 IDs.
     let mut q=fresh();q.reserve_initial_render().unwrap();
     assert_eq!(q.reserve_initial_render(),Err(EINVAL));
     let initial=q.initial_render;
     for owner in 1000..1002 {let id=q.reserve_compute(owner,Some(4),0).unwrap();q.publish(id).unwrap();}
     let mut installed=Vec::new();
     for owner in 3..127 {let id=q.reserve(owner,if owner%2==0 {DataMaster::Tiling}else{DataMaster::Fragment},None).unwrap();q.publish(id).unwrap();installed.push(id);}
     assert_eq!(q.leased,QID_COUNT);
     for (n,old) in initial.into_iter().enumerate() {
      let old=old.unwrap();let id=q.reserve_compute(2000+n as u64,None,0).unwrap();
      assert_eq!(id.qid,old.qid);assert_eq!(q.leased,QID_COUNT);
      assert!(!q.can_cancel_unpublished(old));assert_eq!(q.cancel_unpublished(old),Err(EINVAL));
      q.publish(id).unwrap();
     }
     assert_eq!(q.reserve_compute(3000,None,0),Err(ENOSPC));
     for id in installed {assert!(!q.can_cancel_unpublished(id));assert_eq!(q.entries[id.qid as usize].unwrap().id,id);}
     // A bootstrap token that became published is never replaced.
     let mut q=fresh();q.reserve_initial_render().unwrap();let protected=q.initial_render[0].unwrap();q.publish(protected).unwrap();
     for owner in 3..129 {q.reserve(owner,DataMaster::Tiling,None).unwrap();}
     assert_eq!(q.leased,QID_COUNT);
     assert_eq!(q.reserve_compute(1000,None,0).unwrap().qid,1);
     assert_eq!(q.reserve_compute(1001,None,0),Err(ENOSPC));
     assert_eq!(q.entries[0].unwrap().id,protected);
     // Compute stops at the render reserve, then replaces only the bootstrap
     // identities no render pair can use; the reserve itself stays free.
     for queues in [0usize,40,58] {
      let mut q=fresh();q.reserve_initial_render().unwrap();let initial=q.initial_render;
      for owner in 3..17 {q.reserve(owner,DataMaster::Tiling,None).unwrap();}
      let mut compute=0;
      while q.reserve_compute(1000+compute,None,queues).is_ok() {compute+=1;}
      assert_eq!(compute as usize,QID_COUNT-2-render_floor(queues)+2);
      assert!(q.initial_render.iter().all(Option::is_none));
      for old in initial.into_iter().flatten() {assert_eq!(q.entries[old.qid as usize].unwrap().id.engine,DataMaster::Compute);}
      assert_eq!(QID_COUNT-q.leased,render_floor(queues)-14,"render can still install a pair for every open queue");
      assert!(!q.compute_available(queues));
     }
     // Ordinary free capacity leaves the initialization reservations untouched.
     let mut q=fresh();q.reserve_initial_render().unwrap();let initial=q.initial_render;
     let id=q.reserve_compute(1000,Some(4),0).unwrap();assert_eq!(id.qid,4);assert_eq!(q.initial_render,initial);
     println!("initial reservations: replaced only beyond the open-queue render floor; physical count unchanged; published and stale identities plus that floor protected");

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

def compute_takeover_tests(root, source):
    s=(source/'g17/queue/compute.rs').read_text()
    def item(start):
     at=s.index(start);begin=s.index('{',at);depth=1;end=begin+1
     while depth:
      depth+=(s[end]=='{')-(s[end]=='}');end+=1
     return s[at:end]
    methods='\n'.join(item('pub(crate) fn '+name+'(') for name in ['release_owner','idle_owned','yield_idle_owner','reusable'])
    fixture=root/'compute-takeover-controls.rs'
    fixture.write_text('''#![allow(dead_code)]
type Result<T=()> = std::result::Result<T,i32>;
const EBUSY:i32=-16;const EIO:i32=-5;
fn now_ns()->u64 {42}
#[derive(Clone,Copy,PartialEq,Eq,Debug)] enum PoolPhase {Populated,ReleaseSent(u32),Vacant}
struct Fifo {len:usize}
#[derive(Clone,Copy,PartialEq,Eq,Debug)] struct Binding {context:u32}
struct Previous {binding:Option<Binding>,context:u32,publication:Option<(u64,u64)>}
struct Graph {owner:u32}
impl Graph {fn set_owner(&mut self,context:&u32)->Result {self.owner=*context;Ok(())}}
struct Queue {owner:Option<u64>,released:bool,installed:bool,growing:bool,pool_phase:PoolPhase,
 needs_fresh_backing:bool,exit:Option<()>,quarantined:bool,awaiting_witness:bool,active:Fifo,replays:Fifo,
 previous:Option<Previous>,binding:Option<Binding>,retirement_ready:bool,released_at:u64,
 retired_by_teardown:bool,graph:Graph}
impl Queue {
'''+methods+'''
}
fn idle()->Queue {Queue {owner:Some(1),released:false,installed:true,growing:false,pool_phase:PoolPhase::Populated,
 needs_fresh_backing:false,exit:None,quarantined:false,awaiting_witness:false,active:Fifo {len:0},replays:Fifo {len:0},
 previous:None,binding:Some(Binding {context:1}),retirement_ready:true,released_at:0,retired_by_teardown:false,graph:Graph {owner:1}}}
fn main(){
 let mut q=idle();assert!(q.idle_owned());assert!(!q.reusable());
 q.yield_idle_owner().unwrap();
 assert!(q.reusable(),"a yielded queue is exactly a closed owner's reusable queue");
 assert_eq!((q.owner,q.released,q.retirement_ready,q.released_at),(None,true,true,42));
 assert_eq!(q.binding,Some(Binding {context:1}),"the old binding stays until bind_owner supersedes it");
 // A queue bound at submission whose packet has not published is also idle.
 let mut q=idle();q.retirement_ready=false;assert!(q.idle_owned());q.yield_idle_owner().unwrap();assert!(q.reusable());
 let cases:[(&str,fn(&mut Queue));13]=[
  ("released",|q|q.released=true),("never installed",|q|q.installed=false),("growing",|q|q.growing=true),
  ("pool release",|q|q.pool_phase=PoolPhase::ReleaseSent(1)),("vacant",|q|q.pool_phase=PoolPhase::Vacant),
  ("fresh backing",|q|q.needs_fresh_backing=true),("exit",|q|q.exit=Some(())),("quarantined",|q|q.quarantined=true),
  ("witness",|q|q.awaiting_witness=true),("active",|q|q.active.len=1),("replay",|q|q.replays.len=1),
  ("superseded",|q|q.previous=Some(Previous {binding:None,context:0,publication:None})),("detached",|q|q.binding=None),
 ];
 for (name,change) in cases {
  let mut q=idle();change(&mut q);
  assert!(!q.idle_owned(),"{} must not be taken over",name);
  let before=(q.owner,q.released,q.retirement_ready,q.released_at);
  assert_eq!(q.yield_idle_owner(),Err(EBUSY));
  assert_eq!((q.owner,q.released,q.retirement_ready,q.released_at),before,"{}: refusal changes nothing",name);
 }
 let mut q=idle();q.owner=None;assert!(!q.idle_owned(),"no owner");
 println!("compute takeover: idle live queue yields into reusable state; 14 busy/transitional states refused unchanged");
 // An owner that rebound a detached queue and closed without publishing.
 let mut q=idle();q.retirement_ready=false;assert_eq!(q.release_owner(),Ok(false));
 assert_eq!(q.owner,None);assert!(q.reusable(),"QID returns to service at close");
 // Controls: retained work, a superseded binding and a pending replay keep the owner.
 let mut q=idle();q.retirement_ready=false;q.active.len=1;q.release_owner().unwrap();assert_eq!(q.owner,Some(1));
 let mut q=idle();q.retirement_ready=false;q.replays.len=1;q.release_owner().unwrap();assert_eq!(q.owner,Some(1));
 let mut q=idle();q.retirement_ready=false;q.exit=Some(());q.release_owner().unwrap();assert_eq!(q.owner,Some(1));
 let mut q=idle();q.retirement_ready=false;q.binding=Some(Binding {context:2});
 q.previous=Some(Previous {binding:Some(Binding {context:1}),context:1,publication:None});
 q.release_owner().unwrap();assert_eq!((q.owner,q.binding,q.graph.owner),(None,Some(Binding {context:1}),1),"unpublished successor reverts");
 let mut q=idle();q.retirement_ready=false;
 q.previous=Some(Previous {binding:None,context:1,publication:Some((1,1))});
 q.release_owner().unwrap();assert_eq!(q.owner,Some(1),"published successor waits for its witness");
 let mut q=idle();q.installed=false;q.retirement_ready=false;assert_eq!(q.release_owner(),Ok(true),"never installed: cancel QID");
 println!("compute close: unpublished rebinding returns its QID; work, replay, exit and superseded controls retained");
}
''')
    subprocess.run([os.environ.get('RUSTC','rustc'),'--edition=2021',str(fixture),'-o',str(root/'compute-takeover-controls')],check=True)
    subprocess.run([str(root/'compute-takeover-controls')],check=True)

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
    # pair lookups or a collector allocation from its watchdog: either error
    # stops the runtime for every client.
    for path, name in [('g17/runtime/render/settle.rs', 'pub(in crate::g17) fn timeout_render('),
                       ('g17/runtime/compute_control.rs', 'pub(super) fn timeout_compute(')]:
        text = (source / path).read_text()
        at = text.index(name)
        body = text[at:text.index('\n    }\n', at)]
        guard = body.index('if !packet.is_published() {\n            return Ok(());')
        collector = body.index('let Ok(mut deferred) = DeferredBatch::worker() else {')
        assert guard < collector and 'DeferredBatch::worker()?' not in body, name
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

def compute_admission_tests(root, source):
    """held40 on hardware: 40 held clients and 12 fresh ones beside a desktop,
    with every QID leased (108 render identities, 20 compute queues). Render
    surplus beyond the open-queue floor cannot become compute capacity, so a
    client without a queue must be able to take an idle one over."""
    g17 = source / "g17"
    def item(s,start):
     at=s.index(start);begin=s.index('{',at);depth=1;end=begin+1
     while depth:
      depth+=(s[end]=='{')-(s[end]=='}');end+=1
     return s[at:end]
    s=(g17/'kick.rs').read_text()
    fw=(g17/'fw/kick.rs').read_text()
    runtime=(g17/'runtime.rs').read_text()
    constants=re.search(r'const QID_MAX: [^;]+;',fw).group(0)+'\n'+'\n'.join(re.search(r'const '+name+r': [^;]+;',s).group(0) for name in ['QID_COUNT','MIN_RENDER_IDENTITIES'])
    constants+='\n'+item(s,'fn render_floor(')
    header='''#![allow(dead_code, unused_macros)]
type Result<T=()> = std::result::Result<T,i32>;
const ENOSPC:i32=-28; const EINVAL:i32=-22; const EOVERFLOW:i32=-75;
macro_rules! resource_exhausted { ($s:literal)=>{ENOSPC}; }
#[derive(Clone,Copy,PartialEq,Eq,Debug)] enum DataMaster {Compute,Tiling,Fragment}
#[derive(Clone,Copy,PartialEq,Eq,Debug)] struct Id {qid:u8,owner:u64,engine:DataMaster}
#[derive(Clone,Copy)] struct Identity {id:Id,published:bool}
'''+constants+'\n'+item(s,'pub(crate) struct QueueIds')+'\nimpl QueueIds {\n'+''.join(item(s,name+'(')+'\n' for name in ['pub(crate) fn reserve','pub(crate) fn reserve_compute','fn reserve_for','fn pair_identities','pub(crate) fn render_surplus','pub(crate) fn compute_available','pub(crate) fn publish','pub(crate) fn can_cancel_unpublished'])+'}\n'+item(runtime,'fn takeover_allowed(')+'\n'
    test='''
fn fresh()->QueueIds {QueueIds {entries:[None;QID_COUNT],initial_render:[None;2],compute_publication:0,leased:0}}
// Physical compute queues: their bound owner, or none once released.
struct Queues {owners:Vec<Option<u64>>,takeovers:usize}
// One publication: the locked rebind or takeover, then the off-lock attempt,
// as compute_packet alternates them. True once the client has a queue.
fn publish(q:&mut QueueIds,queues:&mut Queues,client:u64,open:usize)->bool {
 let mut contended=false;
 for _ in 0..3 {
  if queues.owners.contains(&Some(client)) {return true;}
  if let Some(slot)=queues.owners.iter_mut().find(|owner|owner.is_none()) {*slot=Some(client);return true;}
  if takeover_allowed(q.compute_available(open),q.render_surplus(open),contended) {
   // Every other owner's queue is idle between its iterations.
   if let Some(slot)=queues.owners.iter_mut().find(|owner|owner.is_some_and(|o|o!=client)) {
    *slot=Some(client);queues.takeovers+=1;return true;
   }
  }
  match q.reserve_compute(10_000+client,None,open) {
   Ok(id)=>{q.publish(id).unwrap();queues.owners.push(Some(client));return true;}
   Err(error)=>{assert_eq!(error,ENOSPC);contended=true;}
  }
 }
 false
}
fn main(){
 let mut q=fresh();
 for owner in 1..=108u64 {
  let id=q.reserve(owner,if owner%2==0 {DataMaster::Tiling}else{DataMaster::Fragment},None).unwrap();
  q.publish(id).unwrap();
 }
 let mut queues=Queues {owners:Vec::new(),takeovers:0};
 for n in 0..20u64 {let id=q.reserve_compute(1000+n,None,0).unwrap();q.publish(id).unwrap();queues.owners.push(None);}
 assert_eq!(q.leased,QID_COUNT);
 let desktop=11;
 let mut open=desktop+40;
 assert!(!q.compute_available(open)&&q.render_surplus(open)>=2,"snapshot: no fresh QID, render surplus beyond the floor");
 for round in 0..10 {
  for client in 1..=40u64 {
   assert!(publish(&mut q,&mut queues,client,open),"held client {} could not bind in round {}",client,round);
  }
 }
 for fresh_client in 41..=52u64 {
  open+=1;
  for _ in 0..3 {
   assert!(publish(&mut q,&mut queues,fresh_client,open),"fresh client {} could not bind",fresh_client);
   for client in 1..=40u64 {assert!(publish(&mut q,&mut queues,client,open),"held client {} could not bind again",client);}
  }
  for owner in queues.owners.iter_mut().filter(|owner|**owner==Some(fresh_client)) {*owner=None;}
  open-=1;
 }
 assert_eq!(queues.owners.len(),20,"no QID changed engine");
 assert!(queues.takeovers>0);
 let held=queues.takeovers;
 // render50: 47 clients fill the 58 contexts, so the render floor leaves no
 // surplus. Eight fail render capacity and close; the floor drops and the
 // surplus returns while the 39 others still need compute queues.
 let mut q=fresh();
 for owner in 1..=108u64 {
  let id=q.reserve(owner,if owner%2==0 {DataMaster::Tiling}else{DataMaster::Fragment},None).unwrap();
  q.publish(id).unwrap();
 }
 let mut queues=Queues {owners:Vec::new(),takeovers:0};
 for n in 0..20u64 {let id=q.reserve_compute(1000+n,None,0).unwrap();q.publish(id).unwrap();queues.owners.push(None);}
 let mut open=desktop+47;
 assert!(q.render_surplus(open)<2,"58 open queues: the floor covers every render identity");
 for client in 1..=47u64 {assert!(publish(&mut q,&mut queues,client,open),"render50 client {} could not bind",client);}
 for client in 40..=47u64 {for owner in queues.owners.iter_mut().filter(|owner|**owner==Some(client)) {*owner=None;}}
 open-=8;
 assert!(q.render_surplus(open)>=2&&!q.compute_available(open),"closes leave render surplus and no fresh QID");
 // Every survivor publishes while each other one holds a queue.
 for round in 0..5 {
  for client in 1..=39u64 {
   assert!(publish(&mut q,&mut queues,client,open),"render50 survivor {} could not bind in round {}",client,round);
  }
 }
 println!("compute admission: held40 snapshot, 40 held and 12 fresh clients share 20 queues by takeover ({} takeovers); render50 survivors keep binding after closes restore render surplus ({} takeovers); none starves",held,queues.takeovers);
}
'''
    code=header+test
    for name,old,new in [('fixed',None,None),('render-surplus-gate',
                         'contended || (!compute_available && render_surplus < 2)',
                         '(!compute_available && render_surplus < 2)')]:
        current=code
        if old is not None:
            assert current.count(old)==1, name
            current=current.replace(old,new)
        fixture=root/('compute-admission-'+name+'.rs');fixture.write_text(current)
        subprocess.run([os.environ.get('RUSTC','rustc'),'--edition=2021',str(fixture),'-o',str(fixture.with_suffix(''))],check=True)
        result=subprocess.run([str(fixture.with_suffix(''))],capture_output=True,text=True)
        if old is None:
            assert result.returncode==0,result.stderr
            print(result.stdout.strip())
        else:
            assert result.returncode!=0 and 'could not bind' in result.stderr,result.stderr
            print('PASS negative control:',name)


def forfeit_donor_tests(root, source):
    """held40 with the compositor as a forfeit donor. Under full slot pressure
    a refused held client takes the compositor's least recently used sibling;
    the compositor keeps rendering two overlapping commands every frame, while
    the recipient holds the pair and after it closes. Runs the actual admission
    State; the registry side is modelled from the rules checked below."""
    g17 = source / 'g17'
    def item(s, start):
        at = s.index(start); begin = s.index('{', at); depth = 1; end = begin + 1
        while depth:
            depth += (s[end] == '{') - (s[end] == '}'); end += 1
        return s[at:end]
    def body(s, start):
        return item(s, start)
    # The modelled rules, anchored in the driver source.
    pair = (g17 / 'queue/render.rs').read_text()
    assert re.search(r'if foreign \{\s*self\.displaced = Some\(retired\);',
                     body(pair, 'pub(crate) fn bind_owner(')), \
        'a foreign rebind displaces the old graph: revisit this model'
    teardown = (g17 / 'runtime/render/teardown.rs').read_text()
    assert re.search(r'if self\.retired\[index\]\.buffer_id\(\)\.is_some\(\) \{\s*index \+= 1;\s*continue;',
                     body(teardown, 'pub(in crate::g17::runtime) fn collect_reclaimable(')), \
        'a displaced graph with memory stays until its context is detached: revisit this model'
    assert 'pairs: [Option<u8>; 2],' in (g17 / 'context/identity.rs').read_text(), \
        'a context has two render work-state channels: revisit this model'

    s = (g17 / 'queue/admission.rs').read_text(); cfg = (source / 'hw/t8140.rs').read_text()
    constants = '\n'.join(re.search(r'pub\(crate\) const ' + name + r': [^;]+;', cfg).group(0)
                          for name in ['RENDER_SLOTS', 'RENDER_PAIRS_PER_OWNER', 'RENDER_DEPTH'])
    mmu = (source / 'mmu.rs').read_text()
    limits = '\n'.join(re.search(r'const ' + name + r': [^;]+;', mmu).group(0)
                       for name in ['EXECUTION_CONTEXT_MASK', 'MAX_EXECUTION_CONTEXTS'])
    header = '''#![allow(dead_code, unused_macros)]
    type Result<T=()> = std::result::Result<T,i32>;
    const ENOSPC:i32=-28; const EIO:i32=-5;
    macro_rules! resource_exhausted { ($s:literal)=>{ENOSPC}; }
    type KVec<T> = Vec<T>; struct InFlight;
    mod cfg {\n''' + constants + '}\n' + limits + '\n'
    methods = ['const fn new', 'fn exhausted', 'fn refusal', 'fn advance', 'fn take_idle', 'fn acquire',
               'fn claim_sibling', 'fn forfeit_sibling', 'fn release']
    code = header + item(s, 'struct State') + '\nimpl State {\n' + \
        re.search(r'    const MASK: u64 = .*;', s).group(0) + '\n' + \
        '\n'.join(item(s, n) for n in methods) + '\n}\n' + '''
    use std::collections::{HashMap,HashSet};
    // Registry side of a publication: the graph installed on each pair,
    // displaced graphs of open owners (retained for their scheduler until
    // that owner's own release), and each context's two render work-state
    // channels, bound to the pairs it published on.
    struct Model {graph:[u64;64],retained:Vec<(u64,usize)>,channels:HashMap<u64,Vec<usize>>,open:HashSet<u64>}
    impl Model {
     fn publish(&mut self,owner:u64,slot:usize) {
      if self.graph[slot]!=owner {
       if self.retained.contains(&(owner,slot)) {
        panic!("owner {} would install a second graph on pair {} while its displaced graph there is retained for its scheduler",owner,slot);
       }
       let previous=self.graph[slot];
       if previous!=0 && self.open.contains(&previous) {self.retained.push((previous,slot));}
       self.graph[slot]=owner;
      }
      let channels=self.channels.entry(owner).or_default();
      if !channels.contains(&slot) {
       assert!(channels.len()<2,"owner {} has no work-state channel for pair {}: it published on {:?}",owner,slot,channels);
       channels.push(slot);
      }
     }
     fn close(&mut self,owner:u64) {self.open.remove(&owner);self.retained.retain(|(vm,_)|*vm!=owner);}
    }
    // Pool::release_owner on the State: logical close of every slot of `owner`.
    fn close(s:&mut State,m:&mut Model,owner:u64) {
     for slot in 0..cfg::RENDER_SLOTS {if s.owners[slot]==owner {s.released|=1u64<<slot;}}
     for value in s.forfeited.iter_mut().filter(|v|**v==owner) {*value=0;}
     m.close(owner);
    }
    // One command: admission, then construction and publication on its pair.
    fn render(s:&mut State,m:&mut Model,owner:u64)->u8 {
     let slot=s.acquire(owner,None).unwrap_or_else(||panic!("owner {} found no pair",owner));
     m.publish(owner,slot as usize);
     slot
    }
    // Forfeit candidates: idle installed pairs of open owners with two installed pairs.
    fn candidates(s:&State,m:&Model)->Vec<(u8,u64)> {
     (1..cfg::RENDER_SLOTS).filter(|slot|{
      let donor=s.owners[*slot];
      donor!=0 && m.open.contains(&donor) && m.graph[*slot]==donor && s.refs[*slot]==0
       && (1..cfg::RENDER_SLOTS).filter(|other|s.owners[*other]==donor && m.graph[*other]==donor).count()>=2
     }).map(|slot|(slot as u8,s.owners[slot])).collect()
    }
    fn main(){
     const COMPOSITOR:u64=1;
     let mut s=State::new();
     let mut m=Model{graph:[0;64],retained:Vec::new(),channels:HashMap::new(),open:HashSet::new()};
     // The desktop and earlier clients hold every pair: 31 owners render on two, one on one.
     for owner in 1..=32 {
      m.open.insert(owner);
      let a=render(&mut s,&mut m,owner);
      if owner<32 {let b=render(&mut s,&mut m,owner);assert_ne!(a,b);s.release(b);}
      s.release(a);
     }
     assert_eq!(s.owners[1..].iter().filter(|o|**o!=0).count(),63);
     // Everyone but the compositor renders on both pairs again; the compositor
     // keeps drawing on one, so its sibling is the least recently used pair.
     for owner in 2..=31 {let a=render(&mut s,&mut m,owner);let b=render(&mut s,&mut m,owner);s.release(b);s.release(a);}
     let first=render(&mut s,&mut m,COMPOSITOR);s.release(first);
     let mine:Vec<usize>=(1..cfg::RENDER_SLOTS).filter(|slot|s.owners[*slot]==COMPOSITOR).collect();
     assert_eq!(mine.len(),2);
     // held40: refused clients take idle siblings of open owners.
     let mut recipients=Vec::new();
     for owner in 100..140 {
      m.open.insert(owner);
      assert_eq!(s.refusal(owner),Err(ENOSPC));
      if let Some((slot,donor))=s.forfeit_sibling(&candidates(&s,&m),0,owner) {recipients.push((owner,slot as usize,donor));}
     }
     let &(holder,given,_)=recipients.iter().find(|r|r.2==COMPOSITOR).expect("the compositor donates its least recently used sibling");
     let kept=*mine.iter().find(|slot|**slot!=given).unwrap();
     for round in 0..10 {
      for &(owner,_,_) in &recipients {let slot=render(&mut s,&mut m,owner);s.release(slot);}
      // The compositor's recipient draws most recently of all.
      let slot=render(&mut s,&mut m,holder);s.release(slot);
      // Two overlapping compositor commands per frame.
      let x=render(&mut s,&mut m,COMPOSITOR);let y=render(&mut s,&mut m,COMPOSITOR);
      assert!(x as usize!=given && y as usize!=given,"round {}: the compositor uses its own pair, never the one its recipient holds",round);
      s.release(y);s.release(x);
     }
     assert_eq!(s.owners[given],holder);
     assert!(m.retained.contains(&(COMPOSITOR,given)),"control: the recipient displaced the compositor's graph");
     // The held clients exit; the compositor keeps drawing for 20 frames,
     // beside fresh clients that reuse the released pairs.
     for &(owner,_,_) in &recipients {close(&mut s,&mut m,owner);}
     for owner in 100..140 {m.open.remove(&owner);}
     for round in 0..20 {
      let x=render(&mut s,&mut m,COMPOSITOR);let y=render(&mut s,&mut m,COMPOSITOR);
      assert_eq!((x as usize,y as usize),(kept,kept),"round {}: the compositor stays on the pair it kept",round);
      if round<12 {
       let fresh=200+round as u64;m.open.insert(fresh);
       let slot=render(&mut s,&mut m,fresh);s.release(slot);close(&mut s,&mut m,fresh);
      }
      s.release(y);s.release(x);
     }
     assert_eq!(s.owners.iter().filter(|o|**o==COMPOSITOR).count(),1,"a donor receives no other pair while open");
     // Once the compositor itself closes, its record clears.
     close(&mut s,&mut m,COMPOSITOR);
     assert!(!s.forfeited.contains(&COMPOSITOR));
     println!("forfeit donor: the compositor donates its idle sibling under held40, renders every frame on the pair it kept while the recipient holds the other and after it exits, and never takes back a pair its displaced graph still holds");
    }
    '''
    # The donor rule before this fix: a donor takes back the pair it gave away
    # once free. Its next command there waits for its own retained graph.
    reclaim = [
        ('    forfeited: [u64; cfg::RENDER_SLOTS],\n}',
         '    forfeited: [u64; cfg::RENDER_SLOTS],\n    forfeited_slots: [u8; cfg::RENDER_SLOTS],\n}'),
        ('            forfeited: [0; cfg::RENDER_SLOTS],\n',
         '            forfeited: [0; cfg::RENDER_SLOTS],\n            forfeited_slots: [0; cfg::RENDER_SLOTS],\n'),
        ('''        if self.owners.iter().filter(|id| **id == owner).count() >= cfg::RENDER_PAIRS_PER_OWNER
            || self.forfeited.contains(&owner)
        {
            return;
        }
        let free = !self.active & Self::MASK & !self.quarantined & !self.kill_hold;
        let eligible = |slot: usize| free & (1u64 << slot) != 0;
''', '''        if self.owners.iter().filter(|id| **id == owner).count() >= cfg::RENDER_PAIRS_PER_OWNER {
            return;
        }
        let free = !self.active & Self::MASK & !self.quarantined & !self.kill_hold;
        let eligible = |slot: usize| free & (1u64 << slot) != 0;
        if let Some(record) = self.forfeited.iter().position(|id| *id == owner) {
            let slot = usize::from(self.forfeited_slots[record]);
            if eligible(slot)
                && self.owners[slot] != owner
                && (self.released & (1u64 << slot) != 0 || self.owners[slot] == 0)
            {
                self.forfeited[record] = 0;
                self.forfeited_slots[record] = 0;
                self.released &= !(1u64 << slot);
                self.owners[slot] = owner;
                self.epochs[slot] = 0;
            }
            return;
        }
'''),
        ('''        if !self.forfeited.contains(&donor) {
            self.forfeited[record] = donor;
        }
''', '''        let record = self
            .forfeited
            .iter()
            .position(|value| *value == donor)
            .unwrap_or(record);
        self.forfeited[record] = donor;
        self.forfeited_slots[record] = slot;
'''),
    ]
    # Without the donor rule a donor claims any free sibling: a third pair for
    # a context whose two work-state channels are bound.
    unguarded = [('''
            || self.forfeited.contains(&owner)
        {
            return;
        }''', ''' {
            return;
        }''')]
    for name, edits, expected in [('fixed', [], None),
                                  ('donor-reclaims-its-pair', reclaim, 'second graph on pair'),
                                  ('donor-claims-any-sibling', unguarded, 'no work-state channel')]:
        current = code
        for old, new in edits:
            assert current.count(old) == 1, (name, old[:50])
            current = current.replace(old, new)
        fixture = root / ('forfeit-donor-' + name + '.rs'); fixture.write_text(current)
        subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', str(fixture), '-o',
                        str(fixture.with_suffix(''))], check=True)
        result = subprocess.run([str(fixture.with_suffix(''))], capture_output=True, text=True)
        if expected is None:
            assert result.returncode == 0, result.stderr
            print(result.stdout.strip())
        else:
            assert result.returncode != 0 and expected in result.stderr, result.stderr
            print('PASS negative control:', name)

def shared_vm_donor_tests(root, source):
    """A VM with several render queues (one DRM file, as quickshell or a GPU
    process with several GL contexts) under held40 pressure. One queue donates
    its idle sibling; its displaced graph keeps that VM's view of the pair
    until the queue's own scheduler release. The VM's other queues are then
    offered the freed pair: an existing queue claiming a sibling, and a new
    queue. Runs the actual admission State and the actual per-VM view records
    of mmu/lifetime.rs; graph lifetimes follow the rules checked in
    forfeit_donor_tests."""
    g17 = source / 'g17'
    def item(s, start):
        at = s.index(start); begin = s.index('{', at); depth = 1; end = begin + 1
        while depth:
            depth += (s[end] == '{') - (s[end] == '}'); end += 1
        return s[at:end]
    s = (g17 / 'queue/admission.rs').read_text(); cfg = (source / 'hw/t8140.rs').read_text()
    constants = '\n'.join(re.search(r'pub\(crate\) const ' + name + r': [^;]+;', cfg).group(0)
                          for name in ['RENDER_SLOTS', 'RENDER_PAIRS_PER_OWNER', 'RENDER_DEPTH'])
    mmu = (source / 'mmu.rs').read_text()
    limits = '\n'.join(re.search(r'const ' + name + r': [^;]+;', mmu).group(0)
                       for name in ['EXECUTION_CONTEXT_MASK', 'MAX_EXECUTION_CONTEXTS'])
    lifetime = (source / 'mmu/lifetime.rs').read_text()
    views = item(lifetime, 'pub(super) struct DriverMappings') + '\n' + item(lifetime, 'struct PoolMappings') + '\n' + \
        item(lifetime, 'impl DriverMappings') + '\nimpl Vm {\n' + '\n'.join(
            item(lifetime, 'pub(crate) fn ' + name + '(') for name in [
                'install_driver_mappings', 'render_pool_tvb_blocks', 'install_render_pool_mappings',
                'append_render_pool_mappings_prepared', 'truncate_render_pool_mappings',
                'clear_render_pool_mappings_if_owner']) + '\n}\n'
    views = views.replace('pub(super) ', '').replace('pub(crate) ', '')
    header = '''#![allow(dead_code, unused_macros, unused_imports)]
    use std::ops::Range;
    type Result<T=()> = std::result::Result<T,i32>;
    const ENOSPC:i32=-28; const EIO:i32=-5; const EINVAL:i32=-22; const EBUSY:i32=-16; const EOVERFLOW:i32=-75;
    macro_rules! resource_exhausted { ($s:literal)=>{ENOSPC}; }
    struct InFlight;
    #[derive(Clone,Copy)] struct Flags; const GFP_KERNEL:Flags=Flags;
    struct KVec<T>(Vec<T>);
    impl<T> KVec<T> {
     const fn new()->Self {KVec(Vec::new())}
     fn push(&mut self,v:T,_:Flags)->Result {self.0.push(v);Ok(())}
     fn push_within_capacity(&mut self,v:T)->std::result::Result<(),T> {if self.0.len()<self.0.capacity() {self.0.push(v);Ok(())} else {Err(v)}}
     fn reserve(&mut self,n:usize,_:Flags)->Result {self.0.reserve(n);Ok(())}
     fn remove(&mut self,i:usize)->std::result::Result<T,()> {if i<self.0.len() {Ok(self.0.remove(i))} else {Err(())}}
     fn pop(&mut self)->Option<T> {self.0.pop()}
     fn drain_all(&mut self)->std::vec::Drain<'_,T> {self.0.drain(..)}
     fn len(&self)->usize {self.0.len()}
     fn capacity(&self)->usize {self.0.capacity()}
     fn is_empty(&self)->bool {self.0.is_empty()}
     fn iter(&self)->std::slice::Iter<'_,T> {self.0.iter()}
     fn iter_mut(&mut self)->std::slice::IterMut<'_,T> {self.0.iter_mut()}
    }
    struct KernelMapping(u64);
    impl KernelMapping {fn iova_range(&self)->Range<u64> {self.0..self.0+1}}
    struct Mutex<T>(std::sync::Mutex<T>);
    impl<T> Mutex<T> {fn lock(&self)->std::sync::MutexGuard<'_,T> {self.0.lock().unwrap()}}
    struct VmLifetime {driver:Mutex<Option<DriverMappings>>}
    struct Vm {lifetime:Option<std::sync::Arc<VmLifetime>>}
    mod hw {pub mod t8140 {pub mod queues {''' + re.search(r'pub\(crate\) const RENDER_SLOTS: [^;]+;', cfg).group(0).replace('pub(crate)', 'pub') + '''}}}
    mod cfg {\n''' + constants + '}\n' + limits + '\n'
    methods = ['const fn new', 'fn exhausted', 'fn refusal', 'fn advance', 'fn take_idle', 'fn acquire',
               'fn claim_sibling', 'fn forfeit_sibling', 'fn release']
    follow = ',None' if 'fn acquire(&mut self, owner: u64, follow' in s else ''
    code = header + views + item(s, 'struct State') + '\nimpl State {\n' + \
        re.search(r'    const MASK: u64 = .*;', s).group(0) + '\n' + \
        '\n'.join(item(s, n) for n in methods) + '\n}\n' + '''
    use std::collections::{HashMap,HashSet};
    // The graph on each pair (owner, view id); a parked pair has none.
    // Displaced graphs of open queues keep their VM view until that queue's
    // scheduler release; a closed queue's graphs are released and parked.
    struct Model {graph:[(u64,u64);64],retained:Vec<(u64,usize,u64)>,vm:HashMap<u64,usize>,vms:Vec<Vm>,open:HashSet<u64>,next:u64}
    impl Model {
     fn view(&self,owner:u64)->&Vm {&self.vms[self.vm[&owner]]}
     fn publish(&mut self,owner:u64,slot:usize) {
      let (previous,id)=self.graph[slot];
      if previous==owner {return;}
      if previous!=0 && self.vm[&previous]==self.vm[&owner] {
       // Same VM: the pair is rebound without a new graph.
       self.graph[slot]=(owner,id);return;
      }
      self.next+=1;let new=self.next;
      if let Err(error)=self.view(owner).install_render_pool_mappings(slot as u8,new,1,KVec(vec![KernelMapping(new)])) {
       assert_eq!(error,EBUSY);
       panic!("queue {} (VM {}) stalls on pair {}: another graph of its VM keeps its view there until queue {:?} closes",
              owner,self.vm[&owner],slot,self.retained.iter().filter(|r|r.1==slot).map(|r|r.0).collect::<Vec<_>>());
      }
      if previous!=0 {
       if self.open.contains(&previous) {self.retained.push((previous,slot,id));}
       else {self.view(previous).clear_render_pool_mappings_if_owner(slot as u8,id);}
      }
      self.graph[slot]=(owner,new);
     }
     fn close(&mut self,owner:u64) {
      self.open.remove(&owner);
      for slot in 0..64 {
       let (o,id)=self.graph[slot];
       if o==owner {self.view(owner).clear_render_pool_mappings_if_owner(slot as u8,id);self.graph[slot]=(0,0);}
      }
      let mine:Vec<_>=self.retained.iter().filter(|r|r.0==owner).cloned().collect();
      for (o,slot,id) in mine {self.view(o).clear_render_pool_mappings_if_owner(slot as u8,id);}
      self.retained.retain(|r|r.0!=owner);
     }
    }
    fn close(s:&mut State,m:&mut Model,owner:u64) {
     for slot in 0..cfg::RENDER_SLOTS {if s.owners[slot]==owner {s.released|=1u64<<slot;}}
     for value in s.forfeited.iter_mut().filter(|v|**v==owner) {*value=0;}
     m.close(owner);
    }
    fn render(s:&mut State,m:&mut Model,owner:u64)->u8 {
     let slot=s.acquire(owner FOLLOW).unwrap_or_else(||panic!("queue {} found no pair",owner));
     m.publish(owner,slot as usize);
     slot
    }
    fn candidates(s:&State,m:&Model)->Vec<(u8,u64)> {
     (1..cfg::RENDER_SLOTS).filter(|slot|{
      let donor=s.owners[*slot];
      donor!=0 && m.open.contains(&donor) && m.graph[*slot].0==donor && s.refs[*slot]==0
       && (1..cfg::RENDER_SLOTS).filter(|other|s.owners[*other]==donor && m.graph[*other].0==donor).count()>=2
     }).map(|slot|(slot as u8,s.owners[slot])).collect()
    }
    fn main(){
     // VM 1 hosts queues 1 (donor), 32 (one pair) and later 50 (new); every other queue has its own VM.
     let (donor,sibling,late)=(1u64,32u64,50u64);
     let mut s=State::new();
     let mut m=Model{graph:[(0,0);64],retained:Vec::new(),vm:HashMap::new(),vms:Vec::new(),open:HashSet::new(),next:0};
     let vm_of=|m:&mut Model,owner:u64,shared:bool| {
      let index=if shared && !m.vms.is_empty() {0} else {
       let vm=Vm{lifetime:Some(std::sync::Arc::new(VmLifetime{driver:Mutex(std::sync::Mutex::new(None))}))};
       vm.install_driver_mappings(KVec::new(),KVec::new()).unwrap();m.vms.push(vm);m.vms.len()-1};
      m.vm.insert(owner,index);m.open.insert(owner);
     };
     for owner in 1..=32 {
      vm_of(&mut m,owner,owner==donor||owner==sibling);
      let a=render(&mut s,&mut m,owner);
      if owner<32 {let b=render(&mut s,&mut m,owner);s.release(b);}
      s.release(a);
     }
     for owner in 2..=31 {let a=render(&mut s,&mut m,owner);let b=render(&mut s,&mut m,owner);s.release(b);s.release(a);}
     let first=render(&mut s,&mut m,donor);s.release(first);
     let mut recipients=Vec::new();
     for owner in 100..140 {
      vm_of(&mut m,owner,false);
      assert_eq!(s.refusal(owner),Err(ENOSPC));
      if let Some((slot,from))=s.forfeit_sibling(&candidates(&s,&m),0,owner) {recipients.push((owner,slot as usize,from));}
     }
     let &(holder,given,_)=recipients.iter().find(|r|r.2==donor).expect("the VM's queue donates its least recently used sibling");
     for _ in 0..5 {
      for &(owner,_,_) in &recipients {let slot=render(&mut s,&mut m,owner);s.release(slot);}
      let x=render(&mut s,&mut m,donor);let y=render(&mut s,&mut m,donor);s.release(y);s.release(x);
     }
     let displaced=m.retained.iter().find(|r|r.0==donor&&r.1==given).expect("control: the donor's displaced graph keeps its view").2;
     // Only the donor's recipient exits: the given pair is the one free slot.
     close(&mut s,&mut m,holder);
     // The VM's queue with one pair claims it as its sibling and draws on both.
     for round in 0..10 {
      let x=render(&mut s,&mut m,sibling);let y=render(&mut s,&mut m,sibling);
      if round==0 {assert_eq!(y as usize,given,"control: the VM's other queue is offered the freed pair");}
      s.release(y);s.release(x);
     }
     // That queue exits; a new queue of the same VM is offered the pair.
     close(&mut s,&mut m,sibling);
     vm_of(&mut m,late,true);
     let x=render(&mut s,&mut m,late);let y=render(&mut s,&mut m,late);
     assert!(x as usize==given||y as usize==given,"control: the new queue of the VM is offered the freed pair");
     s.release(y);s.release(x);
     let view=m.view(donor);
     assert_eq!(view.render_pool_tvb_blocks(given as u8,displaced),Some(1),"the donor's displaced graph keeps its view");
     assert_eq!(view.render_pool_tvb_blocks(given as u8,m.graph[given].1),Some(1),"the new graph has its own view");
     close(&mut s,&mut m,donor);
     assert_eq!(m.view(late).render_pool_tvb_blocks(given as u8,displaced),None,"the donor's release drops only its view");
     assert_eq!(m.view(late).render_pool_tvb_blocks(given as u8,m.graph[given].1),Some(1));
     println!("shared VM: a queue of a multi-queue VM donates its sibling under held40; the VM's other queue and a new queue install and draw on the freed pair while the donor's displaced graph keeps its own view");
    }
    '''.replace(' FOLLOW', follow)
    one_view = [('if views.iter().any(|pool| pool.id == id) {', 'if !views.is_empty() {')]
    for name, edits, expected in [('fixed', [], None), ('one-view-per-pair', one_view, 'stalls on pair')]:
        current = code
        for old, new in edits:
            assert current.count(old) == 1, (name, old)
            current = current.replace(old, new)
        fixture = root / ('shared-vm-' + name + '.rs'); fixture.write_text(current)
        subprocess.run([os.environ.get('RUSTC', 'rustc'), '--edition=2021', str(fixture), '-o',
                        str(fixture.with_suffix(''))], check=True)
        result = subprocess.run([str(fixture.with_suffix(''))], capture_output=True, text=True)
        if expected is None:
            assert result.returncode == 0, result.stderr
            print(result.stdout.strip())
        else:
            assert result.returncode != 0 and expected in result.stderr, result.stderr
            print('PASS negative control:', name)

if __name__ == '__main__':
    with tempfile.TemporaryDirectory(prefix='asahi-resource-tests-') as temporary:
        admission_tests(Path(temporary), DRIVER)
        forfeit_donor_tests(Path(temporary), DRIVER)
        shared_vm_donor_tests(Path(temporary), DRIVER)
        queue_tests(Path(temporary), DRIVER)
        compute_takeover_tests(Path(temporary), DRIVER)
        compute_admission_tests(Path(temporary), DRIVER)
        lower_window_gate_tests(Path(temporary), DRIVER)
        watchdog_publication_guard_tests(DRIVER)
        shared_alias_tests(Path(temporary), DRIVER)
