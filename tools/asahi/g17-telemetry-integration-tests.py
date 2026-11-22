#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the production ring transport and C formatter without hardware."""
import argparse
from pathlib import Path
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / 'drivers/gpu/drm/asahi'


def item(source, signature):
    start = source.index(signature)
    end = source.index('{', start) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


RUST_HEADER = r'''
#![allow(dead_code)]
extern crate self as kernel;
mod prelude {
    pub type Error = i32;
    pub type Result<T = ()> = std::result::Result<T, i32>;
    pub const EIO: i32 = -5;
    pub const EAGAIN: i32 = -11;
    pub const GFP_KERNEL: u32 = 0;
    pub struct KVec<T>(Vec<T>);
    impl<T> KVec<T> {
        pub fn with_capacity(n:usize,_:u32)->Result<Self> { Ok(Self(Vec::with_capacity(n))) }
        pub fn push(&mut self,v:T,_:u32)->Result { self.0.push(v);Ok(()) }
    }
    impl<T> core::ops::Deref for KVec<T> { type Target=[T];fn deref(&self)->&[T] { &self.0 } }
    impl<T> core::ops::DerefMut for KVec<T> { fn deref_mut(&mut self)->&mut [T] { &mut self.0 } }
}
mod sync { pub use std::sync::Arc; }
mod debugfs {
    pub trait Writer { fn write(&self, f:&mut std::fmt::Formatter<'_>)->std::fmt::Result; }
}
mod device {
    pub struct Device;
    impl Device { pub fn name(&self)->&std::ffi::CStr { std::ffi::CStr::from_bytes_with_nul(b"test.gpu\0").unwrap() } }
    impl AsRef<Device> for Device { fn as_ref(&self) -> &Device { self } }
}
mod fmt { pub use std::fmt::*; }
macro_rules! dev_err { ($dev:expr, $($t:tt)*) => { let _ = ($dev, format_args!($($t)*)); }; }
mod g17 {
    use std::sync::atomic::Ordering;
    // Per test thread, so a test that moves the clock cannot race the others.
    thread_local! { static NOW: core::cell::Cell<u64> = const { core::cell::Cell::new(1) }; }
    fn now_ns() -> u64 { NOW.with(|now| now.get()) }
    #[derive(Clone, Copy, Debug)]
    enum Role { Primary = 0, Secondary = 1 }
    mod trace { pub fn report(_:u8, _:u8, _:&[u32;18]) {} pub fn message(_:u32,_:u64) {} pub const CONTROL:u32=0; }
    mod fw {
        pub mod variant {
            #[derive(Clone, Copy, Debug)]
            pub enum InitGraph { V25G83, G17P }
        }
        pub mod channels {
            pub const EVENT_WORDS:usize=18;
            pub const EVENT_RECORD_SIZE:usize=72;
            pub const RX_PRODUCER_OFFSET:usize=0x20;
            pub const TRACE_SLOTS:u32=4;
            pub const TRACE_TYPES:u32=1;
            pub const STATISTICS_SLOTS:u32=8;
            pub const STATISTICS_TYPES:u32=0x5fff0000;
            pub const DEVICE_CONTROL_SLOTS:u32=8;
            pub struct ControlRecord;
            #[derive(Clone, Copy)]
            pub enum Opcode { Dummy }
            pub type DeviceControlOp = Opcode;
            impl Opcode { pub fn asserts_gpu_power(self)->bool { false } }
            impl ControlRecord {
                pub fn bytes(&self)->&[u8] { &[] }
                pub fn opcode(&self)->Opcode { Opcode::Dummy }
                pub fn asserts_gpu_power(&self)->bool { false }
            }
        }
        // M5's statistics tags and power rule, as the family methods below read them.
        pub mod m5 {
            M5_STATISTICS_TYPES
            pub mod power { M5_ASSERTS_GPU_POWER }
        }
        pub mod initdata { pub mod status {
            pub const TRACE_STATE:u32=0;
            pub const TRACE_RING:u32=0x100;
            pub const STATS_STATE:u32=0x1000;
            pub const STATS_RING:u32=0x2000;
        } }
    }
    mod initdata {
        use super::{Role,fw::{channels::*,initdata::status::*}};
        use crate::prelude::*;
        use std::{cell::RefCell,sync::atomic::AtomicU32};
        pub struct InitData {
            pub cursors:[[AtomicU32;2];2],
            pub slots:RefCell<[[[u32;18];8];2]>,
            pub m5:bool,
        }
        impl InitData {
            pub fn new()->Self { Self { cursors:std::array::from_fn(|_|std::array::from_fn(|_|AtomicU32::new(0))), slots:RefCell::new([[[0;18];8];2]), m5:false } }
            INITDATA_FAMILY
            pub fn push(&self,role:Role,words:[u32;18]) {
                use std::sync::atomic::Ordering::Relaxed;
                let producer=self.cursors[role as usize][1].load(Relaxed);
                self.slots.borrow_mut()[role as usize][producer as usize]=words;
                self.cursors[role as usize][1].store((producer+1)%STATISTICS_SLOTS,Relaxed);
            }
            pub fn status_word(&self,role:Role,offset:usize)->Result<&AtomicU32> {
                Ok(&self.cursors[role as usize][usize::from(offset==STATS_STATE as usize+RX_PRODUCER_OFFSET)])
            }
            pub fn status_read_words<const N:usize>(&self,role:Role,offset:usize)->Result<[u32;N]> {
                let mut words=[0;N];
                words.copy_from_slice(&self.slots.borrow()[role as usize][(offset-STATS_RING as usize)/EVENT_RECORD_SIZE][..N]);
                Ok(words)
            }
            pub fn control_consumer(&self)->Result<&AtomicU32> { Ok(&self.cursors[0][0]) }
            pub fn control_producer(&self)->Result<&AtomicU32> { Ok(&self.cursors[0][1]) }
            pub fn publish_control(&self,_:u32,_:&[u8],_:bool,_:u32)->Result { Ok(()) }
        }
    }
    struct Recovery(bool);
    impl Recovery { fn pending(&self)->bool { self.0 } }
    struct Firmware {
        rings:channel::Rings,
        init:initdata::InitData,
        powered:bool,
        epoch:u64,
        recovery:Recovery,
    }
    impl Firmware {
        fn roles(&self)->&'static [Role] { &[Role::Primary,Role::Secondary] }
        fn observe_statistics_power(&mut self) {
            if !self.rings.statistics_suspended() {
                let roles=self.roles();
                self.rings.observe_statistics_power(&crate::device::Device,&self.init,roles,self.powered,self.epoch);
            }
        }
    }
    struct State(std::sync::Mutex<Option<Box<Firmware>>>);
    impl State {
        fn lock(&self)->std::sync::MutexGuard<'_,Option<Box<Firmware>>> { self.0.lock().unwrap() }
    }
    struct Variant { graph:fw::variant::InitGraph, m5_commands:bool }
    struct Shared { state:State, dev:crate::device::Device, variant:Variant, crashed:std::sync::atomic::AtomicBool }
    mod reports;
'''
RUST_TESTS = r'''
    fn reader(graph:fw::variant::InitGraph)->stats::Reader {
        stats::Reader(std::sync::Arc::new(Shared {
            state:State(std::sync::Mutex::new(Some(Box::new(Firmware {
                rings:channel::Rings::new(graph,false).unwrap(),
                init:initdata::InitData::new(),powered:true,epoch:0,recovery:Recovery(false),
            })))),dev:crate::device::Device,variant:Variant { graph, m5_commands:false },crashed:false.into(),
        }),None)
    }

    #[test]
    fn reader_drains_without_events_and_does_not_reuse_old_samples() {
        use {fw::variant::InitGraph,Role::*};
        for graph in [InitGraph::V25G83,InitGraph::G17P] {
            let reader=reader(graph);
            let sample=|tiler,renderer,device|[0x13,0,0,tiler,renderer,0,device,0,0,0,0,0,0,0,0,0,0,0];
            {
                let state=reader.0.state.lock();
                let fw=state.as_ref().unwrap();
                fw.init.push(Primary,sample(25,75,50));
                fw.init.push(Secondary,sample(35,85,65));
            }
            let view=reader.read();
            assert_eq!((view.util_status,view.tiler,view.renderer,view.device,view.util_samples),
                       (telemetry::VALID,30,80,57,2));
            assert_eq!(reader.read(),view); // Empty reads do not add samples.
            NOW.with(|now|now.set(stats::max_age_ns(graph)+1));
            assert_eq!(reader.read().util_status,telemetry::STALE);
            {
                let state=reader.0.state.lock();
                state.as_ref().unwrap().init.push(Secondary,sample(0,0,0));
            }
            let view=reader.read();
            assert_eq!((view.util_status,view.device,view.util_samples),(telemetry::VALID,0,1));
            // 26A428 reports its state in tags 0x19/0x1b, none of which arrived; 25G83 never does.
            assert_eq!(view.pstate_status,match graph { InitGraph::G17P=>telemetry::UNAVAILABLE, _=>telemetry::UNSUPPORTED });
            NOW.with(|now|now.set(1));
        }
    }

    #[test]
    fn reader_respects_suspend_recovery_power_epochs_and_detach() {
        use {fw::variant::InitGraph,Role::*};
        for graph in [InitGraph::V25G83,InitGraph::G17P] {
            let reader=reader(graph);
            let sample=[0x13,0,0,25,75,0,50,0,0,0,0,0,0,0,0,0,0,0];
            let push=||reader.0.state.lock().as_ref().unwrap().init.push(Primary,sample);
            push();
            assert_eq!(reader.read().util_status,telemetry::VALID);
            reader.suspend(true);
            push();
            assert_eq!(reader.read().util_status,telemetry::STALE);
            reader.suspend(false);
            push();
            assert_eq!(reader.read().util_status,telemetry::STALE);
            push();
            assert_eq!(reader.read().util_status,telemetry::VALID);
            reader.0.state.lock().as_mut().unwrap().epoch+=1;
            push();
            assert_eq!(reader.read().util_status,telemetry::STALE);
            push();
            assert_eq!(reader.read().util_status,telemetry::VALID);
            reader.0.state.lock().as_mut().unwrap().powered=false;
            push();
            // Gated cores run no work: the last sample stays until it ages
            // out, then the GPU reads idle. Neither is stale.
            assert_eq!(reader.read().util_status,telemetry::VALID);
            reader.0.state.lock().as_mut().unwrap().powered=true;
            assert_eq!(reader.read().util_status,telemetry::STALE);
            push();
            assert_eq!(reader.read().util_status,telemetry::VALID);
            reader.0.state.lock().as_mut().unwrap().recovery.0=true;
            push();
            assert_eq!(reader.read().util_status,telemetry::STALE);
            reader.0.state.lock().as_mut().unwrap().recovery.0=false;
            assert_eq!(reader.read().util_status,telemetry::STALE);
            push();
            assert_eq!(reader.read().util_status,telemetry::VALID);
            reader.0.crashed.store(true,Ordering::Release);
            push();
            assert_eq!(reader.read().util_status,telemetry::STALE);
            *reader.0.state.lock()=None;
            assert_eq!(reader.read().util_status,telemetry::UNAVAILABLE);
        }
    }

    // Complete V26A428 payloads; unqualified words must never become pstate.
    const HIST_IDLE:[u32;18]=[0x12,0x0dc6727d,0,8,4,1,0,3,7,0,0,1000,0,0,0,0,0,0];
    const HIST_ACTIVE:[u32;18]=[0x12,0x0003c9f5,0,10,0,1,0,3,9,0,0,1000,0,0,0,0,0,0];
    const HIST_OTHER:[u32;18]=[0x11,0x04952a22,0,0x85498734,0,1,0,9,10,0,0,1000,0,0,0,0,0,0];
    const TRANSITION:[u32;18]=[0x1a,0xf93dc8f2,0,1,0,1,0,3,9,0,0,1000,0,0,0,0,0,0];

    fn report(rings:&channel::Rings)->String {
        struct View<'a>(&'a channel::Rings);
        impl std::fmt::Display for View<'_> {
            fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result { self.0.write_reports(f) }
        }
        format!("{}",View(rings))
    }

    #[test]
    fn captured_histograms_are_preserved_without_inventing_pstate() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        for graph in [InitGraph::V25G83,InitGraph::G17P] {
            let init=initdata::InitData::new();
            let mut rings=Rings::new(graph,false).unwrap();
            for words in [HIST_IDLE,HIST_ACTIVE,HIST_OTHER,TRANSITION] {
                init.push(Secondary,words);
                rings.drain_statistics(&dev,&init,Secondary);
                // 26A428 integrates busy time from its utilisation records; none arrived.
                let busy=match graph { InitGraph::G17P=>telemetry::VALID, _=>telemetry::UNSUPPORTED };
                assert_eq!(rings.read_statistics(1),telemetry::Readout { busy_status:busy,..Default::default() });
                assert!(report(&rings).contains(&format!("words={:08x?}",words)));
            }
        }
    }

    #[test]
    fn power_cutoff_discards_only_the_old_prefix_per_role_including_wrap() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        for start in [0,5,7] {
            let init=initdata::InitData::new();
            let mut rings=Rings::new(InitGraph::G17P,false).unwrap();
            for &role in &[Primary,Secondary] {
                for cursor in &init.cursors[role as usize] { cursor.store(start,Ordering::Relaxed); }
                for words in [HIST_OTHER,HIST_ACTIVE,TRANSITION] { init.push(role,words); }
            }
            rings.observe_statistics_power(&dev,&init,&[Primary,Secondary],true,1);
            for &role in &[Primary,Secondary] {
                for words in [HIST_OTHER,HIST_ACTIVE,TRANSITION] { init.push(role,words); }
                rings.drain_statistics(&dev,&init,role);
            }
            let text=report(&rings);
            for role in [0,1] { for tag in [0x11,0x12,0x1a] {
                let line=text.lines().find(|line|line.starts_with(&format!("role={} tag={:#x} ",role,tag))).unwrap();
                assert!(line.contains("count=2 host_ns=1 admitted=1 last_admitted=true"),"{}",line);
            } }
            assert_eq!(rings.read_statistics(1),telemetry::Readout { busy_status:telemetry::VALID,..Default::default() });
        }
    }

    #[test]
    fn empty_boundary_keeps_new_qualified_records_but_late_wake_is_conservative() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        let init=initdata::InitData::new();
        // The decoder's tag 0x10 path: only a firmware without 26A428's state records decodes it.
        let mut rings=Rings::new(InitGraph::V25G83,false).unwrap();
        let util=[0x13,0,0,8,97,0,97,0,0,0,0,0,0,0,0,0,0,0];
        let pstate=[0x10,0x2500,0,0,0,0,6,0,0,0,0,0,0,0,0,0,0,0];
        rings.observe_statistics_power(&dev,&init,&[Primary,Secondary],true,1);
        init.push(Primary,util);
        init.push(Secondary,pstate);
        rings.drain_statistics(&dev,&init,Primary);
        rings.drain_statistics(&dev,&init,Secondary);
        let view=rings.read_statistics(1);
        assert_eq!((view.util_status,view.device,view.pstate_status,view.pstate),(telemetry::VALID,97,telemetry::VALID,6));
        rings.observe_statistics_power(&dev,&init,&[Primary,Secondary],false,1);
        init.push(Primary,util);
        init.push(Secondary,pstate);
        // A wake seen after records arrived cannot prove which side they belong to.
        rings.observe_statistics_power(&dev,&init,&[Primary,Secondary],true,1);
        rings.drain_statistics(&dev,&init,Primary);
        rings.drain_statistics(&dev,&init,Secondary);
        let view=rings.read_statistics(1);
        assert_eq!((view.util_status,view.pstate_status),(telemetry::STALE,telemetry::STALE));
        init.push(Primary,util);
        init.push(Secondary,pstate);
        rings.drain_statistics(&dev,&init,Primary);
        rings.drain_statistics(&dev,&init,Secondary);
        assert_eq!(rings.read_statistics(1).pstate_status,telemetry::VALID);
    }

    #[test]
    fn v25g83_reports_pstate_unsupported_and_keeps_utilisation() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        // 26A428 reports its state on the A18 Pro and on M5; 25G83 never does.
        assert!(!stats::reports_pstate(InitGraph::V25G83,false));
        assert!(stats::reports_pstate(InitGraph::G17P,false));
        assert!(stats::reports_pstate(InitGraph::G17P,true));
        let graph=InitGraph::V25G83;
        let empty=stats::qualify(graph,false,telemetry::Readout::default());
        assert_eq!((empty.util_status,empty.pstate_status),(telemetry::UNAVAILABLE,telemetry::UNSUPPORTED));
        let init=initdata::InitData::new();
        let mut rings=Rings::new(graph,false).unwrap();
        init.push(Primary,[0x13,0,0,8,97,0,97,0,0,0,0,0,0,0,0,0,0,0]);
        // Even a decodable tag 0x10 does not override the firmware's match data.
        init.push(Primary,[0x10,0x2500,0,0,0,0,6,0,0,0,0,0,0,0,0,0,0,0]);
        rings.drain_statistics(&dev,&init,Primary);
        assert_eq!(rings.read_statistics(1).pstate_status,telemetry::VALID);
        let view=stats::qualify(graph,false,rings.read_statistics(1));
        assert_eq!((view.util_status,view.device,view.pstate_status,view.pstate,view.pstate_timestamp),
                   (telemetry::VALID,97,telemetry::UNSUPPORTED,0,0));
        rings.invalidate_statistics();
        let view=stats::qualify(graph,false,rings.read_statistics(1));
        assert_eq!((view.util_status,view.pstate_status),(telemetry::STALE,telemetry::UNSUPPORTED));
    }

    #[test]
    fn a18_pro_reports_its_state_through_the_reader() {
        use {fw::variant::InitGraph,Role::*};
        // States 0 to 10; an interval record, then the state in words 1 and 3.
        let mhz=[0,338,492,618,796,928,1056,1170,1278,1338,1470];
        let reader=stats::Reader(reader(InitGraph::G17P).0,stats::StateFrequencies::new(mhz.into_iter()));
        let push=|words|reader.0.state.lock().as_ref().unwrap().init.push(Secondary,words);
        // Tag 0x10 is not this firmware's state report.
        push([0x10,0x2500,0,0,0,0,6,0,0,0,0,0,0,0,0,0,0,0]);
        assert_eq!(reader.read().pstate_status,telemetry::UNAVAILABLE);
        assert_eq!(reader.frequency(),Some((None,1_470_000_000)));
        push([0x19,0x2000,0x40,0x1000,0x40,0,0,0,0,0,0,0,0,0,0,0,0,0]);
        push([0x1b,7,0,7,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
        let view=reader.read();
        assert_eq!((view.pstate_status,view.pstate,view.pstate_timestamp,view.achieved_frequency_hz),
                   (telemetry::VALID,7,0x40_0000_2000,1_170_000_000));
        assert_eq!(reader.frequency(),Some((Some(1_170_000_000),1_470_000_000)));
        // A state beyond the table is not a state.
        push([0x1b,11,0,11,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
        assert_eq!(reader.read().pstate_status,telemetry::UNAVAILABLE);
        push([0x1b,10,0,10,0,0,0,0,0,0,0,0,0,0,0,0,0,0]);
        assert_eq!(reader.read().achieved_frequency_hz,1_470_000_000);
        // Gated cores run at no state: off, 0 Hz.
        reader.0.state.lock().as_mut().unwrap().powered=false;
        let view=reader.read();
        assert_eq!((view.pstate_status,view.pstate,view.achieved_frequency_hz),(telemetry::VALID,0,0));
        assert_eq!(reader.frequency(),Some((Some(0),1_470_000_000)));
        // Power-up starts a new epoch: no state until the firmware reports one.
        reader.0.state.lock().as_mut().unwrap().powered=true;
        assert_eq!(reader.read().pstate_status,telemetry::STALE);
        assert_eq!(reader.frequency(),Some((None,1_470_000_000)));
    }

    #[test]
    fn one_drain_is_one_utilisation_delivery() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        let init=initdata::InitData::new();
        let mut rings=Rings::new(InitGraph::V25G83,false).unwrap();
        let util=[0x13,0,0,8,97,0,97,0,0,0,0,0,0,0,0,0,0,0];
        // Three 8 ms records per producer update, one update every 24 ms.
        for n in 0..3u64 {
            NOW.with(|now| now.set(1_000+n*24_000_000));
            for _ in 0..3 { init.push(Primary,util); }
            rings.drain_statistics(&dev,&init,Primary);
        }
        let last=1_000+2*24_000_000;
        assert!(report(&rings).contains("delivery_intervals_ns=[Some(24000000), Some(24000000)]"),"{}",report(&rings));
        assert_eq!(rings.read_statistics(last+16_000_000).util_status,telemetry::VALID);
        assert_eq!(rings.read_statistics(last+39_999_999).util_status,telemetry::VALID);
        assert_eq!(rings.read_statistics(last+40_000_000).util_status,telemetry::STALE);
    }

    #[test]
    fn off_drains_and_invalid_boundary_producers_never_admit_live_data() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        let init=initdata::InitData::new();
        let mut rings=Rings::new(InitGraph::G17P,false).unwrap();
        rings.observe_statistics_power(&dev,&init,&[Primary,Secondary],false,0);
        init.push(Secondary,HIST_IDLE);
        rings.drain_statistics(&dev,&init,Secondary);
        assert!(report(&rings).contains("admitted=0 last_admitted=false"));
        init.cursors[1][1].store(fw::channels::STATISTICS_SLOTS,Ordering::Relaxed);
        rings.observe_statistics_power(&dev,&init,&[Primary,Secondary],true,1);
        init.cursors[1][1].store(1,Ordering::Relaxed);
        init.push(Primary,[0x13,0,0,100,100,0,100,0,0,0,0,0,0,0,0,0,0,0]);
        rings.drain_statistics(&dev,&init,Primary);
        assert_eq!(rings.read_statistics(1).util_status,telemetry::UNAVAILABLE);
        assert!(report(&rings).contains("quarantine=[[false, false], [false, true]]"));
    }

    #[test]
    fn debugfs_reads_do_not_change_pending_cutoffs_or_admission() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let graph=InitGraph::G17P;
        let dev=crate::device::Device;
        let init=initdata::InitData::new();
        let mut rings=Rings::new(graph,false).unwrap();
        init.push(Secondary,HIST_IDLE);
        rings.observe_statistics_power(&dev,&init,&[Primary,Secondary],true,1);
        init.push(Secondary,HIST_ACTIVE);
        let shared=std::sync::Arc::new(Shared { state:State(std::sync::Mutex::new(Some(Box::new(Firmware {
            rings,init:initdata::InitData::new(),powered:true,epoch:0,recovery:Recovery(false),
        })))),dev,variant:Variant { graph, m5_commands:false },crashed:false.into() });
        let view=reports::View(shared.clone());
        struct View<'a>(&'a reports::View);
        impl std::fmt::Display for View<'_> {
            fn fmt(&self,f:&mut std::fmt::Formatter<'_>)->std::fmt::Result { crate::debugfs::Writer::write(self.0,f) }
        }
        let before=format!("{}",View(&view));
        for _ in 0..10 { assert_eq!(format!("{}",View(&view)),before); }
        let mut state=shared.state.lock();
        let firmware=state.as_mut().unwrap();
        firmware.rings.drain_statistics(&shared.dev,&init,Secondary);
        assert!(report(&firmware.rings).contains("admitted=1 last_admitted=true"));
    }
    #[test]
    fn roles_merge_and_any_statistics_quarantine_disables_both() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        let init=initdata::InitData::new();
        let mut rings=Rings::new(InitGraph::G17P,false).unwrap();
        let sample=|ta,render,device|[0x13,0,0,ta,render,0xdeadbeef,device,0,0,0,0,0,0,0,0,0,0,0];
        init.push(Primary,sample(25,75,50));
        init.push(Secondary,sample(35,85,65));
        rings.drain_statistics(&dev,&init,Primary);
        rings.drain_statistics(&dev,&init,Secondary);
        let view=rings.read_statistics(1);
        assert_eq!((view.tiler,view.renderer,view.device,view.util_samples),(30,80,57,2));
        assert_eq!(view.util_status,telemetry::VALID);
        init.cursors[1][1].store(fw::channels::STATISTICS_SLOTS,Ordering::Relaxed);
        rings.drain_statistics(&dev,&init,Secondary);
        assert_eq!(rings.read_statistics(1).util_status,telemetry::STALE);
        init.cursors[1][1].store(1,Ordering::Relaxed);
        for _ in 0..3 {
            init.push(Primary,sample(100,100,100));
            rings.drain_statistics(&dev,&init,Primary);
        }
        assert_eq!(rings.read_statistics(1).util_status,telemetry::STALE);
    }
    #[test]
    fn suspend_gating_and_restart_discard_queued_epoch_records() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        let init=initdata::InitData::new();
        let mut rings=Rings::new(InitGraph::V25G83,false).unwrap();
        let words=[0x13,0,0,25,75,0,50,0,0,0,0,0,0,0,0,0,0,0];
        init.push(Primary,words);
        rings.drain_statistics(&dev,&init,Primary);
        assert_eq!(rings.read_statistics(1).util_status,telemetry::VALID);
        for boundary in 0..4 {
            init.push(Primary,words); // Unread at the epoch boundary.
            match boundary {
                0=> { rings.suspend_statistics(true); rings.drain_statistics(&dev,&init,Primary); rings.suspend_statistics(false); init.push(Primary,words); }
                1=>rings.observe_statistics_power(&dev,&init,&[Primary],false,0),
                2=>rings.observe_statistics_power(&dev,&init,&[Primary],true,1), // Wake observed by a fast publisher.
                _=>rings.invalidate_statistics(), // Restart handshake.
            }
            // A power gate reads idle (valid); every other boundary is stale.
            let expect=if boundary==1 {telemetry::VALID} else {telemetry::STALE};
            assert_eq!(rings.read_statistics(1).util_status,expect);
            rings.drain_statistics(&dev,&init,Primary);
            assert_eq!(rings.read_statistics(1).util_status,expect);
            rings.observe_statistics_power(&dev,&init,&[Primary],true,u64::from(boundary >= 2));
            init.push(Primary,words);
            rings.drain_statistics(&dev,&init,Primary);
            assert_eq!(rings.read_statistics(1).util_status,telemetry::VALID);
        }
    }
    #[test]
    fn pstate_uses_drain_order_without_role_deduplication() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        let init=initdata::InitData::new();
        let mut rings=Rings::new(InitGraph::V25G83,false).unwrap();
        init.push(Primary,[0x10,0x1000,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0]);
        init.push(Secondary,[0x10,0x2500,0,0,0,0,6,0,0,0,0,0,0,0,0,0,0,0]);
        rings.drain_statistics(&dev,&init,Primary);
        rings.drain_statistics(&dev,&init,Secondary);
        assert_eq!(rings.read_statistics(1).pstate,6);
        init.push(Primary,[0x10,0x1000,0,0,0,0,4,0,0,0,0,0,0,0,0,0,0,0]);
        rings.drain_statistics(&dev,&init,Primary);
        assert_eq!(rings.read_statistics(1).pstate_status,telemetry::UNAVAILABLE);
    }
}
'''

C_HEADER = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint32_t u32;
typedef uint64_t u64;
#define PAGE_SIZE 4096
#define READ_ONCE(x) (x)
#define container_of(ptr,type,member) ((type*)((char*)(ptr)-offsetof(type,member)))
struct device {};
struct device_attribute {
    struct { const char *name; unsigned mode; } attr;
    ssize_t (*show)(struct device*,struct device_attribute*,char*);
    ssize_t (*store)(struct device*,struct device_attribute*,const char*,size_t);
};
struct notifier_block { int (*notifier_call)(struct notifier_block*,unsigned long,void*); };
#define PM_SUSPEND_PREPARE 1
#define PM_HIBERNATION_PREPARE 2
#define PM_RESTORE_PREPARE 3
#define PM_POST_SUSPEND 4
#define PM_POST_HIBERNATION 5
#define PM_POST_RESTORE 6
#define NOTIFY_OK 1
#define ENODEV 19
#define ENOMEM 12
#define EINVAL 22
#define PAGE_SHIFT 12
static u32 submitted_pct;
static unsigned int limit_writes;
static void asahi_set_mem_limit_pct(u32 pct) {
    submitted_pct=pct;
    limit_writes++;
}
#define DEFINE_MUTEX(n) int n
#define mutex_lock(m) ((void)(m))
#define mutex_unlock(m) ((void)(m))
static unsigned long totalram_pages(void) { return 1UL<<18; }
static u64 div64_u64(u64 a,u64 b) { return a/b; }
static int kstrtou32(const char *s,unsigned base,u32 *res) {
    char *end;unsigned long v=strtoul(s,&end,base);
    if(end==s || (*end && !(*end=='\n' && !end[1]))) return -EINVAL;
    *res=(u32)v;return 0;
}
#define BUILD_BUG_ON(x) _Static_assert(!(x),#x)
#define kzalloc_obj(x) calloc(1,sizeof(x))
#define sysfs_attr_init(x) ((void)0)
#define kfree(x) free(x)
static int registered, removed, created, fail_file, fail_notifier, suspend_calls;
static int register_pm_notifier(struct notifier_block *nb) { (void)nb; if(fail_notifier)return -12; registered++; return 0; }
static int unregister_pm_notifier(struct notifier_block *nb) { (void)nb; removed++; return 0; }
static int device_create_file(struct device *dev,struct device_attribute *attr) { (void)dev;(void)attr; return ++created==fail_file?-12:0; }
static void device_remove_file(struct device *dev,struct device_attribute *attr) { (void)dev;(void)attr; }
static int emit(char *buf,size_t size,const char *fmt,...) {
    va_list ap;va_start(ap,fmt);int n=vsnprintf(buf,size,fmt,ap);va_end(ap);return n<(int)size?n:(int)size-1;
}
#define scnprintf emit
#define sysfs_emit(buf,...) emit(buf,PAGE_SIZE,__VA_ARGS__)
#define sysfs_emit_at(buf,at,...) emit(buf+at,PAGE_SIZE-(at),__VA_ARGS__)
static ssize_t gpu_memory_show(struct device *dev,struct device_attribute *attr,char *buf) { (void)dev;(void)attr;(void)buf;return 0; }
static u32 *asahi_idle_word;
static struct device_attribute dev_attr_idle_off_delay_ms;
'''
C_TESTS = r'''
static struct asahi_qualified_stats current;
static void read_stats(const void *ptr,struct asahi_qualified_stats *out) { (void)ptr;*out=current; }
static void suspend_stats(const void *ptr,bool suspended) { (void)ptr;suspend_calls+=suspended?1:10; }
static bool read_frequency(const void *ptr,u32 pstate,u64 *hz) {
    static const u64 table[]={0,338000000,618000000,492000000};
    (void)ptr;
    if(pstate>=sizeof(table)/sizeof(*table))return false;
    *hz=table[pstate];return true;
}
int main(void) {
    struct asahi_stats_snapshot snap;
    struct asahi_stats_attribute stats={.snapshot=&snap,.export_enabled=true};
    char old[PAGE_SIZE],now[PAGE_SIZE];
    for(unsigned seed=0;seed<512;seed++) {
        snap=(struct asahi_stats_snapshot){
            .util1=seed, .util2=seed+11, .util3=~seed, .util4=seed*0x01010101u,
            .pstate=seed%16, .avg_power_mw=seed*1234567u,
            .temperature_raw=seed^0xff00aa11u, .temperature_scale=seed+37,
            .temperature_tmin=seed+42, .temperature_tmax=seed+73,
            .busy_ns=((u64)seed<<32)|(UINT32_MAX-seed),
            .jobs=UINT64_MAX-(u64)seed*101,
        };
        ssize_t before=old_agx_stats_show(NULL,&stats.attr,old);
        ssize_t after=agx_stats_show(NULL,&stats.attr,now);
        assert(before==after && !memcmp(old,now,before+1));
        stats.frequency=read_frequency;
        after=agx_stats_show(NULL,&stats.attr,now);
        assert(after>before && !memcmp(old,now,before));
        static const char *expected[]={"0", "338000000", "618000000", "492000000"};
        char suffix[128];
        snprintf(suffix,sizeof(suffix),"requested_frequency_hz %s\n",
                 snap.pstate<4?expected[snap.pstate]:"unsupported");
        assert(!strcmp(now+before,suffix));
        stats.frequency=NULL;
    }
    stats.export_enabled=false;
    assert(agx_stats_show(NULL,&stats.attr,now)==12);
    assert(!strcmp(now,"unsupported\n"));
    stats.export_enabled=true;stats.read=read_stats;
    for(unsigned status=0;status<3;status++) {
        current=(struct asahi_qualified_stats){.util_status=status,.pstate_status=status,.tiler=30,.renderer=80,.device=57,.pstate=6};
        agx_stats_show(NULL,&stats.attr,now);
        assert(strstr(now,"busy_ns unsupported\n"));
        assert(strstr(now,"power_mw unsupported\n"));
        assert(strstr(now,"temperature_raw unsupported\n"));
        assert(strstr(now,"requested_frequency_hz unsupported\n"));
        assert(strstr(now,"achieved_frequency_hz unsupported\n"));
        if(status==1) {
            assert(strstr(now,"util_tiler_percent 30\nutil_renderer_percent 80\nutil_device_percent 57\n"));
            assert(strstr(now,"pstate 6\n"));
        } else {
            assert(strstr(now,"util_device_percent unsupported\n"));
            assert(strstr(now,"pstate unsupported\n"));
            assert(strstr(now,status==2?"pstate_status stale\n":"pstate_status unavailable\n"));
            assert(strstr(now,status==2?"utilisation_status stale\n":"utilisation_status unavailable\n"));
        }
    }
    /* 26A428 reports the state the GPU runs at; state 0 (gated cores) runs at 0 Hz. */
    current=(struct asahi_qualified_stats){.util_status=1,.pstate_status=1,.pstate=10,.achieved_frequency_hz=1470000000};
    agx_stats_show(NULL,&stats.attr,now);
    assert(strstr(now,"pstate_status valid\npstate 10\nrequested_frequency_hz unsupported\nachieved_frequency_hz 1470000000\n"));
    current.pstate=0;current.achieved_frequency_hz=0;
    agx_stats_show(NULL,&stats.attr,now);
    assert(strstr(now,"pstate 0\nrequested_frequency_hz unsupported\nachieved_frequency_hz 0\n"));
    current.pstate_status=2;
    agx_stats_show(NULL,&stats.attr,now);
    assert(strstr(now,"pstate unsupported\nrequested_frequency_hz unsupported\nachieved_frequency_hz unsupported\n"));
    /* G16S: pstate unsupported by firmware, utilisation still valid. */
    current=(struct asahi_qualified_stats){.util_status=1,.pstate_status=3,.tiler=30,.renderer=80,.device=57,.pstate=0,
                                          .util_samples=19,.util_host_ns=123};
    agx_stats_show(NULL,&stats.attr,now);
    assert(strstr(now,"utilisation_status valid\nutil_tiler_percent 30\nutil_renderer_percent 80\nutil_device_percent 57\n"
                      "pstate_status unsupported\npstate unsupported\nrequested_frequency_hz unsupported\n"));
    assert(strstr(now,"utilisation_source firmware-statistics-tag-0x13\nutilisation_samples 19\nutilisation_host_ns 123\n"));
    struct device dev;
    struct asahi_gpu_memory memory={0};
    void *handle=NULL;
    assert(!asahi_sysfs_register(&dev,&snap,1,&memory,&handle,read_stats,suspend_stats,NULL));
    struct asahi_stats_attribute *registered_stats=handle;
    assert(registered==1 && removed==0);
    asahi_stats_pm_notify(&registered_stats->pm_notifier,PM_SUSPEND_PREPARE,NULL);
    asahi_stats_pm_notify(&registered_stats->pm_notifier,PM_POST_SUSPEND,NULL);
    assert(suspend_calls==11);
    asahi_sysfs_unregister(&dev,handle);
    assert(removed==1);
    struct asahi_gpu_memory limits={.limit=536870912,.client_limit=536870912};
    assert(!asahi_sysfs_register(&dev,&snap,1,&limits,&handle,NULL,NULL,NULL));
    struct asahi_stats_attribute *limit_stats=handle;
    char pct[16];
    assert(limit_stats->limit_attr.show(&dev,&limit_stats->limit_attr,pct)==3 && !strcmp(pct,"50\n"));
    assert(limit_stats->limit_attr.store(&dev,&limit_stats->limit_attr,"99\n",3)==3);
    assert(limit_writes==1 && submitted_pct==99);
    assert(limit_stats->limit_attr.store(&dev,&limit_stats->limit_attr,"100",3)==3);
    assert(limit_writes==2 && submitted_pct==100);
    assert(limit_stats->limit_attr.store(&dev,&limit_stats->limit_attr,"abc",3)==-EINVAL);
    assert(limit_writes==2);
    assert(limit_stats->limit_attr.store(&dev,&limit_stats->limit_attr,"0",1)==1);
    assert(limit_writes==3 && submitted_pct==0);
    assert(limits.limit==536870912 && limits.client_limit==536870912);
    limits.client_limit=0;
    assert(limit_stats->limit_attr.show(&dev,&limit_stats->limit_attr,pct)==2 && !strcmp(pct,"0\n"));
    asahi_sysfs_unregister(&dev,handle);
    for(int failure=1;failure<=2;failure++) {
        created=0;fail_file=failure;handle=NULL;
        assert(asahi_sysfs_register(&dev,&snap,1,&memory,&handle,read_stats,suspend_stats,NULL)==-12);
        assert(registered==removed);
        assert(handle==NULL);
    }
    fail_file=0;fail_notifier=1;created=0;
    assert(asahi_sysfs_register(&dev,&snap,1,&memory,&handle,read_stats,suspend_stats,NULL)==-12);
    assert(created==0 && registered==removed);
    puts("PASS: 512 byte-identical legacy prefixes and frequency lookups, G17 capabilities, PM lifecycle and registration rollback");
}
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--baseline', default='ebcf9cb777a434867e9d880892ee2b46887728bb',
                    help='revision containing the legacy formatter')
    ap.add_argument('--mutation', choices=['fields', 'range', 'timestamp', 'age', 'delivery',
                                           'quarantine', 'epoch', 'cutoff', 'whole-batch', 'legacy-format', 'read-drain'])
    args = ap.parse_args()
    work = args.out or Path(tempfile.mkdtemp(prefix='asahi-g17-telemetry-'))
    work.mkdir(parents=True, exist_ok=True)
    work = work.resolve()
    channel = (DRIVER / 'g17/channel.rs').read_text().split('/// One bounded event-ring pass.')[0]
    # Compile the production transport and decoder; only MMIO/shared memory,
    # kernel logging and allocation adapters are replaced by host fixtures.
    channel = '\n'.join(line for line in channel.splitlines() if not line.startswith('//!'))
    (work / 'channel.rs').write_text(channel)
    reports = (DRIVER / 'g17/reports.rs').read_text()
    store = reports.split('/// The view owns the session')[0]
    tests = reports.split('#[kernel::macros::kunit_tests(asahi_g17_reports)]')[1]
    view = reports[reports.index('/// The view owns the session'):reports.index('fn write_power(')]
    power_stub = '''
fn write_power(_: &super::Firmware, f:&mut kernel::fmt::Formatter<'_>)->kernel::fmt::Result {
    writeln!(f,"power fixture")
}
'''
    reports = '\n'.join(line for line in store.splitlines() if not line.startswith('//!')) + view + power_stub + tests
    (work / 'reports.rs').write_text(reports)
    telemetry = (DRIVER / 'g17/telemetry.rs').read_text()
    changes = {
        'fields': ('[words[3], words[4], words[6]]', '[words[3], words[4], words[5]]'),
        'range': ('value > 100', 'value > 101'),
        'timestamp': ('u64::from(words[1]) | (u64::from(words[2]) << 32)',
                      'u64::from(words[2]) | (u64::from(words[1]) << 32)'),
        'age': ('now_ns - received >= limit', 'now_ns - received > limit'),
        'delivery': ('[Some(a), Some(b)] => a.min(b),', '[Some(a), Some(b)] => a.max(b),'),
        'quarantine': ('.any(|rings| rings[ReportRing::Statistics as usize])', '.any(|_| false)'),
        'epoch': ('self.discard_statistics = [true; 2]', 'self.discard_statistics = [false; 2]'),
        'cutoff': ('accept_statistics && statistics_cutoff.is_none()', 'accept_statistics'),
        'whole-batch': ('            self.telemetry.invalidate();', '            self.invalidate_statistics();'),
    }
    if args.mutation in changes:
        before, after = changes[args.mutation]
        if args.mutation in ('quarantine', 'epoch', 'cutoff', 'whole-batch'):
            assert before in channel
            channel = channel.replace(before, after, 1)
            (work / 'channel.rs').write_text(channel)
        else:
            assert before in telemetry
            telemetry = telemetry.replace(before, after, 1)
    (work / 'telemetry.rs').write_text(telemetry)
    stats_rs = (DRIVER / 'g17/stats.rs').read_text()
    if args.mutation == 'read-drain':
        assert 'for &role in firmware.roles()' in stats_rs
        stats_rs = stats_rs.replace('for &role in firmware.roles()', 'for &role in &[]', 1)
    policy = '\n'.join(item(stats_rs, signature) for signature in
                       ['pub(super) const fn max_age_ns', 'pub(super) const fn protocol(', 'pub(super) const fn reports_pstate',
                        'pub(super) fn qualify('])
    # The rings take the family's statistics tags and control power rule from
    # InitData; build those production methods into the fixture's InitData.
    initdata = (DRIVER / 'g17/initdata.rs').read_text()
    family = '\n'.join(item(initdata, signature).replace('pub(super) ', 'pub ', 1) for signature in
                       ['pub(super) fn statistics_types(', 'pub(super) fn control_asserts_gpu_power('])
    m5 = (DRIVER / 'g17/fw/m5/mod.rs').read_text()
    m5_types = next(x for x in m5.splitlines() if 'const STATISTICS_TYPES:' in x).replace('pub(crate)', 'pub')
    m5_power = item((DRIVER / 'g17/fw/m5/power.rs').read_text(),
                    'pub(crate) const fn asserts_gpu_power(').replace('pub(crate)', 'pub', 1)
    header = (RUST_HEADER.replace('M5_STATISTICS_TYPES', m5_types).replace('M5_ASSERTS_GPU_POWER', m5_power)
              .replace('INITDATA_FAMILY', family))

    reader_rs = ('#[derive(Clone, Copy, Debug, PartialEq, Eq)]\n' + item(stats_rs, 'pub(crate) struct StateFrequencies')
                 + '\n' + item(stats_rs, 'impl StateFrequencies') + '\n'
                 + 'pub(crate) struct Reader(pub(super) Arc<Shared>, pub(super) Option<StateFrequencies>);\n'
                 + item(stats_rs, 'impl Reader'))
    (work / 'rings.rs').write_text(header + 'mod stats { use super::{fw::variant::InitGraph, telemetry::Readout, Shared};\n'
                                  + 'use std::sync::{Arc,atomic::Ordering};\n' + reader_rs + '\n'
                                  + policy + '\n}\nmod telemetry;\nmod channel;\n' + RUST_TESTS)
    # Rust resolves child modules relative to the inline parent's directory.
    (work / 'g17').mkdir(exist_ok=True)
    for name in ['channel.rs', 'telemetry.rs', 'reports.rs']:
        (work / 'g17' / name).write_text((work / name).read_text())
    subprocess.run(['rustc', '--edition=2021', '--test', str(work / 'rings.rs'), '-o', str(work / 'rings')], check=True)
    subprocess.run([str(work / 'rings')], check=True)
    source = (DRIVER / 'sysfs.c').read_text()
    if args.mutation == 'legacy-format':
        source = source.replace('READ_ONCE(snap->util1)', 'READ_ONCE(snap->util2)', 1)
    header = (DRIVER / 'sysfs.h').read_text()
    baseline = subprocess.check_output(['git', '-C', str(ROOT), 'show', args.baseline + ':drivers/gpu/drm/asahi/sysfs.c'], text=True)
    pieces = [C_HEADER]
    for signature in ['struct asahi_qualified_stats']:
        pieces.append(item(header, signature) + ';')
    pieces += [line for line in header.splitlines() if line.startswith('typedef')]
    for signature in ['struct asahi_stats_snapshot', 'struct asahi_gpu_memory', 'struct asahi_stats_attribute']:
        pieces.append(item(source, signature) + ';')
    pieces.append(item(baseline, 'static ssize_t agx_stats_show').replace('agx_stats_show(', 'old_agx_stats_show(', 1))
    pieces += [line for line in source.splitlines() if line.startswith('static DEFINE_MUTEX(asahi_limit_lock)')]
    for signature in ['static const char *asahi_stats_quality', 'static ssize_t agx_qualified_stats_show',
                      'static int asahi_stats_pm_notify', 'static ssize_t agx_stats_show',
                      'static ssize_t gpu_mem_limit_pct_show', 'static ssize_t gpu_mem_limit_pct_store',
                      'int asahi_sysfs_register(', 'void asahi_sysfs_unregister(']:
        pieces.append(item(source, signature))
    pieces.append(C_TESTS)
    (work / 'formatter.c').write_text('\n'.join(pieces))
    subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror', '-Wno-unused-parameter',
                    str(work / 'formatter.c'), '-o', str(work / 'formatter')], check=True)
    subprocess.run([str(work / 'formatter')], check=True)


if __name__ == '__main__':
    try:
        main()
    except subprocess.CalledProcessError as error:
        print(f'child exit: {error.returncode}')
        raise SystemExit(error.returncode if error.returncode > 0 else 128 - error.returncode)
