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
    pub type Result<T = ()> = std::result::Result<T, i32>;
    pub const EIO: i32 = -5;
    pub const EAGAIN: i32 = -11;
}
mod device {
    pub struct Device;
    impl AsRef<Device> for Device { fn as_ref(&self) -> &Device { self } }
}
mod fmt { pub use std::fmt::*; }
macro_rules! dev_err { ($dev:expr, $($t:tt)*) => { let _ = ($dev, format_args!($($t)*)); }; }
mod g17 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static NOW: AtomicU64 = AtomicU64::new(1);
    fn now_ns() -> u64 { NOW.load(Ordering::Relaxed) }
    #[derive(Clone, Copy, Debug)]
    enum Role { Primary = 0, Secondary = 1 }
    mod trace { pub fn report(_:u8, _:u8, _:&[u32;18]) {} pub fn message(_:u32,_:u64) {} pub const CONTROL:u32=0; }
    mod fw {
        pub mod variant {
            #[derive(Clone, Copy)]
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
            impl Opcode { pub fn asserts_gpu_power(self)->bool { false } }
            impl ControlRecord {
                pub fn bytes(&self)->&[u8] { &[] }
                pub fn opcode(&self)->Opcode { Opcode::Dummy }
                pub fn asserts_gpu_power(&self)->bool { false }
            }
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
        }
        impl InitData {
            pub fn new()->Self { Self { cursors:std::array::from_fn(|_|std::array::from_fn(|_|AtomicU32::new(0))), slots:RefCell::new([[[0;18];8];2]) } }
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
    mod reports {
        use super::Role;
        pub struct Reports;
        impl Reports {
            pub fn new()->crate::prelude::Result<Self> { Ok(Self) }
            pub fn record(&mut self,_:Role,_:[u32;18],_:u64) {}
            pub fn write(&self,_:&mut crate::fmt::Formatter<'_>)->crate::fmt::Result { Ok(()) }
        }
    }
'''
RUST_TESTS = r'''
    #[test]
    fn roles_merge_and_any_statistics_quarantine_disables_both() {
        use {channel::Rings,fw::variant::InitGraph,Role::*};
        let dev=crate::device::Device;
        let init=initdata::InitData::new();
        let mut rings=Rings::new(InitGraph::G17P).unwrap();
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
        let mut rings=Rings::new(InitGraph::V25G83).unwrap();
        let words=[0x13,0,0,25,75,0,50,0,0,0,0,0,0,0,0,0,0,0];
        init.push(Primary,words);
        rings.drain_statistics(&dev,&init,Primary);
        assert_eq!(rings.read_statistics(1).util_status,telemetry::VALID);
        for boundary in 0..4 {
            init.push(Primary,words); // Unread at the epoch boundary.
            match boundary {
                0=> { rings.suspend_statistics(true); rings.drain_statistics(&dev,&init,Primary); rings.suspend_statistics(false); init.push(Primary,words); }
                1=>rings.observe_statistics_power(false,0),
                2=>rings.observe_statistics_power(true,1), // Wake observed by a fast publisher.
                _=>rings.invalidate_statistics(), // Restart handshake.
            }
            assert_eq!(rings.read_statistics(1).util_status,telemetry::STALE);
            rings.drain_statistics(&dev,&init,Primary);
            assert_eq!(rings.read_statistics(1).util_status,telemetry::STALE);
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
        let mut rings=Rings::new(InitGraph::G17P).unwrap();
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
        }
    }
    struct device dev;
    struct asahi_gpu_memory memory={0};
    void *handle=NULL;
    assert(!asahi_sysfs_register(&dev,&snap,1,&memory,&handle,read_stats,suspend_stats));
    struct asahi_stats_attribute *registered_stats=handle;
    assert(registered==1 && removed==0);
    asahi_stats_pm_notify(&registered_stats->pm_notifier,PM_SUSPEND_PREPARE,NULL);
    asahi_stats_pm_notify(&registered_stats->pm_notifier,PM_POST_SUSPEND,NULL);
    assert(suspend_calls==11);
    asahi_sysfs_unregister(&dev,handle);
    assert(removed==1);
    for(int failure=1;failure<=2;failure++) {
        created=0;fail_file=failure;handle=NULL;
        assert(asahi_sysfs_register(&dev,&snap,1,&memory,&handle,read_stats,suspend_stats)==-12);
        assert(registered==removed);
        assert(handle==NULL);
    }
    fail_file=0;fail_notifier=1;created=0;
    assert(asahi_sysfs_register(&dev,&snap,1,&memory,&handle,read_stats,suspend_stats)==-12);
    assert(created==0 && registered==removed);
    puts("PASS: 512 byte-identical legacy outputs, G17 capabilities, PM lifecycle and registration rollback");
}
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--out', type=Path)
    ap.add_argument('--baseline', default='ebcf9cb777a434867e9d880892ee2b46887728bb',
                    help='revision containing the legacy formatter')
    ap.add_argument('--mutation', choices=['fields', 'range', 'timestamp', 'age',
                                           'quarantine', 'epoch', 'legacy-format'])
    args = ap.parse_args()
    work = args.out or Path(tempfile.mkdtemp(prefix='asahi-g17-telemetry-'))
    work.mkdir(parents=True, exist_ok=True)
    work = work.resolve()
    channel = (DRIVER / 'g17/channel.rs').read_text().split('/// One bounded event-ring pass.')[0]
    # Compile the production transport and decoder; only MMIO/shared memory,
    # kernel logging and allocation adapters are replaced by host fixtures.
    channel = '\n'.join(line for line in channel.splitlines() if not line.startswith('//!'))
    (work / 'channel.rs').write_text(channel)
    telemetry = (DRIVER / 'g17/telemetry.rs').read_text()
    changes = {
        'fields': ('[words[3], words[4], words[6]]', '[words[3], words[4], words[5]]'),
        'range': ('value > 100', 'value > 101'),
        'timestamp': ('u64::from(words[1]) | (u64::from(words[2]) << 32)',
                      'u64::from(words[2]) | (u64::from(words[1]) << 32)'),
        'age': ('now_ns - received >= max_age_ns', 'now_ns - received > max_age_ns'),
        'quarantine': ('.any(|rings| rings[ReportRing::Statistics as usize])', '.any(|_| false)'),
        'epoch': ('self.discard_statistics = [true; 2]', 'self.discard_statistics = [false; 2]'),
    }
    if args.mutation in changes:
        before, after = changes[args.mutation]
        if args.mutation in ('quarantine', 'epoch'):
            assert before in channel
            channel = channel.replace(before, after, 1)
            (work / 'channel.rs').write_text(channel)
        else:
            assert before in telemetry
            telemetry = telemetry.replace(before, after, 1)
    (work / 'telemetry.rs').write_text(telemetry)
    policy = item((DRIVER / 'g17/stats.rs').read_text(), 'pub(super) const fn max_age_ns')
    (work / 'rings.rs').write_text(RUST_HEADER + 'mod stats { use super::fw::variant::InitGraph;\n'
                                  + policy + '\n}\nmod telemetry;\nmod channel;\n' + RUST_TESTS)
    # Rust resolves child modules relative to the inline parent's directory.
    (work / 'g17').mkdir(exist_ok=True)
    for name in ['channel.rs', 'telemetry.rs']:
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
    for signature in ['static const char *asahi_stats_quality', 'static ssize_t agx_qualified_stats_show',
                      'static int asahi_stats_pm_notify', 'static ssize_t agx_stats_show',
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
