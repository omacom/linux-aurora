#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Actual native G15 ADT parser/admission and RTKit constructor on public synthetic inputs.

Uses the production ADT byte parser, performance parser, region admission,
T6030 firmware rows and RTKit constructor. Resource/owner lookup and RTKit init
are models. It does not read firmware, a device or private machine data.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
G15 = ROOT / 'drivers/gpu/drm/asahi/g15'


def item(s, marker):
    a = s.index(marker); i = s.index('{', a) + 1; depth = 1
    while depth:
        depth += (s[i] == '{') - (s[i] == '}'); i += 1
    return s[a:i]


PREFIX = r'''
#![allow(dead_code, unused_variables)]
use std::{cell::Cell, ffi::{c_void, CStr}, ops::Range, sync::Arc};
type Result<T = ()> = std::result::Result<T, i32>;
const EINVAL: i32 = -22;
macro_rules! dev_err { ($($x:tt)*) => {{}}; }
#[macro_export] macro_rules! c_str { ($s:literal) => { match CStr::from_bytes_with_nul(concat!($s, "\0").as_bytes()) { Ok(s) => s, Err(_) => panic!("bad C string") } }; }
extern crate self as kernel;
pub mod str { pub use std::ffi::CStr; }
mod device { pub struct Device; impl Device { pub fn as_raw(&self) -> *mut c_void { 1usize as *mut c_void } } use std::ffi::c_void; }
const PERF_STATES: usize = 14;
const CLUSTERS: usize = 4;
const REGION_NAMES: [&str; 6] = ["ttbs", "tables", "handoff", "shared-l2", "text", "data"];
const BASE: [u64; 6] = [0x100000000, 0x100100000, 0x100200000, 0x100300000, 0x200000000, 0x20005c000];
const SIZES: [u64; 6] = [0x4000, 0x40000, 0x4000, 0x4000, 0x5c000, 0x114000];
thread_local! {
    static OWNERS: Cell<u8> = const { Cell::new(15) };
    static OWNER_CALLS: Cell<u32> = const { Cell::new(0) };
    static RAM_OK: Cell<bool> = const { Cell::new(true) };
    static TEXT_RAM: Cell<bool> = const { Cell::new(false) };
    static INIT_ERROR: Cell<i32> = const { Cell::new(0) };
    static COOKIE: Cell<*const Shared> = const { Cell::new(core::ptr::null()) };
}
mod glue {
    use super::*;
    pub fn reserved_uat(index: u32, start: u64, size: u64) -> bool {
        OWNER_CALLS.set(OWNER_CALLS.get() + 1);
        index < 4 && BASE[index as usize] == start && SIZES[index as usize] == size
            && OWNERS.get() & (1 << index) != 0
    }
    pub fn ram_disjoint(start: u64, _size: u64) -> bool {
        start >= BASE[4] && !TEXT_RAM.get()
    }
    pub fn ram_within(start: u64, _size: u64) -> bool { start < BASE[4] && RAM_OK.get() }
}
#[derive(Clone)]
struct Node { name: &'static str, props: Vec<(&'static str, Vec<u8>)>, kids: Vec<Node> }
impl Node {
    fn encode(&self) -> Vec<u8> {
        let mut out = Vec::new();
        let mut props = self.props.clone();
        props.push(("name", [self.name.as_bytes(), &[0]].concat()));
        out.extend_from_slice(&(props.len() as u32).to_le_bytes());
        out.extend_from_slice(&(self.kids.len() as u32).to_le_bytes());
        for (name, data) in props {
            let mut n = [0u8; 32]; n[..name.len()].copy_from_slice(name.as_bytes());
            out.extend_from_slice(&n); out.extend_from_slice(&(data.len() as u32).to_le_bytes());
            out.extend_from_slice(&data); while out.len() % 4 != 0 { out.push(0); }
        }
        for child in &self.kids { out.extend_from_slice(&child.encode()); }
        out
    }
}
fn fixture(kind: u8) -> Vec<u8> {
    let mut sgx = Node { name: "sgx", props: vec![("compatible", b"gpu,t6030\0".to_vec())], kids: vec![] };
    for (i, (b, s)) in [("gpu-region-base", "gpu-region-size"), ("gfx-shared-region-base", "gfx-shared-region-size"), ("gfx-handoff-base", "gfx-handoff-size"), ("gfx-shared-l2-region-base", "gfx-shared-l2-region-size")].iter().enumerate() {
        sgx.props.push((b, BASE[i].to_le_bytes().to_vec())); sgx.props.push((s, SIZES[i].to_le_bytes().to_vec()));
    }
    for (name, value) in [("perf-state-count", 14u32), ("gpu-num-perf-states", 13), ("perf-state-table-count", 1), ("gpu-power-sample-period", 8), ("gpu-pwr-sample-period-aic-clks", 200000)] { sgx.props.push((name, value.to_le_bytes().to_vec())); }
    for (name, mv) in [("perf-states", 600u32), ("perf-states-sram", 700)] {
        let mut table = vec![];
        for i in 0..14u32 { table.extend_from_slice(&(i * 100000000).to_le_bytes()); table.extend_from_slice(&mv.to_le_bytes()); }
        sgx.props.push((name, table));
    }
    let mut segments = vec![];
    for i in 0..2 {
        segments.extend_from_slice(&BASE[4+i].to_le_bytes());
        segments.extend_from_slice(&(hw::UPPER + if i == 0 { 0 } else { SIZES[4] } + if kind == 3 { 0x4000 } else { 0 }).to_le_bytes());
        segments.extend_from_slice(&BASE[4+i].to_le_bytes());
        segments.extend_from_slice(&(SIZES[4+i] as u32).to_le_bytes());
        segments.extend_from_slice(&(if i == 0 { 1u32 } else { 0 }).to_le_bytes());
    }
    let mut nub_segments = segments.clone(); if kind == 2 { nub_segments[0] ^= 1; }
    let nub = Node { name: "iop-gfx-nub", props: vec![("segment-ranges", nub_segments), ("pre-loaded", (if kind == 1 { 0u32 } else { 1 }).to_le_bytes().to_vec())], kids: vec![] };
    let asc = Node { name: "gfx-asc", props: vec![("segment-ranges", segments), ("segment-names", b"__TEXT;__DATA\0".to_vec())], kids: vec![nub] };
    let chosen = Node { name: "chosen", props: vec![("dram-base", (if kind == 4 { u64::MAX } else { 0x100000000u64 }).to_le_bytes().to_vec()), ("dram-size", 0x200000000u64.to_le_bytes().to_vec())], kids: vec![] };
    Node { name: "root", props: vec![], kids: vec![Node { name: "arm-io", props: vec![], kids: vec![sgx, asc] }, chosen] }.encode()
}
struct Shared;
struct Rtkit { rtk: *mut c_void }
static RTKIT_OPS: u8 = 0;
fn from_err_ptr(p: *mut c_void) -> Result<*mut c_void> { if (p as isize) < 0 { Err(p as isize as i32) } else { Ok(p) } }
mod bindings {
    use super::*;
    pub unsafe fn apple_rtkit_init(_dev: *mut c_void, cookie: *mut c_void, name: *const core::ffi::c_char, index: u32, _ops: *const u8) -> *mut c_void {
        assert!(name.is_null(), "maintained DT mailbox must be selected by index");
        assert_eq!(index, 0);
        COOKIE.set(cookie.cast());
        let e = INIT_ERROR.get(); if e == 0 { 1usize as *mut c_void } else { e as isize as *mut c_void }
    }
}
'''

CASES = r'''
fn main() {
    let bytes = fixture(0); let adt = Adt { data: &bytes }; let dev = device::Device;
    OWNERS.set(15); OWNER_CALLS.set(0); RAM_OK.set(true); TEXT_RAM.set(false);
    let parsed = parse(&dev, &adt).expect("maintained reserved-RAM ADT admission");
    assert_eq!(OWNER_CALLS.get(), 4, "all four owner reservations must be checked"); assert_eq!(parsed.ttbs, BASE[0]..BASE[0]+SIZES[0]);
    assert_eq!(parsed.perf_mhz[13], 1300); assert_eq!(parsed.cluster_mv[13][0], 600);
    for missing in 0..4 {
        OWNERS.set(15 & !(1 << missing)); OWNER_CALLS.set(0);
        assert!(parse(&dev, &adt).is_err(), "every maintained UAT owner is mandatory");
        assert_eq!(OWNER_CALLS.get(), missing + 1);
    }
    OWNERS.set(15); RAM_OK.set(false); assert!(parse(&dev, &adt).is_err()); RAM_OK.set(true);
    TEXT_RAM.set(true); assert!(parse(&dev, &adt).is_err()); TEXT_RAM.set(false);
    for kind in 1..=4 {
        OWNER_CALLS.set(0); let bytes = fixture(kind);
        assert!(parse(&dev, &Adt { data: &bytes }).is_err()); assert_eq!(OWNER_CALLS.get(), 0);
    }
    assert_eq!(BUILDS.len(), 4, "maintained B0 firmware row must not require an absent profile symbol");
    for (row, machine) in [(1, &b"apple,j516s"[..]), (2, &b"apple,j514s"[..])] {
        assert_eq!(BUILDS[row].machine.unwrap().to_bytes(), machine);
        assert_eq!((BUILDS[row].text_size, BUILDS[row].data_size, BUILDS[row].marker), (0x5c000, 0x114000, 0x58e88));
        assert_eq!(BUILDS[row].text_sha256, [0xda,0x26,0xec,0xca,0xb1,0xf8,0x59,0x8d,0xa8,0x0c,0x85,0x5e,0xca,0x91,0x65,0x20,0x40,0x6d,0x1d,0x06,0x70,0xf1,0xd4,0x70,0xe7,0x9b,0xb9,0xb0,0x22,0x09,0xdb,0x53]);
    }
    // Issue #247: this J514S build must pin without the bring-up override.
    use identity::lookup::{Measured, Verdict, verdict};
    let measured = Measured {
        text_size: 0x5c000, data_size: 0x114000, marker: 0x58e88,
        sha256: [0xcd,0x05,0xaf,0x6a,0x82,0x8b,0x52,0xe4,
                 0xc2,0xfe,0xd2,0x00,0x86,0xd3,0xd1,0x8a,
                 0x7d,0xb4,0x2e,0x03,0xa8,0xfd,0x6f,0x83,
                 0x54,0xf9,0xf0,0x6e,0x0a,0x61,0x36,0x3e],
    };
    let identify = |m: &Measured, machine: &[u8]| verdict(m, BUILDS.iter().map(|b| (
        Measured { text_size: b.text_size, data_size: b.data_size,
                   marker: b.marker, sha256: b.text_sha256 },
        b.machine.map_or(true, |board| board.to_bytes() == machine),
    )), 0);
    assert!(matches!(identify(&measured, b"apple,j514s"), Verdict::Pinned(_)),
            "J514S cd05 firmware must pin without a prefix override");
    assert_eq!(identify(&measured, b"apple,j516s"), Verdict::Refused);
    let mut changed = measured;
    changed.sha256[31] ^= 1;
    assert_eq!(identify(&changed, b"apple,j514s"), Verdict::Refused);
    changed = measured; changed.text_size += 0x1000;
    assert_eq!(identify(&changed, b"apple,j514s"), Verdict::Refused);
    changed = measured; changed.data_size += 0x1000;
    assert_eq!(identify(&changed, b"apple,j514s"), Verdict::Refused);
    changed = measured; changed.marker += 8;
    assert_eq!(identify(&changed, b"apple,j514s"), Verdict::Refused);
    for error in [0, -517, -12] {
        INIT_ERROR.set(error); let shared = Arc::new(Shared); let result = Rtkit::new(&dev, &shared);
        assert_eq!(result.is_ok(), error == 0);
        assert_eq!(Arc::strong_count(&shared), if error == 0 { 2 } else { 1 });
        if error == 0 { unsafe { drop(Arc::from_raw(COOKIE.get())); } }
    }
    println!("PASS actual ADT/owner/performance parser, B0 identity row, unnamed RTKit mailbox and cookie lifetime");
}
'''


def main():
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('--output', type=Path); p.add_argument('--mutations', action='store_true'); a=p.parse_args()
    out=(a.output or Path(tempfile.mkdtemp(prefix='g15-maintained-handoff-'))).resolve(); out.mkdir(parents=True, exist_ok=True)
    adt=(G15/'adt.rs').read_text(); boot=(G15/'boot.rs').read_text(); hw=(G15/'hw.rs').read_text()
    record=(G15/'board/record.rs').read_text()
    helpers='\n'.join(item(adt,m) for m in ["pub(crate) struct GpuAdt", "struct Adt<'a>", 'fn le32(', 'fn le64(', 'fn strncmp_eq(', "impl Adt<'_>", 'fn collect_perf(', 'fn parse(', 'fn admit_regions('])
    constants='\n'.join(re.search(r'pub\(crate\) const '+n+r':[^;]+;', hw).group(0) for n in ['PAGE','PA_MASK','TTBS_SIZE','TABLES_SIZE','HANDOFF_SIZE','SHARED_L2_SIZE','UPPER'])
    # GpuAdt carries the T6031 sensor rows, sized by the record builder's production constant.
    record_constants=re.search(r'pub\(crate\) const SENSOR_ROW_COUNT:[^;]+;', record).group(0)
    start=hw.index('        firmware: &['); start=hw.index('[',start); end=start+1; depth=1
    while depth: depth+=(hw[end]=='[')-(hw[end]==']'); end+=1
    firmware='static BUILDS: &[FirmwareBuild] = &'+hw[start:end]+';\n'+item(hw,'pub(crate) struct FirmwareBuild')
    constructor=item(boot,'    fn new(dev: &device::Device, shared: &Arc<Shared>)')
    identity='mod identity {\n'+''.join(f'#[path = "{G15 / "identity" / (name + ".rs")}"] pub(crate) mod {name};\n' for name in ['matching', 'lookup'])+'}\n'
    source=PREFIX+'\nmod hw {\n'+constants+'\n}\nmod record {\n'+record_constants+'\n}\n'+helpers+'\n'+firmware+'\nimpl Rtkit {\n'+constructor+'\n}\n'+identity+CASES
    variants={'actual':source}
    if a.mutations:
        variants['skip-owner-check']=source.replace('if !glue::reserved_uat(i as u32, r.start, r.end - r.start) {','if false {')
        variants['skip-fourth-owner']=source.replace('region[..4].iter().enumerate()', 'region[..3].iter().enumerate()')
        variants['drop-reserved-ram']=source.replace('Some(dram_base..dram_base + dram_size), true)?;', 'Some(dram_base..dram_base + dram_size), false)?;')
        variants['require-obsolete-b0-profile']=source.replace('          FirmwareBuild {\n            name: "J516S B0', '          #[cfg(CONFIG_SILICONFOREST_NATIVE_GPU)]\n          FirmwareBuild {\n            name: "J516S B0')
        variants['named-mailbox']=source.replace('cookie, core::ptr::null(), 0, &RTKIT_OPS', 'cookie, c"mbox".as_ptr(), 0, &RTKIT_OPS')
    receipt={'scope':__doc__,'sources':{p:hashlib.sha256(t.encode()).hexdigest() for p,t in [('adt.rs',adt),('boot.rs',boot),('hw.rs',hw),('board/record.rs',record)]},'variants':{}}
    for name,text in variants.items():
        if name!='actual' and text==source: raise SystemExit('NOT_APPLIED '+name)
        src=out/(name+'.rs');src.write_text(text); binary=out/name
        subprocess.run([os.environ.get('HOSTRUSTC','rustc'),'--edition=2021',str(src),'-o',str(binary)],check=True)
        r=subprocess.run([str(binary)],capture_output=True,text=True)
        receipt['variants'][name]={'exit':r.returncode,'stdout':r.stdout,'stderr':r.stderr}
        if name=='actual' and r.returncode: raise SystemExit(r.stderr)
        if name!='actual' and not r.returncode: raise SystemExit('SURVIVED '+name)
        if name!='actual':
            expected={'skip-owner-check':'all four owner reservations must be checked','skip-fourth-owner':'all four owner reservations must be checked','drop-reserved-ram':'maintained reserved-RAM ADT admission','require-obsolete-b0-profile':'maintained B0 firmware row must not require an absent profile symbol','named-mailbox':'maintained DT mailbox must be selected by index'}[name]
            receipt['variants'][name]['intended_failure']=expected in r.stderr
            if expected not in r.stderr: raise SystemExit('INVALID unrelated failure '+name+'\n'+r.stderr)
    (out/'result.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(receipt,indent=2))
if __name__=='__main__': main()
