#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile valid generated bindings for explicitly matching C/Rust targets."""
if not __debug__:
    raise SystemExit("optimized Python is unsupported: validation assertions must remain enabled")

import argparse,hashlib,json,re,subprocess,sys
from pathlib import Path
P=Path(__file__).resolve().parent
ap=argparse.ArgumentParser();ap.add_argument('--bindgen',required=True);ap.add_argument('--output-dir',required=True,type=Path);ap.add_argument('--rustc',default='rustc');ap.add_argument('--clang',default='clang');ap.add_argument('--rust-target');ap.add_argument('--clang-target');ap.add_argument('--rust-sysroot');ap.add_argument('--rust-libdir');ap.add_argument('--baseline-bindgen');a=ap.parse_args()
O=a.output_dir.resolve();assert not O.exists(),'use a fresh evidence directory';O.mkdir(parents=True)
def run(cmd):return subprocess.run(cmd,capture_output=True,text=True)
host=next(x.split(': ',1)[1] for x in subprocess.check_output([a.rustc,'-vV'],text=True).splitlines() if x.startswith('host: '));rt=a.rust_target or host;ct=a.clang_target or rt
arch=lambda t:{'arm64':'aarch64','amd64':'x86_64'}.get(t.split('-')[0],t.split('-')[0])
assert arch(rt)==arch(ct),('C/Rust architecture mismatch',ct,rt)
rust=[a.rustc,'--edition=2021','--crate-type=rlib','--crate-name','layout_check','--deny=unknown-lints','--emit=obj','-Cpanic=abort','--target='+rt]
if a.rust_sysroot:rust+=['--sysroot='+a.rust_sysroot]
if a.rust_libdir:rust+=['-L',str(Path(a.rust_libdir).resolve())]
probe=O/'target.rs';probe.write_text('#![no_std]\npub fn available() -> usize { core::mem::size_of::<usize>() }\n');r=run(rust+[str(probe),'-o',str(O/'target.o')]);(O/'target.log').write_text(r.stdout+r.stderr)
if r.returncode:
 status='SKIP' if 'E0463' in r.stderr and 'core' in r.stderr else 'FAIL'
 result={'status':status,'reason':'matching Rust target/core unavailable' if status=='SKIP' else 'target preflight compile failed','clang_target':ct,'rust_target':rt,'rust_argv':rust,'log':'target.log'};(O/'result.json').write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result));sys.exit(77 if status=='SKIP' else 1)
access='''
unsafe fn check_fields(a: &mut outer, b: &mut wrapper, c: &mut union_member, d: &mut trailing_member) {
 a.__bindgen_anon_1.__bindgen_anon_1.words[19] = 1;
 b.__bindgen_anon_1.words[19] = 2;
 c.__bindgen_anon_1.bytes[23] = 3;
 d.__bindgen_anon_1.words[0] = 4;
}
'''
cases=[];equal={};toolset=[('fixed',a.bindgen)]+([('baseline',a.baseline_bindgen)] if a.baseline_bindgen else [])
for fixture,language,extra in [('aligned-empty.h','c',[]),('aligned-empty.hpp','c++',[]),('anonymous-members.h','c',['-fms-extensions']),('tag-only.h','c',[]),('bitfields.h','c',[])]:
 header=P/'tests'/fixture
 # Also ask the C compiler to accept the ordinary fixture independently.
 ccmd=[a.clang,'--target='+ct,'-x',language,*extra,'-Wno-microsoft-anon-tag','-c',str(header),'-o',str(O/(fixture+'.c.o'))];cr=run(ccmd);(O/(fixture+'.c.log')).write_text(cr.stdout+cr.stderr);assert cr.returncode==0,(fixture,cr.stderr)
 for label,tool in toolset:
  # The baseline comparison covers layout repairs; this fixture checks the fixed tool
  # against the selected compiler, including compatibility adapters when needed.
  if fixture == 'bitfields.h' and label == 'baseline':continue
  cmd=[tool,str(header),'--rust-target','1.85','--no-doc-comments','--use-core','--no-derive-debug','--ctypes-prefix','core::ffi','--','-target',ct,'-x',language,*extra]
  g=run(cmd);assert g.returncode==0,(fixture,g.stderr)
  assert re.search(r'\bconst\s+_\s*:\s*\(\s*\)\s*=',g.stdout) and not re.search(r'#\s*\[\s*test\s*\]',g.stdout),'generator did not emit unconditional const layout checks'
  rs=O/(fixture+'.'+label+'.rs');rs.write_text('#![no_std]\n'+g.stdout+(access if fixture=='anonymous-members.h' and label=='fixed' else ''))
  rcmd=rust+[str(rs),'-o',str(O/(fixture+'.'+label+'.o'))];r=run(rcmd);(O/(fixture+'.'+label+'.log')).write_text(r.stdout+r.stderr)
  expected=label=='fixed' or fixture=='tag-only.h';assert (r.returncode==0)==expected,(fixture,label,r.stderr)
  if not expected:assert 'E0080' in r.stderr,'baseline failure must be a const layout mismatch'
  cases.append({'fixture':fixture,'tool':label,'clang_argv':ccmd,'generator_argv':cmd,'rust_argv':rcmd,'compile_exit':r.returncode,'expected':'layout compile success' if expected else 'const layout rejection','generated_sha256':hashlib.sha256(rs.read_bytes()).hexdigest()})
  if fixture=='tag-only.h':equal[label]=g.stdout
if a.baseline_bindgen:assert equal['baseline']==equal['fixed'],'ordinary tag-only binding changed'
result={'status':'PASS','clang_target':ct,'rust_target':rt,'rust_host':host,'explicit_rust_target':True,'cases':cases,'baseline_checked':bool(a.baseline_bindgen),'hardware_executed':False};(O/'result.json').write_text(json.dumps(result,indent=2)+'\n');print('PASS matching C/Rust targets:',ct,rt)
