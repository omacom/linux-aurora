#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Check retained DART tables and firmware translation publication."""
from pathlib import Path
import argparse, hashlib, json, re, subprocess
ROOT=Path(__file__).resolve().parents[2]
NAMES=('apple_dart_fw_handoff_enabled','apple_dart_pte_to_paddr','apple_dart_fw_table_ram','apple_dart_fw_table_decode','apple_dart_fw_slot_owned','apple_dart_fw_leaf_decode','apple_dart_fw_mapping_matches','apple_dart_fw_page_matches','apple_dart_fw_slot_mirrors','apple_dart_publish_root','apple_dart_retire_root','apple_dart_free_fw_root','apple_dart_snapshot_fw_root','apple_dart_fw_slot_foreign','apple_dart_domain_uses_fw','apple_dart_check_fw_map','apple_dart_inherited_phys','apple_dart_fw_slot_range','apple_dart_get_fw_resv_regions','apple_dart_fw_invalidate','apple_dart_hw_unmap_locked_ttbr','apple_dart_retire_error','apple_dart_t8110_hw_invalidate_tlb','apple_dart_release_device','apple_dart_domain_free')
CASES=("legacy-map-match","legacy-map-mismatch","legacy-lookup","legacy-snapshot",'mirror','changed-leaf','wrong-permission','wrong-physical','malformed-leaf','vacant','changed-owner','retire','reserved-ram','unreserved','mmio','misalignment','address-width','root-encoding','leaf-as-table','snapshot','snapshot-malformed','snapshot-unreserved','snapshot-root-change','snapshot-map-failure','snapshot-allocation-failure','range-three','range-offset','range-width','range-overflow','range-underflow','range-four','range-four-last','map-firmware','map-hole','map-host','map-owned','map-unmatched','map-attached','map-wrong-physical','map-wrong-permission','map-changed-root','map-four','lookup-inherited','lookup-changed-leaf','lookup-changed-root','lookup-unmatched','reserve-firmware','reserve-teardown','reserve-unmatched','reserve-owned','invalidate-three','invalidate-four','invalidate-changed-root','invalidate-error','core-inherited','core-hole','core-changed-leaf','core-second-dart','snapshot-unmatched','snapshot-four','range-four-width','range-three-last','handoff-matched','handoff-missing','handoff-duplicate','handoff-zero','handoff-unlocked','handoff-other-soc','retire-free','retire-free-timeout','retire-release','retire-release-timeout','retire-rollback-timeout')
def function(s,name):
    match=re.search(r'^static[^;{}]*\b'+re.escape(name)+r'\([^;{}]*\)\s*\{',s,re.M)
    if not match:return None
    start=s.index('{',match.start());end=start+1;depth=1
    while depth:depth+=(s[end]=='{')-(s[end]=='}');end+=1
    return s[match.start():end]
def main():
    p=argparse.ArgumentParser();p.add_argument('--ref');p.add_argument('--source',type=Path);p.add_argument('--out',type=Path,required=True);p.add_argument('--case',action='append');a=p.parse_args()
    s=a.source.read_text() if a.source else subprocess.check_output(['git','-C',str(ROOT),'show',a.ref+':drivers/iommu/apple-dart.c'],text=True) if a.ref else (ROOT/'drivers/iommu/apple-dart.c').read_text()
    bodies={n:function(s,n) for n in NAMES};missing=[n for n,b in bodies.items() if b is None]
    if missing and not a.case:p.error('production functions missing: '+','.join(missing))
    core_path=ROOT/'drivers/iommu/iommu.c';core= subprocess.check_output(['git','-C',str(ROOT),'show',a.ref+':drivers/iommu/iommu.c'],text=True) if a.ref else core_path.read_text()
    core_body=function(core,'iommu_create_device_fw_mappings')
    fixture=Path(__file__).with_name('apple-dart-fw')/'fixture.c';code=fixture.read_text().replace('/* PRODUCTION */','\n'.join(b for b in bodies.values() if b)).replace('/* CORE */',core_body)
    a.out.mkdir(parents=True,exist_ok=False);c=a.out/'production.c';c.write_text(code);binary=a.out/'production'
    argv=['cc','-std=gnu11','-Werror',str(c),'-o',str(binary)];subprocess.run(argv,check=True)
    results=[]
    for name in a.case or CASES:
        run=subprocess.run([str(binary),name],capture_output=True,text=True);results.append(dict(case=name,exit=run.returncode,stdout=run.stdout,stderr=run.stderr))
    receipt=dict(source_sha256=hashlib.sha256(s.encode()).hexdigest(),functions={n:hashlib.sha256(b.encode()).hexdigest() for n,b in bodies.items() if b},core_function_sha256=hashlib.sha256(core_body.encode()).hexdigest(),fixture_sha256=hashlib.sha256(fixture.read_bytes()).hexdigest(),compile=argv,controls=results,limits='Actual production function bodies; RAM allocator, mapped memory and firmware mutations controlled on host; no hardware execution')
    (a.out/'receipt.json').write_text(json.dumps(receipt,indent=2)+'\n');print(json.dumps(results,indent=2))
    if any(x['exit'] for x in results):raise SystemExit(1)
if __name__=='__main__':main()
