#!/usr/bin/env python3
"""Exercise production full firmware lookup against the actual T6031 rows.

No firmware image is read. The J514C row is a bring-up candidate, not a hardware pass.
"""
from pathlib import Path
import re,subprocess,tempfile
root=Path(__file__).resolve().parents[2];hw=root/'drivers/gpu/drm/asahi/g15'
source=(hw/'hw/t6031.rs').read_text();rows=[]
for m in re.finditer(r'FirmwareBuild\s*\{(.*?)text_sha256:\s*\[(.*?)\],\s*\}',source,re.S):
 body,raw=m.groups();digest=re.findall(r'0x[0-9a-f]+',raw);assert len(digest)==32
 fields={key:re.search(r'\b'+key+r':\s*(0x[0-9a-f]+)',body).group(1) for key in ('text_size','data_size','marker')}
 machine=re.search(r'machine:\s*Some\(kernel::c_str!\("([^"]+)"\)\)',body)
 rows.append((fields,digest,machine.group(1) if machine else None))
assert len(rows)==3 and rows[2][2]=='apple,j514c'
measured=[];machines=[]
for f,d,machine in rows:
 measured.append('Measured {text_size:'+f['text_size']+',data_size:'+f['data_size']+',marker:'+f['marker']+',sha256:['+','.join(d)+']}')
 machines.append('None' if machine is None else 'Some("'+machine+'")')
code='''#[path="MATCHING"] mod matching;
#[path="LOOKUP"] mod lookup;
use lookup::{Measured,Verdict};
fn rows(board:&str)->Vec<(Measured,bool)> {
 let measurements=[MEASUREMENTS];let machines:[Option<&str>;3]=[MACHINES];
 measurements.into_iter().zip(machines).map(|(m,b)|(m,b.map_or(true,|b|b==board))).collect()
}
#[test] fn j514c_exact_candidate() {
 let exact=rows("apple,j514c")[2].0;
 assert_eq!(lookup::verdict(&exact,rows("apple,j514c"),0),Verdict::Pinned(2));
 for board in ["apple,j516c","apple,j514m","apple,j516m"] {
  assert_eq!(lookup::verdict(&exact,rows(board),0),Verdict::Refused);
 }
 let mut bad=exact;bad.sha256[31]^=1;
 assert_eq!(lookup::verdict(&bad,rows("apple,j514c"),0),Verdict::Refused);
 let mut bad=exact;bad.text_size+=0x4000;
 assert_eq!(lookup::verdict(&bad,rows("apple,j514c"),0),Verdict::Refused);
 let mut bad=exact;bad.data_size+=0x4000;
 assert_eq!(lookup::verdict(&bad,rows("apple,j514c"),0),Verdict::Refused);
 let mut bad=exact;bad.marker+=8;
 assert_eq!(lookup::verdict(&bad,rows("apple,j514c"),0),Verdict::Refused);
}
#[test] fn existing_rows_unchanged() {
 for i in 0..2 {let m=rows("apple,j516c")[i].0;
 assert_eq!(lookup::verdict(&m,rows("apple,j516c"),0),Verdict::Pinned(i));}
}
'''.replace('MATCHING',str(hw/'identity/matching.rs')).replace('LOOKUP',str(hw/'identity/lookup.rs')).replace('MEASUREMENTS',','.join(measured)).replace('MACHINES',','.join(machines))
with tempfile.TemporaryDirectory(prefix='g15-fw-') as td:
 p=Path(td);(p/'test.rs').write_text(code)
 subprocess.run(['rustc','--edition=2021','--test',str(p/'test.rs'),'-o',str(p/'test')],check=True)
 subprocess.run([str(p/'test')],check=True)
