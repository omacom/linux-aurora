#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Actual native identity and live-session predicates; classic stays separate."""
from pathlib import Path
import os,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[2]
header=(ROOT/'drivers/gpu/drm/apple/j613-25g83.h').read_text()
def function(name):
 pos=header.index('static inline bool '+name+'(')
 return header[pos:header.index('\n}',pos)+2]
source=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <stdio.h>
typedef uint32_t u32;
#define BIT(n) (1U<<(n))
#define J613_25G83_DCP_UUID "C042E95C-B9D8-3F0E-94B3-582A08AA6FDD"
'''+function('j613_25g83_identity')+function('j613_25g83_external_state')+r'''
int main(void) {
 u32 version[]={26,6,2},state[]={1,16,34,1};
 assert(j613_25g83_identity(true,1,version,3,J613_25G83_DCP_UUID));
 for(unsigned i=0;i<3;i++) { version[i]++;assert(!j613_25g83_identity(true,1,version,3,J613_25G83_DCP_UUID));version[i]--; }
 assert(!j613_25g83_identity(false,1,version,3,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,0,version,3,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,1,version,2,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,1,version,3,"other image"));
 assert(j613_25g83_external_state(state,4));
 for(unsigned i=0;i<4;i++)assert(!j613_25g83_external_state(state,i));
 assert(!j613_25g83_external_state(NULL,4));
 for(unsigned bit=0;bit<32;bit++) {
  state[1]=16^(1U<<bit);assert(j613_25g83_external_state(state,4)==(bit!=4));
  state[1]=16;state[3]=1^(1U<<bit);assert(j613_25g83_external_state(state,4)==(bit!=0));state[3]=1;
 }
 state[0]=0;assert(!j613_25g83_external_state(state,4));state[0]=2;assert(!j613_25g83_external_state(state,4));
 puts("PASS native26 exact board/profile/version/image and powered RUN/locked-DART handoff admission");
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t);(p/'check.c').write_text(source)
 subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-O2','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p/'check.c'),'-o',str(p/'check')],check=True)
 subprocess.run([str(p/'check')],check=True)
