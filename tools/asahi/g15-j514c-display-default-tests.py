#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Check the real T6031 native-display default selector and owner opt-out."""
import pathlib, subprocess, tempfile
ROOT = pathlib.Path(__file__).resolve().parents[2]
s = (ROOT / 'drivers/soc/apple/t6030-display-gate.c').read_text()
def item(name):
 a=s.index(name);a=s.rfind('static ',0,a);b=s.index('\n}',a)+2;return s[a:b]
compat=item('gate_t6031_is_compatible(const char *compat)')
selector=item('gate_t6031_default_board(void)')
wanted=item('gate_t6031_wanted(int sw, bool default_board)')
assert 'const char *board = gate_t6031_default_board();' in s
assert 'gate_t6031_wanted(gate_t6031_switch, !!board)' in s
source=r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#define __init
static const char *board, *soc;
static int of_machine_is_compatible(const char *s)
{ return (board && !strcmp(board,s)) || (soc && !strcmp(soc,s)); }
'''+compat+'\n'+selector+'\n'+wanted+r'''
int main(void)
{
 const char *boards[]={"apple,j514c","apple,j516c","apple,j514m","apple,j516m","apple,j514s","apple,j999"};
 const char *socs[]={"apple,t6031","apple,t6034","apple,t6030",NULL};
 for(unsigned i=0;i<sizeof(boards)/sizeof(boards[0]);i++) for(unsigned j=0;j<sizeof(socs)/sizeof(socs[0]);j++) {
  board=boards[i];soc=socs[j];bool expected=i<2 && j==0;
  const char *admitted=gate_t6031_default_board();assert(!!admitted==expected);
  if(admitted) assert(!strcmp(admitted,board));
  assert(gate_t6031_wanted(-1,!!admitted)==expected);
  assert(!gate_t6031_wanted(0,!!admitted));
  assert(gate_t6031_wanted(1,!!admitted));
 }
 return 0;
}
'''
mutants={
 'control':source,
 'missing-j514c':source.replace('gate_t6031_is_compatible("apple,j514c")','false'),
 'missing-soc':source.replace('!gate_t6031_is_compatible("apple,t6031")','false'),
 'lost-optout':source.replace('return sw < 0 ? default_board : sw;','return sw < 0 ? default_board : true;'),
}
with tempfile.TemporaryDirectory(prefix='j514c-display-default-') as temp:
 for name,c in mutants.items():
  exe=pathlib.Path(temp)/name
  subprocess.run(['cc','-std=gnu11','-Wall','-Wextra','-Werror','-x','c','-','-o',str(exe)],input=c,text=True,check=True)
  r=subprocess.run([str(exe)],capture_output=True)
  assert (r.returncode==0)==(name=='control'),(name,r.stderr)
  print(name+': '+('pass' if name=='control' else 'caught'))
print('Normal J514C/J516C display admission and explicit owner opt-outs: PASS')
