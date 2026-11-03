#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Actual system PM callbacks, synthetic register/PM boundaries; no device access."""
from pathlib import Path
import subprocess,tempfile
R=Path(__file__).resolve().parents[2]
def fn(s,n):
 a=s.index('static int '+n+'(');b=s.index('\n}',a)+2;return s[a:b]
s=(R/'drivers/iommu/apple-dart.c').read_text();h=(R/'drivers/soc/apple/rtkit-helper.c').read_text()
assert 'if (dart->live_mtp)\n\t\treturn -EBUSY;' in s
pre=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <errno.h>
#define dev_dbg(...) ((void)0)
#define dev_err(...) ((void)0)
#define DART_TCR(d,s) (s)
#define APPLE_ASC_CPU_CONTROL 0x44
#define APPLE_ASC_CPU_CONTROL_RUN 16
#define PM_SUSPEND_TO_IDLE 1
static int pm_suspend_target_state=1;
struct device {void *data;bool awake;};
struct hw {unsigned tcr_bypass;};
struct apple_dart {bool live_mtp,live_mtp_sleep;struct hw *hw;unsigned tcr;};
struct apple_rtkit_helper {bool adopted;unsigned char *asc_base;};
static void *dev_get_drvdata(struct device *d){return d->data;}
static unsigned apple_dart_readl(struct apple_dart *d,unsigned sid){assert(sid==1);return d->tcr;}
static void device_set_awake_path(struct device *d){d->awake=true;}
static unsigned forced_suspend,forced_resume;
static int pm_runtime_force_suspend(struct device *d){forced_suspend++;return 7;}
static int pm_runtime_force_resume(struct device *d){forced_resume++;return 8;}
static bool of_machine_is_compatible(const char *s){return true;}
static unsigned readl(void *p){return *(unsigned *)p;}
'''
body=fn(s,'apple_dart_system_suspend')+fn(s,'apple_dart_system_resume')+fn(h,'apple_rtkit_helper_suspend')+fn(h,'apple_rtkit_helper_resume')
for forbidden in ['writel','hw_reset','invalidate_tlb','apple_rtkit_wake','apple_rtkit_boot','apple_rtkit_quiesce']:
 assert forbidden not in body,forbidden
checks=r'''
int main(void){
 struct hw hw={.tcr_bypass=3};struct apple_dart a={.live_mtp=true,.hw=&hw,.tcr=3};struct device d={.data=&a};
 assert(!apple_dart_system_suspend(&d)&&d.awake&&a.live_mtp_sleep);
 assert(!apple_dart_system_resume(&d)&&!a.live_mtp_sleep&&a.tcr==3&&!forced_suspend&&!forced_resume);
 // Abort unwind is the same retained transaction, and can be repeated.
 assert(!apple_dart_system_suspend(&d));assert(!apple_dart_system_resume(&d));
 pm_suspend_target_state=2;assert(apple_dart_system_suspend(&d)==-EBUSY);pm_suspend_target_state=1;
 a.tcr=0;assert(apple_dart_system_suspend(&d)==-EIO);a.tcr=3;
 assert(!apple_dart_system_suspend(&d));a.tcr=0;assert(apple_dart_system_resume(&d)==-EIO&&a.tcr==0);
 a.live_mtp=false;assert(apple_dart_system_suspend(&d)==7&&apple_dart_system_resume(&d)==8);
 unsigned regs[32]={0};struct apple_rtkit_helper h={.adopted=true,.asc_base=(void *)regs};d.data=&h;d.awake=false;
 regs[0x44/4]=16;assert(!apple_rtkit_helper_suspend(&d)&&d.awake);assert(!apple_rtkit_helper_resume(&d));
 regs[0x44/4]=0;assert(apple_rtkit_helper_resume(&d)==-EHOSTDOWN&&!regs[0x44/4]);
 assert(apple_rtkit_helper_suspend(&d)==-EHOSTDOWN);
 h.adopted=false;assert(!apple_rtkit_helper_suspend(&d)&&!apple_rtkit_helper_resume(&d));
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'pm.c';p.write_text(pre+body+checks)
 subprocess.run(['clang','-std=gnu11','-Wall','-Werror','-fsanitize=address,undefined',str(p),'-o',str(p.with_suffix(''))],check=True)
 subprocess.run([str(p.with_suffix(''))],check=True)
print('PASS actual MTP/DART system PM: retained normal/abort cycles, runtime delegation, non-s2idle refusal, lost bypass/RUN refuses without reset/INIT')
