#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run production queue recovery and PM transitions against a strict DMA model."""
import os
from pathlib import Path
import hashlib
import subprocess
import sys
from host_source import function

HERE = Path(__file__).resolve().parent
ROOT = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else HERE.parent

PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <errno.h>
#include <stddef.h>
#define AVD_MAX_FRAMES 16
#define AVD_FRAME_FREE 0
#define AVD_FRAME_SUBMITTED 2
#define AVD_FRAME_DONE 3
#define AVD_RECOVER_GAP_MS 20
#define AVD_STORM_MAX 2
#define AVD_STORM_WINDOW 60000
#define msecs_to_jiffies(x) (x)
#define jiffies_to_msecs(x) ((unsigned int)(x))
#define time_before(a,b) ((long)((a)-(b))<0)
#define time_after(a,b) time_before(b,a)
#define WARN_ONCE(v,...) (assert(!(v)),false)
#define WRITE_ONCE(a,b) ((a)=(b))
#define spin_lock_irqsave(l,f) do { (void)(l); (f)=0; } while(0)
#define spin_unlock_irqrestore(l,f) do { (void)(l); (void)(f); } while(0)
#define mutex_lock(l) ((void)(l))
#define mutex_unlock(l) ((void)(l))
#define dev_warn(d,...) ((void)(d))
#define dev_err(d,...) ((void)(d))
#define list_for_each_entry(c,h,n) for((void)(h),(c)=NULL;(c);(c)=NULL)
typedef int irqreturn_t;
struct avd_dev;
struct avd_ctx { bool session_started; };
struct frame { int state,status; };
struct avd_backend_ops {
 void (*shutdown)(struct avd_dev *);
 int (*resume)(struct avd_dev *);
 int (*suspend)(struct avd_dev *);
};
struct queue {
 bool (*idle)(struct avd_dev *);
 void (*resubmit)(struct avd_dev *);
};
struct avd_dev {
 void *dev,*domain,*empty_domain,*rstc,*compat_active;
 const struct avd_backend_ops *backend;
 struct {
  bool dart_active,dma_parked,irq_enabled,hw_on;
  int irq,hw_lock,lock;unsigned int irq_gen;
  const struct queue *queue;
 } pm;
 bool faulted;unsigned int storm_count;unsigned long storm_start,last_recover,rest_until;
 struct frame frames[AVD_MAX_FRAMES];int queue_lock,contexts;
};
static struct avd_dev d;
static unsigned long jiffies=100;
static void *attached;
static int reset_error,boot_error,attach_error;
static unsigned int resets,boots,attaches,detaches,shutdowns,resubmits,drains,irq_opens,irq_closes,step,block_step,reset_step;
static bool pm_runtime_suspended(void *dev) { (void)dev;return false; }
static void disable_irq_nosync(int irq) { assert(irq==1);irq_closes++; }
static void enable_irq(int irq) { assert(irq==1);assert(!d.pm.dma_parked);irq_opens++; }
static int iommu_attach_device(void *domain,void *dev) {
 (void)dev;assert(domain==d.empty_domain&&!d.pm.hw_on&&!d.pm.irq_enabled);
 attaches++;if(attach_error)return attach_error;attached=domain;block_step=++step;return 0;
}
static void iommu_detach_device(void *domain,void *dev) {
 (void)dev;assert(domain==d.empty_domain&&attached==domain&&!reset_error);
 detaches++;attached=d.domain;
}
static void *iommu_get_domain_for_dev(void *dev) { (void)dev;return attached; }
static int reset_control_reset(void *rstc) {
 (void)rstc;assert(!d.domain||attached==d.empty_domain);assert(!d.pm.hw_on&&!d.pm.irq_enabled);
 resets++;reset_step=++step;assert(!d.domain||block_step<reset_step);return reset_error;
}
static void usleep_range(unsigned int lo,unsigned int hi) { assert(lo==5000&&hi==6000); }
static void msleep(unsigned int ms) { assert(ms==20);jiffies+=ms; }
static void shutdown_backend(struct avd_dev *avd) { assert(avd==&d);shutdowns++; }
static int avd_pm_backend_resume(struct avd_dev *avd) {
 assert(avd==&d&&!d.pm.dma_parked&&attached==d.domain);boots++;return boot_error;
}
static bool idle(struct avd_dev *avd) {
 for(int i=0;i<16;i++)if(avd->frames[i].state!=AVD_FRAME_FREE)return false;
 return true;
}
static void resubmit(struct avd_dev *avd) { (void)avd;resubmits++; }
static void avd_pm_drain_unmaps(struct avd_dev *avd) { assert(avd->pm.dart_active);drains++; }
'''

MAIN = r'''
static const struct avd_backend_ops backend={.shutdown=shutdown_backend};
static const struct queue queue={.idle=idle,.resubmit=resubmit};
static int domain_token,empty_token;
static void init(void) {
 d=(struct avd_dev){.dev=&d,.domain=&domain_token,.empty_domain=&empty_token,.backend=&backend,
 .pm={.dart_active=true,.irq_enabled=true,.hw_on=true,.irq=1,.queue=&queue}};
 attached=d.domain;reset_error=boot_error=attach_error=0;
 resets=boots=attaches=detaches=shutdowns=resubmits=drains=irq_opens=irq_closes=step=0;
}
int main(void) {
 init();assert(!avd_pm_recover(&d));assert(resets==1&&boots==1&&attaches==1&&detaches==1);
 assert(!d.pm.dma_parked&&!d.pm.irq_enabled&&attached==d.domain);
 avd_pm_enable_irq(&d);assert(irq_opens==1);
 init();reset_error=-EIO;d.frames[0].state=AVD_FRAME_SUBMITTED;avd_reset_queue(&d,-ETIMEDOUT);
 assert(d.faulted&&resets==1&&!boots&&!detaches&&d.pm.dma_parked&&attached==d.empty_domain);
 assert(d.frames[0].state==AVD_FRAME_DONE&&!d.pm.irq_enabled);
 d.frames[0].state=AVD_FRAME_FREE;
 assert(!avd_pm_power_off(&d)&&!avd_pm_power_on(&d));
 assert(!boots&&!resubmits&&!d.pm.irq_enabled&&attached==d.empty_domain);
 init();boot_error=-ETIMEDOUT;avd_reset_queue(&d,-ETIMEDOUT);
 assert(d.faulted&&resets==1&&boots==1&&attaches==2&&detaches==1&&d.pm.dma_parked);
 assert(attached==d.empty_domain&&!d.pm.irq_enabled);
 init();avd_reset_queue(&d,-EIO);avd_reset_queue(&d,-EIO);avd_reset_queue(&d,-EIO);
 assert(resets==2&&boots==2&&attaches==3&&detaches==2&&!d.faulted&&d.rest_until);
 assert(d.pm.dma_parked&&attached==d.empty_domain&&!d.pm.irq_enabled);
 unsigned int old_boots=boots,old_resets=resets;
 assert(!avd_pm_power_off(&d)&&!avd_pm_power_on(&d));
 assert(boots==old_boots&&resets==old_resets&&!resubmits&&!d.pm.irq_enabled);
 /* The expiry path (dispatch-test) calls recover under a PM reference. */
 jiffies=d.rest_until;assert(!avd_pm_recover(&d));assert(resets==3&&!d.pm.dma_parked);
 init();attach_error=-EIO;assert(avd_pm_recover(&d)==-EIO);assert(!resets&&!boots&&!detaches);
 init();d.pm.dart_active=false;assert(avd_pm_park(&d)==-EHOSTDOWN);assert(!shutdowns&&!resets&&!attaches);
 puts("PASS: production recovery/PM: park before reset, failed reset/boot stays blocked, budget rests, parked resume never boots, expiry recovery");
}
'''


def main():
    out = Path(sys.argv[1]).resolve()
    out.mkdir(parents=True, exist_ok=True)
    pm = (ROOT / 'avd-pm.c').read_text()
    queue = (ROOT / 'avd-queue.c').read_text()
    (out / 'inputs.sha256').write_text(''.join(
        hashlib.sha256((ROOT / n).read_bytes()).hexdigest() + '  ' + n + '\n'
        for n in ['avd-pm.c', 'avd-queue.c']))
    code = out / 'recovery.c'
    code.write_text(PRELUDE + ''.join(function(pm, n) for n in [
        'avd_pm_enable_irq', 'avd_pm_disable_irq', 'avd_pm_park', 'avd_pm_recover',
        'avd_pm_power_on', 'avd_pm_power_off']) +
        function(queue, 'avd_reset_queue') + MAIN)
    exe = out / 'recovery-test'
    subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra',
                    '-Werror', '-fsanitize=address,undefined', '-g', str(code),
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)


if __name__ == '__main__':
    main()
