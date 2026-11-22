#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise production endpoint admission across firmware/profile combinations."""
from pathlib import Path
import subprocess, tempfile, re
root = Path(__file__).resolve().parents[2]
s = (root/'drivers/gpu/drm/apple/iomfb.c').read_text()
a = s.index('int iomfb_start_rtkit(')
b = s.index('\nvoid iomfb_shutdown(', a)
body = s[a:b]
pre = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>
typedef uint64_t dma_addr_t;
#define DCP_FIRMWARE_H17P 1
#define DCP_FIRMWARE_V_27_0 2
#define DCP_FIRMWARE_V_26_6_DESKTOP 3
#define DCP_IOMFB_METHODS_H17P_EXTERNAL 4
#define REMOTE_ALLOC_ENDPOINT 0x2d
#define IOMFB_ENDPOINT 0x20
#define DCP_SHMEM_SIZE 1
#define GFP_KERNEL 0
struct apple_dcp { int fw_compat; struct {int iomfb_method_profile;} hw;
 struct {int lock; bool active, scanout, stopped;} iomfb;
 void *rtk, *shmem, *dev; bool ext; };
static int alloc_calls, iomfb_calls, fail_ep;
/* Firmware 27 lists the endpoints it uses; whether that list has the allocator. */
static bool alloc_advertised;
static bool apple_rtkit_has_endpoint(void *r, int ep) {return ep!=REMOTE_ALLOC_ENDPOINT || alloc_advertised;}
#define dev_info(...) ((void)0)
static bool iomfb_uses_queue(struct apple_dcp *d) {return false;}
static bool dcp_is_ext26(struct apple_dcp *d) {return d->ext;}
static void mutex_lock(int *p) {}
static void mutex_unlock(int *p) {}
static int apple_rtkit_start_ep(void *r, int ep) {
 if (ep==fail_ep) return -EINVAL;
 if(ep==REMOTE_ALLOC_ENDPOINT) alloc_calls++; else iomfb_calls++;
 return 0;
}
static void *dma_alloc_coherent(void *d,int n,dma_addr_t *a,int flags) {*a=0; return (void *)1;}
static int dcpep_set_shmem(dma_addr_t a) {return 0;}
static void dcp_send_message(struct apple_dcp *d,int ep,int m) {}
'''
main = r'''
int main(void) {
 for(int fw=0;fw<4;fw++) for(int ext=0;ext<2;ext++) for(int profile=0;profile<5;profile+=4) for(int adv=0;adv<2;adv++) {
  struct apple_dcp d={.fw_compat=fw,.ext=ext,.hw.iomfb_method_profile=profile};
  /* Firmware 27 starts the allocator only where it advertises it; H17P always does. */
  bool alloc=(fw==1 || (fw==2 && adv)) && !ext && profile!=4;
  alloc_advertised=adv;
  alloc_calls=iomfb_calls=0;fail_ep=-1;
  assert(iomfb_start_rtkit(&d)==0);
  assert(alloc_calls==alloc);assert(iomfb_calls==(fw!=3));
  /* The native26 map has no allocator: it must still start IOMFB. */
  if(ext) { fail_ep=REMOTE_ALLOC_ENDPOINT;assert(!iomfb_start_rtkit(&d)); }
  if(alloc) { fail_ep=REMOTE_ALLOC_ENDPOINT;assert(iomfb_start_rtkit(&d)==-EINVAL); }
 }
 return 0;
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t)/'test.c';p.write_text(pre+body+main)
 subprocess.run(['cc','-Werror','-Wno-unused-function','-fsanitize=address,undefined',str(p),'-o',str(p.with_suffix(''))],check=True)
 subprocess.run([str(p.with_suffix(''))],check=True)
print('PASS production IOMFB startup firmware/profile/advertisement matrix and allocator failures')
