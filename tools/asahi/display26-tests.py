#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Host controls for production J613 profile, swap and PMP bridge helpers."""
from pathlib import Path
import argparse
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]


def run(*args):
    subprocess.run(args, check=True)


def write(root, name, text):
    target = root / name
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(text)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dtb-dir', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='display26-host-') as directory:
        tmp = Path(directory)
        write(tmp, 'linux/types.h', '''#pragma once
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;
''')
        write(tmp, 'linux/string.h', '#include <string.h>\n')
        write(tmp, 'linux/bitops.h', '#pragma once\n#define BIT(x) (1U << (x))\nstatic int fls(unsigned x){return x?32-__builtin_clz(x):0;}\n')
        write(tmp, 'linux/bits.h', '#include "bitops.h"\n#define GENMASK_ULL(h,l) ((~0ULL << (l)) & (~0ULL >> (63-(h))))\n')
        write(tmp, 'linux/unaligned.h', '''#pragma once
#include <string.h>
#include "types.h"
static void put_unaligned_le16(u16 v,void*p){u8*b=p;b[0]=v;b[1]=v>>8;}
static void put_unaligned_le32(u32 v,void*p){u8*b=p;for(int i=0;i<4;i++)b[i]=v>>(8*i);}
static void put_unaligned_le64(u64 v,void*p){u8*b=p;for(int i=0;i<8;i++)b[i]=v>>(8*i);}
static u32 get_unaligned_le32(const void*p){const u8*b=p;u32 v=0;for(int i=0;i<4;i++)v|=(u32)b[i]<<(8*i);return v;}
static u64 get_unaligned_le64(const void*p){const u8*b=p;u64 v=0;for(int i=0;i<8;i++)v|=(u64)b[i]<<(8*i);return v;}
''')
        write(tmp, 'profile.c', '''#include <assert.h>
#include <stdio.h>
#include "drivers/gpu/drm/apple/j613-25g83.h"
#include "drivers/gpu/drm/apple/iomfb_v26_6_swap.h"
int main(void) {
 u32 v[]={26,6,2},c[]={1,345,416,712000000,0},m[]={1,1,1};
 assert(j613_25g83_identity(true,1,v,3,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(false,1,v,3,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,0,v,3,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,2,v,3,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,1,v,2,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,1,v,4,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,1,NULL,3,J613_25G83_DCP_UUID));
 assert(!j613_25g83_identity(true,1,v,3,NULL));
 assert(!j613_25g83_identity(true,1,v,3,"C042E95C-B9D8-3F0E-94B3-582A08AA6FDE"));
 for(int i=0;i<3;i++){v[i]++;assert(!j613_25g83_identity(true,1,v,3,J613_25G83_DCP_UUID));v[i]--;}
 assert(j613_25g83_clock(c,5));
 assert(!j613_25g83_clock(NULL,5));
 assert(!j613_25g83_clock(c,4)); assert(!j613_25g83_clock(c,6));
 for(int i=0;i<5;i++){u32 old=c[i];c[i]=i==4?1:0;assert(!j613_25g83_clock(c,5));c[i]=old;}
 assert(j613_25g83_mappings(m,3)); assert(!j613_25g83_mappings(NULL,3));
 assert(!j613_25g83_mappings(m,2)); assert(!j613_25g83_mappings(m,4));
 for(int i=0;i<3;i++){m[i]=0;assert(!j613_25g83_mappings(m,3));m[i]=2;assert(!j613_25g83_mappings(m,3));m[i]=1;}
 u8 surface[DCP_V26_SURFACE_SIZE],swap[DCP_V26_SWAP_SIZE];
 dcp_v26_encode_surface(surface,10240,2560,1600,true);
 assert(get_unaligned_le32(surface+0x0b)==0x42475241);
 assert(get_unaligned_le32(surface+0x29)==16384000);
 assert(surface[2]==1);
 dcp_v26_encode_swap(swap,17,0,surface,0x10000004000ULL,2560,1600,64,0);
 assert(get_unaligned_le32(swap+0x98)==17);
 assert(get_unaligned_le64(swap+0xe38)==0x10000004000ULL);
 assert(get_unaligned_le32(swap+0x110)==64);
 assert(!memcmp(swap+0x588,surface,sizeof(surface)));
 assert(swap[0x1bcb]==0 && swap[0x1bcc]==1);
 dcp_v26_encode_swap(swap,18,0,NULL,0,0,0,0,0);
 assert(get_unaligned_le32(swap+0x98)==18);
 assert(get_unaligned_le64(swap+0xe38)==0 && swap[0x1bcb]==1);
 puts("profile/clock/all-three mapping and v26 swap controls: PASS");
}
''')
        run('cc', '-std=gnu11', '-Wall', '-Werror', '-Wno-unused-function',
            '-I'+str(tmp), '-I'+str(ROOT), str(tmp/'profile.c'), '-o', str(tmp/'profile'))
        run(str(tmp/'profile'))
        # Platform functions are mocked; the bridge's actual decisions and cleanup execute.
        for name in ('device', 'export', 'mutex', 'of'):
            write(tmp, 'linux/'+name+'.h', '#include "mock.h"\n')
        write(tmp, 'linux/errno.h', '#define EPROBE_DEFER 517\n#define ENODEV 19\n#define EINVAL 22\n#define EBUSY 16\n#define EIO 5\n#define ETIMEDOUT 110\n')
        write(tmp, 'linux/kconfig.h', '#define IS_REACHABLE(x) 1\n#define IS_ENABLED(x) 1\n')
        write(tmp, 'mock.h', '''#pragma once
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <pthread.h>
#include <string.h>
#include <linux/types.h>
struct device_node { int id; }; struct device { struct device_node *of_node; bool bound; int refs; };
struct device_link { int unused; };
struct mutex { pthread_mutex_t lock; };
#define DEFINE_MUTEX(n) struct mutex n={PTHREAD_MUTEX_INITIALIZER}
struct mutex_guard { struct mutex *m; };
static struct mutex_guard enter(struct mutex *m){pthread_mutex_lock(&m->lock);return (struct mutex_guard){m};}
static void leave(struct mutex_guard *g){pthread_mutex_unlock(&g->m->lock);}
#define JOIN2(a,b) a##b
#define JOIN(a,b) JOIN2(a,b)
#define guard(x) struct mutex_guard JOIN(g,__LINE__) __attribute__((cleanup(leave))) = enter
#define scoped_guard(x,m) for(bool once=true;once;) for(struct mutex_guard g __attribute__((cleanup(leave)))=enter(m);once;once=false)
#define EXPORT_SYMBOL_GPL(x)
#define DL_FLAG_AUTOREMOVE_CONSUMER 1
static struct device_node *phandle; static bool link_fail;
static struct device_node *of_parse_phandle(struct device_node*n,const char*k,int index){return phandle;}
static void of_node_put(struct device_node*n){}
static bool device_is_bound(struct device*d){return d->bound;}
static struct device *get_device(struct device*d){d->refs++;return d;}
static void put_device(struct device*d){d->refs--;}
static struct device_link *device_link_add(struct device*c,struct device*s,unsigned flags){static struct device_link l;assert(flags==1);return link_fail?NULL:&l;}
/* This harness exercises the J613/25G83 bridge, not the T6030 GFX report. */
static bool of_machine_is_compatible(const char *name){return !strcmp(name,"apple,t8132");}
static bool of_device_is_compatible(const struct device_node*n,const char*name){return false;}
''')
        write(tmp, 'bridge.c', '''#include <stdio.h>
#include "drivers/soc/apple/pmp-export.c"
static int ready_result,send_result,calls;
int apple_pmp_report_wait_supplier_ready(struct device*d,unsigned timeout){assert(d==pmp_device);assert(timeout==5000);return ready_result;}
int apple_pmp_send_power_command(const void*data,u8 command,u16 id,u32 enabled){assert(data==pmp_data);calls++;return send_result;}
int apple_pmp_report_check_t6030_gfx(struct device*d){assert(!"unexpected T6030 report on J613");return -EIO;}
int apple_pmp_report_set_t6030_gfx(struct device*d,bool enabled){assert(!"unexpected T6030 report on J613");return -EIO;}
int main(void){
 struct device_node n={1},wrong={2}; struct device supplier={&n,true,0},consumer={NULL,false,0}; int data;
 assert(apple_pmp_set_device_power(0xf,8,1)==-ENODEV);
 assert(apple_pmp_link_device(&consumer)==-EINVAL);
 phandle=&n;assert(apple_pmp_link_device(&consumer)==-EPROBE_DEFER);
 assert(apple_pmp_register(&supplier,&data)==0);
 assert(apple_pmp_register(&supplier,&data)==-EBUSY);
 supplier.bound=false;assert(apple_pmp_link_device(&consumer)==-EPROBE_DEFER);supplier.bound=true;
 phandle=&wrong;assert(apple_pmp_link_device(&consumer)==-EPROBE_DEFER);phandle=&n;
 link_fail=true;assert(apple_pmp_link_device(&consumer)==-EINVAL);assert(supplier.refs==0);link_fail=false;
 assert(apple_pmp_link_device(&consumer)==0);assert(supplier.refs==0);
 assert(apple_pmp_set_device_power(0xd,8,1)==-EINVAL);
 assert(apple_pmp_set_device_power(0xf,16,1)==-EINVAL);
 assert(apple_pmp_set_device_power(0xf,8,2)==-EINVAL);assert(calls==0);
 ready_result=-EPROBE_DEFER;assert(apple_pmp_set_device_power(0xf,8,1)==-EPROBE_DEFER);assert(calls==0);
 ready_result=-ETIMEDOUT;assert(apple_pmp_set_device_power(0xf,8,1)==-ETIMEDOUT);assert(calls==0);
 ready_result=-EIO;assert(apple_pmp_set_device_power(0xf,8,1)==-EIO);assert(calls==0);
 ready_result=0;send_result=-EIO;assert(apple_pmp_set_device_power(0xf,8,1)==-EIO);assert(calls==1);
 send_result=0;assert(apple_pmp_set_device_power(0xf,8,1)==0);assert(calls==2);
 assert(apple_pmp_set_device_power(0xf,5,1)==0);assert(calls==3);
 assert(apple_pmp_set_device_power(0xe,5,0)==0);assert(calls==4);
 apple_pmp_unregister(&wrong);assert(pmp_data==&data);
 apple_pmp_unregister(&data);assert(!pmp_data && !pmp_device);
 assert(apple_pmp_set_device_power(0xf,8,1)==-ENODEV);
 puts("production PMP bridge supplier/ready/ack controls: PASS");
}
''')
        run('cc', '-std=gnu11', '-pthread', '-Wall', '-Werror', '-I'+str(tmp),
            '-I'+str(ROOT/'include'), '-I'+str(ROOT), str(tmp/'bridge.c'), '-o', str(tmp/'bridge'))
        run(str(tmp/'bridge'))
    if args.dtb_dir:
        verify_dtbs(args.dtb_dir)


def verify_dtbs(directory):
    new = directory/'t8122-j613-25g83.dtb'
    old = directory/'t8122-j613.dtb'
    def get(path,node,prop,kind='s',required=True):
        result=subprocess.run(['fdtget','-t',kind,str(path),node,prop],capture_output=True,text=True)
        if required:
            assert result.returncode==0,(node,prop,result.stderr)
            return result.stdout.strip()
        return result.returncode==0
    assert get(new,'/soc/dcp@28ec00000','apple,j613-25g83-profile','u')=='1'
    assert get(new,'/soc/dcp@28ec00000','apple,firmware-compat','u')=='26 6 2'
    nodes=['/soc/dcp@28ec00000','/soc/dcp@28ec00000/piodma','/soc/display-subsystem',
           '/soc/mailbox@28ec08000','/soc/iommu@28d30c000','/soc/iommu@28d304000',
           '/soc/pmp@2d0500000','/soc/mailbox@2d0c08000','/soc/iommu@2d0300000','/soc/pmp-report@2d03c0000']
    for node in nodes:
        assert get(new,node,'status')=='disabled'
        assert not get(new,node,'apple,j613-25g83-mapping-handoff',required=False)
    assert not get(old,'/soc/dcp@28ec00000','apple,j613-25g83-profile',required=False)
    assert not get(new,'/soc/pmp@2d0500000','apple,tunable-uuid',required=False)
    assert not get(new,'/soc/pmp@2d0500000','apple,board-id',required=False)
    # Find the compiled MTP node, independent of DTS source labels.
    proc=subprocess.run(['dtc','-I','dtb','-O','dts',str(new)],capture_output=True,text=True,check=True)
    assert 'apple,power-method = <0x02>;' in proc.stdout
    proc=subprocess.run(['dtc','-I','dtb','-O','dts',str(old)],capture_output=True,text=True,check=True)
    assert 'apple,power-method = <0x02>;' not in proc.stdout
    print('compiled25/current14 profile/disabled topology/trackpad controls: PASS')


if __name__=='__main__':
    main()
