#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Native AV identity/attachment predicates and public EDID wire validation."""
from pathlib import Path
import os,re,subprocess,tempfile
ROOT=Path(__file__).resolve().parents[2]
source=(ROOT/'drivers/gpu/drm/apple/epic/dpavservep.c').read_text()
def function(name):
 m=re.search(r'^(?:static )?bool '+name+r'\(',source,re.M);assert m
 return source[m.start():source.index('\n}',m.start())+2]
pre=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include "epic/dpavserv-edid.h"
typedef int64_t s64;
#define DPAVSERV_ENDPOINT 0x28
#define DPAVSERV_ENDPOINT_V2 0x2a
#define U16_MAX 65535
#define DCP_IOMFB_METHODS_H17P_EXTERNAL 2
#define READ_ONCE(x) (x)
#define lockdep_assert_held(x) ((void)0)
struct apple_epic_service;
struct apple_dcp {
 bool native,typec_cable_connected,quiescing,external_physically_absent;u64 typec_generation;int hpd_mutex;
 struct { int iomfb_method_profile; } hw;
 struct { int lock;bool enabled;u64 epoch;struct apple_epic_service *service; } dcpavserv;
};
struct ep { struct apple_dcp *dcp;u32 endpoint;bool wire_v2; };
struct apple_epic_service { struct ep *ep;u32 channel; };
static bool dcp_is_ext26(struct apple_dcp *d) { return d->native; }
'''
check=r'''
static void checksum(u8 *b) { u8 sum=0;b[127]=0;for(unsigned i=0;i<127;i++)sum+=b[i];b[127]=-sum; }
int main(void) {
 struct apple_dcp d={.native=true,.typec_cable_connected=true,.typec_generation=7};
 struct ep ep={&d,0x28,true};struct apple_epic_service svc={&ep,1};
 assert(dcpavserv_identity(&svc,"dcpav-service-epic","DCPDPService",0));
 assert(!dcpavserv_identity(&svc,"dcpav-service-epic","DCPDPService",1));
 assert(!dcpavserv_identity(&svc,"dcpav-service-epic","DCPDP13Service",0));
 ep.endpoint=0x2a;assert(!dcpavserv_identity(&svc,"dcpav-service-epic","DCPDPService",0));
 ep.endpoint=0x28;svc.channel=0;assert(!dcpavserv_identity(&svc,"dcpav-service-epic","DCPDPService",0));svc.channel=1;
 d.dcpavserv.enabled=true;d.dcpavserv.service=&svc;d.dcpavserv.epoch=10;
 assert(dcpavserv_edid_current(&d,7)&&dcpavserv_edid_service_current(&d,10));
 assert(!dcpavserv_edid_current(&d,6)&&!dcpavserv_edid_service_current(&d,9));
 d.external_physically_absent=true;assert(!dcpavserv_edid_current(&d,7)&&!dcpavserv_edid_service_current(&d,10));
 d.external_physically_absent=false;d.quiescing=true;assert(!dcpavserv_edid_current(&d,7));
 d.quiescing=false;d.typec_cable_connected=false;assert(!dcpavserv_edid_current(&d,7));
 u8 wire[48+8+256]={0};u8 *base=wire+56;const u8 header[]={0,255,255,255,255,255,255,0};
 afkv2_put64(wire,EDID_BUF_SIZE);afkv2_put64(wire+32,8+256);memcpy(base,header,8);base[126]=1;checksum(base);checksum(base+128);
 size_t size=0;assert(!dpavserv_edid_size(wire,sizeof(wire),&size)&&size==256);
 for(size_t n=0;n<sizeof(wire);n++)assert(dpavserv_edid_size(wire,n,&size));
 base[130]=1;assert(dpavserv_edid_size(wire,sizeof(wire),&size));base[130]=0;
 base[126]=2;checksum(base);assert(dpavserv_edid_size(wire,sizeof(wire),&size));
 puts("PASS native26 AV identity, stale attachment/owner, bounded EDID length, extension count and checksum failures");
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t);(p/'check.c').write_text(pre+'\n'.join(function(n) for n in ['dcpavserv_identity','dcpavserv_edid_current','dcpavserv_edid_service_current'])+check)
 subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined','-I'+str(ROOT/'tools/testing/selftests/drivers/gpu/apple-afkv2/host-include'),'-I'+str(ROOT/'drivers/gpu/drm/apple'),str(p/'check.c'),'-o',str(p/'check')],check=True)
 subprocess.run([str(p/'check')],check=True)
