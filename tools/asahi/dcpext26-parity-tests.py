#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Ordered native startup and the native DP callback contract, actual C bodies.

Synthetic transport/MMIO boundaries. This cannot prove a physical picture or PM.
"""
from pathlib import Path
import os,re,subprocess,tempfile
R=Path(__file__).resolve().parents[2]
def fn(s,n):
 m=re.search(r'^(?:static )?(?:int|bool|void) '+n+r'\([^;]*?\)\n\{',s,re.M|re.S);assert m,n
 i=m.end();depth=1
 while depth:depth+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[m.start():i]
def run(name,code):
 with tempfile.TemporaryDirectory() as t:
  p=Path(t)/'test.c';p.write_text(code)
  subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-parameter','-Wno-missing-field-initializers','-Wno-sign-compare','-fsanitize=address,undefined',str(p),'-o',str(p.with_suffix(''))],check=True)
  subprocess.run([str(p.with_suffix(''))],check=True)
 print('PASS',name)
dcp=(R/'drivers/gpu/drm/apple/dcp.c').read_text()
pre=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <errno.h>
typedef uint8_t u8;typedef uint16_t u16;typedef uint32_t u32;typedef uint64_t u64;
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,y) ((x)=(y))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*(x)))
#define IS_ERR(p) ((intptr_t)(p)<0)
#define PTR_ERR(p) ((int)(intptr_t)(p))
#define ERR_PTR(e) ((void *)(intptr_t)(e))
#define dev_info(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define APPLE_DCP_COPROC_CPU_CONTROL 0x44
#define APPLE_DCP_COPROC_CPU_CONTROL_RUN 16
#define APPLE_DCP_COPROC_CPU_STATUS 0x48
#define msecs_to_jiffies(x) (x)
struct device {int unused;};
struct apple_dcp_afkep {u8 endpoint;bool match_epic_name;};
struct apple_dcp {struct device *dev;u8 *coproc_reg;void *rtk;bool crashed;int ext26_link_ready;
 struct apple_dcp_afkep *ext26_aux[3];};
static char trace[256];static unsigned used;static char fault;
static void note(char c){assert(used<255);trace[used++]=c;trace[used]=0;}
static int step(char c){note(c);return fault==c?-EIO:0;}
static void reset(char f){memset(trace,0,sizeof(trace));used=0;fault=f;}
static int dev_err_probe(struct device *d,int e,const char *fmt,...){return e;}
static u32 readl(void *p){u32 x;memcpy(&x,p,4);return x;}
static void writel(u32 x,void *p){assert(x&16);note('R');memcpy(p,&x,4);}
static int dcp_pin_live_session(struct apple_dcp *d){return step('P');}
static const int rtkit_ops;
static void *apple_rtkit_init(struct device *dev,struct apple_dcp *d,const char *name,int channel,const int *ops){assert(!strcmp(name,"mbox")&&!channel&&ops==&rtkit_ops);return step('I')?ERR_PTR(-EIO):d;}
static void dcp_release_rtkit(void *d){}
static int devm_add_action_or_reset(struct device *d,void (*action)(void *),void *p){return step('A');}
static int apple_rtkit_wake(void *r){return step('W');}
static int apple_rtkit_boot(void *r){return step('B');}
static int iomfb_start_rtkit(struct apple_dcp *d){return step('L');}
static unsigned wait_for_completion_timeout(int *c,unsigned t){note('l');return fault!='l';}
static int systemep_init(struct apple_dcp *d){return step('0');}
static int ibootep_init(struct apple_dcp *d){return step('3');}
static int dptx_controller_init(struct apple_dcp *d){return step('4');}
static int dpavservep_init(struct apple_dcp *d){return step('8');}
static int dptxep_init(struct apple_dcp *d){return step('a');}
static const int ext26_aux_ops[1];
static struct apple_dcp_afkep aux[3];static unsigned auxiliary;
static struct apple_dcp_afkep *afk_init(struct apple_dcp *d,u8 ep,const int *ops){assert(ops==ext26_aux_ops);struct apple_dcp_afkep *a=&aux[auxiliary++%3];a->endpoint=ep;return a;}
static int afk_start(struct apple_dcp_afkep *ep){char c=ep->endpoint==0x27?'7':ep->endpoint==0x29?'9':'c';return step(c);}
static void iomfb_ext26_start(struct apple_dcp *d){note('S');}
'''
checks=r'''
int main(void){
 u8 mmio[0x50]={0};struct device dev={0};struct apple_dcp d={.dev=&dev,.coproc_reg=mmio};u32 run=16;
 reset(0);memcpy(mmio+0x44,&run,4);assert(!dcp_ext26_start_resident(&d));assert(!strcmp(trace,"PIAW"));
 reset(0);memset(mmio,0,sizeof(mmio));assert(!dcp_ext26_start_resident(&d));assert(!strcmp(trace,"PIARB")&&readl(mmio+0x44)==16);
 const char stages[]="PIAWB";
 for(unsigned i=0;i<sizeof(stages)-1;i++){
  bool warm=stages[i]!='B';run=warm?16:0;memcpy(mmio+0x44,&run,4);reset(stages[i]);
  assert(dcp_ext26_start_resident(&d)==-EIO);
  assert(!strchr(trace,warm?'B':'W')); /* no fallback or second INIT */
  if(stages[i]=='P'||stages[i]=='I'||stages[i]=='A')assert(!strchr(trace,'R'));
 }
 reset(0);assert(!dcp_ext26_start_services(&d));assert(!strcmp(trace,"Ll034789acS"));
 for(const char *p="Ll034789ac";*p;p++){
  reset(*p);assert(dcp_ext26_start_services(&d)<0);assert(!strchr(trace,'S'));
 }
 reset(0);d.crashed=true;assert(dcp_ext26_start_services(&d)==-EIO&&!strchr(trace,'0'));
 puts("ordered actual startup: pin/mailbox/action before RUN; one warm INIT or resident boot; DCPLink then all eight AFK then A401; every boundary fault prevents successor");
}
'''
run('native first-session order',pre+fn(dcp,'dcp_ext26_start_resident')+fn(dcp,'dcp_ext26_start_services')+checks)
# The complete observed callback shape set, tested against the real validators.
s=(R/'drivers/gpu/drm/apple/iomfb_h17p.c').read_text()
a=s.index('struct dcp_callback_size {');b=s.index('\nstruct dcp_h17p_hotplug_request',a)
cb=s[a:b]
corpus={0:(0,4),1:(0,4),2:(0,0),3:(4,60),6:(84,80),100:(0,0),101:(0,4),102:(68,0),104:(68,0),107:(64,0),108:(0,4),109:(0,4),110:(0,4),111:(0,4),112:(0,4),113:(0,4),114:(4164,4100),115:(16,8),116:(16,4),121:(0,4),123:(0,4),125:(100,36),127:(4,4),128:(4104,4),129:(64,4),134:(0,4),201:(12,16),206:(0,4),207:(0,4),209:(0,8),300:(16,0),401:(80,12),408:(8,8),413:(4168,4),414:(80,4),415:(76,4),451:(20,28),454:(4,4),552:(4164,4),561:(4164,4),563:(76,4),565:(72,4),567:(128,4),572:(4,4),575:(88,76),576:(4,0),582:(8,4),585:(228,224),588:(0,0),590:(1840,0),593:(4,0),599:(0,0)}
run('observed and explicit native callback corpus',pre[:pre.index('struct device {')]+ '#define IOMFB_MAX_CB 600\n'+cb+fn(s,'ext26_callback_size')+'\nint main(void){\n'+''.join(f'assert(ext26_callback_size({t},{i},{o}));\n' for t,(i,o) in corpus.items())+'puts("all native callback shapes admitted by production dispatch");}\n')
# DP callback replies and PHY/crossbar ordering, isolated from unrelated HDMI paths.
s=(R/'drivers/gpu/drm/apple/dptxep.c').read_text();h=(R/'drivers/gpu/drm/apple/dptxep.h').read_text();a=h.index('enum dptx_apcall {');b=h.index('\n};',a)+3
pre2=pre[:pre.index('struct device {')]+h[a:b]+r'''
#define LINK_RATE_HBR2 0x14
#define LINK_RATE_RBR 6
#define LINK_RATE_HBR 10
#define LINK_RATE_HBR3 30
#define PHY_MODE_DP 1
#define min(a,b) ((a)<(b)?(a):(b))
static void put_unaligned_le32(u32 v,void *p){memcpy(p,&v,4);}
static void put_unaligned_le64(u64 v,void *p){memcpy(p,&v,8);}
static u64 get_unaligned_le64(const void *p){u64 v;memcpy(&v,p,8);return v;}
static u32 get_unaligned_le32(const void *p){u32 v;memcpy(&v,p,4);return v;}
static u32 word(const void *p){u32 v;memcpy(&v,p,4);return v;}
struct apple_dcp {int unused;};struct ep {struct apple_dcp *dcp;};
struct dptx_port {void *atcphy;u32 lane_count,native26_max_lanes,link_rate;int linkcfg_completion;};
struct apple_epic_service {struct ep *ep;void *cookie;};
union phy_configure_opts {struct {u32 lanes;} dp;};
static int fault,up,down,delay,fallback,training;static unsigned capacity=4;
static int phy_validate(void *phy,int mode,int n,union phy_configure_opts *v){v->dp.lanes=capacity;return fault;}
static int dcp_direct_crossbar_link(struct apple_dcp *d,bool on){if(on)up++;else down++;return fault;}
static void mdelay(unsigned n){assert(n==10);delay++;}
static void complete(int *c){(*c)++;}
static int dptx_j700_training(struct apple_epic_service *s,bool w,const void *d,size_t n,void *r,size_t room){training++;return 0;}
static int dptx_j700_validate_lanes(struct dptx_port *p,u64 n){return n<=capacity&&n!=3?0:-EINVAL;}
static int dptxport_call_set_link_rate(struct apple_epic_service *s,const void *d,size_t n,void *r,size_t room){assert(down);return 0;}
static int dptxport_call(struct apple_epic_service *s,u32 cmd,const void *d,size_t n,void *r,size_t room){fallback++;return 0;}
'''
run('native DP callback wire and ordering',pre2+fn(s,'dptxport_native26_call')+r'''
int main(void){
 struct apple_dcp d={0};struct ep ep={&d};struct dptx_port p={.atcphy=&d};struct apple_epic_service svc={&ep,&p};u8 input[160]={0},out[160];
 #define CALL(c,n,r) dptxport_native26_call(&svc,0,(c),input,(n),out,(r))
 memset(out,0xa5,sizeof(out));assert(!CALL(2,32,32));assert(word(out+20)==3&&word(out+24)==3&&!word(out+16));
 assert(!CALL(7,32,32)&&word(out+16)==0x14);
 assert(!CALL(18,32,32)&&word(out+16)==1);assert(!CALL(21,160,160)&&word(out)==1);
 assert(!CALL(10,32,32)&&p.native26_max_lanes==4&&!p.lane_count);
 put_unaligned_le64(4,input+16);assert(!CALL(12,32,32)&&p.lane_count==4&&p.linkcfg_completion==1);
 put_unaligned_le64(3,input+16);assert(CALL(12,32,32)==-EINVAL);
 assert(!CALL(0,16,16)&&up==1);assert(!CALL(1,16,16)&&down==1);
 assert(!CALL(5,16,16)&&down==2);p.link_rate=0x14;assert(!CALL(6,16,16)&&delay==1&&up==2);
 fault=-EIO;assert(CALL(0,16,16)==-EIO);put_unaligned_le64(0x14,input+16);assert(CALL(9,32,32)==-EIO&&!fallback);fault=0;
 put_unaligned_le64(0x99,input+16);assert(CALL(9,32,32)==-EINVAL);
 assert(!dptxport_native26_call(&svc,1,3,input,80,out,80)&&training==1);
 assert(dptxport_native26_call(&svc,2,3,input,80,out,80)==-EOPNOTSUPP);
 puts("native DP HPD/tiled=1, drive offsets20/24, maxHBR2, distinct max/active lanes, crossbar errors propagated");
}
''')
# Deferred OPEN executes only after A401, including ops without a prepare/revalidate hook.
s=(R/'drivers/gpu/drm/apple/afk.c').read_text()
pre3=pre[:pre.index('struct device {')]+r'''
#define kfree(p) ((void)0)
#define dev_err(...) ((void)0)
#define EPIC_TYPE_NOTIFY 1
#define EPIC_CAT_REPORT 2
#define AFKV2_OPEN 3
struct apple_epic_service;
struct apple_epic_service_ops {int (*revalidate)(struct apple_epic_service *,const char *,const char *,int);void (*init)(struct apple_epic_service *,const char *,const char *,int);};
struct apple_epic_service {bool pending_open,enabled;const struct apple_epic_service_ops *ops;const char *pending_name,*pending_class;int pending_unit;u32 channel;};
struct apple_dcp {bool ext26_services_open;struct apple_dcp_afkep *dptxep;};
static bool dptx_native_controllers_ready(struct apple_dcp *d){return true;}
struct apple_dcp_afkep {struct apple_dcp *dcp;unsigned num_channels;struct apple_epic_service services[2];};
static bool dcp_is_ext26(struct apple_dcp *d){return true;}
static unsigned sent,published,discarded,initialized;static int fault;
static void afk_discard_pending_open(struct apple_epic_service *s){discarded++;s->pending_open=false;}
static void *afk_epic_find_service(struct apple_dcp_afkep *e,u32 c){return NULL;}
static int afk_send_epic(struct apple_dcp_afkep *e,u32 c,u16 tag,int type,int cat,u8 subtype,const void *data,size_t size){assert(c&&subtype==AFKV2_OPEN&&!size);sent++;return fault;}
static void afk_service_publish(struct apple_epic_service *s){published++;s->enabled=true;}
static void afk_populate_service_debugfs(struct apple_epic_service *s){}
static void initialized_service(struct apple_epic_service *s,const char *n,const char *c,int u){initialized++;}
'''
run('A401 service OPEN barrier',pre3+fn(s,'afk_retry_service_init')+r'''
int main(void){struct apple_dcp d={0};struct apple_epic_service_ops ops={.init=initialized_service};struct apple_dcp_afkep e={.dcp=&d,.num_channels=1};e.services[0]=(struct apple_epic_service){.pending_open=true,.ops=&ops,.channel=1};
 afk_retry_service_init(&e);assert(!sent&&!published&&!initialized&&e.services[0].pending_open);
 d.ext26_services_open=true;afk_retry_service_init(&e);assert(sent==1&&published==1&&initialized==1&&!e.services[0].pending_open&&e.services[0].enabled);
 afk_retry_service_init(&e);assert(sent==1);ops.init=NULL;e.services[0].pending_open=true;fault=-EIO;afk_retry_service_init(&e);assert(discarded==1&&published==1);
 puts("native deferred OPEN survives absent revalidate/init hooks, opens after A401 once, rejects failed transport");}
''')
# Native property transfer's sentinel count and replacing a prior transfer.
s=(R/'drivers/gpu/drm/apple/iomfb_h17p.c').read_text()
# fn() also accepts the u8 return type for this small callback.
m=re.search(r'^static u8 ext26_prop_start\([^;]*?\)\n\{',s,re.M|re.S);a=m.start();i=m.end();depth=1
while depth:depth+=(s[i]=='{')-(s[i]=='}');i+=1
run('native property transfer count',pre[:pre.index('struct device {')]+r'''
#define SZ_1M (1U<<20)
#define GFP_KERNEL 0
#define kfree free
#define max_t(t,a,b) ((t)(a)>(t)(b)?(t)(a):(t)(b))
static bool fail_alloc;
static void *kzalloc(size_t n,int f){return fail_alloc?NULL:calloc(1,n);}
struct dcp_chunks {void *data;size_t length,received;bool contiguous;u64 modes_generation;};
struct apple_dcp {struct dcp_chunks chunks;};
static u64 generation;
static u64 dcp_modes_transfer_begin(struct apple_dcp *d){return ++generation;}
static void dcp_firmware_metadata_begin(struct apple_dcp *d){}
'''+s[a:i]+r'''
int main(void){struct apple_dcp d={0};assert(!ext26_prop_start(&d,0));assert(!ext26_prop_start(&d,(1U<<20)+2));
 assert(ext26_prop_start(&d,4097)&&d.chunks.length==4096&&d.chunks.contiguous&&!d.chunks.received);
 memset(d.chunks.data,0x55,4096);d.chunks.received=11;assert(ext26_prop_start(&d,2)&&d.chunks.length==1&&!d.chunks.received&&!*(u8 *)d.chunks.data);
 assert(ext26_prop_start(&d,1)&&d.chunks.length==0&&d.chunks.data);free(d.chunks.data);d.chunks.data=NULL;
 fail_alloc=true;assert(!ext26_prop_start(&d,4097)&&!d.chunks.data);puts("native D127 count-1, replaced transfer, empty payload, bounds and allocation refusal");}
''')
# Physical-address allocation and PIODMA lifetime: exercise the common allocator's
# native branch, plus the native wire admission, rather than a model allocator.
s=(R/'drivers/gpu/drm/apple/iomfb_template.c').read_text();pos=s.index('dcpep_cb_allocate_buffer(struct');a=s.rfind('static struct ',0,pos);i=s.index('\n}',pos)+2
alloc=s[a:i]
h=(R/'drivers/gpu/drm/apple/iomfb_h17p.c').read_text()
pre4=pre[:pre.index('struct device {')]+r'''
#define SZ_16K 16384UL
#define DCP_MAX_MAPPINGS 128
#define GFP_KERNEL 0
#define DMA_ATTR_FORCE_CONTIGUOUS 8
#define DCP_FW_VERSION(a,b,c) ((a)*10000+(b)*100+(c))
#define DCP_FW_VER DCP_FW_VERSION(26,6,0)
#define DCP_FW_NAME(n) n
#define round_down(x,a) ((x)&~((__typeof__(x))(a)-1))
#define check_add_overflow(a,b,p) __builtin_add_overflow((a),(b),(p))
#define dev_warn(...) ((void)0)
struct scatterlist {u64 phys;};
struct sg_table {struct scatterlist *sgl;unsigned orig_nents;};
struct dcp_mem_descriptor {size_t size;void *buf;u64 dva;struct sg_table map;bool piodma_mapped;};
struct apple_dcp {bool native;void *dev;struct dcp_mem_descriptor memdesc[128];unsigned long memdesc_map[2],ext26_released[2];};
struct dcp_allocate_buffer_req {u64 size;};
struct dcp_allocate_buffer_resp {u32 mem_desc_id;u64 dva_size,dva,paddr;};
static bool dcp_is_ext26(struct apple_dcp *d){return d->native;}
static bool test_bit(unsigned id,const unsigned long *b){return !!(b[id/64]&(1UL<<(id%64)));}
static void set_bit(unsigned id,unsigned long *b){b[id/64]|=1UL<<(id%64);}
static unsigned find_first_zero_bit(unsigned long *b,unsigned n){for(unsigned i=0;i<n;i++)if(!test_bit(i,b))return i;return n;}
static unsigned attributes,frees,maps;static bool fragmented;
static void *dma_alloc_attrs(void *d,size_t n,u64 *address,int f,unsigned attrs){attributes=attrs;*address=0x800000;void *p=NULL;assert(!posix_memalign(&p,16384,n));return p;}
static void *dma_alloc_coherent(void *d,size_t n,u64 *a,int f){attributes=0;*a=0x800000;return calloc(1,n);}
static int dma_get_sgtable(void *d,struct sg_table *t,void *buf,u64 a,size_t n){static struct scatterlist sg={0x200000};t->sgl=&sg;t->orig_nents=fragmented?2:1;return 0;}
static void sg_free_table(struct sg_table *t){}
static u64 sg_phys(struct scatterlist *s){return s->phys;}
static void dma_free_attrs(void *d,size_t n,void *p,u64 a,unsigned attrs){assert(attrs==8);frees++;free(p);}
static void dma_free_coherent(void *d,size_t n,void *p,u64 a){frees++;free(p);}
static u64 get_unaligned_le64(const void *p){u64 v;memcpy(&v,p,8);return v;}
static u32 get_unaligned_le32(const void *p){u32 v;memcpy(&v,p,4);return v;}
static void put_unaligned_le32(u32 v,void *p){memcpy(p,&v,4);}
static void put_unaligned_le64(u64 v,void *p){memcpy(p,&v,8);}
static bool trampoline_map_piodma_h17p(struct apple_dcp *d,int tag,void *out,void *in){u64 id=get_unaligned_le64(in);maps++;d->memdesc[id].piodma_mapped=true;put_unaligned_le32(id,out);put_unaligned_le64(d->memdesc[id].dva,(u8 *)out+4);return true;}
'''
run('native contiguous buffer and PIODMA wire ownership',pre4+alloc+fn(h,'ext26_map_piodma')+r'''
int main(void){struct apple_dcp d={.native=true};set_bit(0,d.memdesc_map);struct dcp_allocate_buffer_req q={4096};
 struct dcp_allocate_buffer_resp r=dcpep_cb_allocate_buffer(&d,&q);assert(r.mem_desc_id==1&&r.dva_size==16384&&r.paddr==0x200000&&r.dva==0x800000&&attributes==8);
 u8 in[12]={0},out[16];put_unaligned_le64(1,in);put_unaligned_le32(0,in+8);ext26_map_piodma(&d,201,out,in);assert(!maps&&!get_unaligned_le32(out));
 put_unaligned_le32(1,in+8);ext26_map_piodma(&d,201,out,in);assert(maps==1&&get_unaligned_le32(out)==1&&get_unaligned_le64(out+4)==r.dva&&!get_unaligned_le32(out+12));
 ext26_map_piodma(&d,201,out,in);assert(maps==1&&get_unaligned_le64(out+4)==r.dva);set_bit(1,d.ext26_released);ext26_map_piodma(&d,201,out,in);assert(maps==1&&!get_unaligned_le32(out)&&d.memdesc[1].buf);
 fragmented=true;r=dcpep_cb_allocate_buffer(&d,&q);assert(!r.mem_desc_id&&frees==1&&!test_bit(2,d.memdesc_map));
 free(d.memdesc[1].buf);puts("native D451 physically contiguous16K reply and D201 word1, packed16-byte reply, repeat mapping and logical retention");}
''')
