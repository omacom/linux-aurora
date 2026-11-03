#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Native AFK identity -> deferred OPEN, and nested D121 transport regressions.

Extract production C bodies. Firmware/queues are peers, not hardware proof.
An optional --capture JSON supplies bounded logged prefixes, never fabricated
complete captures. Unlogged dictionaries below are explicitly synthetic.
"""
from pathlib import Path
import argparse,json,os,re,subprocess,tempfile
R=Path(__file__).resolve().parents[2];A=R/'drivers/gpu/drm/apple'
def fn(s,name):
    matches=list(re.finditer(r'(?m)^[^\n;{}]*\b'+name+r'\([^;{}]*\)\n\{',s))
    if not matches: raise RuntimeError('missing body '+name)
    m=matches[0];start=m.start()
    if s[start:m.end()].lstrip().startswith(name+'('):start=s.rfind('\n',0,start-1)+1
    end=m.end();depth=1
    while depth:depth+=(s[end]=='{')-(s[end]=='}');end+=1
    return s[start:end]
def block(s,name):
    start=s.index(name);end=s.index('\n};',start)+3;return s[start:end]
def run(name,code,reject=False):
    with tempfile.TemporaryDirectory() as t:
        p=Path(t)/'test.c';p.write_text(code)
        subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-Wall','-Wextra','-Werror','-Wno-unused-function','-Wno-unused-parameter','-Wno-sign-compare','-Wno-missing-field-initializers','-fsanitize=address,undefined',str(p),'-o',str(p.with_suffix(''))],check=True)
        result=subprocess.run([str(p.with_suffix(''))],capture_output=reject,check=False)
        if reject:
            if result.returncode == 0:raise RuntimeError('mutant survived: '+name)
        else:result.check_returncode()
    print('PASS',name,flush=True)
args=argparse.ArgumentParser();args.add_argument('--capture',type=Path);args.add_argument('--mutants',action='store_true');args=args.parse_args()
pre=r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <limits.h>
typedef uint8_t u8;typedef uint16_t u16;typedef uint32_t u32;typedef uint64_t u64;typedef int64_t s64;typedef uint32_t __le32;
#define __packed __attribute__((packed))
#define GFP_KERNEL 0
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*(x)))
#define round_up(x,a) (((x)+(a)-1)&~((a)-1))
#define ALIGN round_up
#define IS_ERR(p) ((uintptr_t)(p)>=(uintptr_t)-4095)
#define IS_ERR_OR_NULL(p) (!(p)||IS_ERR(p))
#define PTR_ERR(p) ((int)(intptr_t)(p))
#define ERR_PTR(e) ((void *)(intptr_t)(e))
#define min_t(t,a,b) ((t)(a)<(t)(b)?(t)(a):(t)(b))
#define BIT(n) (1UL<<(n))
#define U32_MAX UINT32_MAX
#define S64_MIN INT64_MIN
#define S64_MAX INT64_MAX
#define kmalloc(n,f) malloc(n)
#define kzalloc_obj(v) calloc(1,sizeof(v))
#define kfree(p) free((void *)(p))
#define max(a,b) ((a)>(b)?(a):(b))
#define dev_info(...) ((void)0)
#define dev_err(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define dev_warn(...) ((void)0)
#define dev_warn_ratelimited(...) ((void)0)
#define EXPORT_SYMBOL_IF_KUNIT(...)
#define spin_lock_irqsave(l,f) ((f)=0)
#define spin_unlock_irqrestore(l,f) ((void)(f))
static void put_unaligned_le32(u32 v,void *p){memcpy(p,&v,4);}
static u32 get_unaligned_le32(const void *p){u32 v;memcpy(&v,p,4);return v;}
static u64 get_unaligned_le64(const void *p){u64 v;memcpy(&v,p,8);return v;}
'''
parser=(A/'parser.c').read_text();afk=(A/'afk.c').read_text();dptx=(A/'dptxep.c').read_text();dcp=(A/'dcp.c').read_text();h=(A/'afk.h').read_text()
codec=re.sub(r'^#include[^\n]*\n','',(A/'afkv2-codec.h').read_text(),flags=re.M)
identity=pre+codec+r'''
#define DCP_FIRMWARE_H17P 1
#define DCP_FIRMWARE_V_14_7 2
#define AFK_MAX_CHANNEL 16
#define SYSTEM_ENDPOINT 0x20
#define DISP0_ENDPOINT 0x23
#define EPIC_TYPE_NOTIFY 0
#define EPIC_CAT_REPORT 0
#define bitmap_empty(p,n) 1
#define MAX_PENDING_CMDS 16
struct dcp_parse_ctx {void *dcp;const void *blob;u32 pos,len;};
struct apple_epic_service;struct apple_dcp_afkep;
struct apple_epic_service_ops {const char *name;bool reusable,optional;int (*prepare)(struct apple_epic_service *,const char *,const char *,s64);int (*revalidate)(struct apple_epic_service *,const char *,const char *,s64);void (*init)(struct apple_epic_service *,const char *,const char *,s64);void (*teardown)(struct apple_epic_service *);};
struct apple_epic_service {struct apple_dcp_afkep *ep;const struct apple_epic_service_ops *ops;bool enabled,torndown,pending_open;unsigned users;int lock;u32 channel;void *cookie;const char *pending_name,*pending_class;s64 pending_unit;int cmd_map[1];};
struct dptx_port {struct apple_epic_service *service;s64 unit;bool enabled,connected;int enable_completion;};
struct apple_dcp {bool native,ext26_services_open;int fw_compat,index,dptx_controller_ready;struct {unsigned num_dptx_ports;} hw;struct dptx_port dptxport[2];void *dev;struct apple_dcp_afkep *systemep,*ibootep,*dptx_controller_ep,*dcpavservep,*dptxep,*ext26_aux[3];};
struct apple_dcp_afkep {struct apple_dcp *dcp;u32 endpoint,num_channels;bool wire_v2,stopping,match_epic_name;int lock;void *wq,*service_work;const struct apple_epic_service_ops *ops;struct apple_epic_service services[AFK_MAX_CHANNEL];};
static bool dcp_is_ext26(struct apple_dcp *d){return d->native;}
static bool dcp_is_v14_7_external(struct apple_dcp *d){return false;}
static bool dcp_fw_is_14_7(struct apple_dcp *d){return d->fw_compat==DCP_FIRMWARE_V_14_7;}
static size_t strscpy(char *d,const void *v,size_t n){const char *s=v;size_t l=strnlen(s,n-1);memcpy(d,s,l);d[l]=0;return l;}
static void complete(int *c){(*c)++;}
#define trace_dptxport_init(...)
#define smp_store_release(p,v) (*(p)=(v))
static unsigned opened[256][16],queued,published;
static int open_error;
static void afk_populate_service_debugfs(struct apple_epic_service *s){}
static void afk_recv_handle_teardown(struct apple_dcp_afkep *e,u32 c){}
static void afk_retry_service_init(struct apple_dcp_afkep *ep);
void afk_service_retry(struct apple_dcp_afkep *ep);
bool dptx_native_controllers_ready(struct apple_dcp *dcp);
static int queue_work(void *wq,void *work){struct apple_dcp_afkep *e=wq;assert(e->dcp->ext26_services_open||!e->stopping);queued++;afk_retry_service_init(e);return 1;}
static int afk_send_epic(struct apple_dcp_afkep *e,u32 ch,u16 tag,int ty,int cat,u8 sub,const void *p,size_t n){assert(sub==AFKV2_OPEN&&!n&&ch<16&&e->dcp->ext26_services_open);opened[e->endpoint][ch]++;return open_error;}
'''
identity+=parser[parser.index('#define DCP_PARSE_HEADER'):parser.index('static int parse_bool(')]
# Drop the audio-only helper region from the extracted prefix.
identity=re.sub(r'#if IS_ENABLED\(CONFIG_DRM_APPLE_AUDIO\).*?#endif','',identity,flags=re.S)
identity+=block(parser,'struct iterator {')+'\n'+fn(parser,'iterator_begin')+'\n'+fn(parser,'parse')+'\n'+fn(parser,'parse_epic_service_init')+'\n'
for name in ['afk_service_reclaimable','afk_service_reinit','afk_service_discard','afk_service_publish','afk_service_matches']:
    identity+=fn(h,name)+'\n'
identity+=fn(dptx,'dptx_native_controllers_ready')+'\n'+fn(dptx,'dptx_native_controller_prepare')+'\n'+fn(dptx,'dptx_controller_service_init')+'\n'
for name in ['afk_match_service','afk_epic_find_service','afk_discard_pending_open','afk_retry_service_init','afk_service_retry']:
    identity+=fn(afk,name)+'\n'
service=(A/'dptx-service.h').read_text()
for name in ['dptx_service_port','dptx_service_bind','dptx_service_publish']:
    identity+=fn(service,name)+'\n'
identity+=fn(dptx,'dptxport_prepare')+'\n'+fn(dcp,'dcp_ext26_open_services')+'\n'+fn(afk,'afk_recv_handle_init')+r'''
static void port_init(struct apple_epic_service *s,const char *n,const char *c,s64 u){dptx_service_publish(s);published++;}
static const struct apple_epic_service_ops port_ops[]={{.name="AppleDCPDPTXRemotePort",.prepare=dptxport_prepare,.init=port_init},{0}};
static const struct apple_epic_service_ops controller_ops[]={{.name="AppleDCPDPTXController",.prepare=dptx_native_controller_prepare,.init=dptx_controller_service_init},{0}};
static const struct apple_epic_service_ops av_ops[]={{.name="DCPDPService"},{0}};
struct blob {u8 p[2048];size_t n;};
static void tag(struct blob *b,u32 t,u32 n){b->n=ALIGN(b->n,4);put_unaligned_le32(t<<24|n,b->p+b->n);b->n+=4;}
static void text(struct blob *b,const char *s){tag(b,9,strlen(s));memcpy(b->p+b->n,s,strlen(s));b->n+=strlen(s);}
static void number(struct blob *b,s64 v){tag(b,4,64);memcpy(b->p+b->n,&v,8);b->n+=8;}
static struct blob description(const char *name,const char *class,s64 unit,u32 channel,bool alias_first){
 struct blob b={0};memcpy(b.p,"dispext0",9);b.n=32;put_unaligned_le32(0xd3,b.p+b.n);b.n+=4;tag(&b,1,4);
 if(alias_first){text(&b,"interface-id");number(&b,channel);}
 text(&b,"EPICName");text(&b,name);text(&b,"EPICProviderClass");text(&b,class);text(&b,"EPICUnit");number(&b,unit);
 if(!alias_first){text(&b,"interface-id");number(&b,channel);}return b;
}
static void announce(struct apple_dcp_afkep *e,unsigned ch,struct blob b){
 u8 record[4096];size_t size;struct afkv2_frame f;
 assert(!afkv2_encode_frame(record,sizeof(record),0,ch,0,0x11,AFKV2_REPORT,4,b.p,b.n,&size));
 assert(!afkv2_decode_frame(record,size,false,&f));afk_recv_handle_init(e,f.interface,(void *)f.payload,f.payload_size);
}
int main(void){
 struct apple_dcp d={.native=true,.fw_compat=DCP_FIRMWARE_H17P,.index=1,.hw.num_dptx_ports=2};struct apple_dcp_afkep eps[8]={0};
 struct apple_dcp_afkep **slots[]={&d.systemep,&d.ibootep,&d.dptx_controller_ep,&d.ext26_aux[0],&d.dcpavservep,&d.ext26_aux[1],&d.dptxep,&d.ext26_aux[2]};
 const unsigned ids[]={0x20,0x23,0x24,0x27,0x28,0x29,0x2a,0x2c};
 for(unsigned i=0;i<8;i++){*slots[i]=&eps[i];eps[i]=(struct apple_dcp_afkep){.dcp=&d,.endpoint=ids[i],.wire_v2=true};eps[i].wq=&eps[i];}
 d.dptxep->ops=port_ops;d.dptx_controller_ep->ops=controller_ops;d.dcpavservep->ops=av_ops;
 // Infrastructure dictionaries have interface metadata but no EPIC identity, as on the working daily run.
 struct blob infrastructure={0};memcpy(infrastructure.p,"system",7);infrastructure.n=32;
 put_unaligned_le32(0xd3,infrastructure.p+32);infrastructure.n+=4;tag(&infrastructure,1,2);
 text(&infrastructure,"interface-id");number(&infrastructure,1);text(&infrastructure,"ep-has-desc-mgr");tag(&infrastructure,11,1);
 announce(d.systemep,1,infrastructure);announce(d.ibootep,1,infrastructure);assert(!d.systemep->num_channels&&!d.ibootep->num_channels);
 // Both alias orders, distinct interface and EPICUnit, pass through production AFK/parser/prepare.
 announce(d.dptxep,1,description("dcpdptx-port-epic","AppleDCPDPTXRemotePort",0,1,false));
 announce(d.dptxep,3,description("dcpdptx-port-epic","AppleDCPDPTXRemotePort",1,3,true));
 assert(d.dptxep->num_channels==2&&d.dptxport[0].unit==0&&d.dptxport[1].unit==1);
 assert(!d.dptxport[0].service&&!d.dptxport[1].service);
 for(unsigned i=0;i<4;i++)announce(d.dptx_controller_ep,2*i+1,description(i&1?"dcpdp-controller-epic":"dcpav-controller-epic","AppleDCPDPTXController",i/2,2*i+1,false));
 announce(d.dcpavservep,1,description("dcpav-service-epic","DCPDPService",0,1,false));
 // No service becomes visible and no OPEN goes out before A401.
 assert(!published);for(unsigned i=0;i<8;i++)for(unsigned c=0;c<16;c++)assert(!opened[ids[i]][c]);
 d.ext26_services_open=true;afk_service_retry(d.dptxep);assert(!published&&!opened[0x2a][1]);
 queued=0;dcp_ext26_open_services(&d);assert(queued==9&&published==2&&dptx_native_controllers_ready(&d));
 assert(d.dptxport[0].service&&d.dptxport[1].service&&d.dptxport[0].enabled&&d.dptxport[1].enabled);
 assert(opened[0x2a][1]==1&&opened[0x2a][3]==1&&opened[0x28][1]==1);
 for(unsigned i=0;i<4;i++)assert(opened[0x24][2*i+1]==1);
 dcp_ext26_open_services(&d);assert(published==2&&opened[0x2a][1]==1);
 d.dptx_controller_ep->services[0].enabled=false;assert(!dptx_native_controllers_ready(&d));d.dptx_controller_ep->services[0].enabled=true;
 // Native invalid units still fail closed; truncated identities publish nothing.
 announce(d.dptxep,5,description("dcpdptx-port-epic","AppleDCPDPTXRemotePort",3,5,false));assert(!opened[0x2a][5]);
 struct blob short_b={0};short_b.n=1;announce(d.dptxep,7,short_b);assert(!opened[0x2a][7]);
 // Panel aliases retain their existing meaning; native explicit EPIC keys do not consume interface-id.
 struct blob b=description("dcpdptx-port-epic","AppleDCPDPTXRemotePort",0,3,false);struct dcp_parse_ctx ctx;const char *n,*c;s64 u=-1;
 assert(!parse(b.p+32,b.n-32,&ctx));assert(!parse_epic_service_init(&ctx,&n,&c,&u,false)&&u==0);free((void *)n);free((void *)c);
 assert(!parse(b.p+32,b.n-32,&ctx));assert(!parse_epic_service_init(&ctx,&n,&c,&u,true)&&u==3);free((void *)n);free((void *)c);
 // Preserve panel-only lowercase aliases, which native26 intentionally does not accept.
 struct blob panel={0};put_unaligned_le32(0xd3,panel.p);panel.n=4;tag(&panel,1,3);
 text(&panel,"compartment");text(&panel,"system");text(&panel,"name");text(&panel,"system");text(&panel,"interface-id");number(&panel,1);
 assert(!parse(panel.p,panel.n,&ctx));assert(!parse_epic_service_init(&ctx,&n,&c,&u,true)&&u==1&&!strcmp(n,"system"));free((void *)n);free((void *)c);
 assert(!parse(panel.p,panel.n,&ctx));assert(parse_epic_service_init(&ctx,&n,&c,&u,false)==-ENOENT);
 // A failed OPEN cannot publish a prepared port; closed workqueues are never retried.
 struct apple_dcp_afkep failed={.dcp=&d,.endpoint=0x2a,.wire_v2=true,.ops=port_ops};failed.wq=&failed;
 d.dptxport[0].service=NULL;d.dptxport[0].enabled=false;open_error=-EIO;announce(&failed,1,description("dcpdptx-port-epic","AppleDCPDPTXRemotePort",0,1,false));assert(!d.dptxport[0].service&&!failed.services[0].enabled);
 failed.stopping=true;unsigned q=queued;afk_service_retry(&failed);assert(queued==q);
 d.native=false;failed.stopping=false;afk_service_retry(&failed);assert(queued==q); // classic only retries its EDID endpoint
 for(unsigned i=0;i<8;i++)for(unsigned j=0;j<eps[i].num_channels;j++){free((void *)eps[i].services[j].pending_name);free((void *)eps[i].services[j].pending_class);}
 puts("actual native AFK parser/identity/port admission -> A401 queues all eight endpoints -> exact once OPEN and publication; aliases, truncation, bad units, transport failure and classic retry guard");
}
'''
run('native AFK announcements through deferred OPEN',identity)
# Compile the actual command allocator and ACK dispatch with the production D121
# callback. A peer can write one Boolean byte or all four bytes, then nest D003.
iom=(A/'iomfb.c').read_text();native=(A/'iomfb_h17p.c').read_text();ih=(A/'iomfb.h').read_text();internal=(A/'dcp-internal.h').read_text()
transport=pre+r'''
#define DCP_FIRMWARE_H17P 1
#define DCP_IOMFB_METHODS_H17G 2
#define DCP_IOMFB_METHODS_H17P_EXTERNAL 3
#define DCP_MAX_CALL_DEPTH 8
#define IOMFB_MAX_CB 600
#define DCP_PACKET_ALIGNMENT (0x40)
#define IOMFB_ENDPOINT 0x37
#define IOMFB_MESSAGE_TYPE 15ULL
#define IOMFB_MESSAGE_TYPE_MSG 2
#define IOMFB_MSG_CONTEXT 0xf00ULL
#define IOMFB_MSG_OFFSET 0xffff0000ULL
#define IOMFB_MSG_LENGTH 0xffffffff00000000ULL
#define IOMFB_MSG_ACK 64ULL
#define FIELD_PREP(m,v) (((u64)(v)<<__builtin_ctzll(m))&(m))
#define FIELD_GET(m,v) (((v)&(m))>>__builtin_ctzll(m))
#define WARN_ON(x) assert(!(x))
#define IS_ENABLED(x) 0
#define current ((void *)1)
#define dma_wmb() ((void)0)
#define dma_rmb() ((void)0)
#define trace_iomfb_push(...)
#define IOMFB_METHOD(t,n) [n]={#n,{t[0],t[1],t[2],t[3]},{0},{0}}
#define IOMFB_METHOD_H17(t,h,n) [n]={#n,{t[0],t[1],t[2],t[3]},{h[0],h[1],h[2],h[3]},{0}}
#define IOMFB_METHOD_EXT26(t,e,n) [n]={#n,{t[0],t[1],t[2],t[3]},{0},{e[0],e[1],e[2],e[3]}}
struct apple_dcp;typedef void (*dcp_callback_t)(struct apple_dcp *,void *,void *);
struct dcp_method_entry {const char *name;char tag[4],tag_h17g[4],tag_ext26[4];};
'''
transport+=block(ih,'enum dcp_context_id {')+'\n'+block(ih,'struct dcp_packet_header {')+'\n'+block(internal,'struct dcp_channel {')+'\n'+block(ih,'enum dcpep_method {')+r'''
struct apple_dcp {void *dev;u8 *shmem;int fw_compat;bool native,crashed,quiescing,ext26_blocked,ext26_power_sent;u64 ext26_power_ns;struct {int iomfb_method_profile;}hw;struct {void *owner;} iomfb;struct dcp_channel ch_cb,ch_cmd,ch_oobcb,ch_oobcmd,ch_async,ch_oobasync;};
static bool iomfb_uses_queue(struct apple_dcp *d){return true;}
static bool dcp_is_ext26(struct apple_dcp *d){return d->native;}
static int iomfb_queue_command(struct apple_dcp *d,bool o,const struct dcp_method_entry *m,u32 i,u32 n,void *p,dcp_callback_t cb,void *c){abort();}
static u64 ktime_get_ns(void){return 1;}
static void dcp_send_message(struct apple_dcp *,u8,u64);
static void dcp_ack(struct apple_dcp *,enum dcp_context_id);
static void dcp_push(struct apple_dcp *,bool,const struct dcp_method_entry *,u32,u32,void *,dcp_callback_t,void *);
static void boot_done(struct apple_dcp *,void *,void *);
static unsigned commands,callback_acks;static int bad_status,corrupt;static bool byte_reply;static u8 memory[0x100000];
'''
for name in ['dcp_tx_offset','dcp_call_context','dcp_get_channel','dcp_packet_start','dcp_push_depth','dcp_pop_depth','dcpep_msg','dcpep_ack','dcp_push','dcp_ack','dcpep_handle_ack']:
    code=fn(iom,name)
    if name in ['dcp_push','dcp_ack']:code='static '+code
    transport+=code+'\n'
transport+=block(native,'static const struct dcp_method_entry dcp_methods[')+'\n'+block(native,'struct ext26_boot {')+'\n'
for name in ['ext26_boot_next','ext26_boot_callback','ext26_bandwidth']:
    transport+=fn(native,name)+'\n'
transport+=fn((A/'iomfb_template.c').read_text(),'boot_done')+r'''
static void callback(struct apple_dcp *d,int tag,unsigned in,unsigned out,unsigned off){
 struct dcp_channel *ch=&d->ch_cb;u8 *record=d->shmem+0x60000+off;
 // Firmware-originated callback record; production handlers and ACK share this backing.
 memset(record,0,12+in+out);struct dcp_packet_header *h=(void *)record;h->in_len=in;h->out_len=out;
 if(tag==3){u8 raw[4]={0x00,0x0b,0x20,0xf1};memcpy(record+12,raw,4);} // daily D003 input, captured in full
 unsigned depth=dcp_push_depth(&ch->depth);ch->output[depth]=record+12+in;ch->end[depth]=off+ALIGN(12+in+out,64);
 bool ack=tag==121?ext26_boot_callback(d,tag,record+12+in,record+12):ext26_bandwidth(d,tag,record+12+in,record+12);
 if(ack)dcp_ack(d,DCP_CONTEXT_CB);
}
static void dcp_send_message(struct apple_dcp *d,u8 ep,u64 message){
 assert(ep==IOMFB_ENDPOINT);enum dcp_context_id context=FIELD_GET(IOMFB_MSG_CONTEXT,message);
 if(message&IOMFB_MSG_ACK){callback_acks++;return;}
 unsigned offset=FIELD_GET(IOMFB_MSG_OFFSET,message);struct dcp_packet_header *h=(void *)(d->shmem+dcp_tx_offset(context)+offset);u8 *in=(u8 *)(h+1),*out=in+h->in_len;
 const char *tags[]={"983A","644A","920A","864A","000A","564A"};unsigned inputs[]={0,0,0,4,4,0},outputs[]={0,4,0,0,4,4};
 assert(commands<6&&!memcmp(h->tag,tags[commands],4)&&h->in_len==inputs[commands]&&h->out_len==outputs[commands]);
 assert(context==DCP_CONTEXT_CB&&offset>=64);if(h->in_len)assert(get_unaligned_le32(in)==1);
 unsigned call=++commands;
 if(call==5){callback(d,3,4,60,64);assert(get_unaligned_le32(d->shmem+0x60000+64+12+4+56)==1);}
 if(h->out_len){u32 value=call>=5?1:0;if(bad_status==call)value=0;if(byte_reply)*out=value;else put_unaligned_le32(value,out);}
 if(corrupt==call)h->tag[0]^=1;
 dcpep_handle_ack(d,context);
}
int main(void){
 for(unsigned full=0;full<2;full++){
  memset(memory,0,sizeof(memory));struct apple_dcp d={.shmem=memory,.native=true,.fw_compat=1,.iomfb.owner=current};d.ch_cmd.depth=1;commands=callback_acks=0;byte_reply=!full;
  callback(&d,121,0,4,0);assert(!d.crashed&&commands==6&&callback_acks==2&&!d.ch_cb.depth&&get_unaligned_le32(memory+0x60000+12)==1);
 }
 for(bad_status=5;bad_status<=6;bad_status++){
  memset(memory,0,sizeof(memory));struct apple_dcp d={.shmem=memory,.native=true,.fw_compat=1,.iomfb.owner=current};d.ch_cmd.depth=1;commands=callback_acks=0;byte_reply=true;callback(&d,121,0,4,0);assert(d.crashed&&commands==bad_status&&callback_acks==1&&d.ch_cb.depth==1);
 }
 bad_status=0;
 for(corrupt=1;corrupt<=6;corrupt++){
  memset(memory,0,sizeof(memory));struct apple_dcp d={.shmem=memory,.native=true,.fw_compat=1,.iomfb.owner=current};d.ch_cmd.depth=1;commands=callback_acks=0;callback(&d,121,0,4,0);assert(d.crashed&&commands==corrupt);
 }
 puts("actual dcp_push/shared memory/ACK/D121 nested D003: Boolean byte and word replies pass, semantic refusal and corrupt command records stop successors and retain the session");
}
'''
run('native D121 through actual shared-memory command and ACK bodies',transport)
if args.capture:
    capture=json.loads(args.capture.read_text());assert capture['candidate_raw_messages_logged'] is False
    for record in capture['daily_afk_prefixes']:
        b=bytes.fromhex(record['hex']);assert b[:4]==b'IOP ' and len(b)==128
        declared=int.from_bytes(b[20:24],'little')+8
        assert len(b)-16<declared and record['truncated'] is True
    print('PASS logged prefixes are explicitly truncated, not padded or claimed as full candidate records')

if args.mutants:
    old_alias=re.sub(r'ep->dcp->fw_compat == DCP_FIRMWARE_H17P &&\s*!dcp_is_ext26\(ep->dcp\)', 'ep->dcp->fw_compat == DCP_FIRMWARE_H17P', identity)
    if old_alias==identity:raise RuntimeError('identity mutation missed')
    run('reject restored panel interface-id alias on native port',old_alias,True)
    old_retry=identity.replace('(!dcp_is_ext26(ep->dcp) && ep != ep->dcp->dcpavservep)', 'ep != ep->dcp->dcpavservep')
    if old_retry==identity:raise RuntimeError('retry mutation missed')
    run('reject restored EDID-only deferred OPEN queue',old_retry,True)
    old_padding=transport.replace('dcp_is_ext26(dcp) ? 0 : 0xff', '0xff')
    if old_padding==transport:raise RuntimeError('padding mutation missed')
    run('reject restored native all-ones Boolean reply padding',old_padding,True)
