# SPDX-License-Identifier: GPL-2.0-only OR MIT
# Copyright 2026 Ryan Murray
"""Actual native EDID admission, service epochs, stale replies and RX progress."""
from pathlib import Path
from common import block, source_defines, apple_function, internal_inline

def rebind(root):
    base = root / 'drivers/gpu/drm/apple'
    def function(file,signature):
        text=(base/file).read_text();start=text.index(signature);opening=text.index('{',start)
        while ';' in text[start:opening]:
            start=text.index(signature,start+1);opening=text.index('{',start)
        end,depth=opening+1,1
        while depth:
            depth+=(text[end]=='{')-(text[end]=='}');end+=1
        return text[start:end]
    prefix=r'''
#undef NDEBUG
#include <assert.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>
#include <stddef.h>
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#include "afkv2-codec.h"
typedef int64_t s64;
typedef u64 dma_addr_t;
#define U16_MAX UINT16_MAX
#define EBUSY 16
#define EOPNOTSUPP 95
#define ENOENT 2
#define MAX_PENDING_CMDS 16
#define AFK_MAX_CHANNEL 16
#define AFKV2_INTERFACES (1U << 16)
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define WARN_ON(x) (x)
#define bitmap_empty(p,n) (!*(p))
#define spin_lock_irqsave(p,f) do { assert(!*(p));*(p)=true;(f)=0; } while(0)
#define spin_unlock_irqrestore(p,f) do { (void)(f);assert(*(p));*(p)=false; } while(0)
static int test_bit(unsigned int bit,const unsigned long *p) { return !!(*p & (1UL<<bit)); }
static void bitmap_release_region(unsigned long *p,unsigned int bit,int order) { (void)order;*p&=~(1UL<<bit); }
#define le32_to_cpu(x) (x)
#define cpu_to_le32(x) (x)
#define dev_warn(...) ((void)0)
#define dev_err(...) do { diagnostic_errors++; } while (0)
#define dev_dbg(dev,fmt,...) do { if(!strcmp(fmt,"DCPAVSERV: legacy EDID service published\n")) legacy_publishes++; } while (0)
#define dev_info(...) do { diagnostic_information++; } while (0)

static unsigned diagnostic_errors,diagnostic_information,legacy_publishes;
#define DCP_FIRMWARE_H17P 17
/* The T6030 14.7 backend never matches these H17P scenarios. */
#define DCP_FIRMWARE_V_14_7 (DCP_FIRMWARE_H17P + 1)
#define DCP_FIRMWARE_V_14_7_TEMPLATE (DCP_FIRMWARE_H17P + 2)
#define EPIC_TYPE_NOTIFY 0
#define EPIC_CAT_REPORT 0
#define max(a,b) ((a)>(b)?(a):(b))
#define strscpy(dst,src,n) snprintf(dst,n,"%s",(char *)(src))
#define trace_dcpavserv_init(...) ((void)0)
#define dma_rmb() ((void)0)
#define DRM_CONNECTOR_REGISTERED 1
#define DPAVSERV_ENDPOINT_V2 0x2a
#define DCP_IOMFB_METHODS_H17P_EXTERNAL 2
#define EPIC_SUBTYPE_STD_SERVICE 0xc0
#define EPIC_SUBTYPE_ANNOUNCE_H17P 0x11
#define to_platform_device(x) (x)
#define ERR_PTR(x) ((void *)(intptr_t)(x))
#define IS_ERR(x) ((uintptr_t)(x)>(uintptr_t)-4096)
#define lockdep_assert_held(p) assert(*(p))
static bool *hpd_lock,*av_lock;
static void mutex_lock(bool *p) { if(p==hpd_lock) assert(!*av_lock);assert(!*p);*p=true; }
static void mutex_unlock(bool *p) { assert(*p);*p=false; }
static void complete(int *p) { (*p)++; }
static void reinit_completion(int *p) { *p=0; }
static void schedule_work(unsigned int *p) { (*p)++; }
struct work_struct { bool pending; };
static bool queue_work(void *wq,struct work_struct *w) { assert(wq);bool fresh=!w->pending;w->pending=true;return fresh; }
/* Only the 14.7 external role queues the AV client close; these providers never do. */
static bool __attribute__((unused)) cancel_work_sync(struct work_struct *w) { bool queued=w->pending;assert(!queued);return queued; }
struct apple_dcp;struct apple_dcp_afkep;struct apple_epic_service;
void afk_service_retry(struct apple_dcp_afkep *ep);
struct apple_epic_service_ops { char name[32];bool reusable,optional;void (*init)(struct apple_epic_service *,const char *,const char *,s64);int (*prepare)(struct apple_epic_service *,const char *,const char *,s64);int (*revalidate)(struct apple_epic_service *,const char *,const char *,s64);void (*teardown)(struct apple_epic_service *); };
struct epic_cmd_info { u16 tag;u8 type;size_t reply_length,reply_len;bool reply_len_valid;void *rxbuf,*txbuf;dma_addr_t rxbuf_dma,txbuf_dma;size_t rxlen,txlen;u32 retcode;bool done,free_on_ack;int *completion; };
struct apple_epic_service {
    struct apple_dcp_afkep *ep;const struct apple_epic_service_ops *ops;
    bool lock,enabled,torndown;unsigned int users;u32 channel;u8 cmd_tag;
    unsigned long cmd_map[1];struct epic_cmd_info cmds[MAX_PENDING_CMDS];
    struct { u64 generation,epoch; } admission;void *cookie;
    bool pending_open;const char *pending_name,*pending_class;s64 pending_unit;
};
struct apple_dcp_afkep { u8 *command_tags;bool lock,stopping;void *wq;struct work_struct service_work;struct apple_dcp *dcp;const struct apple_epic_service_ops *ops;bool wire_v2,inline_v2,match_epic_name;u32 endpoint;unsigned int num_channels;struct apple_epic_service services[AFK_MAX_CHANNEL]; };
struct apple_connector { void *dcp;bool connected,edid_invalidation_pending;struct { int registration_state; } base;unsigned int hotplug_wq; };
struct apple_dcp {
    void *dev;int fw_compat;struct apple_dcp_afkep *dcpavservep;struct apple_connector *connector;
    bool external_physically_absent,hpd_mutex,quiescing,typec_cable_connected;u64 typec_generation;unsigned int nr_modes;
    struct { int iomfb_method_profile; bool fw_14_7_endpoints; } hw; bool ext_dead,external_native;
    struct { bool lock,enabled;u64 epoch;int enable_completion;struct apple_epic_service *service,*opened;
             u64 opened_epoch,close_gen,terminated_epoch,close_epoch;bool opening;struct work_struct close_wq; } dcpavserv;
};
'''+internal_inline(root,'dcp_fw_is_14_7')+internal_inline(root,'dcp_is_v14_7_external')+r'''struct epic_cmd { u32 retcode;u64 rxbuf,txbuf;u32 rxlen,txlen; };
static unsigned int dma_frees,opens;

static int open_error,open_action;
void dpavservep_detach(struct apple_dcp *dcp);
static void afk_recv_handle_teardown(struct apple_dcp_afkep *ep, u32 channel);
static void inject_owner(struct apple_dcp_afkep *ep,u32 pending_channel);
static void dma_free_coherent(void *dev,size_t size,void *p,dma_addr_t address)
{ (void)dev;assert(size && (uintptr_t)p==address);free(p);dma_frees++; }
static void afk_remove_service_debugfs(struct apple_epic_service *s) { (void)s; }
static void afk_populate_service_debugfs(struct apple_epic_service *s) { (void)s; }
static int afk_send_epic(struct apple_dcp_afkep *e,u32 channel,u16 tag,int type,int category,u8 subtype,const void *p,size_t n)
{
    assert(channel && !tag && type==EPIC_TYPE_NOTIFY && category==EPIC_CAT_REPORT && subtype==AFKV2_OPEN && !p && !n);
    opens++;
    assert(!e->dcp->dcpavserv.enabled && !e->dcp->dcpavserv.service);
    for(unsigned int i=0;i<e->num_channels;i++) {
        struct apple_epic_service *s=&e->services[i];
        if(s->channel==channel && !s->torndown) assert(!s->enabled && !s->users);
    }
    if(open_action==1) e->dcp->typec_generation++;
    if(open_action==2) dpavservep_detach(e->dcp);
    if(open_action==4) inject_owner(e,channel);
    if(open_action==5) e->dcp->typec_cable_connected=false;
    if(open_action==6) e->dcp->quiescing=true;
    if(open_action==7) e->dcp->external_physically_absent=true;
    if(open_action==3) {
        for(unsigned int i=0;i<e->num_channels;i++)
            if(e->services[i].channel==channel && !e->services[i].torndown) e->services[i].channel++;
    }
    return open_error;
}
static void afk_v2_callback(struct apple_dcp_afkep *e,const struct afkv2_frame *f,const struct afkv2_command *c) { (void)e;(void)f;(void)c;assert(!"unexpected callback"); }

struct item { const char *key,*value; s64 integer; };
struct dcp_parse_ctx { struct item *items;unsigned int count,index;bool key; };
struct iterator { struct dcp_parse_ctx *handle;unsigned int idx,len; };
#define kfree(p) free((void *)(p))
#define PTR_ERR(p) ((int)(intptr_t)(p))
static struct item announcement_items[3];
static int parse(u8 *p,size_t n,struct dcp_parse_ctx *ctx)
{ (void)p;assert(n==37);*ctx=(struct dcp_parse_ctx){.items=announcement_items,.count=3};return 0; }
static int iterator_begin(struct dcp_parse_ctx *h,struct iterator *it,bool dict)
{ assert(dict);h->index=0;h->key=true;*it=(struct iterator){.handle=h,.len=h->count};return 0; }
static char *parse_string(struct dcp_parse_ctx *h)
{
    struct item *i=&h->items[h->index];const char *p;
    if(h->key) { p=i->key;h->key=false; } else { p=i->value;h->index++;h->key=true; }
    if(!p)
        return ERR_PTR(-EINVAL);
    char *copy=malloc(strlen(p)+1);assert(copy);strcpy(copy,p);return copy;
}
static int parse_int(struct dcp_parse_ctx *h,s64 *out)
{ assert(!h->key);*out=h->items[h->index++].integer;h->key=true;return 0; }
static int skip(struct dcp_parse_ctx *h) { assert(!h->key);h->index++;h->key=true;return 0; }
'''
    parts=[function('afk.c','static u16 afk_command_tag('),function('afk.c','void afk_service_retry('),function('parser.c','int parse_epic_service_init('),function('afk.h','static inline struct apple_epic_service *afk_service_get('),
           function('afk.h','static inline void afk_service_put('),function('afk.h','static inline void afk_service_retire('),function('afk.h','static inline void afk_service_disable('),
           function('afk.h','static inline bool afk_service_matches('),function('afk.h','static inline bool afk_service_reclaimable('),
           function('afk.h','static inline bool\nafk_service_reinit('),function('afk.h','static inline void afk_service_discard('),
           function('afk.h','static inline void afk_service_publish('),
           function('epic/dpavservep.c','static void dcpavserv_notify_edid('),function('epic/dpavservep.c','static bool dcpavserv_identity('),
           function('epic/dpavservep.c','static bool dcpavserv_admission_current('),function('epic/dpavservep.c','static int dcpavserv_prepare('),
           function('epic/dpavservep.c','static int dcpavserv_revalidate('),function('epic/dpavservep.c','static void dcpavserv_init('),function('epic/dpavservep.c','static void dcpavserv_invalidate_edid('),
           function('epic/dpavservep.c','static void dcpavserv_teardown('),function('epic/dpavservep.c','void dpavservep_detach('),
           function('afk.c','static const struct apple_epic_service_ops *\nafk_match_service('),
           function('afk.c','static struct apple_epic_service *afk_epic_find_service('),
           function('afk.h','static inline bool afk_service_retired('),function('afk.c','static bool afk_epic_channel_retired('),
           function('afk.c','static void afk_discard_pending_open('),function('afk.c','static void afk_retry_service_init('),function('afk.c','static void afk_service_worker('),function('afk.c','static void afk_recv_handle_init('),
           function('epic/dpavservep.c','static int dcpdpserv_prepare('),
           function('epic/dpavservep.c','static void dcpdpserv_teardown('),function('afk.c','static void afk_recv_handle_teardown('),
           function('afk.c','static void afk_recv_handle_reply('),function('afk.c','static void afk_v2_receive(')]
    suffix=r'''
static struct apple_dcp d;
static struct apple_dcp_afkep ep;
static struct apple_connector con;
static void run_retries(void) { assert(!ep.lock);if(ep.service_work.pending) { ep.service_work.pending=false;afk_service_worker(&ep.service_work); } }
static const struct apple_epic_service_ops ops={.name="dcpav-service-epic",.reusable=true,.prepare=dcpavserv_prepare,.revalidate=dcpavserv_revalidate,.init=dcpavserv_init,.teardown=dcpavserv_teardown};
static const struct apple_epic_service_ops announcement_ops[]={
    {.name="dcpav-service-epic",.reusable=true,.prepare=dcpavserv_prepare,.revalidate=dcpavserv_revalidate,.init=dcpavserv_init,.teardown=dcpavserv_teardown},
    {.name="dcpdp-service-epic",.reusable=true,.optional=true,.prepare=dcpdpserv_prepare,.teardown=dcpdpserv_teardown},
    {.name=""}
};
static const char *name="dcpav-service-epic",*class="AppleDCPDP2HDMI";
static const char *native_class="DCPDP13Service";
static struct apple_epic_service *prepare(u32 channel,s64 unit,const char *n,const char *c)
{
    for(unsigned int i=0;i<AFK_MAX_CHANNEL;i++) if(afk_service_reinit(&ep.services[i],&ep,&ops,channel)) {
        struct apple_epic_service *s=&ep.services[i];
        if(dcpavserv_prepare(s,n,c,unit)) { afk_service_discard(s);return NULL; }
        return s;
    }
    assert(!"no reusable slot");return NULL;
}
static void publish(struct apple_epic_service *s,s64 unit)
{ afk_service_publish(s);dcpavserv_init(s,name,class,unit); }
static void disconnect(void)
{
    mutex_lock(&d.hpd_mutex);d.typec_generation++;d.typec_cable_connected=false;mutex_unlock(&d.hpd_mutex);
    dpavservep_detach(&d);
}
static void connect(void)
{ mutex_lock(&d.hpd_mutex);d.typec_generation++;d.typec_cable_connected=true;mutex_unlock(&d.hpd_mutex); }
static void closed(u16 interface,u16 flags,u8 result)
{
    u8 packet[25];size_t written;
    assert(!afkv2_encode_frame(packet,sizeof(packet),1,interface,0,AFKV2_CLOSED,AFKV2_REPORT,flags,&result,1,&written));
    afk_v2_receive(&ep,0,0,packet,written);
}
static void announce(u16 channel,const char *compartment,const char *provider,s64 unit)
{
    u8 payload[69]={0},packet[93];size_t written;
    memcpy(payload,"synthetic",10);
    announcement_items[0]=(struct item){"compartment",compartment,0};
    announcement_items[1]=(struct item){"name",provider,0};
    announcement_items[2]=(struct item){"interface-id",NULL,unit};
    assert(!afkv2_encode_frame(packet,sizeof(packet),1,channel,0,EPIC_SUBTYPE_ANNOUNCE_H17P,AFKV2_REPORT,0,payload,sizeof(payload),&written));
    afk_v2_receive(&ep,0,0,packet,written);
}
static struct apple_epic_service *injected_owner;
static void inject_owner(struct apple_dcp_afkep *e,u32 pending_channel)
{
    assert(e==&ep);
    injected_owner=prepare(pending_channel+2,pending_channel+2,name,class);assert(injected_owner);
    publish(injected_owner,pending_channel+2);
    assert(d.dcpavserv.service==injected_owner && injected_owner->users==1);
}
static void repeated_announcements(void)
{
    ep.ops=announcement_ops;ep.match_epic_name=true;ep.num_channels=0;d.fw_compat=DCP_FIRMWARE_H17P;
    /* More than the entire slot budget; wrap channel identity as well. */
    for(unsigned int i=0;i<4*AFK_MAX_CHANNEL;i++) {
        u16 channel=1+4*(i%4);unsigned int before=opens;
        /* Flip, same-attachment revalidation, return flip; swap both providers. */
        bool native=((i%4==1 || i%4==2) != !!((i/4)%2));
        const char *provider=native?native_class:class;
        announce(channel,name,provider,channel);
        struct apple_epic_service *owner=d.dcpavserv.service;
        if(native) assert(owner && "native DP EDID provider must publish");
        assert(owner->ops==&announcement_ops[0]);
        assert(owner && owner->channel==channel && owner->users==1 && opens==before+1);
        assert(ep.num_channels<=2); /* Real allocator reuses, no slot exhaustion. */
        u64 epoch=d.dcpavserv.epoch;
        announce(channel,name,provider,channel); /* Active duplicate: no extra OPEN/pin. */
        assert(opens==before+1 && owner->users==1 && d.dcpavserv.epoch==epoch);
        announce(channel+2,"dcpdp-service-epic",provider,channel+2);
        assert(opens==before+1 && d.dcpavserv.service==owner && d.dcpavserv.epoch==epoch);
        if(i%4==1) {
            u64 generation=d.typec_generation;
            closed(channel,0x6a01,0); /* Revalidation retires service, not attachment. */
            assert(!d.dcpavserv.service && d.typec_generation==generation && d.dcpavserv.epoch>epoch);
        } else {
            disconnect();connect();
        }
    }
    unsigned int missing_ops_before=opens;
    assert(afk_match_service(&ep,"missing-service-epic")==NULL);
    assert(afk_match_service(&ep,"dcpav-service-epic")==&announcement_ops[0]);
    announce(13,"missing-service-epic",native_class,13);
    announce(13,"dcpav-service-epic-extra",native_class,13);
    assert(opens==missing_ops_before && !d.dcpavserv.service);
    assert(ep.num_channels==2); /* Auxiliary rejection leaves a reusable slot. */
    unsigned int before=opens;
    announce(13,name,"synthetic-wrong-class",13);assert(opens==before && !d.dcpavserv.service);
    char long_class[161];memset(long_class,'x',160);long_class[160]=0;
    announce(13,name,long_class,13);assert(opens==before && !d.dcpavserv.service);
    for(unsigned int variant=0;variant<2;variant++) {
        const char *provider=variant?native_class:class;
        assert(!prepare(0,0,name,provider));assert(!prepare(65536,65536,name,provider));
        assert(!prepare(13,-1,name,provider));assert(!prepare(13,0,name,provider));
        assert(!prepare(13,1,name,provider));assert(!prepare(13,13,NULL,provider));
        assert(!prepare(13,13,"dcpdp-service-epic",provider));
        announce(13,"wrong-compartment",provider,13);assert(opens==before && !d.dcpavserv.service);
        ep.endpoint=0x2c;announce(13,name,provider,13);assert(opens==before && !d.dcpavserv.service);ep.endpoint=0x2a;
        d.quiescing=true;announce(13,name,provider,13);assert(opens==before && !d.dcpavserv.service);d.quiescing=false;
        disconnect();announce(13,name,provider,13);assert(opens==before && !d.dcpavserv.service);connect();
    }
    for(unsigned int variant=0;variant<4;variant++) {
        const char *forged[]={"DCPDP13ServiceExtra","dcpdp13service","AppleDCPDP2HDMIExtra","unrelated-provider"};
        announce(13,name,forged[variant],13);assert(opens==before && !d.dcpavserv.service);
    }
    assert(!prepare(13,13,name,NULL));
    open_error=-EIO;announce(13,name,native_class,13);assert(opens==++before && !d.dcpavserv.service);open_error=0;
    for(unsigned int window=0;window<5;window++) {
        const int actions[]={1,2,3,5,6};open_action=actions[window];
        announce(13,name,native_class,13);
        assert(opens==++before && !d.dcpavserv.service && !d.dcpavserv.enabled);
        d.typec_cable_connected=true;d.quiescing=false;
    }
    open_action=4;announce(13,name,native_class,13);assert(opens==++before && d.dcpavserv.service==injected_owner);
    assert(injected_owner->users==1);dpavservep_detach(&d);assert(!injected_owner->users);
    open_action=0;announce(13,name,native_class,13);assert(opens==++before && d.dcpavserv.service);
    struct apple_epic_service *old=d.dcpavserv.service;
    assert(afk_service_get(old)==old);ep.command_tags[13]=1;old->cmd_map[0]=1;
    disconnect();connect();
    announce(13,name,native_class,13); /* Same channel still owns a pending reply. */
    assert(opens==before && !d.dcpavserv.service && old->users==1 && old->channel==13);
    /* The firmware need not announce again after the old reply drains. */
    old->cmds[0]=(struct epic_cmd_info){.tag=0,.type=0xc0,.rxlen=32,.txlen=32,.free_on_ack=true};
    old->cmds[0].rxbuf=calloc(1,32);old->cmds[0].txbuf=calloc(1,32);
    old->cmds[0].rxbuf_dma=(uintptr_t)old->cmds[0].rxbuf;
    old->cmds[0].txbuf_dma=(uintptr_t)old->cmds[0].txbuf;
    u8 drain_oob[32],drain_reply[56];size_t drain_written;
    assert(!afkv2_encode_oob_command(drain_oob,sizeof(drain_oob),0,old->cmds[0].rxbuf_dma,32,old->cmds[0].txbuf_dma,32));
    afkv2_put32(drain_oob+4,0);
    assert(!afkv2_encode_frame(drain_reply,sizeof(drain_reply),1,13,0,0xc0,AFKV2_RESPONSE,0,drain_oob,sizeof(drain_oob),&drain_written));
    afk_v2_receive(&ep,0,0,drain_reply,drain_written);
    run_retries();
    assert(d.dcpavserv.service && "a blocked reannouncement must recover after the old ACK without another announcement");
    assert(d.dcpavserv.service!=old && opens==++before && old->users==1 && !old->cmd_map[0]);
    struct apple_epic_service *resumed=d.dcpavserv.service;
    resumed->cmd_map[0]=1;
    /* The DMA allocator can reuse both old addresses for the new command. */
    resumed->lock=true;
    resumed->cmds[0]=(struct epic_cmd_info){.tag=afk_command_tag(resumed,0),.type=0xc0,
        .rxbuf_dma=old->cmds[0].rxbuf_dma,.txbuf_dma=old->cmds[0].txbuf_dma,.rxlen=32,.txlen=32};
    resumed->lock=false;
    afk_v2_receive(&ep,0,0,drain_reply,drain_written);
    assert(!resumed->cmds[0].done && resumed->cmd_map[0] && "old ACK must not complete the next owner's command");
    resumed->cmd_map[0]=0;
    dpavservep_detach(&d);
    announce(1,name,class,1);assert(opens==++before && d.dcpavserv.service!=old);
    struct apple_epic_service *fresh=d.dcpavserv.service;u64 epoch=d.dcpavserv.epoch;
    dcpavserv_teardown(old);assert(d.dcpavserv.service==fresh && d.dcpavserv.epoch==epoch);
    old->cmd_map[0]=0;afk_service_put(old);disconnect();connect();
    run_retries();
    announce(13,name,native_class,13);assert(opens==++before && d.dcpavserv.service);
    struct apple_epic_service *owner=d.dcpavserv.service;
    u64 owner_epoch=d.dcpavserv.epoch;
    announce(5,name,class,5); /* A second admitted identity cannot displace the owner. */
    assert(opens==before && d.dcpavserv.service==owner && d.dcpavserv.epoch==owner_epoch);
    dpavservep_detach(&d);
    /* All slots held by retired readers: fail safely, then recover after puts. */
    for(unsigned int i=0;i<AFK_MAX_CHANNEL;i++) {
        struct apple_epic_service *s=&ep.services[i];
        assert(afk_service_reinit(s,&ep,&ops,100+i));afk_service_publish(s);
        assert(afk_service_get(s)==s);afk_service_retire(s);
    }
    announce(13,name,native_class,13);assert(opens==before && !d.dcpavserv.service);
    for(unsigned int i=0;i<AFK_MAX_CHANNEL;i++) afk_service_put(&ep.services[i]);
    announce(13,name,native_class,13);assert(opens==++before && d.dcpavserv.service);
    dpavservep_detach(&d);
    assert(!d.hpd_mutex && !d.dcpavserv.lock);
    /* Legacy remains unit-zero and class-agnostic, with its original epoch fence. */
    ep.wire_v2=false;ep.endpoint=0x28;d.external_physically_absent=true;
    assert(!prepare(13,13,name,native_class));
    struct apple_epic_service *legacy=prepare(13,0,name,native_class);assert(legacy);
    dpavservep_detach(&d);afk_service_publish(legacy);dcpavserv_init(legacy,name,native_class,0);
    assert(!d.dcpavserv.service && legacy->torndown);
    legacy=prepare(13,0,name,native_class);assert(legacy);
    legacy_publishes=0;d.typec_generation++;afk_service_publish(legacy);dcpavserv_init(legacy,name,native_class,0);
    assert(legacy_publishes==1);
    assert(d.dcpavserv.service==legacy && legacy->users==1);dpavservep_detach(&d);
    assert(dcpdpserv_prepare(legacy,"dcpdp-service-epic",native_class,0)==0);
    ep.wire_v2=true;ep.endpoint=0x2a;d.external_physically_absent=false;
    announce(U16_MAX,name,native_class,U16_MAX);assert(d.dcpavserv.service && d.dcpavserv.service->channel==U16_MAX);
    dpavservep_detach(&d);
    puts("PASS: actual AFKV2 HDMI/native DP alternating over 64 cycles: flip/revalidation/return, strict identity, stale OPEN generation/epoch/owner, auxiliary isolation, reader/command pins and legacy controls");
}

static void deferred_announcements(void)
{
    for(unsigned int cycle=0;cycle<64;cycle++) {
        unsigned int event=cycle%10,before=opens,freed=dma_frees;
        u16 channel=1+4*(cycle%4);
        const char *provider=(cycle%2)?native_class:class;
        announce(channel,name,provider,channel);
        struct apple_epic_service *old=d.dcpavserv.service;
        assert(old && opens==++before && afk_service_get(old)==old);
        old->cmd_map[0]=1;
        old->cmds[0]=(struct epic_cmd_info){.tag=0,.type=0xc0,.rxlen=32,.txlen=32,.free_on_ack=true};
        old->cmds[0].rxbuf=calloc(1,32);old->cmds[0].txbuf=calloc(1,32);
        old->cmds[0].rxbuf_dma=(uintptr_t)old->cmds[0].rxbuf;
        old->cmds[0].txbuf_dma=(uintptr_t)old->cmds[0].txbuf;
        u8 oob[32],reply[56];size_t written;
        assert(!afkv2_encode_oob_command(oob,sizeof(oob),0,old->cmds[0].rxbuf_dma,32,old->cmds[0].txbuf_dma,32));
        afkv2_put32(oob+4,0);
        assert(!afkv2_encode_frame(reply,sizeof(reply),1,channel,0,0xc0,AFKV2_RESPONSE,0,oob,sizeof(oob),&written));
        u64 generation=d.typec_generation;
        closed(channel,0x6a01,0);
        assert(!d.dcpavserv.service && old->users==1 && old->torndown && d.typec_generation==generation);
        u64 epoch=d.dcpavserv.epoch;
        announce(channel,name,provider,channel);
        run_retries();
        assert(opens==before && !d.dcpavserv.service && dma_frees==freed);
        for(unsigned int i=0;i<ep.num_channels;i++) {
            struct apple_epic_service *pending=&ep.services[i];
            if(!pending->pending_open) continue;
            assert(dcpavserv_revalidate(pending,pending->pending_name,pending->pending_class,pending->pending_unit)==0);
            d.dcpavserv.enabled=true;
            assert(dcpavserv_revalidate(pending,pending->pending_name,pending->pending_class,pending->pending_unit)==-EBUSY);
            d.dcpavserv.enabled=false;
        }
        if(event==1) d.typec_generation++;
        if(event==2) dpavservep_detach(&d);
        if(event==3) d.typec_cable_connected=false;
        if(event==4) d.quiescing=true;
        struct apple_epic_service *other=NULL;
        if(event==5) { other=prepare(channel+2,channel+2,name,class);publish(other,channel+2); }
        if(event==6) assert(!ep.stopping);
        if(event==7) open_error=-EIO;
        if(event==8) announce(channel,name,provider,channel);
        if(event==9) { d.typec_generation++;announce(channel,name,provider,channel); }
        afk_v2_receive(&ep,0,0,reply,written);
        if(event==6) { assert(ep.service_work.pending);ep.stopping=true; }
        run_retries();
        assert(dma_frees==freed+2 && old->users==1 && !old->cmd_map[0]);
        if(event==0 || event==8 || event==9) {
            assert(d.dcpavserv.service && d.dcpavserv.service!=old);
            assert(d.dcpavserv.service->channel==channel && opens==++before);
            assert(d.dcpavserv.epoch>epoch && d.dcpavserv.service->users==1);
        } else if(event==5) {
            assert(d.dcpavserv.service==other && opens==before);
        } else {
            assert(!d.dcpavserv.service && opens==before+(event==7));
        }
        afk_v2_receive(&ep,0,0,reply,written); /* Duplicate late ACK cannot touch the new owner. */
        assert(dma_frees==freed+2);
        afk_service_put(old);run_retries();
        assert(!old->users);
        if(event==6) {
            for(unsigned int i=0;i<ep.num_channels;i++)
                if(ep.services[i].pending_open) afk_discard_pending_open(&ep.services[i]);
        }
        ep.stopping=false;open_error=0;d.quiescing=false;d.typec_cable_connected=true;
        dpavservep_detach(&d);run_retries();
        for(unsigned int i=0;i<ep.num_channels;i++) assert(!ep.services[i].pending_open);
    }
    puts("PASS: 64 actual CLOSED/reannounce/late-ACK cycles with deferred OPEN, unchanged attachment, duplicate announcement/reply, stale generation/epoch, cable/quiesce/owner/stop and OPEN-error controls");
}
static int injected_prepare_error;
static int failed_prepare(struct apple_epic_service *s,const char *n,const char *c,s64 u)
{ (void)s;(void)n;(void)c;(void)u;return injected_prepare_error; }
static void optional_admission_controls(void)
{
    ep.ops=announcement_ops;ep.match_epic_name=true;ep.num_channels=0;d.fw_compat=DCP_FIRMWARE_H17P;
    diagnostic_errors=diagnostic_information=0;
    unsigned before=opens;
    announce(3,"dcpdp-service-epic",class,3);
    announce(5,"dcpdp-service-epic",native_class,5);
    assert(!diagnostic_errors && diagnostic_information==2 && opens==before);
    assert(!d.dcpavserv.enabled && !d.dcpavserv.service);
    for(unsigned i=0;i<ep.num_channels;i++)
        assert(!ep.services[i].enabled && !ep.services[i].users && !ep.services[i].pending_open);
    struct apple_epic_service_ops faulty[]={announcement_ops[1],{.name=""}};
    faulty[0].prepare=failed_prepare;ep.ops=faulty;
    const int faults[]={-EIO,-ENODEV};
    for(unsigned i=0;i<2;i++) {
        injected_prepare_error=faults[i];diagnostic_errors=diagnostic_information=0;
        announce(7,"dcpdp-service-epic",class,7);
        assert(diagnostic_errors==1 && !diagnostic_information && opens==before);
    }
    faulty[0].optional=false;injected_prepare_error=-EOPNOTSUPP;
    diagnostic_errors=diagnostic_information=0;
    announce(9,"dcpdp-service-epic",class,9);
    assert(diagnostic_errors==1 && !diagnostic_information && opens==before);
    memset(ep.services,0,sizeof(ep.services));ep.num_channels=AFK_MAX_CHANNEL;ep.ops=announcement_ops;
    puts("PASS optional absence remains closed; required unsupported and genuine optional failures remain errors");
}
int main(void)
{
    d.hw.iomfb_method_profile=2;d.typec_cable_connected=true;d.typec_generation=1;d.nr_modes=1;d.connector=&con;d.dev=&d;
    con.dcp=&d;con.connected=true;con.base.registration_state=1;
    ep.dcp=&d;ep.wire_v2=true;ep.endpoint=0x2a;ep.num_channels=AFK_MAX_CHANNEL;d.dcpavservep=&ep;
    hpd_lock=&d.hpd_mutex;av_lock=&d.dcpavserv.lock;
    optional_admission_controls();
    struct item h17p[]={{"compartment","dcpav-service-epic",0},{"name","AppleDCPDP2HDMI",0},{"interface-id",NULL,13}};
    struct dcp_parse_ctx parse_ctx={.items=h17p,.count=3};const char *parsed_name,*parsed_class;s64 parsed_unit=-1;
    assert(!parse_epic_service_init(&parse_ctx,&parsed_name,&parsed_class,&parsed_unit,true));
    assert(parsed_unit==13 && !strcmp(parsed_name,name) && !strcmp(parsed_class,class));
    struct apple_epic_service *proof=prepare(13,parsed_unit,parsed_name,parsed_class);assert(proof);afk_service_discard(proof);
    free((void *)parsed_name);free((void *)parsed_class);
    struct item legacy[]={{"EPICName","dcpav-service-epic",0},{"EPICProviderClass","legacy-class",0},{"EPICUnit",NULL,0}};
    parse_ctx=(struct dcp_parse_ctx){.items=legacy,.count=3};
    assert(!parse_epic_service_init(&parse_ctx,&parsed_name,&parsed_class,&parsed_unit,false));assert(parsed_unit==0);
    free((void *)parsed_name);free((void *)parsed_class);
    ep.wq=&ep;ep.command_tags=calloc(AFKV2_INTERFACES,1);assert(ep.command_tags);
    struct apple_epic_service *old=prepare(1,1,name,class);assert(old);publish(old,1);
    assert(d.dcpavserv.service==old && old->users==1);
    assert(afk_service_get(old)==old); /* Delayed EDID reader pins the retired slot. */
    old->cmds[0]=(struct epic_cmd_info){.tag=0,.type=0xc0,.rxlen=32,.txlen=32,.free_on_ack=true};
    old->cmds[0].rxbuf=calloc(1,32);old->cmds[0].txbuf=calloc(1,32);assert(old->cmds[0].rxbuf && old->cmds[0].txbuf);
    old->cmds[0].rxbuf_dma=(uintptr_t)old->cmds[0].rxbuf;old->cmds[0].txbuf_dma=(uintptr_t)old->cmds[0].txbuf;old->cmd_map[0]=1;
    disconnect();assert(!d.dcpavserv.service && !d.dcpavserv.enabled && old->users==1 && old->torndown && !old->enabled);
    assert(con.edid_invalidation_pending && !afk_service_get(old));
    assert(!afk_service_reinit(old,&ep,&ops,5));
    assert(!prepare(5,5,name,class)); /* No admission while physically disconnected. */
    connect();struct apple_epic_service *fresh=prepare(5,5,name,class);assert(fresh && fresh!=old);publish(fresh,5);
    assert(d.dcpavserv.service==fresh && fresh->users==1);
    u64 epoch=d.dcpavserv.epoch;
    dcpavserv_teardown(old);assert(d.dcpavserv.service==fresh && d.dcpavserv.epoch==epoch);
    closed(1,0x6a01,0);assert(d.dcpavserv.service==fresh && d.dcpavserv.epoch==epoch && !dma_frees);
    u8 oob[32],reply[56];size_t written;
    assert(!afkv2_encode_oob_command(oob,sizeof(oob),0,old->cmds[0].rxbuf_dma,32,old->cmds[0].txbuf_dma,32));afkv2_put32(oob+4,0);
    assert(!afkv2_encode_frame(reply,sizeof(reply),1,1,0,0xc0,AFKV2_RESPONSE,0,oob,sizeof(oob),&written));
    afk_v2_receive(&ep,0,0,reply,written);
    assert(dma_frees==2 && !old->cmd_map[0] && old->users==1 && d.dcpavserv.service==fresh && d.dcpavserv.epoch==epoch);
    assert(!afk_service_reinit(old,&ep,&ops,9));afk_service_put(old);
    afk_v2_receive(&ep,0,0,reply,written);assert(dma_frees==2); /* Duplicate late ACK. */
    disconnect();connect();struct apple_epic_service *next=prepare(9,9,name,class);assert(next);publish(next,9);
    assert(d.dcpavserv.service==next);
    closed(1,0x6a01,0);closed(5,0x6a01,0);assert(d.dcpavserv.service==next);
    disconnect();connect();next=prepare(13,13,name,class);assert(next);publish(next,13);assert(d.dcpavserv.service==next);
    closed(13,0x6a01,1);assert(d.dcpavserv.service==next); /* Nonzero result rejected. */
    closed(13,0x6a00,0);assert(d.dcpavserv.service==next); /* Wrong low flags rejected. */
    d.dcpavservep=NULL;closed(13,0x6a01,0);assert(d.dcpavserv.service==next);d.dcpavservep=&ep;
    closed(13,0x6a01,0);assert(!d.dcpavserv.service && !d.dcpavserv.enabled);
    u8 result_byte=0;struct afkv2_frame notification={.category=AFKV2_REPORT,.type=AFKV2_CLOSED,.interface_flags=1,.payload=&result_byte,.payload_size=1};
    assert(afkv2_closed_report(&notification));notification.token=1;assert(!afkv2_closed_report(&notification));
    notification.token=0;notification.payload_size=0;assert(!afkv2_closed_report(&notification));
    assert(!prepare(0,0,name,class));assert(!prepare(65536,65536,name,class));
    assert(!prepare(9,1,name,class));assert(!prepare(9,9,"wrong",class));assert(!prepare(9,9,name,"wrong"));
    struct apple_epic_service *pending=prepare(9,9,name,class);assert(pending);
    disconnect();connect();publish(pending,9);
    assert(!d.dcpavserv.service && pending->torndown && !pending->enabled && !pending->users);
    pending=prepare(9,9,name,class);assert(pending);
    mutex_lock(&d.hpd_mutex);d.typec_generation++;mutex_unlock(&d.hpd_mutex);
    publish(pending,9); /* Generation change alone, without registry retirement. */
    assert(!d.dcpavserv.service && pending->torndown && !pending->enabled);
    pending=prepare(13,13,name,class);assert(pending);
    dpavservep_detach(&d);publish(pending,13); /* Registry retirement with no physical change. */
    assert(!d.dcpavserv.service && pending->torndown && !pending->enabled);
    ep.wire_v2=false;ep.endpoint=0x28;d.hw.iomfb_method_profile=0;
    assert(!prepare(7,7,name,class));pending=prepare(7,0,name,NULL);assert(pending);publish(pending,0);
    assert(d.dcpavserv.service==pending && pending->users==1);dpavservep_detach(&d);
    assert(!pending->users && !d.hpd_mutex && !d.dcpavserv.lock);
    memset(&ep.services,0,sizeof(ep.services));ep.wire_v2=true;ep.endpoint=0x2a;d.hw.iomfb_method_profile=2;
    repeated_announcements();
    deferred_announcements();
    free(ep.command_tags);ep.command_tags=NULL;
    puts("PASS: actual dynamic 1/5/9/13 binding, forged identities, prepare/init epochs, physical retirement, reader pins, real old CLOSED/late ACK isolation and legacy unit 0");
}
'''
    return prefix + '\n'.join(parts) + suffix

def admission(root):

    base = root / 'drivers/gpu/drm/apple'

    def function(file, signature):
        source = (base / file).read_text()
        start = source.index(signature)
        opening = source.index('{', start)
        while ';' in source[start:opening]:
            start = source.index(signature, start + 1)
            opening = source.index('{', start)
        end, depth = opening + 1, 1
        while depth:
            depth += (source[end] == '{') - (source[end] == '}')
            end += 1
        return source[start:end]

    av = (base / 'epic/dpavservep.c').read_text()
    af = (base / 'afk.c').read_text()
    announce = af[af.index('static void afk_recv_handle_init('):af.index('static void afk_recv_handle_init(') + 6500]
    assert announce.index('ops->prepare(service,') < announce.index('EPIC_CAT_REPORT, AFKV2_OPEN')
    assert '.prepare = dcpavserv_prepare,' in av and '.prepare = dcpdpserv_prepare,' in av
    prefix = r'''
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint64_t u64;
typedef int64_t s64;
#define U16_MAX UINT16_MAX
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define ERR_PTR(x) ((void *)(intptr_t)(x))
#define PTR_ERR(x) ((int)(intptr_t)(x))
#define IS_ERR(x) ((uintptr_t)(x) > (uintptr_t)-4096)
#define spin_lock_irqsave(p,f) do { assert(!*(p)); *(p)=true; (f)=0; } while(0)
#define spin_unlock_irqrestore(p,f) do { (void)(f); assert(*(p)); *(p)=false; } while(0)
#define lockdep_assert_held(p) assert(*(p))
static bool *acquire(bool *p) { assert(!*p); *p=true; return p; }
static void release(bool **p) { if(*p) { assert(**p); **p=false; } }
static bool *once(bool *p) { assert(*p); *p=false; return NULL; }
#define scoped_guard(k,p) for(bool *_lock __attribute__((cleanup(release)))=acquire(p); _lock; _lock=once(_lock))
static void mutex_lock(bool *p) { (void)acquire(p); }
static void mutex_unlock(bool *p) { (void)once(p); }
#define dev_dbg(...) ((void)0)
#define dev_info(...) ((void)0)
#define dev_err(...) ((void)0)
#define dev_warn(...) ((void)0)
#define trace_dcpavserv_init(...) ((void)0)
#define DRM_CONNECTOR_REGISTERED 1
#define to_platform_device(x) (x)
static void reinit_completion(int *p) { *p=0; }
static void complete(int *p) { *p=1; }
static void schedule_work(int *p) { (*p)++; }
#define dev_dbg_ratelimited(...) ((void)0)
#define dev_warn_ratelimited(...) ((void)0)
#define trace_dcp_recv_msg(...) ((void)0)
#define DCP_IOMFB_METHODS_H17P_EXTERNAL 2
#define DCP_FIRMWARE_H17P 2
/* The 14.7 open-before-EDID handshake is for T6030 external processors only. */
#define DCP_FIRMWARE_V_14_7 (DCP_FIRMWARE_H17P + 1)
#define AV_ENDPOINT 0x29
#define SYSTEM_ENDPOINT 0x20
#define DISP0_ENDPOINT 0x23
#define DPAVSERV_ENDPOINT 0x28
#define DPAVSERV_ENDPOINT_V2 0x2a
#define DPTX_ENDPOINT 0x2a
#define DPTX_ENDPOINT_V2 0x2c
#define DPTX_CONTROLLER_ENDPOINT 0x24
#define IOMFB_ENDPOINT 0x37
struct apple_dcp;
struct apple_connector { void *dcp; struct { int registration_state; } base; bool connected, edid_invalidation_pending; int hotplug_wq; };
struct apple_dcp_afkep { struct apple_dcp *dcp; u8 endpoint; bool wire_v2; };
struct apple_epic_service { struct apple_dcp_afkep *ep; unsigned int users; bool enabled,torndown; struct { u64 generation,epoch; } admission; void *cookie; unsigned int channel; };
struct apple_dcp {
    struct apple_dcp_afkep *dcpavservep, *dptxep, *avep, *systemep, *ibootep, *dptx_controller_ep;
    struct { bool lock, enabled; u64 epoch; int enable_completion; struct apple_epic_service *service, *opened;
             u64 opened_epoch, close_gen, terminated_epoch; bool opening; } dcpavserv;
    struct apple_connector *connector; unsigned int nr_modes;
    struct { int iomfb_method_profile; bool fw_14_7_endpoints; } hw; bool ext_dead,external_native;
    bool external_physically_absent,hpd_mutex, typec_cable_connected, quiescing;
    u64 typec_generation;
    int fw_compat;
    void *dev;
};
struct drm_edid { int identity; };
'''+internal_inline(root,'dcp_is_v14_7_external')+r'''static unsigned int reads, allocations, legacy_reads, receives;
static unsigned int event;
static struct apple_dcp_afkep *received_ep;
static struct apple_epic_service *afk_service_get(struct apple_epic_service *s)
{ if(!s || !s->enabled || s->torndown) return NULL; s->users++; return s; }
static void afk_service_put(struct apple_epic_service *s)
{ assert(s->users); s->users--; }
static void afk_service_retire(struct apple_epic_service *s) { s->enabled=false;s->torndown=true; }
static void drm_edid_free(const struct drm_edid *e)
{ if(e) { assert(allocations); allocations--; free((void *)e); } }
static void dcpavserv_teardown(struct apple_epic_service *s);
static const struct drm_edid *dcpavserv_read_edid_inline(struct apple_epic_service *s)
{
    struct apple_dcp *d=s->ep->dcp;
    assert(!d->hpd_mutex && !d->dcpavserv.lock && s->users==2);
    reads++;
    if(event==1 || event==2) { d->typec_generation++; d->typec_cable_connected=(event==2); }
    if(event==3) { s->enabled=false; dcpavserv_teardown(s); }
    struct drm_edid *e=malloc(sizeof(*e)); assert(e); e->identity=42; allocations++; return e;
}
static const struct drm_edid *dcpavserv_read_edid_legacy(struct apple_epic_service *s)
{ legacy_reads++; return dcpavserv_read_edid_inline(s); }
static int __attribute__((unused)) afk_service_call(struct apple_epic_service *s,unsigned short group,unsigned command,const void *data,size_t data_len,size_t data_pad,void *output,size_t output_len,size_t output_pad)
{ (void)s;(void)group;(void)command;(void)data;(void)data_len;(void)data_pad;(void)output;(void)output_len;(void)output_pad;assert(!"14.7 open-before-EDID on H17P");return -ENODEV; }
/* Only the 14.7 external role opens the AV client and validates every EDID block. */
static int __attribute__((unused)) dcpavserv_v14_open(struct apple_dcp *d,struct apple_epic_service *s,u64 epoch,u64 gen)
{ (void)d;(void)s;(void)epoch;(void)gen;assert(!"14.7 AV client open on H17P");return -ENODEV; }
static bool __attribute__((unused)) drm_edid_valid(const struct drm_edid *e) { (void)e;assert(!"14.7 EDID validation on H17P");return false; }
static void iomfb_recv_msg(struct apple_dcp *d, u64 message) { (void)d; (void)message; }
static void afk_receive_message(struct apple_dcp_afkep *ep, u64 message)
{ (void)message; received_ep=ep; receives++; }
'''
    parts = [function('epic/dpavservep.c', 'static void dcpavserv_notify_edid('),
             function('epic/dpavservep.c', 'static void dcpavserv_invalidate_edid('),
             function('epic/dpavservep.c', 'static void dcpavserv_teardown('),
             function('dcp.c', 'static struct apple_dcp_afkep *dcp_afkep('),
             function('dcp.c', 'static void dcp_recv_msg('),
             function('epic/dpavservep.c', 'static bool dcpavserv_identity('),
             function('epic/dpavservep.c', 'static bool dcpavserv_admission_current('),
             function('epic/dpavservep.c', 'static int dcpavserv_prepare('),
             function('epic/dpavservep.c', 'static int dcpdpserv_prepare('),
             function('epic/dpavservep.c', 'static void dcpavserv_init('),
             function('epic/dpavservep.c', 'bool dcpavserv_edid_current('),
             function('epic/dpavservep.c', 'bool dcpavserv_edid_service_current('),
             function('epic/dpavservep.c', 'const struct drm_edid *dcpavserv_copy_edid(')]
    suffix = r'''
int main(void)
{
    u64 epoch=0;
    int device;
    struct apple_connector connector={.dcp=&device,.base.registration_state=DRM_CONNECTOR_REGISTERED,.connected=true};
    struct apple_dcp d={.fw_compat=DCP_FIRMWARE_H17P,.dev=&device,.connector=&connector,.nr_modes=1};
    struct apple_dcp_afkep av={.dcp=&d,.endpoint=0x28}, tx={.dcp=&d,.endpoint=0x2a};
    d.dcpavservep=&av; d.dptxep=&tx;
    assert(dcp_afkep(&d,0x28)==&av && dcp_afkep(&d,0x2a)==&tx && !dcp_afkep(&d,0x2c));
    dcp_recv_msg(&d,0x28,1); assert(received_ep==&av && receives==1);
    dcp_recv_msg(&d,0x2a,2); assert(received_ep==&tx && receives==2);
    d.hw.iomfb_method_profile=DCP_IOMFB_METHODS_H17P_EXTERNAL;
    av.endpoint=0x2a; av.wire_v2=true; tx.endpoint=0x2c;
    assert(!dcp_afkep(&d,0x28) && dcp_afkep(&d,0x2a)==&av && dcp_afkep(&d,0x2c)==&tx);
    dcp_recv_msg(&d,0x28,3); assert(receives==2);
    dcp_recv_msg(&d,0x2a,4); assert(received_ep==&av && receives==3);
    dcp_recv_msg(&d,0x2c,5); assert(received_ep==&tx && receives==4);
    d.dcpavservep=NULL; dcp_recv_msg(&d,0x2a,6); assert(receives==4); d.dcpavservep=&av;
    struct apple_epic_service s={.ep=&av,.users=1,.enabled=true,.channel=1};
    const char *name="dcpav-service-epic", *class="AppleDCPDP2HDMI";
    assert(!dcpavserv_prepare(&s,name,class,1));
    assert(dcpavserv_prepare(&s,name,class,0)==-ENODEV);
    assert(dcpavserv_prepare(&s,name,class,3)==-ENODEV);
    assert(dcpavserv_prepare(&s,name,class,-1)==-ENODEV);
    assert(dcpavserv_prepare(&s,"dcpdp-service-epic",class,1)==-ENODEV);
    assert(dcpavserv_prepare(&s,name,"wrong",1)==-ENODEV);
    assert(dcpavserv_prepare(&s,NULL,class,1)==-ENODEV);
    assert(dcpavserv_prepare(&s,name,NULL,1)==-ENODEV);
    av.endpoint=0x28; assert(dcpavserv_prepare(&s,name,class,1)==-ENODEV); av.endpoint=0x2a;
    assert(dcpdpserv_prepare(&s,"dcpdp-service-epic",class,3)==-EOPNOTSUPP);
    d.dcpavserv.enabled=true; assert(dcpavserv_prepare(&s,name,class,1)==-EBUSY);
    d.dcpavserv.service=&s; d.dcpavserv.epoch=1; d.typec_generation=1; d.typec_cable_connected=true;
    const struct drm_edid *e=dcpavserv_copy_edid(&d,1,&epoch);
    assert(!IS_ERR(e) && s.users==1 && reads==1); drm_edid_free(e);
    d.typec_cable_connected=false;
    assert(PTR_ERR(dcpavserv_copy_edid(&d,1,&epoch))==-ESTALE && reads==1);
    d.typec_cable_connected=true;
    assert(PTR_ERR(dcpavserv_copy_edid(&d,0,&epoch))==-ESTALE && reads==1);
    event=1; assert(PTR_ERR(dcpavserv_copy_edid(&d,1,&epoch))==-ESTALE && !allocations && s.users==1);
    event=2; d.typec_cable_connected=true;
    assert(PTR_ERR(dcpavserv_copy_edid(&d,2,&epoch))==-ESTALE && !allocations && s.users==1);
    event=0; e=dcpavserv_copy_edid(&d,3,&epoch); assert(!IS_ERR(e)); drm_edid_free(e);
    event=3; assert(PTR_ERR(dcpavserv_copy_edid(&d,3,&epoch))==-ENODEV && !allocations && !s.users);
    assert(PTR_ERR(dcpavserv_copy_edid(&d,3,&epoch))==-ENODEV);
    assert(connector.edid_invalidation_pending && connector.hotplug_wq && d.dcpavserv.epoch==2);
    /* Same slot and physical generation: CLOSE/re-OPEN gets a fresh ticket. */
    s.enabled=true; assert(!dcpavserv_prepare(&s,name,class,1)); dcpavserv_init(&s,name,class,1);
    assert(d.dcpavserv.service==&s && s.users==1 && d.dcpavserv.epoch==3);
    mutex_lock(&d.dcpavserv.lock);
    assert(!dcpavserv_edid_service_current(&d,1) && dcpavserv_edid_service_current(&d,3));
    mutex_unlock(&d.dcpavserv.lock);
    event=0; e=dcpavserv_copy_edid(&d,3,&epoch); assert(!IS_ERR(e) && epoch==3);
    /* Simulate retirement in the final validation-to-publication gap. */
    s.enabled=false; dcpavserv_teardown(&s); s.enabled=true; assert(!dcpavserv_prepare(&s,name,class,1)); dcpavserv_init(&s,name,class,1);
    mutex_lock(&d.dcpavserv.lock); assert(!dcpavserv_edid_service_current(&d,epoch)); mutex_unlock(&d.dcpavserv.lock);
    drm_edid_free(e); assert(d.typec_generation==3 && d.dcpavserv.epoch==5 && s.users==1);
    s.enabled=false; dcpavserv_teardown(&s);
    av.wire_v2=false; av.endpoint=0x28; d.hw.iomfb_method_profile=0;
    d.dcpavserv.enabled=false; assert(!dcpavserv_prepare(&s,name,NULL,0));
    assert(dcpavserv_prepare(&s,name,class,1)==-ENODEV);
    assert(!dcpdpserv_prepare(&s,NULL,NULL,3));
    d.dcpavserv.service=&s; d.dcpavserv.enabled=true; s.enabled=true; s.users=1; event=0;
    d.typec_cable_connected=false; e=dcpavserv_copy_edid(&d,0,&epoch);
    assert(!IS_ERR(e) && legacy_reads==1 && s.users==1); drm_edid_free(e);
    d.hw.iomfb_method_profile=0;av.wire_v2=false;d.external_physically_absent=true;
    mutex_lock(&d.hpd_mutex);assert(dcpavserv_edid_current(&d,0));mutex_unlock(&d.hpd_mutex);
    assert(!allocations && !d.hpd_mutex && !d.dcpavserv.lock);
    puts("PASS: production old/new lookup and receive, exact service admission before OPEN, delayed disconnect/replug, service detach pins, legacy transport");
}
'''
    return prefix + '\n'.join(parts) + suffix

def rx_progress(root):

    def source(path):
        return (root / path).read_text()

    def function(text, signature):
        start = text.index(signature)
        opening = text.index('{', start)
        while ';' in text[start:opening]:
            start = text.index(signature, start+1)
            opening = text.index('{', start)
        end, depth = opening+1, 1
        while depth:
            depth += (text[end]=='{')-(text[end]=='}')
            end += 1
        return text[start:end]

    dcp=source('drivers/gpu/drm/apple/dcp.c')
    iomfb=source('drivers/gpu/drm/apple/iomfb.c')
    af=source('drivers/gpu/drm/apple/afk.c')
    rtkit=source('drivers/soc/apple/rtkit.c')
    template=source('drivers/gpu/drm/apple/iomfb_template.c')
    assert 'alloc_ordered_workqueue("rtkit-%s"' in rtkit
    assert '.recv_message = dcp_recv_msg' in dcp
    attributes=template[template.index('"DisplayAttributes"'):]
    # Catalogue ownership: DisplayAttributes records the dimensions for the
    # transfer's attachment and defers the DRM update to dimensions_wq.
    owned='dcp_attributes_replace(' in attributes[:attributes.index('}')]
    assert ('dcp_attributes_replace(dcp, &ctx,' if owned else 'dcp_set_dimensions(dcp);') in attributes
    assert 'queue_work(ep->wq, &work->work)' in af
    prefix=r'''
#undef NDEBUG
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <pthread.h>
#include <time.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define GFP_KERNEL 0
#define kzalloc(n,f) calloc(1,n)
#define kfree(p) free(p)
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define INIT_WORK(p,f) ((p)->fn=(f))
#define dev_warn(...) ((void)0)
#define DCP_FW_VER 26
#define DCP_FW_VERSION(a,b,c) (a)
#define SZ_1M (1024*1024)
#define DCP_IOMFB_METHODS_H17P_EXTERNAL 2
#define dev_warn_ratelimited(...) ((void)0)
#define dev_dbg_ratelimited(...) ((void)0)
#define trace_dcp_recv_msg(...) ((void)0)
#define DCP_FIRMWARE_H17P 2
#define IOMFB_ENDPOINT 0x37
#define AV_ENDPOINT 0x29
#define SYSTEM_ENDPOINT 0x20
#define DISP0_ENDPOINT 0x23
#define DPTX_CONTROLLER_ENDPOINT 0x24
#define APPLE_RTKIT_EP_MGMT 0
#define APPLE_RTKIT_EP_CRASHLOG 1
#define APPLE_RTKIT_EP_SYSLOG 2
#define APPLE_RTKIT_EP_DEBUG 3
#define APPLE_RTKIT_EP_IOREPORT 4
#define APPLE_RTKIT_EP_OSLOG 8
#define APPLE_RTKIT_APP_ENDPOINT_START 0x20
struct mutex { pthread_mutex_t native; };
static _Thread_local struct mutex *locks[4];
static _Thread_local unsigned int depth;
static bool held(struct mutex *m) { for(unsigned int i=0;i<depth;i++) if(locks[i]==m) return true; return false; }
static void mutex_lock(struct mutex *m) { assert(!held(m) && depth<4); assert(!pthread_mutex_lock(&m->native)); locks[depth++]=m; }
static void mutex_unlock(struct mutex *m) { assert(depth && locks[depth-1]==m); depth--; assert(!pthread_mutex_unlock(&m->native)); }
#define lockdep_assert_held(p) assert(held(p))
#define spin_lock_irqsave(p,f) do { mutex_lock(p); (f)=0; } while(0)
#define spin_unlock_irqrestore(p,f) do { (void)(f); mutex_unlock(p); } while(0)
struct work_struct { void (*fn)(struct work_struct *); };
struct workqueue_struct { struct work_struct *pending; pthread_cond_t ready; pthread_mutex_t lock; };
struct drm_device { struct { struct mutex mutex; } mode_config; };
struct drm_connector { struct drm_device *dev; struct { int width_mm,height_mm; } display_info; };
struct apple_connector { struct drm_connector base; struct work_struct hotplug_wq; };
struct apple_dcp;
struct apple_dcp_afkep { struct apple_dcp *dcp; u8 endpoint; struct mutex lock; bool stopping; struct workqueue_struct *wq; };
struct test_mode_ticket { u64 generation; bool associated; };
struct apple_dcp {
    bool quiescing; int fw_compat; void *dev;
    struct { int iomfb_method_profile; bool fw_14_7_endpoints; } hw; bool ext_dead,external_native;
    struct { struct mutex lock; u64 epoch; bool enabled; void *service; } dcpavserv;
    struct test_mode_ticket dummy_ticket;
    struct { u64 connection_sequence; } rate_proof;
    struct { struct mutex lock; struct { u64 generation; bool connected; } epoch; } metadata;
    u64 typec_generation; bool typec_cable_connected;
    struct { size_t length; void *data; u64 av_service_epoch, automatic_connection_sequence; bool automatic_service_pending; struct test_mode_ticket mode_ticket; } chunks;
    int width_mm,height_mm; struct { int width_mm,height_mm; } panel;
    unsigned int nr_modes; struct { struct { int width_mm,height_mm; } mode; } modes[1];
    struct apple_connector *connector;
    struct apple_dcp_afkep *dcpavservep,*dptxep,*avep,*systemep,*ibootep,*dptx_controller_ep;
    struct { struct mutex lock; bool stopped; void *owner; } iomfb;
};
struct afk_receive_message_work { struct apple_dcp_afkep *ep; u64 message; struct work_struct work; };
struct apple_rtkit_ops { void (*recv_message)(void *,u8,u64); void (*recv_message_timed)(void *,u8,u64,u64); };
struct apple_rtkit { bool shutting_down, sync_syslog_ack; const struct apple_rtkit_ops *ops; void *cookie,*dev; };
struct apple_rtkit_rx_work { u64 received_ns; struct apple_rtkit *rtk; u8 ep; u64 msg; struct work_struct work; };
#define current ((void *)1)
static pthread_mutex_t reply_lock=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t reply_ready=PTHREAD_COND_INITIALIZER;
static bool reply_done;
static unsigned int advances, dimension_updates;
static struct workqueue_struct dimensions_queue={.ready=PTHREAD_COND_INITIALIZER,.lock=PTHREAD_MUTEX_INITIALIZER};
static struct workqueue_struct afk_queue={.ready=PTHREAD_COND_INITIALIZER,.lock=PTHREAD_MUTEX_INITIALIZER};
static void queue_work(struct workqueue_struct *q,struct work_struct *w)
{ assert(!pthread_mutex_lock(&q->lock)); assert(!q->pending); q->pending=w; pthread_cond_signal(&q->ready); assert(!pthread_mutex_unlock(&q->lock)); }
static void schedule_work(struct work_struct *w) { queue_work(&dimensions_queue,w); }
static void afk_receive_message_worker(struct work_struct *w)
{
    struct afk_receive_message_work *a=container_of(w,struct afk_receive_message_work,work);
    assert(a->ep->endpoint==0x2a && a->message==7);
    assert(!pthread_mutex_lock(&reply_lock)); reply_done=true;
    pthread_cond_signal(&reply_ready); assert(!pthread_mutex_unlock(&reply_lock)); free(a);
}
static bool iomfb_uses_queue(struct apple_dcp *d) { (void)d; return true; }
static void iomfb_queue_advance(struct apple_dcp *d) { assert(held(&d->iomfb.lock)); advances++; }
static void dcp_set_dimensions(struct apple_dcp *d);
static u8 dcpep_cb_prop_start(struct apple_dcp *d,u32 *length);
static void dcp_firmware_metadata_begin(struct apple_dcp *d) { (void)d; }
#define dcp_metadata_begin(e) ((struct test_mode_ticket){.generation=(e)->generation,.associated=(e)->connected})
static void iomfb_recv_message(struct apple_dcp *d,u64 message)
{
    assert(message==1 && held(&d->iomfb.lock)); dcp_set_dimensions(d);
    u32 length=64;
    assert(dcpep_cb_prop_start(d,&length));
}
#define rx_unused(name) static void __attribute__((unused)) name(struct apple_rtkit *r,u64 m) { (void)r;(void)m; }
rx_unused(apple_rtkit_management_rx)
rx_unused(apple_rtkit_crashlog_rx)
rx_unused(apple_rtkit_syslog_rx)
rx_unused(apple_rtkit_kdebug_rx)
rx_unused(apple_rtkit_ioreport_rx)
rx_unused(apple_rtkit_oslog_rx)
'''
    parts=[function(dcp,'void dcp_set_dimensions('), function(template,'static u8 dcpep_cb_prop_start(')]
    if 'void dcp_update_connector_dimensions(' in dcp:
        parts.append(function(dcp,'void dcp_update_connector_dimensions('))
    else:
        parts.append('static void dcp_update_connector_dimensions(struct apple_dcp *d, struct drm_connector *c) { (void)d; (void)c; }')
    parts += [function(iomfb,'void iomfb_recv_msg('),function(af,'int afk_receive_message('),
              function(dcp,'static struct apple_dcp_afkep *dcp_afkep('),
              function(dcp,'static void dcp_recv_msg('),function(rtkit,'static void apple_rtkit_rx_work(')]
    suffix=r'''
static struct drm_device drm={.mode_config.mutex.native=PTHREAD_MUTEX_INITIALIZER};
static struct apple_connector connector={.base.dev=&drm};
static struct apple_dcp d={.connector=&connector,.width_mm=600,.height_mm=340,.nr_modes=1,.fw_compat=DCP_FIRMWARE_H17P,.iomfb.lock.native=PTHREAD_MUTEX_INITIALIZER};
static struct apple_dcp_afkep av={.dcp=&d,.endpoint=0x2a,.lock.native=PTHREAD_MUTEX_INITIALIZER,.wq=&afk_queue};
static void dimension_work(struct work_struct *w)
{
    (void)w;
    mutex_lock(&drm.mode_config.mutex);
    dcp_update_connector_dimensions(&d,&connector.base); dimension_updates++;
    mutex_unlock(&drm.mode_config.mutex);
}
static void *worker(void *arg)
{
    struct workqueue_struct *q=arg;
    assert(!pthread_mutex_lock(&q->lock));
    while(!q->pending) assert(!pthread_cond_wait(&q->ready,&q->lock));
    struct work_struct *w=q->pending; q->pending=NULL;
    assert(!pthread_mutex_unlock(&q->lock)); w->fn(w); return NULL;
}
static const struct apple_rtkit_ops ops={.recv_message=dcp_recv_msg};
static struct apple_rtkit rtk={.ops=&ops,.cookie=&d};
static void *ordered_rx(void *arg)
{
    (void)arg;
    /* One ordered RX worker must complete DisplayAttributes before AFK. */
    struct apple_rtkit_rx_work *a=calloc(1,sizeof(*a)); assert(a);
    a->rtk=&rtk; a->ep=IOMFB_ENDPOINT; a->msg=1; apple_rtkit_rx_work(&a->work);
    a=calloc(1,sizeof(*a)); assert(a);
    a->rtk=&rtk; a->ep=0x2a; a->msg=7; apple_rtkit_rx_work(&a->work);
    return NULL;
}
int main(void)
{
    pthread_t rx,afk,dimensions;
    (void)schedule_work; /* The negative baseline does not defer dimensions. */
    d.dcpavservep=&av; connector.hotplug_wq.fn=dimension_work;
    d.hw.iomfb_method_profile=DCP_IOMFB_METHODS_H17P_EXTERNAL;
    assert(!pthread_mutex_init(&d.dcpavserv.lock.native,NULL)); d.dcpavserv.epoch=1;
    assert(!pthread_mutex_init(&d.metadata.lock.native,NULL)); d.metadata.epoch.generation=1;
    d.metadata.epoch.connected=true; d.typec_generation=1; d.typec_cable_connected=true;
    mutex_lock(&drm.mode_config.mutex); /* DRM get_modes holds this while waiting. */
    assert(!pthread_create(&afk,NULL,worker,&afk_queue));
    assert(!pthread_create(&dimensions,NULL,worker,&dimensions_queue));
    assert(!pthread_mutex_lock(&reply_lock));
    assert(!pthread_create(&rx,NULL,ordered_rx,NULL));
    struct timespec deadline; assert(!clock_gettime(CLOCK_REALTIME,&deadline)); deadline.tv_sec+=2;
    int rc=0;
    while(!reply_done && !rc) rc=pthread_cond_timedwait(&reply_ready,&reply_lock,&deadline);
    if(!reply_done) { fputs("FAIL: ordered RTKit RX blocked before AFK reply while mode_config.mutex held\n",stderr); fflush(stderr); _Exit(1); }
    assert(!pthread_mutex_unlock(&reply_lock));
    assert(!dimension_updates); /* Deferred worker cannot take the probe's mutex. */
    mutex_unlock(&drm.mode_config.mutex);
    assert(!pthread_join(rx,NULL) && !pthread_join(afk,NULL) && !pthread_join(dimensions,NULL));
    assert(advances==1 && dimension_updates==1 && connector.base.display_info.width_mm==600 && connector.base.display_info.height_mm==340);
    assert(d.modes[0].mode.width_mm==600 && d.modes[0].mode.height_mm==340);
    assert(d.chunks.length==64 && d.chunks.data); free(d.chunks.data);
    assert(!pthread_mutex_destroy(&d.dcpavserv.lock.native));
    assert(!pthread_mutex_destroy(&d.metadata.lock.native));
    puts("PASS: actual ordered RTKit RX -> synchronous IOMFB dimensions/property-start callbacks -> AFK enqueue makes reply progress while DRM mutex held; deferred dimensions update completes after release");
}
'''
    if owned:
        prefix=prefix.replace('struct apple_connector { struct drm_connector base; struct work_struct hotplug_wq; };',
            'struct apple_connector { struct drm_connector base; struct work_struct hotplug_wq,bl_sync_wq; };',1)
        prefix=prefix.replace('    int width_mm,height_mm; struct { int width_mm,height_mm; } panel;',
            '    int width_mm,height_mm; struct { int width_mm,height_mm; } panel;\n'
            '    struct mutex modes_lock; u64 modes_generation,dimensions_generation; struct work_struct dimensions_wq; bool ext_backlight;',1)
        prefix=prefix.replace('struct { size_t length; void *data;','struct { size_t length; void *data; u64 modes_generation;',1)
        prefix=prefix.replace('static void schedule_work(struct work_struct *w) { queue_work(&dimensions_queue,w); }','''
static struct workqueue_struct hotplug_queue={.ready=PTHREAD_COND_INITIALIZER,.lock=PTHREAD_MUTEX_INITIALIZER};
static void (*hotplug_fn)(struct work_struct *);
/* dimensions_wq and the connector's hotplug work run on separate workers. */
static void schedule_work(struct work_struct *w) { queue_work(w->fn==hotplug_fn?&hotplug_queue:&dimensions_queue,w); }
static struct mutex *guard_lock(struct mutex *m) { mutex_lock(m); return m; }
static void guard_unlock(struct mutex **m) { if(*m) mutex_unlock(*m); }
#define guard(kind) struct mutex * __attribute__((cleanup(guard_unlock))) _guard = guard_lock
#define scoped_guard(kind,l) for(struct mutex *_scoped __attribute__((cleanup(guard_unlock)))=guard_lock(l);_scoped;mutex_unlock(_scoped),_scoped=NULL)
#define DCP_FIRMWARE_V_14_7 (DCP_FIRMWARE_H17P + 1)
#define DCP_FIRMWARE_V_14_7_TEMPLATE (DCP_FIRMWARE_H17P + 2)
static bool dcp_has_panel(struct apple_dcp *d) { (void)d; return false; }
/* The parser is a leaf here: the attributes report the display's 60x34 cm. */
struct dcp_parse_ctx { int unused; };
static int parse_display_attributes(struct dcp_parse_ctx *h,int *w,int *ht,bool *backlight)
{ (void)h; *w=600; *ht=340; *backlight=false; return 0; }
static int dcp_attributes_replace(struct apple_dcp *d,struct dcp_parse_ctx *h,u64 generation);
static u64 dcp_modes_transfer_begin(struct apple_dcp *d);''',1)
        old_rx='''    assert(message==1 && held(&d->iomfb.lock)); dcp_set_dimensions(d);'''
        assert old_rx in prefix
        prefix=prefix.replace(old_rx,'''    struct dcp_parse_ctx attributes={0};
    assert(message==1 && held(&d->iomfb.lock));
    assert(!dcp_attributes_replace(d,&attributes,dcp_modes_transfer_begin(d)));''',1)
        prefix=prefix.replace('static void dcp_set_dimensions(struct apple_dcp *d);','static void dcp_set_dimensions(struct apple_dcp *d,u64 generation);',1)
        prefix=prefix.replace('static u64 dcp_modes_transfer_begin(struct apple_dcp *d);','static u64 dcp_modes_transfer_begin(struct apple_dcp *d);\n'+internal_inline(root,'dcp_fw_is_14_7'),1)
        property_header=(root/'drivers/gpu/drm/apple/dcp-property.h').read_text()
        parts=['#include <string.h>\n'+''.join(line+'\n' for line in property_header.splitlines() if not line.startswith('#include'))
               .replace('static inline ','static inline __attribute__((unused)) ').replace('SZ_16M','(16*SZ_1M)')]+parts
        parts+=[apple_function(root,name) for name in ('dcp_modes_transfer_begin','dcp_modes_dimensions_work','dcp_attributes_replace')]
        old_main='''    d.dcpavservep=&av; connector.hotplug_wq.fn=dimension_work;'''
        assert old_main in suffix
        suffix=suffix.replace(old_main,'''    pthread_t hotplug;
    d.dcpavservep=&av; connector.hotplug_wq.fn=dimension_work; hotplug_fn=dimension_work;
    d.dimensions_wq.fn=dcp_modes_dimensions_work;
    assert(!pthread_mutex_init(&d.modes_lock.native,NULL));
    assert(!pthread_create(&hotplug,NULL,worker,&hotplug_queue));''',1)
        old_join='''    assert(!pthread_join(rx,NULL) && !pthread_join(afk,NULL) && !pthread_join(dimensions,NULL));'''
        assert old_join in suffix
        suffix=suffix.replace(old_join,old_join+'\n    assert(!pthread_join(hotplug,NULL));',1)
    return prefix + '\n'.join(parts) + suffix


def startup(root):
    source = (root / 'drivers/gpu/drm/apple/epic/dpavservep.c').read_text()
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#define DCP_IOMFB_METHODS_H17P_EXTERNAL 2
#define DPAVSERV_ENDPOINT 0x28
#define DPAVSERV_ENDPOINT_V2 0x2a
#define IS_ERR(p) ((uintptr_t)(p) > (uintptr_t)-4096)
#define PTR_ERR(p) ((int)(intptr_t)(p))
#define ERR_PTR(e) ((void *)(intptr_t)(e))
#define dev_warn(...) do { warnings++; } while (0)
#define dev_dbg(...) do { debug++; } while (0)
struct apple_dcp;
struct apple_dcp_afkep {
    struct apple_dcp *dcp;
    unsigned endpoint;
    bool wire_v2,inline_v2,match_epic_name;
};
struct apple_dcp {
    void *dev;
    struct { int iomfb_method_profile; bool fw_14_7_endpoints; } hw; bool ext_dead,external_native;
    struct { unsigned enable_completion; bool enabled; } dcpavserv;
    struct apple_dcp_afkep *dcpavservep;
};
static const unsigned dpavservep_ops[] = {0};
static struct apple_dcp_afkep endpoint;
static int init_error,start_error;
static unsigned warnings,waits,starts,shutdowns,detaches,debug;
static bool announce;
static void init_completion(unsigned *c) { *c=0; }
static unsigned msecs_to_jiffies(unsigned ms) { return ms; }
static unsigned wait_for_completion_timeout(unsigned *c,unsigned timeout)
{ assert(timeout==1000);waits++;return *c; }
static struct apple_dcp_afkep *afk_init(struct apple_dcp *d,unsigned ep,const void *ops)
{
    assert(ops==dpavservep_ops);
    endpoint=(struct apple_dcp_afkep){.dcp=d,.endpoint=ep};
    return init_error ? ERR_PTR(init_error) : &endpoint;
}
static int afk_start(struct apple_dcp_afkep *ep)
{
    starts++;
    assert(ep==&endpoint&&ep->match_epic_name);
    if(announce&&!start_error) {
        ep->dcp->dcpavserv.enabled=true;
        ep->dcp->dcpavserv.enable_completion=1;
    }
    return start_error;
}
static void afk_shutdown(struct apple_dcp_afkep *ep)
{ assert(ep==&endpoint);shutdowns++; }
static void dpavservep_detach(struct apple_dcp *d)
{ detaches++;d->dcpavserv.enabled=false; }
''' + block(source, 'int dpavservep_init(') + r'''
int main(void)
{
    for(unsigned v2=0;v2<2;v2++) {
        for(unsigned early=0;early<2;early++) {
            struct apple_dcp d={.hw={.iomfb_method_profile=v2?2:0}};
            announce=early;waits=warnings=starts=shutdowns=detaches=debug=0;
            assert(dpavservep_init(&d)==0);
            assert(starts==1&&waits==!v2&&!warnings&&!shutdowns&&!detaches);
            assert(d.dcpavservep==&endpoint&&endpoint.wire_v2==v2&&!endpoint.inline_v2);
            assert(endpoint.endpoint==(v2?DPAVSERV_ENDPOINT_V2:DPAVSERV_ENDPOINT));
            /* A missing optional service must not be fabricated as enabled. */
            assert(d.dcpavserv.enabled==early);
            assert(debug==(!v2&&!early));
            const int errors[]={-ENOMEM,-ETIMEDOUT};
            for(unsigned i=0;i<sizeof(errors)/sizeof(errors[0]);i++) {
                d=(struct apple_dcp){.hw={.iomfb_method_profile=v2?2:0}};
                start_error=errors[i];waits=warnings=starts=shutdowns=detaches=0;
                assert(dpavservep_init(&d)==errors[i]);
                assert(starts==1&&!waits&&shutdowns==1&&detaches==1);
                assert(!d.dcpavservep&&!d.dcpavserv.enabled);
                start_error=0;
            }
            init_error=-ENOMEM;starts=shutdowns=detaches=0;
            assert(dpavservep_init(&d)==-ENOMEM);
            assert(!starts&&!shutdowns&&!detaches);
            init_error=0;
        }
    }
    puts("PASS optional legacy discovery, AFKV2 early/absent service, endpoint failures");
    return 0;
}
'''
