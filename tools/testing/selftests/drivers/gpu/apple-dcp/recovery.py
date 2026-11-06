# SPDX-License-Identifier: GPL-2.0-only OR MIT
# Copyright 2026 Ryan Murray
"""Retained reviewer controls: actual AFK sender/receive, queue and EDID functions.

The wrap controls deliberately retain an expected replay assertion failure.
They document an unresolved peer contract, not successful replay protection.
"""
from pathlib import Path
import json, hashlib, subprocess
from common import classic_unit

def check(root, output):
    import edid
    out = output / 'service-recovery-controls'
    def extract(text,signature):
        start=text.index(signature);opening=text.index('{',start)
        while ';' in text[start:opening]:
            start=text.index(signature,start+1);opening=text.index('{',start)
        end,depth=opening+1,1
        while depth:
            depth+=(text[end]=='{')-(text[end]=='}');end+=1
        return text[start:end]

    unit=edid.rebind(root)
    # Retain the original exact fixture separately. Correct only its causal ordering.
    corrected=unit.replace('assert(afk_service_get(old)==old);old->cmd_map[0]=1;',
                           'assert(afk_service_get(old)==old);old->cmd_tag=1;old->cmd_map[0]=1;')
    corrected=corrected.replace('    old->cmd_tag=1;\n','')

    afk=(root/'drivers/gpu/drm/apple/afk.c').read_text()
    sender=extract(afk,'static int afk_send_command_timeout_len(')
    # The legacy reply-length variant moved the sender body into
    # afk_send_command_common(); afk_send_command_timeout_len() wraps it.
    if 'static int afk_send_command_common(' in afk:
        sender=extract(afk,'static int afk_send_command_common(')+'\n'+sender
    unit=unit[:unit.index('int main(void)')]
    unit=unit.replace('static unsigned int dma_frees,opens;',r'''
    static unsigned int dma_frees,opens;
    #define SZ_1M (1U<<20)
    #define GFP_KERNEL 0
    #define DECLARE_COMPLETION_ONSTACK(c) int c=0
    #define init_completion(c) (*(c)=0)
    #define msecs_to_jiffies(x) (x)
    #define cpu_to_le64(x) (x)
    #define kmalloc(n,g) malloc(n)
    #define EPIC_TYPE_COMMAND 3
    #define EPIC_CAT_COMMAND 3
    static int bitmap_find_free_region(unsigned long *p,int n,int order) {
        (void)order;for(int i=0;i<n;i++) if(!test_bit(i,p)) { *p|=1UL<<i;return i; } return -ENOSPC;
    }
    static void *dma_pool[2];static bool dma_busy[2];static unsigned int dma_cursor;
    static bool reuse_dma,ack_current,inject_late;
    static u8 current_reply[128],saved_reply[128];static size_t current_size,saved_size;
    static u16 current_channel;static u8 current_tag;
    static void afk_v2_receive(struct apple_dcp_afkep *,u32,u32,const void *,size_t);
    static void *dma_alloc_coherent(void *dev,size_t n,dma_addr_t *address,int flags) {
        (void)dev;(void)flags;assert(n==32);void *p;
        if(reuse_dma) {
            unsigned int i=dma_cursor++%2;assert(!dma_busy[i]);dma_busy[i]=true;
            if(!dma_pool[i]) dma_pool[i]=calloc(1,n);
            p=dma_pool[i];memset(p,0,n);
        } else p=calloc(1,n);
        assert(p);*address=(uintptr_t)p;return p;
    }
    static int wait_for_completion_timeout(int *completion,unsigned int timeout) {
        (void)timeout;
        if(inject_late) {
            afk_v2_receive(&ep_placeholder,0,0,saved_reply,saved_size);
            assert(!*completion && "late ACK completed an actual new production command");
        }
        if(ack_current) afk_v2_receive(&ep_placeholder,0,0,current_reply,current_size);
        return *completion;
    }
    '''.replace('&ep_placeholder','active_ep'))
    # The wait boundary must know the active endpoint before the fixture globals.
    unit=unit.replace('static unsigned int dma_frees,opens;',
                      'static struct apple_dcp_afkep *active_ep;\nstatic unsigned int dma_frees,opens;')
    unit=unit.replace('{ (void)dev;assert(size && (uintptr_t)p==address);free(p);dma_frees++; }',r'''
    { (void)dev;assert(size && (uintptr_t)p==address);
      if(reuse_dma) { unsigned int i=p==dma_pool[0]?0:1;assert(p==dma_pool[i] && dma_busy[i]);dma_busy[i]=false; }
      else free(p);dma_frees++; }
    ''')
    marker='    assert(channel && !tag && type==EPIC_TYPE_NOTIFY'
    unit=unit.replace(marker,r'''
        if(type==EPIC_TYPE_COMMAND) {

            assert(category==EPIC_CAT_COMMAND);
            if(e->inline_v2) {
                struct afkv2_command decoded;u8 response[40]={0};
                assert(!afkv2_decode_command(p,n,&decoded) && !decoded.oob && decoded.value==32 && decoded.id==tag);
                response[1]=decoded.id;
                assert(!afkv2_encode_frame(current_reply,sizeof(current_reply),1,channel,0,subtype,AFKV2_RESPONSE,0,response,sizeof(response),&current_size));
                current_channel=channel;current_tag=tag;return 0;
            }
            assert(n==sizeof(struct epic_cmd));
            const struct epic_cmd *cmd=p;u8 oob[32];
            assert(!afkv2_encode_oob_command(oob,sizeof(oob),tag,cmd->rxbuf,cmd->rxlen,cmd->txbuf,cmd->txlen));
            afkv2_put32(oob+4,0);
            assert(!afkv2_encode_frame(current_reply,sizeof(current_reply),1,channel,0,subtype,AFKV2_RESPONSE,0,oob,sizeof(oob),&current_size));
            current_channel=channel;current_tag=tag;return 0;
        }
    '''+marker)

    tests=r'''
    static void setup(void) {
        d.hw.iomfb_method_profile=2;d.typec_cable_connected=true;d.typec_generation=1;
        d.nr_modes=1;d.connector=&con;d.dev=&d;
        con.dcp=&d;con.connected=true;con.base.registration_state=1;
        ep.dcp=&d;ep.wire_v2=true;ep.endpoint=0x2a;ep.wq=&ep;ep.command_tags=calloc(AFKV2_INTERFACES,1);assert(ep.command_tags);
        d.dcpavservep=&ep;ep.ops=announcement_ops;ep.match_epic_name=true;
        d.fw_compat=DCP_FIRMWARE_H17P;hpd_lock=&d.hpd_mutex;av_lock=&d.dcpavserv.lock;active_ep=&ep;
    }
    static int send_actual(struct apple_epic_service *s,bool ack,bool late) {
        u8 payload[32]={0},output[32]={0};u32 code;
        ack_current=ack;inject_late=late;
        int result=afk_send_command_timeout_len(s,0xc0,payload,32,output,32,&code,1,NULL);
        inject_late=false;return result;
    }
    static void save_ack(void) { memcpy(saved_reply,current_reply,current_size);saved_size=current_size; }
    static void cleanup(void) {
        dpavservep_detach(&d);run_retries();
        for(unsigned int i=0;i<ep.num_channels;i++) assert(!ep.services[i].users && !ep.services[i].cmd_map[0] && !ep.services[i].pending_open);
        for(unsigned int i=0;i<2;i++) { assert(!dma_busy[i]);free(dma_pool[i]); }
        free(ep.command_tags);ep.command_tags=NULL;
    }
    int main(int argc,char **argv) {

        assert(argc==2);setup();reuse_dma=true;
        if(!strncmp(argv[1],"inline-",7)) ep.inline_v2=true;
        const char *provider=!strncmp(argv[1],"hdmi-",5)?class:native_class;
        announce(13,name,provider,13);struct apple_epic_service *old=d.dcpavserv.service;assert(old);
        if(!strcmp(argv[1],"deferred-recovery") || !strcmp(argv[1],"inline-deferred-recovery") || !strcmp(argv[1],"hdmi-deferred-recovery")) {
            assert(afk_service_get(old)==old);assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();
            closed(13,0x6a01,0);announce(13,name,native_class,13);run_retries();assert(!d.dcpavserv.service);
            afk_v2_receive(&ep,0,0,saved_reply,saved_size);run_retries();
            assert(d.dcpavserv.service && d.dcpavserv.service!=old && "R64 loses the admitted reannouncement");
            assert(send_actual(d.dcpavserv.service,true,true)==0);afk_service_put(old);
        } else if(!strcmp(argv[1],"drained-reader-rebind") || !strcmp(argv[1],"inline-drained-reader-rebind")) {
            assert(afk_service_get(old)==old);assert(send_actual(old,true,false)==0);save_ack();
            closed(13,0x6a01,0);announce(13,name,native_class,13);run_retries();
            assert(d.dcpavserv.service && d.dcpavserv.service!=old);
            assert(send_actual(d.dcpavserv.service,true,true)==0);afk_service_put(old);


        } else if(!strcmp(argv[1],"initial-discovery")) {
            dpavservep_detach(&d);d.typec_generation=0;d.typec_cable_connected=false;
            announce(13,name,native_class,13);assert(d.dcpavserv.service && !d.external_physically_absent);
        } else if(!strcmp(argv[1],"initial-absence")) {
            dpavservep_detach(&d);d.typec_generation=0;d.typec_cable_connected=false;d.external_physically_absent=true;
            unsigned int before=opens;announce(13,name,native_class,13);
            assert(!d.dcpavserv.service && opens==before);
        } else if(!strcmp(argv[1],"initial-open-absence")) {
            dpavservep_detach(&d);open_action=7;
            announce(13,name,native_class,13);assert(!d.dcpavserv.service);
            open_action=0;d.external_physically_absent=false;
        } else if(!strcmp(argv[1],"busy-original-ticket")) {
            dpavservep_detach(&d);
            struct apple_epic_service *pending=prepare(13,13,name,native_class);assert(pending);
            d.dcpavserv.enabled=true;
            assert(dcpavserv_revalidate(pending,name,native_class,13)==-EBUSY);
            d.dcpavserv.enabled=false;afk_service_discard(pending);
        } else if(!strcmp(argv[1],"channel-reuse")) {
            assert(afk_service_get(old)==old);assert(send_actual(old,true,false)==0);save_ack();
            closed(13,0x6a01,0);announce(5,name,class,5);run_retries();
            struct apple_epic_service *middle=d.dcpavserv.service;assert(middle && middle!=old && middle->channel==5);
            assert(send_actual(middle,true,true)==0);
            closed(5,0x6a01,0);announce(13,name,native_class,13);run_retries();
            assert(d.dcpavserv.service && d.dcpavserv.service==middle && d.dcpavserv.service!=old);
            assert(send_actual(d.dcpavserv.service,true,true)==0);
            afk_service_put(old);
        } else if(!strcmp(argv[1],"multiple-first-late-acks")) {
            reuse_dma=false;assert(afk_service_get(old)==old);
            assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();
            u8 first[56];size_t first_size=saved_size;memcpy(first,saved_reply,first_size);
            assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();
            assert(old->cmd_map[0]==3);
            closed(13,0x6a01,0);announce(13,name,native_class,13);run_retries();assert(!d.dcpavserv.service);
            afk_v2_receive(&ep,0,0,first,first_size);run_retries();
            assert(!d.dcpavserv.service && old->cmd_map[0]==2 && old->users==1);
            unsigned int freed=dma_frees;
            afk_v2_receive(&ep,0,0,first,first_size);run_retries();
            assert(dma_frees==freed && old->cmd_map[0]==2 && !d.dcpavserv.service);
            afk_v2_receive(&ep,0,0,saved_reply,saved_size);run_retries();
            assert(d.dcpavserv.service && d.dcpavserv.service!=old && !old->cmd_map[0]);
            afk_service_put(old);
        } else if(!strcmp(argv[1],"tag-wrap") || !strcmp(argv[1],"inline-tag-wrap")) {
            assert(send_actual(old,true,false)==0);save_ack();
            for(unsigned int i=0;i<15;i++) assert(send_actual(old,true,true)==0);
            assert(send_actual(old,true,true)==0);
        } else if(!strcmp(argv[1],"pending-closed")) {
            assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();
            closed(13,0x6a01,0);announce(13,name,native_class,13);run_retries();
            assert(!d.dcpavserv.service);
            closed(13,0x6a01,0);
            afk_v2_receive(&ep,0,0,saved_reply,saved_size);run_retries();
            assert(!d.dcpavserv.service && "CLOSED during pending recovery must invalidate the pending OPEN");
        } else if(!strcmp(argv[1],"actual-tag-before-announcement")) {
            assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();assert(ep.command_tags[13]==1);
            closed(13,0x6a01,0);announce(13,name,native_class,13);
            afk_v2_receive(&ep,0,0,saved_reply,saved_size);run_retries();
            assert(d.dcpavserv.service && send_actual(d.dcpavserv.service,true,true)==0);
        } else if(!strncmp(argv[1],"open-race-",10)) {
            assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();
            closed(13,0x6a01,0);announce(13,name,native_class,13);run_retries();
            if(!strcmp(argv[1],"open-race-generation")) open_action=1;
            else if(!strcmp(argv[1],"open-race-epoch")) open_action=2;
            else if(!strcmp(argv[1],"open-race-owner")) open_action=4;
            else if(!strcmp(argv[1],"open-race-cable")) open_action=5;
            else if(!strcmp(argv[1],"open-race-quiescing")) open_action=6;
            else if(!strcmp(argv[1],"open-race-absence")) open_action=7;
            else assert(!"unknown OPEN race");
            afk_v2_receive(&ep,0,0,saved_reply,saved_size);run_retries();
            if(open_action==4) assert(d.dcpavserv.service==injected_owner);
            else assert(!d.dcpavserv.service);
            open_action=0;d.quiescing=false;d.typec_cable_connected=true;d.external_physically_absent=false;
        } else if(!strcmp(argv[1],"physical-absence")) {
            assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();
            closed(13,0x6a01,0);announce(13,name,native_class,13);run_retries();
            d.external_physically_absent=true;
            afk_v2_receive(&ep,0,0,saved_reply,saved_size);run_retries();
            assert(!d.dcpavserv.service && "physical absence authority must fence deferred OPEN before cable/generation update");
        } else if(!strncmp(argv[1],"fence-",6)) {
            assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();
            closed(13,0x6a01,0);announce(13,name,native_class,13);run_retries();
            unsigned int before=opens;
            if(!strcmp(argv[1],"fence-epoch")) dpavservep_detach(&d);
            else if(!strcmp(argv[1],"fence-generation")) d.typec_generation++;
            else if(!strcmp(argv[1],"fence-cable")) d.typec_cable_connected=false;
            else if(!strcmp(argv[1],"fence-quiescing")) d.quiescing=true;
            else if(!strcmp(argv[1],"fence-owner")) inject_owner(&ep,13);
            else if(!strcmp(argv[1],"fence-open-failure")) open_error=-EIO;
            else assert(!"unknown fence");
            afk_v2_receive(&ep,0,0,saved_reply,saved_size);run_retries();
            assert(opens==before+(!strcmp(argv[1],"fence-open-failure")));
            if(!strcmp(argv[1],"fence-owner")) assert(d.dcpavserv.service==injected_owner);
            else assert(!d.dcpavserv.service);
            for(unsigned int i=0;i<ep.num_channels;i++) {
                assert(!ep.services[i].pending_open && !ep.services[i].pending_name && !ep.services[i].pending_class);
            }
            open_error=0;d.quiescing=false;d.typec_cable_connected=true;
        } else assert(!"unknown case");
        cleanup();printf("PASS actual production sender/receive: %s\n",argv[1]);
    }
    '''
    unit=unit.replace('bool hpd_mutex,quiescing,typec_cable_connected;',
                      'bool external_physically_absent,hpd_mutex,quiescing,typec_cable_connected;')
    if 'afk_reply_size_valid(' in sender:
        # The legacy reply-length path checks its size with the real helper; the
        # T6030 dcpext (dcp->external) timeout diagnostic never runs for H17P.
        unit=unit.replace('struct apple_dcp_afkep { u8 *command_tags;',
                          'struct apple_dcp_afkep { struct { bool ready;u32 *rptr,*wptr; } txbfr,rxbfr;u8 *command_tags;',1)
        unit=unit.replace('bool external_physically_absent,hpd_mutex,quiescing,typec_cable_connected;',
                          'bool external_physically_absent,hpd_mutex,quiescing,typec_cable_connected;bool external;void *rtk;',1)
        sender='''#include "afk_reply.h"
    #define U32_MAX UINT32_MAX
    #ifndef EOVERFLOW
    #define EOVERFLOW 75
    #endif
    static bool apple_rtkit_is_running(void *r) { (void)r;assert(!"T6030 dcpext diagnostic on H17P");return false; }
    static bool apple_rtkit_is_crashed(void *r) { (void)r;assert(!"T6030 dcpext diagnostic on H17P");return false; }
    '''+sender
    unit+=sender+'\n'+tests

    out.mkdir(parents=True,exist_ok=True)
    includes=[root/'drivers/gpu/drm/apple',root/'tools/testing/selftests/drivers/gpu/apple-afkv2/host-include']
    results=[]
    def run(name,text,cases):
        text=classic_unit(text)
        area=out/name;area.mkdir(exist_ok=True);(area/'host.c').write_text(text)
        cmd=['clang','-std=gnu11','-Wall','-Werror','-Wno-unused-function','-O1','-g','-fsanitize=address,undefined','-fno-omit-frame-pointer']+['-I'+str(p) for p in includes]+[str(area/'host.c'),'-o',str(area/'host')]
        (area/'command.json').write_text(json.dumps(cmd,indent=2)+'\n')
        proc=subprocess.run(cmd,capture_output=True,text=True);(area/'compile.log').write_text(proc.stdout+proc.stderr)
        results.append({'name':name,'phase':'compile','exit':proc.returncode,'unit_sha256':hashlib.sha256(text.encode()).hexdigest()})
        if proc.returncode: return
        for case in cases:
            proc=subprocess.run([str(area/'host'),*case],capture_output=True,text=True,timeout=20)
            (area/((case[0] if case else 'case-0')+'.log')).write_text(proc.stdout+proc.stderr)
            results.append({'name':name,'case':case,'exit':proc.returncode,'output':proc.stdout+proc.stderr})
    run('fixture-order-corrected',corrected,[[]])
    cases=[[c] for c in ['initial-discovery','initial-absence','initial-open-absence','busy-original-ticket','actual-tag-before-announcement','deferred-recovery','inline-deferred-recovery','hdmi-deferred-recovery','drained-reader-rebind','inline-drained-reader-rebind','channel-reuse','multiple-first-late-acks','tag-wrap','inline-tag-wrap','pending-closed',
                         'fence-epoch','fence-generation','fence-cable','fence-quiescing','fence-owner','fence-open-failure',
                         'open-race-generation','open-race-epoch','open-race-owner','open-race-cable','open-race-quiescing','open-race-absence','physical-absence']]
    run('actual-sender',unit,cases)
    # Actual RX worker loop and quiesce; only ring ingress/workqueue/kernel APIs are mocks.
    queued=unit
    queued=queued.replace('struct work_struct { bool pending; };',r'''
    struct work_struct { bool pending;void (*fn)(struct work_struct *);struct work_struct *next; };
    #define INIT_WORK(w,f) do { (w)->fn=(f);(w)->pending=false;(w)->next=NULL; } while(0)
    static struct work_struct *fifo_head,*fifo_tail;static bool destroyed,rx_running;
    ''')
    queued=queued.replace('static bool queue_work(void *wq,struct work_struct *w) { assert(wq);bool fresh=!w->pending;w->pending=true;return fresh; }',r'''
    static bool queue_work(void *wq,struct work_struct *w) {
        assert(wq && !destroyed);if(w->pending) return false;
        w->pending=true;w->next=NULL;if(fifo_tail) fifo_tail->next=w;else fifo_head=w;fifo_tail=w;return true;
    }
    static void run_one_work(void) {
        assert(fifo_head);struct work_struct *w=fifo_head;fifo_head=w->next;if(!fifo_head) fifo_tail=NULL;
        w->pending=false;w->next=NULL;assert(w->fn);w->fn(w);
    }
    static void destroy_workqueue(void *wq) { assert(wq && !destroyed);while(fifo_head) run_one_work();destroyed=true; }
    #define IS_ERR_OR_NULL(p) (!(p) || IS_ERR(p))
    #define debugfs_remove_recursive(p) ((void)(p))
    #define kzalloc(n,g) calloc(1,n)
    #define kvfree(p) free(p)
    #define ESHUTDOWN 108
    #define RBEP_TYPE 0xffff
    #define FIELD_GET(mask,val) ((val)&(mask))
    #define RBEP_INIT_ACK 0xa0
    #define RBEP_START_ACK 0x86
    #define RBEP_SHUTDOWN_ACK 0xc1
    #define RBEP_GETBUF 0x89
    #define RBEP_INIT_TX 0x8a
    #define RBEP_INIT_RX 0x8b
    #define RBEP_RECV 0x85
    #define complete_all(p) complete(p)
    ''')
    afh=(root/'drivers/gpu/drm/apple/afk.h').read_text()
    # The actual RX loop uses the real ring buffers instead of the diagnostic mock.
    queued=queued.replace('struct apple_dcp_afkep { struct { bool ready;u32 *rptr,*wptr; } txbfr,rxbfr;u8 *command_tags;',
                          'struct apple_dcp_afkep { u8 *command_tags;',1)
    queued=queued.replace('struct apple_dcp_afkep { u8 *command_tags;bool lock,stopping;',
                          'typedef u32 __le32;\n'+extract(afh,'struct afk_ringbuffer {')+';\n'
                          'struct apple_dcp_afkep { u8 *command_tags;int started,stopped;struct afk_ringbuffer txbfr,rxbfr;void *debugfs_entry;bool lock,stopping;')
    queued=queued.replace('bool external_physically_absent,hpd_mutex,quiescing,typec_cable_connected;',
                          'bool retain_dma,external_physically_absent,hpd_mutex,quiescing,typec_cable_connected;')
    queued=queued.replace('static void run_retries(void) { assert(!ep.lock);if(ep.service_work.pending) { ep.service_work.pending=false;afk_service_worker(&ep.service_work); } }',
                          'static void run_retries(void) { assert(!ep.lock);while(fifo_head) run_one_work(); }')
    queued=queued.replace('    if(type==EPIC_TYPE_COMMAND) {',
                          '    if(e->stopping) return -ESHUTDOWN;\n    if(type==EPIC_TYPE_COMMAND) {')
    queued=queued[:queued.index('int main(int argc')]
    queued+=r'''
    struct afk_receive_message_work { struct apple_dcp_afkep *ep;u64 message;struct work_struct work; };
    static void afk_getbuf(struct apple_dcp_afkep *e,u64 m) { (void)e;(void)m;assert(!"unexpected GETBUF"); }
    static void afk_init_rxtx(struct apple_dcp_afkep *e,u64 m,struct afk_ringbuffer *r) { (void)e;(void)m;(void)r;assert(!"unexpected INIT"); }
    static unsigned int rx_step,drain_warnings;static bool close_pending;static __le32 rx_rptr,rx_wptr;
    #define dev_warn_ratelimited(...) (drain_warnings++)
    static bool afk_recv(struct apple_dcp_afkep *e) {
        assert(e==&ep);rx_running=true;
        if(rx_step==0) closed(13,0x6a01,0);
        else if(rx_step==1) announce(13,name,native_class,13);
        else if(rx_step==2) afk_v2_receive(e,0,0,saved_reply,saved_size);
        else if(rx_step==3 && close_pending) closed(13,0x6a01,0);
        else { assert(!d.dcpavserv.service && "recovery must wait for the RX drain loop to finish");rx_running=false;return false; }
        assert(!d.dcpavserv.service && "recovery must remain behind RX on the ordered queue");rx_step++;return true;
    }
    '''
    af=(root/'drivers/gpu/drm/apple/afk.c').read_text()
    for signature in ['static inline unsigned int afk_ring_drain_budget(', 'static inline unsigned int afk_ring_drain(']:
        queued+=extract(afh,signature)+'\n'
    for signature in ['static bool afk_recv_entry(', 'static void afk_recv_drain(',
                      'static void afk_receive_message_worker(', 'int afk_receive_message(', 'void afk_quiesce(', 'static void afk_release_context(']:
        queued+=extract(af,signature)+'\n'
    queued+=r'''
    int main(int argc,char **argv) {
        assert(argc==2);setup();reuse_dma=true;INIT_WORK(&ep.service_work,afk_service_worker);
        ep.rxbfr=(struct afk_ringbuffer){.ready=true,.rptr=&rx_rptr,.wptr=&rx_wptr,.bufsz=0x1000,.stride=0x40};
        announce(13,name,native_class,13);struct apple_epic_service *old=d.dcpavserv.service;assert(old);
        assert(send_actual(old,false,false)==-ETIMEDOUT);save_ack();run_retries();
        unsigned int before=opens;
        if(!strncmp(argv[1],"ordered-",8)) {
            close_pending=!strcmp(argv[1],"ordered-pending-closed");
            assert(!afk_receive_message(&ep,RBEP_RECV));assert(fifo_head);
            run_one_work();assert(!rx_running && opens==before && !d.dcpavserv.service);
            assert(fifo_head==&ep.service_work);run_retries();
            if(close_pending) assert(!d.dcpavserv.service && "ordered CLOSED must cancel pending OPEN");
            else { assert(d.dcpavserv.service && opens==before+1);assert(send_actual(d.dcpavserv.service,true,true)==0); }
            cleanup();
        } else {
            assert(!strcmp(argv[1],"quiesce-pending"));assert(afk_service_get(old)==old);
            closed(13,0x6a01,0);announce(13,name,native_class,13);
            afk_v2_receive(&ep,0,0,saved_reply,saved_size);assert(ep.service_work.pending);
            afk_quiesce(&ep);assert(destroyed && !ep.wq && ep.stopping && !fifo_head);
            assert(opens==before && !d.dcpavserv.service && old->users==1);
            for(unsigned int i=0;i<ep.num_channels;i++) {
                assert(!ep.services[i].pending_open && !ep.services[i].pending_name && !ep.services[i].pending_class);
            }
            afk_service_put(old);assert(!fifo_head && !old->users);
            assert(afk_receive_message(&ep,RBEP_RECV)==-ESHUTDOWN);assert(!fifo_head);
            afk_quiesce(&ep);assert(!fifo_head);
            struct apple_dcp_afkep *empty=calloc(1,sizeof(*empty));assert(empty);empty->dcp=&d;
            afk_release_context(empty);
            cleanup();
        }
        assert(!drain_warnings);
        printf("PASS actual RX worker/quiesce: %s\n",argv[1]);
    }
    '''
    run('actual-queue-quiesce',queued,[[c] for c in ['ordered-recovery','ordered-pending-closed','quiesce-pending']])

    # Actual EDID wrapper exercises its owner/reader pins, epoch and generation exits.
    reader=unit[:unit.index('int main(int argc')]
    reader+=r'''
    #define ESTALE 116
    struct drm_edid { int marker; };
    static unsigned int edid_allocations,edid_frees;static int read_action;
    struct reader_guard { bool *lock;bool once; };
    static struct reader_guard lock_reader_guard(bool *lock) { mutex_lock(lock);return (struct reader_guard){lock,true}; }
    static void unlock_reader_guard(struct reader_guard *guard) { mutex_unlock(guard->lock); }
    #define scoped_guard(kind,p) for(struct reader_guard _guard __attribute__((cleanup(unlock_reader_guard)))=lock_reader_guard(p);_guard.once;_guard.once=false)
    static void drm_edid_free(const struct drm_edid *edid) { assert(edid && !IS_ERR(edid));free((void *)edid);edid_frees++; }
    static const struct drm_edid *dcpavserv_read_edid_inline(struct apple_epic_service *service) {
        assert(service->users==2 && "owner plus actual EDID wrapper reader pin");
        assert(send_actual(service,true,false)==0);
        if(read_action==1) { closed(service->channel,0x6a01,0);announce(5,name,class,5);assert(d.dcpavserv.service!=service); }
        if(read_action==2) d.typec_generation++;
        if(read_action==3) d.external_physically_absent=true;
        if(read_action==4) { dpavservep_detach(&d);d.quiescing=true; }
        struct drm_edid *edid=malloc(sizeof(*edid));assert(edid);edid->marker=1;edid_allocations++;return edid;
    }
    static const struct drm_edid *dcpavserv_read_edid_legacy(struct apple_epic_service *service) { return dcpavserv_read_edid_inline(service); }    /* The 14.7 open-before-EDID handshake is for T6030 external processors only. */
    static int __attribute__((unused)) afk_service_call(struct apple_epic_service *s,u16 group,u32 command,const void *data,size_t data_len,size_t data_pad,void *output,size_t output_len,size_t output_pad)
{ (void)s;(void)group;(void)command;(void)data;(void)data_len;(void)data_pad;(void)output;(void)output_len;(void)output_pad;assert(!"14.7 open-before-EDID on H17P");return -ENODEV; }
    /* Only the 14.7 external role opens the AV client and validates every EDID block. */
    #ifndef ECANCELED
    #define ECANCELED 125
    #endif
    static int __attribute__((unused)) dcpavserv_v14_open(struct apple_dcp *d,struct apple_epic_service *s,u64 epoch,u64 gen)
    { (void)d;(void)s;(void)epoch;(void)gen;assert(!"14.7 AV client open on H17P");return -ENODEV; }
    static bool __attribute__((unused)) drm_edid_valid(const struct drm_edid *e) { (void)e;assert(!"14.7 EDID validation on H17P");return false; }
    '''
    dpav=(root/'drivers/gpu/drm/apple/epic/dpavservep.c').read_text()
    for signature in ['bool dcpavserv_edid_current(', 'bool dcpavserv_edid_service_current(', 'const struct drm_edid *dcpavserv_copy_edid(']:
        reader+=extract(dpav,signature)+'\n'
    reader+=r'''
    int main(int argc,char **argv) {
        assert(argc==2);setup();reuse_dma=true;announce(13,name,class,13);
        struct apple_epic_service *old=d.dcpavserv.service;assert(old);
        if(!strcmp(argv[1],"reader-owner")) read_action=1;
        else if(!strcmp(argv[1],"reader-generation")) read_action=2;
        else if(!strcmp(argv[1],"reader-physical-absence")) read_action=3;
        else if(!strcmp(argv[1],"reader-quiescing")) read_action=4;
        else assert(!strcmp(argv[1],"reader-success"));
        u64 epoch;const struct drm_edid *edid=dcpavserv_copy_edid(&d,1,&epoch);
        if(!read_action) { assert(!IS_ERR(edid));drm_edid_free(edid); }
        else { assert(IS_ERR(edid) && "actual EDID wrapper must reject invalidated ownership/physical state"); }
        if(read_action==1 || read_action==4) assert(!old->users);else assert(old->users==1);
        assert(edid_allocations==edid_frees && !d.hpd_mutex && !d.dcpavserv.lock);
        d.quiescing=false;cleanup();printf("PASS actual copy_edid: %s\n",argv[1]);
    }
    '''
    run('actual-edid-reader',reader,[[c] for c in ['reader-success','reader-owner','reader-generation','reader-quiescing','reader-physical-absence']])
    mutations=[
        ('restart-tags','afkv2_command_id(ep->command_tags[service->channel]++, slot)','afkv2_command_id(0, slot)'),
        ('no-drain-retry','\tafk_service_retry(service->ep);',''),
        ('ignore-old-command','\t\tif (afk_epic_find_service(ep, service->channel))\n\t\t\tcontinue;',''),
        ('ignore-epoch','service->admission.epoch != dcp->dcpavserv.epoch ||\n\t\t ','false ||\n\t\t '),
        ('refresh-generation','!dcpavserv_admission_current(dcp, service->admission.generation))\n\t\tret = -ENODEV;','!dcpavserv_admission_current(dcp, dcp->typec_generation))\n\t\tret = -ENODEV;'),
        ('ignore-owner','if (dcp->dcpavserv.enabled)\n\t\tret = -EBUSY;\n\telse if (service->admission.epoch','if (false)\n\t\tret = -EBUSY;\n\telse if (service->admission.epoch'),
        ('ignore-stop','if (!READ_ONCE(ep->stopping))\n\t\tafk_retry_service_init(ep);','if (true)\n\t\tafk_retry_service_init(ep);'),
    ]

    for name,text,case in [
        ('cancel-closed',unit.replace('if (pending->pending_open && pending->channel == channel)','if (false && pending->pending_open && pending->channel == channel)'), 'pending-closed'),
        ('reader-lookup',unit.replace('(service->enabled ||','(service->users || service->enabled ||'), 'drained-reader-rebind'),
        ('reset-interface-history',unit.replace('service->cookie = NULL;', 'service->cookie = NULL; if(ep->wire_v2 && ep->command_tags) ep->command_tags[channel]=0;',1), 'channel-reuse'),
        ('absence-admission',unit.replace('!READ_ONCE(dcp->external_physically_absent) &&','true &&',1), 'initial-absence'),
        ('absence-publication',unit.replace('!READ_ONCE(dcp->external_physically_absent) &&','true &&',1), 'initial-open-absence'),
        ('absence-reader',reader.replace('!READ_ONCE(dcp->external_physically_absent)', 'true'), 'reader-physical-absence'),
    ]:
        run('mutation-'+name,text,[[case]])

    for name,old,new in mutations:
        assert corrected.count(old)==1,(name,corrected.count(old))
        run('mutation-'+name,corrected.replace(old,new),[[]])
    negative={
        'actual-sender':{'tag-wrap','inline-tag-wrap'},
            'actual-queue-quiesce':set(),
        'actual-edid-reader':set(),
    }
    for result in results:
        case=result.get('case',[])
        fail_expected=bool('case' in result and (result['name'].startswith('mutation-') or (case and case[0] in negative.get(result['name'],set()))))
        result['assertion_failure_expected']=fail_expected
        result['expectation_met']=result['exit']==(-6 if fail_expected else 0)
        if fail_expected: result['expectation_met'] = result['expectation_met'] and 'Assertion' in result.get('output','')
        if case and case[0] in {'tag-wrap','inline-tag-wrap'}:
            result['expectation_met'] = result['expectation_met'] and 'late ACK completed an actual new production command' in result.get('output','')
    (out/'results.json').write_text(json.dumps(results,indent=2)+'\n')

    assert all(r['expectation_met'] for r in results), "retained recovery control expectation failed; see results.json"
