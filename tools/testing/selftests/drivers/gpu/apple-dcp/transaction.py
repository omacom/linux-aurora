# SPDX-License-Identifier: GPL-2.0-only
# Copyright 2026 Ryan Murray
"""Exact production callbacks with the adverse-review counterexample assertions.

Firmware, MMIO and USB role operations are synthetic leaves. The concurrency
case executes production transactions with real pthread connector/fabric locks.
"""
import quiesce
from common import block, apple_function


def controls(ROOT):
    tests = []
    def check(name, unit, cases=((),), negative=False, flags=()):
        assert not negative
        tests.append((name, unit, cases, flags))
    base=quiesce.make_unit(ROOT)
    # Actual generic role helper, omitted by the author's role-leaf fixture.
    core=(ROOT/'drivers/usb/typec/tipd/core.c').read_text()
    generic=base.replace(block(base,'int main('),block(base,'int main(').replace('int main(','int __attribute__((unused)) prior_main(',1),1)
    generic+=r'''
    int main(void) {
     struct cd321x c;initial(&c);data.remove=NULL;role_error=-EBUSY;
     tipd_remove(&c.tps);
     fprintf(stderr,"generic role failure: role_calls=%d error=%d reset=%d gpio=%d events=%s\n",role_calls,last_error,reset_calls,gpio_calls,events);
     assert(last_error==-EBUSY&&reset_calls==0&&gpio_calls==0&&unregisters==1);
    }
    '''
    check('generic-role-error-lost',generic)
    second_role=generic.replace('data.remove=NULL;role_error=-EBUSY;', 'role_error=-EBUSY;',1)
    second_role=second_role.replace("role_calls++;return role_error;", "role_calls++;return role_calls==1?0:role_error;",1)
    check('cd-second-role-error-lost',second_role)

    # Replace the modeled DCP set/deactivate bodies with exact production bodies.
    dcp=(ROOT/'drivers/gpu/drm/apple/dcp.c').read_text()
    actual=base.replace(block(base,'int main('),block(base,'int main(').replace('int main(','int __attribute__((unused)) prior_main(',1),1)
    actual=actual.replace('struct typec_mux_state { void *alt;', 'struct typec_altmode { unsigned svid; };\nstruct typec_mux_state { struct typec_altmode *alt;',1)
    actual=actual.replace('struct connector { bool connected; };','struct encoder { unsigned possible_crtcs; };\nstruct connector { bool connected,edid_live,edid_invalidation_pending; void *dcp; struct encoder *port_encoder; unsigned candidate_crtcs; int bl_sync_wq; };\n#define apple_connector connector\nstruct mux_control { int unused; };',1)
    actual=actual.replace('struct apple_dcp { int external_lock,external_error;', '''struct apple_dcp { int external_lock,external_error;
     bool external_power_attempted,dptx_tunnel,tb_clock_ok,tb_clock_refused,fixed_route_selected;
     int tb_lock; unsigned typec_generation,dptx_dfp_port,dptx_phy,fixed_dptx_phy,fixed_mux_index,typec_reconnect_tries;
     void *tb_dpin_set_active,*tb_dpin_ctx,*phy,*fixed_phy;
     struct mux_control *xbar,*fixed_xbar;
     /* desktop 26.x hybrid hand-over and dual-pipe state; fw_compat is never desktop here */
     bool peer_starved,typec_narrow,hdmi_yielded;
     struct apple_dcp_typec_route *active_typec_route,*typec_routes;
     struct connector *connector,*fixed_connector;int connector_type,fixed_connector_type;
     struct mux_control *direct_xbar_up; bool external_native, ext_dead; int fw_compat; unsigned long long tb_generation;
     int modes_lock; unsigned long long modes_generation; bool modes_admitted,modes_provisional;
     struct { int iomfb_method_profile; } hw;
     /* Thunderbolt DP IN events queued for the DPTX port (dcp_tb_events_retire()) */
     int tb_event_lock,tb_event_work; unsigned long tb_events; unsigned long long tb_event_generation;
     struct apple_dcp_typec_route *tb_event_route;
    ''',1)
    # The hybrid hand-over walks a port's routes; the only route is this port's.
    actual=actual.replace('struct apple_dcp_typec_port { int link;', 'struct apple_dcp_typec_port { int link,routes; bool typec_yielded;',1)
    # A port's owner slots are one array, as in production (one per DP IN with fixed
    # CRTCs): the model's owner is slot 0, and the others stay NULL on this port.
    actual=actual.replace('struct apple_dcp_typec_route *owner,*target,', 'union { struct apple_dcp_typec_route *owner; struct apple_dcp_typec_route *slots[DCP_FABRIC_DPIN_MAX]; }; struct apple_dcp_typec_route *target,',1)
    actual=actual.replace('#define list_for_each_entry(p,h,m) for((void)(h),(p)=sole_port();(p);(p)=NULL)',
        '#define list_for_each_entry(p,h,m) for((void)(h),(p)=_Generic((p),struct apple_dcp_typec_route *:sole_route(),default:sole_port());(p);(p)=NULL)',1)
    actual=actual.replace('static struct apple_dcp_typec_port *sole_port(void) { return &port; }',
        'static struct apple_dcp_typec_port *sole_port(void) { return &port; }\nstatic struct apple_dcp_typec_route * __attribute__((unused)) sole_route(void) { return &route; }',1)
    actual=actual.replace('bool applied_valid,safe_pending,dp_wanted,dp_hpd,hpd;', 'struct typec_altmode *applied_alt; unsigned long applied_mode; unsigned applied_status,applied_conf; bool applied_valid,safe_pending,dp_wanted,dp_hpd,hpd;',1)
    actual=actual.replace('void *xbar; bool deactivate_xbar_done; int deactivate_error; bool selected,tunnel;', 'struct mux_control *xbar,*active_xbar; void *phy; unsigned dptx_phy,tunnel_dpin,mux_index;unsigned long long tunnel_generation;bool deactivate_xbar_done; int deactivate_error;bool xbar_up,selected,tunnel,sleep_parked;',1)
    extra=r'''
    /* actual DCP leaves */
    #define WRITE_ONCE(x,v) ((x)=(v))
    /* No T8132 dcpext here: every route has DP alt mode (production: the dcpext compatible). */
    struct apple_dcp_typec_route;
    static bool dcp_typec_route_no_dp_alt(const struct apple_dcp_typec_route *route) { (void)route; return false; }
    struct apple_dcp;
    static bool dcp_is_t8132_ext(const struct apple_dcp *dcp) { (void)dcp; return false; }
    static bool dcp_t6041_ext_v27(const struct apple_dcp *dcp) { (void)dcp; return false; }
    #define dcp_queue_typec_reconnect(dcp,delay) ((void)(dcp),(void)(delay))
    #define dev_info_once(...) ((void)0)
    #define USB_TYPEC_DP_SID 0xff01
    #define TYPEC_DP_STATE_A 10
    #define TYPEC_DP_STATE_F 15
    #define DP_STATUS_HPD_STATE 1
    #define DP_STATUS_IRQ_HPD 2
    #define DRM_MODE_CONNECTOR_USB 1
    #define MUX_IDLE_DISCONNECT -1
    #define symbol_get(fn) (&fn)
    #define symbol_put(fn) ((void)0)
    struct typec_displayport_data { u32 status,conf; };
    static int gpiod_get_value_cansleep(void *p);
    static int dcp_dpxbar_preselect(struct mux_control *m,int s) { (void)m;(void)s;return 0; }
    static int dcp_dpxbar_tunnel_select_source(struct mux_control *m,int s) { (void)m;(void)s;return 0; }
    static int __attribute__((unused)) apple_atc_dp_tunnel_rate(void *p,unsigned n,unsigned rate) { (void)p;(void)n;(void)rate;return 0; }
    static int xbar_error,xbar_calls;
static int mux_control_deselect(struct mux_control *m) { (void)m;event('C');xbar_calls++;return xbar_error; }
    static int mux_control_select(struct mux_control *m,unsigned n) { (void)m;(void)n;return 0; }
    static void dpavservep_detach(struct apple_dcp *d) { (void)d; }
    /* Desktop 26.6 dual-pipe peers: no scenario here is on that firmware. */
    static void __attribute__((unused)) dcp_peer_release_26_6(struct apple_dcp *d) { (void)d;assert(!"desktop 26.6 peer release"); }
    /* failed takes of the hybrid, reset when a Type-C display leaves DP */
    static unsigned int __attribute__((unused)) dcp_hybrid_fails;
    static void dcp_firmware_metadata_connection(struct apple_dcp *d,unsigned n,bool c) { (void)d;(void)n;(void)c; }
    static bool dcp_typec_keep_order(void) { return false; }
    static struct apple_dcp_typec_route * __attribute__((unused)) dcp_typec_lowest_free(struct apple_dcp_typec_port *p) { (void)p;return &route; }
    static struct apple_dcp_typec_route * __attribute__((unused)) dcp_typec_free_route(struct apple_dcp_typec_port *p) { (void)p;return &route; }
    static int connect_calls;
    static int dcp_dptx_connect_oob(void *dev,unsigned n) { (void)dev;(void)n;connect_calls++;display.external_cable=true;display.dptxport[0].connected=true;display.typec_cable_connected=true;connector.connected=true;return 0; }
    static void dcp_retrain_oob(struct connector *c) { (void)c; }
    static bool dcp_typec_sink_relink(struct apple_dcp *d) { (void)d;return false; }
    /* An M1/M2 crossbar is not the T8142 layout, and an M1/M2 port has no fixed-CRTC connectors. */
    static bool dcp_xbar_t8142(struct mux_control *m) { (void)m;return false; }
    static bool dcp_typec_fixed_crtcs(void) { return false; }
    /* Deactivation retires queued DP IN events and drains their work. */
    static int tb_event_cancels;
    static bool cancel_work_sync(int *w) { assert(w==&display.tb_event_work);tb_event_cancels++;return false; }
    '''
    # Catalogue/EDID ownership: deactivation ends the Type-C attachment of the
    # pipeline's modes and moves the connector's pipeline through the real helpers.
    extra+='''
    struct platform_device;
    #ifndef DCP_IOMFB_METHODS_H17P_EXTERNAL
    #define DCP_IOMFB_METHODS_H17P_EXTERNAL 2
    #endif
    /* T6030 external processors (dcp->external_native) are not in these scenarios. */
    static void __attribute__((unused)) dcp_external_sink_irq(struct apple_dcp *d) { (void)d;assert(!"T6030 external sink IRQ"); }
    static bool __attribute__((unused)) iomfb_v14_7_external_failed(struct apple_dcp *d) { (void)d;assert(!"T6030 external state");return false; }
    /* fw_compat is UNKNOWN here: never H17P, 14.7 or 26.6. */
    #ifndef DCP_FIRMWARE_H17P
    #define DCP_FIRMWARE_H17P 5
    #endif
    /* No route here is a tunnel, and no IRQ_HPD arrives for a disconnected sink. */
    static int __attribute__((unused)) dcp_route_clock_rate(struct apple_dcp_typec_route *r,void *phy,unsigned dpin,u8 rate) { (void)r;(void)phy;(void)dpin;(void)rate;assert(!"tunnel pixel clock");return -EINVAL; }
    static int __attribute__((unused)) dcp_dptx_recover_irq(struct apple_dcp *d) { (void)d;assert(!"IRQ_HPD recovery");return -EINVAL; }
    static int __attribute__((unused)) of_property_read_string(void *np,const char *name,const char **out) { (void)np;(void)name;(void)out;return -EINVAL; }
    '''+''.join(apple_function(ROOT,name) for name in ('apple_connector_invalidate_edid','apple_connector_set_pipeline','dcp_modes_end_typec','dcp_typec_port_busy','dcp_tb_events_retire'))
    pos=actual.index('static int dcp_typec_route_deactivate(')
    actual=actual[:pos]+extra+actual[pos:]
    # The fabric core split: availability is the real dcp_fabric_available() over
    # the real ownership (SAFE lease included) and HDMI presence of the pipeline.
    # T6030 dock support keys availability and HDMI settling on the ordered
    # dual-stream policy, the real dcp_fabric_ordered_dual_stream() of the wiring.
    available=''.join(block(dcp,marker)+'\n' for marker in (
        'static bool dcp_typec_safe_held(struct apple_dcp *dcp)\n{', 'bool dcp_typec_dual_stream(void)\n{',
        'bool dcp_typec_ordered_dual_stream(void)\n{',
        'static bool dcp_hdmi_settle_enabled(struct apple_dcp *dcp)\n{',
        'static enum dcp_fabric_presence_state dcp_hdmi_presence(struct apple_dcp *dcp)\n{') if marker in dcp)
    if 'bool dcp_typec_ordered_dual_stream(void)\n{' in dcp:
        # These scenarios are an M1/M2 Type-C port, never the native T6030 or T6050 machine.
        available='''static bool of_machine_is_compatible(const char *compat) { assert(!strcmp(compat,"apple,t6030")||!strcmp(compat,"apple,t6050"));return false; }
    '''+available
    # Route-follow: the fixed output, or a failed T6030 external processor, keeps the route busy.
    available+=apple_function(ROOT,'dcp_typec_route_busy')
    # Hybrid-first policy: the real helper over the module parameter's and the
    # device tree's defaults (-1: from the DT; no apple,typec-hybrid-first here).
    hybrid_first=apple_function(ROOT,'dcp_typec_hybrid_first')
    if hybrid_first:
        available='static int typec_hybrid_first=-1;\n    static bool typec_hybrid_first_dt;\n    '+hybrid_first+available
    if available:
        actual=actual.replace('struct apple_dcp { int external_lock,external_error;','struct apple_dcp { int external_lock,external_error;\n     struct dcp_fabric_presence hdmi_presence; int hdmi_presence_lock; unsigned nr_typec_routes;',1)
        available='''#define atomic_read(p) (*(p))
    #define spin_lock_irqsave(l,f) ((void)(l),(f)=0)
    #define spin_unlock_irqrestore(l,f) ((void)(l),(void)(f))
    static int dcp_dual_stream_routes;
    '''+available
    actual=actual.replace(block(actual,'static bool dcp_typec_route_available('), 'static bool dcp_typec_route_fixed_output_busy(struct apple_dcp_typec_route *r) { (void)r;return false; }\n'+available+block(dcp,'static bool dcp_typec_route_available('),1)
    actual=actual.replace(block(actual,'static int dcp_typec_route_deactivate('),block(dcp,dcp.rindex('static int dcp_typec_route_deactivate(')),1)
    pos=actual.index('static struct typec_mux_dev phy_provider')
    # route_set asks whether a tunnel holds any of the port's slots: the real helper.
    tunneled='#ifndef ARRAY_SIZE\n#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))\n#endif\n'+apple_function(ROOT,'dcp_typec_port_tunneled')
    route_apply=apple_function(ROOT,'dcp_typec_route_apply')
    if route_apply:
        # The ordinary SAFE fixture supplies a leaf for the T8142-only finish
        # path. Here set must execute the complete production apply body too.
        stub=block(actual,'static int dcp_typec_route_apply(')
        actual=actual.replace(stub, 'static int dcp_typec_route_apply(struct typec_mux_dev *,struct typec_mux_state *);', 1)
        pos=actual.index('static struct typec_mux_dev phy_provider')
    actual=actual[:pos]+tunneled+block(dcp,'static bool dcp_typec_route_is_dp(')+'\n'+route_apply+block(dcp,'static int dcp_typec_route_set(')+'\n'+actual[pos:]
    actual=actual.replace(block(actual,'static int display_set('),'''static int display_set(struct typec_mux_dev *m,struct typec_mux_state *s) { event('S');return dcp_typec_route_set(m,s); }''',1)
    actual=actual.replace("(void)m;assert(s->mode==TYPEC_STATE_SAFE);event('L');", "(void)m;(void)s;event('L');",1)
    actual=actual.replace('connector=(struct connector){.connected=true};','connector=(struct connector){.connected=true};',1)
    actual=actual.replace('phy_provider=(struct typec_mux_dev){.set=phy_set};','display.typec_routes=&route;display.fixed_connector_type=DRM_MODE_CONNECTOR_USB;display.active_typec_route=&route;\n phy_provider=(struct typec_mux_dev){.set=phy_set};',1)
    controls=r'''
static int late_finish_error(struct typec_mux_dev *m,struct typec_mux_state *s) { (void)m;(void)s;return -EIO; }
    int main(int argc,char **argv) {
     (void)argc;struct cd321x c;initial(&c);struct typec_mux_state safe={.mode=TYPEC_STATE_SAFE};
     if(argv[1][0]=='o') {
      display.tb_events=1;display.tb_event_route=&route;display.tb_event_generation=3;
      assert(typec_mux_set(&mux,&safe)==0);
      assert(stops==1&&!port.owner&&!port.safe_pending&&!route.selected);
      assert(!display.tb_events&&!display.tb_event_route&&!display.tb_event_generation&&tb_event_cancels>=1);
      puts("PASS actual DCP prepare + route_set + route_deactivate + framework normal SAFE");
     }
     if(argv[1][0]=='r') {
      mux.mux_devs[0]=&display_provider;mux.mux_devs[1]=&phy_provider;phy_error=-ETIMEDOUT;
      assert(typec_mux_set(&mux,&safe)==-ETIMEDOUT);
      fprintf(stderr,"display-first failed SAFE: owner=%p lease=%d selected=%d events=%s\n",(void *)port.owner,port.safe_pending,route.selected,events);
      assert(port.owner==&route&&port.safe_pending&&route.selected);
     }
     if(argv[1][0]=='d') {
      phy_error=-ETIMEDOUT;assert(typec_mux_set(&mux,&safe)==-ETIMEDOUT&&port.safe_pending&&port.owner==&route);
      phy_error=0;struct typec_altmode alt={.svid=USB_TYPEC_DP_SID};
      struct typec_displayport_data data={.status=DP_STATUS_HPD_STATE};
      struct typec_mux_state dp={.alt=&alt,.mode=TYPEC_DP_STATE_A,.data=&data};
      int ret=typec_mux_set(&mux,&dp);
      fprintf(stderr,"DP after failed SAFE: ret=%d connects=%d pending=%d wanted=%d hpd=%d\n",ret,connect_calls,port.safe_pending,port.dp_wanted,port.hpd);
      assert(ret<0&&connect_calls==0&&!port.dp_wanted);
     }
     if(argv[1][0]=='a') {
      struct altmode alt={.mux=&mux,.adev={.svid=USB_TYPEC_DP_SID}};phy_error=-ETIMEDOUT;
      int ret=typec_altmode_set_mux(&alt,TYPEC_STATE_SAFE,NULL);
      fprintf(stderr,"altmode SAFE: ret=%d stops=%d phy_calls=%d lease=%d\n",ret,stops,phy_calls,port.safe_pending);
      assert(ret==-ETIMEDOUT&&stops==1&&port.safe_pending);
     }
     if(argv[1][0]=='x') {
      xbar_error=-EIO;assert(typec_mux_set(&mux,&safe)==-EIO);
      assert(port.owner==&route&&port.safe_pending&&!port.safe_finish_ready&&route.selected&&route.deactivate_xbar_done&&route.deactivate_error==-EIO);
      xbar_error=0;assert(typec_mux_set(&mux,&safe)==-EIO&&port.owner==&route&&port.safe_pending&&xbar_calls==1);
     }
     if(argv[1][0]=='f') {
      mux.mux_devs[0]=&display_provider;mux.mux_devs[1]=&phy_provider;phy_provider.finish=late_finish_error;
      assert(typec_mux_set(&mux,&safe)==-EIO);
      assert(port.owner==&route&&port.safe_pending&&port.safe_finish_ready&&!route.selected&&xbar_calls==1);
      assert(!dcp_typec_route_available(&route));
      struct apple_dcp peer={0};struct apple_dcp_typec_route peer_route={.dcp=&peer};
      assert(dcp_typec_route_available(&peer_route));
      phy_provider.finish=NULL;
      assert(typec_mux_set(&mux,&safe)==0&&!port.owner&&!port.safe_pending&&xbar_calls==1);
     }
     return 0;
    }
    '''
    bus=(ROOT/'drivers/usb/typec/bus.c').read_text()
    actual+='\nstruct altmode { struct typec_mux *mux; struct typec_altmode adev; };\n'+block(bus,'static inline int\ntypec_altmode_set_mux(')+'\n'+controls
    check('actual-dcp-normal-safe',actual,(('ordered',),))
    check('actual-dcp-finalizer-recovery',actual,(('xbar-error',),('finalizer',)))
    check('provider-order-releases-failed-lease',actual,(('release',),))
    check('failed-safe-allows-new-dp-admission',actual,(('dp',),))
    check('altmode-safe-skips-quiescence',actual,(('altmode',),))

    # Real pthread fabric/consumer mutexes. Pause the SAFE actor after its actual
    # DCP prepare has returned and released its locks, before any set callback.
    # A second handle can reference the same providers (class/altmode/CD handles
    # are separately acquired in the real framework).
    concurrent=actual.replace(block(actual,'int main('),block(actual,'int main(').replace('int main(','int __attribute__((unused)) route_controls_main(',1),1)
    concurrent=concurrent.replace('#define guard(k) (void)','')
    concurrent=concurrent.replace('#define scoped_guard(k,l) for(int once=((void)(l),1);once;once=0)','')
    locks=r'''
    #include <pthread.h>
    static pthread_mutex_t fabric_mutex=PTHREAD_MUTEX_INITIALIZER,external_mutex=PTHREAD_MUTEX_INITIALIZER,tb_mutex=PTHREAD_MUTEX_INITIALIZER,modes_mutex=PTHREAD_MUTEX_INITIALIZER,tb_event_mutex=PTHREAD_MUTEX_INITIALIZER;
    struct real_guard { pthread_mutex_t *mutex; bool held; };
    static struct real_guard take_lock(int *key) {
     pthread_mutex_t *m=key==&dcp_typec_fabric_lock?&fabric_mutex:key==&display.external_lock?&external_mutex:key==&display.modes_lock?&modes_mutex:key==&display.tb_event_lock?&tb_event_mutex:&tb_mutex;
     assert(pthread_mutex_lock(m)==0);return (struct real_guard){m,true};
    }
    static void drop_lock(struct real_guard *g) { if(g->held){g->held=false;assert(pthread_mutex_unlock(g->mutex)==0);} }
    #define guard(k) struct real_guard __attribute__((cleanup(drop_lock))) _guard = take_lock
    #define scoped_guard(k,l) for(struct real_guard __attribute__((cleanup(drop_lock))) _scope=take_lock(l);_scope.held;drop_lock(&_scope))
    static bool observed_unquiesced_safe;
    '''
    # Before the DCP leaves: the extracted helpers take real locks too.
    pos=concurrent.index('/* actual DCP leaves */')
    concurrent=concurrent[:pos]+locks+concurrent[pos:]
    concurrent=concurrent.replace("(void)m;(void)s;event('L');phy_calls++;", "(void)m;if(s->mode==TYPEC_STATE_SAFE&&display.dptxport[0].connected)observed_unquiesced_safe=true;event('L');phy_calls++;",1)
    concurrent+=r'''
    static pthread_mutex_t schedule_lock=PTHREAD_MUTEX_INITIALIZER;
    static pthread_cond_t schedule_cond=PTHREAD_COND_INITIALIZER;
    static bool prepared,continue_safe;
    static int safe_ret,dp_ret;
    static int scheduled_prepare(struct typec_mux_dev *m,struct typec_mux_state *s) {
     int ret=dcp_typec_route_prepare(m,s);
     if(s->mode==TYPEC_STATE_SAFE&&!s->alt) {
      pthread_mutex_lock(&schedule_lock);prepared=true;pthread_cond_broadcast(&schedule_cond);
      while(!continue_safe)pthread_cond_wait(&schedule_cond,&schedule_lock);
      pthread_mutex_unlock(&schedule_lock);
     }
     return ret;
    }
    static void *safe_actor(void *unused) {
     (void)unused;struct typec_mux_state safe={.mode=TYPEC_STATE_SAFE};safe_ret=typec_mux_set(&mux,&safe);return NULL;
    }
    int main(void) {
     struct cd321x c;initial(&c);display_provider.prepare=scheduled_prepare;
     pthread_t thread;assert(pthread_create(&thread,NULL,safe_actor,NULL)==0);
     pthread_mutex_lock(&schedule_lock);while(!prepared)pthread_cond_wait(&schedule_cond,&schedule_lock);pthread_mutex_unlock(&schedule_lock);
     struct typec_mux second_handle=mux;
     struct typec_altmode alt={.svid=USB_TYPEC_DP_SID};struct typec_displayport_data data={.status=DP_STATUS_HPD_STATE};
     struct typec_mux_state dp={.alt=&alt,.mode=TYPEC_DP_STATE_A,.data=&data};
     dp_ret=typec_mux_set(&second_handle,&dp);
     pthread_mutex_lock(&schedule_lock);continue_safe=true;pthread_cond_broadcast(&schedule_cond);pthread_mutex_unlock(&schedule_lock);
     assert(pthread_join(thread,NULL)==0);
     fprintf(stderr,"overlapping handles: dp_ret=%d safe_ret=%d connected_at_SAFE_PHY=%d events=%s\n",dp_ret,safe_ret,observed_unquiesced_safe,events);
     assert(!observed_unquiesced_safe);
    }
    '''
    check('concurrent-prepare-set-stale-quiescence',concurrent,flags=('-pthread',))
    check('connector-context-lifetime', lifetime(ROOT), flags=('-pthread',))
    check('probe-transaction-rollback', probe(ROOT))
    check('provider-unregister-drain', unregister(ROOT), flags=('-pthread',))
    check('park-before-host-role-exit', clock_order(ROOT))
    return tests


def lifetime(root):
    mux = (root / 'drivers/usb/typec/mux.c').read_text()
    unit = quiesce.make_unit(root)
    unit = unit.replace(block(unit, 'int main('),
                        block(unit, 'int main(').replace('int main(', 'int __attribute__((unused)) prior_main(', 1), 1)
    unit = unit.replace('struct typec_mux_context { struct mutex lock; };', r"""
struct list_head { struct list_head *next,*prev; };
struct fwnode_handle { unsigned refs; };
struct typec_mux_context { struct list_head list; struct fwnode_handle *fwnode; struct mutex lock; unsigned users; };
""")
    helpers = r"""
#include <stdlib.h>
#include <stdint.h>
#undef guard
#undef list_for_each_entry
static struct mutex typec_mux_contexts_lock={.value=PTHREAD_MUTEX_INITIALIZER};
static struct list_head typec_mux_contexts={&typec_mux_contexts,&typec_mux_contexts};
static struct fwnode_handle *fwnode_handle_get(struct fwnode_handle *f) { f->refs++;return f; }
static void fwnode_handle_put(struct fwnode_handle *f) { assert(f->refs);f->refs--; }
static void mutex_init(struct mutex *m) { assert(!pthread_mutex_init(&m->value,NULL)); }
static struct mutex *registry_lock(struct mutex *m) { assert(!pthread_mutex_lock(&m->value));return m; }
static void registry_unlock(struct mutex **m) { mutex_unlock(*m); }
#define guard(k) struct mutex * __attribute__((cleanup(registry_unlock))) held=registry_lock
#define kzalloc_obj(x) calloc(1,sizeof(x))
#define kfree(x) free(x)
#define ERR_PTR(e) ((void *)(intptr_t)(e))
#define list_for_each_entry(p,h,m) for(p=container_of((h)->next,__typeof__(*p),m); &(p)->m!=(h);p=container_of((p)->m.next,__typeof__(*p),m))
static void list_add_tail(struct list_head *n,struct list_head *h) { n->prev=h->prev;n->next=h;h->prev->next=n;h->prev=n; }
static void list_del(struct list_head *n) { n->prev->next=n->next;n->next->prev=n->prev; }
"""
    unit += helpers + block(mux, 'static struct typec_mux_context *typec_mux_context_get(') + '\n'
    unit += block(mux, 'static void typec_mux_context_put(') + '\n'
    unit += r"""
static struct typec_mux downstream;
static int forwarded,commits,aborts,finish_error;
static int forward_set(struct typec_mux_dev *m,struct typec_mux_state *s) { (void)m;forwarded++;return typec_mux_set(&downstream,s); }
static int finish(struct typec_mux_dev *m,struct typec_mux_state *s) { (void)m;(void)s;return finish_error; }
static void commit(struct typec_mux_dev *m,struct typec_mux_state *s) { (void)m;(void)s;commits++; }
static void abort_callback(struct typec_mux_dev *m,struct typec_mux_state *s) { (void)m;(void)s;aborts++; }
int main(void) {
 struct fwnode_handle a={0},b={0};
 struct typec_mux_context *one=typec_mux_context_get(&a),*two=typec_mux_context_get(&a),*other=typec_mux_context_get(&b);
 assert(one==two&&one!=other&&one->users==2&&a.refs==1&&b.refs==1);
 struct cd321x c;initial(&c);struct typec_mux_state safe={.mode=TYPEC_STATE_SAFE};
 struct typec_mux second=mux; mux.context=one;second.context=two;
 assert(typec_mux_begin(&mux,&safe)==0);
 unsigned before=phy_calls;assert(typec_mux_set(&second,&safe)==-EBUSY&&phy_calls==before);
 typec_mux_abort(&mux);assert(port.safe_pending&&port.owner==&route);
 phy_error=-ETIMEDOUT;assert(typec_mux_set(&second,&safe)==-ETIMEDOUT&&port.owner==&route);
 phy_error=0;assert(typec_mux_set(&second,&safe)==0&&!port.owner&&!port.safe_pending);
 struct typec_mux_dev leaf={.set=phy_set,.finish=finish,.commit=commit,.abort=abort_callback};
 downstream=(struct typec_mux){.context=other,.num_mux_devs=1,.mux_devs={&leaf}};
 struct typec_mux_dev forward={.set=forward_set};
 mux=(struct typec_mux){.context=one,.num_mux_devs=1,.mux_devs={&forward}};
 assert(typec_mux_set(&mux,&safe)==0&&forwarded==1&&commits==1);
 finish_error=-EIO;assert(typec_mux_set(&downstream,&safe)==-EIO&&commits==1&&aborts==1);
 finish_error=0;assert(typec_mux_set(&downstream,&safe)==0&&commits==2);
 downstream.context=one;before=phy_calls;
 assert(typec_mux_set(&mux,&safe)==-EBUSY&&phy_calls==before); // cyclic same-connector forwarding
 leaf.removed=true;downstream.context=other;before=phy_calls;
 assert(typec_mux_set(&downstream,&safe)==-ENODEV&&phy_calls==before&&aborts==1);
 typec_mux_context_put(one);assert(a.refs==1&&two->users==1);
 typec_mux_context_put(two);assert(a.refs==0);
 typec_mux_context_put(other);assert(b.refs==0&&typec_mux_contexts.next==&typec_mux_contexts);
 puts("PASS actual connector registry, shared handles, forwarding, finalize/abort and removed-provider guards");
}
"""
    return unit


def probe(root):
    core = (root / 'drivers/usb/typec/tipd/core.c').read_text()
    unit = quiesce.make_unit(root)
    unit = unit.replace(block(unit, 'int main('), block(unit, 'int main(').replace('int main(', 'int __attribute__((unused)) prior_main(', 1), 1)
    unit = unit.replace('struct tipd_data {', 'struct fwnode_handle;\nstruct tipd_data { unsigned irq_mask1; int (*init)(struct tps6598x *); int (*switch_power_state)(struct tps6598x *, int); int (*register_port)(struct tps6598x *, struct fwnode_handle *); bool (*read_data_status)(struct tps6598x *); bool (*read_power_status)(struct tps6598x *); int (*connect)(struct tps6598x *,u32); void (*ready)(struct tps6598x *); void *irq_handler;')
    unit = unit.replace('struct tps6598x {', 'struct tps6598x { bool dbma_at_probe,wakeup; u32 status; bool (*sleep_event)(struct tps6598x *);')
    helpers = r"""
#undef dev_err
#define dev_err(...) ((void)0)
#define TPS_SETUP_MS 1000
#define TPS_SYSTEM_POWER_STATE_S0 0
#define TPS_MODE_PTCH 2
#define TPS_MODE_DBMA 3
#define TPS_REG_VID 0
#define TPS_REG_INT_MASK1 1
#define TPS_STATUS_PLUG_PRESENT 1
#define GPIOD_OUT_LOW 0
#include <stdint.h>
#undef IS_ERR
#define IS_ERR(p) ((uintptr_t)(p)>=(uintptr_t)-4095)
#define PTR_ERR(p) ((int)(intptr_t)(p))
#define IRQF_SHARED 0
#define IRQF_ONESHOT 0
#define IRQF_NO_SUSPEND 0
#define POLL_INTERVAL 1000
#define INIT_DELAYED_WORK(...) ((void)0)
#define tps6598x_poll_work 0
#define devm_device_init_wakeup(...) ((void)0)
#define enable_irq_wake(...) ((void)0)
#define dev_err_probe(...) (-EINVAL)
struct fwnode_handle { int unused; };
static int connect_error,irq_error,register_error,node_puts,mux_get_error,controller_mode;
#define EPROBE_DEFER 517
/* Real OF/profile combinations are covered by j613-mux-order-tests.py. */
static bool gate_pending;
static bool cd321x_display_gate_pending(struct tps6598x *tps) { return gate_pending; }
static void *devm_gpiod_get_optional(void *d,const char *s,int v) { (void)d;(void)s;(void)v;return NULL; }
static void msleep(unsigned ms) { (void)ms; }
static bool device_is_compatible(void *d,const char *s) { (void)d;(void)s;return false; }
static int tps6598x_read32(struct tps6598x *t,int reg,u32 *v) { (void)t;(void)reg;*v=1;return 0; }
static int tps6598x_check_mode(struct tps6598x *t) { (void)t;return controller_mode; }
static int apply_patch(struct tps6598x *t) { (void)t;return 0; }
static int tps6598x_write64(struct tps6598x *t,int reg,unsigned v) { (void)t;(void)reg;(void)v;return 0; }
static bool tps6598x_read_status(struct tps6598x *t,u32 *s) { (void)t;*s=TPS_STATUS_PLUG_PRESENT;return true; }
static bool tps6598x_read_power_status(struct tps6598x *t) { (void)t;return true; }
static struct fwnode_handle node;
static struct fwnode_handle *device_get_named_child_node(void *d,const char *s) { (void)d;(void)s;return &node; }
static void fw_devlink_purge_absent_suppliers(struct fwnode_handle *f) { (void)f; }
static void *fwnode_usb_role_switch_get(struct fwnode_handle *f) { (void)f;return (void *)1; }
static struct typec_mux *fwnode_typec_mux_get(struct fwnode_handle *f) { (void)f;return mux_get_error?(struct typec_mux *)(intptr_t)mux_get_error:&mux; }
static void fwnode_handle_put(struct fwnode_handle *f) { (void)f;node_puts++; }
static int devm_tps6598_psy_register(struct tps6598x *t) { (void)t;return 0; }
static int devm_request_threaded_irq(void *d,int irq,void *h,void *th,int flags,const char *name,void *arg) { (void)d;(void)irq;(void)h;(void)th;(void)flags;(void)name;(void)arg;return irq_error; }
static int system_power_efficient_wq;
static unsigned msecs_to_jiffies(unsigned ms) { return ms; }
static void queue_delayed_work(int wq,int *work,unsigned delay) { (void)wq;(void)work;(void)delay; }
static bool device_property_read_bool(void *d,const char *p) { (void)d;(void)p;return false; }
static void tipd_debugfs_register(struct tps6598x *t) { (void)t; }
static int register_port(struct tps6598x *t,struct fwnode_handle *f) { (void)t;(void)f;return register_error; }
static bool read_data_status(struct tps6598x *t) { (void)t;return true; }
static int connect(struct tps6598x *t,u32 status) { (void)t;(void)status;return connect_error; }
static int cd321x_reset_inherited_connection(struct tps6598x *t,u32 *s) { (void)t;(void)s;return 0; }
"""
    helpers += '\n#define of_machine_is_compatible(name) (!strcmp((name), \"apple,t8140\"))\n#define BIT_ULL(n) (1ULL << (n))\n#define SN201202X_INT_DP_SID_UPDATE BIT_ULL(37)\n'
    helpers += block(core, 'static bool cd321x_uses_dp_sid(') + '\n' + block(core, 'static u64 cd321x_irq_mask(') + '\n'
    unit += helpers + block(core, 'int tipd_init(') + r"""
static void probe_initial(struct cd321x *c) {
 initial(c);ordinary_port=true;phy_provider.safe_after_role=true;c->tps.irq=1;data.register_port=register_port;data.read_data_status=read_data_status;data.read_power_status=tps6598x_read_power_status;data.connect=connect;
 connect_error=irq_error=register_error=node_puts=mux_get_error=0;
 /* tipd_init() resets only a controller it patched (stable a2b95607926d, mainline c5df31f10880) */
 controller_mode=TPS_MODE_PTCH;data.init=apply_patch;
}
int main(void) {
 struct cd321x c;
 probe_initial(&c);gate_pending=true;
 assert(tipd_init(&c.tps)==-EPROBE_DEFER&&!node_puts&&!reset_calls&&!gpio_calls);
 gate_pending=false;
 probe_initial(&c);irq_error=-ENOSPC;phy_error=-ETIMEDOUT;
 assert(tipd_init(&c.tps)==-ENOSPC&&reset_calls==0&&gpio_calls==0&&port.owner==&route&&port.safe_pending&&unregisters==1&&node_puts==1);
 probe_initial(&c);connect_error=-EAGAIN;role_error=-EBUSY;
 assert(tipd_init(&c.tps)==-EAGAIN&&reset_calls==0&&gpio_calls==0&&phy_calls==0&&port.owner==&route&&port.safe_pending);
 probe_initial(&c);irq_error=-ENOSPC;
 assert(tipd_init(&c.tps)==-ENOSPC&&reset_calls==1&&!port.owner&&!port.safe_pending&&unregisters==1);
 probe_initial(&c);irq_error=-ENOSPC;controller_mode=0;
 assert(tipd_init(&c.tps)==-ENOSPC&&reset_calls==0&&!port.owner&&!port.safe_pending&&unregisters==1);
 probe_initial(&c);mux_get_error=-ENODEV;
 assert(tipd_init(&c.tps)==-ENODEV&&reset_calls==0&&stops==0&&phy_calls==0&&unregisters==0&&node_puts==1);
 probe_initial(&c);register_error=-EIO;
 assert(tipd_init(&c.tps)==-EIO&&reset_calls==0&&stops==0&&phy_calls==0&&unregisters==0);
 puts("PASS actual probe rollback preserves primary errors, blocks reset after SAFE/role/registration errors and resets only a patched controller");
}
"""
    return unit


def unregister(root):
    """Actual unregister/begin/end with sleeping SRCU leaves and two threads."""
    source = (root / 'drivers/usb/typec/mux.c').read_text()
    unit = quiesce.make_unit(root)
    unit = unit.replace(block(unit, 'int main('), block(unit, 'int main(').replace('int main(', 'int __attribute__((unused)) prior_main(', 1), 1)
    unit = unit.replace('struct typec_mux_dev {', 'struct device { bool unregistered; };\nstruct typec_mux_dev { struct device dev;')
    old = block(unit, 'static int srcu_read_lock(') + '\n' + block(unit, 'static void srcu_read_unlock(')
    leaves = r"""
static pthread_mutex_t lifetime_lock=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t lifetime_cond=PTHREAD_COND_INITIALIZER;
static unsigned readers;
static bool draining;
static int srcu_read_lock(int *s) { (void)s;pthread_mutex_lock(&lifetime_lock);readers++;pthread_mutex_unlock(&lifetime_lock);return 0; }
static void srcu_read_unlock(int *s,int i) { (void)s;(void)i;pthread_mutex_lock(&lifetime_lock);assert(readers);readers--;pthread_cond_broadcast(&lifetime_cond);pthread_mutex_unlock(&lifetime_lock); }
static void synchronize_srcu(int *s) { (void)s;pthread_mutex_lock(&lifetime_lock);draining=true;pthread_cond_broadcast(&lifetime_cond);while(readers)pthread_cond_wait(&lifetime_cond,&lifetime_lock);pthread_mutex_unlock(&lifetime_lock); }
static void device_unregister(struct device *d) { assert(!readers);d->unregistered=true; }
#define WRITE_ONCE(x,v) ((x)=(v))
"""
    unit = unit.replace(old, leaves, 1)
    unit += block(source, 'void typec_mux_unregister(') + r"""
static void *remove_provider(void *arg) { typec_mux_unregister(arg);return NULL; }
int main(void) {
 struct cd321x c;initial(&c);struct typec_mux_state safe={.mode=TYPEC_STATE_SAFE};
 assert(typec_mux_begin(&mux,&safe)==0&&readers==1);
 pthread_t thread;assert(!pthread_create(&thread,NULL,remove_provider,&phy_provider));
 pthread_mutex_lock(&lifetime_lock);while(!draining)pthread_cond_wait(&lifetime_cond,&lifetime_lock);pthread_mutex_unlock(&lifetime_lock);
 assert(phy_provider.removed&&!phy_provider.dev.unregistered&&readers==1);
 assert(typec_mux_end(&mux)==0); // pre-existing transaction finishes before removal returns
 assert(!pthread_join(thread,NULL)&&phy_provider.dev.unregistered&&readers==0);
 unsigned before=phy_calls;
 assert(typec_mux_set(&mux,&safe)==-ENODEV&&phy_calls==before&&readers==0);
 puts("PASS actual unregister drains the open transaction and tombstone blocks stale handles");
}
"""
    return unit


def clock_order(root):
    unit = quiesce.make_unit(root)
    unit = unit.replace(block(unit, 'int main('), block(unit, 'int main(').replace('int main(', 'int __attribute__((unused)) prior_main(', 1), 1)
    unit = unit.replace('static int power_error,', 'static bool backend_clock=true,parked;\nstatic int power_error,')
    unit = unit.replace("event('R');role_calls++;return role_error;", "event('R');role_calls++;if(!role_error)backend_clock=false;return role_error;", 1)
    unit = unit.replace("event('L');phy_calls++;", "event('L');phy_calls++;if(parked)return 0;if(!backend_clock)return -ENODEV;if(!phy_error)parked=true;", 1)
    unit += r"""
int main(void) {
 (void)current;struct cd321x c;initial(&c);
 tipd_remove(&c.tps);
 assert(!last_error&&reset_calls==1&&gpio_calls==1&&parked&&!backend_clock);
 assert(position('Q')<position('L')&&position('L')<position('R'));
 assert(!port.owner&&!port.safe_pending);
 puts("PASS actual transaction parks before the role leaf removes its backend clock");
}
"""
    return unit
