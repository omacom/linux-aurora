# SPDX-License-Identifier: GPL-2.0-only
# Copyright 2026 Ryan Murray
"""Actual mux prepare/set and TPS removal ordering; firmware/PHY leaves are mocks."""
from common import block, atc_function, atc_cio_helpers, atc_mux_noop, fabric_core, fabric_rebalance

def make_unit(source, parent=False, reference=None):
 reference=source if reference is None else reference
 core=(source/'drivers/usb/typec/tipd/core.c').read_text();mux=(source/'drivers/usb/typec/mux.c').read_text()
 atc=(source/'drivers/phy/apple/atc.c').read_text()
 dcp=(reference/'drivers/gpu/drm/apple/dcp.c').read_text()
 prefix=r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <pthread.h>
typedef unsigned u32;
#define IS_ERR(p) false
#define IS_ERR_OR_NULL(p) (!(p))
#define EXPORT_SYMBOL_GPL(x)
#define guard(k) (void)
#define scoped_guard(k,l) for(int once=((void)(l),1);once;once=0)
#define READ_ONCE(x) (x)
#define schedule_work(w) ((void)(w))
#define WARN_ON_ONCE(x) (x)
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define to_platform_device(p) (p)
#define typec_mux_get_drvdata(m) ((m)->data)
#define dev_dbg(...) ((void)0)
#define dev_warn(...) ((void)0)
#define dev_info(...) ((void)0)
#define dev_name(p) "device"
#define list_for_each_entry(p,h,m) for((void)(h),(p)=sole_port();(p);(p)=NULL)
#define lockdep_assert_held(p) ((void)(p))
#define dev_err(dev,fmt,ret) do { (void)(dev);last_error=(ret); } while(0)
enum {TYPEC_STATE_SAFE,TYPEC_STATE_USB,TYPEC_ORIENTATION_NONE,TYPEC_PWR_MODE_USB,TYPEC_THUNDERBOLT_SWITCH_OFF};
enum usb_role {USB_ROLE_NONE,USB_ROLE_HOST,USB_ROLE_DEVICE};
enum typec_data_role {TYPEC_HOST,TYPEC_DEVICE};
#define TPS_STATUS_TO_TYPEC_PORTROLE(s) (s)
#define TPS_STATUS_TO_TYPEC_VCONN(s) (s)
#define TPS_STATUS_TO_TYPEC_DATAROLE(s) (s)
struct typec_mux_state { void *alt; unsigned long mode; void *data; };
struct typec_mux_dev { bool removed, safe_after_role; void (*commit)(struct typec_mux_dev *,struct typec_mux_state *); void (*abort)(struct typec_mux_dev *,struct typec_mux_state *); int (*finish)(struct typec_mux_dev *,struct typec_mux_state *); void *data; int (*set)(struct typec_mux_dev *,struct typec_mux_state *); int (*prepare)(struct typec_mux_dev *,struct typec_mux_state *); bool transient_timeouts; };
struct task_struct { int unused; };
static _Thread_local struct task_struct this_task;
#define current (&this_task)
struct mutex { pthread_mutex_t value; };
struct typec_mux_context { struct mutex lock; };
static struct typec_mux_context shared_context={.lock={.value=PTHREAD_MUTEX_INITIALIZER}};
static int typec_mux_srcu;
static int srcu_read_lock(int *s) { (void)s;return 0; }
static void srcu_read_unlock(int *s,int i) { (void)s;(void)i; }
static int mutex_trylock(struct mutex *m) { return pthread_mutex_trylock(&m->value)==0; }
static void mutex_unlock(struct mutex *m) { assert(!pthread_mutex_unlock(&m->value)); }
struct typec_mux { struct typec_mux_context *context; struct typec_mux_state transaction; struct task_struct *owner; int srcu_index; unsigned prepared; bool applied; unsigned num_mux_devs; struct typec_mux_dev *mux_devs[3]; bool timeout_transient; };
struct connector { bool connected; };
struct apple_dcp { int external_lock,external_error; bool external_cable,external_stopped; struct { bool connected; } dptxport[1]; void *hdmi_hpd; bool active; bool external,typec_cable_connected; void *dev; struct connector *typec_connector; bool ext_backlight; };
struct apple_dcp_typec_route;
struct apple_dcp_typec_port { int link; void *connector_np; struct apple_dcp_typec_route *owner,*target,*secondary_target,*preferred_route; void *applied_alt; unsigned long applied_mode; unsigned applied_status,applied_conf; bool applied_valid,safe_pending,safe_finish_ready,dp_wanted,dp_hpd,hpd,busy_noted;
    /* Held for a J714s firmware 27 sleep (dcp_fabric_sleep_release()): never here. */
    bool sleep_held,sleep_withdrawn; };
struct apple_dcp_typec_route { struct apple_dcp_typec_port *port; struct apple_dcp *dcp; void *xbar; bool deactivate_xbar_done; int deactivate_error; bool selected,tunnel; };
static int dcp_typec_fabric_lock,dcp_typec_ports,deactivate_calls;
struct drm_device { int unused; };
static struct apple_dcp display;
static struct connector connector;
static struct apple_dcp_typec_port port;
static struct apple_dcp_typec_route route;
static struct apple_dcp_typec_port *sole_port(void) { return &port; }
static char events[100];static unsigned count;
static int power_error,phy_error,tbt_error,role_error,last_error,phy_calls,reset_calls,gpio_calls,role_calls,stops,unregisters;
static bool ordinary_port,controller_active,phy_off,safe_noop;
static int ordinary_power_off(void);
static int ordinary_safe(void);
static void ordinary_initial(void);
static void ordinary_usb2_check(void);
static void event(char c) { assert(count+1<sizeof(events));events[count++]=c;events[count]=0; }
/* This fixture covers M1/M2 role removal, never T8142 DP admission. */
static bool dcp_t8142_ext(struct apple_dcp *d) { (void)d;return false; }
static int dcp_typec_route_apply(struct typec_mux_dev *m,struct typec_mux_state *s)
{ (void)m;(void)s;assert(!"unexpected T8142 route admission");return -EINVAL; }
static bool dcp_is_external_h17p(struct apple_dcp *d) { return d->external; }
static int dcp_dptx_disconnect_oob(void *dev,u32 index) { (void)dev;assert(index==0);event('Q');stops++;if(power_error){display.external_error=power_error;return power_error;}display.external_cable=false;display.external_stopped=true;display.dptxport[0].connected=false;display.typec_cable_connected=false;connector.connected=false;return 0; }
static struct drm_device *dcp_typec_drm(void) { return NULL; }
static void dcp_typec_plan(struct drm_device *d,struct apple_dcp_typec_port *p,unsigned n) { (void)d;(void)p;(void)n; }
static bool dcp_typec_tunnel_held(struct apple_dcp *d) { (void)d;return false; }
/* The desktop 26.6 (T6041) Type-C path is not under test: nothing here is on that firmware. */
#define DCP_FIRMWARE_V_26_6_DESKTOP 100
static void __attribute__((unused)) dcp_typec_route_phy_off(struct apple_dcp_typec_route *r)
{ (void)r; /* never powered: the real one returns at once */ }
static int dcp_typec_route_deactivate(struct apple_dcp_typec_route *r) { deactivate_calls++;r->selected=false;return 0; }
static void dcp_typec_port_attach(struct apple_dcp_typec_port *p) { (void)p; }
static int gpiod_get_value_cansleep(void *p) { (void)p;return 0; }
static int dcp_dptx_connect(struct apple_dcp *d,unsigned n) { (void)d;(void)n;return 0; }
static bool dcp_typec_route_available(struct apple_dcp_typec_route *r) { return !r->selected; }
static int dcp_typec_route_activate(struct apple_dcp_typec_route *r,void *x) { (void)x;r->selected=true;return 0; }
struct tps6598x;
struct tipd_data { REMOVE_RETURN (*remove)(struct tps6598x *); void (*unregister_port)(struct tps6598x *); int (*reset)(struct tps6598x *); };
struct typec_port { struct typec_mux *mux; };
struct tps6598x;
struct tipd_contaminant_ops { int (*update)(struct tps6598x *); void (*stop)(struct tps6598x *); };
struct tps6598x { bool remove_safe_prepared; struct typec_mux *mux; void *partner,*dev,*psy,*role_sw,*reset,*partner_altmode_dp; int irq,wq_poll; struct typec_port *port; struct tipd_data *data; const struct tipd_contaminant_ops *contaminant_ops; };
struct cd321x { struct tps6598x tps; struct typec_mux *mux; void *tbt_switch; };
struct typec_thunderbolt_switch_data { int state; };
static void cd321x_stop_work(struct cd321x *c) { (void)c;event('W'); }
static int typec_thunderbolt_switch_set(void *p,void *s) { (void)p;(void)s;event('B');return tbt_error; }
static int usb_role_switch_set_role(void *p,int r) { (void)p;assert(r==USB_ROLE_NONE);if(ordinary_port&&!role_error){int ret=ordinary_power_off();if(ret)return ret;controller_active=false;phy_off=true;}event('R');role_calls++;return role_error; }
static enum usb_role __attribute__((unused)) usb_role_switch_get_role(void *p) { (void)p;return controller_active?USB_ROLE_HOST:USB_ROLE_NONE; }
static void typec_unregister_partner(void *p) { (void)p;event('P'); }
/* These fixtures never enter DP on the partner, so its altmode stays NULL. */
static void typec_altmode_update_active(void *a,bool v) { (void)a;(void)v;assert(!"no partner DP altmode here"); }
static void typec_unregister_altmode(void *a) { (void)a;assert(!"no partner DP altmode here"); }
#define typec_set_pwr_opmode(...) ((void)0)
#define typec_set_pwr_role(...) ((void)0)
#define typec_set_vconn_role(...) ((void)0)
static int typec_set_orientation(void *p,int o) { (void)p;assert(o==TYPEC_ORIENTATION_NONE);return 0; }
static void typec_set_data_role(struct typec_port *p,enum typec_data_role r) { (void)p;(void)r;event('D'); }
static void dcp_typec_pipeline_freed(void) { event('F'); }
static void power_supply_changed(void *p) { (void)p; }
static void tipd_debugfs_unregister(struct tps6598x *t) { (void)t; }
/* The replug port list and its work (tipd_port_replug()) are outside these fixtures. */
static void tipd_port_list_del(struct tps6598x *t) { (void)t; }
static void cancel_delayed_work_sync(void *w) { (void)w; }
static void devm_free_irq(void *d,int i,void *t) { (void)d;(void)i;(void)t; }
static void typec_mux_put(struct typec_mux *m) { (void)m; }
static void usb_role_switch_put(void *p) { (void)p;event('V'); }
static void gpiod_set_value_cansleep(void *p,int v) { (void)p;assert(v==1);gpio_calls++;event('G'); }

'''.replace('REMOVE_RETURN','void' if parent else 'int')
 policy=block(mux,'bool typec_mux_safe_after_role(')+'\n' if 'bool typec_mux_safe_after_role(' in mux else ''
 functions=policy+'\n'.join(block(mux, marker) for marker in ('static void typec_mux_unlock(', 'void typec_mux_abort(', 'int typec_mux_begin(', 'int typec_mux_apply(', 'int typec_mux_end(', 'int typec_mux_prepare(')) if not parent else block((reference/'drivers/usb/typec/mux.c').read_text(),'int typec_mux_prepare(')+'\n'
 functions+=block(mux,'int typec_mux_set(')+'\n'+block(dcp,'static int dcp_typec_route_prepare(')+'\n'+block(dcp,'static int dcp_typec_route_finish(')+'\n'+block(dcp,'static void dcp_typec_route_commit(')+'\n'
 functions+=fabric_rebalance(dcp)
 if 'static bool tps6598x_safe_after_role(' in core:
  functions+=block(core,'static bool tps6598x_safe_after_role(')+'\n'
 if not parent:functions+=block(core,'static void tps6598x_drop_partner(')+'\n'+block(core,'static int tps6598x_set_data_role(')+'\n'
 # The fixed-hub SuperSpeed handover moved the locked mux body to atcphy_mux_switch();
 # atcphy_mux_set_locked() also reports whether a SuperSpeed port may come back up.
 split=[atc_cio_helpers(atc)]+[atc_function(atc,name) for name in ('atcphy_usb3_port_ready','atcphy_mux_check','atcphy_mux_park')]
 split+=[atc_mux_noop(atc),atc_function(atc,'atcphy_mux_switch')]
 functions+=block(core,'static '+('void' if parent else 'int')+' tps6598x_disconnect(')+'\n'+block(core,'static '+('void' if parent else 'int')+' cd321x_remove(')+'\n'+block(core,'void tipd_remove(')+'\n'
 ordinary_phy=r'''
/* Actual ordinary-port ATC power-off, mux recheck and USB2 init; MMIO leaves. */
#undef WARN_ON_ONCE
#define WARN_ON_ONCE(x) ({ bool hit=(x);phy_warns+=hit;hit; })
#define fallthrough __attribute__((fallthrough))
#define KUNIT_STATIC_STUB_REDIRECT(...) ((void)0)
enum atcphy_mode { APPLE_ATCPHY_MODE_OFF, APPLE_ATCPHY_MODE_USB2,
 APPLE_ATCPHY_MODE_USB3, APPLE_ATCPHY_MODE_USB3_DP, APPLE_ATCPHY_MODE_DP,
 APPLE_ATCPHY_MODE_TBT, APPLE_ATCPHY_MODE_USB4 };
enum atcphy_pipehandler_state { ATCPHY_PIPEHANDLER_STATE_DUMMY, ATCPHY_PIPEHANDLER_STATE_USB3 };
GENERATION_ENUM
struct atcphy_hw { GENERATION_FIELD bool has_usb4,has_t8140_dp,dp_t8122; u32 usb2phy_sig_host; };
struct apple_atcphy {
 struct { void *usb3_dp; } tunables;
 void *dev; int lock;
 const struct atcphy_hw *hw;
 struct { void *usb2phy; } regs;
 bool fixed_usb2,host_active,mode_dirty,pipe_pending,usb_off_pending,ss_tunables,dp_tunables,cio_tunables,t8132_dp_alt;
 bool usb3_port_off,park_owed,dp_only,pd_was_off;
 bool t8103_dummy_pending,t8103_usb3_retry,t8103_usb3_workaround;
 enum atcphy_mode mode,typec_mode;
 enum atcphy_pipehandler_state pipe_state;
 /* A CIO sleep is T8132's, covered by tools/asahi/atc-cio-sleep-tests.py. */
 struct { bool prepared; } cio_sleep;
};
struct phy { struct apple_atcphy *data; };
#define phy_get_drvdata(p) ((p)->data)
#define USB2PHY_SIG 0
#define USB2PHY_SIG_HOST 1
USB2_RUN_DEFINES
/* Retained USB2 power is exercised by dwc3-teardown, not these role removals. */
static u32 readl(void *p) { (void)p;assert(false);return 0; }
static struct { enum atcphy_pipehandler_state pipehandler_state; bool enable_dp_aux; } atcphy_modes[7];
static struct apple_atcphy ordinary_phy;
static struct phy usb_phy={.data=&ordinary_phy};
static unsigned phy_warns,configure_calls,lane_requests;
static void atcphy_restore_after_pd_off(struct apple_atcphy *p) { assert(!p->pd_was_off); }
static int atcphy_park_pipehandler(struct apple_atcphy *p, bool keep_dp) { (void)keep_dp;p->pipe_state=ATCPHY_PIPEHANDLER_STATE_DUMMY;return 0; }
static int atcphy_configure_pipehandler(struct apple_atcphy *p,bool on) { (void)p;(void)on;assert(false);return -EINVAL; }
/* Role exit parks the ordinary PIPE first; a live legacy park is never owed here. */
static int __attribute__((unused)) atcphy_park_live_t8103_pipehandler(struct apple_atcphy *p) { (void)p;assert(false);return -EINVAL; }
static struct atcphy_hw ordinary_hw GENERATION_INIT;
static int atcphy_configure(struct apple_atcphy *p,enum atcphy_mode mode) { configure_calls++;lane_requests+=mode!=APPLE_ATCPHY_MODE_OFF;p->mode=mode;p->mode_dirty=false;return 0; }
static void set32(void *p,unsigned v) { (void)p;(void)v;assert(false); }
static void atcphy_usb2_power_on(struct apple_atcphy *p) { (void)p;assert(false); }
''' + ''.join(split) + '\n'.join(block(atc, marker) for marker in (
  'static int atcphy_usb3_power_off(', 'static int atcphy_mux_set_locked(',
  'static int atcphy_usb2_init(')) + r'''
static int ordinary_power_off(void) { return atcphy_usb3_power_off(&usb_phy); }
static void ordinary_initial(void) {
 ordinary_phy=(struct apple_atcphy){.hw=&ordinary_hw,.mode=APPLE_ATCPHY_MODE_USB3,.typec_mode=APPLE_ATCPHY_MODE_USB3,.pipe_state=ATCPHY_PIPEHANDLER_STATE_USB3};
 phy_warns=configure_calls=lane_requests=0;
}
static int ordinary_safe(void) {
 unsigned before=configure_calls;
 int ret=atcphy_mux_set_locked(&ordinary_phy,APPLE_ATCPHY_MODE_OFF);
 assert(!phy_warns);
 if(!ret){assert(ordinary_phy.mode==APPLE_ATCPHY_MODE_OFF);assert(configure_calls==before);}
 return ret;
}
static void ordinary_usb2_check(void) {
 unsigned before=lane_requests;
 assert(atcphy_usb2_init(&usb_phy)==0);
 assert(ordinary_phy.typec_mode==APPLE_ATCPHY_MODE_OFF&&lane_requests==before);
}
'''
 # The ordinary port is the T8122 profile unless a case selects the T8103
 # generation, whose live legacy PIPE park the mux switch now consults.
 ordinary_phy=ordinary_phy.replace('USB2_RUN_DEFINES', '\n'.join(
  line for line in atc.splitlines()
  if line.startswith(('#define USB2PHY_USBCTL ', '#define USB2PHY_USBCTL_USB_MODE ',
                      '#define USB2PHY_USBCTL_RUN '))))
 generation='enum atcphy_generation {' in atc
 ordinary_phy=ordinary_phy.replace('GENERATION_ENUM\n',block(atc,'enum atcphy_generation {')+';\n' if generation else '',1)
 ordinary_phy=ordinary_phy.replace('GENERATION_FIELD ','enum atcphy_generation gen; ' if generation else '',1)
 ordinary_phy=ordinary_phy.replace(' GENERATION_INIT',' ={.gen=ATCPHY_GENERATION_T8122}' if generation else '',1)
 if split[-1]:
  # An ordinary port takes no SuperSpeed port down, so none may come up.
  ordinary_phy=ordinary_phy.replace(' int ret=atcphy_mux_set_locked(&ordinary_phy,APPLE_ATCPHY_MODE_OFF);',
   ' bool port_on;\n int ret=atcphy_mux_set_locked(&ordinary_phy,APPLE_ATCPHY_MODE_OFF,&port_on);\n assert(!port_on);',1)
 controls=r'''
static struct typec_mux_dev phy_provider,display_provider;
static struct typec_mux mux;
static struct typec_port class_port;
static struct tipd_data data;

static int phy_set(struct typec_mux_dev *m,struct typec_mux_state *s) { (void)m;assert(s->mode==TYPEC_STATE_SAFE);event('L');phy_calls++;if(ordinary_port){int ret=ordinary_safe();if(ret)return ret;assert(phy_off);safe_noop=true;}return phy_error; }
static int display_set(struct typec_mux_dev *m,struct typec_mux_state *s) { (void)m;(void)s;event('S');if(port.owner&&display.external&&connector.connected) {int r=dcp_dptx_disconnect_oob(display.dev,0);if(r)return r;}if(port.safe_pending)return 0;port.owner=NULL;return 0; }
static void unregister_port(struct tps6598x *t) { (void)t;unregisters++;event('U'); }
static int reset(struct tps6598x *t) { (void)t;reset_calls++;event('X');return 0; }
static void initial(struct cd321x *c) {
 count=0;events[0]=0;power_error=phy_error=tbt_error=role_error=last_error=phy_calls=reset_calls=gpio_calls=role_calls=stops=unregisters=deactivate_calls=0;
 ordinary_port=false;controller_active=true;phy_off=safe_noop=false;ordinary_initial();
 connector=(struct connector){.connected=true};display=(struct apple_dcp){.external=true,.external_cable=true,.dptxport={{.connected=true}},.typec_cable_connected=true,.typec_connector=&connector};
 port=(struct apple_dcp_typec_port){.owner=&route,.applied_valid=true,.dp_wanted=true,.dp_hpd=true,.hpd=true};route=(struct apple_dcp_typec_route){.port=&port,.dcp=&display,.selected=true};
 phy_provider=(struct typec_mux_dev){.set=phy_set};display_provider=(struct typec_mux_dev){.data=&route,.set=display_set,.prepare=dcp_typec_route_prepare,.finish=dcp_typec_route_finish,.commit=dcp_typec_route_commit};
 mux=(struct typec_mux){.context=&shared_context,.num_mux_devs=2,.mux_devs={&phy_provider,&display_provider}};class_port.mux=&mux;
 data=(struct tipd_data){.remove=cd321x_remove,.unregister_port=unregister_port,.reset=reset};
 *c=(struct cd321x){.tps={.mux=&mux,.partner=(void *)1,.port=&class_port,.data=&data,.reset=(void *)1},.mux=&mux};
}
static int position(char c) { for(unsigned i=0;i<count;i++)if(events[i]==c)return i;return -1; }
int main(int argc,char **argv) {
 (void)argc;struct cd321x c;struct typec_mux_state safe={.mode=TYPEC_STATE_SAFE};
 initial(&c);
 if(!strcmp(argv[1],"ordered")) {tipd_remove(&c.tps);fprintf(stderr,"ordered remove: %s\n",events);assert(position('Q')>=0&&position('Q')<position('L')&&position('L')<position('R'));assert(!last_error&&reset_calls==1&&gpio_calls==1&&unregisters==1&&phy_calls==1);}
 if(!strcmp(argv[1],"generic-disconnect")) {
  c.tps.role_sw=&c;
  assert(tps6598x_disconnect(&c.tps,0)==0);
  assert(position('Q')<position('L')&&position('L')<position('R'));
  assert(phy_calls==1&&!port.owner&&!mux.owner);
 }
 if(!strncmp(argv[1],"ordinary-",9)) {
  ordinary_port=true;c.tps.role_sw=&c;phy_provider.safe_after_role=true;
  if(!strcmp(argv[1],"ordinary-role-error"))role_error=-EBUSY;
  if(!strcmp(argv[1],"ordinary-phy-error"))phy_error=-ETIMEDOUT;
  if(!strcmp(argv[1],"ordinary-role-reprobe"))role_error=-EBUSY;
  if(!strncmp(argv[1],"ordinary-unsupported-",21))role_error=-EOPNOTSUPP;
  if(!strncmp(argv[1],"ordinary-unregistered-",22)) {
   role_error=-EOPNOTSUPP;assert(ordinary_power_off()==0);controller_active=false;phy_off=true;
  }
  if(!strcmp(argv[1],"ordinary-disconnect")||!strcmp(argv[1],"ordinary-unregistered-disconnect")||!strcmp(argv[1],"ordinary-unsupported-disconnect")) {
   int ret=tps6598x_disconnect(&c.tps,0);
   if(controller_active) {
    assert(ret==-EOPNOTSUPP&&phy_calls==0&&role_calls==1);
    assert(port.owner==&route&&port.safe_pending&&!mux.owner);
   } else {
    assert(ret==0);
    assert(position('Q')>=0&&position('Q')<position('R')&&position('R')<position('L'));
    assert(phy_calls==1&&safe_noop&&!port.owner&&!mux.owner);ordinary_usb2_check();
   }
  } else {
   tipd_remove(&c.tps);
   if(role_error&&(role_error!=-EOPNOTSUPP||controller_active)) {
    assert(last_error==role_error&&phy_calls==0&&role_calls==1);
    assert(port.owner==&route&&port.safe_pending&&!reset_calls&&!gpio_calls);
    if(!strcmp(argv[1],"ordinary-role-reprobe")) {
     /* Probe submits SAFE through this same mux while the PIPE is live. */
     assert(typec_mux_set(&mux,&safe)==-EBUSY);
     assert(controller_active&&!phy_off&&!safe_noop&&phy_calls==1);
     assert(port.owner==&route&&port.safe_pending&&!mux.owner);
     /* A later successful shutdown allows probe's SAFE retry to commit. */
     role_error=0;assert(usb_role_switch_set_role(c.tps.role_sw,USB_ROLE_NONE)==0);
     assert(typec_mux_set(&mux,&safe)==0&&safe_noop&&!port.owner&&!mux.owner);
    }
   } else {
    assert(position('Q')>=0&&position('Q')<position('R')&&position('R')<position('L'));
    assert(phy_calls==1&&!controller_active&&safe_noop);
    if(phy_error)assert(last_error==-ETIMEDOUT&&port.owner==&route&&port.safe_pending&&!reset_calls&&!gpio_calls);
    else {assert(!last_error&&reset_calls==1&&gpio_calls==1&&!port.owner);ordinary_usb2_check();}
   }
   assert(unregisters==1&&!mux.owner);
  }
 }
 if(!strncmp(argv[1],"noop-fixed-",11)) {
  c.tps.role_sw=&c;
  if(!strcmp(argv[1],"noop-fixed-disconnect"))assert(tps6598x_disconnect(&c.tps,0)==0);
  else tipd_remove(&c.tps);
  assert(position('Q')<position('L')&&position('L')<position('R'));
  assert(controller_active&&!phy_off&&!safe_noop&&phy_calls==1&&!port.owner&&!mux.owner);
 }
 if(argv[1][0]=='p') {power_error=-EIO;tipd_remove(&c.tps);assert(port.safe_pending&&last_error==-EIO&&phy_calls==0&&role_calls==0&&reset_calls==0&&gpio_calls==0&&port.owner==&route&&unregisters==1);}
 if(argv[1][0]=='a') {phy_error=-ETIMEDOUT;tipd_remove(&c.tps);fprintf(stderr,"failed SAFE: error=%d reset=%d events=%s\n",last_error,reset_calls,events);assert(port.safe_pending&&last_error==-ETIMEDOUT&&phy_calls==1&&reset_calls==0&&gpio_calls==0&&port.owner==&route&&unregisters==1);}
 if(argv[1][0]=='r') {role_error=-EBUSY;tipd_remove(&c.tps);assert(last_error==-EBUSY&&phy_calls==1&&reset_calls==0&&gpio_calls==0&&port.safe_pending&&port.owner==&route);}
 if(argv[1][0]=='b') {tbt_error=-EIO;tipd_remove(&c.tps);assert(last_error==-EIO&&stops==0&&phy_calls==0&&role_calls==0&&reset_calls==0);}
 if(argv[1][0]=='m') {power_error=-EIO;assert(typec_mux_set(&mux,&safe)==-EIO&&phy_calls==0&&port.owner==&route);}
 if(argv[1][0]=='d') {assert(typec_mux_prepare(&mux,&safe)==0&&stops==1&&phy_calls==0&&port.owner==&route);assert(typec_mux_prepare(&mux,&safe)==0&&stops==1&&port.owner==&route);}
 if(argv[1][0]=='e') {display.external_error=-EIO;port.hpd=false;display.typec_cable_connected=false;connector.connected=false;assert(typec_mux_set(&mux,&safe)==-EIO&&phy_calls==0&&port.owner==&route);}
 /* A latched failure is no less live once the port reads released: SAFE stays refused. */
 if(argv[1][0]=='w') {display.external_error=-ETIMEDOUT;display.dptxport[0].connected=false;display.external_stopped=true;port.hpd=false;assert(typec_mux_set(&mux,&safe)==-ETIMEDOUT&&phy_calls==0&&port.owner==&route&&deactivate_calls==0);}
 if(argv[1][0]=='l') {display.external=false;assert(typec_mux_prepare(&mux,&safe)==0&&stops==1&&port.safe_pending&&port.owner==&route);}
 if(argv[1][0]=='f') {assert(typec_mux_prepare(&mux,&safe)==0&&port.safe_pending);dcp_typec_rebalance_locked(NULL,0);assert(port.owner==&route&&route.selected&&deactivate_calls==0);port.safe_pending=false;dcp_typec_rebalance_locked(NULL,0);assert(port.owner==NULL&&deactivate_calls==1);}
 if(argv[1][0]=='t') {route.tunnel=true;assert(typec_mux_prepare(&mux,&safe)==0&&stops==0&&port.dp_wanted);}
 puts("PASS actual ordered prepare/set/remove control");return 0;
}
'''
 if 'role_switch_controls_phy' in core:
  prefix=prefix.replace('struct tipd_data {','struct tipd_data { bool role_switch_controls_phy;')
  controls=controls.replace('data=(struct tipd_data){','data=(struct tipd_data){.role_switch_controls_phy=true,')
  controls=controls.replace('c.tps.role_sw=&c;\n  assert(tps6598x_disconnect', 'c.tps.role_sw=&c;data.role_switch_controls_phy=false;\n  assert(tps6598x_disconnect')
 fabric=fabric_core(reference)
 if fabric:
  # The fabric core split: rebalancing runs the real dcp_fabric_run_rebalance()
  # and dcp_fabric_run_deactivate() over the real decisions, and the adapter
  # hands the core each route's struct dcp_fabric_route.
  prefix=prefix.replace('typedef unsigned u32;\n','typedef unsigned u32;\ntypedef unsigned long long u64;\ntypedef unsigned char u8;\n'+fabric,1)
  prefix=prefix.replace('struct apple_dcp_typec_route { ','struct apple_dcp_typec_route { struct dcp_fabric_route core; ',1)
  prefix=prefix.replace('#define lockdep_assert_held(p) ((void)(p))\n','''#define lockdep_assert_held(p) ((void)(p))
#define list_first_entry_or_null(h,t,m) ((void)(h),sole_port())
#define list_is_last(p,h) ((void)(p),(void)(h),true)
#define list_next_entry(p,m) ((void)(p),(struct apple_dcp_typec_port *)NULL)
''',1)
  # The real plan snapshots each route into its core before deciding.
  prefix=prefix.replace('static void dcp_typec_plan(struct drm_device *d,struct apple_dcp_typec_port *p,unsigned n) { (void)d;(void)p;(void)n; }',
   'static void dcp_typec_plan(struct drm_device *d,struct apple_dcp_typec_port *p,unsigned n) { (void)d;(void)p;(void)n;route.core.tunnel=route.tunnel; }',1)
 return prefix+functions+ordinary_phy+controls
