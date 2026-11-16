# SPDX-License-Identifier: GPL-2.0-only
# Copyright 2026 Ryan Murray
"""Actual PM/HCD/mux/probe boundaries using the shared PHY extraction fixture."""
from pathlib import Path
import os, re, subprocess
import phy
from common import block, tipd_pm_header, atc_function

def ownership(root, usb_unreachable=False):
    ns=phy.parking(root)
    s=ns['atc'];has_pm='static int atcphy_usb3_pm_get(' in s
    # The fixed-hub SuperSpeed handover powers the SuperSpeed root port off around
    # a change, where the PM wrapper used to resume the link on that port.
    port_power='static int atcphy_usb3_port_off(' in s
    link_resume='static int atcphy_usb3_link_get(' in s
    fields='\n    struct device_node *np;'
    if port_power:
        fields+='\n    struct delayed_work usb3_port_work, park_work;\n    unsigned int usb3_port_retries, park_retries;'
    prod=ns['production'].replace('struct apple_atcphy {','struct apple_atcphy {'+fields,1)
    if port_power:
        # The PIPE may only move while the SuperSpeed port of the held root is down.
        mux_write='if (p == (void *)&pipe_regs[PIPEHANDLER_MUX_CTRL / 4]) mux_writes++;'
        assert mux_write in prod
        prod=prod.replace(mux_write,'if (p == (void *)&pipe_regs[PIPEHANDLER_MUX_CTRL / 4]) { mux_writes++; assert(!pipe_change_window() || usb3_port_down); }',1)
        prod=r'''
struct work_struct { int unused; };
struct delayed_work { struct work_struct work; unsigned int queued; unsigned long delay; };
static bool usb3_port_down;
static bool pipe_change_window(void);
'''+prod
    if has_pm:
        prod = prod.replace(block(prod, 'static int atcphy_mux_set('), '')
    # Non-mux fixture entry points continue to exercise actual source. Only the new
    # wrapper owns PM; the fixture's original helpers call it through this declaration.
    prefix=ns['prefix']+'\nstruct device_node;\nstatic int atcphy_mux_set(struct typec_mux_dev *, struct typec_mux_state *);\n'
    prefix=prefix.replace('struct typec_mux_dev { struct apple_atcphy *data; };',
     '''struct typec_mux_dev { struct apple_atcphy *data;
 int (*set)(struct typec_mux_dev *, struct typec_mux_state *);
 int (*prepare)(struct typec_mux_dev *, struct typec_mux_state *);
 int (*finish)(struct typec_mux_dev *, struct typec_mux_state *);
 void (*commit)(struct typec_mux_dev *, struct typec_mux_state *);
 void (*abort)(struct typec_mux_dev *, struct typec_mux_state *);bool removed, transient_timeouts; };
 struct typec_mux_context { struct { bool held; } lock; };
 struct task_struct { int unused; };static struct task_struct test_task;
 #define current (&test_task)
 static int typec_mux_srcu;
 static int srcu_read_lock(int *s) { (void)s;return 0; }
 static void srcu_read_unlock(int *s,int i) { (void)s;(void)i; }
 #define mutex_trylock(m) ((m)->held ? 0 : ((m)->held=true))
 #define mutex_unlock(m) ((m)->held=false)
 struct typec_mux { unsigned int num_mux_devs; struct typec_mux_dev **mux_devs; struct typec_mux_context *context; struct typec_mux_state transaction; struct task_struct *owner; int srcu_index; unsigned prepared; bool applied, timeout_transient; };
 #define IS_ERR_OR_NULL(p) (!(p))''')
    prefix=prefix.replace('#define WARN_ON_ONCE(v) assert(!(v))', 'static bool checked_warn(bool v) { assert(!v);return v; }\n#define WARN_ON_ONCE(v) checked_warn(v)')
    prefix=prefix.replace('#define guard(kind) (void)',r'''
struct lock_guard { int *lock; };
static struct lock_guard lock_take(int *lock) { assert(!*lock); *lock=1; return (struct lock_guard){lock}; }
static void lock_release(struct lock_guard *g) { if(g->lock) { assert(*g->lock==1); *g->lock=0; } }
#define guard(kind) struct lock_guard __attribute__((cleanup(lock_release))) _guard = lock_take
#define scoped_guard(kind, ptr) for (struct lock_guard __attribute__((cleanup(lock_release))) _scope = lock_take(ptr); _scope.lock; lock_release(&_scope), _scope.lock=NULL)
''')
    # Basic USB/device APIs are modeled, but get/put/wrapper and all parking/
    # restoration bodies are extracted verbatim from the chosen public source.
    pmfixture=r'''
#define CONFIG_USB 1
#define READ_ONCE(x) (x)
#define IS_REACHABLE(x) USB_REACHABLE
struct device_node { int id; };
struct device_driver { const char *name; };
struct device { int refs; struct device *parent; struct device_driver *driver; };
struct platform_device { struct device dev; };
struct usb_device;
struct usb_hcd;
struct usb_bus { struct device *sysdev, *controller; struct usb_hcd *hcd; struct usb_device *root_hub; };
struct usb_hcd { int state; unsigned int rh_registered:1;
 struct usb_bus self; unsigned long flags;
 struct usb_hcd *shared_hcd,*primary_hcd; struct {bool pending;} died_work; };
enum { HC_STATE_RUNNING=1, HC_STATE_QUIESCING=0x85, HC_STATE_HALT=0 };
enum { HCD_FLAG_RH_RUNNING,HCD_FLAG_DEAD,HCD_FLAG_POLL_RH,HCD_FLAG_HW_ACCESSIBLE };
static int __attribute__((unused)) hcd_root_hub_lock;
#define spin_lock_irqsave(lock, flags) do { assert(!*(lock));*(lock)=1;(flags)=0; } while(0)
#define spin_unlock_irqrestore(lock, flags) do { assert(*(lock));*(lock)=0;(void)(flags); } while(0)
#define clear_bit(bit, p) (*(p)&=~(1UL<<(bit)))
#define set_bit(bit, p) (*(p)|=1UL<<(bit))
#define usb_hcd_is_primary_hcd(hcd) (!(hcd)->primary_hcd||(hcd)->primary_hcd==(hcd))
#define usb_set_device_state(u, s) ((u)->state=(s))
#define usb_kick_hub_wq(u) ((void)(u))
#define schedule_work(w) ((w)->pending=true)
#define bus_to_hcd(bus) ((bus)->hcd)
#define HCD_DEAD(hcd) (!!((hcd)->flags&(1UL<<HCD_FLAG_DEAD)))
#define HCD_HW_ACCESSIBLE(hcd) (!!((hcd)->flags&(1UL<<HCD_FLAG_HW_ACCESSIBLE)))
struct usb_device { struct usb_device *parent; int speed,state,refs,maxchild; struct usb_bus *bus; struct device dev; };
enum { USB_SPEED_SUPER=4, USB_STATE_NOTATTACHED=0 };
static struct device_node ep_node,ctrl_node;
static struct platform_device controller;
static struct device host_device;
static struct device_driver controller_driver={.name="dwc3-apple"}, host_driver={.name="xhci-hcd"};
static bool controller_locked, host_locked;
static struct device wrong_controller;
static struct usb_bus bus,wrong_bus;
static struct usb_hcd hcd;
static struct usb_device roots[4], link_device;
static bool link_present, link_suspended, link_locked;
static int link_pm_refs, link_resumes, link_error, link_detach_on_resume, link_dead_on_resume;
static int detach_at_lookup, quiesce_at_lookup;
static struct apple_atcphy *pm_phy;
static bool root_locked, graph_missing, controller_missing, root_missing;
static int node_refs, pm_refs, resumes, resume_error, detach_on_lock, detach_on_resume;
static int inaccessible_on_resume, dead_on_resume;
static void node_put(struct device_node **n) { if(*n) node_refs--; }
#define __free(x) __attribute__((cleanup(node_put)))
static struct device_node *of_graph_get_endpoint_by_regs(struct device_node *n,int p,int e) { (void)n; assert(p==1&&e==-1); if(graph_missing)return NULL;node_refs++;return &ep_node; }
static struct device_node *of_graph_get_remote_port_parent(struct device_node *n) { assert(n==&ep_node);node_refs++;return &ctrl_node; }
static struct platform_device *of_find_device_by_node(struct device_node *n) { assert(n==&ctrl_node);if(controller_missing)return NULL;controller.dev.refs++;return &controller; }
static struct device *get_device(struct device *d) { assert(d);d->refs++;return d; }
static void device_lock(struct device *d) {
 if(d==&controller.dev){assert(!controller_locked&&!host_locked&&!root_locked);controller_locked=true;}
 else {assert(d==&host_device&&controller_locked&&!host_locked&&!root_locked);host_locked=true;}
}
static void device_unlock(struct device *d) {
 if(d==&host_device){assert(host_locked&&!root_locked);host_locked=false;}
 else {assert(d==&controller.dev&&controller_locked&&!host_locked&&!root_locked);controller_locked=false;}
}
static void put_device(struct device *d) { assert(d->refs>0);d->refs--; }
static struct usb_device *usb_get_dev(struct usb_device *u) { u->refs++;return u; }
static void usb_put_dev(struct usb_device *u) { assert(u->refs>0);u->refs--; }
static int usb_for_each_dev(void *data,int (*fn)(struct usb_device *,void *)) { if(root_missing)return 0;for(unsigned int i=0;i<4;i++){int r=fn(&roots[i],data);if(r)return r;}return 0; }
/* Root state changes while the root is held: at the lookup of its child or, without one, at its resume. */
static void root_held_events(struct usb_device *u) {
 if(detach_at_lookup)usb_set_device_state(u,USB_STATE_NOTATTACHED);
 if(quiesce_at_lookup)hcd.state=HC_STATE_QUIESCING;
}
#if LINK_RESUME
static struct usb_device *usb_hub_find_child(struct usb_device *u, int port) { assert(u==&roots[3]&&root_locked&&port==1);
 root_held_events(u);
 return link_present?&link_device:NULL; }
#endif
static void usb_lock_device(struct usb_device *u) {
 if(u==&roots[3]){assert(!root_locked);if(REQUIRES_PLATFORM_LOCKS)assert(controller_locked&&host_locked);root_locked=true;if(detach_on_lock)u->state=0;}
 else {assert(u==&link_device&&root_locked&&!link_locked);link_locked=true;}
}
static void usb_unlock_device(struct usb_device *u) {
 if(u==&roots[3]){assert(root_locked&&!link_locked);root_locked=false;}
 else {assert(u==&link_device&&link_locked&&root_locked);link_locked=false;}
}
static int pm_runtime_resume_and_get(struct device *d) {
 assert(root_locked&&!pm_phy->lock);
 if(d==&link_device.dev){
  assert(link_locked);link_resumes++;
  struct lock_guard g=lock_take(&pm_phy->lock);lock_release(&g);
  if(link_error)return link_error;
  link_pm_refs++;link_suspended=false;stuck_ack=false;
  if(link_detach_on_resume)link_device.state=0;
  if(link_dead_on_resume)set_bit(HCD_FLAG_DEAD,&hcd.flags);
  return 0;
 }
 assert(d==&roots[3].dev);resumes++;
 /* A re-entrant PHY lock acquisition during resume must remain possible. */
 struct lock_guard g=lock_take(&pm_phy->lock);lock_release(&g);
 if(resume_error)return resume_error;
 pm_refs++;stuck_ack=link_present&&link_device.state!=USB_STATE_NOTATTACHED&&link_suspended;if(detach_on_resume)roots[3].state=0;
 if(!LINK_RESUME)root_held_events(&roots[3]);
 if(inaccessible_on_resume)clear_bit(HCD_FLAG_HW_ACCESSIBLE,&hcd.flags);
 if(dead_on_resume)set_bit(HCD_FLAG_DEAD,&hcd.flags);
 return 0;
}
static void pm_runtime_put_noidle(struct device *d) {
 if(d==&link_device.dev){assert(link_pm_refs==1);link_pm_refs--;}
 else {assert(d==&roots[3].dev&&pm_refs==1);pm_refs--;}
}
static void usb_mark_last_busy(struct usb_device *u) {
 assert(root_locked&&!pm_phy->lock);
 if(u==&link_device)assert(link_locked);else assert(u==&roots[3]&&!link_locked);
}
static void pm_runtime_put_autosuspend(struct device *d) {
 assert(root_locked&&!pm_phy->lock);
 if(d==&link_device.dev){assert(link_locked&&link_pm_refs==1);link_pm_refs--;}
 else {assert(d==&roots[3].dev&&!link_locked&&pm_refs==1);pm_refs--;}
}
#define dev_info(...) ((void)0)
#if PORT_POWER
#define to_delayed_work(w) container_of(w, struct delayed_work, work)
#define msecs_to_jiffies(ms) ((unsigned long)(ms))
struct workqueue_struct { int unused; };
static struct workqueue_struct freezable_wq;
#define system_freezable_wq (&freezable_wq)
static unsigned int port_offs, port_ons;
static int port_off_error, port_on_error;
static bool port_disabled;
static bool mod_delayed_work(struct workqueue_struct *wq, struct delayed_work *w, unsigned long delay)
{ assert(wq==system_freezable_wq&&!pm_phy->lock);w->queued++;w->delay=delay;return true; }
static bool pipe_change_window(void) { return root_locked; }
/* Port power of the held root; the device on it goes, and with it its hold on the PIPE. */
static int usb_hub_port_power_off(struct usb_device *u, int port)
{
 assert(u==&roots[3]&&port==1&&root_locked&&pm_refs==1&&!pm_phy->lock);
 port_offs++;
 if(port_off_error)return port_off_error;
 if(port_disabled)return 1;
 if(link_present&&link_device.state!=USB_STATE_NOTATTACHED){link_device.state=USB_STATE_NOTATTACHED;stuck_ack=false;}
 usb3_port_down=true;
 return 0;
}
/* Only once the PIPE is back on the USB3 backend of the running host. */
static int usb_hub_port_power_on(struct usb_device *u, int port)
{
 assert(u==&roots[3]&&port==1&&root_locked&&pm_refs==1&&!pm_phy->lock);
 assert(usb3_port_down&&pm_phy->host_active&&!pm_phy->mode_dirty&&!pm_phy->pipe_pending);
 assert(pm_phy->pipe_state==ATCPHY_PIPEHANDLER_STATE_USB3);
 port_ons++;
 if(port_on_error)return port_on_error;
 if(port_disabled)return 1;
 usb3_port_down=false;
 return 0;
}
#endif
'''
    suffix=ns['suffix'].replace('int main(', 'int __attribute__((unused)) fixture_main(',1)
    controls=r'''
static void prepare(struct apple_atcphy *p,bool fixed,bool t8140,bool swap)
{
    hw.has_t8140_dp=t8140;initial(p,fixed,false,swap);
    p->np=&ctrl_node;pm_phy=p;
    hcd=(struct usb_hcd){.state=HC_STATE_RUNNING,.rh_registered=true,.flags=1UL<<HCD_FLAG_HW_ACCESSIBLE};
    controller.dev.driver=&controller_driver;
    host_device=(struct device){.parent=&controller.dev,.driver=&host_driver};
    controller_locked=host_locked=false;
    bus.sysdev=&controller.dev;bus.controller=&host_device;bus.hcd=&hcd;wrong_bus.sysdev=&wrong_controller;
    roots[0]=(struct usb_device){.speed=3,.state=1,.bus=&bus};
    roots[1]=(struct usb_device){.parent=&roots[0],.speed=4,.state=1,.bus=&bus};
    roots[2]=(struct usb_device){.speed=4,.state=1,.bus=&wrong_bus};
    roots[3]=(struct usb_device){.speed=5,.state=1,.bus=&bus,.maxchild=1};
    hcd.self.root_hub=&roots[3];hcd.self.controller=&host_device;
    root_locked=graph_missing=controller_missing=root_missing=false;
    node_refs=pm_refs=resumes=resume_error=detach_on_lock=detach_on_resume=0;controller.dev.refs=0;
    inaccessible_on_resume=dead_on_resume=0;
    link_device=(struct usb_device){.parent=&roots[3],.bus=&bus,.speed=5,.state=1};
    link_present=link_suspended=link_locked=false;
    link_pm_refs=link_resumes=link_error=link_detach_on_resume=link_dead_on_resume=0;
    detach_at_lookup=quiesce_at_lookup=0;
#if PORT_POWER
    usb3_port_down=port_disabled=false;port_offs=port_ons=0;port_off_error=port_on_error=0;
#endif
}
static int safe(struct apple_atcphy *p)
{
 struct typec_mux_dev mux={p};struct typec_mux_state state={.mode=TYPEC_STATE_SAFE};
 return atcphy_mux_set(&mux,&state);
}
static void balanced(struct apple_atcphy *p)
{
 assert(!controller_locked&&!host_locked&&!host_device.refs&&!p->lock&&!root_locked&&!link_locked&&!link_pm_refs&&!link_device.refs&&!pm_refs&&!node_refs&&!controller.dev.refs);
 for(unsigned int i=0;i<4;i++)assert(!roots[i].refs);
}
int main(void)
{
 struct apple_atcphy p;
#if !USB_REACHABLE
 { prepare(&p,true,true,false);unsigned int before=writes;
 struct typec_mux_dev provider={.data=&p,.set=atcphy_mux_set};
 struct typec_mux_dev *providers[]={&provider};struct typec_mux_context typec_test_context={0};struct typec_mux mux={.num_mux_devs=1,.mux_devs=providers,.context=&typec_test_context};
 struct typec_mux_state state={.mode=TYPEC_STATE_SAFE};
 assert(typec_mux_set(&mux,&state)==-ENODEV&&writes==before);balanced(&p);
 puts("PASS USB-unreachable fixed route blocks before MMIO");return 0; }
#endif
 /* HC death before rh_registered is set need not mark the root NOTATTACHED. */
 prepare(&p,true,true,false);hcd.rh_registered=false;usb_hc_died(&hcd);
 assert(roots[3].state!=USB_STATE_NOTATTACHED&&HCD_DEAD(&hcd));unsigned int before_hcd=writes;
 int hcd_ret=safe(&p);fprintf(stderr,"dead HCD admission returned %d, PM resumes %d\n",hcd_ret,resumes);
 assert(hcd_ret==-ENODEV&&writes==before_hcd&&resumes==0);balanced(&p);
 if(HCD_DEAD_ONLY){puts("PASS dead HCD rejected before PM/MMIO");return 0;}
 for(unsigned int gate=0;gate<2;gate++){
  prepare(&p,true,true,false);before_hcd=writes;
  if(gate==0)inaccessible_on_resume=1;
  if(gate==1)dead_on_resume=1;
  assert(safe(&p)==-ENODEV&&writes==before_hcd);balanced(&p);
 }
 for(unsigned int swap=0;swap<2;swap++){
   prepare(&p,true,true,swap);stuck_ack=true;
   assert(safe(&p)==0);assert(resumes==1);balanced(&p);
 }
 /* Resume failure must return before ANY lane/core programming. */
 prepare(&p,true,true,false);unsigned int before=writes;resume_error=-EIO;
 unsigned int valid=p.dp_usb_context_valid,dirty=p.dp_usb_context_dirty,owned=p.dp_owned_pmas;
 assert(safe(&p)==-EIO&&writes==before&&p.dp_usb_context_valid==valid&&p.dp_usb_context_dirty==dirty&&p.dp_owned_pmas==owned);balanced(&p);
 resume_error=0;assert(safe(&p)==0);balanced(&p);
 /* PM resume is NOT proof of ACK. Real parking timeout stays fatal. */
 prepare(&p,true,true,false);fail_caller="atcphy_pipehandler_lock";before=writes;
 assert(safe(&p)==-ETIMEDOUT&&writes==before&&p.pipe_state==ATCPHY_PIPEHANDLER_STATE_USB3&&p.mode_dirty);balanced(&p);
 fail_caller=NULL;assert(safe(&p)==0);balanced(&p);
 for(unsigned int missing=0;missing<5;missing++){
  prepare(&p,true,true,false);before=writes;
  if(missing==0)graph_missing=true;
  if(missing==1)controller_missing=true;
  if(missing==2)root_missing=true;
  if(missing==3)detach_on_lock=1;
  if(missing==4)detach_on_resume=1;
  assert(safe(&p)==(missing==0||missing>=3?-ENODEV:-EAGAIN)&&writes==before);balanced(&p);
 }
 /* Root appears after independent fixed-host probe: missing root is retryable. */
 prepare(&p,true,true,false);root_missing=true;before=writes;
 assert(safe(&p)==-EAGAIN&&writes==before);balanced(&p);
 root_missing=false;assert(safe(&p)==0);balanced(&p);
 /* BIST faults are independent of the runtime clock wake; never force readiness. */
 for(unsigned int swap=0;swap<2;swap++){
   prepare(&p,true,true,swap);assert(safe(&p)==0);balanced(&p);
   orient(&p,!swap);bist_status_stuck=true;
   assert(select_mode(&p,APPLE_ATCPHY_MODE_USB3_DP)==-ETIMEDOUT&&p.mode_dirty&&p.pipe_pending);balanced(&p);
   bist_status_stuck=false;recover();assert(select_mode(&p,APPLE_ATCPHY_MODE_USB3_DP)==0);balanced(&p);
 }
 /* Clean same-mode updates bypass PM; dirty lanes or PIPE retry still need it. */
 struct typec_altmode alt={.svid=USB_TYPEC_DP_SID};
 struct typec_mux_dev mux={&p};struct typec_mux_state same={.mode=TYPEC_DP_STATE_D,.alt=&alt};
 prepare(&p,false,true,false);before=writes;assert(atcphy_mux_set(&mux,&same)==0&&resumes==0&&writes==before);balanced(&p);
 prepare(&p,true,false,false);before=writes;assert(atcphy_mux_set(&mux,&same)==0&&resumes==0&&writes==before);balanced(&p);
 prepare(&p,true,true,false);before=writes;
 root_missing=true;resume_error=-EIO;
 assert(atcphy_mux_set(&mux,&same)==0&&writes==before&&resumes==0);balanced(&p);
 for(unsigned int retry=0;retry<3;retry++){
  prepare(&p,true,true,false);before=writes;root_missing=true;
  if(retry==0)p.mode_dirty=true;
  if(retry==1)p.pipe_pending=true;
  if(retry==2)p.pipe_state=ATCPHY_PIPEHANDLER_STATE_DUMMY;
  assert(atcphy_mux_set(&mux,&same)==-EAGAIN&&writes==before&&resumes==0);balanced(&p);
 }
 /* Binding/identity failure must not reach USB PM or PHY programming. */
 for(unsigned int missing=0;missing<3;missing++){
  prepare(&p,true,true,false);before=writes;
  if(missing==0)controller.dev.driver=NULL;
  if(missing==1)host_device.driver=NULL;
  if(missing==2)host_device.parent=&wrong_controller;
  assert(safe(&p)==(missing==2?-ENODEV:-EAGAIN)&&writes==before&&resumes==0);balanced(&p);
 }
 /* Root state can change during lookup despite USB device locks. */
 for(unsigned int event=0;event<2;event++){
  prepare(&p,true,true,false);before=writes;
  if(event==0)detach_at_lookup=1;else quiesce_at_lookup=1;
  int ret=safe(&p);fprintf(stderr,"post-child admission event %u returned %d writes %u\n",event,ret,writes-before);
  assert(ret==-ENODEV&&writes==before);balanced(&p);
 }
#if LINK_RESUME
 /* Selectively suspended upstream link is not resumed by root ownership alone. */
 for(unsigned int swap=0;swap<2;swap++){
  prepare(&p,true,true,swap);link_present=link_suspended=stuck_ack=true;
  int link_ret=safe(&p);fprintf(stderr,"selective link SAFE returned %d, root resumes %d, link resumes %d\n",link_ret,resumes,link_resumes);
  assert(link_ret==0&&resumes==1&&link_resumes==1&&!link_suspended);balanced(&p);
 }
 for(unsigned int fault=0;fault<3;fault++){
  prepare(&p,true,true,false);link_present=link_suspended=true;before=writes;
  unsigned int retained=p.dp_usb_context_valid,owners=p.dp_owned_pmas;
  if(fault==0)link_error=-EIO;
  if(fault==1)link_detach_on_resume=1;
  if(fault==2)link_dead_on_resume=1;
  assert(safe(&p)==(fault?-ENODEV:-EIO)&&writes==before&&p.dp_usb_context_valid==retained&&p.dp_owned_pmas==owners);balanced(&p);
 }
 prepare(&p,true,true,false);link_present=link_suspended=true;link_error=-EIO;
 assert(safe(&p)==-EIO);balanced(&p);link_error=0;assert(safe(&p)==0);balanced(&p);
 prepare(&p,true,true,false);link_present=link_suspended=true;fail_caller="atcphy_pipehandler_lock";before=writes;
 assert(safe(&p)==-ETIMEDOUT&&writes==before&&p.mode_dirty&&p.pipe_state==ATCPHY_PIPEHANDLER_STATE_USB3);balanced(&p);
 fail_caller=NULL;assert(safe(&p)==0);balanced(&p);
#else
 /* The device on the SuperSpeed port goes with the port, before the PIPE moves. */
 for(unsigned int swap=0;swap<2;swap++){
  prepare(&p,true,true,swap);link_present=link_suspended=stuck_ack=true;
  int link_ret=safe(&p);fprintf(stderr,"port-off SAFE returned %d, root resumes %d, port offs %u\n",link_ret,resumes,port_offs);
  assert(link_ret==0&&resumes==1&&link_resumes==0&&link_device.state==USB_STATE_NOTATTACHED);
  /* SAFE has no USB3: the port stays off until a USB3 mode has the PIPE back. */
  assert(port_offs==1&&!port_ons&&usb3_port_down&&p.usb3_port_off);balanced(&p);
  assert(select_mode(&p,APPLE_ATCPHY_MODE_USB3_DP)==0&&p.pipe_state==ATCPHY_PIPEHANDLER_STATE_USB3);
  assert(port_offs==2&&port_ons==1&&!usb3_port_down&&!p.usb3_port_off);balanced(&p);
 }
 /* A port that does not go down stops the change before any PHY programming. */
 prepare(&p,true,true,false);link_present=link_suspended=true;port_off_error=-EIO;before=writes;
 unsigned int retained=p.dp_usb_context_valid,owners=p.dp_owned_pmas;
 assert(safe(&p)==-EIO&&writes==before&&p.dp_usb_context_valid==retained&&p.dp_owned_pmas==owners);
 assert(!port_ons&&!usb3_port_down&&!p.usb3_port_off);balanced(&p);
 port_off_error=0;assert(safe(&p)==0&&usb3_port_down);balanced(&p);
 /* A port that someone else switched off stays theirs. */
 prepare(&p,true,true,false);port_disabled=usb3_port_down=true;
 assert(safe(&p)==0&&port_offs==1&&!p.usb3_port_off);balanced(&p);
 assert(select_mode(&p,APPLE_ATCPHY_MODE_USB3_DP)==0&&!port_ons&&usb3_port_down);balanced(&p);
 /* A port that does not come back fails the change; the same mode then only powers it on. */
 prepare(&p,true,true,false);assert(safe(&p)==0);port_on_error=-EIO;
 assert(select_mode(&p,APPLE_ATCPHY_MODE_USB3_DP)==-EIO&&port_ons==1&&p.usb3_port_off&&p.usb3_port_work.queued==1);
 balanced(&p);port_on_error=0;before=writes;
 assert(select_mode(&p,APPLE_ATCPHY_MODE_USB3_DP)==0&&port_ons==2&&!p.usb3_port_off&&writes==before);balanced(&p);
 /* PM resume is NOT proof of ACK; the port is down and stays down. */
 prepare(&p,true,true,false);link_present=link_suspended=true;fail_caller="atcphy_pipehandler_lock";before=writes;
 assert(safe(&p)==-ETIMEDOUT&&writes==before&&p.mode_dirty&&p.pipe_state==ATCPHY_PIPEHANDLER_STATE_USB3);
 assert(usb3_port_down&&p.usb3_port_off&&!port_ons);balanced(&p);
 fail_caller=NULL;assert(safe(&p)==0);balanced(&p);
#endif
 for(unsigned int count=0;count<2;count++){
  prepare(&p,true,true,false);roots[3].maxchild=count?2:0;before=writes;
  assert(safe(&p)==(count?-ENODEV:-EAGAIN)&&writes==before);balanced(&p);
 }
 prepare(&p,true,true,false);link_present=true;link_device.state=USB_STATE_NOTATTACHED;
 assert(safe(&p)==0&&link_resumes==0);balanced(&p);
 /* Startup with no old child is allowed; no fake dock or link is required. */
 prepare(&p,true,true,false);assert(safe(&p)==0&&link_resumes==0);balanced(&p);
 puts("PASS actual upstream-link ownership: both PMAs, selective suspend, resume/removal/death/ACK failures, retry, unknown port map and no-child startup");
 puts("PASS actual PM wrapper + actual parking/restore: both PMAs, gated clock, PM error/retry, fatal ACK/retry, graph/controller/root/removal gates, legacy/ordinary and cached mode");
 return 0;
}
'''
    helpers=''
    if has_pm:
     helpers=ns['block'](s,'struct atcphy_usb3_pm {')+';\n'
     helpers+='#if IS_REACHABLE(CONFIG_USB)\n'
     if link_resume:
      helpers+=ns['function'](s,'atcphy_usb3_link_get')+'\n'+ns['function'](s,'atcphy_usb3_link_put')+'\n'
     helpers+=atc_function(s,'atcphy_usb3_is_root')
     helpers+=ns['function'](s,'atcphy_usb3_pm_find')+'\n#endif\n'
     helpers+=ns['function'](s,'atcphy_usb3_pm_get')+'\n'+ns['function'](s,'atcphy_usb3_pm_put')+'\n'
     # The port power steps and the retries that the mux update queues; the
     # delayed work functions themselves are left to the KUnit tests.
     helpers+=''.join(atc_function(s,name) for name in
       ('atcphy_usb3_port_off','atcphy_usb3_port_on','atcphy_usb3_port_retry_delay',
        'atcphy_usb3_port_retry','atcphy_mux_in_mode'))
     helpers+=ns['function'](s,'atcphy_mux_is_noop')+'\n'
     helpers+=''.join(atc_function(s,name) for name in
       ('atcphy_park_retry_delay','atcphy_park_retry','atcphy_park_due','atcphy_mux_change'))
     helpers+=ns['function'](s,'atcphy_mux_update')+'\n'
     # Lane fixtures provide the target mapping and locked body; use the real callback here.
     marker=re.search(r'^static int atcphy_mux_set\(',s,re.M).group()
     helpers+=ns['block'](s,marker)+'\n'
    else:
     # Parent runs its ACTUAL mux callback from production, without a substituted wrapper.
     pmfixture=pmfixture.replace('static ', 'static __attribute__((unused)) ')
    if usb_unreachable:
     pmfixture=pmfixture.replace('static ', 'static __attribute__((unused)) ')
    typec_source=(root/'drivers/usb/typec/mux.c').read_text()
    typec_caller='\n'.join(ns['block'](typec_source, marker) for marker in
      ('static void typec_mux_unlock(', 'void typec_mux_abort(', 'int typec_mux_begin(', 'int typec_mux_apply(', 'int typec_mux_end(', 'int typec_mux_set('))+'\n'
    hcd_source=(root/'drivers/usb/core/hcd.c').read_text()
    hcd_death=ns['block'](hcd_source,'void usb_hc_died (')+'\n'
    text=prefix+prod+'\n#define REQUIRES_PLATFORM_LOCKS '+str(int('device_lock(&pm->controller->dev)' in s))+'\n#define PORT_POWER '+str(int(port_power))+'\n#define LINK_RESUME '+str(int(link_resume))+'\n#define USB_REACHABLE '+str(int(not usb_unreachable))+'\n#define HCD_DEAD_ONLY '+str(int(False))+'\n'+pmfixture+helpers+typec_caller+hcd_death+suffix+controls
    return text

def probe(root):
    def source(path):
        return (root/path).read_text()


    def function(text, name):
        match = re.search(r'^static (?:int|void)\s*\n?' + name + r'\(', text, re.M)
        assert match, name
        start = match.start()
        opening = text.index('{', start)
        depth = 1
        end = opening + 1
        while depth:
            depth += (text[end] == '{') - (text[end] == '}')
            end += 1
        return text[start:end]


    prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
#include <string.h>
#define EPROBE_DEFER 517
#define IS_ERR(p) ((intptr_t)(p)<0)
#define ERR_PTR(e) ((void *)(intptr_t)(e))
#define dev_info(...) ((void)0)
#define PTR_ERR(p) ((int)(intptr_t)(p))
#define IS_ERR_OR_NULL(p) (!(p)||IS_ERR(p))
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define guard(kind) (void)
#define TYPEC_STATE_SAFE 0
#define INIT_DELAYED_WORK(p,fn) ((p)->pending=false)
#define atomic_set(p,n) (*(p)=(n))
struct work { bool pending; };
struct fwnode_handle { int refs; };
struct typec_mux_state { void *alt,*data; unsigned long mode; };
struct device;
struct tps6598x { int lock; void *port; struct device *dev; };
struct notifier_block { int (*notifier_call)(struct notifier_block *,unsigned long,void *); };
'''
    pm = tipd_pm_header(root)
    mocks = r'''
struct cd321x {
 struct tps6598x tps;
 struct work update_work,resume_work;
 int link_event,ready_event,restore_event;
 struct cd321x_pm_state pm;
 void *mux,*tbt_switch,*port_altmode_dp,*port_altmode_tbt;
 struct fwnode_handle *connector_fwnode;
 struct typec_mux_state state;
 struct notifier_block tbt_notifier;
 bool state_valid,tbt_notifier_registered;
};
static struct fwnode_handle connector;
static int mux_error, notifier_error, switch_error;
static int ports,altmodes,muxes,switches,notifiers,stop_calls;
struct typec_thunderbolt_switch;
static bool keep_waiting=true;
static bool cd321x_tbt_switch_keep_waiting(struct device *d) { (void)d;return keep_waiting; }
static int tps6598x_register_port(struct tps6598x *t,struct fwnode_handle *f) {
 (void)f;t->port=t;ports++;return 0;
}
static int cd321x_register_port_altmodes(struct cd321x *c) {
 c->port_altmode_dp=c;c->port_altmode_tbt=c;altmodes+=2;return 0;
}
static void *fwnode_typec_mux_get(struct fwnode_handle *f) { (void)f;muxes++;return &muxes; }
static struct fwnode_handle *fwnode_find_reference(struct fwnode_handle *f,const char *s,int n) {
 (void)f;(void)s;(void)n;connector.refs++;return &connector;
}
static void __attribute__((unused)) fwnode_handle_put(struct fwnode_handle *f) {
 if(f){assert(f->refs>0);f->refs--;}
}
static void *fwnode_typec_thunderbolt_switch_get(struct fwnode_handle *f) {
 (void)f;if(switch_error)return (void *)(intptr_t)switch_error;switches++;return &switches;
}
static int typec_set_mode(void *p,unsigned long m) { (void)p;assert(m==TYPEC_STATE_SAFE);return mux_error; }
static int cd321x_tbt_notify(struct notifier_block *n,unsigned long e,void *d) { (void)n;(void)e;(void)d;return 0; }
static int typec_thunderbolt_switch_register_notifier(void *s,struct notifier_block *n) {
 (void)n;if(!s)return 0;if(notifier_error)return notifier_error;notifiers++;return 0;
}
static void typec_thunderbolt_switch_put(void *s) { if(s){assert(switches>0);switches--;} }
static void typec_mux_put(void *m) { assert(m&&muxes>0);muxes--; }
static void typec_unregister_altmode(void *a) { if(a){assert(altmodes>0);altmodes--;} }
static void typec_unregister_port(void *p) { assert(p&&ports>0);ports--; }
static void cd321x_stop_work(struct cd321x *c) {
 stop_calls++;if(c->tbt_notifier_registered){if(c->tbt_switch)notifiers--;c->tbt_notifier_registered=false;}
 cd321x_pm_remove(&c->pm);c->resume_work.pending=c->update_work.pending=false;
}
'''
    core = source('drivers/usb/typec/tipd/core.c')
    switch_get = block(core, 'VISIBLE_IF_KUNIT struct typec_thunderbolt_switch *\ncd321x_tbt_switch_get(')
    production = switch_get.replace('VISIBLE_IF_KUNIT', 'static', 1) + '\n'
    init_work = block(core, 'VISIBLE_IF_KUNIT void cd321x_init_work(')
    production += init_work.replace('VISIBLE_IF_KUNIT', 'static', 1) + '\n'
    production += function(core, 'cd321x_register_port') + '\n' + function(core, 'cd321x_unregister_port')
    controls = r'''
static void reset(struct cd321x *c) {
 memset(c,0,sizeof(*c));connector.refs=0;
 mux_error=notifier_error=switch_error=0;stop_calls=0;keep_waiting=true;
 assert(!ports&&!altmodes&&!muxes&&!switches&&!notifiers);
}
static void balanced(void) {
 assert(!ports&&!altmodes&&!muxes&&!switches&&!notifiers&&!connector.refs);
}
int main(void) {
 (void)cd321x_pm_notifications;
 struct cd321x c;
 struct cd321x_pm_state budget={0};
 cd321x_pm_init(&budget);assert(cd321x_pm_ready(&budget));
 assert(cd321x_display_taken(-ENOSPC,true)&&!cd321x_display_taken(-EBUSY,true)&&!cd321x_display_taken(0,true)&&cd321x_display_taken(-EBUSY,false)&&!cd321x_display_taken(-ENOSPC,false)&&!cd321x_display_taken(0,false));
 for(unsigned int i=0;i<CD321X_RESUME_ATTEMPTS;i++){
  assert(cd321x_pm_begin_read(&budget));cd321x_pm_snapshot_ready(&budget);
  assert(cd321x_pm_retry(&budget,false)==(i+1<CD321X_RESUME_ATTEMPTS));
 }
 assert(budget.phase==CD321X_PM_STALE&&!cd321x_pm_can_update(&budget));
 /* Root appearance does not itself deliver a Type-C event. */
 assert(!cd321x_pm_begin_read(&budget));
 assert(cd321x_pm_event(&budget)&&cd321x_pm_begin_read(&budget));
 cd321x_pm_snapshot_ready(&budget);cd321x_pm_complete(&budget);
 assert(cd321x_pm_can_update(&budget));cd321x_pm_prepare(&budget);
 assert(!cd321x_pm_can_update(&budget)&&cd321x_pm_resume(&budget));
 assert(cd321x_pm_link_event(&budget));
 puts("PASS actual retry budget becomes stale without a fresh event");
 struct cd321x_pm_state lost={0};
 cd321x_pm_init(&lost);assert(!cd321x_pm_event_lost(&lost)&&cd321x_pm_ready(&lost));
 for(unsigned int i=0;i<CD321X_RESUME_ATTEMPTS;i++){
  assert(cd321x_pm_begin_read(&lost));cd321x_pm_snapshot_ready(&lost);cd321x_pm_retry(&lost,false);
 }
 /* Latched events raise no interrupt, so even a stale port must read them. */
 assert(lost.phase==CD321X_PM_STALE&&cd321x_pm_event_lost(&lost));
 for(unsigned int i=0,delay=0;i<4*CD321X_RESUME_ATTEMPTS;i++){
  unsigned int expected=i?(2*delay<CD321X_ACK_RETRY_MAX_MS?2*delay:CD321X_ACK_RETRY_MAX_MS):CD321X_ACK_RETRY_MS;
  assert(cd321x_pm_begin_read(&lost));delay=cd321x_pm_ack_failed(&lost);
  assert(delay==expected&&lost.attempts_left==CD321X_RESUME_ATTEMPTS&&!cd321x_pm_can_update(&lost));
 }
 assert(lost.ack_retry_ms==CD321X_ACK_RETRY_MAX_MS&&cd321x_pm_acked(&lost)&&!cd321x_pm_acked(&lost));
 cd321x_pm_prepare(&lost);assert(!cd321x_pm_event_lost(&lost)&&lost.phase==CD321X_PM_PREPARED);
 cd321x_pm_remove(&lost);assert(!cd321x_pm_event_lost(&lost)&&lost.phase==CD321X_PM_REMOVED);
 puts("PASS actual lost events reopen a stale port; unacknowledged events keep their budget with capped backoff");
 if(TEST_DEFER){
  reset(&c);mux_error=-EAGAIN;
  int ret=cd321x_register_port(&c.tps,&connector);
  fprintf(stderr,"missing root probe returned %d (expected -EPROBE_DEFER)\n",ret);
  assert(ret==-EPROBE_DEFER&&!c.state_valid&&!notifiers&&!c.resume_work.pending);
  balanced();assert(!stop_calls);
  /* Supplier completion reprobes the consumer, with a fresh retry budget. */
  reset(&c);assert(cd321x_register_port(&c.tps,&connector)==0&&c.state_valid);
  assert(cd321x_pm_ready(&c.pm)&&cd321x_pm_begin_read(&c.pm));
  cd321x_pm_snapshot_ready(&c.pm);cd321x_pm_complete(&c.pm);
  assert(c.pm.phase==CD321X_PM_RUNNING);
  cd321x_unregister_port(&c.tps);balanced();
 }
 if(TEST_FWNODE){
  reset(&c);notifier_error=-ENOMEM;
  assert(cd321x_register_port(&c.tps,&connector)==-ENOMEM);balanced();
  reset(&c);switch_error=-ENODEV;
  assert(cd321x_register_port(&c.tps,&connector)==-ENODEV);balanced();
 }
 /* Unrelated permanent errors retain the existing charging-port probe behavior. */
 reset(&c);mux_error=-EIO;
 assert(cd321x_register_port(&c.tps,&connector)==0&&!c.state_valid);
 cd321x_unregister_port(&c.tps);balanced();
 /* Missing router defers while suppliers may still bind, then is optional. */
 reset(&c);switch_error=-EPROBE_DEFER;
 assert(cd321x_register_port(&c.tps,&connector)==-EPROBE_DEFER);balanced();
 reset(&c);switch_error=-EPROBE_DEFER;keep_waiting=false;
 assert(cd321x_register_port(&c.tps,&connector)==0&&c.state_valid&&!c.tbt_switch);
 cd321x_unregister_port(&c.tps);balanced();
 puts("PASS actual cd321x port: mux defer/reprobe, rollback, fwnode balance and ordinary scope");
}
'''

    # Newer trees report an occupied pipeline as -ENOSPC on every SoC.
    # Keep the old two-argument contract covered when testing an older tree.
    if re.search(r'cd321x_display_taken\(int ret\)', pm):
        old = ' assert(cd321x_display_taken(-ENOSPC,true)&&!cd321x_display_taken(-EBUSY,true)&&!cd321x_display_taken(0,true)&&cd321x_display_taken(-EBUSY,false)&&!cd321x_display_taken(-ENOSPC,false)&&!cd321x_display_taken(0,false));'
        assert controls.count(old) == 1
        controls = controls.replace(old, r'''
 /* An occupied display permits USB progress; transient/other errors retry. */
 assert(cd321x_display_taken(-ENOSPC));
 assert(!cd321x_display_taken(-EBUSY)&&!cd321x_display_taken(-EAGAIN));
 assert(!cd321x_display_taken(-ETIMEDOUT)&&!cd321x_display_taken(-EIO));
 assert(!cd321x_display_taken(0));
''')

    # A mux that holds a change up is retried outside the cable-state budget.
    if 'cd321x_pm_mux_held(' in pm:
        lost = ' puts("PASS actual lost events reopen a stale port; unacknowledged events keep their budget with capped backoff");\n'
        assert controls.count(lost) == 1
        controls = controls.replace(lost, lost + r'''/* The time of the failure, once the limit of the retries is on time */
#ifdef CD321X_MUX_HELD_MS
#define MUX_HELD(pm,ms) cd321x_pm_mux_held(pm,ms)
#else
#define MUX_HELD(pm,ms) ((void)(ms),cd321x_pm_mux_held(pm))
#endif
 struct cd321x_pm_state held={0};
 held.phase=CD321X_PM_RUNNING;
 for(unsigned int i=0,delay=0,now=0;i<4*CD321X_RESUME_ATTEMPTS;i++,now+=delay){
  unsigned int expected=i?(2*delay<CD321X_MUX_RETRY_MAX_MS?2*delay:CD321X_MUX_RETRY_MAX_MS):CD321X_MUX_RETRY_MS;
  delay=MUX_HELD(&held,now);
  assert(delay==expected&&held.attempts_left==CD321X_RESUME_ATTEMPTS&&held.force_reconnect&&!cd321x_pm_can_update(&held));
  /* An event brings the retry forward, but not a read already under way. */
  assert(cd321x_pm_mux_event(&held)&&cd321x_pm_begin_read(&held));
  cd321x_pm_snapshot_ready(&held);assert(!cd321x_pm_mux_event(&held));
 }
 assert(held.mux_retry_ms==CD321X_MUX_RETRY_MAX_MS&&cd321x_pm_mux_done(&held)&&!cd321x_pm_mux_done(&held));
 cd321x_pm_complete(&held);assert(held.phase==CD321X_PM_RUNNING&&!held.force_reconnect);
 MUX_HELD(&held,0);cd321x_pm_prepare(&held);assert(!held.mux_retry_ms&&!cd321x_pm_mux_event(&held));
 puts("PASS actual held mux keeps the cable-state budget with capped backoff");
''')
    # ...for a bounded number of retries, and any other failure ends the hold-up.
    if 'CD321X_MUX_RETRIES' in pm:
        held = ' puts("PASS actual held mux keeps the cable-state budget with capped backoff");\n'
        assert controls.count(held) == 1
        controls = controls.replace(held, held + r''' struct cd321x_pm_state stuck={0};
 stuck.phase=CD321X_PM_RUNNING;
 for(unsigned int i=0;i<CD321X_MUX_RETRIES;i++){
  assert(MUX_HELD(&stuck,0)&&cd321x_pm_begin_read(&stuck));cd321x_pm_snapshot_ready(&stuck);
 }
 assert(!MUX_HELD(&stuck,0)&&stuck.phase==CD321X_PM_STALE&&stuck.force_reconnect);
 assert(!stuck.mux_retry_ms&&!stuck.mux_retries&&!cd321x_pm_mux_event(&stuck)&&!cd321x_pm_begin_read(&stuck));
 assert(cd321x_pm_event(&stuck)&&cd321x_pm_begin_read(&stuck));cd321x_pm_snapshot_ready(&stuck);
 assert(MUX_HELD(&stuck,0)==CD321X_MUX_RETRY_MS);
 assert(cd321x_pm_begin_read(&stuck));cd321x_pm_snapshot_ready(&stuck);
 assert(cd321x_pm_retry(&stuck,true)&&!stuck.mux_retry_ms&&!stuck.mux_retries&&!cd321x_pm_mux_event(&stuck));
 MUX_HELD(&stuck,0);cd321x_pm_remove(&stuck);assert(!stuck.mux_retry_ms&&!stuck.mux_retries);
#ifdef CD321X_MUX_HELD_MS
 /* Retries that events bring forward, a second apart, do not end it sooner. */
 struct cd321x_pm_state brought={0};
 brought.phase=CD321X_PM_RUNNING;
 for(unsigned int now=0;now<CD321X_MUX_HELD_MS;now+=1000){
  assert(MUX_HELD(&brought,now)&&cd321x_pm_begin_read(&brought));cd321x_pm_snapshot_ready(&brought);
 }
 assert(!MUX_HELD(&brought,CD321X_MUX_HELD_MS)&&brought.phase==CD321X_PM_STALE&&!brought.mux_retry_ms);
#endif
 puts("PASS actual held mux gives up after its retries until an event; other failures end the hold-up");
''')

    return prefix + pm + mocks + production + '\n#define TEST_DEFER 1\n#define TEST_FWNODE 1\n' + controls


def kconfig(root, output):
    """Run the actual optional USB dependency through a real Kconfig producer."""
    source = (root / 'drivers/phy/apple/Kconfig').read_text()
    dependency = next(line.strip() for line in source.splitlines()
                      if line.strip() == 'depends on USB || !USB')
    directory = output / 'usb-kconfig'
    directory.mkdir(parents=True, exist_ok=True)
    kconfig = directory / 'Kconfig'
    kconfig.write_text('config MODULES\n\tbool "modules"\n\tmodules\n\tdefault y\n'
                      'config USB\n\ttristate "USB"\n'
                      'config PHY_APPLE_ATC\n\ttristate "PHY"\n\t' + dependency + '\n')
    compiler = Path(os.environ.get('KCONFIG_CONF', root / 'scripts/kconfig/conf'))
    if not compiler.is_file():
        subprocess.run(['make', '-C', str(root), 'O=' + str(directory / 'producer'),
                        'ARCH=arm64', 'LLVM=1', 'allnoconfig'], check=True)
        compiler = directory / 'producer/scripts/kconfig/conf'
    for usb, phy_setting in [('n', 'y'), ('y', 'y'), ('m', 'y'), ('m', 'm')]:
        case = directory / (usb + '-' + phy_setting)
        case.mkdir(exist_ok=True)
        config = case / 'config'
        config.write_text('CONFIG_MODULES=y\n' +
                          ('CONFIG_USB=' + usb + '\n' if usb != 'n' else
                           '# CONFIG_USB is not set\n') + 'CONFIG_PHY_APPLE_ATC=' + phy_setting + '\n')
        env = dict(os.environ, KCONFIG_CONFIG=str(config),
                   KCONFIG_AUTOCONFIG=str(case / 'auto.conf'),
                   KCONFIG_AUTOHEADER=str(case / 'autoconf.h'))
        result = subprocess.run([str(compiler.resolve()), '--olddefconfig', str(kconfig)],
                                cwd=case, env=env, capture_output=True, text=True)
        (case / 'run.log').write_text(result.stdout + result.stderr)
        result.check_returncode()
        expected = 'm' if usb == 'm' else 'y'
        assert 'CONFIG_PHY_APPLE_ATC=' + expected in config.read_text()


def probe_dispatch(root):
    """Connect real port probe -> typec_set_mode -> mux core -> PM wrapper."""
    unit = ownership(root)
    unit = unit.replace('int main(void)', 'static int __attribute__((unused)) ownership_fixture_main(void)', 1)
    unit = unit.replace('#define IS_ERR_OR_NULL(p) (!(p))', '#define IS_ERR(p) ((intptr_t)(p)<0)\n #define IS_ERR_OR_NULL(p) (!(p)||IS_ERR(p))')
    port = probe(root)
    port = port[:port.index('static void reset(struct cd321x *c)')]
    # Type-C and mutex declarations already belong to the shared PM fixture.
    port = port.replace('struct typec_mux_state { void *alt,*data; unsigned long mode; };', '')
    port = port.replace('#define guard(kind) (void)', '')
    port = port.replace('static int typec_set_mode(void *p,unsigned long m) { (void)p;assert(m==TYPEC_STATE_SAFE);return mux_error; }', '')
    port = port.replace('(void)f;t->port=t;ports++;return 0;', '(void)f;t->port=&probe_port;ports++;return 0;')
    # Real class dispatch requires the port's mux; its allocator remains a mock.
    declarations = 'struct typec_port { struct typec_mux *mux; };\nstatic struct typec_port probe_port;\n'
    dispatch = block((root / 'drivers/usb/typec/class.c').read_text(), 'int typec_set_mode(')
    marker = port.index('static int tps6598x_register_port(')
    port = port[:marker] + declarations + dispatch + '\n' + port[marker:]
    import re
    # Only helpers used at this boundary belong in this translation unit;
    # the separate probe group exercises the complete retry state machine.
    used = {'cd321x_pm_new_connection', 'cd321x_pm_init', 'cd321x_pm_remove', 'cd321x_pm_ready',
            'cd321x_pm_begin_read', 'cd321x_pm_snapshot_ready',
            'cd321x_pm_complete', 'cd321x_pm_can_update',
            # which, once retries of a held mux exist, ends them on removal
            'cd321x_pm_mux_done'}
    for match in list(re.finditer(r'^static inline (?:void|bool|unsigned int) (cd321x_\w+)\(', port, re.M))[::-1]:
        if match.group(1) not in used:
            body = block(port, match.group())
            port = port[:match.start()] + port[match.start() + len(body):]
    port = port.replace('static int mux_error, notifier_error, switch_error;',
                        'static int notifier_error, switch_error;')
    existing = set(re.findall(r'^\s*#define (\w+)', unit, re.M))
    port = '\n'.join(line for line in port.splitlines()
                     if not (line.startswith('#define ') and line.split()[1].split('(')[0] in existing))
    controls = r"""
static void port_balance(void)
{
    assert(!ports&&!altmodes&&!muxes&&!switches&&!notifiers&&!connector.refs);
}
int main(void)
{
    struct apple_atcphy p;
    struct cd321x c={};
    struct typec_mux_dev provider={.data=&p,.set=atcphy_mux_set};
    struct typec_mux_dev *providers[]={&provider};
    struct typec_mux_context typec_test_context={0};struct typec_mux route={.num_mux_devs=1,.mux_devs=providers,.context=&typec_test_context};probe_port.mux=&route;
    prepare(&p,true,true,false);root_missing=true;unsigned int before=writes;
    assert(cd321x_register_port(&c.tps,&connector)==-EPROBE_DEFER);
    assert(!c.state_valid&&!resumes&&writes==before);balanced(&p);port_balance();
    memset(&c,0,sizeof(c));root_missing=false;
    assert(cd321x_register_port(&c.tps,&connector)==0 && c.state_valid && resumes==1);
    assert(cd321x_pm_ready(&c.pm) && cd321x_pm_begin_read(&c.pm));
    cd321x_pm_snapshot_ready(&c.pm);cd321x_pm_complete(&c.pm);
    assert(cd321x_pm_can_update(&c.pm));
    balanced(&p);cd321x_unregister_port(&c.tps);port_balance();
    memset(&c,0,sizeof(c));prepare(&p,true,true,false);notifier_error=-ENOMEM;
    assert(cd321x_register_port(&c.tps,&connector)==-ENOMEM);
    balanced(&p);port_balance();notifier_error=0;
    memset(&c,0,sizeof(c));prepare(&p,true,true,false);resume_error=-EIO;before=writes;
    assert(cd321x_register_port(&c.tps,&connector)==0 && !c.state_valid && writes==before);
    balanced(&p);cd321x_unregister_port(&c.tps);port_balance();
    puts("PASS: actual CD321x probe -> typec_set_mode -> typec_mux_set -> bounded PM wrapper; delayed root reprobe and rollback references");
}
"""
    return unit + '\n' + port + '\n' + controls


def upstream_resume_boundary(root):
    """Actual usbcore dispatch: root bus resume does not resume its child."""
    source = (root / 'drivers/usb/core/generic.c').read_text()
    function = block(source, 'int usb_generic_driver_resume(')
    prefix = r"""
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdio.h>
typedef int pm_message_t;
struct usb_device { struct usb_device *parent; };
static int bus_resumes, port_resumes, notifications, port_error;
static bool child_suspended = true;
static int hcd_bus_resume(struct usb_device *u, pm_message_t m)
{ (void)m; assert(!u->parent); bus_resumes++; return 0; }
static int usb_port_resume(struct usb_device *u, pm_message_t m)
{ (void)m; assert(u->parent); port_resumes++; if(port_error)return port_error; child_suspended=false; return 0; }
static void usbfs_notify_resume(struct usb_device *u)
{ (void)u; notifications++; }
"""
    controls = r"""
int main(void) {
 struct usb_device root={0}, child={.parent=&root};
 assert(usb_generic_driver_resume(&root,1)==0);
 assert(bus_resumes==1&&port_resumes==0&&child_suspended&&notifications==1);
 assert(usb_generic_driver_resume(&child,1)==0);
 assert(bus_resumes==1&&port_resumes==1&&!child_suspended&&notifications==2);
 child_suspended=true;port_error=-EIO;
 assert(usb_generic_driver_resume(&child,1)==-EIO&&child_suspended&&notifications==2);
 puts("PASS actual usbcore root/child resume dispatch and failed child resume");
 return 0;
}
"""
    return prefix + function + controls


def controller_lifecycle(root):
    from teardown_pm import unit_for
    return unit_for(root)


def asynchronous_root_state(root):
    """Actual USB state API does not use the USB device mutex as its fence."""
    source = (root / 'drivers/usb/core/hub.c').read_text()
    prefix = r"""
#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
enum usb_device_state { USB_STATE_NOTATTACHED, USB_STATE_CONFIGURED, USB_STATE_SUSPENDED };
struct usb_device;
struct usb_port { struct usb_device *child; };
struct usb_hub { struct usb_port **ports; };
struct device { int unused; };
struct config { struct { unsigned bmAttributes; } desc; };
struct usb_device { struct device dev; struct usb_device *parent; struct usb_hub *hub; enum usb_device_state state; int maxchild; unsigned quirks; struct config *actconfig; unsigned long active_duration; bool device_locked; };
static unsigned long jiffies=10;
static int device_state_lock;
#define USB_QUIRK_IGNORE_REMOTE_WAKEUP 1
#define USB_CONFIG_ATT_WAKEUP 2
#define spin_lock_irqsave(l,f) do { assert(!*(l));*(l)=1;(f)=0; } while(0)
#define spin_unlock_irqrestore(l,f) do { assert(*(l));*(l)=0;(void)(f); } while(0)
static struct usb_hub *usb_hub_to_struct_hub(struct usb_device *u) { return u->hub; }
static void update_port_device_state(struct usb_device *u) { (void)u; }
static void trace_usb_set_device_state(struct usb_device *u) { (void)u; }
static void device_set_wakeup_capable(struct device *d,int w) { (void)d;(void)w; }
"""
    controls = r"""
int main(void) {
 struct usb_device child={.state=USB_STATE_CONFIGURED,.device_locked=true};
 struct usb_port port={.child=&child};struct usb_port *ports[]={&port};
 struct usb_hub hub={.ports=ports};
 struct usb_device root={.state=USB_STATE_CONFIGURED,.maxchild=1,.hub=&hub,.device_locked=true};
 child.parent=&root;
 usb_set_device_state(&root,USB_STATE_NOTATTACHED);
 assert(root.state==USB_STATE_NOTATTACHED&&child.state==USB_STATE_NOTATTACHED);
 assert(root.device_locked&&child.device_locked&&!device_state_lock);
 usb_set_device_state(&child,USB_STATE_CONFIGURED);
 assert(child.state==USB_STATE_NOTATTACHED);
 puts("PASS actual asynchronous USB detach while device locks are held; no resurrection");
 return 0;
}
"""
    return prefix + block(source, 'static void update_usb_device_state(') + block(source, 'static void recursively_mark_NOTATTACHED(') + block(source, 'void usb_set_device_state(') + controls
