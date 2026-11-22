#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run production regulatory/PCIe PM and TX worker code against observing stubs."""
import argparse
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / 'drivers/net/wireless/broadcom/brcm80211/brcmfmac'


def function(source, name):
    match = re.search(r'^(?:static\s+)?(?:void|int|s32)\s+' + name +
                      r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise ValueError('Missing function ' + name)
    end = source.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
typedef int32_t s32; typedef uint32_t u32; typedef uint8_t u8; typedef uint32_t __le32;
#define le32_to_cpu(x) (x)
#define BIT(n) (1U << (n))
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define smp_load_acquire(p) (*(p))
#define brcmf_dbg(...) ((void)0)
#define bphy_err(...) ((void)drvr)
#define bphy_info_once(...) ((void)drvr)
#define brcmf_info(...) ((void)0)
#define brcmf_err(...) ((void)0)
#define __brcmf_err(...) ((void)0)
#define spin_lock_irqsave(l,f) ((void)(l),(f)=0)
#define spin_unlock_irqrestore(l,f) ((void)(l),(void)(f))
#define BRCMF_MAX_IFS 4
#define BRCMF_H2D_HOST_D3_INFORM 3
#define BRCMF_H2D_HOST_D0_INFORM 0
#define BRCMF_PCIE_MBDATA_TIMEOUT 1
#define BCMA_CORE_PCIE2 1
#define BRCMFMAC_PCIE_STATE_UP 1
#define BRCMFMAC_PCIE_STATE_DOWN 0
#define wait_event_timeout(q,c,t) ((void)(c))
enum brcmf_bus_state { BRCMF_BUS_DOWN, BRCMF_BUS_UP };
/* STOP_REASONS */
struct net_device { bool stopped; unsigned drains, wakes; };
struct brcmf_pub;
struct brcmf_cfg80211_vif { unsigned long sme_state; int wdev; struct brcmf_cfg80211_vif *next; };
struct brcmf_if { struct brcmf_cfg80211_vif *vif; struct brcmf_pub *drvr; struct net_device *ndev;
 unsigned netif_stop; int netif_stop_lock, bsscfgidx; };
struct device { void *data; };
struct pci_dev { struct device dev; int irq; bool state_saved; };
struct brcmf_pciedev_info { bool mbdata_completed, intr, ready, timer, sleep_reset_blocked, setup_complete;
 bool wowl_irq, wowl_enabled, irq_wake_enabled, wowl_sleeping, fwseed, s2idle_d0;
 int state, mbdata_resp_wait; void *ci; struct pci_dev *pdev;
 struct { int intmask, mailboxmask; } *reginfo; };
struct pcie_priv { struct brcmf_pciedev_info *devinfo; };
struct brcmf_bus { enum brcmf_bus_state state; struct brcmf_pub *drvr; bool wowl_transport, wowl_wake;
 struct { struct pcie_priv *pcie; } bus_priv; };
struct brcmf_proto { void *pd; };
struct brcmf_pub { struct brcmf_bus *bus_if; struct brcmf_if *iflist[BRCMF_MAX_IFS];
 struct brcmf_proto *proto; };
struct brcmf_cfg80211_info { struct brcmf_pub *pub;
 bool suspended, country_pending, force_band_setup; char country_alpha2[2];
unsigned long scan_status; struct wiphy *wiphy;
struct {struct brcmf_cfg80211_vif *first;} vif_list;
struct {bool active, nd_enabled, lpas; int pre_pmmode;} wowl; };
struct wiphy { struct brcmf_cfg80211_info *cfg; };
struct regulatory_request { char alpha2[2]; int initiator; };
struct brcmf_fil_country_le { char alpha2[2]; __le32 rev; char ccode[4]; };
struct work_struct { bool queued; };
struct brcmf_msgbuf_work_item { struct brcmf_msgbuf_work_item *next; };
struct brcmf_msgbuf { struct brcmf_pub *drvr; bool tx_suspended;
 struct work_struct flowring_work, txflow_work; void *txflow_wq;
 unsigned long flow_map[1]; unsigned max_flowrings;
 struct brcmf_msgbuf_work_item *pending; };
#define for_each_set_bit(i,p,n) for ((i)=0;(i)<(n);(i)++) if ((*(p)&BIT(i)))
#define clear_bit(i,p) (*(p)&=~BIT(i))
static struct brcmf_bus bus;
static struct brcmf_pub pub;
static struct brcmf_proto proto;
static struct brcmf_msgbuf msgbuf;
static struct net_device ndev[2];
static struct brcmf_if ifp[2];
static struct brcmf_cfg80211_info cfg;
static struct wiphy wiphy;
static struct device device;
static struct pci_dev pdev;
static struct brcmf_pciedev_info devinfo;
static struct pcie_priv pcie;
static unsigned country_gets, sets, bands, txflows, creates, cancels, removes, probes;
static int failure, translation, retain, d0_error, probe_error;
static bool ack;
static char applied[2];
static void netif_stop_queue(struct net_device *n) { n->stopped=true; }
static void netif_wake_queue(struct net_device *n) {
 assert(bus.state==BRCMF_BUS_UP); assert(devinfo.ready && devinfo.intr);
 n->stopped=false; n->wakes++;
}
static void netif_tx_disable(struct net_device *n) {
 assert(bus.state==BRCMF_BUS_UP); assert(ifp[n-ndev].netif_stop & BRCMF_NETIF_STOP_REASON_BUS);
 n->stopped=true; n->drains++;
}
static void *dev_get_drvdata(struct device *d) {return d->data;}
static struct brcmf_cfg80211_info *wiphy_to_cfg(struct wiphy *w) {return w->cfg;}
static struct brcmf_if *brcmf_get_ifp(struct brcmf_pub *p,int i) {return p->iflist[i];}
static void can_command(void) {assert(!cfg.suspended && bus.state==BRCMF_BUS_UP);}
static int brcmf_fil_iovar_data_get(struct brcmf_if *i,const char *s,void *v,size_t n) {
 (void)i;(void)s;(void)n;can_command();country_gets++;memset(v,0,sizeof(struct brcmf_fil_country_le));return failure==1?-EIO:0;
}
static int brcmf_fil_iovar_data_set(struct brcmf_if *i,const char *s,void *v,size_t n) {
 (void)i;(void)s;(void)n;can_command();sets++;memcpy(applied,v,2);return failure==2?-EIO:0;
}
static int brcmf_translate_country_code(struct brcmf_pub *p,char *a,struct brcmf_fil_country_le *c) {
 (void)p;memcpy(c->alpha2,a,2);return translation;
}
static int brcmf_setup_wiphybands(struct brcmf_cfg80211_info *c) {
 (void)c;can_command();bands++;return failure==3?-EIO:0;
}
static struct brcmf_msgbuf_work_item *brcmf_msgbuf_dequeue_work(struct brcmf_msgbuf *m) {
 struct brcmf_msgbuf_work_item *w=m->pending;if(w)m->pending=w->next;return w;
}
static void brcmf_msgbuf_flowring_create_worker(struct brcmf_msgbuf *m,struct brcmf_msgbuf_work_item *w) {
 (void)w;assert(!m->tx_suspended && devinfo.state==BRCMFMAC_PCIE_STATE_UP);creates++;
}
static void brcmf_msgbuf_txflow(struct brcmf_msgbuf *m,u32 i) {
 (void)i;assert(!m->tx_suspended && devinfo.state==BRCMFMAC_PCIE_STATE_UP);txflows++;
}
#define kfree free
static bool schedule_work(struct work_struct *w) {w->queued=true;return true;}
static bool queue_work(void *q,struct work_struct *w) {(void)q;return schedule_work(w);}
static void cancel_work_sync(struct work_struct *w) {
 assert(msgbuf.tx_suspended);w->queued=false;cancels++;
}
static void brcmf_pcie_fwcon_timer(struct brcmf_pciedev_info *d,bool on) {d->timer=on;}
#define PM_SUSPEND_TO_IDLE 1
#define PM_SUSPEND_ON 0
static int pm_suspend_target_state = PM_SUSPEND_TO_IDLE;
static bool device_may_wakeup(struct device *d) {(void)d;return true;}
static int wake_error, save_error; static unsigned d3_informs, d0_informs;
static int enable_irq_wake(int irq) {(void)irq;return wake_error;}
static int pci_save_state(struct pci_dev *p) {if(save_error)return save_error;p->state_saved=true;return 0;}
static void pci_restore_state(struct pci_dev *p) {p->state_saved=false;}
static void brcmf_pcie_disable_irq_wake(struct brcmf_pciedev_info *d) {d->irq_wake_enabled=false;}
static int brcmf_pcie_pm_leave_D3(struct device *dev);
static int brcmf_pcie_send_mb_data(struct brcmf_pciedev_info *d,int mb) {
 if(mb==BRCMF_H2D_HOST_D3_INFORM) {d3_informs++;
  assert(ndev[0].stopped && ndev[1].stopped && ndev[0].drains && ndev[1].drains);
  assert(msgbuf.tx_suspended && cancels==2);d->mbdata_completed=ack;
 } else {d0_informs++;assert(d->state==BRCMFMAC_PCIE_STATE_UP);return d0_error;}
 return 0;
}
static int brcmf_pcie_read_pcie32(struct brcmf_pciedev_info *d,int r) {(void)d;(void)r;return retain;}
static void brcmf_pcie_select_core(struct brcmf_pciedev_info *d,int c) {(void)d;(void)c;}
static void brcmf_pcie_intr_enable(struct brcmf_pciedev_info *d) {d->intr=true;}
static void brcmf_pcie_hostready(struct brcmf_pciedev_info *d) {assert(d->intr);d->ready=true;}
static void brcmf_chip_detach(void *c) {(void)c;}
static void brcmf_pcie_remove(struct pci_dev *p) {(void)p;removes++;}
static int brcmf_pcie_probe(struct pci_dev *p,void *id) {(void)p;(void)id;probes++;return probe_error;}
/* Not a J713 sleep: the radio sleep hooks do nothing, as on every other board. */
static bool apple_radio_sleep_owned(struct device *d) {(void)d;return false;}
static int apple_radio_sleep_endpoint_prepare(struct device *d,void (*finish)(struct device *)) {(void)d;(void)finish;return 0;}
static int apple_radio_sleep_endpoint_suspended(struct device *d) {(void)d;return 0;}
static int apple_radio_sleep_endpoint_resume(struct device *d) {(void)d;return 0;}
static void apple_radio_sleep_endpoint_resumed(struct device *d,int r) {(void)d;(void)r;}
/* Only a sleep that owns the radio gets to these. */
static void brcmf_pcie_sleep_finish(struct device *d) {(void)d;abort();}
static void brcmf_bus_cancel_reset_work(struct brcmf_bus *b) {(void)b;abort();}
static int brcmf_pcie_pm_leave_D3(struct device *dev);
'''

CALLBACK_STUBS = r'''
#define BRCMF_FEAT_WOWL_ARP_ND 1
#define BRCMF_FEAT_PNO 2
#define BRCMF_C_SET_PM 1
#define BRCMF_E_PFN_NET_FOUND 1
#define BRCMF_SCAN_STATUS_BUSY 1
#define BRCMF_VIF_STATUS_READY 2
#define WLAN_REASON_UNSPECIFIED 1
#define test_bit(i,p) ((*(p)&BIT(i))!=0)
#define list_for_each_entry(v,h,m) for ((v)=(h)->first;(v);(v)=(v)->next)
struct cfg80211_wowlan {bool any, magic_pkt;};
#define BRCMF_FEAT_WOWL 3
#define BRCMF_FEAT_LPAS 4
#define GFP_KERNEL 0
static unsigned lpas_restores, wake_reports;
static void cfg80211_report_wowlan_wakeup(int *w,void *i,int g) {(void)w;(void)i;(void)g;wake_reports++;}
static int brcmf_configure_lpas(struct brcmf_cfg80211_info *c,struct brcmf_if *i,struct cfg80211_wowlan *w) {(void)c;(void)i;(void)w;can_command();return 0;}
static int brcmf_restore_lpas(struct brcmf_cfg80211_info *c,struct brcmf_if *i) {
 (void)i;assert(bus.state==BRCMF_BUS_UP);lpas_restores++;c->wowl.lpas=c->wowl.active=false;return failure==4?-EIO:0;
}
static bool vif_up;
static struct net_device *cfg_to_ndev(struct brcmf_cfg80211_info *c) {return c->pub->iflist[0]->ndev;}
static struct brcmf_if *netdev_priv(struct net_device *n) {return &ifp[n-ndev];}
static bool check_vif_up(struct brcmf_cfg80211_vif *v) {(void)v;return vif_up;}
static bool brcmf_feat_is_enabled(struct brcmf_if *i,int f) {(void)i;(void)f;return false;}
static void brcmf_report_wowl_wakeind(struct wiphy *w,struct brcmf_if *i) {(void)w;(void)i;can_command();}
static void brcmf_fil_iovar_int_set(struct brcmf_if *i,const char *s,int n) {(void)i;(void)s;(void)n;can_command();}
static void brcmf_config_wowl_pattern(struct brcmf_if *i,const char *s,void *p,int n,void *q,int t) {(void)i;(void)s;(void)p;(void)n;(void)q;(void)t;can_command();}
static void brcmf_configure_arp_nd_offload(struct brcmf_if *i,bool b) {(void)i;(void)b;can_command();}
static void brcmf_fil_cmd_int_set(struct brcmf_if *i,int c,int n) {(void)i;(void)c;(void)n;can_command();}
static void brcmf_cfg80211_sched_scan_stop(struct wiphy *w,struct net_device *n,int i) {(void)w;(void)n;(void)i;can_command();}
static void brcmf_fweh_unregister(struct brcmf_pub *p,int e) {(void)p;(void)e;}
#define brcmf_fweh_register(p,e,f) ((void)(p),(void)(e))
static void brcmf_abort_scanning(struct brcmf_cfg80211_info *c) {(void)c;can_command();}
static void brcmf_bus_wowl_config(struct brcmf_bus *b,bool e) {(void)b;(void)e;}
static void brcmf_link_down(struct brcmf_cfg80211_vif *v,int r,bool b) {(void)v;(void)r;(void)b;can_command();}
static void brcmf_delay(int t) {(void)t;}
static void brcmf_set_mpc(struct brcmf_if *i,int v) {(void)i;(void)v;can_command();}
static int brcmf_configure_wowl(struct brcmf_cfg80211_info *c,struct brcmf_if *i,struct cfg80211_wowlan *w) {(void)c;(void)i;(void)w;can_command();return 0;}
static void brcmf_keepalive_start(struct brcmf_if *i,int n) {(void)i;(void)n;can_command();}
'''

MAIN = r'''
static void reset(void) {
 memset(&bus,0,sizeof(bus));memset(&pub,0,sizeof(pub));memset(ndev,0,sizeof(ndev));
 memset(ifp,0,sizeof(ifp));memset(&cfg,0,sizeof(cfg));memset(&msgbuf,0,sizeof(msgbuf));
 memset(&devinfo,0,sizeof(devinfo));
 bus.state=BRCMF_BUS_UP;bus.drvr=&pub;bus.bus_priv.pcie=&pcie;pcie.devinfo=&devinfo;
 pub.bus_if=&bus;pub.proto=&proto;proto.pd=&msgbuf;msgbuf.drvr=&pub;msgbuf.max_flowrings=4;
 for(int i=0;i<2;i++){ifp[i].drvr=&pub;ifp[i].ndev=&ndev[i];pub.iflist[i]=&ifp[i];}
 cfg.pub=&pub;cfg.wiphy=&wiphy;wiphy.cfg=&cfg;vif_up=true;device.data=&bus;devinfo.pdev=&pdev;
 static typeof(*devinfo.reginfo) regs;devinfo.reginfo=&regs;
 devinfo.state=BRCMFMAC_PCIE_STATE_UP;devinfo.intr=devinfo.ready=devinfo.timer=true;
 country_gets=sets=bands=txflows=creates=cancels=removes=probes=0;
 failure=translation=d0_error=probe_error=wake_error=save_error=0;ack=true;retain=1;d3_informs=d0_informs=0;
 pm_suspend_target_state=PM_SUSPEND_TO_IDLE;
}
static void hint(const char *a) {
 struct regulatory_request r={.alpha2={a[0],a[1]}};brcmf_cfg80211_reg_notifier(&wiphy,&r);
}
int main(void) {
 unsigned cases=0;
for(int up=0;up<2;up++)for(int fail=0;fail<2;fail++) {
 reset();vif_up=up;
 assert(!brcmf_cfg80211_suspend(&wiphy,NULL) && cfg.suspended);
 assert(!brcmf_pcie_pm_enter_D3(&device));hint("AU");hint("GB");
 assert(!country_gets && cfg.country_pending);
 assert(!brcmf_pcie_pm_leave_D3(&device));failure=fail;
  /* A regulatory error keeps its retry and existing diagnostics; it must
   * not make cfg80211 shut down every interface from the PM callback. */
  assert(!brcmf_cfg80211_resume(&wiphy));
 assert(!cfg.suspended && cfg.country_pending==!!fail && country_gets==1);
 if(!fail)assert(!memcmp(applied,"GB",2));cases++;
}
 for(int sleep=0;sleep<2;sleep++)for(int down=0;down<2;down++) {
  reset();cfg.suspended=sleep;bus.state=down?BRCMF_BUS_DOWN:BRCMF_BUS_UP;
  hint("AU");hint("GB");hint("00");hint("9X");
  if(sleep || down) {
   assert(!country_gets && !sets && !bands && cfg.country_pending);
   assert(!memcmp(cfg.country_alpha2,"GB",2));
   cfg.suspended=false;bus.state=BRCMF_BUS_UP;
   assert(!brcmf_cfg80211_update_country(&cfg));
   assert(country_gets==1 && sets==1 && bands==1 && !memcmp(applied,"GB",2));
  } else assert(country_gets==2 && sets==2 && bands==2);
  assert(!cfg.country_pending);unsigned before=country_gets;
  assert(!brcmf_cfg80211_update_country(&cfg) && country_gets==before);cases++;
 }
 for(int fail=1;fail<=3;fail++) {
  reset();cfg.force_band_setup=true;failure=fail;hint("AU");
  assert(cfg.country_pending && cfg.force_band_setup);
  failure=0;assert(!brcmf_cfg80211_update_country(&cfg));
  assert(!cfg.country_pending && !cfg.force_band_setup);cases++;
 }
 reset();translation=-EAGAIN;hint("AU");assert(!sets && !cfg.country_pending);
 cfg.force_band_setup=true;hint("AU");assert(sets==1 && bands==1);cases++;
 reset();translation=-EINVAL;hint("AU");assert(!sets && cfg.country_pending);cases++;
 for(unsigned stopped=0;stopped<8;stopped++) {
  reset();ifp[1].netif_stop=stopped;ndev[1].stopped=!!stopped;
  assert(!brcmf_pcie_pm_enter_D3(&device));
  assert(bus.state==BRCMF_BUS_DOWN && msgbuf.tx_suspended && !devinfo.timer);
  assert(devinfo.state==BRCMFMAC_PCIE_STATE_DOWN);
  /* Simulate completion work arriving after cancellation. It must retain
   * both pending ring creation and the TX bitmap without touching PCIe. */
  msgbuf.pending=calloc(1,sizeof(*msgbuf.pending));msgbuf.flow_map[0]=BIT(1)|BIT(3);
  brcmf_msgbuf_flowring_worker(&msgbuf.flowring_work);
  brcmf_msgbuf_txflow_worker(&msgbuf.txflow_work);
  assert(msgbuf.pending && !creates && !txflows && msgbuf.flow_map[0]==10);
  brcmf_txflowblock_if(&ifp[0],BRCMF_NETIF_STOP_REASON_FLOW,false);
  assert(ndev[0].stopped);
  devinfo.intr=devinfo.ready=false;
  assert(!brcmf_pcie_pm_leave_D3(&device));
  assert(bus.state==BRCMF_BUS_UP && !msgbuf.tx_suspended && devinfo.timer);
  assert(!ndev[0].stopped && ndev[1].stopped==!!stopped && ifp[1].netif_stop==stopped);
  assert(msgbuf.flowring_work.queued && msgbuf.txflow_work.queued);
  brcmf_msgbuf_flowring_worker(&msgbuf.flowring_work);
  brcmf_msgbuf_txflow_worker(&msgbuf.txflow_work);
  assert(!msgbuf.pending && creates==1 && txflows==2 && !msgbuf.flow_map[0]);cases++;
 }
 /* An LPAS resume restores firmware policy and still leaves the suspended
  * state, so a country hint deferred across the sleep is applied. */
 for(int fail=0;fail<2;fail++) {
  reset();lpas_restores=wake_reports=0;static struct brcmf_cfg80211_vif lv;ifp[0].vif=&lv;assert(!brcmf_pcie_pm_enter_D3(&device));
  cfg.suspended=cfg.wowl.active=cfg.wowl.lpas=true;hint("GB");assert(!country_gets && cfg.country_pending);
  assert(!brcmf_pcie_pm_leave_D3(&device));failure=fail?4:0;
  assert(brcmf_cfg80211_resume(&wiphy)==(fail?-EIO:0));
  assert(lpas_restores==1 && wake_reports==1 && !cfg.suspended && !cfg.wowl.active);
  assert(!cfg.country_pending && country_gets==1 && !memcmp(applied,"GB",2));cases++;
 }
 /* WoWLAN in s2idle keeps the firmware awake: no D3 inform, the function
  * stays in D0 with its MSI wake-armed, and the resume is a hot one. */
 for(int fail=0;fail<2;fail++) {
  reset();devinfo.wowl_irq=devinfo.wowl_enabled=true;wake_error=fail?-EINVAL:0;
  int ret=brcmf_pcie_pm_enter_D3(&device);
  assert(!d3_informs && devinfo.state==BRCMFMAC_PCIE_STATE_UP);
  if(fail) {
   assert(ret==-EINVAL && !devinfo.wowl_sleeping && !devinfo.irq_wake_enabled);
   assert(bus.state==BRCMF_BUS_UP && !msgbuf.tx_suspended && devinfo.timer);cases++;continue;
  }
  assert(!ret && devinfo.wowl_sleeping && devinfo.irq_wake_enabled && pdev.state_saved);
  assert(bus.state==BRCMF_BUS_DOWN && msgbuf.tx_suspended && !devinfo.timer && ndev[0].stopped);
  devinfo.intr=false;
  assert(!brcmf_pcie_pm_leave_D3(&device));
  assert(!devinfo.wowl_sleeping && !devinfo.irq_wake_enabled && !pdev.state_saved && devinfo.intr);
  assert(bus.state==BRCMF_BUS_UP && !msgbuf.tx_suspended && devinfo.timer && !ndev[0].stopped);
  assert(!d0_informs && !removes && !probes);cases++;
 }
 /* Apple firmware in s2idle without WoWLAN stays in D0: no D3 inform, the
  * PCI state is saved so PCI PM keeps the power state, TX is fenced as for
  * D3, and the resume neither informs D0 nor reprobes. */
 for(int fail=0;fail<2;fail++) {
  reset();devinfo.fwseed=true;save_error=fail?-EINVAL:0;
  int ret=brcmf_pcie_pm_enter_D3(&device);
  assert(!d3_informs && devinfo.state==BRCMFMAC_PCIE_STATE_UP && !devinfo.wowl_sleeping);
  if(fail) {
   assert(ret==-EINVAL && !devinfo.s2idle_d0 && !pdev.state_saved);
   assert(bus.state==BRCMF_BUS_UP && !msgbuf.tx_suspended && devinfo.timer);cases++;continue;
  }
  assert(!ret && devinfo.s2idle_d0 && pdev.state_saved && !devinfo.irq_wake_enabled);
  assert(bus.state==BRCMF_BUS_DOWN && msgbuf.tx_suspended && !devinfo.timer && ndev[0].stopped);
  devinfo.intr=false;
  assert(!brcmf_pcie_pm_leave_D3(&device));
  assert(!devinfo.s2idle_d0 && !pdev.state_saved && devinfo.intr);
  assert(bus.state==BRCMF_BUS_UP && !msgbuf.tx_suspended && devinfo.timer && !ndev[0].stopped);
  assert(msgbuf.flowring_work.queued && msgbuf.txflow_work.queued);
  assert(!d0_informs && !removes && !probes);cases++;
 }
 /* Outside s2idle (hibernation) Apple firmware still gets the D3 handshake. */
 reset();devinfo.fwseed=true;pm_suspend_target_state=PM_SUSPEND_ON;
 assert(!brcmf_pcie_pm_enter_D3(&device) && d3_informs==1 && !devinfo.s2idle_d0);
 assert(devinfo.state==BRCMFMAC_PCIE_STATE_DOWN && !pdev.state_saved);cases++;
 reset();ack=false;assert(brcmf_pcie_pm_enter_D3(&device)==-EIO);
 assert(bus.state==BRCMF_BUS_UP && !msgbuf.tx_suspended && devinfo.timer && !ndev[0].stopped);cases++;
 for(int cold=0;cold<2;cold++)for(int fail=0;fail<2;fail++) {
  reset();assert(!brcmf_pcie_pm_enter_D3(&device));retain=!cold;d0_error=cold?0:-EIO;
  probe_error=fail?-EIO:0;assert(brcmf_pcie_pm_leave_D3(&device)==probe_error);
  assert(removes==1 && probes==1 && ndev[0].stopped && msgbuf.tx_suspended);cases++;
 }
 printf("%u regulatory, TX queue, worker retention and PCIe PM controls passed\n",cases);
 return 0;
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path)
    parser.add_argument('--mutation', choices=['country-gate', 'queue-reasons', 'worker-gate',
                                              'resume-order', 'tx-drain', 'lpas-resume',
                                              's2idle-d3'])
    args = parser.parse_args()
    sources = {name: (DRIVER / (name + '.c')).read_text()
               for name in ('cfg80211', 'core', 'pcie', 'msgbuf')}
    reasons = (DRIVER / 'core.h').read_text()
    reasons = re.search(r'enum brcmf_netif_stop_reason \{.*?\};', reasons, re.S)[0]
    groups = {
        'cfg80211': ['brcmf_cfg80211_update_country', 'brcmf_cfg80211_reg_notifier',
                    'brcmf_cfg80211_suspend', 'brcmf_cfg80211_resume'],
        'core': ['brcmf_txflowblock_if', 'brcmf_bus_change_state'],
        'msgbuf': ['brcmf_msgbuf_flowring_worker', 'brcmf_msgbuf_txflow_worker',
                   'brcmf_msgbuf_suspend', 'brcmf_msgbuf_resume'],
        'pcie': ['brcmf_pcie_wowl_wake', 'brcmf_pcie_wowl_sleep', 'brcmf_pcie_s2idle_sleep',
                 'brcmf_pcie_pm_enter_D3', 'brcmf_pcie_pm_leave_D3'],
    }
    body = ''.join(function(sources[group], name)
                   for group, names in groups.items() for name in names)
    if args.mutation == 'country-gate':
        body = body.replace('!cfg->country_pending || cfg->suspended ||\n\t    drvr->bus_if->state != BRCMF_BUS_UP',
                            '!cfg->country_pending')
    elif args.mutation == 'queue-reasons':
        body = body.replace('if (!ifp->netif_stop)\n\t\t\tnetif_wake_queue',
                            'if (true)\n\t\t\tnetif_wake_queue')
    elif args.mutation == 'worker-gate':
        body = body.replace('if (READ_ONCE(msgbuf->tx_suspended))', 'if (false)')
    elif args.mutation == 'resume-order':
        body = body.replace('brcmf_pcie_intr_enable(devinfo);',
                            'brcmf_bus_change_state(bus, BRCMF_BUS_UP);\n\t\tbrcmf_pcie_intr_enable(devinfo);')
    elif args.mutation == 'lpas-resume':
        body = body.replace('err = brcmf_restore_lpas(cfg, ifp);\n\t\t\tgoto out;',
                            'return brcmf_restore_lpas(cfg, ifp);')
    elif args.mutation == 's2idle-d3':
        body = body.replace('if (devinfo->fwseed && pm_suspend_target_state == PM_SUSPEND_TO_IDLE)',
                            'if (false)')
    elif args.mutation == 'tx-drain':
        body = body.replace('netif_tx_disable(ifp->ndev);', '(void)ifp;')
    code = PREFIX.replace('/* STOP_REASONS */', reasons) + CALLBACK_STUBS + body + MAIN

    def run(out):
        out.mkdir(parents=True, exist_ok=True)
        source = out / 'controls.c'
        source.write_text(code)
        exe = out / 'controls'
        subprocess.run(['clang', '-std=gnu11', '-O1', '-g', '-fsanitize=address,undefined',
                        '-fno-sanitize-recover=all', str(source), '-o', str(exe)], check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        receipt = {'exit': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr,
                   'limits': 'Observing host stubs; no firmware, kernel TX locking or hardware timing proof.'}
        (out / 'results.json').write_text(json.dumps(receipt, indent=2) + '\n')
        print(json.dumps(receipt))
        return result.returncode

    if args.out:
        raise SystemExit(run(args.out))
    with tempfile.TemporaryDirectory(prefix='brcmf-suspend-') as temp:
        raise SystemExit(run(Path(temp)))


if __name__ == '__main__':
    main()
