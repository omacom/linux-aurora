#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Compile the connect worker's report path, its log-level helper, the move
report helpers and the complete nl80211 encoder."""
from pathlib import Path
import argparse
import hashlib
import json
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
LINK = Path('drivers/net/wireless/mediatek/mt7932/link.c')
HEADER = Path('drivers/net/wireless/mediatek/mt7932/mt7932.h')

PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
typedef uint8_t u8; typedef uint16_t u16; typedef int gfp_t;
#define WLAN_STATUS_SUCCESS 0
#define WLAN_STATUS_UNSPECIFIED_FAILURE 1
#define WLAN_REASON_UNSPECIFIED 1
#define WLAN_REASON_DEAUTH_LEAVING 3
#define NL80211_TIMEOUT_UNSPECIFIED 0
#define GFP_ATOMIC 0
#define GFP_KERNEL 0
#define ETH_ALEN 6
#define WLAN_PMKID_LEN 16
#define WLAN_EID_TIM 5
#define NLA_HDRLEN 4
#define BIT(n) (1U<<(n))
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define rcu_dereference(x) (x)
#define rcu_read_lock() ((void)0)
#define rcu_read_unlock() ((void)0)
#define spin_lock_irqsave(l,f) ((void)(l),(f)=0)
#define spin_unlock_irqrestore(l,f) ((void)(l),(void)(f))
#define spin_lock(l) ((void)(l))
#define spin_unlock(l) ((void)(l))
#define mutex_lock(l) ((void)(l))
#define mutex_unlock(l) ((void)(l))
#define dev_info(...) ((void)info_logs++)
#define dev_dbg(...) ((void)0)
#define dev_err(...) ((void)err_logs++)
#define for_each_valid_link(cr,l) for((l)=0;(l)<1;(l)++)
struct sk_buff { unsigned values[32]; bool seen[32]; };
struct nlattr { int n; };
struct element { unsigned datalen; };
struct cfg80211_bss_ies { unsigned len; const u8 *data; };
struct cfg80211_bss { const u8 *bssid; const struct cfg80211_bss_ies *beacon_ies; u16 beacon_interval; };
struct net_device { int ifindex; };
struct cfg80211_registered_device { int wiphy_idx,wiphy; };
struct cfg80211_connect_resp_params {int status,timeout_reason;const u8 *req_ie,*resp_ie,*ap_mld_addr;size_t req_ie_len,resp_ie_len;unsigned valid_links;bool assoc_encrypted;
 struct {const u8 *kek,*pmk,*pmkid;size_t kek_len,pmk_len;bool update_erp_next_seq_num;u16 erp_next_seq_num;} fils;
 struct {const u8 *addr,*bssid;struct cfg80211_bss *bss;u16 status;} links[1];};
enum {NL80211_ATTR_WIPHY,NL80211_ATTR_IFINDEX,NL80211_ATTR_MAC,NL80211_ATTR_STATUS_CODE,NL80211_ATTR_TIMED_OUT,NL80211_ATTR_TIMEOUT_REASON,NL80211_ATTR_REQ_IE,NL80211_ATTR_RESP_IE,NL80211_ATTR_FILS_ERP_NEXT_SEQ_NUM,NL80211_ATTR_FILS_KEK,NL80211_ATTR_PMK,NL80211_ATTR_PMKID,NL80211_ATTR_MLO_LINKS,NL80211_ATTR_MLO_LINK_ID,NL80211_ATTR_BSSID,NL80211_ATTR_ASSOC_ENCRYPTED};
#define NL80211_CMD_CONNECT 1
#define NL80211_MCGRP_MLME 1
static int nl80211_fam;
static struct sk_buff encoded;
static struct sk_buff *nlmsg_new(size_t n,gfp_t f){memset(&encoded,0,sizeof(encoded));return &encoded;}
static void *nl80211hdr_put(struct sk_buff*m,int a,int b,int c,int d){return m;}
static void nlmsg_free(struct sk_buff*m){}
static int nla_put_u32(struct sk_buff*m,int a,unsigned v){m->values[a]=v;m->seen[a]=true;return 0;}
static int nla_put_u16(struct sk_buff*m,int a,u16 v){return nla_put_u32(m,a,v);}
static int nla_put_u8(struct sk_buff*m,int a,u8 v){return nla_put_u32(m,a,v);}
static int nla_put_flag(struct sk_buff*m,int a){return nla_put_u32(m,a,1);}
static int nla_put(struct sk_buff*m,int a,size_t n,const void*p){m->seen[a]=true;return 0;}
static size_t nla_total_size(size_t n){return n+4;}
static struct nlattr *nla_nest_start(struct sk_buff*m,int a){static struct nlattr n;return &n;}
static void nla_nest_end(struct sk_buff*m,struct nlattr*n){}
static void genlmsg_end(struct sk_buff*m,void*h){}
static int wiphy_net(int*w){return 0;}
static void genlmsg_multicast_netns(int*f,int n,struct sk_buff*m,int p,int g,gfp_t t){}
ENCODER
struct mt7932 {bool link_failed,stopping,connect_open,assoc_request_seen,connecting,connected,disconnecting,connect_cancelled,connect_publishing,connect_lost,rf_ready,connect_pmk_valid,roam_moving,roam_landed;
 int connect_error,response_lock,command_mutex,data_lock,wiphy,power_work;u16 disconnect_reason;struct net_device *netdev;struct cfg80211_bss *connect_bss;u8 connect_bssid[6],connect_pmk[32],connect_sae_pwd[63],connect_sae_pwd_length,assoc_request_ies[512];unsigned assoc_request_ie_len;
 struct {struct {bool ht_supported;}ht_cap;}band2;bool power_tim,data_ready,station_signal_valid,station_rx_rate_valid,station_tx_rate_valid;unsigned long station_tx_bytes,station_rx_bytes,station_tx_packets,station_rx_packets,station_connected;u16 station_beacon_tu;};
static unsigned long jiffies=1;
static unsigned info_logs,err_logs,notifications,disconnections,retirements,releases,authorizations,carrier,work_count,carrier_offs,deferred;
static u16 last_reason;static bool last_local;
static struct cfg80211_connect_resp_params saved;
static bool mt_rf_allowed(struct mt7932*m){return m->rf_ready;}
static void mt_transport_snapshot(struct mt7932*m){}
static void mt_cqm_reset(struct mt7932*m){}
static void memzero_explicit(void*p,size_t n){memset(p,0,n);}
static int mt_retire_connection(struct mt7932*m){retirements++;return 0;}
static void mt_peer_release(struct mt7932*m){releases++;}
static const struct element *cfg80211_find_elem(int id,const u8*p,size_t n){return NULL;}
static bool smp_load_acquire(bool*p){return *p;}
static void cfg80211_put_bss(int w,struct cfg80211_bss*b){releases++;}
static void cfg80211_disconnected(struct net_device*n,u16 r,const u8*p,size_t l,bool local,gfp_t f){disconnections++;}
static void cfg80211_connect_done(struct net_device*n,struct cfg80211_connect_resp_params*p,gfp_t f){struct cfg80211_registered_device r={};saved=*p;notifications++;nl80211_send_connect_result(&r,n,p,f);}
static void schedule_work(int*w){work_count++;}
static void netif_carrier_on(struct net_device*n){carrier++;}
static void netif_carrier_off(struct net_device*n){carrier_offs++;}
static void mt_reg_apply_deferred_locked(struct mt7932*m){deferred++;}
MOVE_RULES
static void mt_report_disconnected(struct mt7932*m,u16 r,bool local,gfp_t f){disconnections++;last_reason=r;last_local=local;}
static void netif_wake_queue(struct net_device*n){}
static void cfg80211_port_authorized(struct net_device*n,const u8*b,const u8*p,size_t l,gfp_t f){authorizations++;}
HELPER
static void report_worker(struct mt7932 *m,int ret,bool retirement_attempted,bool auth_submitted){
 struct cfg80211_connect_resp_params response={};unsigned long flags;bool submitted=true,peer_owned=true,cancelled;bool roam __attribute__((unused));struct mt7932_connect_report outcome __attribute__((unused));u16 reason;
REPORT
int main(void){struct net_device n={1};struct cfg80211_bss b={.beacon_interval=100};
 int errors[]={0,-ETIMEDOUT,-ECONNREFUSED,-EPROTO,-ECANCELED,-ECONNRESET,-EIO};
 for(unsigned i=0;i<sizeof(errors)/sizeof(*errors);i++)for(unsigned k=0;k<4;k++){
  bool auth=k&1,failed=k&2;
  struct mt7932 m={.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b,.data_ready=true,.link_failed=failed};
  info_logs=err_logs=notifications=disconnections=retirements=authorizations=carrier=work_count=0;
  memset(m.connect_pmk,0x5a,sizeof(m.connect_pmk));memset(m.connect_sae_pwd,0x5a,sizeof(m.connect_sae_pwd));m.connect_sae_pwd_length=63;m.connect_pmk_valid=true;
  report_worker(&m,errors[i],false,auth);
  // Only an AP-side outcome (timeout, refusal, reset) of a request that
  // reached the AP, with the firmware healthy, is logged at info level.
  bool peer=auth&&!failed&&(errors[i]==-ETIMEDOUT||errors[i]==-ECONNREFUSED||errors[i]==-ECONNRESET);
  assert(info_logs==(errors[i]&&peer)&&err_logs==(errors[i]&&!peer));
  for(unsigned j=0;j<sizeof(m.connect_pmk);j++)assert(!m.connect_pmk[j]);
  for(unsigned j=0;j<sizeof(m.connect_sae_pwd);j++)assert(!m.connect_sae_pwd[j]);
  assert(!m.connect_sae_pwd_length&&!m.connect_pmk_valid);assert(notifications==1&&disconnections==0);
  assert(saved.status==(errors[i]==-ETIMEDOUT?-1:errors[i]?1:0));
  assert(encoded.seen[NL80211_ATTR_TIMED_OUT]==(errors[i]==-ETIMEDOUT));
  assert(encoded.seen[NL80211_ATTR_TIMEOUT_REASON]==(errors[i]==-ETIMEDOUT));
  assert(!encoded.seen[NL80211_ATTR_TIMEOUT_REASON]||encoded.values[NL80211_ATTR_TIMEOUT_REASON]==NL80211_TIMEOUT_UNSPECIFIED);
  assert(encoded.values[NL80211_ATTR_STATUS_CODE]==(errors[i]?1:0));
  assert(m.connect_bss==NULL&&m.connected==!errors[i]);
  /* A connection records its AP's beacon interval, for the lost-AP check. */
  assert(m.station_beacon_tu==(errors[i]?0:100));
  assert(retirements==!!errors[i]);assert(authorizations==!errors[i]&&carrier==!errors[i]);
 }
 // Cancellation consumes the BSS and reports disconnect, not a connect result.
 struct mt7932 m={.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b,.connect_cancelled=true,.connect_error=-ECANCELED};
 notifications=disconnections=retirements=authorizations=carrier=0;
 info_logs=err_logs=0;
 report_worker(&m,0,false,true);assert(!info_logs&&!err_logs);assert(notifications==0&&disconnections==1&&retirements==1&&!m.connected);assert(authorizations==0&&carrier==0);
 // A stale success cannot publish carrier after a terminal error.
 m=(struct mt7932){.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b,.connect_error=-ECONNRESET};
 notifications=disconnections=retirements=authorizations=carrier=0;
 info_logs=err_logs=0;
 report_worker(&m,0,false,true);assert(info_logs==1&&!err_logs);assert(saved.status==1&&notifications==1&&retirements==1&&!m.connected&&!encoded.seen[NL80211_ATTR_TIMED_OUT]);assert(authorizations==0&&carrier==0);
 // Failed retry retirement retains ownership and is not attempted again.
 m=(struct mt7932){.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b};
 notifications=disconnections=retirements=releases=authorizations=carrier=0;
 info_logs=err_logs=0;
 report_worker(&m,-EIO,true,true);assert(!info_logs&&err_logs==1);
 assert(saved.status==1&&notifications==1&&!retirements&&!releases&&!m.connected);
 assert(!authorizations&&!carrier);
 // Already-known IEEE AP status is preserved by the complete netlink encoder.
 struct cfg80211_registered_device r={};struct cfg80211_connect_resp_params ap={.status=17};
 nl80211_send_connect_result(&r,&n,&ap,0);assert(encoded.values[NL80211_ATTR_STATUS_CODE]==17&&!encoded.seen[NL80211_ATTR_TIMED_OUT]);
 // A cancelled connect reports the cancel's reason and applies no deferral.
 m=(struct mt7932){.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b,.connect_cancelled=true,.connect_error=-ECANCELED,.disconnect_reason=8};
 notifications=disconnections=retirements=authorizations=carrier=carrier_offs=deferred=0;
 report_worker(&m,0,false,true);assert(notifications==0&&disconnections==1&&last_reason==8&&last_local&&!deferred);
 // A move that lands takes over the connection through one connect result.
 m=(struct mt7932){.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b,.data_ready=true,.roam_moving=true};
 notifications=disconnections=retirements=authorizations=carrier=carrier_offs=deferred=0;info_logs=err_logs=0;
 report_worker(&m,0,false,true);assert(notifications==1&&saved.status==0&&!disconnections&&m.connected&&m.roam_landed&&!m.roam_moving);
 assert(carrier==1&&!carrier_offs&&authorizations==1&&!deferred&&!retirements&&!info_logs&&!err_logs);
 // A move that fails reports its failed result, then ends the old connection.
 m=(struct mt7932){.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b,.data_ready=true,.roam_moving=true,.roam_landed=true,.disconnect_reason=8};
 notifications=disconnections=retirements=authorizations=carrier=carrier_offs=deferred=0;info_logs=err_logs=0;
 report_worker(&m,-ETIMEDOUT,false,true);assert(notifications==1&&saved.status==-1&&disconnections==1&&last_reason==WLAN_REASON_DEAUTH_LEAVING&&last_local);
 assert(!m.connected&&!m.roam_landed&&!m.roam_moving&&carrier_offs==1&&deferred==1&&retirements==1&&!carrier&&!authorizations&&info_logs==1);
 // A cancelled move reports its failed result, then the cancel's reason.
 m=(struct mt7932){.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b,.roam_moving=true,.connect_cancelled=true,.connect_error=-ECANCELED,.disconnect_reason=8};
 notifications=disconnections=retirements=authorizations=carrier=carrier_offs=deferred=0;
 report_worker(&m,0,false,true);assert(notifications==1&&saved.status==1&&disconnections==1&&last_reason==8&&last_local);
 assert(!m.connected&&!m.roam_landed&&!m.roam_moving&&carrier_offs==1&&deferred==1&&!authorizations);
 // An ordinary connect after a landed move clears the mark.
 m=(struct mt7932){.connecting=true,.rf_ready=true,.netdev=&n,.connect_bss=&b,.data_ready=true,.roam_landed=true};
 notifications=disconnections=retirements=authorizations=carrier=carrier_offs=deferred=0;
 report_worker(&m,0,false,true);assert(notifications==1&&!disconnections&&m.connected&&!m.roam_landed&&!carrier_offs&&!deferred);
 puts("PASS 17 connect report, info-vs-error log level, nl80211 timeout/rejection/cancellation and move report controls");}
'''


def peer_helper(source):
    """mt_connect_failed_at_peer(), which picks the report's log level; empty
    for an older driver that has none."""
    marker = 'static bool mt_connect_failed_at_peer('
    if marker not in source:
        return ''
    start = source.index(marker)
    return source[start:source.index('\n}\n', start) + 3]


def move_helpers(header):
    """The move report helpers from mt7932.h; empty for a driver without
    them."""
    marker = 'struct mt7932_connect_report {'
    if marker not in header:
        return ''
    start = header.index(marker)
    last = header.index('static inline bool mt7932_disconnect_local(', start)
    return header[start:header.index('\n}\n', last) + 3]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--source-root', type=Path, default=ROOT)
    parser.add_argument('--old-source', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    root = args.source_root
    source = (root/LINK).read_text()
    start = source.index('report:\n', source.index('void mt_connect_work('))
    end = source.index('\n/* Narrow ordinary OPEN', start)
    report = source[start:end]
    helper = peer_helper(source)
    move = move_helpers((root/HEADER).read_text())
    net = (root/'net/wireless/nl80211.c').read_text()
    start = net.index('void nl80211_send_connect_result(')
    end = net.index('\nvoid nl80211_send_roamed(', start)
    encoder = net[start:end]
    with tempfile.TemporaryDirectory(prefix='mt7932-result-') as temporary:
        out = args.output or Path(temporary)
        out.mkdir(parents=True, exist_ok=True)
        cases = {'fixed': report}
        helpers = {'fixed': helper}
        if args.old_source:
            old = args.old_source.read_text()
            start = old.index('report:\n', old.index('void mt_connect_work('))
            end = old.index('\n/* Narrow ordinary OPEN', start)
            cases['old'] = old[start:end]
            helpers['old'] = peer_helper(old)
        results = {}
        for name, body in cases.items():
            c = out/(name+'.c')
            c.write_text(PREFIX.replace('MOVE_RULES', move).replace('ENCODER', encoder).replace('HELPER', helpers[name])
                         .replace('REPORT', body))
            binary = out/name
            subprocess.run(['cc','-std=gnu11','-Wall','-Werror','-Wno-unused-parameter',
                            '-Wno-unused-function','-I/usr/include',str(c),'-o',str(binary)], check=True)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            (out/(name+'.log')).write_text(result.stdout+result.stderr)
            results[name] = {'exit': result.returncode, 'report_sha256': hashlib.sha256(body.encode()).hexdigest(),
                             'encoder_sha256': hashlib.sha256(encoder.encode()).hexdigest(),
                             'binary_sha256': hashlib.sha256(binary.read_bytes()).hexdigest()}
            assert (result.returncode==0)==(name=='fixed'), results
            print(name, result.returncode, result.stdout.strip())
        (out/'results.json').write_text(json.dumps(results, indent=2)+'\n')


if __name__ == '__main__':
    main()
