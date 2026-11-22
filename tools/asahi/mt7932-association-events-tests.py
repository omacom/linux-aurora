#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise association events, bounded waits and calibration reply ownership."""
from pathlib import Path
import argparse
import hashlib
import json
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
DRIVER = Path("drivers/net/wireless/mediatek/mt7932")


def function(source, name):
    match = re.search(r"^(?:static )?(?:inline )?[^\n;]+\b" + re.escape(name) +
                      r"\([^;]*?\)\s*\{", source, re.M)
    if not match:
        raise ValueError("missing function " + name)
    start = source.index("{", match.start())
    depth = 1
    end = start + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <limits.h>
enum nl80211_band { NL80211_BAND_2GHZ, NL80211_BAND_5GHZ, NL80211_BAND_6GHZ=3, NUM_NL80211_BANDS=6 };
enum nl80211_mfp { NL80211_MFP_NO, NL80211_MFP_REQUIRED, NL80211_MFP_OPTIONAL };
enum nl80211_sae_pwe_mechanism { NL80211_SAE_PWE_UNSPECIFIED, NL80211_SAE_PWE_HUNT_AND_PECK, NL80211_SAE_PWE_HASH_TO_ELEMENT, NL80211_SAE_PWE_BOTH };
#define WLAN_AKM_SUITE_SAE 0x000fac08
#define WLAN_AKM_SUITE_OWE 0x000fac12
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define smp_load_acquire(p) (*(p))
#define container_of(p,t,f) ((t*)((char*)(p)-offsetof(t,f)))
#define BIT(n) (1U << (n))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*(x)))
#define GFP_ATOMIC 0
#define IEEE80211_CHAN_DISABLED 1
#define IEEE80211_CHAN_RADAR 2
#define IEEE80211_CHAN_NO_IR 4
#define NL80211_CHAN_WIDTH_20 0
#define NL80211_CHAN_WIDTH_40 1
#define NL80211_CHAN_WIDTH_80 2
#define WLAN_REASON_UNSPECIFIED 1
#define WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY 4
#define WLAN_EID_HT_CAPABILITY 45
struct element {u8 id,datalen,data[];};
#define WLAN_EID_EXTENSION 255
#define WLAN_EID_EXT_HE_CAPABILITY 35
#define dev_info(...) ((void)0)
#define dev_dbg_ratelimited(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define spin_lock_irqsave(l,f) ((void)(l),(f)=0)
#define spin_unlock_irqrestore(l,f) ((void)(l),(void)(f))
#define spin_lock(l) ((void)(l))
#define spin_unlock(l) ((void)(l))
#define msecs_to_jiffies(x) (x)
#define time_before(a,b) ((long)((a)-(b))<0)
#define DIV_ROUND_UP(n,d) (((n)+(d)-1)/(d))
static unsigned command_locked;
#define mutex_unlock(l) do { assert(command_locked); command_locked=0; } while(0)
#define mutex_lock(l) do { assert(!command_locked); command_locked=1; } while(0)
static u16 get_unaligned_le16(const void*p){const u8*b=p;return b[0]|((u16)b[1]<<8);}
static u32 get_unaligned_le32(const void*p){const u8*b=p;return b[0]|((u32)b[1]<<8)|((u32)b[2]<<16)|((u32)b[3]<<24);}
static void put_unaligned_le16(u16 v,void*p){u8*b=p;b[0]=v;b[1]=v>>8;}
static void put_unaligned_le32(u32 v,void*p){u8*b=p;for(unsigned i=0;i<4;i++)b[i]=v>>(8*i);}
static bool ieee80211_he_capa_size_ok(const void*p,size_t n){return false;}
HEADERS
struct completion {unsigned done;};
static void complete(struct completion*c){if(c->done!=UINT_MAX)c->done++;}
static void complete_all(struct completion*c){c->done=UINT_MAX;}
static void reinit_completion(struct completion*c){c->done=0;}
struct work_struct;
struct ieee80211_channel {int center_freq;};
struct cfg80211_chan_def {struct ieee80211_channel *chan;int width,center_freq1;};
struct test_band {struct {bool ht_supported;}ht_cap;};
struct test_wiphy {struct test_band *bands[NUM_NL80211_BANDS];};
static struct test_wiphy wiphy;
struct mt7932 {
 void *pdev,*netdev;struct test_wiphy *wiphy;int response_lock,command_mutex,cal_work,disconnect_work,data_lock;
 bool data_ready,roam_moving;u32 station_rx_packets;unsigned long station_rx_time;u16 station_beacon_tu;
 struct completion assoc_start,assoc_done,discovery_done,disconnect_done,cal_response,response;
 bool connecting,connected,disconnecting,disconnect_seen,disconnect_event_received;
 bool disconnect_local,connect_open,assoc_request_seen,peer_valid,link_failed,stopping,rf_ready;
 u8 connect_channel,connect_center,peer_wtbl,smart_version,waiting;
 enum nl80211_band connect_band; u8 six_assoc_power[2][1020]; bool six_legacy_assoc;
 int waiting_eid;
 struct cfg80211_chan_def connect_chandef;
 struct test_band band2,band5;
 int connect_error;u16 disconnect_reason;
 u8 assoc_request_ies[512];unsigned assoc_request_ie_len;u32 connection_generation,event_cipher;
 struct mt7932_peer_ids peer_ids;
 struct mt7932_cal_completion cal_state;
 u8 cal_requests[32][16],cal_request_seq[32];unsigned cal_request_head,cal_request_count;
 u8 reply[MT7932_RX_CAPACITY];size_t reply_length;u64 packets;
};
static int ieee80211_channel_to_frequency(int ch,enum nl80211_band band){return (band==NL80211_BAND_2GHZ?2407:band==NL80211_BAND_5GHZ?5000:5950)+5*ch;}
static bool usable=true;
static unsigned cal_calls,queue_calls,scheduled;
static unsigned long jiffies;
static bool retried;
static unsigned first_elapsed,cal_elapsed,queue_elapsed,final_budget;
static int queue_error,cal_error;
static struct ieee80211_channel channel;
static struct ieee80211_channel *ieee80211_get_channel(void*w,int f){channel.center_freq=f;return &channel;}
static bool cfg80211_chandef_valid(const struct cfg80211_chan_def*d){return true;}
static bool cfg80211_chandef_usable(void*w,const struct cfg80211_chan_def*d,int flags){return usable;}
static bool mt7932_assoc_phy_valid(const void*b,const void*p,size_t n){return true;}
static const struct element *cfg80211_find_ext_elem(int id,const void*p,size_t n){return NULL;}
/* The HT lookup feeds a debug log; it walks only the exported request IEs. */
static unsigned ht_lookups;static const void *ht_found;
static const struct element *cfg80211_find_elem(int id,const void*p,size_t n){
 const u8*b=p;size_t o=0;assert(id==WLAN_EID_HT_CAPABILITY&&p&&n<=512);ht_lookups++;ht_found=NULL;
 while(n-o>=2&&b[o+1]<=n-o-2){if(b[o]==id)return ht_found=b+o;o+=2+b[o+1];}
 return NULL;
}
static void netif_carrier_off(void*n){}
static void netif_stop_queue(void*n){}
static void schedule_work(void*w){scheduled++;}
static void mt_scan_event(struct mt7932*m,const struct mt7932_event*e){}
static void mt_bss_presence(struct mt7932*m,const struct mt7932_event*e){}
#define IS_ENABLED(option) 0
static void mt_awdl_event(struct mt7932*m,const struct mt7932_event*e){}
static void mt_data_complete(struct mt7932*m,const u8*p,size_t n){}
static void mt_packet_receive(struct mt7932*m,const u8*p,size_t n){}
static void mt_hif_event(struct mt7932*m,const struct mt7932_event*e){}
static void mt_wow_reason_event(struct mt7932*m,const struct mt7932_event*e){}
static void mt_rf_fail_locked(struct mt7932*m,int e){m->rf_ready=false;m->link_failed=true;m->cal_state.error=e;m->cal_state.active=false;m->connect_error=e;complete_all(&m->assoc_done);complete_all(&m->cal_response);}
static unsigned recoveries,reports,retires;static int quiesce_ret,retire_ret;static u16 reported_reason;static bool reported_local;
static void mt_request_recovery(struct mt7932*m,const char*why){assert(m->link_failed&&why&&*why);recoveries++;}
static int mt_scan_quiesce(struct mt7932*m){return quiesce_ret;}
static int mt_retire_connection(struct mt7932*m){retires++;if(retire_ret)m->link_failed=true;return retire_ret;}
static void mt_report_disconnected(struct mt7932*m,u16 reason,bool local,int gfp){reports++;reported_reason=reason;reported_local=local;}
static void mt_reg_apply_deferred_locked(struct mt7932*m){}
CHANNEL
LINK
LOSS
RECEIVE
static void receive(struct mt7932*m,u8 eid,u8 seq,const u8*body,size_t n,u32 selector){
 u8 packet[600]={};size_t offset=eid==0xee?48:36,total=offset+n;
 assert(total<=sizeof(packet));put_unaligned_le32((7U<<27)|total,packet);
 put_unaligned_le16(total-24,packet+24);packet[28]=eid;packet[29]=seq;
 if(eid==0xee)put_unaligned_le32(selector,packet+40);
 memcpy(packet+offset,body,n);mt_receive(m,packet,total);
}
static void start(struct mt7932*m,u8 ch){u8 b[24]={};put_unaligned_le32(mt7932_channel_band(m->connect_band,ch),b+12);b[17]=ch;receive(m,0xee,0,b,sizeof(b),0x4d);}
static void report(struct mt7932*m,bool valid){u8 b[52]={};b[11]=mt7932_channel_band(m->connect_band,m->connect_channel);b[12]=m->connect_channel;b[17]=m->peer_ids.wtbl[0];b[20]=valid?m->peer_ids.station[0]:12;b[21]=m->peer_ids.wtbl[1];put_unaligned_le32(0x000fac04,b+48);receive(m,0xee,0,b,sizeof(b),0x40);}
static void done(struct mt7932*m){u8 b[2]={};receive(m,0xee,0,b,sizeof(b),0x42);}
static void cal_null(struct mt7932*m,u8 seq){u8 b[20]={};b[3]=1;receive(m,0xd6,seq,b,sizeof(b),0);}
static int mt_association_calibration(struct mt7932*m){
 assert(command_locked);cal_calls++;jiffies+=cal_elapsed;if(cal_error)return cal_error;
 int e=mt7932_cal_begin(&m->cal_state,1);if(e)return e;
 cal_null(m,181);assert(m->cal_state.done&&m->cal_state.received==1);
 return m->cal_state.error;
}
static int mt_request(struct mt7932*m,u8 cid,bool runtime,bool set,bool wait,const void*body,size_t n){
 assert(command_locked&&cid==0x6b&&runtime&&set&&!wait&&n==32);
 queue_calls++;jiffies+=queue_elapsed;return queue_error;
}
enum scenario {SUCCESS,DUP_SUCCESS,DUP_BEFORE_SUCCESS,DUP_TIMEOUT,CANCEL,LATE_ERROR,UNOWNED_REPORT,DUP_BAD_CHANNEL,NO_START,DONE_WITHOUT_REPORT,DISCONNECT,DUP_FLOOD};
static enum scenario which;
static struct mt7932 *active;
static unsigned first_waits,final_waits,total_wait_budget;
static unsigned long wait_for_completion_timeout(struct completion*c,unsigned long timeout){
 assert(!command_locked);total_wait_budget+=timeout;
 if(c==&active->assoc_start){assert(timeout==(retried?10000:5000));first_waits++;jiffies+=first_elapsed;if(which!=NO_START)start(active,active->connect_channel);if(which==DUP_BEFORE_SUCCESS)start(active,active->connect_channel);}
 else {assert(c==&active->assoc_done&&timeout==(retried?final_budget:10000));final_waits++;
  if(which==DUP_SUCCESS||which==DUP_TIMEOUT||which==DUP_BAD_CHANNEL)start(active,which==DUP_BAD_CHANNEL?6:active->connect_channel);
  if(which==DUP_FLOOD)for(unsigned i=0;i<1000;i++)start(active,active->connect_channel);
  if(which==CANCEL||which==LATE_ERROR){active->disconnecting=which==CANCEL;active->connect_error=which==CANCEL?-ECANCELED:-EIO;complete(c);report(active,true);done(active);}
  if(which==DISCONNECT){u8 b[5]={1,0,0,0,8};receive(active,0xee,0,b,sizeof(b),0x44);}
  if(which==UNOWNED_REPORT)report(active,false);
  if(which==DONE_WITHOUT_REPORT)done(active);
  if(which==SUCCESS||which==DUP_SUCCESS||which==DUP_BEFORE_SUCCESS){report(active,true);done(active);}
 }
 if(!c->done)return 0;if(c->done!=UINT_MAX)c->done--;return timeout;
}
#define time_after_eq(a,b) ((long)((a)-(b))>=0)
#define max_t(t,a,b) ((t)(a)>(t)(b)?(t)(a):(t)(b))
static int worker_wait(struct mt7932*m){int ret=0;u8 queue[32];bool retry=retried;unsigned long deadline=jiffies+10000,timeout;command_locked=1;
WAIT
out:
 assert(command_locked);command_locked=0;return ret?:m->connect_error;
}
static struct mt7932 fresh(void){return (struct mt7932){.netdev=(void*)1,.wiphy=&wiphy,.connecting=true,.rf_ready=true,.connect_channel=11,.smart_version=12,.peer_ids={{1,2},{3,4,5,6}}};}
static void reset(struct mt7932*m,enum scenario s){*m=fresh();active=m;which=s;cal_calls=queue_calls=scheduled=first_waits=final_waits=total_wait_budget=0;recoveries=reports=retires=0;quiesce_ret=retire_ret=0;queue_error=cal_error=0;usable=true;command_locked=0;jiffies=first_elapsed=cal_elapsed=queue_elapsed=0;retried=false;final_budget=10000;}
int main(void){struct mt7932 m;unsigned cases=0;
 for(enum scenario s=SUCCESS;s<=DUP_FLOOD;s++){
  reset(&m,s);int e=worker_wait(&m);
  if(s==SUCCESS||s==DUP_SUCCESS||s==DUP_BEFORE_SUCCESS)assert(!e&&m.peer_valid);
  else if(s==DUP_TIMEOUT||s==DUP_FLOOD||s==NO_START)assert(e==-ETIMEDOUT&&!m.peer_valid);
  else if(s==CANCEL)assert(e==-ECANCELED&&!m.peer_valid);
  else if(s==LATE_ERROR)assert(e==-EIO&&!m.peer_valid);
  else if(s==DISCONNECT)assert(e==-ECONNRESET&&!m.peer_valid);
  else if(s==UNOWNED_REPORT||s==DUP_BAD_CHANNEL)assert(e==-EPROTO&&!m.peer_valid);
  else assert(s==DONE_WITHOUT_REPORT&&e==-ECONNREFUSED&&!m.peer_valid);
  assert(first_waits==1&&final_waits==(s!=NO_START));assert(cal_calls==(s!=NO_START)&&queue_calls==cal_calls);assert(total_wait_budget<=15000);cases++;
 }
 reset(&m,SUCCESS);queue_error=-ENOSPC;assert(worker_wait(&m)==-ENOSPC&&cal_calls==1&&queue_calls==1&&!final_waits);cases++;
 reset(&m,SUCCESS);cal_error=-EIO;assert(worker_wait(&m)==-EIO&&cal_calls==1&&!queue_calls&&!final_waits);cases++;
 // A late result after terminal publication cannot acquire another peer.
 reset(&m,SUCCESS);m.connecting=false;report(&m,true);done(&m);assert(!m.peer_valid&&!m.assoc_done.done);cases++;
 // D7 queues only the firmware request, with its original sequence and words.
 reset(&m,SUCCESS);u8 request[16]={};put_unaligned_le32(0x1000,request+4);put_unaligned_le32(0,request+8);put_unaligned_le32(11,request+12);
 receive(&m,0xd7,90,request,16,0);assert(m.cal_request_count==1&&m.cal_request_seq[0]==90&&!memcmp(m.cal_requests[0],request,16)&&scheduled==1&&!cal_calls);cases++;
 assert(!mt7932_cal_begin(&m.cal_state,mt7932_cal_request_replies(request)));
 for(unsigned i=0;i<4;i++){cal_null(&m,90+i);assert(m.cal_state.received==i+1&&m.cal_state.done==(i==3));}
 assert(m.cal_state.done&&!m.link_failed&&m.cal_request_count==1);cases++;
 // D6 ownership precedes ordinary pending command sequence matching.
 reset(&m,SUCCESS);m.waiting=88;m.waiting_eid=0xd6;assert(!mt7932_cal_begin(&m.cal_state,1));cal_null(&m,88);assert(m.cal_state.done&&!m.reply_length&&!m.response.done);cases++;
 cal_null(&m,89);assert(m.link_failed&&m.cal_state.error==-EPROTO&&!m.rf_ready);cases++;
 reset(&m,SUCCESS);m.disconnecting=true;receive(&m,0xd7,91,request,16,0);assert(!m.cal_request_count&&!scheduled&&m.link_failed);cases++;
 reset(&m,SUCCESS);m.cal_request_count=32;receive(&m,0xd7,92,request,16,0);assert(m.cal_request_count==32&&m.link_failed&&!scheduled);cases++;
 reset(&m,SUCCESS);receive(&m,0xd7,93,request,15,0);assert(!m.cal_request_count&&m.link_failed);cases++;
 reset(&m,SUCCESS);usable=false;start(&m,11);assert(m.connect_error==-EPROTO&&m.assoc_start.done&&m.assoc_done.done&&!m.connect_center);cases++;
 // The retry shares one deadline across start, calibration, queue and done.
 reset(&m,SUCCESS);retried=true;first_elapsed=7000;cal_elapsed=1000;queue_elapsed=500;final_budget=1500;
 assert(!worker_wait(&m)&&first_waits==1&&final_waits==1&&jiffies==8500);cases++;
 reset(&m,SUCCESS);retried=true;first_elapsed=7000;cal_elapsed=3000;
 assert(worker_wait(&m)==-ETIMEDOUT&&cal_calls==1&&!queue_calls&&!final_waits);cases++;
 reset(&m,SUCCESS);retried=true;first_elapsed=7000;cal_elapsed=2000;queue_elapsed=1000;
 assert(worker_wait(&m)==-ETIMEDOUT&&queue_calls==1&&!final_waits);cases++;
 reset(&m,NO_START);retried=true;
 assert(worker_wait(&m)==-ETIMEDOUT&&total_wait_budget==10000&&!final_waits);cases++;
 // A well-formed firmware rejection is retryable; malformed ownership is not.
 reset(&m,SUCCESS);u8 rejected[52]={};put_unaligned_le16(1,rejected);
 receive(&m,0xee,0,rejected,sizeof(rejected),0x40);
 assert(m.connect_error==-ECONNREFUSED&&m.assoc_start.done&&m.assoc_done.done&&!m.peer_valid);cases++;
 // The generated request IEs are exported once; the HT log reads the copy.
 reset(&m,SUCCESS);u8 ies[32]={};put_unaligned_le32(28,ies);ies[4]=WLAN_EID_HT_CAPABILITY;ies[5]=26;for(unsigned i=0;i<26;i++)ies[6+i]=i+1;ht_lookups=0;
 receive(&m,0xee,0,ies,sizeof(ies),0x5e);
 assert(!m.connect_error&&m.assoc_request_seen&&m.assoc_request_ie_len==28&&!memcmp(m.assoc_request_ies,ies+4,28)&&ht_lookups==1&&ht_found==m.assoc_request_ies);cases++;
 reset(&m,SUCCESS);ies[5]=25;ht_lookups=0;receive(&m,0xee,0,ies,sizeof(ies),0x5e);
 assert(m.connect_error==-EPROTO&&!m.assoc_request_seen&&!ht_lookups&&m.assoc_start.done&&m.assoc_done.done);cases++;
 // EID 0x13: the firmware lost the station's AP; the host ends that connection, BSS 0 only.
 u8 lost[4]={0,3,3,0};
 #define CONNECTED(m) do{reset(&(m),SUCCESS);(m).connecting=false;(m).connected=true;(m).peer_valid=true;}while(0)
 CONNECTED(m);receive(&m,0x13,0,lost,4,0);
 assert(!m.connected&&m.disconnecting&&!m.disconnect_local&&m.disconnect_reason==WLAN_REASON_DISASSOC_DUE_TO_INACTIVITY&&scheduled==1);cases++;
 receive(&m,0x13,0,lost,4,0);assert(scheduled==1);cases++;
 CONNECTED(m);lost[0]=1;receive(&m,0x13,0,lost,4,0);assert(m.connected&&!m.disconnecting&&!scheduled);lost[0]=0;cases++;
 CONNECTED(m);receive(&m,0x13,0,lost,3,0);assert(m.connected&&!scheduled);cases++;
 // A frame from the AP within its beacon-loss span (7 beacon intervals: 717 ms at 100 TU) keeps it,
 // whatever cause the firmware gives; an older one, or no known interval, doesn't.
 CONNECTED(m);m.data_ready=true;m.station_rx_packets=1;m.station_beacon_tu=100;jiffies=5000;m.station_rx_time=4500;
 receive(&m,0x13,0,lost,4,0);assert(m.connected&&!scheduled);cases++;
 lost[1]=0x0e;receive(&m,0x13,0,lost,4,0);assert(m.connected&&!scheduled);lost[1]=3;cases++;
 CONNECTED(m);m.data_ready=true;m.station_rx_packets=1;m.station_beacon_tu=100;jiffies=5000;m.station_rx_time=4000;
 receive(&m,0x13,0,lost,4,0);assert(!m.connected&&scheduled==1);cases++;
 CONNECTED(m);m.data_ready=true;m.station_rx_packets=1;m.station_beacon_tu=0;jiffies=5000;m.station_rx_time=4999;
 receive(&m,0x13,0,lost,4,0);assert(!m.connected&&scheduled==1);cases++;
 // AWDL's station-record command answers with ID 0x13 too: the reply goes to its waiter.
 CONNECTED(m);m.waiting=7;m.waiting_eid=0x13;receive(&m,0x13,7,lost,4,0);
 assert(m.connected&&!scheduled&&m.reply_length&&m.response.done);cases++;
 // A disconnect whose scan never ends latches the link, asks for one reset, and is still reported.
 CONNECTED(m);m.connected=false;m.disconnecting=true;m.disconnect_reason=3;m.disconnect_local=true;quiesce_ret=-EBUSY;
 mt_disconnect_work((struct work_struct*)&m.disconnect_work);
 assert(m.link_failed&&recoveries==1&&!retires&&reports==1&&reported_reason==3&&reported_local&&!m.disconnecting);cases++;
 // A clean one asks for none; a failed retire latches it itself (its own request is in the retire).
 CONNECTED(m);m.connected=false;m.disconnecting=true;mt_disconnect_work((struct work_struct*)&m.disconnect_work);
 assert(!m.link_failed&&!recoveries&&retires==1&&reports==1);cases++;
 printf("PASS %u association event/wait/calibration ownership controls\n",cases);
}
'''


def main():
    parser=argparse.ArgumentParser()
    parser.add_argument("--source-root",type=Path,default=ROOT)
    parser.add_argument("--out",type=Path,required=True)
    parser.add_argument("--old-source",type=Path)
    args=parser.parse_args()
    root=args.source_root;out=args.out;out.mkdir(parents=True,exist_ok=True)
    link=(root/DRIVER/"link.c").read_text();rx=(root/DRIVER/"mt7932.c").read_text()
    parts={"CHANNEL":function(link,"mt_channel_event"),"LINK":function(link,"mt_link_event"),"RECEIVE":function(rx,"mt_receive")}
    parts["LOSS"]="\n\n".join(function(link,n) for n in ("mt_end_connection_locked","mt_beacon_timeout_event","mt_disconnect_work"))
    worker=function(link,"mt_connect_attempt")
    start=worker.index("\t/* Genuine D7 requests")
    parts["WAIT"]=worker[start:worker.index("\nout:",start)]
    headers=[]
    for n in ["protocol.h","channels.h","bandwidth.h","association.h"]:
        header = re.sub(r"^#include .*\n", "", (root/DRIVER/n).read_text(), flags=re.M)
        if n == "association.h":
            # These event fixtures do not use the sleeping crypto/admission helpers.
            start = header.index("/* The first AKM")
            end = header.index("struct mt7932_peer_ids")
            header = header[:start] + header[end:]
        headers.append(header)
    cal=(root/DRIVER/"calibration.h").read_text()
    headers.append(cal[cal.index("struct mt7932_cal_completion {"):cal.index("struct mt7932_cal_segment {")])
    # The receive path's other events, by their production IDs: the firmware's HIF and WoW reports.
    headers.append(re.search(r"^#define MT7932_CID_HIF_CTRL\b.*$", rx, re.M).group(0))
    headers.append(re.search(r"^#define MT7932_EID_WOW_REASON\b.*$", (root/DRIVER/"wow.h").read_text(), re.M).group(0))
    # The beacon-timeout event and its beacon-loss span.
    mh=(root/DRIVER/"mt7932.h").read_text()
    b=mh.index("#define MT7932_EID_BEACON_TIMEOUT");e=mh.index("\n}\n",mh.index("mt7932_beacon_loss_ms("))+3
    headers.append(mh[b:e])
    parts["HEADERS"]="\n".join(headers)
    candidates={"production":parts}
    if args.old_source:
        old=args.old_source.read_text();bad=dict(parts);bad["LINK"]=function(old,"mt_link_event");candidates["old-malformed-event"]=bad
    bad=dict(parts);bad["LINK"]=parts["LINK"].replace("!m->connecting || m->disconnecting || m->connect_error","!m->connecting");candidates["mutant-late-result"]=bad
    bad=dict(parts);bad["LINK"]=parts["LINK"].replace("!mt7932_assoc_resources(","false && !mt7932_assoc_resources(");candidates["mutant-peer-ownership"]=bad
    bad=dict(parts);bad["RECEIVE"]=parts["RECEIVE"].replace("event.eid == 0xd6","event.eid == 0xff");candidates["mutant-d6-ownership"]=bad
    bad=dict(parts);bad["WAIT"]=parts["WAIT"].replace("10000","20000");candidates["mutant-deadline"]=bad
    def mutant(name,key,old,new):
        b=dict(parts);assert old in parts[key],name;b[key]=parts[key].replace(old,new);candidates[name]=b
    mutant("mutant-beacon-any-bss","LOSS","if (body[0] || !m->connected","if ((body[0] && 0) || !m->connected")
    mutant("mutant-beacon-dropped","RECEIVE","event.eid == MT7932_EID_BEACON_TIMEOUT &&","0 &&")
    mutant("mutant-beacon-steals-reply","RECEIVE","!mt7932_event_matches(&event, m->waiting, m->waiting_eid)) {\n\t\tmt_beacon_timeout_event","1) {\n\t\tmt_beacon_timeout_event")
    mutant("mutant-beacon-ignores-rx","LOSS","rx_recent = m->station_rx_packets &&","rx_recent = false && m->station_rx_packets &&")
    mutant("mutant-beacon-span-doubled","LOSS","mt7932_beacon_loss_ms(m->station_beacon_tu)","mt7932_beacon_loss_ms(m->station_beacon_tu) * 2")
    mutant("mutant-disconnect-no-recovery","LOSS","\t\tmt_request_recovery(m, \"a scan didn't end before the connection did\");\n","")
    results={}
    for name,content in candidates.items():
        c=out/(name+".c");s=PREFIX
        for key,body in content.items():s=s.replace("\n"+key+"\n", "\n"+body+"\n")
        c.write_text(s);binary=out/name
        subprocess.run(["cc","-std=gnu11","-Wall","-Werror","-Wno-misleading-indentation","-Wno-unused-function","-Wno-unused-parameter","-fsanitize=address,undefined","-fno-omit-frame-pointer",str(c),"-o",str(binary)],check=True)
        run=subprocess.run([str(binary)],capture_output=True,text=True)
        (out/(name+".log")).write_text(run.stdout+run.stderr)
        results[name]={"exit":run.returncode,"parts_sha256":{k:hashlib.sha256(v.encode()).hexdigest() for k,v in content.items()},"binary_sha256":hashlib.sha256(binary.read_bytes()).hexdigest()}
        print(name,run.returncode,run.stdout.strip())
        assert (run.returncode==0)==(name=="production"),results
    (out/"results.json").write_text(json.dumps(results,indent=2)+"\n")


if __name__=="__main__":main()
