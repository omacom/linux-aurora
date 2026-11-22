#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise real scan batching and discovery waits with synthetic channels."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
DRIVER = ROOT / 'drivers/net/wireless/mediatek/mt7932'


def function(source, name):
    match = re.search(r'^(?:static )?(?:inline )?[^\n;]+\b' + re.escape(name) +
                      r'\([^;]*?\)\s*\{', source, re.M)
    if not match:
        raise ValueError('missing function ' + name)
    end = source.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end]


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8; typedef uint16_t u16; typedef uint32_t u32; typedef uint64_t u64;
enum nl80211_band { NL80211_BAND_2GHZ, NL80211_BAND_5GHZ, NL80211_BAND_6GHZ=3 };
#define BIT_ULL(n) (1ULL << (n))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof(*(x)))
#define IEEE80211_CHAN_DISABLED 1
#define IEEE80211_CHAN_NO_IR 2
#define IEEE80211_CHAN_RADAR 4
#define IEEE80211_CHAN_NO_20MHZ 8
#define NL80211_SCAN_FLAG_COLOCATED_6GHZ 0x4000
#define WLAN_AKM_SUITE_SAE 0x000fac08
#define WLAN_AKM_SUITE_OWE 0x000fac12
struct ieee80211_channel {enum nl80211_band band; u32 flags; u8 hw_value;};
struct ieee80211_supported_band {enum nl80211_band band; unsigned n_channels; struct ieee80211_channel *channels;};
struct cfg80211_ssid {u8 ssid[32]; unsigned ssid_len;};
struct cfg80211_scan_request {
 struct cfg80211_ssid *ssids; unsigned n_ssids,n_channels,flags,n_6ghz_params;
 const u8 *ie; size_t ie_len; struct ieee80211_channel *channels[80];
};
static u16 get_unaligned_le16(const void *p){const u8 *b=p;return b[0]|b[1]<<8;}
static void put_unaligned_le16(u16 v,void *p){u8 *b=p;b[0]=v;b[1]=v>>8;}
static void put_unaligned_le32(u32 v,void *p){u8 *b=p;for(unsigned i=0;i<4;i++)b[i]=v>>(8*i);}
#define READ_ONCE(x) (x)
#define spin_lock_irqsave(l,f) ((void)(l),(f)=0)
#define spin_unlock_irqrestore(l,f) ((void)(l),(void)(f))
#define msecs_to_jiffies(x) (x)
#define ktime_get_boottime_ns() 0ULL
#define dev_dbg(...) ((void)0)
static bool command_locked=true;
#define mutex_unlock(l) do {assert(command_locked);command_locked=false;} while(0)
#define mutex_lock(l) do {assert(!command_locked);command_locked=true;} while(0)
static void reinit_completion(int *c){*c=0;}
static void schedule_delayed_work(int *w,unsigned long t){assert(t==60000);}
struct mt7932 {
 struct cfg80211_scan_request *scan_request;
 unsigned scan_batch,scan_cursor,scan_band,scan_seq;
 int response_lock,scan_done,scan_timeout_work,command_mutex,discovery_done;
 bool scan_aborted,disconnecting,connected,scan_finished,rf_ready;
 bool connect_cancelled,discovering,discovery_finished,link_failed;
 u8 scan_home_channel; enum nl80211_band scan_home_band;
 u64 six_legacy_channels, connect_heard_since;
 unsigned connect_akm; int connect_error;
 struct ieee80211_supported_band band2,band5,band6;
 struct ieee80211_channel *connect_channel_req;
 u8 connect_ssid[32]; unsigned connect_ssid_length;
 void *connect_bss,*pdev;
};
static unsigned commands,waits,cancels,recoveries;
static bool silent;
static u8 bodies[4][0x4d4];
static struct mt7932 *active;
static bool mt_rf_allowed(struct mt7932 *m){return m->rf_ready;}
static int mt_request(struct mt7932 *m,u8 cid,bool runtime,bool set,bool reply,const void *body,size_t n){
 if(cid==0x1b){assert(command_locked&&runtime&&set&&!reply&&n==4);cancels++;return 0;}
 assert(command_locked&&cid==3&&runtime&&set&&!reply&&n==0x4d4&&commands<4);
 memcpy(bodies[commands++],body,n);return 0;
}
static unsigned long wait_for_completion_timeout(int *c,unsigned long timeout){
 assert(!command_locked&&c==&active->discovery_done&&commands);
 /* The firmware never ends the scan: neither the wait nor the cancel's. */
 if(silent&&timeout==1000){assert(cancels);return 0;}
 const u8 *b=bodies[commands-1];assert(!b[2]);
 assert(timeout==5000+233U*b[0x9f]);
 assert(timeout>220U*b[0x9f]+100);
 waits++;if(silent)return 0;active->discovery_finished=true;return 1;
}
static void *mt_connect_find_bss(struct mt7932 *m){return commands==2?m:NULL;}
/* Only a latch that a reset settles asks for one. */
static void mt_request_recovery(struct mt7932 *m,const char *why){assert(m->link_failed&&why&&*why);recoveries++;}
HEADERS
SUBMIT
DISCOVER
static void reset(struct mt7932 *m){
 memset(m,0,sizeof(*m));memset(bodies,0,sizeof(bodies));
 m->rf_ready=true;commands=waits=cancels=recoveries=0;silent=false;active=m;command_locked=true;
}
int main(void){
 struct mt7932 m;struct cfg80211_scan_request request={};
 struct cfg80211_ssid ssid={.ssid="hidden",.ssid_len=6};
 struct ieee80211_channel channels[59];
 for(unsigned i=0;i<59;i++){
  channels[i]=(struct ieee80211_channel){.band=NL80211_BAND_6GHZ,.hw_value=1+4*i};
  request.channels[i]=channels+i;
 }
 request.n_channels=59;request.ssids=&ssid;request.n_ssids=1;
 /* Even legacy channels must listen when RNR constraints are not serialized. */
 for(unsigned context=0;context<2;context++){
  reset(&m);m.scan_request=&request;m.six_legacy_channels=(1ULL<<59)-1;
  request.flags=context?0:NL80211_SCAN_FLAG_COLOCATED_6GHZ;
  request.n_6ghz_params=context?1:0;
  assert(mt_scan_submit_next(&m)==0);assert(commands==1&&bodies[0][0x9f]==32);
  assert(mt_scan_submit_next(&m)==0);assert(commands==2&&bodies[1][0x9f]==27);
  assert(mt_scan_submit_next(&m)==1);assert(commands==2);
  for(unsigned batch=0;batch<2;batch++){
   const u8 *b=bodies[batch];assert(!b[2]&&!b[3]&&!b[4]);
   assert(get_unaligned_le16(b+0x4c2)==220);
   for(unsigned i=0;i<b[0x9f];i++){
    assert(b[0xa0+2*i]==3&&b[0xa1+2*i]==1+4*(batch*32+i));
   }
  }
 }
 /* Ordinary hidden-SSID legacy probing remains active, with home context. */
 reset(&m);m.scan_request=&request;m.six_legacy_channels=(1ULL<<59)-1;
 m.connected=true;m.scan_home_channel=5;m.scan_home_band=NL80211_BAND_6GHZ;
 request.flags=request.n_6ghz_params=0;
 assert(mt_scan_submit_next(&m)==0);
 assert(bodies[0][2]==1&&bodies[0][3]==4&&bodies[0][4]==1);
 assert(!memcmp(bodies[0]+12,"hidden",6));
 assert(bodies[0][0x4cc]==5&&bodies[0][0x4cd]==3);
 /* NO_IR remains passive, even for an explicit legacy SSID. */
 reset(&m);m.scan_request=&request;m.six_legacy_channels=(1ULL<<59)-1;
 request.n_channels=1;channels[0].flags=IEEE80211_CHAN_NO_IR;
 assert(mt_scan_submit_next(&m)==0);assert(!bodies[0][2]);channels[0].flags=0;
 /* A class-only full-band internal discovery needs both 32/27-channel waits. */
 reset(&m);m.connect_akm=WLAN_AKM_SUITE_SAE;
 m.band6=(struct ieee80211_supported_band){.band=NL80211_BAND_6GHZ,.n_channels=59,.channels=channels};
 assert(mt_connect_discover(&m)==0);
 assert(commands==2&&waits==2&&!m.link_failed&&!m.discovering&&command_locked&&!recoveries);
 /* A discovery the firmware never ends latches the link and asks for one reset. */
 reset(&m);m.connect_akm=WLAN_AKM_SUITE_SAE;silent=true;
 m.band6=(struct ieee80211_supported_band){.band=NL80211_BAND_6GHZ,.n_channels=59,.channels=channels};
 assert(mt_connect_discover(&m)==-ETIMEDOUT);
 assert(commands==1&&cancels==1&&m.link_failed&&recoveries==1&&!m.discovering&&command_locked);
 puts("MT7932 RNR batching, directed scans and full-band discovery: pass");return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path, required=True)
    args = parser.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    frontend = (DRIVER / 'frontend.c').read_text()
    headers = '\n'.join(re.sub(r'^#include[^\n]*\n', '', (DRIVER / name).read_text(),
                               flags=re.M) for name in ('channels.h', 'scan.h'))
    parts = {'HEADERS': headers,
             'SUBMIT': function(frontend, 'mt_scan_submit_next'),
             'DISCOVER': function(frontend, 'mt_connect_discover')}
    variants = {'production': parts}
    bad = dict(parts)
    bad['HEADERS'] = bad['HEADERS'].replace(
        '(request->flags & NL80211_SCAN_FLAG_COLOCATED_6GHZ)', 'false').replace(
        'request->n_6ghz_params ||', 'false ||')
    variants['mutant-rnr-probes'] = bad
    bad = dict(parts)
    bad['DISCOVER'] = bad['DISCOVER'].replace('mt7932_scan_discovery_timeout(body)', '5000')
    variants['mutant-short-wait'] = bad
    bad = dict(parts)
    bad['DISCOVER'] = re.sub(r'\n\t*mt_request_recovery\([^;]*\);', '', bad['DISCOVER'])
    assert bad['DISCOVER'] != parts['DISCOVER']
    variants['mutant-no-recovery'] = bad
    results = {}
    for name, content in variants.items():
        source = PREFIX
        for key, value in content.items():
            source = source.replace('\n' + key + '\n', '\n' + value + '\n')
        c = args.out / (name + '.c')
        binary = args.out / name
        c.write_text(source)
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Werror', '-Wno-unused-function',
                        '-Wno-unused-variable', '-fsanitize=address,undefined',
                        '-fno-omit-frame-pointer', str(c), '-o', str(binary)], check=True)
        run = subprocess.run([str(binary.resolve())], capture_output=True, text=True)
        (args.out / (name + '.log')).write_text(run.stdout + run.stderr)
        results[name] = {'exit': run.returncode,
                         'source_sha256': hashlib.sha256(source.encode()).hexdigest()}
        print(name, json.dumps(results[name]), run.stdout.strip())
        if name == 'production':
            assert run.returncode == 0, run.stderr
        else:
            assert run.returncode == -6 and 'Assertion' in run.stderr, run.stderr
    (args.out / 'results.json').write_text(json.dumps(results, indent=2) + '\n')


if __name__ == '__main__':
    main()
