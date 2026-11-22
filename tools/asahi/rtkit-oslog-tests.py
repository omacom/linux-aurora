#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Check OSLog counter acknowledgements and inherited-session reset."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DRIVER = 'drivers/soc/apple/rtkit.c'


def function(source, name):
    match = re.search(r'^(?:static )?(?:void|int|bool) ' + name + r'\(', source, re.M)
    assert match, name
    opening = source.index('{', match.end())
    depth, end = 1, opening + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[match.start():end] + '\n'


PREFIX = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
typedef uint8_t u8; typedef uint64_t u64; typedef uint32_t u32;
#define GENMASK_ULL(h,l) ((~0ULL << (l)) & (~0ULL >> (63-(h))))
#define FIELD_GET(m,v) (((v) & (m)) >> __builtin_ctzll(m))
#define ARRAY_SIZE(x) (sizeof(x)/sizeof((x)[0]))
#define APPLE_RTKIT_MAX_ENDPOINTS 256
#define WRITE_ONCE(x,v) ((x)=(v))
/* As the shared Apple configuration builds it. */
#define CONFIG_APPLE_SOC_SLEEP 1
#define IS_ENABLED(option) (option)
#define cmpxchg(p,o,n) __sync_val_compare_and_swap((p),(o),(n))
#define spin_lock_irqsave(l,f) ((void)(l), (f)=0)
#define spin_unlock_irqrestore(l,f) ((void)(l), (void)(f))
struct apple_rtkit_shmem { size_t size; };
struct mailbox { int rx_lock; void (*rx)(void); };
struct delayed_work { int pending; };
struct apple_rtkit {
 void *dev, *wq;
 struct mailbox *mbox;
 struct apple_rtkit_shmem oslog_buffer, syslog_buffer, ioreport_buffer, crashlog_buffer;
 struct apple_rtkit_shmem kdebug_buffer0, kdebug_buffer1, coreanalytics_buffer, pmp_aux_buffer;
 int sync_ack_fifo;
 bool sync_syslog_ack, oslog_inherited, syslog_inherited, crashlog_inherited, crashed, shutting_down;
 bool oslog_echo;
 u64 oslog_owed;
 /* Read only by the J714s firmware 27 sleep; a failed echo sets it. */
 bool v27_rx_failed;
 int boot_result;
 unsigned iop_power_state, ap_power_state;
 unsigned long endpoints[4], started[4];
 char *syslog_msg_buffer;
 size_t syslog_n_entries, syslog_msg_size;
 int epmap_completion, iop_pwr_ack_completion, ap_pwr_ack_completion, coreanalytics_completion;
 struct delayed_work crashlog_late_work;
};
static unsigned sent, warnings, errors, requests, detachments, flushes, freed, checks;
static int send_error, setup_error, start_error;
static u64 sent_message;
static bool neo, m5;
static bool of_machine_is_compatible(const char *s)
{ return (!strcmp(s, "apple,t8140") && neo) || (!strcmp(s, "apple,t8142") && m5); }
#define kfifo_reset(f) (*(f)=0)
#define dev_dbg(...) ((void)0)
#define dev_warn(...) (++warnings)
#define dev_err(...) (++errors)
static void set_bit(unsigned n, unsigned long *b) { b[n/64] |= 1UL << (n%64); }
static void bitmap_zero(unsigned long *b, unsigned n) { memset(b,0,n/8); }
static void reinit_completion(int *c) { *c=0; }
static void complete_all(int *c) { *c=1; }
static void apple_rtkit_rx(void) {}
static void apple_rtkit_detach_rx(struct apple_rtkit *r) { ++detachments; r->shutting_down=true; }
static void flush_workqueue(void *w) { (void)w; ++flushes; }
#define cancel_delayed_work_sync(w) ((void)(w), false)
static void apple_rtkit_free_buffer(struct apple_rtkit *r, struct apple_rtkit_shmem *b)
{ (void)r; ++freed; b->size=0; }
#define kfree(p) ((void)(p))
static int apple_mbox_start(struct mailbox *m) { (void)m; return start_error; }
static int apple_rtkit_send_message(struct apple_rtkit *r, u8 ep, u64 msg, void *c, bool atomic)
{ (void)r; assert(ep==APPLE_RTKIT_EP_OSLOG && c==NULL && !atomic); ++sent; sent_message=msg; return send_error; }
static int apple_rtkit_common_rx_get_buffer(struct apple_rtkit *r,
 struct apple_rtkit_shmem *b, u8 ep, u64 msg)
{ assert(b==&r->oslog_buffer && ep==APPLE_RTKIT_EP_OSLOG);
 ++requests; if (!setup_error) b->size=FIELD_GET(APPLE_RTKIT_OSLOG_SIZE,msg); return setup_error; }
'''

MAIN = r'''
static void reset(void) { sent=warnings=errors=requests=0; send_error=setup_error=0; }
static void event(struct apple_rtkit *r, u64 msg, bool ack)
{
 reset(); apple_rtkit_oslog_rx(r,msg);
 assert(sent==(unsigned)ack && requests==0 && errors==0);
 if (ack) { assert(sent_message==msg && warnings==0); }
 else { assert(warnings==1); }
 ++checks;
}
int main(void)
{
 struct mailbox m={0}; struct apple_rtkit r={.mbox=&m};
 const u64 type=(u64)APPLE_RTKIT_OSLOG_LOG<<56;
 const uint32_t counters[]={0,1,4095,4096,UINT32_MAX,0,0x80000000};
 for (unsigned owner=0; owner<3; owner++) {
  r.oslog_buffer.size=owner==1?4096:0; r.oslog_inherited=false;
  if (owner==2) { apple_rtkit_mark_running(&r); assert(r.oslog_inherited); }
  for (unsigned i=0;i<ARRAY_SIZE(counters);i++) event(&r,type|counters[i],owner!=0);
  for (unsigned bit=32;bit<56;bit++) event(&r,type|(1ULL<<bit)|23,false);
  for (unsigned kind=0;kind<256;kind++) {
   if (kind==APPLE_RTKIT_OSLOG_BUFFER_REQUEST || kind==APPLE_RTKIT_OSLOG_LOG || (kind>=3 && kind<=5)) continue;
   event(&r,(u64)kind<<56,false);
  }
 }
 for (unsigned fail=0;fail<2;fail++) {
  memset(&r,0,sizeof(r));r.mbox=&m; reset();setup_error=fail?-5:0;
  const u64 req=((u64)APPLE_RTKIT_OSLOG_BUFFER_REQUEST<<56)|(4096ULL<<36);
  apple_rtkit_oslog_rx(&r,req); assert(requests==1 && sent==0);
  event(&r,type|UINT32_MAX,!fail);++checks;
 }
 r.oslog_buffer.size=4096;
 for (unsigned failure=0;failure<2;failure++) {
  reset();send_error=failure?-5:0;apple_rtkit_oslog_rx(&r,type|UINT32_MAX);
  assert(sent==1 && sent_message==(type|UINT32_MAX));assert(errors==failure && warnings==0);++checks;
 }
 for (unsigned fail=0;fail<2;fail++) {
  memset(&r,0,sizeof(r));r.mbox=&m;apple_rtkit_mark_running(&r);
  assert(r.oslog_inherited && (r.endpoints[0]&(1UL<<APPLE_RTKIT_EP_OSLOG)));
  struct apple_rtkit_shmem *buffers[]={&r.oslog_buffer,&r.syslog_buffer,&r.ioreport_buffer,
   &r.crashlog_buffer,&r.kdebug_buffer0,&r.kdebug_buffer1,&r.coreanalytics_buffer,&r.pmp_aux_buffer};
  for (unsigned i=0;i<ARRAY_SIZE(buffers);i++) buffers[i]->size=4096;
  r.crashed=true;r.syslog_msg_size=128;
  detachments=flushes=freed=0;start_error=fail?-5:0;
  assert(apple_rtkit_reinit(&r)==start_error);
  assert(!r.oslog_inherited && !r.syslog_inherited && !r.crashlog_inherited);
  assert(!r.crashed && !r.syslog_msg_size && freed==ARRAY_SIZE(buffers));
  // Every buffer of the old firmware session goes, the OSLog one included.
  for (unsigned i=0;i<ARRAY_SIZE(buffers);i++) assert(!buffers[i]->size);
  assert(r.endpoints[0]==(1UL<<APPLE_RTKIT_EP_MGMT));
  assert(detachments==(fail?2U:1U) && flushes==(fail?2U:1U));
  event(&r,type|1,false);++checks;
 }
 for (unsigned policy=0;policy<3;policy++) {
  neo=policy==0; m5=policy==1; r.sync_syslog_ack=policy==2;
  reset(); apple_rtkit_oslog_rx(&r,type|23);
  assert(sent==0 && warnings==0 && errors==0); ++checks;
 }
 printf("%u OSLog counter/channel/session controls passed\n",checks);
 return 0;
}
'''


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--ref', help='Git source revision to exercise')
    parser.add_argument('--out', type=Path)
    parser.add_argument('--mutate', choices=['admission', 'channel', 'counter', 'reset', 'buffer'])
    args = parser.parse_args()
    source = subprocess.check_output(['git', 'show', args.ref + ':' + DRIVER], cwd=ROOT, text=True) if args.ref else (ROOT / DRIVER).read_text()
    constants = '\n'.join(re.findall(r'^#define APPLE_RTKIT_OSLOG_.*$', source, re.M))
    constants += '\n' + '\n'.join(re.findall(r'enum \{.*?\n\};', source, re.S)[:2])
    # Older receivers have no LOG constant or inherited field writes.
    if not re.search(r'#define APPLE_RTKIT_OSLOG_LOG\s', constants):
        constants += '\n#define APPLE_RTKIT_OSLOG_LOG 2\n'
    # The session reset reinit() and a cold restart share, where the driver has one.
    names = ['apple_rtkit_oslog_rx', 'apple_rtkit_mark_running']
    # The echo's predicate, where the receiver is built on one.
    if 'static bool apple_rtkit_oslog_echoed(' in source:
        names[:0] = ['apple_rtkit_oslog_unanswered', 'apple_rtkit_oslog_echoed']
    if 'static void apple_rtkit_reset_session(' in source:
        names.append('apple_rtkit_reset_session')
    names.append('apple_rtkit_reinit')
    bodies = '\n'.join(function(source, name) for name in names)
    if args.mutate == 'admission':
        bodies = bodies.replace('(!rtk->oslog_buffer.size && !rtk->oslog_inherited)', 'false')
        bodies = bodies.replace('(rtk->oslog_buffer.size || rtk->oslog_inherited)', 'true')
    elif args.mutate == 'channel':
        bodies = bodies.replace('(msg & APPLE_RTKIT_OSLOG_LOG_RESERVED)', 'false')
    elif args.mutate == 'counter':
        bodies = bodies.replace('msg, NULL, false);', '(msg & ~0xffffffffULL) | ((u32)msg % 4096), NULL, false);')
    elif args.mutate == 'reset':
        bodies = bodies.replace('rtk->oslog_inherited = false;', 'rtk->oslog_inherited = true;')
    elif args.mutate == 'buffer':
        bodies = bodies.replace('apple_rtkit_free_buffer(rtk, &rtk->oslog_buffer);', '')
    # A mutation that no longer matches the driver would test nothing.
    assert not args.mutate or bodies != '\n'.join(
        function(source, name) for name in names), \
        'stale mutation: ' + args.mutate
    code = PREFIX.replace('struct apple_rtkit_shmem', constants + '\nstruct apple_rtkit_shmem', 1) + bodies + MAIN

    def run(out):
        out.mkdir(parents=True, exist_ok=True)
        cfile, exe = out / 'controls.c', out / 'controls'
        cfile.write_text(code)
        compiler = subprocess.run(['clang', '-std=gnu11', '-O1', '-g', '-fsanitize=address,undefined', '-fno-sanitize-recover=all', str(cfile), '-o', str(exe)], capture_output=True, text=True)
        (out / 'compile.log').write_text(compiler.stdout + compiler.stderr)
        compiler.check_returncode()
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        receipt = {'source_sha256': hashlib.sha256(source.encode()).hexdigest(), 'bodies_sha256': hashlib.sha256(bodies.encode()).hexdigest(), 'mutation': args.mutate, 'ref': args.ref, 'exit': result.returncode, 'stdout': result.stdout, 'stderr': result.stderr, 'limits': 'Mailbox, allocation and workqueue operations use observing callbacks; no firmware sleep test.'}
        (out / 'results.json').write_text(json.dumps(receipt, indent=2) + '\n')
        print(result.stdout or result.stderr)
        return result.returncode != 0

    if args.out:
        return run(args.out)
    with tempfile.TemporaryDirectory(prefix='rtkit-oslog-') as out:
        return run(Path(out))


if __name__ == '__main__':
    raise SystemExit(main())
