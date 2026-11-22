#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run production CLPC demand/hold logic with controlled PMGR reads and time."""
from pathlib import Path
import os
import re
import subprocess
import tempfile

ROOT = Path(os.environ.get('CLPC_ROOT', Path(__file__).resolve().parents[2]))


def item(text, signature):
    start = text.index(signature)
    begin = text.index('{', start)
    depth, end = 1, begin + 1
    while depth:
        depth += (text[end] == '{') - (text[end] == '}')
        end += 1
    return text[start:end]


def main():
    source = (ROOT / 'drivers/soc/apple/clpc.c').read_text()
    header = (ROOT / 'include/linux/soc/apple/pmc.h').read_text()
    rail_shift = re.search(r'^#define APPLE_PMC_RAIL_SHIFT\(rail\).*$', header, re.M).group(0)
    dense_samples = re.search(r'^#define APPLE_CLPC_GPU_DENSE_SAMPLES\s.*$', source, re.M).group(0)
    reasons = '\n'.join(re.findall(r'^#define APPLE_CLPC_REASON_\w+\s.*$', source, re.M))
    types = '\n'.join(item(source, s) + ';' for s in [
        'struct apple_clpc_hw {', 'struct apple_clpc_vote {'])
    tables = '\n'.join(item(source, 'static const struct apple_clpc_hw ' + n + ' =') + ';'
                       for n in ['apple_clpc_hw_t8132', 'apple_clpc_hw_t8140',
                                 'apple_clpc_hw_t6040', 'apple_clpc_hw_t6041'])
    methods = '\n'.join(item(source, s) for s in [
        'static unsigned int apple_clpc_vote(', 'static u32 apple_clpc_floor(',
        'static bool apple_clpc_gpu_streaming(',
        'static bool apple_clpc_gpu_dense_step(',
        'static bool apple_clpc_gpu_dense(',
        'static bool apple_clpc_gpu_limited(',
        'static void apple_clpc_gpu_floor_count(',
        'static bool apple_clpc_gpu_input_demand(',
        'static bool apple_clpc_ane_input_demand(',
        'static void apple_clpc_ane_cluster(',
        'static void apple_clpc_sample('])
    code = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint32_t u32;
typedef uint64_t u64;
#define APPLE_CLPC_LEVELS 4
#define APPLE_CLPC_HOLD_MS 32
#define APPLE_CLPC_SAMPLE_MS 4
#define APPLE_CLPC_GPU_BACKOFF_MS 100
#define HZ 1000
#define max(a,b) ((a) > (b) ? (a) : (b))
#define GENMASK(h,l) (((1u << ((h)-(l)+1))-1) << (l))
#define BIT(n) (1u << (n))
#define APPLE_PMC_RAIL_SHIFT(r) ((r)*8)
#define time_after_eq(a,b) ((long)((a)-(b)) >= 0)
#define time_after(a,b) ((long)((a)-(b)) > 0)
#define time_before(a,b) ((long)((a)-(b)) < 0)
#define msecs_to_jiffies(ms) (ms)
static int warned;
#define dev_warn_once(d, ...) ((void)(d), warned++)
enum apple_clpc_rail { APPLE_CLPC_DCS, APPLE_CLPC_SOC, APPLE_CLPC_RAILS };
''' + types + r'''
struct apple_clpc_gpu { bool active; u64 sequence; };
struct apple_clpc_ane { int lock; bool active, pending; };
#define spin_lock_irqsave(l, f) ((void)(l), (f) = 0)
#define spin_unlock_irqrestore(l, f) ((void)(l), (void)(f))
struct apple_clpc_cluster { unsigned int request[APPLE_CLPC_RAILS]; };
// cpufreq QoS stand-ins: one policy, the last requested minimum is recorded.
#define FREQ_QOS_MIN 1
#define FREQ_QOS_MIN_DEFAULT_VALUE 0
struct freq_qos_request { bool active; int value; };
struct cpufreq_policy { int constraints; };
static struct cpufreq_policy policy;
static int qos_min = -1, qos_adds;
static bool freq_qos_request_active(struct freq_qos_request *r) { return r->active; }
static struct cpufreq_policy *cpufreq_cpu_get(int cpu) { (void)cpu; return &policy; }
static void cpufreq_cpu_put(struct cpufreq_policy *p) { (void)p; }
static int freq_qos_add_request(int *c, struct freq_qos_request *r, int type, int v) {
 (void)c; assert(type==FREQ_QOS_MIN && !r->active); r->active=true; r->value=v; qos_min=v; qos_adds++; return 1;
}
static int freq_qos_update_request(struct freq_qos_request *r, int v) { assert(r->active); r->value=v; qos_min=v; return 1; }
struct apple_clpc {
 const struct apple_clpc_hw *hw;
 struct apple_clpc_cluster clusters[1];
 struct apple_clpc_vote vote[APPLE_CLPC_RAILS];
 unsigned int nr_clusters, agent;
 unsigned long samples, next_scan, update_errors, gpu_read_errors;
 void *dev;
 void *covered, *pmc, *gpu_pmgr;
 u32 gpu_offset, mask, last_floor, last_reason;
 bool gpu_on, gpu_limited, gpu_floor_released;
 unsigned long gpu_on_since, gpu_sampled, gpu_limited_at;
 u64 gpu_sequence;
 unsigned int gpu_dense_streak;
 unsigned long gpu_floor_releases, gpu_floor_returns;
 unsigned long gpu_streaming_samples, gpu_dense_samples;
 u64 last_vote_ns;
 struct apple_clpc_gpu *gpu;
 struct apple_clpc_ane *ane;
 unsigned int ane_states[APPLE_CLPC_RAILS];
 u32 ane_cluster_khz;
 int ane_cpu;
 struct freq_qos_request ane_qos;
 unsigned int ane_qos_khz;
 unsigned long ane_until;
 bool low_power;
};
static unsigned long jiffies;
static u64 ktime_get_mono_fast_ns(void) { return jiffies * 1000000ull; }
static bool apple_clpc_gpu_active(struct apple_clpc_gpu *g, u64 *u, u64 *s) { *u=jiffies; *s=g->sequence; return g->active; }
static void *cpu_possible_mask;
static u32 pmgr_status, pmp_floor, published;
static int read_error, vote_error;
static bool queued_idle;
static bool cpumask_equal(void *a, void *b) { return a == b; }
static void apple_clpc_find_clusters(struct apple_clpc *c) { (void)c; }
static void apple_clpc_sample_cluster(struct apple_clpc *c, struct apple_clpc_cluster *cl) { (void)c; (void)cl; }
static int regmap_read(void *map, u32 offset, u32 *status) {
 assert(map); (void)offset; *status=pmgr_status; return read_error;
}
static u32 apple_pmc_floor_read(void *pmc, unsigned int agent) {
 (void)pmc; assert(agent==0); return pmp_floor;
}
static int apple_pmc_floor_update(void *pmc, unsigned int agent, u32 mask, u32 floor) {
 (void)pmc; assert(agent==1 && mask==3); published=floor; return vote_error;
}
static void apple_clpc_queue(struct apple_clpc *c, bool idle) { (void)c; queued_idle=idle; }
#define READ_ONCE(x) (x)
static void apple_clpc_profile_limit(struct apple_clpc *c) { (void)c; }
''' + dense_samples + '\n' + reasons + '\n' + tables + '\n' + methods + r'''
// T8140 ANE work holds its DRAM and fabric states while it runs, a request
// shorter than one sample is still seen once, and the hold then expires.
static void check_t8140_ane(void) {
 struct apple_clpc_ane ane={0};
 struct apple_clpc c={.hw=&apple_clpc_hw_t8140,.agent=1,.mask=3,.ane=&ane};
 c.ane_states[APPLE_CLPC_DCS]=c.hw->ane[APPLE_CLPC_DCS];
 c.ane_states[APPLE_CLPC_SOC]=c.hw->ane[APPLE_CLPC_SOC];
 read_error=vote_error=0; pmgr_status=0; pmp_floor=0; jiffies=1000; published=~0u;
 apple_clpc_sample(&c); assert(published==0 && c.last_reason==0);
 ane.active=true; jiffies=1004; apple_clpc_sample(&c);
 assert(published==0x34 && c.last_reason==32);
 ane.active=false; ane.pending=true; jiffies=1008; apple_clpc_sample(&c);
 assert(published==0x34 && c.last_reason==32 && !ane.pending);
 jiffies=1012; apple_clpc_sample(&c); assert(published==0x34 && c.last_reason==16);
 jiffies=1040; apple_clpc_sample(&c); assert(published==0 && queued_idle);
}

// T6040 ANE work also holds the second P cluster at its qualified frequency,
// added on first use, for the vote hold after the last request, then drops it.
static void check_t6040_ane(void) {
 struct apple_clpc_ane ane={0};
 struct apple_clpc c={.hw=&apple_clpc_hw_t6040,.agent=1,.mask=3,.ane=&ane};
 c.ane_states[APPLE_CLPC_DCS]=c.hw->ane[APPLE_CLPC_DCS];
 c.ane_states[APPLE_CLPC_SOC]=c.hw->ane[APPLE_CLPC_SOC];
 c.ane_cpu=9; c.ane_cluster_khz=c.hw->ane_cluster_khz;
 assert(c.hw->ane_cluster==2 && c.ane_cluster_khz==3756000);
 read_error=vote_error=0; pmgr_status=0; pmp_floor=0; jiffies=2000; published=~0u; qos_min=-1; qos_adds=0;
 apple_clpc_sample(&c); assert(published==0 && qos_adds==0 && queued_idle);
 ane.active=true; jiffies=2004; apple_clpc_sample(&c);
 assert(published==0x25 && c.last_reason==32 && qos_adds==1 && qos_min==3756000);
 ane.active=false; jiffies=2008; apple_clpc_sample(&c);
 assert(qos_min==3756000 && !queued_idle);
 jiffies=2036; apple_clpc_sample(&c); assert(published==0 && qos_min==0 && c.ane_qos_khz==0 && queued_idle);
 ane.active=true; jiffies=2040; apple_clpc_sample(&c); assert(qos_adds==1 && qos_min==3756000);
}

// T8132's G16 firmware keeps graphics powered while the GPU idles: the
// M5/F3 floor follows the work the GPU driver reports, as on T6040.
static void check_t8132(void) {
 struct apple_clpc_gpu gpu={0};
 struct apple_clpc c={.hw=&apple_clpc_hw_t8132,.agent=1,.mask=3,.gpu=&gpu};
 assert(!c.hw->gpu_demand);
 read_error=vote_error=0; pmgr_status=0xf0; pmp_floor=0; jiffies=0; published=~0u;
 apple_clpc_sample(&c); assert(published==0 && queued_idle && c.last_reason==0);
 gpu.active=true; jiffies=100;
 apple_clpc_sample(&c); assert(published==0x35 && c.last_reason==2);
 gpu.active=false; jiffies=131; apple_clpc_sample(&c);
 assert(published==0x35 && c.last_reason==16);
 jiffies=132; apple_clpc_sample(&c); assert(published==0 && queued_idle);
}

// T6041 (M4 Max) votes DCS 5 / fabric 3 for CPU and host GPU demand without a
// streak, samples on time while idle, preserves T6041's DCS 5 / fabric 2 ANE
// floor, and carries the 2808 MHz Low Power ceiling.
static void check_t6041(void) {
 struct apple_clpc_gpu gpu={0};
 struct apple_clpc_ane ane={0};
 struct apple_clpc c={.hw=&apple_clpc_hw_t6041,.agent=1,.mask=3,.gpu=&gpu,.ane=&ane};
 c.ane_states[APPLE_CLPC_DCS]=c.hw->ane[APPLE_CLPC_DCS];
 c.ane_states[APPLE_CLPC_SOC]=c.hw->ane[APPLE_CLPC_SOC];
 assert(!c.hw->gpu_demand && c.hw->gpu_input && c.hw->sustain==0 && c.hw->idle_on_time);
 assert(!apple_clpc_hw_t8132.idle_on_time && !apple_clpc_hw_t8140.idle_on_time && !apple_clpc_hw_t6040.idle_on_time);
 assert(c.hw->low_power_khz==2808000 && apple_clpc_hw_t6040.low_power_khz==0);
 read_error=vote_error=0; pmgr_status=0xf0; pmp_floor=0; jiffies=3000; published=~0u; qos_adds=0;
 apple_clpc_sample(&c); assert(published==0 && queued_idle && c.last_reason==0);
 gpu.active=true; jiffies=3004; apple_clpc_sample(&c);
 assert(published==0x35 && c.last_reason==2);
 gpu.active=false; jiffies=3035; apple_clpc_sample(&c);
 assert(published==0x35 && c.last_reason==16);
 jiffies=3036; apple_clpc_sample(&c);
 assert(published==0 && queued_idle);
 ane.active=true; jiffies=3040; apple_clpc_sample(&c);
 assert(published==0x25 && c.last_reason==32 && qos_adds==0);
 gpu.active=true; jiffies=3044; apple_clpc_sample(&c);
 assert(published==0x35 && c.last_reason==34);
 gpu.active=false; jiffies=3048; apple_clpc_sample(&c);
 assert(published==0x25 && c.last_reason==32);
 ane.active=false; jiffies=3080; apple_clpc_sample(&c);
 assert(published==0 && queued_idle);
}

// A failed power-state read still protects an active graphics consumer.
static void check_read_error(void) {
 struct apple_clpc c={.hw=&apple_clpc_hw_t8140,.agent=1,.mask=3,.gpu_pmgr=(void *)1};
 read_error=vote_error=0; pmp_floor=4; pmgr_status=0xf0; warned=0;
 for (jiffies=100; jiffies<=164; jiffies+=4) apple_clpc_sample(&c);
 assert(published==0x34 && c.last_reason==4);
 jiffies=168; pmgr_status=0; read_error=-EIO;
 apple_clpc_sample(&c); assert(published==0x34 && c.last_reason==4);
 // The failure is counted for debugfs and warned about once.
 assert(c.gpu_read_errors==1 && warned==1);
 jiffies=172; read_error=0; apple_clpc_sample(&c);
 assert(published==0x34 && c.last_reason==16);
 jiffies=204; apple_clpc_sample(&c); assert(published==0);
 assert(c.gpu_read_errors==1);
}

static void check_t8140(void) {
 struct apple_clpc c={.hw=&apple_clpc_hw_t8140,.agent=1,.mask=3,.gpu_pmgr=(void *)1};
 read_error=vote_error=0; pmp_floor=4; pmgr_status=0xf0;
 for (jiffies=100; jiffies<=164; jiffies+=4) {
  apple_clpc_sample(&c);
  assert(published==(jiffies<164 ? 0 : 0x34));
 }
 // The PMP's lower DCS vote suppresses the GPU floor during heavy work.
 pmp_floor=3;
 for (jiffies=168; jiffies<=204; jiffies+=4) apple_clpc_sample(&c);
 assert(published==0 && c.gpu_limited && !queued_idle);
 assert(c.gpu_floor_releases==1 && c.gpu_floor_returns==0);
 pmp_floor=4;
 for (jiffies=208; jiffies<=304; jiffies+=4) apple_clpc_sample(&c);
 assert(published==0x34 && c.last_reason==4);
 assert(c.gpu_floor_releases==1 && c.gpu_floor_returns==1);
 assert(c.gpu_streaming_samples && !c.gpu_dense_samples);
 jiffies=308; pmgr_status=0; apple_clpc_sample(&c);
 jiffies=340; apple_clpc_sample(&c); assert(published==0 && queued_idle);
}
// Dense light work: graphics powers off between submissions, so it never
// streams, but 2+ submissions in each of 4 periods earn the GPU floor.
static void check_t8140_dense(void) {
 struct apple_clpc_gpu gpu={0};
 struct apple_clpc c={.hw=&apple_clpc_hw_t8140,.agent=1,.mask=3,.gpu_pmgr=(void *)1,.gpu=&gpu};
 read_error=vote_error=0; pmp_floor=4; pmgr_status=0; published=~0u;
 jiffies=100; apple_clpc_sample(&c); assert(published==0 && queued_idle);
 for (jiffies=104; jiffies<=116; jiffies+=4) {
  gpu.sequence+=2; apple_clpc_sample(&c);
  assert(published==(jiffies<116 ? 0 : 0x34));
 }
 assert(c.last_reason==8 && c.gpu_dense_samples==1 && !c.gpu_streaming_samples);
 // One sparse period ends the streak; the vote holds, then drops.
 jiffies=120; gpu.sequence+=1; apple_clpc_sample(&c);
 assert(published==0x34 && c.last_reason==16);
 jiffies=148; apple_clpc_sample(&c); assert(published==0 && queued_idle);
 // A sample more than one period late does not count as a dense period.
 for (jiffies=152; jiffies<=164; jiffies+=4) { gpu.sequence+=2; apple_clpc_sample(&c); }
 assert(published==0x34);
 jiffies=176; gpu.sequence+=40; apple_clpc_sample(&c);
 assert(c.last_reason==16);
}

// Low power samples on time only where it has a CPU ceiling to keep
// (low_power_khz); a High Power-only profile stays deferrable when idle.
static void check_low_power(void) {
 struct apple_clpc_gpu gpu={0};
 struct apple_clpc_hw lp=apple_clpc_hw_t6040;
 struct apple_clpc c={.hw=&lp,.agent=1,.mask=3,.gpu=&gpu,.low_power=true};
 read_error=vote_error=0; pmgr_status=0xf0; published=~0u;
 jiffies=3000; apple_clpc_sample(&c); assert(published==0 && queued_idle);
 lp.low_power_khz=2808000;
 jiffies=3004; apple_clpc_sample(&c); assert(published==0 && !queued_idle);
 c.low_power=false;
 jiffies=3008; apple_clpc_sample(&c); assert(published==0 && queued_idle);
}

int main(void) {
 // T6040's supplier is always powered: it must NEVER substitute for work.
 struct apple_clpc_gpu gpu={0};
 struct apple_clpc c={.hw=&apple_clpc_hw_t6040,.agent=1,.mask=3,.gpu=&gpu};
 assert(!c.hw->gpu_demand);
 jiffies=0; read_error=vote_error=0; pmgr_status=0xf0; published=~0u;
 apple_clpc_sample(&c); assert(published==0 && queued_idle && c.last_reason==0);
 gpu.active=true; jiffies=100; apple_clpc_sample(&c);
 assert(published==0x25 && c.last_reason==2 && c.last_floor==0x25);
 assert(c.last_vote_ns==100000000ull);
 // Core/firmware supplier is still powered, but continuous idle ends demand.
 gpu.active=false; jiffies=131; apple_clpc_sample(&c);
 assert(published==0x25 && c.last_reason==16);
 jiffies=132; apple_clpc_sample(&c);
 assert(published==0 && queued_idle && c.last_reason==0);
 // A new submission reasserts demand; independent CPU requests survive release.
 c.nr_clusters=1; c.clusters[0].request[0]=2; c.clusters[0].request[1]=1;
 gpu.active=true; jiffies=200; apple_clpc_sample(&c);
 assert(published==0x25 && c.last_reason==3);
 gpu.active=false; jiffies=232; apple_clpc_sample(&c);
 assert(published==0x12 && c.last_reason==1);
 c.clusters[0].request[0]=c.clusters[0].request[1]=0;
 jiffies=264; apple_clpc_sample(&c); assert(published==0);
 check_t8132();
 check_t6041();
 check_read_error();
 check_t8140();
 check_t8140_dense();
 check_t8140_ane();
 check_low_power();
 check_t6040_ane();
 puts("PASS: T6040, T6041 and T8132 host demand; T8140/T604x ANE, streaming, dense work, read errors and PMP backoff; idle release");
}
'''
    code = code.replace('#define APPLE_PMC_RAIL_SHIFT(r) ((r)*8)', rail_shift)
    with tempfile.TemporaryDirectory(prefix='apple-clpc-gpu-') as tmp:
        root = Path(tmp)
        for name, text in [
            ('fixed', code),
            ('omit-host-demand', code.replace('if (apple_clpc_gpu_input_demand(clpc->hw, clpc->gpu, demand))',
                                              'if (false && apple_clpc_gpu_input_demand(clpc->hw, clpc->gpu, demand))')),            ('omit-ane-demand', code.replace('if (apple_clpc_ane_input_demand(clpc, demand))',
                                             'if (false && apple_clpc_ane_input_demand(clpc, demand))')),
            ('omit-ane-cluster', code.replace('apple_clpc_ane_cluster(clpc, reason & APPLE_CLPC_REASON_ANE, now);',
                                              '(void)apple_clpc_ane_cluster;')),
            ('ane-cluster-no-hold', code.replace('if (time_before(now, clpc->ane_until))',
                                                 'if (active)')),
            ('omit-streaming-demand', code.replace('bool streaming = apple_clpc_gpu_streaming(clpc, now);',
                                                   'bool streaming = (apple_clpc_gpu_streaming(clpc, now), false);')),
            ('omit-dense-demand', code.replace('if (streaming || dense) {', 'if (streaming || (dense && false)) {')),
            ('dense-ignores-late', code.replace('if (!threshold || late || submissions < threshold)',
                                                'if (!threshold || (late && false) || submissions < threshold)')),
            ('uncounted-floor-return', code.replace('clpc->gpu_floor_returns++;', 'clpc->gpu_floor_returns += 0;')),
            ('uncounted-read-error', code.replace('clpc->gpu_read_errors++;', 'clpc->gpu_read_errors += 0;')),
            ('omit-pmp-backoff', code.replace('bool released = apple_clpc_gpu_limited(clpc, now);',
                                              'bool released = (apple_clpc_gpu_limited(clpc, now), false);')),
            ('release-too-early', code.replace('msecs_to_jiffies(APPLE_CLPC_HOLD_MS)', '0')),
            ('retain-at-idle', code.replace('vote->state = 0;', '')),
        ]:
            # A mutation that no longer matches the driver tests nothing.
            assert name == 'fixed' or text != code, 'stale mutation: ' + name
            path = root / (name + '.c')
            path.write_text(text)
            binary = path.with_suffix('')
            subprocess.run([os.environ.get('CC', 'cc'), '-std=gnu11', '-Wall', '-Wextra',
                            '-Werror', str(path), '-o', str(binary)], check=True)
            run = subprocess.run([str(binary)], capture_output=True, text=True)
            if name == 'fixed':
                assert run.returncode == 0, run.stderr
                print(run.stdout.strip())
            else:
                assert run.returncode != 0, name
                print('PASS negative control:', name)


if __name__ == '__main__':
    main()
