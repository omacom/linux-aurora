#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Run CLPC's production profile and managed cleanup callbacks on the host.

The SMC and kernel services are mocked, but stop, halt, profile_set and the
zETM writer are extracted unchanged. --old-ref must compile and fail the
specific missing-off assertion, rather than merely failing to build.
"""
import argparse
import os
from pathlib import Path
import re
import resource
import signal
import subprocess
import tempfile

ROOT = Path(__file__).resolve().parents[2]
DRIVER = 'drivers/soc/apple/clpc.c'


def function(source, name):
    match = re.search(r'^static (?:int|void) ' + name + r'\([^;]*?\n\{', source, re.M)
    if not match:
        raise ValueError('missing production function: ' + name)
    end = source.index('\n}', match.end()) + 2
    return source[match.start():end]


PREFIX = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8;
#define APPLE_CLPC_RAILS 2
#define FREQ_QOS_MIN_DEFAULT_VALUE 0
#define CPUFREQ_POLICY_NOTIFIER 1
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
#define SMC_KEY(x) 0x7a45544d
#define dev_warn(...) (warnings++)
enum platform_profile_option { PLATFORM_PROFILE_BALANCED, PLATFORM_PROFILE_LOW_POWER,
                               PLATFORM_PROFILE_PERFORMANCE };
struct mutex { bool held; };
struct delayed_work { bool pending; };
struct freq_qos_request { bool active; };
struct device { void *data; };
struct apple_smc { int unused; };
struct device_node { int unused; };
struct apple_clpc_gpu { int lock; struct delayed_work *wake; };
struct apple_clpc_ane { struct delayed_work *wake; };
struct cluster { struct freq_qos_request max_qos; };
struct vote { unsigned state; };
struct apple_clpc {
    struct device *dev;
    struct apple_smc *smc;
    struct device_node *smc_np;
    struct apple_clpc_gpu *gpu;
    struct apple_clpc_ane *ane;
    struct mutex sample_lock;
    struct delayed_work work, idle_work;
    bool stopped, high_power, high_power_sent, policy_nb_added, low_power;
    u8 high_power_seq;
    enum platform_profile_option profile;
    unsigned long power_next, ane_until;
    unsigned ane_qos_khz, nr_clusters, agent, mask, saved_floor;
    struct freq_qos_request ane_qos;
    struct cluster clusters[2];
    struct vote vote[APPLE_CLPC_RAILS];
    void *pmc, *debugfs;
    int policy_nb;
};
static struct mutex apple_clpc_clients_lock;
static unsigned long jiffies = 123;
static int system_freezable_power_efficient_wq;
static struct apple_clpc *active;
static unsigned cancels, smc_writes, warnings, queued, qos_removed;
static bool detached, debug_removed, floor_restored, notifier_removed, node_put, freed;
static bool cleanup;
static int smc_result;
static u8 last_record[120];
static void mutex_lock(struct mutex *m) { assert(!m->held); m->held = true; }
static void mutex_unlock(struct mutex *m) { assert(m->held); m->held = false; }
static void dev_set_drvdata(struct device *d, void *p) {
    assert(apple_clpc_clients_lock.held); d->data = p;
}
static void *dev_get_drvdata(struct device *d) { return d->data; }
#define spin_lock_irqsave(lock, flags) do { (void)(lock); (flags) = 0; } while (0)
#define spin_unlock_irqrestore(lock, flags) do { (void)(lock); (void)(flags); } while (0)
static void apple_clpc_ane_detach(struct apple_clpc_ane *a) {
    assert(!active->dev->data && (!active->gpu || !active->gpu->wake));
    if (a) a->wake = NULL;
    detached = true;
}
static void debugfs_remove_recursive(void *d) { (void)d; assert(detached); debug_removed = true; }
static void cancel_delayed_work_sync(struct delayed_work *w) {
    assert(active->stopped && debug_removed && !active->sample_lock.held);
    w->pending = false; cancels++;
}
static void apple_pmc_floor_update(void *p, unsigned a, unsigned mask, unsigned saved) {
    (void)p; (void)a; (void)mask; (void)saved;
    assert(cancels == 4 && !active->work.pending && !active->idle_work.pending);
    floor_restored = true;
}
static int freq_qos_update_request(struct freq_qos_request *q, int value) {
    assert(q == &active->ane_qos && value == FREQ_QOS_MIN_DEFAULT_VALUE);
    return 0;
}
static bool freq_qos_request_active(struct freq_qos_request *q) { return q->active; }
static void freq_qos_remove_request(struct freq_qos_request *q) {
    assert(!active->sample_lock.held); q->active = false; qos_removed++;
}
static void cpufreq_unregister_notifier(int *n, int kind) {
    (void)n; assert(kind == CPUFREQ_POLICY_NOTIFIER && floor_restored);
    assert(!active->sample_lock.held); notifier_removed = true;
}
static void of_node_put(struct device_node *n) {
    (void)n; assert(notifier_removed && qos_removed == 3); node_put = true;
}
static void apple_clpc_free_clusters(struct apple_clpc *c) {
    (void)c; assert(node_put); freed = true;
}
static bool apple_clpc_smc(struct apple_clpc *c) { return c->smc != NULL; }
static void put_unaligned_le32(uint32_t v, u8 *p) {
    for (unsigned i = 0; i < 4; i++) p[i] = v >> (8 * i);
}
static int apple_smc_write(struct apple_smc *s, unsigned key, const u8 *p, unsigned size) {
    assert(s && key == SMC_KEY(zETM) && size == sizeof(last_record));
    assert(active->sample_lock.held && !node_put && !freed);
    if (cleanup) {
        assert(detached && debug_removed && floor_restored && cancels == 4);
        assert(!active->dev->data && !active->gpu->wake && !active->ane->wake);
        assert(active->stopped && !notifier_removed && !qos_removed);
        assert(active->vote[0].state == 0 && active->vote[1].state == 0);
        assert(active->ane_qos_khz == 0 && active->ane_until == jiffies);
    }
    memcpy(last_record, p, size); smc_writes++;
    return smc_result;
}
static void mod_delayed_work(int wq, struct delayed_work *w, unsigned delay) {
    (void)wq; assert(!delay && !active->sample_lock.held); w->pending = true; queued++;
}
'''

CASES = r'''
static struct device dev;
static struct apple_smc smc;
static struct apple_clpc_gpu gpu;
static struct apple_clpc_ane ane;
static struct apple_clpc c;
static void reset(void) {
    memset(&c, 0, sizeof(c)); memset(&gpu, 0, sizeof(gpu)); memset(&ane, 0, sizeof(ane));
    active = &c; dev.data = &c; c.dev = &dev; c.smc = &smc;
    c.gpu = &gpu; c.ane = &ane; gpu.wake = &c.work; ane.wake = &c.work;
    c.work.pending = c.idle_work.pending = true;
    c.policy_nb_added = c.ane_qos.active = true;
    c.nr_clusters = 2; c.clusters[0].max_qos.active = c.clusters[1].max_qos.active = true;
    c.ane_qos_khz = 1000; c.vote[0].state = c.vote[1].state = 3;
    c.high_power = true; c.profile = PLATFORM_PROFILE_BALANCED;
    cancels = smc_writes = warnings = queued = qos_removed = 0;
    detached = debug_removed = floor_restored = notifier_removed = node_put = freed = cleanup = false;
    smc_result = 0; memset(last_record, 0xff, sizeof(last_record));
}
static void record(bool on, unsigned seq) {
    for (unsigned i = 0; i < sizeof(last_record); i++) {
        unsigned expected = i == 0 ? 0x1d : i == 1 ? 1 : i == 2 ? seq :
                            on && i == 114 ? 0x80 : on && i == 115 ? 0x3f : 0;
        assert(last_record[i] == expected);
    }
}
static void stop(void) {
    cleanup = true; apple_clpc_stop(&c);
    assert(freed && node_put && notifier_removed && qos_removed == 3);
    assert(!c.sample_lock.held && !apple_clpc_clients_lock.held);
    assert(c.stopped && cancels == 4 && floor_restored && !dev.data);
}
int main(void) {
    /* A real successful performance callback sets the qualified sent state. */
    reset();
    assert(!apple_clpc_profile_set(&dev, PLATFORM_PROFILE_PERFORMANCE));
    assert(c.high_power_sent && c.profile == PLATFORM_PROFILE_PERFORMANCE && queued == 1);
    record(true, 1);
    stop();
    assert(smc_writes == 2); /* Missing-off negative control must fail here. */
    record(false, 2); assert(!c.high_power_sent && !warnings);

    /* Both capability and a successfully sent on mode qualify cleanup. */
    for (unsigned supported = 0; supported < 2; supported++)
        for (unsigned sent = 0; sent < 2; sent++) {
            reset(); c.high_power = supported; c.high_power_sent = sent;
            stop(); assert(smc_writes == (supported && sent));
            assert(c.high_power_sent == (sent && !supported));
        }
    /* Profile intent alone, including a failed on write, cannot trigger off. */
    reset(); smc_result = -EIO;
    assert(apple_clpc_profile_set(&dev, PLATFORM_PROFILE_PERFORMANCE) == -EIO);
    assert(!c.high_power_sent && c.profile == PLATFORM_PROFILE_BALANCED && !queued);
    stop(); assert(smc_writes == 1 && warnings == 1);

    /* Automatic already withdrew on; removal must not send it twice. */
    reset(); assert(!apple_clpc_profile_set(&dev, PLATFORM_PROFILE_PERFORMANCE));
    assert(!apple_clpc_profile_set(&dev, PLATFORM_PROFILE_LOW_POWER));
    assert(!c.high_power_sent && c.low_power); record(false, 2);
    stop(); assert(smc_writes == 2);

    /* Failure to send off keeps the acknowledged mode and still frees resources. */
    reset(); assert(!apple_clpc_profile_set(&dev, PLATFORM_PROFILE_PERFORMANCE));
    smc_result = -EIO; stop();
    assert(smc_writes == 2 && c.high_power_sent && warnings == 1); record(false, 2);
    reset(); c.high_power_sent = true; c.smc = NULL;
    stop(); assert(!smc_writes && c.high_power_sent && !c.high_power_seq);
    /* The record sequence is an eight-bit counter, including during cleanup. */
    reset(); c.high_power_sent = true; c.high_power_seq = 255;
    stop(); record(false, 0); assert(!c.high_power_sent);
    puts("CLPC High Power: profile state, cleanup ordering, zETM off and SMC failures passed");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--old-ref', help='compile a pre-fix driver and require the missing-off assertion')
    args = parser.parse_args()
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    source = (subprocess.check_output(['git', 'show', args.old_ref + ':' + DRIVER],
                                     cwd=ROOT, text=True) if args.old_ref else (ROOT / DRIVER).read_text())
    # This is the action whose callback runs after devres removes the profile.
    assert source.index('devm_add_action_or_reset(dev, apple_clpc_stop, clpc)') < source.index(
        'devm_platform_profile_register(dev,')
    defines = '\n'.join(re.findall(r'^#define APPLE_CLPC_(?:ZETM_\w+|FLOAT_ONE)\s+[^\n]+', source, re.M))
    body = '\n\n'.join(function(source, n) for n in (
        'apple_clpc_send_high_power', 'apple_clpc_profile_set', 'apple_clpc_halt', 'apple_clpc_stop'))
    with tempfile.TemporaryDirectory(prefix='clpc-high-power-') as tmp:
        out = args.out or Path(tmp)
        out.mkdir(parents=True, exist_ok=True)
        c, exe = out / 'clpc-high-power.c', out / 'clpc-high-power'
        c.write_text(PREFIX + defines + '\n' + body + CASES)
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c11', '-Wall', '-Wextra', '-Werror',
                        str(c), '-o', str(exe)], check=True)
        result = subprocess.run([str(exe)], capture_output=True, text=True)
        (out / 'run.log').write_text(result.stdout + result.stderr)
        if args.old_ref:
            if result.returncode != -signal.SIGABRT or 'smc_writes == 2' not in result.stderr:
                raise SystemExit('old source did not fail the missing-off runtime assertion:\n' +
                                 result.stdout + result.stderr)
            print('CLPC High Power: pre-fix source compiled and failed the missing-off assertion')
        elif result.returncode:
            raise SystemExit(result.stdout + result.stderr)
        else:
            print(result.stdout, end='')


if __name__ == '__main__':
    main()
