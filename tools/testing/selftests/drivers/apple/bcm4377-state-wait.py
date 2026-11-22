#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise production firmware-state publication and phase-specific waits.

The CPU scheduler model injects IRQ updates, unrelated/spurious wakeups,
terminal states, signals and a fixed deadline. It does not qualify hardware.
"""
from pathlib import Path
import resource
import subprocess
import tempfile


def recovery_program(source):
    prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
struct pci_dev { unsigned refs; };
struct work_struct { unsigned unused; };
struct bcm4377_hw { bool disable_aspm; };
struct ring_state { unsigned value; };
struct hci_dev;
struct bcm4377_data {
    unsigned long reset_pending, dma_window_base;struct pci_dev *pdev;struct hci_dev *hdev;
    const struct bcm4377_hw *hw;bool needs_reset,own_reset;unsigned bootstage,rti_status;
    struct ring_state *ring_state;
    struct { unsigned generation,n_entries;unsigned long msgids[1];unsigned *events; } control_h2d_ring;
    struct { bool enabled; } control_ack_ring;
};
struct hci_dev { struct bcm4377_data *data; bool setup; };
struct bcm4377_reset_work { struct work_struct work; struct pci_dev *pdev; };
static unsigned allocations, queued, errors, pci_resets, irq_disables, irq_enables, boots;
static int reset_error;
static bool fail_alloc;
static void *allocated, *bcm4377_reset_wq = (void *)1;
static struct bcm4377_data *resetting;
#define HCI_SETUP 1
#define hci_get_drvdata(hdev) ((hdev)->data)
#define hci_dev_test_flag(hdev,flag) ((void)(flag),(hdev)->setup)
#define test_and_set_bit(bit,p) ({ unsigned long mask=1UL<<(bit); bool old=*(p)&mask; *(p)|=mask; old; })
#define clear_bit(bit,p) ((void)(bit),(*(p))=0)
static void bt_dev_err(struct hci_dev *h,const char *fmt,...) { assert(h && h->data->hdev==h && fmt);errors++; }
static void *allocate(unsigned size) { allocations++;return allocated=fail_alloc?NULL:calloc(1,size); }
#define kzalloc_obj(value) allocate(sizeof(value))
static struct pci_dev *pci_dev_get(struct pci_dev *p) { p->refs++;return p; }
static void bcm4377_reset_work(struct work_struct *w) { (void)w;abort(); }
#define INIT_WORK(w,fn) do { (void)(w);(void)(fn); } while (0)
static void queue_work(void *q,struct work_struct *w) { assert(q==bcm4377_reset_wq && w);queued++; }
#define dev_info(...) ((void)0)
#define dev_err(...) ((void)0)
#define dev_err_once(...) do { errors++; } while (0)
#define WRITE_ONCE(x,v) ((x)=(v))
#define bitmap_zero(p,n) do { assert((n)<=8*sizeof(*(p)));memset(p,0,sizeof(*(p))); } while (0)
static int pci_irq_vector(struct pci_dev *p,unsigned index) { assert(p && !index);return 1; }
static void disable_irq(int irq) { assert(irq==1);irq_disables++; }
static void enable_irq(int irq) { assert(irq==1);irq_enables++; }
static void bcm4377_disable_aspm(struct bcm4377_data *b) { (void)b; }
/* bcm4377_reset_prepare/done must see own_reset across the driver's own reset. */
static int pci_reset_function(struct pci_dev *p) { assert(p==resetting->pdev && resetting->own_reset);pci_resets++;return reset_error; }
static void msleep(unsigned ms) { assert(ms==100); }
static void pci_set_master(struct pci_dev *p) { assert(p); }
static int bcm4377_init_cfg(struct bcm4377_data *b) { (void)b;return 0; }
static int bcm4377_boot(struct bcm4377_data *b) { (void)b;boots++;return 0; }
static int bcm4377_setup_rti(struct bcm4377_data *b) { (void)b;return 0; }
'''
    body = definition(source, 'static void bcm4377_queue_rebind(')
    body += definition(source, 'static void bcm4377_hci_reset(struct hci_dev *hdev)\n{')
    body += definition(source, 'static int bcm4377_pci_reset(struct bcm4377_data *bcm4377)\n{')
    tests = r'''
int main(void)
{
    struct pci_dev pci={0};struct bcm4377_data b={.pdev=&pci};
    struct hci_dev h={.data=&b};b.hdev=&h;resetting=&b;
    b.dma_window_base=0x10000100000UL;
    bcm4377_hci_reset(&h);
    assert(!allocations && !queued && !pci.refs && errors==1 && b.reset_pending);
    bcm4377_hci_reset(&h);
    assert(!allocations && !queued && !pci.refs && errors==1);
    b.dma_window_base=b.reset_pending=0;errors=0;
    bcm4377_hci_reset(&h);
    assert(allocations==1 && queued==1 && pci.refs==1 && errors==1);
    free(allocated);allocated=NULL;
    b.reset_pending=0;h.setup=true;
    bcm4377_hci_reset(&h);assert(allocations==1 && queued==1);
    h.setup=false;fail_alloc=true;
    bcm4377_hci_reset(&h);assert(!b.reset_pending && allocations==2 && queued==1);
    struct bcm4377_hw hw={0};struct ring_state ring={.value=99};
    b.hw=&hw;b.ring_state=&ring;b.needs_reset=true;b.dma_window_base=0x10000100000UL;
    errors=0;
    assert(bcm4377_pci_reset(&b)==-EIO);
    assert(errors==1 && !pci_resets && !irq_disables && !irq_enables && !boots);
    assert(b.needs_reset && ring.value==99);
    b.dma_window_base=0;b.control_h2d_ring.n_entries=4;
    assert(!bcm4377_pci_reset(&b) && pci_resets==1 && irq_disables==1 && irq_enables==1);
    assert(!b.own_reset);
    assert(!b.needs_reset && !ring.value && boots==1);
    reset_error=-EIO;
    assert(bcm4377_pci_reset(&b)==-EIO && b.needs_reset && !b.own_reset);
    assert(pci_resets==2 && irq_disables==2 && irq_enables==2 && boots==1);
    return 0;
}
'''
    return prefix + body + tests


def definition(source, name):
    start = source.index(name)
    start = source.rfind('\n', 0, start) + 1
    brace = source.index('{', start)
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end]


def main():
    root = Path(__file__).resolve().parents[5]
    source = (root / 'drivers/bluetooth/hci_bcm4377.c').read_text()
    code = r'''
#include <assert.h>
#include <errno.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;
typedef struct { int unused; } wait_queue_head_t;
struct bcm4377_data { u32 bootstage, rti_status; wait_queue_head_t event; };
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define dev_dbg(...) ((void)0)
#define ERESTARTSYS 512
struct step { bool update; u32 bootstage, rti_status; int signal; };
static struct bcm4377_data *controller;
static struct step steps[16];
static unsigned count, ticks, wakes;
static u32 expected_boot, expected_rti;
static void wake_up_all(wait_queue_head_t *q)
{
    assert(q==&controller->event);
    /* Wakeup is a publication: both cached fields must already be current. */
    assert(controller->bootstage==expected_boot && controller->rti_status==expected_rti);
    wakes++;
}
static int inject_tick(void);
#define wait_event_interruptible_timeout(q,condition,budget) ({ \
    (void)&(q); long left=(budget), result=0; \
    while (!(condition) && left) { \
        int signal=inject_tick(); \
        if (signal) { result=signal; break; } \
        left--; \
    } \
    if (!result && (condition)) result=left ? left : 1; \
    result; \
})
'''
    code += definition(source, 'static void bcm4377_update_state(') + '\n'
    code += definition(source, 'static int bcm4377_wait_for_state(') + '\n'
    code += r'''
static int inject_tick(void)
{
    assert(ticks<16);
    struct step s=ticks<count ? steps[ticks] : (struct step){0};
    ticks++;
    if (s.signal) return s.signal;
    if (s.update) {
        expected_boot=s.bootstage; expected_rti=s.rti_status;
        bcm4377_update_state(controller,s.bootstage,s.rti_status);
    } else {
        expected_boot=controller->bootstage; expected_rti=controller->rti_status;
        wake_up_all(&controller->event); /* no state change */
    }
    return 0;
}
static void reset(struct bcm4377_data *b, u32 boot, u32 rti)
{
    *b=(struct bcm4377_data){.bootstage=boot,.rti_status=rti};
    controller=b; count=ticks=wakes=0; memset(steps,0,sizeof(steps));
    expected_boot=boot; expected_rti=rti;
}
static void update(u32 boot, u32 rti)
{ steps[count++]=(struct step){.update=true,.bootstage=boot,.rti_status=rti}; }
int main(void)
{
    struct bcm4377_data b;
    reset(&b,0,0);
    expected_boot=2; expected_rti=1;
    bcm4377_update_state(&b,2,1);
    assert(wakes==1);
    bcm4377_update_state(&b,2,1);
    assert(wakes==1); /* duplicate IRQ has no new state to publish */

    reset(&b,2,1);
    wake_up_all(&b.event); /* stale notification from an earlier phase */
    count++; update(3,1); update(3,2); /* spurious, unrelated boot, actual RTI2 */
    assert(bcm4377_wait_for_state(&b,true,2,5)==0 && ticks==3);

    reset(&b,0,0);
    update(1,0); update(2,0);
    assert(bcm4377_wait_for_state(&b,false,2,5)==0 && ticks==2);

    reset(&b,2,1);
    update(2,3);
    assert(bcm4377_wait_for_state(&b,true,2,5)==-ENODEV && ticks==1);
    reset(&b,0,0);
    update(4,0);
    assert(bcm4377_wait_for_state(&b,false,2,5)==-ENXIO && ticks==1);
    reset(&b,2,2);
    assert(bcm4377_wait_for_state(&b,true,1,5)==-ENODEV && !ticks);

    reset(&b,2,1);
    for (unsigned i=0; i<5; i++) update(2+(i%2),1);
    update(2,2); /* target after the deadline must not extend the wait */
    assert(bcm4377_wait_for_state(&b,true,2,5)==-ETIMEDOUT && ticks==5);

    reset(&b,2,1);
    count++; steps[count++]=(struct step){.signal=-ERESTARTSYS};
    assert(bcm4377_wait_for_state(&b,true,2,5)==-ERESTARTSYS && ticks==2);
    reset(&b,2,2);
    assert(bcm4377_wait_for_state(&b,true,2,5)==0 && !ticks);
    puts("BCM4377 state waits: publication order, stale/spurious/intermediate events, terminal states, fixed deadline and signals PASS");
}
'''
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    with tempfile.TemporaryDirectory(prefix='bcm4377-state-') as tmp:
        c = Path(tmp) / 'state.c'
        exe = Path(tmp) / 'state'
        c.write_text(code)
        subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                        str(c), '-o', str(exe)], check=True)
        subprocess.run([str(exe)], check=True)
        for control in ["production", "timeout-reprobe", "pci-function-reset"]:
            tested = source
            if control != "production":
                name = ('static void bcm4377_queue_rebind(' if control == "timeout-reprobe" else
                        'static int bcm4377_pci_reset(struct bcm4377_data *bcm4377)\n{')
                body = definition(source, name)
                assert body.count('if (bcm4377->dma_window_base) {') == 1
                tested = source.replace(body, body.replace('if (bcm4377->dma_window_base) {', 'if (false) {', 1), 1)
            c.write_text(recovery_program(tested))
            subprocess.run(['cc', '-std=gnu11', '-Wall', '-Wextra', '-Werror',
                            str(c), '-o', str(exe)], check=True)
            result = subprocess.run([str(exe)], capture_output=True, text=True)
            if control != "production":
                assert result.returncode and 'Assertion' in result.stderr, result
                print('BCM4388 unguarded ' + control + ' negative control: detected at runtime')
            else:
                assert result.returncode == 0, result.stderr
                print('BCM4388 recovery: high-window reprobe/function reset refused; legacy recovery paths PASS')


if __name__ == '__main__':
    main()
