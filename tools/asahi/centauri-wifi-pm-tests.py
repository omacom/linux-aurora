#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Execute the checkout's Wi-Fi disconnect/PM callbacks with mocked transport.

This checks error propagation and command receipts, not firmware sleep semantics.
Run in a bounded host scope; no kernel build or hardware is needed.
"""
import argparse
import os
from pathlib import Path
import re
import resource
import signal
import shlex
import subprocess
import tempfile


def block(source, name):
    match = re.search(r"^(?:static )?(?:int|void) " + re.escape(name) + r"\(", source, re.M)
    if not match:
        raise RuntimeError(f"Production function {name} missing")
    brace = source.index("{", match.start())
    depth, end = 1, brace + 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    return source[match.start():end]


PRELUDE = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
#define BIT(n) (1U << (n))
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x, v) ((x) = (v))
#define GFP_KERNEL 0
#define BUFFER_SIZE 128
#define TX_BUFFER 2048
#define TR_BASE 0
#define dma_rmb() ((void)0)
#define dma_wmb() ((void)0)
#define dev_err(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define msecs_to_jiffies(x) (x)
#define time_before(a, b) ((long)((a) - (b)) < 0)
#define time_after_eq(a, b) (!time_before(a, b))
static unsigned long jiffies;
struct centauri_t6050_wifi;
struct centauri_t6050 { struct centauri_t6050_wifi *wifi; bool removing; };
struct wiphy { struct centauri_t6050_wifi *wifi; };
struct net_device { int unused; };
struct cfg80211_wowlan { bool any; };
'''

STUBS = r'''
struct centauri_t6050_wifi {
    struct centauri_t6050 *control;
    struct { u8 *window, *bar; uint64_t window_dma; } a;
    int lock, connect_work, link_work, data_tx_queue;
    bool command_failed, stopping, connect_abort, disconnect_pending, connecting, connected;
    bool reply_valid, pm_prepared;
    u16 command_head, sequence, reply_capture, reply_sequence, reply_status;
    u32 reply_command;
    u8 *reply_buffer;
    size_t reply_capacity, reply_size;
    struct centauri_wifi_disconnect_receipt pm_disconnect;
};
static const u16 ring_size[5] = {0, 64};
static unsigned int doorbells, drains, notifications, consumer;
static u16 firmware_status;
static bool async_reply, async_event, wrong_reply;
static bool *submission_probe;
static bool private_access_forbidden;
enum { REPLY, TIMEOUT, LOST };
static int transport_mode;
static void mutex_lock(int *lock) { assert(!*lock); *lock = 1; }
static void mutex_unlock(int *lock) { assert(*lock); *lock = 0; }
static void *wiphy_priv(struct wiphy *wiphy)
{ assert(!private_access_forbidden); return &wiphy->wifi; }
static void usleep_range(int a, int b) { (void)a; (void)b; jiffies += 100; }
static unsigned int readl(void *address) { (void)address; return transport_mode == LOST ? 0 : 2; }
static void writel(unsigned int value, void *address) { (void)value; (void)address; doorbells++; }
static u16 index_read(struct centauri_t6050_wifi *wifi, int array, int index)
{ assert(wifi->lock); (void)array; (void)index; return consumer; }
static void index_write(struct centauri_t6050_wifi *wifi, int array, int index, u16 value)
{ assert(wifi->lock); (void)array; (void)index; (void)value;
  if (submission_probe) assert(*submission_probe); }
static void put_unaligned_le16(u16 value, void *p) { memcpy(p, &value, 2); }
static void put_unaligned_le32(u32 value, void *p) { memcpy(p, &value, 4); }
static void put_unaligned_le64(uint64_t value, void *p) { memcpy(p, &value, 8); }
static void command_fault(struct centauri_t6050_wifi *wifi) { wifi->command_failed = true; }
static void drain(struct centauri_t6050_wifi *wifi)
{
    assert(wifi->lock);
    drains++;
    if (transport_mode != REPLY)
        return;
    if (!wifi->reply_valid) {
        wifi->reply_valid = true;
        wifi->reply_sequence = wifi->reply_capture + wrong_reply;
        wifi->reply_command = 131073 | (async_reply ? BIT(30) : 0);
        wifi->reply_status = firmware_status;
    } else if (async_event) {
        wifi->disconnect_pending = false;
    }
}
static void cancel_scan(struct centauri_t6050_wifi *wifi) { assert(!wifi->lock); }
static void cancel_work_sync(int *work) { (void)work; }
static void cancel_connect_locked(struct centauri_t6050_wifi *wifi) { assert(wifi->lock); }
static void netif_stop_queue(struct net_device *netdev) { (void)netdev; }
static void netif_carrier_off(struct net_device *netdev) { (void)netdev; }
static void skb_queue_purge(int *queue) { (void)queue; }
static void cfg80211_disconnected(struct net_device *netdev, u16 reason, void *ie,
                                  int size, bool local, int flags)
{ (void)netdev; (void)reason; (void)ie; (void)size; (void)local; (void)flags; notifications++; }
'''

TESTS = r'''
static struct centauri_t6050 control;
static struct centauri_t6050_wifi wifi;
static struct wiphy wiphy;
static struct net_device netdev;
static u8 window[16384], bar[8192];
static void reset(void)
{
    memset(&wifi, 0, sizeof(wifi));
    memset(window, 0, sizeof(window));
    control = (struct centauri_t6050) { .wifi = &wifi };
    wifi.control = &control;
    wifi.a.window = window;
    wifi.a.bar = bar;
    wifi.connected = true;
    wiphy.wifi = &wifi;
    doorbells = drains = notifications = consumer = 0;
    firmware_status = 0;
    async_reply = async_event = wrong_reply = false;
    transport_mode = REPLY;
    jiffies = 0;
    submission_probe = NULL;
    private_access_forbidden = false;
}
static void prepare(void) { reset(); centauri_t6050_wifi_pm_prepare(&control); }
static int disconnect(void) { return wifi_disconnect(&wiphy, &netdev, 3); }
static void expect_error(int expected)
{
    unsigned int before = doorbells;
    assert(wifi.pm_disconnect.error == expected);
    assert(centauri_t6050_wifi_pm_error(&control) == expected);
    assert(call_registered_suspend(&wiphy, NULL) == expected);
    assert(doorbells == before); /* Suspend/error lookup cannot issue a retry. */
    assert(!wifi.lock);
}

static void test_registered_pm_contract(void)
{
    cfg80211_registration_check(&registered_pm_ops);
    assert(registration_warnings == 0);
    for (unsigned retired = 0; retired < 2; retired++) {
        reset();
        wifi.pm_prepared = true;
        wifi.pm_disconnect.error = -EREMOTEIO;
        if (retired) {
            control.wifi = NULL;
            wiphy.wifi = (struct centauri_t6050_wifi *)(uintptr_t)1;
        }
        struct centauri_t6050_wifi snapshot = wifi;
        unsigned before_doorbells = doorbells, before_drains = drains;
        private_access_forbidden = true;
        assert(call_registered_resume(&wiphy) == 0);
        private_access_forbidden = false;
        assert(doorbells == before_doorbells && drains == before_drains);
        assert(memcmp(&snapshot, &wifi, sizeof(wifi)) == 0);
    }
}

int main(void)
{
    test_registered_pm_contract();
    prepare();
    assert(!wifi.pm_disconnect.seen);
    expect_error(0); /* Missing transaction remains missing. */
    struct cfg80211_wowlan wowlan = {};
    assert(call_registered_suspend(&wiphy, &wowlan) == 1);
    wowlan.any = true;
    assert(call_registered_suspend(&wiphy, &wowlan) == 0);
    assert(!doorbells && !wifi.pm_disconnect.seen);
    wifi.connected = false;
    assert(call_registered_suspend(&wiphy, &wowlan) == 1);
    wifi.connected = true;
    wifi.connecting = true;
    assert(call_registered_suspend(&wiphy, &wowlan) == 1);
    wifi.connecting = false;
    wifi.command_failed = true;
    assert(call_registered_suspend(&wiphy, &wowlan) == 1);
    wifi.command_failed = false;
    assert(!doorbells && !wifi.pm_disconnect.seen);

    firmware_status = 326;
    assert(disconnect() == 0);
    assert(wifi.pm_disconnect.seen && wifi.pm_disconnect.submitted);
    assert(wifi.pm_disconnect.replied && wifi.pm_disconnect.status == 326);
    assert(wifi.pm_disconnect.command_error == -EREMOTEIO);
    expect_error(0); /* Keep the existing helper normalization. */
    assert(notifications == 1);

    prepare();
    firmware_status = 777;
    assert(disconnect() == -EREMOTEIO);
    expect_error(-EREMOTEIO);
    firmware_status = 0;
    assert(disconnect() == 0);
    assert(wifi.pm_disconnect.status == 777);
    assert(wifi.pm_disconnect.command_error == -EREMOTEIO);
    expect_error(-EREMOTEIO); /* Later success cannot erase the first failure. */
    centauri_t6050_wifi_pm_complete(&control);
    assert(!wifi.pm_prepared && wifi.pm_disconnect.status == 777);
    assert(centauri_t6050_wifi_pm_error(&control) == 0);
    assert(wifi_suspend(&wiphy, NULL) == 0);
    centauri_t6050_wifi_pm_prepare(&control);
    assert(!wifi.pm_disconnect.seen);
    expect_error(0);

    prepare();
    wifi.stopping = true;
    wifi.reply_status = 326; /* Old reply must not become this receipt. */
    assert(disconnect() == 0);
    assert(wifi.pm_disconnect.stopping_skip && wifi.pm_disconnect.seen);
    assert(!wifi.pm_disconnect.submitted && !wifi.pm_disconnect.replied);
    assert(!wifi.pm_disconnect.status && !doorbells);
    expect_error(0);

    prepare();
    wifi.command_failed = true;
    wifi.reply_status = 326;
    assert(disconnect() == -EIO);
    assert(!wifi.pm_disconnect.submitted && !wifi.pm_disconnect.replied);
    assert(!wifi.pm_disconnect.status && !doorbells);
    expect_error(-EIO);

    prepare();
    consumer = 1; /* Full command ring: fail before publication. */
    assert(disconnect() == -ENOSPC);
    assert(!wifi.pm_disconnect.submitted && !wifi.pm_disconnect.replied);
    expect_error(-ENOSPC);

    prepare();
    transport_mode = LOST;
    assert(disconnect() == -EIO);
    assert(wifi.pm_disconnect.submitted && !wifi.pm_disconnect.replied);
    expect_error(-EIO);

    prepare();
    transport_mode = TIMEOUT;
    wifi.reply_status = 326;
    assert(disconnect() == -ETIMEDOUT);
    assert(wifi.pm_disconnect.submitted && !wifi.pm_disconnect.replied);
    assert(!wifi.pm_disconnect.status);
    expect_error(-ETIMEDOUT);

    prepare();
    wrong_reply = true;
    firmware_status = 326; /* A mismatched sequence cannot normalize a timeout. */
    assert(disconnect() == -ETIMEDOUT);
    assert(!wifi.pm_disconnect.replied && !wifi.pm_disconnect.status);
    expect_error(-ETIMEDOUT);

    prepare();
    async_reply = true;
    assert(disconnect() == -ETIMEDOUT);
    assert(wifi.pm_disconnect.submitted && wifi.pm_disconnect.replied);
    assert(wifi.pm_disconnect.waited && !wifi.pm_disconnect.command_error);
    assert(wifi.pm_disconnect.reply_command == (131073 | BIT(30)));
    expect_error(-ETIMEDOUT);

    prepare();
    async_reply = async_event = true;
    assert(disconnect() == 0);
    assert(wifi.pm_disconnect.waited);
    expect_error(0); /* Existing async wait result; no causal attribution added. */

    reset();
    firmware_status = 777;
    assert(disconnect() == -EREMOTEIO);
    assert(!wifi.pm_disconnect.seen); /* Non-PM disconnect does not arm PM. */
    assert(wifi_suspend(&wiphy, NULL) == 0);
    control.wifi = NULL;
    centauri_t6050_wifi_pm_prepare(&control);
    assert(centauri_t6050_wifi_pm_error(&control) == 0);
    centauri_t6050_wifi_pm_complete(&control);
    prepare();
    bool published = false;
    submission_probe = &published;
    mutex_lock(&wifi.lock);
    assert(command_reply(&wifi, 131073, NULL, 0, NULL, NULL, &published) == 0);
    mutex_unlock(&wifi.lock);
    assert(published);
    submission_probe = NULL;
    puts("PASS: production disconnect/transport/PM callbacks and first-error retention");
    return 0;
}
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--out', type=Path)
    parser.add_argument('--negative-controls', action='store_true')
    parser.add_argument('--source-root', type=Path)
    parser.add_argument('--wifi-source', type=Path)
    args = parser.parse_args()
    root = args.source_root or Path(__file__).resolve().parents[2]
    source = (args.wifi_source or root / "drivers/net/wireless/apple/centauri/t6050/centauri_wifi.c").read_text()
    core = (root / "net/wireless/core.c").read_text()
    receipt = re.search(r"struct centauri_wifi_disconnect_receipt \{.*?\n\};", source, re.S)
    if not receipt:
        raise RuntimeError("Production receipt structure missing")
    names = ["command_reply", "wait_disconnect", "wifi_disconnect",
             "centauri_t6050_wifi_pm_prepare", "centauri_t6050_wifi_pm_error",
             "centauri_t6050_wifi_pm_complete", "wifi_suspend"]
    has_resume = bool(re.search(r"^static int wifi_resume\(", source, re.M))
    if has_resume:
        names.append("wifi_resume")
    production = "\n\n".join(block(source, name) for name in names)
    ops = re.search(r"static const struct cfg80211_ops wifi_ops = \{(.*?)\n\};", source, re.S)
    registration = re.search(r"\.suspend\s*=\s*(\w+)", ops.group(1)) if ops else None
    suspend = registration.group(1) if registration else "NULL"
    resume_registration = re.search(r"\.resume\s*=\s*(\w+)", ops.group(1)) if ops else None
    resume_callback = resume_registration.group(1) if resume_registration else "NULL"
    expected_resume = "wifi_resume" if has_resume else "NULL"
    constraint = re.search(r"^\s*WARN_ON\(ops->suspend && !ops->resume\);", core, re.M)
    if not constraint:
        raise RuntimeError("Actual cfg80211 PM registration constraint changed")
    dispatch = f"""
struct cfg80211_ops {{
    int (*suspend)(struct wiphy *, struct cfg80211_wowlan *);
    int (*resume)(struct wiphy *);
}};
static unsigned registration_warnings;
static int mock_warn_on(bool condition)
{{
    registration_warnings += condition;
    return condition;
}}
#define WARN_ON(condition) mock_warn_on(!!(condition))
static void cfg80211_registration_check(const struct cfg80211_ops *ops)
{{
    {constraint.group().strip()}
}}
static const struct cfg80211_ops registered_pm_ops = {{
    .suspend = {suspend},
    .resume = {resume_callback},
}};
static int call_registered_suspend(struct wiphy *wiphy, struct cfg80211_wowlan *wowlan)
{{
    assert(registered_pm_ops.suspend == wifi_suspend);
    return registered_pm_ops.suspend(wiphy, wowlan);
}}
static int call_registered_resume(struct wiphy *wiphy)
{{
    assert(registered_pm_ops.resume && registered_pm_ops.resume == {expected_resume});
    return registered_pm_ops.resume(wiphy);
}}
"""
    production += dispatch
    variants = [("production", production, 0)]
    negative_controls = [
        ("lost-suspend-error", "ret = wifi->pm_disconnect.error;", "ret = 0;"),
        ("overwritten-first-error", "wifi->pm_prepared && !wifi->pm_disconnect.error",
         "wifi->pm_prepared"),
        ("stale-reply", "receipt.replied = receipt.submitted && (!ret || ret == -EREMOTEIO);",
         "receipt.replied = true;"),
        ("wowlan-ignores-trigger", "wowlan->any && ", ""),
    ]
    failures = {
        'lost-suspend-error': 'centauri_t6050_wifi_pm_error(&control) == expected',
        'overwritten-first-error': 'wifi.pm_disconnect.status == 777',
        'stale-reply': '!wifi.pm_disconnect.submitted && !wifi.pm_disconnect.replied',
        'wowlan-ignores-trigger': 'call_registered_suspend(&wiphy, &wowlan) == 1',
    }
    for name, before, after in negative_controls if args.negative_controls else []:
        if before not in production:
            raise RuntimeError(f"Production pattern for {name} changed")
        variants.append((name, production.replace(before, after), -signal.SIGABRT))
    if args.negative_controls:
        needle = "\tif (submitted)\n\t\t*submitted = true;\n\tindex_write(wifi, 3, 1, wifi->command_head);"
        if needle not in production:
            raise RuntimeError("Production publication boundary changed")
        late = production.replace(needle, "\tindex_write(wifi, 3, 1, wifi->command_head);\n\tif (submitted)\n\t\t*submitted = true;", 1)
        variants.append(("late-submission", late, -signal.SIGABRT))
        failures["late-submission"] = "*submission_probe"
    if args.negative_controls and has_resume:
        if resume_registration:
            needle = ".resume = " + resume_callback + ","
            if needle not in production:
                raise RuntimeError("Registered resume initializer changed")
            variants.append(("cfg80211-missing-resume", production.replace(needle, ".resume = NULL,", 1), -signal.SIGABRT))
            failures["cfg80211-missing-resume"] = "registration_warnings == 0"
        for name, replacement, failure in (
            ("resume-private-access", "(void)wiphy_priv(wiphy);", "!private_access_forbidden"),
            ("resume-command", "(void)wiphy; writel(1, NULL);", "doorbells == before_doorbells"),
        ):
            needle = "(void)wiphy;"
            if needle not in production:
                raise RuntimeError("No-private-access resume body changed")
            variants.append((name, production.replace(needle, replacement, 1), -signal.SIGABRT))
            failures[name] = failure
    resource.setrlimit(resource.RLIMIT_CORE, (0, 0))
    compiler = shlex.split(os.environ.get("CC", "cc"))
    with tempfile.TemporaryDirectory(prefix="centauri-wifi-pm-") as directory:
        out = args.out or Path(directory)
        out.mkdir(parents=True, exist_ok=True)
        for name, code, expected in variants:
            harness, binary = out / (name + '.c'), out / name
            harness.write_text(PRELUDE + receipt.group() + STUBS + code + TESTS)
            subprocess.run(compiler + ["-std=gnu11", "-Wall", "-Wextra", "-Werror",
                                      str(harness), "-o", str(binary)], check=True)
            result = subprocess.run([str(binary)], capture_output=True, text=True)
            (out / (name + '.log')).write_text(result.stdout + result.stderr)
            if result.returncode != expected or (expected and (
                    'Assertion' not in result.stderr or failures[name] not in result.stderr)):
                raise RuntimeError(f"{name}: exit {result.returncode}, expected {expected}\n"
                                   + result.stdout + result.stderr)
            print(f"{name}: {'PASS' if expected == 0 else 'correctly rejected'}")


if __name__ == "__main__":
    main()
