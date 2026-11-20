# SPDX-License-Identifier: GPL-2.0-only OR MIT
# Copyright 2026 Ryan Murray
"""RTKit event dispatch and ACK policies with public synthetic messages."""
from common import block

def oslog(root):
    source = (root / 'drivers/soc/apple/rtkit.c').read_text()
    constants = '\n'.join(line for line in source.splitlines()
                          if line.startswith('#define APPLE_RTKIT_OSLOG_'))
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint64_t u64;
typedef uint8_t u8;
#define GENMASK_ULL(h,l) ((~0ULL << (l)) & (~0ULL >> (63-(h))))
#define FIELD_GET(mask,value) (((value) & (mask)) >> __builtin_ctzll(mask))
#define APPLE_RTKIT_EP_OSLOG 8
struct buffer { unsigned sentinel; unsigned size; };
/* v27_rx_failed: read only by the J714s firmware 27 sleep; a failed echo sets it. */
struct apple_rtkit { void *dev; struct buffer oslog_buffer; bool oslog_inherited, sync_syslog_ack, oslog_echo; u64 oslog_owed; bool v27_rx_failed; };
#define cmpxchg(p,o,n) __sync_val_compare_and_swap((p),(o),(n))
#define WRITE_ONCE(x,v) ((x)=(v))
/* As the shared Apple configuration builds it. */
#define CONFIG_APPLE_SOC_SLEEP 1
#define IS_ENABLED(option) (option)
static unsigned warnings, notifications, requests, acks, errors;
static u64 observed;
static bool neo, m5;
#define of_machine_is_compatible(s) ((!strcmp(s,"apple,t8140") && neo) || (!strcmp(s,"apple,t8142") && m5))
#define dev_warn(dev,fmt,msg) do { (void)(dev); (void)(fmt); warnings++; observed=(msg); } while (0)
#define dev_dbg(dev,fmt,msg) do { (void)(dev); (void)(fmt); notifications++; observed=(msg); } while (0)
#define dev_err(dev,fmt,msg) do { (void)(dev); (void)(fmt); (void)(msg); errors++; } while (0)
static int __attribute__((unused)) apple_rtkit_send_message(struct apple_rtkit *rtk, u8 ep, u64 msg, void *completion, bool atomic)
{ (void)rtk; assert(ep==8 && !completion && !atomic); acks++; observed=msg; return 0; }
static void apple_rtkit_common_rx_get_buffer(struct apple_rtkit *rtk, struct buffer *b, u8 ep, u64 msg)
{ assert(b==&rtk->oslog_buffer && ep==8); requests++; observed=msg; }
''' + constants + '\n' + block(source, 'static void apple_rtkit_oslog_rx(') + r'''
static void reset(void) { warnings=notifications=requests=acks=errors=0; observed=0; }
int main(void)
{
    struct apple_rtkit rtk={.oslog_buffer={.sentinel=0x12345678}};
    neo=true;
    for (u8 type=2; type<=5; type++) {
        for (unsigned edge=0; edge<2; edge++) {
            u64 msg=((u64)type<<56) | (edge ? ((1ULL<<56)-1) : 0);
            reset(); apple_rtkit_oslog_rx(&rtk,msg);
            assert(!warnings && notifications==1 && !requests && !acks && !errors);
            assert(observed==msg);
            assert(rtk.oslog_buffer.sentinel==0x12345678);
        }
    }
    puts("PASS recognized opaque types preserve both payload bounds without buffer/ACK operations");
    const u8 unknown[]={0,6,0xff};
    for (unsigned i=0;i<sizeof(unknown);i++) {
        u64 msg=((u64)unknown[i]<<56) | 42;
        reset(); apple_rtkit_oslog_rx(&rtk,msg);
        assert(warnings==1 && !notifications && !requests && observed==msg);
    }
    puts("PASS unknown types remain warnings");
    reset(); apple_rtkit_oslog_rx(&rtk,1ULL<<56);
    assert(requests==1 && !warnings && !notifications && observed==(1ULL<<56));
    puts("PASS buffer requests retain existing dispatch");
#ifdef APPLE_RTKIT_OSLOG_LOG
    /* Other machines acknowledge only admitted channel-zero counters. */
    neo=m5=false;
    reset(); apple_rtkit_oslog_rx(&rtk,2ULL<<56);
    assert(warnings==1 && !acks && !notifications);
    for (unsigned inherited=0; inherited<2; inherited++) {
        rtk.oslog_buffer.size=inherited ? 0 : 4096;
        rtk.oslog_inherited=inherited;
        reset(); apple_rtkit_oslog_rx(&rtk,(2ULL<<56)|UINT32_MAX);
        assert(acks==1 && !warnings && observed==((2ULL<<56)|UINT32_MAX));
        reset(); apple_rtkit_oslog_rx(&rtk,(2ULL<<56)|(1ULL<<32));
        assert(warnings==1 && !acks);
    }
    /* Each tested platform policy overrides channel and buffer admission. */
    rtk.oslog_inherited=false;
    for (unsigned policy=0; policy<3; policy++) {
        neo=policy==0; m5=policy==1; rtk.sync_syslog_ack=policy==2;
        reset(); apple_rtkit_oslog_rx(&rtk,(2ULL<<56)|(1ULL<<32));
        assert(notifications==1 && !warnings && !acks);
    }
    puts("PASS channel-zero ACK admission and Neo/M5/synchronous no-ACK policies");
#else
    (void)neo; (void)m5;
#endif
    return 0;
}
'''


def syslog(root):
    source = (root / 'drivers/soc/apple/rtkit.c').read_text()
    dcp = (root / 'drivers/gpu/drm/apple/dcp.c').read_text()
    header = (root / 'drivers/gpu/drm/apple/dcp-syslog.h').read_text()
    helper = header[header.index('static inline bool'):header.index('\n#endif')]
    # The policy belongs only to the external H17P transport. Other RTKit
    # clients, including the panel and older DCP firmware, keep info logging.
    assert '.syslog_is_debug' not in block(dcp, 'static struct apple_rtkit_ops rtkit_ops =')
    assert '.syslog_is_debug = dcp_rtk_syslog_is_debug' in block(dcp, 'static const struct apple_rtkit_ops external_h17p_rtkit_ops =')
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8;
typedef uint64_t u64;
#define APPLE_RTKIT_EP_SYSLOG 2
/* is_mapped without a buffer: a firmware-owned log, which no buffer here is. */
struct buffer { size_t size; unsigned char *buffer; bool iomem, is_mapped; };
struct apple_rtkit_ops {
    bool (*syslog_is_debug)(void *, const char *, const char *);
};
struct apple_rtkit {
    void *dev, *cookie;
    struct apple_rtkit_ops *ops;
    bool syslog_inherited;
    char *syslog_msg_buffer;
    size_t syslog_msg_size, syslog_n_entries;
    struct buffer syslog_buffer;
};
static unsigned infos, debugs, warnings, copies, acks, calls;
#define dev_info(...) (++infos)
#define dev_dbg(...) (++debugs)
#define dev_warn_ratelimited(...) (++warnings)
static void apple_rtkit_memcpy(struct apple_rtkit *rtk, void *out,
                              struct buffer *b, size_t off, size_t len)
{ (void)rtk; assert(off + len <= b->size); copies++; memcpy(out, b->buffer+off, len); }
static int apple_rtkit_send_message(struct apple_rtkit *rtk, u8 ep, u64 msg, void *c, bool atomic)
{ (void)rtk; assert(ep==2 && msg==0 && c==NULL && !atomic); acks++; return 0; }
''' + helper + '\n' + block(dcp, 'static bool dcp_rtk_syslog_is_debug(') + r'''
static bool policy(void *cookie, const char *context, const char *message)
{ calls++; return dcp_rtk_syslog_is_debug(cookie, context, message); }
''' + block(source, 'static bool should_crop_syslog_char(') + '\n' + block(source, 'static void apple_rtkit_syslog_rx_log(') + r'''
static void reset(void) { infos=debugs=warnings=copies=acks=calls=0; }
static void record(struct apple_rtkit *rtk, const char *context, const char *message)
{
    memset(rtk->syslog_buffer.buffer,0,rtk->syslog_buffer.size);
    memcpy(rtk->syslog_buffer.buffer+8,context,strlen(context));
    memcpy(rtk->syslog_buffer.buffer+32,message,strlen(message));
}
int main(void)
{
    unsigned char ring[288]; char decoded[256];
    struct apple_rtkit_ops ops={.syslog_is_debug=policy};
    struct apple_rtkit rtk={.ops=&ops, .syslog_msg_buffer=decoded,
        .syslog_msg_size=256, .syslog_n_entries=1,
        .syslog_buffer={.size=sizeof(ring),.buffer=ring}};
    const char *messages[]={
        "set_run_mode_safe: deferring: 4 -> 1",
        "set_run_mode_safe: deferring: 2 -> 4",
        "ready_for_run_mode_change: initiating deferred run mode change: 4 -> 1",
        "ready_for_run_mode_change: initiating deferred run mode change: 2 -> 4"
    };
    for (unsigned i=0;i<4;i++) {
        const char *ctx=i<2?"UPPipeDCP_H17P.cpp:2096":"UPPipeDCP_H17P.cpp:2014";
        record(&rtk,ctx,messages[i]); reset(); apple_rtkit_syslog_rx_log(&rtk,0);
        assert(debugs==1 && !infos && !warnings && acks==1 && copies==2 && calls==1);
        char fault[192]; snprintf(fault,sizeof(fault),"%s: timeout",messages[i]);
        record(&rtk,ctx,fault); reset(); apple_rtkit_syslog_rx_log(&rtk,0);
        assert(infos==1 && !debugs && acks==1);
        record(&rtk,"other.cpp:2096",messages[i]); reset(); apple_rtkit_syslog_rx_log(&rtk,0);
        assert(infos==1 && !debugs && acks==1);
    }
    puts("PASS complete transition records use debug; faults and foreign contexts use info; all ACK");
    record(&rtk,"UPPipeDCP_H17P.cpp:2096","set_run_mode_safe: deferring: 4 -> 1\r\n ");
    reset(); apple_rtkit_syslog_rx_log(&rtk,0);
    assert(debugs==1 && acks==1); /* Existing trimming precedes policy. */
    ops.syslog_is_debug=NULL;
    reset(); apple_rtkit_syslog_rx_log(&rtk,0);
    assert(infos==1 && !debugs && !calls && acks==1);
    ops.syslog_is_debug=policy;
    record(&rtk,"UPPipeDCP_H17P.cpp:2096","set_run_mode_safe: error -110");
    reset(); apple_rtkit_syslog_rx_log(&rtk,0);
    assert(infos==1 && !debugs && acks==1);
    rtk.syslog_inherited=true;
    reset(); apple_rtkit_syslog_rx_log(&rtk,0);
    assert(!calls && !copies && !infos && !debugs && acks==1);
    rtk.syslog_inherited=false; rtk.syslog_n_entries=0;
    reset(); apple_rtkit_syslog_rx_log(&rtk,0);
    assert(warnings==1 && !calls && !copies && acks==1);
    puts("PASS default info, trimming, inherited and invalid records retain transport behavior");
    return 0;
}
'''


def t6041_v27_admission(root):
    display_h = (root / 'include/linux/soc/apple/display.h').read_text()
    helper = display_h[display_h.index('#define APPLE_T6041_DCP_26B123_UUID'):
                       display_h.index('\n#endif')]
    iomfb_h = (root / 'drivers/gpu/drm/apple/iomfb.h').read_text()
    methods_decl = iomfb_h[iomfb_h.index('enum dcpep_method {'):
                           iomfb_h.index('#define IOMFB_MAX_CB')]
    v27_c = (root / 'drivers/gpu/drm/apple/iomfb_v27.c').read_text()
    v27_tables = v27_c[v27_c.index('static const struct dcp_method_entry dcp_methods_27_0'):
                       v27_c.index('/* The template\'s calls resolve through')]
    dcp_c = (root / 'drivers/gpu/drm/apple/dcp.c').read_text()
    selector_full = block(dcp_c, 'static enum dcp_firmware_version dcp_check_firmware_version(')
    cutoff = '\n\tret = dcp_read_fw_version(dev, "apple,firmware-compat", compat_str);\n\tif (ret < 0)'
    selector = selector_full[:selector_full.index(cutoff)] + '\n\t(void)v14_board;\n\treturn DCP_FIRMWARE_UNKNOWN;\n}\n'
    return r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
typedef uint32_t u32;
enum dcp_firmware_version {
    DCP_FIRMWARE_UNKNOWN,
    DCP_FIRMWARE_V_26_6_DESKTOP,
    DCP_FIRMWARE_V_27_0,
};
struct dcp_v14_board;
struct apple_dcp_hw_data { enum dcp_firmware_version firmware_compat; };
struct device_node {
    const char *compatible;
    const char *uuid;
    int uuid_count;
    u32 fw[3];
    int fw_cells;
    u32 compat[3];
    int compat_cells;
};
struct device {
    struct device_node *of_node;
    const struct apple_dcp_hw_data *hw;
};
static int of_device_is_compatible(const struct device_node *np, const char *s)
{
    return np && np->compatible && !strcmp(np->compatible, s);
}
static int of_property_count_strings(const struct device_node *np, const char *name)
{
    assert(!strcmp(name, "apple,firmware-uuid"));
    return np ? np->uuid_count : -1;
}
static int of_property_read_string(const struct device_node *np, const char *name, const char **out)
{
    assert(!strcmp(name, "apple,firmware-uuid"));
    if (!np || np->uuid_count < 1 || !np->uuid)
        return -1;
    *out = np->uuid;
    return 0;
}
static int of_property_count_u32_elems(const struct device_node *np, const char *name)
{
    if (!np)
        return -1;
    if (!strcmp(name, "apple,firmware-version"))
        return np->fw_cells;
    if (!strcmp(name, "apple,firmware-compat"))
        return np->compat_cells;
    return -1;
}
static int of_property_read_u32_array(const struct device_node *np, const char *name, u32 *out, size_t n)
{
    const u32 *src;
    int cells;
    if (!np || n != 3)
        return -1;
    if (!strcmp(name, "apple,firmware-version")) {
        src = np->fw;
        cells = np->fw_cells;
    } else if (!strcmp(name, "apple,firmware-compat")) {
        src = np->compat;
        cells = np->compat_cells;
    } else {
        return -1;
    }
    if (cells != 3)
        return -1;
    memcpy(out, src, 3 * sizeof(u32));
    return 0;
}
#define DCP_FW_VERSION_STR_LEN 32
#define GFP_KERNEL 0
#define dev_err(...) ((void)0)
static void *devm_kmemdup(struct device *dev, const void *src, size_t len, unsigned int gfp)
{
    (void)dev; (void)src; (void)len; (void)gfp;
    return NULL;
}
static const struct apple_dcp_hw_data *of_device_get_match_data(struct device *dev)
{
    return dev->hw;
}
static int dcp_read_fw_version(struct device *dev, const char *name, char *out)
{
    u32 v[3];
    if (of_property_count_u32_elems(dev->of_node, name) != 3 ||
        of_property_read_u32_array(dev->of_node, name, v, 3))
        return -1;
    snprintf(out, DCP_FW_VERSION_STR_LEN, "%u.%u.%u", v[0], v[1], v[2]);
    return 3;
}
''' + helper + '\n' + methods_decl + '\n' + v27_tables + '\n' + selector + r'''
static struct device_node node;
static struct apple_dcp_hw_data hw_t6041;
static struct device dev = { .of_node = &node, .hw = &hw_t6041 };

static void reset_t6041_26b123(bool ext)
{
    node = (struct device_node){
        .compatible = ext ? "apple,t6041-dcpext" : "apple,t6041-dcp",
        .uuid = APPLE_T6041_DCP_26B123_UUID,
        .uuid_count = 1,
        .fw = { 27, 2, 0 },
        .fw_cells = 3,
        .compat = { 27, 2, 0 },
        .compat_cells = 3,
    };
}

static void expect_rejected(void)
{
    assert(!apple_t6041_dcp_v27_2(&node));
    assert(!apple_t6041_dcpext_v27_2(&node));
    assert(dcp_check_firmware_version(&dev) == DCP_FIRMWARE_UNKNOWN);
}

int main(void)
{
    assert(!apple_t6041_dcp_v27_2(NULL));
    assert(!apple_t6041_dcpext_v27_2(NULL));

    /*
     * Internal only matches apple_t6041_dcp_v27_2; external only matches
     * apple_t6041_dcpext_v27_2. No of_machine_is_compatible() stub is defined
     * in this translation unit, enforcing at compile time that admission keys
     * solely on the DCP IP compatible and firmware identity.
     */
    reset_t6041_26b123(false);
    assert(apple_t6041_dcp_v27_2(&node));
    assert(!apple_t6041_dcpext_v27_2(&node));
    assert(dcp_check_firmware_version(&dev) == DCP_FIRMWARE_V_27_0);

    reset_t6041_26b123(true);
    assert(!apple_t6041_dcp_v27_2(&node));
    assert(apple_t6041_dcpext_v27_2(&node));
    assert(dcp_check_firmware_version(&dev) == DCP_FIRMWARE_V_27_0);

    for (unsigned ext = 0; ext < 2; ext++) {
        /* Reject non-T6041 DCP compatibles. */
        reset_t6041_26b123(ext); node.compatible = "apple,t6031-dcp"; expect_rejected();

        /* Reject missing, multi-string, or non-26B123 firmware UUIDs (including 26B131). */
        reset_t6041_26b123(ext); node.uuid_count = 0; expect_rejected();
        reset_t6041_26b123(ext); node.uuid_count = 2; expect_rejected();
        reset_t6041_26b123(ext); node.uuid = "00000000-0000-0000-0000-000000000000"; expect_rejected();
        reset_t6041_26b123(ext); node.uuid = "71F140DC-F18D-31B3-9772-537A5CC8400D"; expect_rejected();

        /* Reject malformed or non-27.2.0 firmware/compat tuples. */
        reset_t6041_26b123(ext); node.fw_cells = 2; expect_rejected();
        reset_t6041_26b123(ext); node.compat_cells = 4; expect_rejected();
        reset_t6041_26b123(ext); node.fw[1] = 0; expect_rejected();
        reset_t6041_26b123(ext); node.compat[1] = 0; expect_rejected();
        reset_t6041_26b123(ext); node.fw[2] = 1; node.compat[2] = 1; expect_rejected();
    }

    /* Desktop 26.5.2 and 26.6.2 remain DCP_FIRMWARE_V_26_6_DESKTOP on internal and external. */
    for (unsigned minor = 5; minor <= 6; minor++) {
        for (unsigned ext = 0; ext < 2; ext++) {
            reset_t6041_26b123(ext);
            node.uuid = "other";
            node.fw[0] = 26; node.fw[1] = minor; node.fw[2] = 2;
            node.compat[0] = 26; node.compat[1] = minor; node.compat[2] = 2;
            assert(!apple_t6041_dcp_v27_2(&node));
            assert(!apple_t6041_dcpext_v27_2(&node));
            assert(dcp_check_firmware_version(&dev) == DCP_FIRMWARE_V_26_6_DESKTOP);
        }
    }
    puts("PASS T6041 26B123 v27.2 internal/external admission, refusal controls, and desktop 26.x preservation");
    return 0;
}
'''
