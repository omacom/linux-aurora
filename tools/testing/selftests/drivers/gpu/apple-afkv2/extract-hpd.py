#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Exercise the production DPTX HPD request with mocked service-call replies.

Covers the legacy, AFK-v2 and compact EPIC replies and the 14.7 AV client close.
"""
import pathlib
import re
import sys


def function(source, name):
    match = re.search(r"^(?:static )?int " + name + r"\([^;]*?\)\n\{", source, re.M)
    if not match:
        raise RuntimeError("production definition of " + name + " not found")
    end, depth = match.end(), 1
    while depth:
        depth += (source[end] == "{") - (source[end] == "}")
        end += 1
    # Older trees keep the call body static; the unit declares it below.
    return re.sub(r"^static ", "", source[match.start():end])


afk = pathlib.Path(sys.argv[1]).read_text()
dptx = pathlib.Path(sys.argv[2]).read_text()
command = re.search(r"^struct dcpdptx_hotplug_cmd \{.*?\} __attribute__\(\(packed\)\);",
                    dptx, re.M | re.S)
if not command:
    raise RuntimeError("production HPD request layout not found")
prefix = r'''
#undef NDEBUG
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include "afkv2-codec.h"
typedef u32 __le32;
#define GFP_KERNEL 0
#define kzalloc(n,f) calloc(1,n)
#define kfree(p) free(p)
#define max(a,b) ((a)>(b)?(a):(b))
#define min(a,b) ((a)<(b)?(a):(b))
#define cpu_to_le16(x) (x)
#define cpu_to_le32(x) (x)
#define le16_to_cpu(x) (x)
#define le32_to_cpu(x) (x)
#define EPIC_SERVICE_CALL_MAGIC 0x69706378
#define EPIC_SUBTYPE_STD_SERVICE 0xc0
#define dev_warn(...) do {} while (0)
#define dev_dbg(...) do {} while (0)
#define READ_ONCE(x) (x)
static u32 get_unaligned_le32(const void *p)
{
 const u8 *b = p;

 return b[0] | b[1] << 8 | b[2] << 16 | (u32)b[3] << 24;
}
struct epic_service_call {
 u8 pad0[2]; u16 group; u32 command, data_len, magic; u8 pad1[48];
};
struct dcp { void *dev; };
struct ops { const char *name; };
struct ep { struct dcp *dcp; int endpoint; bool inline_v2; bool compact; };
struct apple_epic_service { struct ep *ep; struct ops *ops; u32 channel; };
int afk_service_call_timeout_len(struct apple_epic_service *service, u16 group,
 u32 command, const void *data, size_t data_len, size_t data_pad, void *output,
 size_t output_len, size_t output_pad, unsigned int timeout_ms, size_t *received);
int afk_service_call_timeout(struct apple_epic_service *service, u16 group,
 u32 command, const void *data, size_t data_len, size_t data_pad, void *output,
 size_t output_len, size_t output_pad, unsigned int timeout_ms);
int dptxport_set_hpd_timeout(struct apple_epic_service *service, bool hpd,
 unsigned int timeout_ms);
/*
 * A matched, successful reply in the request's buffer: a 64-byte header, then
 * the body the firmware declares, of which the echo is the word at 16.  An
 * AFK-v2 transport reports how much of the buffer arrived; a legacy one fills
 * it.  Without a length query AFK-v2 refuses anything short of the buffer.
 */
static bool wire_v2;
static size_t declared, arrived;
static u32 echo_flip, compact_status;
/* An HPD low closes the 14.7 external AV client first; the stub counts it. */
static struct dcp *expected_dcp;
static unsigned int v14_closes, cec_closes, closes_at_send, cec_closes_at_send;
/* Kept even if a mutant drops the call, so the count check is what fails. */
__attribute__((unused)) static void dcpavserv_v14_close(struct dcp *dcp)
{
 assert(dcp == expected_dcp);
 v14_closes++;
}
__attribute__((unused)) static void dcp_cec_close_port(struct dcp *dcp,
 struct apple_epic_service *service)
{
 assert(dcp == expected_dcp && service != NULL);
 cec_closes++;
}
static int afk_send_command_timeout_len(struct apple_epic_service *s, u8 type,
 const void *payload, size_t payload_len, void *output, size_t output_len,
 u32 *retcode, unsigned int timeout_ms, size_t *received)
{
 struct epic_service_call *call = output;
 u8 *body = (u8 *)output + sizeof(*call);
 u32 echo;

 (void)payload; (void)timeout_ms;
 assert(type == EPIC_SUBTYPE_STD_SERVICE && payload_len == output_len);
 assert(output_len == sizeof(*call) + 32 && call->data_len == 32);
 assert(call->group == 8 && call->command == 8);
 closes_at_send = v14_closes;
 cec_closes_at_send = cec_closes;
 if (wire_v2 && !received && arrived < output_len - sizeof(*call))
  return -EMSGSIZE;
 memcpy(&echo, body + 16, sizeof(echo));
 echo ^= echo_flip;
 memcpy(body + 16, &echo, sizeof(echo));
 /* Compact EPIC answers with an IOReturn status in the first word. */
 if (s->ep->compact) {
  for (int i = 0; i < 4; i++)
   body[i] = compact_status >> (8 * i);
 }
 call->data_len = declared;
 *retcode = 0;
 if (received)
  *received = sizeof(*call) + (wire_v2 ? arrived : output_len - sizeof(*call));
 return 0;
}
'''
suffix = r'''
static int failures;
static struct ep *the_ep;
static void expect(bool ok, const char *what, bool hpd, int ret)
{
 printf("%s %s%s: %s hpd=%d declared=%zu arrived=%zu ret=%d\n", ok ? "ok" : "FAIL",
        wire_v2 ? "afk-v2" : "legacy", the_ep->compact ? " compact" : "", what, hpd,
        declared, arrived, ret);
 failures += !ok;
}
/* Every request closes the AV client exactly when it lowers HPD, and before it is sent. */
static int set_hpd(struct apple_epic_service *service, int hpd)
{
 unsigned int before = v14_closes, cec_before = cec_closes;
 int ret = dptxport_set_hpd_timeout(service, hpd, 1000);

 expect(v14_closes == before + !hpd && closes_at_send == before + !hpd &&
        cec_closes == cec_before + !hpd && cec_closes_at_send == cec_before + !hpd,
        "AV/CEC clients closed before an HPD low only", hpd, ret);
 return ret;
}
int main(void)
{
 static const size_t lengths[] = { 0, 4, 8, 12, 16, 20, 32, 36, 65535 };
 struct dcp dcp = { 0 };
 struct ops ops = { "AppleDCPDPTXRemotePort" };
 struct ep ep = { .dcp = &dcp };
 struct apple_epic_service service = { .ep = &ep, .ops = &ops, .channel = 1 };

 the_ep = &ep;
 expected_dcp = &dcp;

 for (int v2 = 0; v2 < 2; v2++) {
  wire_v2 = v2;
  for (size_t i = 0; i < sizeof(lengths) / sizeof(lengths[0]); i++) {
   for (int hpd = 0; hpd < 2; hpd++) {
    int ret;

    declared = lengths[i]; arrived = 32; echo_flip = 0;
    ret = set_hpd(&service, hpd);
    if (declared < sizeof(struct dcpdptx_hotplug_cmd))
     expect(ret < 0, "short reply refused", hpd, ret);
    else if (declared > 32)
     expect(ret < 0, "reply longer than its buffer refused", hpd, ret);
    else
     expect(ret == 0, "complete reply accepted", hpd, ret);
   }
  }
  for (int hpd = 0; hpd < 2; hpd++) {
   int ret;

   declared = 20; arrived = 32; echo_flip = 1;
   ret = set_hpd(&service, hpd);
   expect(ret == -EINVAL, "wrong echo refused", hpd, ret);
   if (!wire_v2)
    continue;
   declared = 20; arrived = 16; echo_flip = 0;
   ret = set_hpd(&service, hpd);
   expect(ret < 0, "reply cut short in transit refused", hpd, ret);
  }
  /* Compact EPIC: the status word decides, there is no echo to compare. */
  ep.compact = true;
  for (int hpd = 0; hpd < 2; hpd++) {
   int ret;

   declared = 20; arrived = 32; echo_flip = 0; compact_status = 0;
   ret = set_hpd(&service, hpd);
   expect(ret == 0, "zero status accepted", hpd, ret);
   echo_flip = 1;
   ret = set_hpd(&service, hpd);
   expect(ret == 0, "echo word ignored", hpd, ret);
   echo_flip = 0; compact_status = 0xe00002c2;
   ret = set_hpd(&service, hpd);
   expect(ret == -EIO, "firmware error status refused", hpd, ret);
   compact_status = 0x00000100;
   ret = set_hpd(&service, hpd);
   expect(ret == -EIO, "any nonzero status byte refused", hpd, ret);
  }
  ep.compact = false; compact_status = 0;
 }
 printf("%d HPD reply checks failed\n", failures);
 if (failures)
  return 1;
 puts("PASS: extracted production DPTX HPD request: short, oversized, wrong-echo and complete replies, legacy, AFK-v2 and compact status, AV client close on HPD low");
 return 0;
}
'''
unit = (prefix + command.group(0) + "\n" + function(afk, "afk_service_call_timeout_len") + "\n" +
        function(afk, "afk_service_call_timeout") + "\n" +
        function(dptx, "dptxport_set_hpd_timeout") + suffix)
pathlib.Path(sys.argv[3]).write_text(unit)
