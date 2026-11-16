# SPDX-License-Identifier: GPL-2.0-only
# Copyright 2026 Ryan Murray
"""Actual parser, bounded rate admission, catalog replacement and HPD."""
from pathlib import Path
import copy, re, struct
from common import block, source_defines, apple_function, internal_inline

def parser_fixture(root):

    base = root / 'drivers/gpu/drm/apple'
    fixture = {'MaxHorizontalImageSize': 60, 'MaxVerticalImageSize': 33, 'Chromaticity': {'Red': {'X': 44608, 'Y': 20288}, 'Green': {'X': 16960, 'Y': 40448}, 'Blue': {'X': 9344, 'Y': 5184}}, 'DefaultWhitePoint': {'X': 20544, 'Y': 21568, 'Gamma': 144179}, 'WhitePoints': [{'X': 20544, 'Y': 21568, 'Gamma': 144179}], 'DefaultColorSpaceIsSRGB': True, 'SupportsPQEOTF': True, 'ProductAttributes': {'LegacyManufacturerID': 1138, 'ProductID': 2895, 'SerialNumber': 305419896, 'WeekOfManufacture': 7, 'YearOfManufacture': 2013, 'AlphanumericSerialNumber': 'ANON000000001', 'ProductName': 'QG270 S3', 'ManufacturerID': 'ACR'}}


    class ObjectPairs(list):
        pass


    def wire(value, out):
        out += b'\0' * (-len(out) % 4)
        if isinstance(value, bool):
            out += struct.pack('<I', (11 << 24) | value)
        elif isinstance(value, int):
            out += struct.pack('<Iq', (4 << 24) | 8, value)
        elif isinstance(value, str):
            data = value.encode()
            out += struct.pack('<I', (9 << 24) | len(data)) + data
        elif isinstance(value, ObjectPairs):
            out += struct.pack('<I', (1 << 24) | len(value))
            for key, child in value:
                wire(key, out)
                wire(child, out)
        elif isinstance(value, list):
            out += struct.pack('<I', (2 << 24) | len(value))
            for child in value:
                wire(child, out)
        elif isinstance(value, dict):
            out += struct.pack('<I', (1 << 24) | len(value))
            for key, child in value.items():
                wire(key, out)
                wire(child, out)
        else:
            raise TypeError(value)


    source = (base / 'parser.c').read_text()
    parser = source[source.index('#define DCP_PARSE_HEADER'):source.index('int parse_epic_service_init(')]
    metadata = (base / 'dcp-display-metadata.h').read_text()
    metadata = block(metadata, 'struct dcp_display_metadata {') + ';\n'
    header = (base / 'parser.h').read_text()
    header = header[header.index('struct apple_dcp;'):header.index('int parse(')]
    # Header helpers stay header-like: unused here, as in any includer.
    header = header.replace('static inline ', 'static inline __attribute__((unused)) ')
    edid_header = (root / 'include/drm/drm_edid.h').read_text()
    edid_header = edid_header[edid_header.index('#define EDID_LENGTH'):edid_header.index('/* EDID matching */')]
    builder = (base / 'dcp-display-metadata.c').read_text()
    builder = '\n'.join(line for line in builder.splitlines() if not line.startswith('#include'))
    epoch = (base / 'dcp-metadata-epoch.h').read_text().replace('#include <linux/types.h>', '')
    timing_fixture = [{
        "HorizontalAttributes": {"Total": 2200, "FrontPorch": 88, "SyncWidth": 44,
                                 "Active": 1920, "PreciseSyncRate": 4423680, "SyncPolarity": 1, "PixelRepetition": 0},
        "VerticalAttributes": {"Total": 1125, "FrontPorch": 4, "SyncWidth": 5,
                               "Active": 1080, "PreciseSyncRate": 3932160, "SyncPolarity": 1, "PixelRepetition": 0},
        "ColorModes": [{"ID": 85, "Score": 1, "IsVirtual": False, "PixelEncoding": 0,
                        "Colorimetry": 10, "Depth": 10, "DynamicRange": 0, "EOTF": 0}],
        "ID": 62, "Score": 16224, "IsPreferred": True, "IsInterlaced": False, "IsVirtual": False,
    }]
    timing_raw = bytearray(struct.pack('<I', 0xd3))
    wire(timing_fixture, timing_raw)
    timing_source = 'static const unsigned char timing_blob[] = {' + ','.join(map(str,timing_raw)) + '};\n'
    proof_checks = []
    for path, value, count in [
        (("IsPreferred",), None, 1), (("IsPreferred",), False, 1),
        (("IsInterlaced",), None, 1), (("IsInterlaced",), True, 1),
        (("HorizontalAttributes", "PixelRepetition"), None, 1),
        (("VerticalAttributes", "PixelRepetition"), 1, 1),
        (("HorizontalAttributes", "SyncPolarity"), None, 1),
        (("VerticalAttributes", "SyncPolarity"), 2, 0),
        (("IsPreferred",), "true", 0),
    ]:
        altered = copy.deepcopy(timing_fixture)
        target = altered[0]
        for key in path[:-1]: target = target[key]
        if value is None: target.pop(path[-1])
        else: target[path[-1]] = value
        raw = bytearray(struct.pack('<I', 0xd3)); wire(altered, raw)
        index = len(proof_checks)
        timing_source += 'static const unsigned char proof_%u[] = {%s};\n' % (index, ','.join(map(str,raw)))
        proof_checks.append((index,count))
    for field in ('IsPreferred', 'HorizontalAttributes'):
        altered = copy.deepcopy(timing_fixture)
        if field == 'IsPreferred':
            altered[0] = ObjectPairs(list(altered[0].items()) + [('IsPreferred', True)])
        else:
            value = altered[0]['HorizontalAttributes']
            altered[0]['HorizontalAttributes'] = ObjectPairs(list(value.items()) + [('SyncPolarity', 1)])
        raw = bytearray(struct.pack('<I', 0xd3)); wire(altered, raw)
        index = len(proof_checks)
        timing_source += 'static const unsigned char proof_%u[] = {%s};\n' % (index, ','.join(map(str,raw)))
        proof_checks.append((index,0))
    proof_test = 'static void timing_proof(void) {\n'
    for index,count in proof_checks:
        proof_test += f"{{ struct dcp_parse_ctx c; unsigned int n=99; assert(!parse(proof_{index},sizeof(proof_{index}),&c)); struct dcp_display_mode *m=enumerate_modes(&c,&n,600,330,0,false,true); assert(!IS_ERR(m) && n=={count}); if(n) assert(!m[0].metadata_timing_valid); free(m); }}\n"
    proof_test += '}\n'
    prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <errno.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;
typedef uint64_t u64;
typedef int64_t s64;
typedef u16 __le16;
typedef u16 __be16;
typedef u32 __le32;
#define __packed __attribute__((packed))
#undef static_assert
#define static_assert(c) _Static_assert(c, #c)
#define IS_ENABLED(x) 0
/* <kunit/visibility.h> without CONFIG_KUNIT, as IS_ENABLED() above */
#define VISIBLE_IF_KUNIT static
#define EXPORT_SYMBOL_IF_KUNIT(symbol)
#define U32_MAX UINT32_MAX
#define U16_MAX UINT16_MAX
#define S64_MAX INT64_MAX
#define S64_MIN INT64_MIN
#define ARRAY_SIZE(a) (sizeof(a) / sizeof((a)[0]))
#define round_up(n, a) (((n) + (a) - 1) & ~((a) - 1))
#define ERR_PTR(n) ((void *)(intptr_t)(n))
#define PTR_ERR(p) ((int)(intptr_t)(p))
#define IS_ERR(p) ((uintptr_t)(p) > (uintptr_t)-4096)
#define IS_ERR_OR_NULL(p) (!(p) || IS_ERR(p))
#define GFP_KERNEL 0
#define kmalloc(n, f) malloc(n)
#define kcalloc(n, s, f) calloc(n,s)
#define kfree(p) free(p)
#define mul_u32_u32(a, b) ((u64)(u32)(a) * (u32)(b))
#define DIV_ROUND_CLOSEST_ULL(n, d) (((n) + (d) / 2) / (d))
#define DIV_ROUND_CLOSEST(n, d) (((n) + (d) / 2) / (d))
#define trace_iomfb_color_mode(...) ((void)0)
#define trace_iomfb_parse_mode_fail(...) ((void)0)
#define trace_iomfb_parse_mode_success(...) ((void)0)
#define trace_iomfb_timing_mode(...) ((void)0)
#define DRM_MODE_TYPE_DRIVER 1
#define DRM_MODE_TYPE_PREFERRED 2
#define DRM_MODE_FLAG_PHSYNC 1
#define DRM_MODE_FLAG_NHSYNC 2
#define DRM_MODE_FLAG_PVSYNC 4
#define DRM_MODE_FLAG_NVSYNC 8
#define cpu_to_le16(v) (v)
#define cpu_to_le32(v) (v)
#define cpu_to_be16(v) __builtin_bswap16(v)
struct drm_display_mode {
    unsigned int type, flags;
    int clock, vdisplay, vsync_start, vsync_end, vtotal;
    int hdisplay, hsync_start, hsync_end, htotal, width_mm, height_mm;
};
static void drm_mode_set_name(struct drm_display_mode *mode) { (void)mode; }
'''
    stubs = r'''
struct drm_edid { struct edid raw; };
static const struct drm_edid *drm_edid_alloc(const void *data, size_t size)
{
    assert(size == 128);
    struct drm_edid *result = malloc(sizeof(*result));
    memcpy(&result->raw, data, size);
    return result;
}
static void drm_edid_free(const struct drm_edid *edid) { free((void *)edid); }
static bool drm_edid_valid(const struct drm_edid *edid)
{
    const u8 *raw = (const u8 *)&edid->raw;
    unsigned int sum = 0;
    for (unsigned int i = 0; i < 128; i++) sum += raw[i];
    return !(sum & 255) && !raw[126] && !memcmp(raw, "\0\xff\xff\xff\xff\xff\xff\0", 8);
}
'''
    suffix = r'''
static void epochs(void)
{
    struct dcp_metadata_epoch e = {0};
    struct dcp_metadata_ticket before = dcp_metadata_begin(&e);
    assert(!dcp_metadata_matches(&e, before));
    dcp_metadata_epoch_connection(&e, 1, true);
    struct dcp_metadata_ticket current = dcp_metadata_begin(&e);
    assert(dcp_metadata_matches(&e, current));
    assert(!dcp_metadata_matches(&e, before)); /* Never retag pre-association metadata. */
    dcp_metadata_epoch_connection(&e, 1, true);
    assert(dcp_metadata_matches(&e, current));
    dcp_metadata_epoch_connection(&e, 2, false);
    assert(!dcp_metadata_matches(&e, current));
    dcp_metadata_epoch_connection(&e, 3, true);
    assert(!dcp_metadata_matches(&e, dcp_metadata_begin(&e))); /* Later attachment ineligible. */
    e = (struct dcp_metadata_epoch){0};
    dcp_metadata_epoch_connection(&e, 9, true);
    current = dcp_metadata_begin(&e);
    dcp_metadata_epoch_connection(&e, 10, true); /* A route change also invalidates. */
    assert(!dcp_metadata_matches(&e, current));
}
int main(int argc, char **argv)
{
    if (argc == 3 && !strcmp(argv[1], "--property")) {
        FILE *f=fopen(argv[2],"rb"); assert(f && !fseek(f,0,SEEK_END));
        long n=ftell(f); assert(n>20 && !fseek(f,0,SEEK_SET));
        unsigned char *b=malloc(n); assert(fread(b,n,1,f)==1); fclose(f);
        assert(dcp_property_complete(b,n-1,n));
        assert(dcp_property_complete(b,n-1,n-1));
        for(unsigned int cut=1;cut<=16;cut++)
            assert(!dcp_property_complete(b,n-1-cut,n-cut));
        assert(!dcp_property_complete(b,n-1,n+1));
        free(b); return 0;
    }
    assert(argc == 4 || argc == 5);
    /* A missing final string byte must not be supplied by zero allocation. */
    unsigned char incomplete[]={0xd3,0,0,0,1,0,0,0x82,1,0,0,0x89,0x42};
    assert(dcp_property_complete(incomplete,sizeof(incomplete),sizeof(incomplete)));
    assert(!dcp_property_complete(incomplete,12,13));
    assert(!dcp_property_complete(NULL,0,0));
    epochs();
    timing_proof();
    FILE *f = fopen(argv[1], "rb");
    assert(f && !fseek(f, 0, SEEK_END));
    long length = ftell(f);
    assert(length > 0 && !fseek(f, 0, SEEK_SET));
    void *data = malloc(length);
    assert(fread(data, length, 1, f) == 1);
    fclose(f);
    struct dcp_parse_ctx ctx;
    struct dcp_display_metadata metadata, unchanged;
    memset(&metadata, 0xa5, sizeof(metadata)); unchanged = metadata;
    assert(!parse(data, length, &ctx));
    int ret = parse_display_metadata(&ctx, &metadata);
    if (atoi(argv[2])) {
        assert(ret == -EINVAL && !memcmp(&metadata, &unchanged, sizeof(metadata)));
        free(data); return 0;
    }
    assert(!ret);
    assert(dcp_property_complete(data,length,length));
    size_t aligned=round_up((size_t)length,4);
    unsigned char *padded=malloc(aligned+4);
    memset(padded,0xa5,aligned+4); memcpy(padded,data,length);
    assert(dcp_property_complete(padded,aligned,aligned+1));
    assert(!dcp_property_complete(data,length-1,length));
    assert(!dcp_property_complete(padded,aligned,aligned+2));
    assert(!dcp_property_complete(padded,aligned+4,aligned+5));
    assert(!dcp_property_complete(padded,aligned-1,aligned+1));
    free(padded);
    struct dcp_parse_ctx timing_ctx;
    unsigned int count = 0;
    assert(!parse(timing_blob, sizeof(timing_blob), &timing_ctx));
    struct dcp_display_mode *timings = enumerate_modes(&timing_ctx, &count,
        metadata.width_cm * 10, metadata.height_cm * 10, 0, false, true);
    assert(!IS_ERR(timings) && count == 1 && timings[0].metadata_timing_valid);
    struct drm_display_mode mode = timings[0].mode;
    assert(mode.flags == (DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC));
    assert(mode.clock == 148500 && mode.hdisplay == 1920 && mode.vdisplay == 1080);
    free(timings);
    const struct drm_edid *edid = dcp_metadata_edid(&metadata, &mode);
    assert(!IS_ERR(edid));
    f = fopen(argv[3], "wb"); assert(f);
    assert(fwrite(&edid->raw, 128, 1, f) == 1); fclose(f);
    drm_edid_free(edid);
    mode.flags = 0;
    assert(IS_ERR(dcp_metadata_edid(&metadata, &mode)));
    mode.flags = DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC;
    mode.clock++;
    assert(IS_ERR(dcp_metadata_edid(&metadata, &mode)));
    free(data);
    if (argc == 5) {
        f=fopen(argv[4],"rb"); assert(f && !fseek(f,0,SEEK_END));
        length=ftell(f); assert(length>0 && !fseek(f,0,SEEK_SET));
        data=malloc(length); assert(fread(data,length,1,f)==1); fclose(f);
        assert(!parse(data,length,&timing_ctx)); count=0;
        timings=enumerate_modes(&timing_ctx,&count,600,330,0,false,true);
        assert(!IS_ERR(timings)); bool found60=false, found120=false;
        for(unsigned int i=0;i<count;i++) {
            assert(timings[i].timing_mode_id!=77 && timings[i].timing_mode_id!=78);
            if(timings[i].timing_mode_id==62) {
                assert(timings[i].color_mode_id==85 && timings[i].sdr_rgb.depth==10);
                assert(timings[i].metadata_timing_valid);
                found60=true;
            }
            if(timings[i].timing_mode_id==74) {
                assert(timings[i].color_mode_id==80 && timings[i].sdr_rgb.depth==8);
                found120=true;
            }
        }
        assert(found60 && found120); free(timings); free(data);
        puts("PASS: actual TimingElements retains 60Hz/color85, 120Hz/color80, excludes 144/180Hz unsafe modes");
    }
    return 0;
}
'''
    return {**locals(), "block": block}

def policy(root):

    base = root / 'drivers/gpu/drm/apple'
    namespace = parser_fixture(root)
    fixture = copy.deepcopy(namespace['timing_fixture'])
    m = fixture[0]
    m['HorizontalAttributes'].update(Total=2080, FrontPorch=48, SyncWidth=32, PreciseSyncRate=1304687407)
    m['VerticalAttributes'].update(Total=1106, FrontPorch=3, SyncWidth=5, PreciseSyncRate=11796583)
    m['ColorModes'][0].update(ID=80, Depth=8)
    m.update(ID=78, UnsafeColorElementIDs=[80], DSCRequiredColorElementIDs=[])
    blobs = ''
    checks = ''

    def case(value, automatic=True, internal=False, external=True, admitted=True):
        nonlocal blobs, checks
        index = blobs.count('static const')
        raw = bytearray(struct.pack('<I', 0xd3))
        namespace['wire'](value, raw)
        blobs += f'static const u8 input_{index}[] = {{{",".join(map(str, raw))}}};\n'
        checks += f'''{{ struct dcp_parse_ctx c; unsigned n=99;
        assert(!parse(input_{index},sizeof(input_{index}),&c));
        c.automatic_1080p180_rgb8={int(automatic)};
        struct dcp_display_mode *m=enumerate_modes(&c,&n,600,330,0,{int(internal)},{int(external)});
        assert(!IS_ERR(m)); bool found=false;
        for(unsigned i=0;i<n;i++) found |= m[i].automatic_rgb8_quirk;
        assert(found=={int(admitted)}); free(m); }}\n'''

    case(fixture)
    case(fixture, automatic=False, admitted=False)
    case(fixture, internal=True, external=False, admitted=False)
    for path, value in [
        (('DSCRequiredColorElementIDs',), [80]),
        (('VerticalAttributes','Total'),1125),
        (('VerticalAttributes','PreciseSyncRate'),11796480),
        (('HorizontalAttributes','PixelRepetition'),1),
        (('IsVirtual',),True), (('IsInterlaced',),True),
        (('ColorModes',0,'Depth'),10), (('ColorModes',0,'EOTF'),2),
        (('ColorModes',0,'DynamicRange'),1), (('ColorModes',0,'PixelEncoding'),2),
        (('ColorModes',0,'Colorimetry'),1),
    ]:
        altered=copy.deepcopy(fixture); target=altered[0]
        for key in path[:-1]: target=target[key]
        target[path[-1]]=value
        case(altered, admitted=False)
    for path, field in [(('HorizontalAttributes',),'Total'),(('ColorModes',0),'Depth'),((),'ID')]:
        altered=copy.deepcopy(fixture);target=altered[0]
        for key in path[:-1]:target=target[key]
        if path:
            target[path[-1]]=namespace['ObjectPairs'](list(target[path[-1]].items())+[(field,target[path[-1]][field])])
        else:
            altered[0]=namespace['ObjectPairs'](list(target.items())+[(field,target[field])])
        case(altered, admitted=False)

    policy='\n'.join(line for line in (base/'dcp-180hz-policy.h').read_text().splitlines() if not line.startswith('#include'))
    proof='\n'.join(line for line in (base/'dcp-180hz.h').read_text().splitlines() if not line.startswith('#include'))
    source=(base/'dcp-180hz.c').read_text().split('static void dcp_180hz_retry_work(')[0]
    source='\n'.join(line for line in source.splitlines() if not line.startswith('#include'))
    block = namespace['block']
    dptx = (base/'dptxep.c').read_text()
    callback_types = '\n'.join(block(dptx, marker)+' __packed;' for marker in ('struct dptxport_apcall_link_rate {', 'struct dptxport_apcall_set_active_lane_count {'))
    callback = block(dptx, 'static int dptxport_call_link_config(')
    dcp = (base/'dcp.c').read_text()
    connection = '\n'.join(block(dcp, marker) for marker in (
        'void dcp_firmware_metadata_connection(', 'static void dcp_firmware_metadata_present_locked(',
        'static void dcp_announce_kick(', 'void dcp_firmware_metadata_announced(',
        'void dcp_firmware_metadata_hotplug('))
    state=r'''
#define U64_MAX UINT64_MAX
#define READ_ONCE(v) (v)
#define WRITE_ONCE(p,v) ((p)=(v))
#define DCP_IOMFB_METHODS_H17P_EXTERNAL 2
#define DPTX_ENDPOINT_V2 0x2c
#define DRM_CONNECTOR_REGISTERED 1
#define to_platform_device(p) (p)
#define lockdep_assert_held(p) assert(*(p))
#define spin_lock_irqsave(p,f) do { assert(!*(p)); *(p)=true; (f)=0; } while(0)
#define spin_unlock_irqrestore(p,f) do { (void)(f); assert(*(p)); *(p)=false; } while(0)
struct work_struct { unsigned scheduled; };
struct drm_edid { u8 data[256]; };
struct apple_connector {
    void *dcp; bool connected, drm_edid_firmware_derived, edid_invalidation_pending;
    u64 drm_edid_generation, drm_edid_service_epoch;
    const struct drm_edid *drm_edid;
    struct { int registration_state; } base;
    struct work_struct hotplug_wq, bl_sync_wq;
};
struct apple_epic { struct apple_dcp *dcp; unsigned endpoint; bool wire_v2; };
struct apple_epic_service { struct apple_epic *ep; void *cookie; bool enabled; };
struct dptx_port { u32 lane_count, link_rate; bool enabled, closed, connected; void *atcphy; struct apple_epic_service *service; };
struct apple_dcp {
    struct { int iomfb_method_profile; bool fw_14_7_endpoints; } hw;
    struct { bool lock, attributes_valid, timing_valid, hpd_seen; struct dcp_metadata_epoch epoch; struct dcp_metadata_ticket catalog_ticket;
        u64 catalog_connection_sequence; } metadata;
    struct { bool lock, enabled; void *service; u64 epoch; } dcpavserv;
    struct dcp_180hz_proof rate_proof;
    struct dptx_port dptxport[1];
    struct apple_connector *connector;
    void *dev, *phy, *active_typec_route;
    bool dptx_tunnel, usb4, typec_cable_connected, quiescing, hpd_mutex;
    u64 typec_generation; unsigned int nr_modes;
    struct dcp_display_mode *modes;
    struct { bool holding, stopped; } announce;
    bool ext_backlight, has_panel;
};
struct rate_guard { bool *lock; };
static struct rate_guard rate_lock(bool *p) { assert(!*p);*p=true;return (struct rate_guard){p}; }
static void rate_unlock(struct rate_guard *g) { assert(*g->lock);*g->lock=false; }
#define guard(kind) struct rate_guard __attribute__((cleanup(rate_unlock))) _guard = rate_lock
static bool dcp_is_usb4_output(struct apple_dcp *d) { return d->usb4; }
static void schedule_work(struct work_struct *w) { w->scheduled++; }
static bool __attribute__((unused)) dcp_has_panel(struct apple_dcp *d) { return d->has_panel; }
static const struct edid *drm_edid_raw(const struct drm_edid *e) { return (const void *)e->data; }
static bool dcpavserv_edid_current(struct apple_dcp *d, u64 generation) {
    assert(d->hpd_mutex);return d->typec_cable_connected&&generation==d->typec_generation&&!d->quiescing;
}
static bool dcpavserv_edid_service_current(struct apple_dcp *d, u64 e) {
    assert(d->dcpavserv.lock); return e==d->dcpavserv.epoch && d->dcpavserv.enabled && d->dcpavserv.service;
}
'''
    callback_stubs=r'''
#define le64_to_cpu(v) (v)
#define le32_to_cpu(v) (v)
#define LINK_RATE_HBR3 0x1e
static int configure_error;
static int dptxport_call_set_active_lane_count(struct apple_epic_service *s, const void *data, size_t size, void *reply, size_t room) {
    if (configure_error) return configure_error;
    if(size<sizeof(struct dptxport_apcall_set_active_lane_count) || room<sizeof(struct dptxport_apcall_set_active_lane_count)) return -EINVAL;
    *(struct dptxport_apcall_set_active_lane_count *)reply=*(const struct dptxport_apcall_set_active_lane_count *)data;
    ((struct dptx_port *)s->cookie)->lane_count=((const struct dptxport_apcall_set_active_lane_count *)data)->lane_count; return 0;
}
static int dptxport_call_set_link_rate(struct apple_epic_service *s, const void *data, size_t size, void *reply, size_t room) {
    if(configure_error) return configure_error;
    if(size<sizeof(struct dptxport_apcall_link_rate) || room<sizeof(struct dptxport_apcall_link_rate)) return -EINVAL;
    *(struct dptxport_apcall_link_rate *)reply=*(const struct dptxport_apcall_link_rate *)data;
    ((struct dptx_port *)s->cookie)->link_rate=((const struct dptxport_apcall_link_rate *)data)->link_rate; return 0;
}
static bool dcp_firmware_metadata_enabled(struct apple_dcp *d) { return d->hw.iomfb_method_profile==2; }
#define dcp_trace_metadata(d,p,v) ((void)0)
'''
    callback_types='typedef u64 __le64;\n'+callback_types
    test=r'''
static void checksum(u8 *p) { unsigned sum=0; for(unsigned i=0;i<127;i++) sum+=p[i]; p[127]=-sum; }
static void qualify(struct apple_dcp *d, struct apple_epic_service *s) {
    struct dptxport_apcall_link_rate rate={.link_rate=0x1e}, rate_reply={};
    struct dptxport_apcall_set_active_lane_count lanes={.lane_count=2}, lanes_reply={};
    assert(!dptxport_call_link_config(s,false,&rate,sizeof(rate),&rate_reply,sizeof(rate_reply)));
    assert(!dptxport_call_link_config(s,true,&lanes,sizeof(lanes),&lanes_reply,sizeof(lanes_reply)));
    (void)d;
}
static bool current(struct apple_dcp *d,const struct dcp_display_mode *m) {
    d->hpd_mutex=d->dcpavserv.lock=d->metadata.lock=true;
    bool result=dcp_180hz_mode_current(d,m);
    d->hpd_mutex=d->dcpavserv.lock=d->metadata.lock=false;
    return result;
}
int main(int argc,char **argv) {
    struct dcp_metadata_epoch epoch={};
    dcp_metadata_epoch_connection(&epoch,7,true);
    assert(dcp_metadata_matches(&epoch,dcp_metadata_begin(&epoch)));
    PARSER_CHECKS
    struct drm_edid raw={}; u8 *p=raw.data;
    struct dcp_display_mode m={};
    if(argc==2) { FILE *f=fopen(argv[1],"rb"); assert(f); assert(fread(p,1,256,f)==256 && fgetc(f)==EOF); fclose(f); }
    else {
        memcpy(p,"\0\xff\xff\xff\xff\xff\xff\0",8);
        p[8]=4;p[9]=0x72;p[10]=0x4f;p[11]=0x0b;p[18]=1;p[19]=4;p[20]=0xa0;p[126]=1;
        u8 *d=p+54; unsigned clock=41409;d[0]=clock;d[1]=clock>>8;
        d[2]=1920&255;d[3]=160;d[4]=0x70;d[5]=1080&255;d[6]=26;d[7]=0x40;
        d[8]=48;d[9]=32;d[10]=0x35;d[17]=0x1e;
        p[75]=0xfc;memcpy(p+77,"QG270 S3\n    ",13);
        u8 *e=p+128;e[0]=2;e[1]=3;e[2]=12;e[4]=0x67;e[5]=0xd8;e[6]=0x5d;e[7]=0xc4;e[8]=1;e[9]=120;e[10]=0x80;
        checksum(p);checksum(e);
    }
    /* Get the advertised DTD's porch/polarity, without copying identity data. */
    const u8 *dt=NULL;
    for(unsigned off=54;off+18<=126;off+=18) if((p[off]|p[off+1]<<8)==41409)dt=p+off;
    for(unsigned off=p[130];off+18<=127;off+=18) if((p[128+off]|p[129+off]<<8)==41409)dt=p+128+off;
    assert(dt);
    m.mode=(struct drm_display_mode){.clock=414090,.hdisplay=1920,.htotal=2080,.vdisplay=1080,.vtotal=1106};
    m.mode.hsync_start=1920+(dt[8]|(dt[11]&0xc0)<<2);m.mode.hsync_end=m.mode.hsync_start+(dt[9]|(dt[11]&0x30)<<4);
    m.mode.vsync_start=1080+(dt[10]>>4|(dt[11]&0xc)<<2);m.mode.vsync_end=m.mode.vsync_start+((dt[10]&15)|(dt[11]&3)<<4);
    m.mode.flags=(dt[17]&2?DRM_MODE_FLAG_PHSYNC:DRM_MODE_FLAG_NHSYNC)|(dt[17]&4?DRM_MODE_FLAG_PVSYNC:DRM_MODE_FLAG_NVSYNC);
    assert(dcp_180hz_edid(p,256,&m.mode));
    assert(!dcp_180hz_edid(p,255,&m.mode));
    u8 saved[256];memcpy(saved,p,256);
    p[10]^=1;checksum(p);assert(!dcp_180hz_edid(p,256,&m.mode));memcpy(p,saved,256);
    p[8]^=1;checksum(p);assert(!dcp_180hz_edid(p,256,&m.mode));memcpy(p,saved,256);
    unsigned hf=0;for(unsigned off=132;off<128+p[130];off+=(p[off]&31)+1) if(p[off+1]==0xd8 && p[off+2]==0x5d && p[off+3]==0xc4)hf=off;
    assert(hf);p[hf+5]=82;checksum(p+128);assert(!dcp_180hz_edid(p,256,&m.mode));memcpy(p,saved,256);
    p[127]^=1;assert(!dcp_180hz_edid(p,256,&m.mode));memcpy(p,saved,256);
    /* Serial changes do not participate in identity matching. */
    p[12]^=0xff;checksum(p);assert(dcp_180hz_edid(p,256,&m.mode));memcpy(p,saved,256);
    assert(dcp_180hz_payload(0x1e,2,414090));assert(!dcp_180hz_payload(0x14,2,414090));assert(!dcp_180hz_payload(0x1e,1,414090));
    struct apple_dcp d={.dev=&d,.phy=&raw,.active_typec_route=&m,.typec_cable_connected=true,.typec_generation=7,.nr_modes=1};
    struct apple_connector con={.dcp=&d,.connected=true,.drm_edid=&raw,.drm_edid_generation=7,.drm_edid_service_epoch=1,.base.registration_state=1};
    struct apple_epic ep={.dcp=&d,.endpoint=0x2c,.wire_v2=true};
    struct apple_epic_service s={.ep=&ep,.cookie=&d.dptxport[0],.enabled=true};
    d.connector=&con;d.hw.iomfb_method_profile=2;d.metadata.epoch.connected=true;d.metadata.epoch.generation=7;
    d.dcpavserv.enabled=true;d.dcpavserv.service=&raw;d.dcpavserv.epoch=1;
    d.dptxport[0]=(struct dptx_port){.enabled=true,.connected=true,.atcphy=d.phy,.service=&s};
    m.unsafe_rgb8_ticket=(struct dcp_metadata_ticket){.associated=true,.generation=7};m.unsafe_rgb8_service_epoch=1;
    m.sdr_rgb=(struct dcp_color_mode){.depth=8,.format=0,.range=0,.eotf=0,.colorimetry=10};
    assert(!current(&d,&m));qualify(&d,&s);assert(current(&d,&m));assert(con.hotplug_wq.scheduled==1);
    #define DENY(field,value) do { __typeof__(field) old=field;field=value;assert(!current(&d,&m));field=old;assert(current(&d,&m)); } while(0)
    DENY(d.typec_generation,8);DENY(con.drm_edid_generation,6);DENY(d.dcpavserv.epoch,3);DENY(con.drm_edid_service_epoch,3);
    /* Timings may precede the very first AV announcement. Only fresh epoch-1
     * EDID plus epoch-1 callback proof may admit this initial receipt. */
    m.unsafe_rgb8_service_epoch=0;m.automatic_service_pending=true;assert(current(&d,&m));
    d.dcpavserv.epoch=3;con.drm_edid_service_epoch=3;qualify(&d,&s);
    assert(!current(&d,&m)); /* Initial receipt never survives retirement. */
    m.unsafe_rgb8_service_epoch=3;m.automatic_service_pending=false;assert(current(&d,&m));
    d.dcpavserv.epoch=1;con.drm_edid_service_epoch=1;m.unsafe_rgb8_service_epoch=1;qualify(&d,&s);
    DENY(con.drm_edid_firmware_derived,true);DENY(d.usb4,true);DENY(d.dptx_tunnel,true);
    DENY(d.quiescing,true);DENY(d.metadata.epoch.connected,false);DENY(d.rate_proof.connection_sequence,1);
    DENY(d.rate_proof.rate.value,0x14);DENY(d.rate_proof.lanes.value,1);DENY(ep.endpoint,0x28);
    DENY(m.sdr_rgb.depth,10);DENY(m.sdr_rgb.colorimetry,1);DENY(m.sdr_rgb.range,1);DENY(m.mode.clock,414091);
    struct dptxport_apcall_link_rate rate={.link_rate=0x1e}, response={};
    configure_error=-EIO;
    assert(dptxport_call_link_config(&s,false,&rate,sizeof(rate),&response,sizeof(response))==-EIO);
    assert(!current(&d,&m));configure_error=0;qualify(&d,&s);
    assert(dptxport_call_link_config(&s,false,&rate,sizeof(rate)-1,&response,sizeof(response))==-EINVAL);
    assert(!current(&d,&m));qualify(&d,&s);
    dcp_firmware_metadata_connection(&d,7,false);dcp_firmware_metadata_connection(&d,7,true);
    assert(!current(&d,&m));qualify(&d,&s);assert(!current(&d,&m));
    m.automatic_connection_sequence=d.rate_proof.connection_sequence;assert(current(&d,&m));
    dcp_firmware_metadata_hotplug(&d,true);dcp_firmware_metadata_hotplug(&d,false);
    assert(!current(&d,&m));
    dcp_firmware_metadata_connection(&d,7,true);m.automatic_connection_sequence=d.rate_proof.connection_sequence;con.edid_invalidation_pending=false;qualify(&d,&s);assert(current(&d,&m));
    struct dcp_180hz_token old=dcp_180hz_config_begin(&s,false);d.typec_generation++;
    dcp_180hz_config_complete(&s,false,old,0x1e,true);assert(!d.rate_proof.rate.valid);d.typec_generation--;qualify(&d,&s);
    old=dcp_180hz_config_begin(&s,true);d.dcpavserv.epoch+=2;
    dcp_180hz_config_complete(&s,true,old,2,true);assert(d.rate_proof.lanes.valid);assert(!current(&d,&m));d.dcpavserv.epoch-=2;qualify(&d,&s);
    old=dcp_180hz_config_begin(&s,false);dcp_180hz_invalidate(&d);
    dcp_180hz_config_complete(&s,false,old,0x1e,true);assert(!current(&d,&m));qualify(&d,&s);
    old=dcp_180hz_config_begin(&s,false);dcp_180hz_config_complete(&s,false,old,0x1e,false);assert(!current(&d,&m));
    d.dcpavserv.epoch=0;old=dcp_180hz_config_begin(&s,false);d.dcpavserv.epoch=1;
    dcp_180hz_config_complete(&s,false,old,0x1e,true);assert(d.rate_proof.rate.valid);qualify(&d,&s);assert(current(&d,&m));
    puts("PASS: actual binary parser, Acer EDID, capacity, callback success, late-proof event, generation/AV lifetime and foreign-sink controls");
}
'''.replace('PARSER_CHECKS', checks)
    unit=namespace['prefix']+namespace['metadata']+namespace['epoch']+namespace['header']+namespace['edid_header']+namespace['parser']+blobs+policy+proof+state+source+callback_types+callback_stubs+callback+connection+test
    return {**locals(), "block": block}

def publication(root):

    n = policy(root)
    n.update({key:n['namespace'][key] for key in ('prefix','metadata','epoch','header','edid_header','parser')})
    base = n['base']
    block = n['block']
    av = (base/'epic/dpavservep.c').read_text()
    iomfb = (base/'iomfb.c').read_text()
    template = (base/'iomfb_template.c').read_text()

    state = n['state']
    state = state.replace('struct apple_connector {', '''struct drm_device { void *dev; struct { bool mutex; } mode_config; };
struct drm_connector { struct drm_device *dev; int registration_state; void *state;
    struct { struct { unsigned min_vfreq,max_vfreq; } monitor_range; int width_mm,height_mm; } display_info;
};
struct apple_connector {''')
    state = state.replace('    struct { int registration_state; } base;', '    struct drm_connector base;')
    state = state.replace('struct apple_epic_service { struct apple_epic *ep; void *cookie; bool enabled; };', '''struct apple_epic_service { struct apple_epic *ep; void *cookie; bool enabled,torndown;
    unsigned users,channel; struct { u64 generation,epoch; } admission;
};''')
    state = state.replace('struct { bool lock, enabled; void *service; u64 epoch; } dcpavserv;', 'struct { bool lock, enabled; void *service, *opened; u64 epoch,opened_epoch; int enable_completion; struct work_struct close_wq; } dcpavserv;')
    # Catalogue/EDID ownership: modes are admitted per attachment under modes_lock.
    state = state.replace('    struct work_struct hotplug_wq, bl_sync_wq;\n};', '    struct work_struct hotplug_wq, bl_sync_wq;\n    bool edid_live; unsigned hotplug_reasons;\n};', 1)
    state = state.replace('u64 typec_generation; unsigned int nr_modes;', '''u64 typec_generation; unsigned int nr_modes;
    struct apple_epic *dcpavservep;
    int fw_compat,width_mm,height_mm,fixed_connector_type; unsigned notch_height;
    bool modes_lock,modes_admitted,modes_provisional,placeholder_retried,external_native; int external_held;
    u64 modes_generation,dimensions_generation; struct work_struct dimensions_wq;
    struct { int width_mm,height_mm; struct { char vendor[4]; } id; } panel;
    struct { size_t length,received; void *data; bool contiguous,payload_complete; u64 modes_generation;
        u64 av_service_epoch,automatic_connection_sequence; bool automatic_service_pending;
        struct dcp_metadata_ticket metadata_ticket,mode_ticket;
    } chunks;''')

    start = state.index('static bool dcpavserv_edid_current(')
    state = state[:start]

    stubs = r'''
#define WRITE_ONCE(p,v) ((p)=(v))
#define platform_device apple_dcp
#define platform_get_drvdata(p) ((struct apple_dcp *)(p))
#define to_apple_connector(p) ((struct apple_connector *)((char *)(p)-offsetof(struct apple_connector,base)))
#define xchg(p,v) __extension__ ({ __typeof__(*(p)) old=*(p); *(p)=(v); old; })
#define DRM_MODE_MATCH_TIMINGS 1
#define DRM_MODE_MATCH_CLOCK 2
#define DCP_FIRMWARE_H17P 2
#define DPAVSERV_ENDPOINT_V2 0x2a
#define DRM_CONNECTOR_eDP 14
#define DRM_MODE_CONNECTOR_eDP 14
#define DCP_FW_VER 26
#define DCP_FW_VERSION(a,b,c) (a)
#define SZ_1M (1024*1024)
#define kzalloc(n,f) calloc(1,n)
#define dev_err(...) ((void)0)
#define dev_warn(...) ((void)0)
#define dev_info(...) ((void)0)
#define dev_dbg(...) ((void)0)
#define trace_dcpavserv_init(...) ((void)0)
#include <stddef.h>
static bool *hpd_lock,*av_lock;
static void mutex_lock(bool *p) { assert(!*p); if(p==hpd_lock) assert(!*av_lock); *p=true; }
static void mutex_unlock(bool *p) { assert(*p); *p=false; }
static void complete(int *p) { (*p)++; }
static void reinit_completion(int *p) { *p=0; }
static struct apple_epic_service *afk_service_get(struct apple_epic_service *s) { if(!s->enabled) return NULL; s->users++; return s; }
static void afk_service_put(struct apple_epic_service *s) { assert(s->users); s->users--; }
static void afk_service_retire(struct apple_epic_service *s) { assert(!*hpd_lock); s->enabled=false; s->torndown=true; }
/* These H17P scenarios never queue the native 14.7 Close worker. */
static bool cancel_work_sync(struct work_struct *w) { assert(!w->scheduled); return false; }
/* Native audio is out of scope: reject entry instead of silently queueing it. */
static void dcp_external_audio_start(struct apple_dcp *d) { assert(!d->external_native); }
/* These H17P scenarios never reach the 14.7 panel's base EDID. */
#define dev_warn_once(...) ((void)0)
static bool dcp_is_v14_7_panel(const struct apple_dcp *d) { (void)d; return false; }
static int dcp_panel_edid_update(struct drm_connector *c,const void *id,int w,int h) { (void)c;(void)id;(void)w;(void)h;assert(false);return 0; }
static void dcp_update_connector_dimensions(struct apple_dcp *d,struct drm_connector *c) { (void)d;(void)c;assert(false); }
static void dpavservep_detach(struct apple_dcp *d);
static struct apple_dcp *probing;
static struct drm_edid transport_edid;
static unsigned raw_allocations,published,reads,retirement_during_read;
static bool fail_update,invalidate_during_duplicate,replace_catalog_during_duplicate;
static void conflicting_catalog_arrived(void);
static const struct drm_edid *dcpavserv_copy_edid(struct apple_dcp *d,u64 generation,u64 *epoch) {
    assert(!d->hpd_mutex && !d->dcpavserv.lock && !d->metadata.lock);
    if(!d->dcpavserv.enabled || !d->dcpavserv.service || generation!=d->typec_generation) return ERR_PTR(-ENODEV);
    *epoch=d->dcpavserv.epoch;reads++;
    struct drm_edid *e=malloc(sizeof(*e)); assert(e); *e=transport_edid;raw_allocations++;
    if(retirement_during_read) { retirement_during_read--;dpavservep_detach(d); }
    return e;
}
static void drm_edid_free(const struct drm_edid *e) { if(e) { assert(raw_allocations);raw_allocations--;free((void *)e); } }
static int drm_edid_connector_update(struct drm_connector *c,const struct drm_edid *e) {
    assert(c->dev->mode_config.mutex); (void)e; return fail_update?-EIO:0;
}
static const struct drm_edid *dcp_firmware_edid(struct apple_dcp *d,u64 *generation) { (void)d; (void)generation; return ERR_PTR(-ENODATA); }
static bool dcp_firmware_metadata_current(struct apple_dcp *d,u64 generation) { (void)d; (void)generation; return false; }
static struct drm_display_mode *drm_mode_duplicate(struct drm_device *dev,const struct drm_display_mode *m) {
    assert(dev->mode_config.mutex);struct drm_display_mode *copy=malloc(sizeof(*copy));assert(copy);*copy=*m;
    if(invalidate_during_duplicate) { invalidate_during_duplicate=false;dcp_180hz_invalidate(probing); }
    if(replace_catalog_during_duplicate) { replace_catalog_during_duplicate=false;conflicting_catalog_arrived(); }
    return copy;
}
static void drm_mode_probed_add(struct drm_connector *c,struct drm_display_mode *m) {
    assert(c->dev->mode_config.mutex && probing->hpd_mutex && probing->dcpavserv.lock && probing->metadata.lock);
    published++;free(m);
}
static void drm_mode_destroy(struct drm_device *dev,struct drm_display_mode *m) { (void)dev; free(m); }
static bool drm_mode_match(const struct drm_display_mode *a,const struct drm_display_mode *b,unsigned flags) {
    (void)flags;return a->clock==b->clock && a->hdisplay==b->hdisplay && a->vdisplay==b->vdisplay && a->htotal==b->htotal && a->vtotal==b->vtotal && a->hsync_start==b->hsync_start && a->hsync_end==b->hsync_end && a->vsync_start==b->vsync_start && a->vsync_end==b->vsync_end && a->flags==b->flags;
}
static void drm_connector_set_vrr_capable_property(struct drm_connector *c,bool capable) { (void)c; (void)capable; }
static void dcp_retry_placeholder_edid(struct apple_dcp *d,const struct drm_edid *e) { (void)d; (void)e; }
static unsigned pending_probes;
void dcp_180hz_retry(struct apple_dcp *d) { (void)d;pending_probes++; }
static void dcp_firmware_metadata_begin(struct apple_dcp *d) {
    unsigned long flags;spin_lock_irqsave(&d->metadata.lock,flags);
    d->chunks.metadata_ticket=dcp_metadata_begin(&d->metadata.epoch);spin_unlock_irqrestore(&d->metadata.lock,flags);
}
static void dcp_firmware_metadata_timing(struct apple_dcp *d) { (void)d; }
static void dcp_firmware_metadata_attributes(struct apple_dcp *d,const struct dcp_display_metadata *m) { (void)d; (void)m; }
/* Deferred to dimensions_wq since the catalogue ownership change. */
static void __attribute__((unused)) dcp_set_dimensions(struct apple_dcp *d) { (void)d; }
struct dcp_set_dcpav_prop_end_req { char key[64]; };
static void rate_scoped_unlock(struct rate_guard *g) { if(g->lock) rate_unlock(g); }
#define scoped_guard(kind,l) for(struct rate_guard _scoped __attribute__((cleanup(rate_scoped_unlock)))=rate_lock(l);_scoped.lock;rate_unlock(&_scoped),_scoped.lock=NULL)
#define atomic_read(p) (*(p))
#define atomic_or(v,p) (*(p)|=(v))
#define SZ_16M (16*1024*1024)
#ifndef BIT
#define BIT(n) (1UL << (n))
#endif
static void *kmemdup_array(const void *src,size_t n,size_t size,int gfp) { (void)gfp;void *p=malloc(n*size);if(p)memcpy(p,src,n*size);return p; }
static void disable_work_sync(struct work_struct *w) { (void)w; }
#ifndef DCP_FIRMWARE_V_14_7
#define DCP_FIRMWARE_V_14_7 (DCP_FIRMWARE_H17P + 1)
#define DCP_FIRMWARE_V_14_7_TEMPLATE (DCP_FIRMWARE_H17P + 2)
#endif
'''+internal_inline(root,'dcp_fw_is_14_7')+r'''/* Off T6030 machines the direct crossbar output is not clocked separately. */
static int __attribute__((unused)) dcp_direct_crossbar_link(struct apple_dcp *d,bool up) { (void)d;(void)up;return 0; }
'''
    transport = '\n'.join(block(av,marker) for marker in [
        'static void dcpavserv_notify_edid(', 'static bool dcpavserv_identity(',
        'static bool dcpavserv_admission_current(', 'static int dcpavserv_prepare(',
        'static void dcpavserv_init(', 'static void dcpavserv_invalidate_edid(',
        'static void dcpavserv_teardown(', 'void dpavservep_detach(',
        'bool dcpavserv_edid_current(', 'bool dcpavserv_edid_service_current(',
    ])
    mode_helpers = iomfb[iomfb.index('static struct dcp_display_mode *dcp_modes_snapshot('):iomfb.index('int dcp_get_modes(')]
    # The catalogue owner check, hotplug queueing, connector EDID helpers and
    # property bounds the catalogue helpers use, all actual.
    property_header = base/'dcp-property.h'
    reasons = re.search(r'^enum dcp_hotplug_reason \{', iomfb, re.M)
    mode_helpers = (''.join(line+'\n' for line in (property_header.read_text().splitlines() if property_header.exists() else ())
                            if not line.startswith('#include')).replace('static inline ', 'static inline __attribute__((unused)) ')
                    + (block(iomfb, reasons.start())+';\n' if reasons else '')
                    + ''.join(apple_function(root, name) for name in ('dcp_queue_hotplug', 'dcp_modes_for_connector',
                              'apple_connector_invalidate_edid', 'apple_connector_edid_set_live'))
                    + mode_helpers)
    publication = block(iomfb,'static void dcp_clear_connector_edid(')+'\n'+block(iomfb,'static void dcp_consume_edid_invalidation(')+mode_helpers+block(iomfb,'int dcp_get_modes(')+block(iomfb,re.search(r'^[^\n;]*\blookup_mode\(',iomfb,re.M).start())
    # Drive malformed replacements through the same chunk/end ingress as firmware.
    # The dictionary cache is a boundary substitute and takes ownership of the blob.
    receipt = r'''
struct dcp_set_dcpav_prop_chunk_req { u32 offset,length; u8 data[4096]; };
#define dcp_connector_update_dict(con,key,chunks) do { (void)(con); (void)(key); free((chunks)->data); } while (0)
''' + '\n'.join(block(template,marker) for marker in [
        'static u8 dcpep_cb_prop_start(', 'static u8 dcpep_cb_prop_chunk(',
        'static bool dcpep_process_chunks(', 'static u8 dcpep_cb_prop_end('])

    main_prefix = n['test'].split('int main(int argc,char **argv) {',1)[1].split('    struct apple_dcp d={',1)[0]
    helpers = n['test'].split('int main(int argc,char **argv) {',1)[0]
    helpers = helpers.replace(block(helpers, 'static void qualify('), '')
    fixture = r'''
static struct apple_dcp d;
static struct apple_connector con;
static struct drm_device drm;
static struct apple_epic ep,av_ep;
static struct apple_epic_service s,av_services[3];
static void reset(void) {
    assert(!raw_allocations);
    d=(struct apple_dcp){.dev=&d,.phy=&transport_edid,.active_typec_route=&con,.typec_cable_connected=true,.typec_generation=7,.fw_compat=2,.width_mm=600,.height_mm=330};
    drm=(struct drm_device){};
    /* Bound to its pipeline, so its EDID is live (apple_connector_set_pipeline()). */
    con=(struct apple_connector){.dcp=&d,.connected=true,.edid_live=true,.base={.dev=&drm,.registration_state=1}};
    ep=(struct apple_epic){.dcp=&d,.endpoint=0x2c,.wire_v2=true};
    av_ep=(struct apple_epic){.dcp=&d,.endpoint=0x2a,.wire_v2=true};
    s=(struct apple_epic_service){.ep=&ep,.cookie=&d.dptxport[0],.enabled=true};
    for(unsigned i=0;i<3;i++) av_services[i]=(struct apple_epic_service){.ep=&av_ep,.enabled=true,.channel=1+4*i};
    d.connector=&con;d.hw.iomfb_method_profile=2;d.metadata.epoch.connected=true;d.metadata.epoch.generation=7;dcp_metadata_epoch_connection(&d.metadata.epoch,7,true);
    d.dptxport[0]=(struct dptx_port){.enabled=true,.connected=true,.atcphy=d.phy,.service=&s};
    d.dcpavservep=&av_ep;hpd_lock=&d.hpd_mutex;av_lock=&d.dcpavserv.lock;probing=&d;
    published=reads=pending_probes=0;fail_update=invalidate_during_duplicate=false;
}
static void admit(unsigned index) {
    struct apple_epic_service *a=&av_services[index];
    assert(!dcpavserv_prepare(a,"dcpav-service-epic","AppleDCPDP2HDMI",a->channel));
    dcpavserv_init(a,"dcpav-service-epic","AppleDCPDP2HDMI",a->channel);
    assert(d.dcpavserv.enabled && d.dcpavserv.service==a);
}
static void start_receipt(void) {
    u32 size=sizeof(input_0);assert(dcpep_cb_prop_start(&d,&size));
    memcpy(d.chunks.data,input_0,size);d.chunks.received=size;d.chunks.contiguous=true;
}
static void finish_receipt(void) {
    struct dcp_set_dcpav_prop_end_req request={.key="TimingElements"};
    assert(dcpep_process_chunks(&d,&request));assert(d.nr_modes==1 && d.modes[0].automatic_rgb8_quirk);
    free(d.chunks.data);d.chunks.data=NULL;
}
static void timings(void) { start_receipt();finish_receipt(); }
static void callbacks(bool lanes) {
    if(lanes) { struct dptxport_apcall_set_active_lane_count a={.lane_count=2},b={};
        assert(!dptxport_call_link_config(&s,true,&a,sizeof(a),&b,sizeof(b))); }
    else { struct dptxport_apcall_link_rate a={.link_rate=0x1e},b={};
        assert(!dptxport_call_link_config(&s,false,&a,sizeof(a),&b,sizeof(b))); }
}
static void probe(bool expected) {
    drm.mode_config.mutex=true;unsigned before=published;
    int count=dcp_get_modes(&con.base);drm.mode_config.mutex=false;
    assert(count==(int)expected && published==before+(unsigned)expected);
    if(d.nr_modes) assert(!!lookup_mode(&d,&d.modes[0].mode)==expected);
    assert(!d.hpd_mutex && !d.dcpavserv.lock && !d.metadata.lock);
}
static void cleanup(void) {
    drm_edid_free(con.drm_edid);con.drm_edid=NULL;free(d.modes);d.modes=NULL;
    free(d.chunks.data);d.chunks.data=NULL;
    if(d.dcpavserv.service)dpavservep_detach(&d);
    assert(!raw_allocations);
}
static void permutations(void) {
    unsigned runs=0;
    for(unsigned a=0;a<4;a++) for(unsigned b=0;b<4;b++) for(unsigned c=0;c<4;c++) for(unsigned e=0;e<4;e++) {
        if(a==b || a==c || a==e || b==c || b==e || c==e)continue;
        reset();unsigned order[]={a,b,c,e};unsigned seen=0;
        for(unsigned i=0;i<4;i++) {
            unsigned notifications=con.hotplug_wq.scheduled;
            switch(order[i]) { case 0:callbacks(false);break;case 1:callbacks(true);break;case 2:timings();break;case 3:admit(0);break; }
            seen|=1u<<order[i];probe(seen==15);
            if(seen==15)assert(con.hotplug_wq.scheduled>notifications);
        }
        assert(current(&d,&d.modes[0]));cleanup();runs++;
    }
    assert(runs==24);
}
'''
    main=r'''
    transport_edid=raw;
    /* Runtime EDID porches are encoded into the timing fixture by this
     * harness, without embedding identity bytes or fabricating tickets. */
    permutations();
    reset();callbacks(false);callbacks(true);timings();probe(false);admit(0);probe(true);
    assert(d.rate_proof.rate.valid && d.rate_proof.lanes.valid); /* No second callback. */
    struct dcp_display_mode old=d.modes[0];
    dcpavserv_teardown(&av_services[0]);probe(false);
    admit(1);probe(false);assert(!memcmp(&old,d.modes,sizeof(old))); /* No retagging. */
    free(d.modes);d.modes=NULL;timings();probe(true); /* Existing physical config survives AV-only CLOSE. */
    dcpavserv_teardown(&av_services[0]);probe(true); /* Old CLOSE cannot revoke the new owner. */
    cleanup();

    reset();callbacks(false);callbacks(true);timings();admit(0);probe(true);
    dcp_firmware_metadata_connection(&d,8,false);d.typec_generation=8;d.typec_cable_connected=false;dpavservep_detach(&d);probe(false);
    d.typec_generation=9;d.typec_cable_connected=true;dcp_firmware_metadata_connection(&d,9,true);
    free(d.modes);d.modes=NULL;timings();admit(1);probe(false); /* Fresh EDID cannot rescue old PHY configuration. */
    callbacks(false);probe(false);callbacks(true);probe(true);cleanup();

    reset();callbacks(false);callbacks(true);start_receipt();dpavservep_detach(&d);admit(0);finish_receipt();probe(false);cleanup();
    reset();callbacks(false);callbacks(true);admit(0);start_receipt();dcpavserv_teardown(&av_services[0]);admit(1);finish_receipt();probe(false);cleanup();
    reset();callbacks(false);callbacks(true);dpavservep_detach(&d);timings();admit(0);probe(true);cleanup(); /* Legitimate receipt awaiting post-retirement admission. */

    reset();callbacks(false);callbacks(true);timings();admit(0);retirement_during_read=1;probe(false);cleanup();
    reset();callbacks(false);callbacks(true);timings();admit(0);invalidate_during_duplicate=true;probe(false);cleanup();
    reset();callbacks(false);callbacks(true);timings();admit(0);fail_update=true;probe(false);assert(!con.drm_edid);cleanup();
    reset();callbacks(false);callbacks(true);timings();admit(0);transport_edid.data[10]^=1;checksum(transport_edid.data);probe(false);cleanup();transport_edid=raw;
    reset();callbacks(false);callbacks(true);timings();admit(0);configure_error=-EIO;
    struct dptxport_apcall_link_rate req={.link_rate=0x1e},resp={};assert(dptxport_call_link_config(&s,false,&req,sizeof(req),&resp,sizeof(resp))==-EIO);
    configure_error=0;probe(false);cleanup();
    puts("PASS: 24 callback/timing/AV arrival permutations and real get_modes/lookup publication; early configuration needs no second callback; CLOSE, physical reconnect, receipt epochs and publication races deny stale proof");
}
'''
    unit=n['prefix']+n['metadata']+n['epoch']+n['header']+n['edid_header']+n['parser']+n['blobs']+n['policy']+n['proof']+state+stubs+transport+n['source']+n['callback_types']+n['callback_stubs']+n['callback']+n['connection']+publication+receipt+helpers+fixture+'int main(int argc,char **argv) {'+main_prefix+main
    # Conflicts are scanned across the full binary receipt, including filtered
    # siblings. Exercise both orders, replacement receipts and equivalent aliases.
    base_fixture=copy.deepcopy(n['fixture'])
    conflict_blobs=''
    conflict_checks=''
    for kind in ('color', 'timing', 'other-color'):
        for reverse in (False, True):
            candidate=copy.deepcopy(base_fixture)
            other=copy.deepcopy(candidate[0])
            if kind=='color':
                other_color=copy.deepcopy(other['ColorModes'][0])
                other_color.update(Depth=10,EOTF=2,Score=999999)
                candidate[0]['ColorModes'].insert(0 if reverse else 1,other_color)
            elif kind=='other-color':
                other.update(ID=79,DSCRequiredColorElementIDs=[80])
                other['ColorModes'][0].update(Depth=10,EOTF=2)
                candidate.insert(0 if reverse else 1,other)
            else:
                other['HorizontalAttributes'].update(Total=2200,FrontPorch=88,SyncWidth=44)
                other['VerticalAttributes'].update(Total=1125,FrontPorch=4,SyncWidth=5,PreciseSyncRate=60<<16)
                other.update(DSCRequiredColorElementIDs=[80])
                candidate.insert(0 if reverse else 1,other)
            raw_blob=bytearray(struct.pack('<I',0xd3));n['namespace']['wire'](candidate,raw_blob)
            label=f'conflict_{kind.replace("-","_")}_{int(reverse)}'
            conflict_blobs+=f'static const u8 {label}[] = {{{",".join(map(str,raw_blob))}}};\n'
            conflict_checks+=f'conflict({label},sizeof({label}),false);conflict({label},sizeof({label}),true);\n'
    identical=copy.deepcopy(base_fixture)
    sibling=copy.deepcopy(identical[0]['ColorModes'][0]);sibling['Score']=123
    identical[0]['ColorModes'].append(sibling)
    raw_blob=bytearray(struct.pack('<I',0xd3));n['namespace']['wire'](identical,raw_blob)
    conflict_blobs+='static const u8 equivalent_ids[] = {'+','.join(map(str,raw_blob))+'};\n'
    conflict_helper=r"""
static void receipt_blob(const u8 *blob,size_t len) {
    u32 size=len;assert(dcpep_cb_prop_start(&d,&size));memcpy(d.chunks.data,blob,size);
    d.chunks.received=size;d.chunks.contiguous=true;
    struct dcp_set_dcpav_prop_end_req request={.key="TimingElements"};
    assert(dcpep_process_chunks(&d,&request));free(d.chunks.data);d.chunks.data=NULL;
}
static void conflicting_catalog_arrived(void) {
    struct dcp_set_dcpav_prop_end_req request={.key="TimingElements"};
    assert(dcpep_process_chunks(&d,&request));free(d.chunks.data);d.chunks.data=NULL;
}
static void conflict(const u8 *blob,size_t len,bool late) {
    reset();callbacks(false);callbacks(true);admit(0);
    struct dcp_display_mode *old=NULL;struct drm_display_mode saved={};
    if(late) { timings();probe(true);old=d.modes;saved=old[0].mode; }
    unsigned before=con.hotplug_wq.scheduled;
    receipt_blob(blob,len);probe(false);
    if(late) { assert(con.hotplug_wq.scheduled>before);assert(!lookup_mode(&d,&saved));free(old); }
    cleanup();
    if(late) {
        reset();callbacks(false);callbacks(true);admit(0);timings();probe(true);old=d.modes;saved=old[0].mode;
        u32 size=len;assert(dcpep_cb_prop_start(&d,&size));memcpy(d.chunks.data,blob,size);
        d.chunks.received=size;d.chunks.contiguous=true;
        before=con.hotplug_wq.scheduled;
        replace_catalog_during_duplicate=true;probe(false);
        assert(con.hotplug_wq.scheduled>before);
        assert(!current(&d,old) && !lookup_mode(&d,&saved));free(old);cleanup();
    }
}
static void conflicting_receipts(void) {
    CONFLICT_CHECKS
    reset();callbacks(false);callbacks(true);admit(0);receipt_blob(equivalent_ids,sizeof(equivalent_ids));probe(true);cleanup();
    puts("PASS: conflicting color/timing IDs, discarded siblings, both receipt orders and late conflicts deny getter/lookup; equivalent color aliases remain eligible");
}
""".replace('CONFLICT_CHECKS',conflict_checks)
    unit=unit.replace('int main(int argc,char **argv) {',conflict_blobs+conflict_helper+'int main(int argc,char **argv) {')
    unit=unit.replace('    permutations();','    permutations();conflicting_receipts();')
    # Execute the production metadata notifier so a manual probe cannot hide a
    # missing connector refresh after the last eligible mode is withdrawn.
    unit=unit.replace('bool lock, attributes_valid, timing_valid, hpd_seen; struct dcp_metadata_epoch epoch;',
        'bool lock, attributes_valid, timing_valid, hpd_seen; struct dcp_metadata_ticket attributes_ticket,timing_ticket; struct drm_display_mode preferred; struct dcp_metadata_epoch epoch;')
    dcp_source=(base/'dcp.c').read_text()
    metadata_notifier='\n'.join(block(dcp_source,marker) for marker in (
        'static void dcp_firmware_metadata_notify(',
        'static bool dcp_firmware_metadata_complete(',
        'void dcp_firmware_metadata_timing('))
    unit=unit.replace(block(unit,'static void dcp_firmware_metadata_timing('),
        '#define dcp_trace_metadata(d,p,v) ((void)0)\nstatic bool dcp_firmware_metadata_enabled(struct apple_dcp *d);\n'+metadata_notifier)

    failed_receipts = r'''
static bool malformed_receipt(const char *key,unsigned fault) {
    u32 size=fault==1 ? 3 : fault==2 ? 4 : sizeof(input_0);
    assert(dcpep_cb_prop_start(&d,&size));
    struct dcp_set_dcpav_prop_chunk_req chunk={.length=size};
    assert(size<=sizeof(chunk.data));memcpy(chunk.data,input_0,size);
    if(fault==0)chunk.data[0]^=1;
    assert(dcpep_cb_prop_chunk(&d,&chunk));
    struct dcp_set_dcpav_prop_end_req request={};
    strcpy(request.key,key);
    bool accepted=dcpep_cb_prop_end(&d,&request);
    assert(!d.chunks.data && !d.chunks.length);
    return accepted;
}
static void failed_replacements(void) {
    for(unsigned fault=0;fault<3;fault++) {
        reset();callbacks(false);callbacks(true);timings();admit(0);probe(true);
        struct dcp_display_mode *old=d.modes;
        struct drm_display_mode saved=old[0].mode;
        unsigned before=con.hotplug_wq.scheduled;
        assert(!malformed_receipt("TimingElements",fault));
        assert(!current(&d,old) && !lookup_mode(&d,&saved));
        assert(con.hotplug_wq.scheduled>before);probe(false);
        /* Fresh catalog can recover using still-current EDID and link proof. */
        timings();probe(true);free(old);cleanup();
    }
    reset();callbacks(false);callbacks(true);timings();admit(0);probe(true);
    assert(!malformed_receipt("DisplayAttributes",0));probe(true);cleanup();
    puts("PASS: actual chunk/end callbacks withdraw malformed replacement exceptions and notify DRM; fresh catalogs recover; unrelated bad properties preserve admission");
}
'''
    unit=unit.replace('int main(int argc,char **argv) {',failed_receipts+'int main(int argc,char **argv) {')
    unit=unit.replace('    permutations();conflicting_receipts();','    permutations();conflicting_receipts();failed_replacements();')
    # Catalogue ownership: lookup_mode() copies the matched mode out and only
    # reports whether one matched; the fixtures only ask whether one did.
    if re.search(r'^bool lookup_mode\(', iomfb, re.M):
        unit=re.sub(r'\blookup_mode\((&d,&[\w.\[\]]+)\)', r'lookup_mode(\1,NULL)', unit)
    return {**locals(), "block": block}


def panel_dimensions(root):
    n = publication(root)
    unit, block = n['unit'], n['block']
    unit = unit.replace('bool modes_lock,modes_admitted,',
                        'bool external_physically_absent,modes_lock,modes_admitted,', 1)
    dcp = (root / 'drivers/gpu/drm/apple/dcp.c').read_text()
    unit = unit.replace(block(unit, 'static bool __attribute__((unused)) dcp_has_panel('),
                        block(dcp, 'bool dcp_has_panel('))
    unit = unit.replace(block(unit, 'static void dcp_update_connector_dimensions('),
                        block(dcp, 'void dcp_update_connector_dimensions('))
    # Ordinary panel modes do not take the 180 Hz exception's metadata lock.
    unit = unit.replace('probing->dcpavserv.lock && probing->metadata.lock',
                        'probing->dcpavserv.lock && '
                        '(!probing->modes[0].automatic_rgb8_quirk || probing->metadata.lock)')
    # Model DRM's display_info reset and the absence of an internal AV EDID.
    unit = unit.replace(block(unit, 'static int drm_edid_connector_update('), r"""
static int drm_edid_connector_update(struct drm_connector *c,const struct drm_edid *e) {
    assert(c->dev->mode_config.mutex);
    c->display_info.width_mm=e ? 600 : 0;
    c->display_info.height_mm=e ? 330 : 0;
    return fail_update ? -EIO : 0;
}
""")
    unit = unit.replace('if(!d->dcpavserv.enabled || !d->dcpavserv.service',
                        'if(dcp_has_panel(d)) return ERR_PTR(-ENODEV);\n'
                        '    if(!d->dcpavserv.enabled || !d->dcpavserv.service')
    checks = r"""
static void panel_dimension_probes(void) {
    reset();callbacks(false);callbacks(true);timings();admit(0);probe(true);
    assert(con.base.display_info.width_mm==600 && con.base.display_info.height_mm==330);
    d.panel.width_mm=302;d.panel.height_mm=196;d.width_mm=0;d.height_mm=0;
    d.modes[0].automatic_rgb8_quirk=false;
    con.edid_invalidation_pending=true;probe(true);
    assert(!con.drm_edid);
    assert(con.base.display_info.width_mm==302 && con.base.display_info.height_mm==196);
    con.edid_invalidation_pending=true;probe(true);
    assert(con.base.display_info.width_mm==302 && con.base.display_info.height_mm==196);
    d.width_mm=300;d.height_mm=190;con.edid_invalidation_pending=true;probe(true);
    assert(con.base.display_info.width_mm==300 && con.base.display_info.height_mm==190);
    d.width_mm=0;d.height_mm=190;con.edid_invalidation_pending=true;probe(true);
    assert(con.base.display_info.width_mm==302 && con.base.display_info.height_mm==196);
    cleanup();
    reset();callbacks(false);callbacks(true);timings();admit(0);probe(true);
    assert(con.base.display_info.width_mm==600 && con.base.display_info.height_mm==330);
    cleanup();
    puts("PASS: internal panel dimensions survive repeated EDID clears; firmware size and DT fallback; external EDID size preserved");
}
"""
    unit = unit.replace('int main(int argc,char **argv) {',
                        checks + 'int main(int argc,char **argv) {')
    return unit.replace('    transport_edid=raw;',
                        '    transport_edid=raw;panel_dimension_probes();', 1)

def hotplug(root):
    n = publication(root)
    unit = n['unit']
    block = n['block']
    base = root / 'drivers/gpu/drm/apple'
    dptx = (base / 'dptxep.c').read_text()
    training = (base / 'dptx-j700-training.h').read_text()
    unit = unit.replace('struct dptx_port { u32 lane_count, link_rate;', '''
union phy_configure_opts { struct { unsigned lanes,link_rate; bool set_lanes,set_rate,set_voltages; } dp; };
struct dptx_port { union phy_configure_opts phy_ops; int linkcfg_completion; u32 pending_link_rate, unit;
u32 lane_count, link_rate;''')
    unit = unit.replace('int fw_compat,width_mm,height_mm,fixed_connector_type;', 'unsigned dptx_phy; int fw_compat,width_mm,height_mm,fixed_connector_type;')
    real_handlers = r'''
#define cpu_to_le32(v) (v)
#define cpu_to_le64(v) (v)
#define LINK_RATE_RBR 0x06
#define LINK_RATE_HBR 0x0a
#define LINK_RATE_HBR2 0x14
#define PHY_MODE_DP 1
#define trace_dptxport_call_set_link_rate(...) ((void)0)
static unsigned phy_capacity=2, phy_calls, hardware_rate, hardware_lanes;
static int validate_error;
static int phy_validate(void *phy,int mode,int submode,union phy_configure_opts *o) {
    assert(phy);(void)mode;(void)submode;
    if(validate_error)return validate_error;
    o->dp.lanes=phy_capacity;return 0;
}
static int phy_configure(void *phy,union phy_configure_opts *o) {
    assert(phy);phy_calls++;
    if(configure_error)return configure_error;
    if(o->dp.set_lanes)hardware_lanes=o->dp.lanes;
    if(o->dp.set_rate)hardware_rate=o->dp.link_rate;
    return 0;
}
/* Varargs: the DPTX unit argument came with the T6030 tunnel clocks. */
static int dcp_tunnel_set_rate(struct apple_dcp *d,void *phy,...) {
    (void)d;(void)phy;assert(false);return 0;
}
/* The T6030 14.7 backend never matches this H17P display. */
#define DCP_FIRMWARE_V_14_7 (DCP_FIRMWARE_H17P + 1)
#define DCP_FIRMWARE_V_14_7_TEMPLATE (DCP_FIRMWARE_H17P + 2)
'''
    real_handlers += block(training, 'static int dptx_j700_validate_lanes(') + '\n'
    real_handlers += block(dptx, 'static int dptxport_call_set_active_lane_count(') + '\n'
    real_handlers += block(dptx, 'static int dptxport_call_set_link_rate(') + '\n'
    for marker in ['static int dptxport_call_set_active_lane_count(', 'static int dptxport_call_set_link_rate(']:
        unit = unit.replace(block(unit, marker), '')
    unit = unit.replace('static int configure_error;', 'static int configure_error;\n' + real_handlers)
    unit = unit.replace('published=reads=pending_probes=0;', 'phy_capacity=2;phy_calls=hardware_rate=hardware_lanes=0;validate_error=0;published=reads=pending_probes=0;')
    unit = unit.replace('assert(current(&d,&d.modes[0]));cleanup();runs++;', 'assert(current(&d,&d.modes[0]));assert(hardware_rate==8100 && hardware_lanes==2 && phy_calls==2);cleanup();runs++;')
    unit = unit.replace('struct apple_dcp {', '''struct apple_dcp {
    struct { bool lock,presentation_paused; } iomfb;
    struct { bool valid,changing; } mode_state;
    struct work_struct external_stop_work,vblank_wq;
    bool external_physically_absent,main_display,typec_crtc_off,external_cable,external_error,external_power_attempted,external_stopped;
    bool external_hotplug_pending,external_hotplug_connected,typec_poweron_hpd;
    u64 external_stop_generation,external_rx_generation,external_crtc_generation;
    unsigned swap_watchdog_retrains;int typec_iomfb_hpd_ready;''')
    stubs = r'''
#define DCP_HOTPLUG_NOTIFY 1
static bool dcp_is_external_h17p(struct apple_dcp *d) { return d->hw.iomfb_method_profile==2; }
static bool dcp_is_typec_output(struct apple_dcp *d) { (void)d;return true; }
static void complete_all(int *p) { (*p)++; }
#define dcp_mode_invalidate(s) ((s)->valid=false)
#define dcp_mode_hotplug(s,c,p) __extension__ ({ (void)(s);if(p)*(p)=(c);1u; })
static void dcp_handle_hotplug_actions(struct apple_dcp *d,unsigned action) { (void)d;(void)action; }
static bool dcp_typec_relink(struct apple_dcp *d) { (void)d;return false; }
'''
    dcp = (root / 'drivers/gpu/drm/apple/dcp.c').read_text()
    template = (root / 'drivers/gpu/drm/apple/iomfb_template.c').read_text()
    functions = stubs + block(dcp, 'bool dcp_external_hotplug(') + '\n' + block(template, 'static void dcpep_cb_hotplug(')

    main=r"""int main(int argc,char **argv) { MAIN_PREFIX
    transport_edid=raw;permutations();conflicting_receipts();failed_replacements();
    unsigned runs=0;
    for(unsigned a=0;a<4;a++)for(unsigned b=0;b<4;b++)for(unsigned c=0;c<4;c++)for(unsigned e=0;e<4;e++) {
        if(a==b||a==c||a==e||b==c||b==e||c==e)continue;
        reset();callbacks(false);callbacks(true);timings();admit(0);probe(true);
        d.external_cable=true;d.external_power_attempted=true;d.external_rx_generation=7;
        d.iomfb.lock=true;u64 connected=1;dcpep_cb_hotplug(&d,&connected);d.iomfb.lock=false;
        struct dcp_display_mode *old=d.modes;struct drm_display_mode saved=old[0].mode;
        u64 sequence=d.rate_proof.sequence,connection=d.rate_proof.connection_sequence;
        d.iomfb.lock=true;connected=0;dcpep_cb_hotplug(&d,&connected);d.iomfb.lock=false;
        assert(d.external_stop_work.scheduled==1 && d.external_stop_generation==7);
        assert(sequence!=d.rate_proof.sequence && connection!=d.rate_proof.connection_sequence);
        assert(!d.rate_proof.rate.valid && !d.rate_proof.lanes.valid);
        assert(!current(&d,old) && !lookup_mode(&d,&saved));
        assert(con.edid_invalidation_pending);probe(false); /* Work deliberately not executed. */
        dpavservep_detach(&d);dcp_180hz_invalidate(&d); /* Deferred retirement/release. */
        unsigned order[]={a,b,c,e};
        for(unsigned i=0;i<4;i++) {
            switch(order[i]) { case 0:callbacks(false);break;case 1:callbacks(true);break;case 2:timings();break;case 3:admit(1);break; }
            probe(false);assert(!current(&d,old));
        }
        assert(d.rate_proof.rate.valid && d.rate_proof.lanes.valid);
        /* Repeated low reports before reassertion are not a new HPD loss. */
        sequence=d.rate_proof.sequence;d.iomfb.lock=true;connected=0;
        dcpep_cb_hotplug(&d,&connected);d.iomfb.lock=false;
        assert(d.rate_proof.sequence==sequence && d.rate_proof.rate.valid && d.rate_proof.lanes.valid);
        d.iomfb.lock=true;connected=1;dcpep_cb_hotplug(&d,&connected);d.iomfb.lock=false;
        assert(d.metadata.epoch.connected && d.metadata.epoch.invalidated);
        probe(true);assert(!current(&d,old));assert(phy_calls==4 && hardware_rate==8100 && hardware_lanes==2);
        free(old);cleanup();runs++;
    }
    assert(runs==24);
    /* Initial bootstrap false and power/paused suppression keep proof intact. */
    reset();callbacks(false);callbacks(true);timings();admit(0);probe(true);
    d.iomfb.lock=true;u64 connected=0;u64 sequence=d.rate_proof.sequence;
    dcpep_cb_hotplug(&d,&connected);assert(d.rate_proof.sequence==sequence && !con.edid_invalidation_pending);
    connected=1;dcpep_cb_hotplug(&d,&connected);
    connected=0;d.typec_crtc_off=true;dcpep_cb_hotplug(&d,&connected);assert(d.rate_proof.sequence==sequence);
    d.typec_crtc_off=false;d.iomfb.presentation_paused=true;dcpep_cb_hotplug(&d,&connected);
    assert(d.rate_proof.sequence==sequence && !con.edid_invalidation_pending);d.iomfb.lock=false;probe(true);
    dcpavserv_teardown(&av_services[0]);cleanup();
    puts("PASS: actual RX HPD loss immediately denies getter/lookup before deferred work; 24 fresh-proof recovery orders need only one rate/lane callback; bootstrap/power/paused suppression preserved");
}""".replace('MAIN_PREFIX',n['main_prefix'])
    unit=unit.replace(block(unit,'int main(int argc,char **argv) {'),functions+'\n'+main)
    if re.search(r'^bool lookup_mode\(', (base / 'iomfb.c').read_text(), re.M):
        unit=re.sub(r'\blookup_mode\((&d,&[\w.\[\]]+)\)', r'lookup_mode(\1,NULL)', unit)
    return unit


def metadata(root, output):
    """Keep malformed/duplicate metadata cases with synthetic bounded wire input."""
    n = parser_fixture(root)
    fixture = n['fixture']
    cases = [(fixture, False)]
    for path, value in [
        (('MaxHorizontalImageSize',), 256), (('MaxVerticalImageSize',), '33'),
        (('Chromaticity', 'Red', 'X'), True), (('Chromaticity', 'Blue', 'Y'), 65536),
        (('Chromaticity', 'Green', 'Y'), 64000), (('DefaultWhitePoint', 'Gamma'), 65535),
        (('DefaultWhitePoint', 'Gamma'), 240000), (('ProductAttributes', 'ManufacturerID'), 'AC'),
        (('ProductAttributes', 'ManufacturerID'), 'A\0R'), (('ProductAttributes', 'LegacyManufacturerID'), 1),
        (('ProductAttributes', 'ProductName'), 'x' * 14), (('ProductAttributes', 'ProductID'), 65536),
        (('ProductAttributes', 'YearOfManufacture'), 1989), (('ProductAttributes', 'WeekOfManufacture'), 54),
    ]:
        bad = copy.deepcopy(fixture)
        target = bad
        for key in path[:-1]:
            target = target[key]
        target[path[-1]] = value
        cases.append((bad, True))
    for key in ('Chromaticity', 'DefaultWhitePoint', 'ProductAttributes'):
        bad = copy.deepcopy(fixture)
        del bad[key]
        cases.append((bad, True))
        second = copy.deepcopy(fixture[key])
        cases.append((n['ObjectPairs'](list(fixture.items()) + [(key, second)]), True))
    for path, field in [(('Chromaticity', 'Red'), 'X'), (('ProductAttributes',), 'ProductName')]:
        bad = copy.deepcopy(fixture)
        target = bad
        for key in path[:-1]:
            target = target[key]
        value = target[path[-1]]
        target[path[-1]] = n['ObjectPairs'](list(value.items()) + [(field, value[field])])
        cases.append((bad, True))
    inputs = output / 'metadata-inputs'
    inputs.mkdir(parents=True, exist_ok=True)
    arguments = []
    for index, (value, bad) in enumerate(cases):
        data = bytearray(struct.pack('<I', 0xd3))
        n['wire'](value, data)
        path = inputs / str(index)
        path.write_bytes(data)
        arguments.append((str(path), str(int(bad)), str(inputs / 'edid')))
    unit = ''.join(n[k] for k in ('prefix', 'metadata', 'epoch', 'header', 'edid_header',
                                  'stubs', 'parser', 'builder', 'timing_source', 'proof_test', 'suffix'))
    return unit, tuple(arguments)


def check_metadata(output):
    raw = (output / 'metadata-inputs/edid').read_bytes()
    assert len(raw) == 128 and sum(raw) % 256 == 0 and raw[126] == 0
    assert raw[:8] == bytes.fromhex('00ffffffffffff00')
    assert raw[8:10] == struct.pack('>H', 1138)
    assert raw[21:24] == bytes([60, 33, 120])
    assert raw[38:54] == b'\x01' * 16 and raw[35:38] == b'\0' * 3
    assert struct.unpack_from('<H', raw, 54)[0] == 14850
    assert raw[56] + ((raw[58] >> 4) << 8) == 1920
    assert raw[57] + ((raw[58] & 15) << 8) == 280
    assert raw[59] + ((raw[61] >> 4) << 8) == 1080
    assert raw[60] + ((raw[61] & 15) << 8) == 45
    assert raw[62:65] == bytes([88, 44, 0x45]) and raw[71] == 0x1e
    chroma = [raw[27] * 4 + ((raw[25] >> 6) & 3), raw[28] * 4 + ((raw[25] >> 4) & 3),
              raw[29] * 4 + ((raw[25] >> 2) & 3), raw[30] * 4 + (raw[25] & 3),
              raw[31] * 4 + ((raw[26] >> 6) & 3), raw[32] * 4 + ((raw[26] >> 4) & 3),
              raw[33] * 4 + ((raw[26] >> 2) & 3), raw[34] * 4 + (raw[26] & 3)]
    assert chroma == [697, 317, 265, 632, 146, 81, 321, 337]
    assert raw[72:77] == bytes.fromhex('000000fc00')


def retry(root):
    source = (root / 'drivers/gpu/drm/apple/dcp-180hz.c').read_text()
    production = '\n'.join(block(source, marker) for marker in (
        'static void dcp_180hz_retry_work(', 'void dcp_180hz_retry(', 'static void dcp_180hz_cancel('))
    prefix = r"""
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
typedef uint64_t u64;
#define READ_ONCE(x) (x)
#define WRITE_ONCE(p,v) ((p)=(v))
#define guard(kind) (void)
#define spin_lock_irqsave(p,f) do { assert(!*(p)); *(p)=true; (f)=0; } while (0)
#define spin_unlock_irqrestore(p,f) do { (void)(f); assert(*(p)); *(p)=false; } while (0)
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define to_delayed_work(p) container_of(p,struct delayed_work,work)
#define to_platform_device(p) (p)
#define msecs_to_jiffies(x) (x)
struct work_struct { unsigned scheduled; };
struct delayed_work { struct work_struct work; unsigned delay; bool pending; };
struct apple_connector { void *dcp; struct work_struct hotplug_wq; };
struct apple_dcp {
    bool hpd_mutex, typec_cable_connected, quiescing, rate_retry_initialized;
    struct { bool lock; } metadata;
    struct { u64 retry_generation; unsigned retries; } rate_proof;
    u64 typec_generation;
    struct apple_connector *connector;
    void *dev;
    struct delayed_work rate_retry_wq;
};
static int system_freezable_wq;
static unsigned submissions, cancellations;
static void schedule_work(struct work_struct *w) { w->scheduled++; }
static void mod_delayed_work(int q,struct delayed_work *w,unsigned delay)
{ (void)q; submissions++; w->pending=true; w->delay=delay; }
static void cancel_delayed_work_sync(struct delayed_work *w)
{ cancellations++; w->pending=false; }
"""
    controls = r"""
int main(void)
{
    struct apple_dcp d={.typec_cable_connected=true,.typec_generation=7};
    struct apple_connector c={.dcp=&d}; d.connector=&c; d.dev=&d;
    dcp_180hz_retry(&d); assert(!submissions);
    d.rate_retry_initialized=true;
    for(unsigned i=0;i<100;i++) dcp_180hz_retry(&d);
    assert(submissions==8 && d.rate_proof.retries==8 && d.rate_retry_wq.delay==300);
    dcp_180hz_retry_work(&d.rate_retry_wq.work); assert(c.hotplug_wq.scheduled==1);
    d.typec_generation++; dcp_180hz_retry_work(&d.rate_retry_wq.work);
    assert(c.hotplug_wq.scheduled==1);
    d.typec_cable_connected=false; dcp_180hz_retry(&d); assert(submissions==8);
    d.typec_cable_connected=true; dcp_180hz_retry(&d); assert(submissions==9 && d.rate_proof.retries==1);
    d.quiescing=true; dcp_180hz_retry(&d); dcp_180hz_retry_work(&d.rate_retry_wq.work);
    assert(submissions==9 && c.hotplug_wq.scheduled==1);
    d.quiescing=false; c.dcp=NULL; dcp_180hz_retry_work(&d.rate_retry_wq.work);
    assert(c.hotplug_wq.scheduled==1);
    c.dcp=&d; dcp_180hz_retry_work(&d.rate_retry_wq.work); assert(c.hotplug_wq.scheduled==2);
    dcp_180hz_cancel(&d); assert(!d.rate_retry_initialized && !d.rate_retry_wq.pending && cancellations==1);
    dcp_180hz_retry(&d); assert(submissions==9);
    puts("PASS: actual eight-retry bound, 300ms delay, stale-generation/disconnect/quiesce/ownership gates and cancellation");
}
"""
    return prefix + production + controls
