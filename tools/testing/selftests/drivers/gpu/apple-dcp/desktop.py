# SPDX-License-Identifier: GPL-2.0-only OR MIT
# Copyright 2026 Ryan Murray
"""Desktop modeset callbacks through the production dispatcher and DMA policy."""
from common import block
import lifecycle


def modeset_unmap(root):
    base = root / 'drivers/gpu/drm/apple'
    iomfb = (base / 'iomfb.c').read_text()
    template = (base / 'iomfb_template.c').read_text()
    desktop = (base / 'iomfb_v26_6_desktop.c').read_text()
    header = (base / 'iomfb_v26_6_desktop.h').read_text()
    # Reuse the transport boundary mocks, retaining the real callback dispatcher.
    prefix = lifecycle.dispatcher(root).split('static bool dcp_external_zero_reply(')[0]
    prefix = prefix.replace('struct dcp_channel { unsigned depth; void *output[8]; unsigned end[8]; };', r'''
struct apple_dcp;
typedef void (*dcp_callback_t)(struct apple_dcp *, void *, void *);
struct dcp_channel {
    unsigned depth; void *output[8], *cookies[8]; unsigned end[8];
    dcp_callback_t callbacks[8]; struct dcp_packet_header header[8];
};
struct sg_table {void *sgl;};
struct dcp_mem_descriptor {void *buf;u64 dva,reg;size_t size;bool piodma_mapped;struct sg_table map;};
''')
    prefix = prefix.replace('    struct dcp_channel channel;', r'''
    struct dcp_channel channel, command;
    struct dcp_mem_descriptor memdesc[4];
    unsigned long memdesc_map[1];void *iommu_dom;
    struct resource {u64 start,end;} *disp_registers[1];int nr_disp_registers;
''')
    prefix = prefix.replace('DCP_CONTEXT_CB = 0, DCP_CONTEXT_ASYNC = 3',
                            'DCP_CONTEXT_CB = 0, DCP_CONTEXT_CMD = 1, DCP_CONTEXT_ASYNC = 3')
    prefix = prefix.replace('(void)c; return &d->channel;',
                            'return c==DCP_CONTEXT_CMD?&d->command:&d->channel;')
    prefix = prefix.replace('((void)dev)', '((void)dcp->dev)')
    prefix += r'''
#include <sys/types.h>
typedef u64 __le64;typedef u32 __le32;
#define __packed __attribute__((packed))
typedef u64 dma_addr_t;
#define DCP_FW_NAME(n) n
#define DCP_FW_H17(v) 0
#define DCP_FW_DESKTOP_26(v) 1
#define SZ_16K 16384U
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define DCP_MAX_MAPPINGS 4
#undef static_assert
#define static_assert(c) _Static_assert(c,#c)
#define trace_iomfb_callback(...) ((void)0)
#define dcp_trace_panel_property(...) ((void)0)
#define GFP_KERNEL 0
#define kmalloc_objs(t,n,f) malloc(sizeof(t)*(n))
#define dev_warn_dma(...) ((void)0)
struct dcp_map_physical_req {u64 paddr,size;u32 flags;u8 dva_null,dva_size_null,padding[2];} __attribute__((packed));
struct dcp_map_physical_resp {u64 dva,dva_size;u32 mem_desc_id;} __attribute__((packed));
struct dcp_map_buf_req {u64 buffer;};
struct dcp_map_buf_resp {u64 buffer,dva;};
struct dcp_unmap_buf_resp {u64 buffer,vaddr,dva;u8 unk,buf_null;};
struct dcp_wait_cookie {unsigned status;bool replied;int done,refcount;};
#define dev_warn_release(...) ((void)0)
#define dev_err_release(...) ((void)0)
#define DMA_BIDIRECTIONAL 0
#define round_down(x,y) ((x) & ~((__typeof__(x))(y)-1))
#define check_add_overflow(a,b,p) __builtin_add_overflow(a,b,p)
static unsigned physical_maps;
static unsigned find_first_zero_bit(unsigned long *map,unsigned count)
{for(unsigned i=0;i<count;i++)if(!(*map & (1UL<<i)))return i;return count;}
static void set_bit(unsigned b,unsigned long *p) {*p |= 1UL<<b;}
static u64 dma_map_resource(struct device *dev,u64 pa,u64 size,int dir,int flags)
{assert(dev && pa==0x123400 && size==4096 && !dir && !flags);physical_maps++;return 0x8000;}
static bool dma_mapping_error(struct device *dev,u64 dva) {assert(dev && dva==0x8000);return false;}
static unsigned frees,resource_unmaps;
static void sg_free_table(struct sg_table *map) {assert(map->sgl);}
static void dma_free_coherent(struct device *dev,size_t size,void *buf,u64 dva)
{assert(dev && size==SZ_16K && buf && dva==0x8000);frees++;}
static void dma_unmap_resource(struct device *dev,u64 dva,size_t size,int dir,int flags)
{assert(dev && dva==0x8000 && size==4096 && !dir && !flags);resource_unmaps++;}
static void clear_bit(unsigned b,unsigned long *p) {*p &= ~(1UL<<b);}
static u32 get_unaligned_le32(const void *p) {u32 v;memcpy(&v,p,4);return v;}
static void put_unaligned_le32(u32 v,void *p) {memcpy(p,&v,4);}
static bool test_bit(unsigned b,unsigned long *p) {return !!(*p & (1UL<<b));}
static u64 get_unaligned_le64(const void *p) {u64 v;memcpy(&v,p,8);return v;}
static u8 dcp_pop_depth(unsigned *p) {assert(*p);return --(*p);}
static void release_wait_cookie(int *p) {assert(false);}
static void kref_put(int *p,void (*release)(int *)) {assert(*p==2);--*p;(void)release;}
#define IOMMU_READ 1
#define IOMMU_WRITE 2
static unsigned maps,unmaps;
static ssize_t iommu_map_sgtable(void *domain,u64 dva,struct sg_table *map,int flags)
{assert(domain && dva==0x8000 && map->sgl && flags==3);maps++;return SZ_16K;}
static size_t iommu_unmap(void *domain,u64 dva,size_t size)
{assert(domain && dva==0x8000 && size==SZ_16K);unmaps++;return size;}
'''
    # dev logging in the DMA helper has no local 'dev', unlike the dispatcher.
    unmap = block(template, 'static bool dcpep_unmap_piodma(')
    unmap = unmap.replace('dev_warn(', 'dev_warn_dma(').replace('dev_err(', 'dev_warn_dma(')
    handlers = block(desktop, 'static const iomfb_cb_handler cb_handlers[')
    binding = '\n'.join(line for line in handlers.splitlines() if '[202]' in line or '[454]' in line)
    mapping = block(template, 'static bool dcpep_map_piodma(').replace('dev_err(', 'dev_warn_dma(')
    release = block(template, 'static u8 dcpep_cb_release_mem_desc(').replace('dev_warn(', 'dev_warn_release(').replace('dev_err(', 'dev_err_release(')
    physical = block(template, 'static bool is_disp_register(') + '\n' + block(template, 'static struct dcp_map_physical_resp\ndcpep_cb_map_physical(').replace('dev_err(', 'dev_err_release(').replace('dev_warn(', 'dev_warn_release(')
    physical += r"""
static bool trampoline_map_physical(struct apple_dcp *dcp,int tag,void *out,void *in)
{struct dcp_map_physical_resp r=dcpep_cb_map_physical(dcp,in);(void)tag;memcpy(out,&r,sizeof(r));return true;}
"""
    # Assert the table still binds the shared production mapper.
    assert '[452] = trampoline_map_physical,' in handlers
    binding += '\n[452] = trampoline_map_physical,'
    wire = block(desktop, 'struct dcp_unmap_buf_req_v26_6 {')+';\n'
    production = (wire + '\n' + physical + '\n' + release + '\n' + block(desktop, 'static bool trampoline_release_mem_desc_v26_6(') + '\n' + mapping + '\n' + unmap + '\n' + block(desktop, 'static bool trampoline_unmap_piodma_v26_6(') +
                  '\n' + block(iomfb, 'static bool dcp_external_zero_reply(') +
                  '\n' + block(iomfb, 'static void dcpep_handle_cb(').replace('struct device *dev =', 'struct device *dev __attribute__((unused)) =') +
                  '\n' + block(iomfb, 'static void dcpep_handle_ack(') +
                  '\n' + block(template, 'static void complete_set_digital_out_mode('))
    return prefix + production + '\n' + r'''
int main(void)
{
    struct device_node node={0};struct device dev={&node};
    bool (*handlers[IOMFB_MAX_CB])(struct apple_dcp *,int,void *,void *)={
''' + binding + r'''
    };
    struct apple_dcp d={.dev=&dev,.fw_compat=DCP_FIRMWARE_V_26_6_DESKTOP,
        .cb_handlers=handlers,.memdesc_map={2},.iommu_dom=&d};
    union {u64 align;u8 data[sizeof(struct dcp_packet_header)+24];} packet;
    struct dcp_packet_header *h=(void *)packet.data;
    struct {struct dcp_packet_header h;u32 input[2];u32 status;} command={
        .h={.in_len=8,.out_len=4},.status=0};
    struct dcp_wait_cookie wait={.status=~0U,.refcount=2};
    /* The firmware pauses an A413 modeset until its D202 is acknowledged. */
    __typeof__(*d.desktop) desktop={0};d.desktop=&desktop;
    d.command.depth=1;d.command.header[0]=command.h;
    d.command.callbacks[0]=complete_set_digital_out_mode;
    d.command.cookies[0]=&wait;d.command.output[0]=&command.status;
    d.memdesc[1]=(struct dcp_mem_descriptor){.buf=&d,.dva=0x8000,.size=4096,.map={.sgl=&d}};
    struct dcp_map_buf_req map_request={.buffer=1};struct dcp_map_buf_resp map_reply;
    assert(dcpep_map_piodma(&d,&map_request,&map_reply));
    assert(maps==1 && map_reply.buffer==1 && map_reply.dva==0x8000);
    memset(&packet,0,sizeof(packet));h->in_len=24;
    struct dcp_unmap_buf_req_v26_6 fields={.buffer=1,.id=1,.dva=0x8000};memcpy(packet.data+sizeof(*h),&fields,24);
    case_tag=202;acks=0;
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,h,sizeof(*h)+24,0);
    assert(acks==1 && !d.crashed && !d.channel.depth && unmaps==1);
    assert(!d.memdesc[1].piodma_mapped && d.command.depth==1 && !wait.done);
    /* No firmware response can arrive until the empty callback ACK above. */
    dcpep_handle_ack(&d,DCP_CONTEXT_CMD);
    assert(wait.done && wait.replied && !wait.status && wait.refcount==1 && !d.command.depth);
    /* A later mode can reuse a descriptor after the firmware unmaps it. */
    assert(dcpep_map_piodma(&d,&map_request,&map_reply) && maps==2);
    /* Descriptor release handles both RAM and mapped resources, once. */
    fields.buffer=1;fields.dva=0x8000;memcpy(packet.data+sizeof(*h),&fields,24);
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,h,sizeof(*h)+24,0);
    assert(unmaps==2);
    case_tag=454;h->in_len=4;h->out_len=4;
    put_unaligned_le32(1,packet.data+sizeof(*h));
    memset(packet.data+sizeof(*h)+4,0xa5,4);
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,h,sizeof(*h)+8,0);
    assert(get_unaligned_le32(packet.data+sizeof(*h)+4)==1);
    assert(frees==1 && !d.memdesc_map[0] && !d.memdesc[1].buf && !d.memdesc[1].size);
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,h,sizeof(*h)+8,0);
    assert(!get_unaligned_le32(packet.data+sizeof(*h)+4) && frees==1);
    d.memdesc_map[0]=1;
    struct resource reg={.start=0x123400,.end=0x1243ff};d.disp_registers[0]=&reg;d.nr_disp_registers=1;
    struct dcp_map_physical_req request={.paddr=0x123400,.size=4096};
    struct {struct dcp_packet_header h;struct dcp_map_physical_req in;struct dcp_map_physical_resp out;} __attribute__((packed)) phys={.h={.in_len=24,.out_len=20},.in=request};
    case_tag=452;
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,&phys,sizeof(phys),0);
    assert(physical_maps==1 && phys.out.mem_desc_id==1 && phys.out.dva==0x8000 && phys.out.dva_size==4096);
    assert(d.memdesc_map[0]==3 && d.memdesc[1].reg==0x123400);
    case_tag=454;put_unaligned_le32(1,packet.data+sizeof(*h));
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,h,sizeof(*h)+8,0);
    assert(get_unaligned_le32(packet.data+sizeof(*h)+4)==1 && resource_unmaps==1 && d.memdesc_map[0]==1);
    for(u32 id=4;id<8;id++) {
        put_unaligned_le32(id,packet.data+sizeof(*h));
        dcpep_handle_cb(&d,DCP_CONTEXT_CB,h,sizeof(*h)+8,0);
        assert(!get_unaligned_le32(packet.data+sizeof(*h)+4) && frees==1);
    }
    /* Out-of-range and overflowing physical regions receive no mapping. */
    case_tag=452;phys.in.paddr=0x124000;
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,&phys,sizeof(phys),0);
    assert(!phys.out.dva_size && physical_maps==1);
    phys.in.paddr=~0ULL;phys.in.size=8192;
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,&phys,sizeof(phys),0);
    assert(!phys.out.dva_size && physical_maps==1);
    unsigned before=acks;
    assert(!iomfb_check_callback_v26_6_desktop(454,4,1));
    assert(!iomfb_check_callback_v26_6_desktop(452,23,20));
    /* The declared desktop shape is exact, with no guessed reply bytes. */
    assert(!iomfb_check_callback_v26_6_desktop(202,26,0));
    assert(!iomfb_check_callback_v26_6_desktop(202,24,4));
    case_tag=202;h->in_len=23;h->out_len=0;
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,h,sizeof(*h)+24,0);
    assert(d.crashed && acks==before && unmaps==2);
    return 0;
}
'''


def brightness_takeover(root):
    import display
    source = (root / 'drivers/gpu/drm/apple/dcp_backlight.c').read_text()
    prefix = display.backlight(root).split('static void completed(')[0]
    prefix = prefix.replace('static inline ', 'static inline __attribute__((unused)) ')
    functions = '\n'.join(block(source, marker) for marker in (
        'static bool dcp_backlight_desktop(', 'static int dcp_get_brightness(',
        'void dcp_backlight_presented(', 'static void dcp_backlight_sync_hint(',
        'int dcp_backlight_register(', 'static int dcp_set_brightness(',
        'static bool dcp_backlight_controls_device('))
    # The 14.7 panel's nits policy and class operations (an M3 internal
    # panel) never run on the desktop panel.
    if 'static bool dcp_backlight_presents_nits(' in source:
        functions = ('static bool iomfb_v14_7_panel_nits(struct apple_dcp *d) '
                     '{(void)d;return false;}\n'
                     'static const int dcp_backlight_ops_v14;\n' +
                     block(source, 'static bool dcp_backlight_presents_nits(') +
                     '\n' + functions)
    return prefix + r'''
#include <stddef.h>
#include <string.h>
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define DCP_FIRMWARE_V_26_6_DESKTOP 5
#define DCP_FIRMWARE_H17P 1
#define DCP_FIRMWARE_V_27_0 6
#define DCP_IOMFB_METHODS_H17G 7
static bool v27_keep_level = true;
static bool pm_suspend_via_firmware(void) {return false;}
#define MAX_BRIGHTNESS_PART2 510U
#define BACKLIGHT_PLATFORM 1
#define BACKLIGHT_SCALE_LINEAR 1
#define BL_CORE_SUSPENDED 1
#define BL_CORE_FBBLANK 2
#define devm_add_action_or_reset(...) (0)
#define DCP_BL_FADE_STEP_MS 16
#define system_dfl_wq NULL
#define IS_ERR(p) false
#define PTR_ERR(p) (-EINVAL)
#define msecs_to_jiffies(n) (n)
#define spin_lock_irqsave(p,f) do {assert(!*(p));*(p)=true;(f)=0;} while(0)
#define spin_unlock_irqrestore(p,f) do {assert(*(p));*(p)=false;(void)(f);} while(0)
struct device {int unused;};
struct backlight_properties {int type,brightness,max_brightness,scale,state;};
struct backlight_device {struct backlight_properties props;void *data;bool ops_lock;};
struct apple_dcp {
    int fw_compat;bool quiescing;
    struct {int iomfb_method_profile;} hw;
    struct {u32 maximum,nits,dac;bool update;struct backlight_device *bl_dev;
        u32 level,fade_from,fade_to;bool fading;unsigned long fade_start;int fade_work;
        struct dcp_bl_presented presented;} brightness;
    struct {bool lock;struct dcp_backlight_state state;unsigned long fade_start;
        int fade_work;void (*kick)(struct apple_dcp *);} backlight;
    struct {bool inherited_brightness_valid;u32 inherited_brightness;} *desktop;
    struct {struct {struct {struct device *dev;} *dev;int mutex;} base;} *crtc;
    struct device *dev;
};
static unsigned long jiffies;
static bool inject_hint,inject_user;
static void mutex_lock(bool *p)
{
    assert(!*p);*p=true;
    if(inject_user) {
        inject_user=false;
        struct backlight_device *bd=(void *)((char *)p - offsetof(struct backlight_device,ops_lock));
        struct apple_dcp *d=bd->data;
        bd->props.brightness=450;
        assert(!dcp_bl_request(&d->backlight.state,450,false,false));
    }
}
static void mutex_unlock(bool *p) {assert(*p);*p=false;}
static struct backlight_device registered;
static const int dcp_backlight_ops;
static void *bl_get_data(struct backlight_device *bd) {return bd->data;}
static bool iomfb_is_m4(struct apple_dcp *d) {(void)d;return false;}
bool dcp_backlight_active(struct apple_dcp *d) {(void)d;return false;}
static unsigned notify_count;
static void dcp_backlight_notify(struct apple_dcp *d) {(void)d;notify_count++;}
static void dcp_backlight_desktop_kick(struct apple_dcp *d) {(void)d;}
static void dcp_backlight_kick(struct apple_dcp *d) {(void)d;}
static unsigned fades;
static void queue_delayed_work(void *q,int *work,unsigned delay)
{assert(!q && delay==16);(void)work;fades++;}
static struct backlight_device *devm_backlight_device_register(struct device *dev,
    const char *name,struct device *parent,void *data,const int *ops,
    const struct backlight_properties *props)
{(void)dev;(void)name;(void)parent;assert(ops==&dcp_backlight_ops);
 registered=(struct backlight_device){.props=*props,.data=data};
 if(inject_hint) {inject_hint=false;struct apple_dcp *d=data;assert(dcp_bl_seed(&d->backlight.state,180));}
 return &registered;}
static int backlight_get_brightness(struct backlight_device *bd)
{return bd->props.state?0:bd->props.brightness;}
static bool backlight_is_blank(struct backlight_device *bd) {return !!bd->props.state;}
static u32 calculate_dac(struct apple_dcp *d,int nits) {(void)d;(void)nits;assert(false);return 0;}
static int drm_modeset_lock_single_interruptible(int *p) {(void)p;assert(false);return 0;}
static void drm_modeset_unlock(int *p) {(void)p;assert(false);}
static int dcp_backlight_update(struct apple_dcp *d) {(void)d;assert(false);return 0;}
/* The DAC fade (M1/M2 firmware) never runs on the desktop panel. */
#define system_freezable_wq NULL
static bool dcp_dac_fade_starts(struct apple_dcp *d,u32 nits) {(void)d;(void)nits;assert(false);return false;}
static void mod_delayed_work(void *q,int *w,unsigned delay) {(void)q;(void)w;(void)delay;assert(false);}
''' + functions + r'''
int main(void)
{
    struct apple_dcp d={.fw_compat=5,.brightness={.maximum=600}};
    __typeof__(*d.desktop) saved={0};d.desktop=&saved;
    assert(!dcp_backlight_register(&d));
    assert(registered.props.brightness==300 && registered.props.max_brightness==600);
    assert(dcp_get_brightness(&registered)==300);
    assert(!dcp_backlight_controls_device(&registered,NULL));
    /* A save/restore client echoes the exposed value, not a new target. */
    assert(!dcp_set_brightness(&registered));
    assert(!d.backlight.state.controlled && !d.backlight.state.dirty);
    registered.props.state=BL_CORE_SUSPENDED|BL_CORE_FBBLANK;
    assert(!dcp_set_brightness(&registered));
    registered.props.state=0;assert(!dcp_set_brightness(&registered));
    assert(!d.backlight.state.controlled && !d.backlight.state.dirty);
    assert(dcp_get_brightness(&registered)==300);
    /* A different explicit target takes over the unknown inherited level. */
    registered.props.brightness=250;assert(!dcp_set_brightness(&registered));
    assert(d.backlight.state.controlled && d.backlight.state.fading && !d.backlight.state.dirty && fades==1);
    assert(!dcp_bl_fade_step(&d.backlight.state,250,250));
    assert(dcp_backlight_controls_device(&registered,NULL));
    struct dcp_backlight_present present;
    assert(!dcp_bl_prepare(&d.backlight.state,true,&present) && present.nits==250);
    assert(dcp_get_brightness(&registered)==250);
    assert(dcp_bl_complete(&d.backlight.state,present.sequence,true));
    assert(dcp_get_brightness(&registered)==250);
    /* Even the former midpoint is an ordinary faded target after takeover. */
    registered.props.brightness=300;assert(!dcp_set_brightness(&registered));
    assert(d.backlight.state.fading && fades==2 && d.backlight.state.target==300);
    /* A real loader hint seeds registration without needing a present. */
    d.brightness.bl_dev=NULL;d.backlight.state=(struct dcp_backlight_state){0};
    inject_hint=true;
    assert(!dcp_backlight_register(&d) && registered.props.brightness==180);
    assert(dcp_get_brightness(&registered)==180);
    assert(dcp_backlight_controls_device(&registered,NULL));
    /* A user write racing the late-hint notification keeps its target. */
    inject_user=true;dcp_backlight_sync_hint(&d);
    assert(d.backlight.state.controlled && d.backlight.state.target==450);
    assert(registered.props.brightness==450 && !registered.ops_lock);
    /*
     * DAC firmware (J414s, 13.5): the report of the loader's level until a
     * present, then the presented level, through a display power cycle whose
     * power-off report of 0 is not followed by one for the lit panel.
     */
    struct apple_dcp dac={.fw_compat=0,.brightness={.maximum=500,.nits=140}};
    struct backlight_device dac_bl={.data=&dac};
    assert(dcp_get_brightness(&dac_bl)==140);
    dcp_backlight_presented(&dac,500);
    assert(dcp_get_brightness(&dac_bl)==500);
    dcp_backlight_presented(&dac,0);dac.brightness.nits=0;
    assert(dcp_get_brightness(&dac_bl)==0);
    dcp_backlight_presented(&dac,500);
    assert(dcp_get_brightness(&dac_bl)==500 && !dac.backlight.lock);
    return 0;
}
'''


def modeset_unknown(root):
    template = (root / 'drivers/gpu/drm/apple/iomfb_template.c').read_text()
    unit = modeset_unmap(root)
    helper = block(template, 'static bool dcp_modeset_callback_failed(')
    # Keep the real modeset completion and its callback-error check.
    helper += r'''
struct dcp_modeset_cookie {struct dcp_wait_cookie wait;bool vrr;};
''' + block(template, 'static void dcp_modeset_actual_done(')
    unit = unit.replace('int main(void)', helper + '\nint main(void)')
    tests = r'''
    /* A fresh channel has an A413 outstanding; inject a valid but unknown D. */
    d.crashed=false;desktop.modeset_active=true;desktop.modeset_callback_error=false;
    struct dcp_modeset_cookie mode={.wait={.status=~0U,.refcount=2},.vrr=true};
    d.command.depth=1;d.command.header[0]=command.h;
    d.command.callbacks[0]=dcp_modeset_actual_done;
    d.command.cookies[0]=&mode;d.command.output[0]=&command.status;
    case_tag=499;h->in_len=4;h->out_len=4;
    memset(packet.data+sizeof(*h)+4,0xa5,4);before=acks;
    dcpep_handle_cb(&d,DCP_CONTEXT_CB,h,sizeof(*h)+8,0);
    assert(acks==before+1 && !d.crashed && !d.channel.depth);
    assert(!get_unaligned_le32(packet.data+sizeof(*h)+4));
    assert(desktop.modeset_callback_error && d.command.depth==1 && !mode.wait.done);
    /* Draining the real ACK fails the set, even if firmware reports success. */
    dcpep_handle_ack(&d,DCP_CONTEXT_CMD);
    assert(mode.wait.done && !mode.wait.replied && mode.wait.status==~0U);
    assert(mode.wait.refcount==1 && !d.command.depth);
    /* The next fixed modeset is independent and completes normally. */
    desktop.modeset_active=false;desktop.modeset_callback_error=false;
    mode=(struct dcp_modeset_cookie){.wait={.status=~0U,.refcount=2}};
    d.command.depth=1;d.command.header[0]=command.h;
    d.command.callbacks[0]=dcp_modeset_actual_done;
    d.command.cookies[0]=&mode;d.command.output[0]=&command.status;
    dcpep_handle_ack(&d,DCP_CONTEXT_CMD);
    assert(mode.wait.done && mode.wait.replied && !mode.wait.status);
    assert(!d.crashed && !d.command.depth && !d.channel.depth);
'''
    return unit.replace('    return 0;\n}', tests + '    return 0;\n}')


def panel_identity(root, output=None):
    import struct
    import rate
    f = rate.parser_fixture(root)
    attributes = {'MaxHorizontalImageSize': 0, 'MaxVerticalImageSize': 0,
                  'ProductAttributes': {'ManufacturerID': 'TST', 'ProductID': 42,
                                        'ProductName': 'Synthetic Internal Panel',
                                        'SerialNumber': 0x12345678}}
    raw = bytearray(struct.pack('<I', 0xd3)); f['wire'](attributes, raw)
    product = bytearray(struct.pack('<I', 0xd3)); f['wire'](attributes['ProductAttributes'], product)
    minimal = bytearray(struct.pack('<I', 0xd3)); f['wire']({'LegacyManufacturerID': 0x5274, 'ProductID': 42, 'WeekOfManufacture': 0, 'YearOfManufacture': 0, 'ProductName': '    ', 'AlphanumericSerialNumber': '    '}, minimal)
    code = (f['prefix'] + f['metadata'] + f['epoch'].replace('static inline ', 'static inline __attribute__((unused)) ') + f['header'] +
            f['edid_header'] + f['parser'] + f['stubs'] + f['builder'])
    source = (root / 'drivers/gpu/drm/apple/dcp.c').read_text()
    helpers = '\n'.join(block(source, name) for name in (
        'void dcp_panel_metadata_replace(', 'static bool dcp_panel_edid_has_identity(', 'void dcp_panel_edid_receive(', 'bool dcp_panel_metadata_current(',
        'const struct drm_edid *dcp_panel_edid_snapshot('))
    backend = (root / 'drivers/gpu/drm/apple/iomfb_v26_6_desktop.c').read_text()
    dictionary = block((root/'drivers/gpu/drm/apple/parser.c').read_text(), 'static int parse_panel_edid_value(')
    dictionary += block((root/'drivers/gpu/drm/apple/parser.c').read_text(), 'int parse_panel_edid_data(')
    # The dictionary record is shared with firmware 27's analytics event.
    dictionary += block((root/'drivers/gpu/drm/apple/iomfb.h').read_text(), 'struct dcp_dict_passthrough_req {')+';\n'
    dictionary += block(backend, 'static bool trampoline_panel_dictionary_v26_6(')
    dictionary += block(backend, 'static bool iomfb_check_callback_exact_v26_6_desktop(')
    dictionary += '\n#define iomfb_check_callback_v26_6_desktop iomfb_check_callback_exact_v26_6_desktop\n'
    bindings = '\n'.join(line for line in block(backend, 'static const iomfb_cb_handler cb_handlers[').splitlines()
                         if '[413]' in line or '[552]' in line or '[561]' in line)
    unit = code + r'''
#define DCP_FIRMWARE_V_26_6_DESKTOP 5
#define trace_iomfb_callback(...) ((void)0)
#define dcp_trace_panel_property(...) ((void)0)
#define GFP_KERNEL 0
#define kmalloc_objs(t,n,f) malloc(sizeof(t)*(n))
#define spin_lock_irqsave(p,f) do {assert(!*(p));*(p)=true;(f)=0;} while(0)
#define spin_unlock_irqrestore(p,f) do {assert(*(p));*(p)=false;(void)(f);} while(0)
#ifndef READ_ONCE
#define READ_ONCE(x) (x)
#endif
''' + block((root / 'drivers/gpu/drm/apple/dcp-internal.h').read_text(), 'struct dcp_panel_identity {') + r''';
struct apple_dcp {
    int fw_compat;bool has_panel;struct {bool lock;} metadata;
    struct dcp_panel_identity *panel_identity;
    struct {u32 width_mm,height_mm,peak_nits;const struct dcp_color_space *color_target;} panel;void *connector;
};
static unsigned notifications;
static bool dcp_has_panel(struct apple_dcp *d) {return d->has_panel;}
static void dcp_queue_hotplug(void *con) {assert(con);notifications++;}
''' + helpers + dictionary + '\nstatic const u8 attributes[]={'+','.join(map(str,raw))+'};\n' + 'static const u8 product[]={'+','.join(map(str,product))+'};\n' + 'static const u8 minimal[]={'+','.join(map(str,minimal))+'};\n' + r'''
int main(int argc, char **argv)
{
    struct dcp_parse_ctx ctx;
    struct dcp_display_metadata metadata;
    assert(!parse(attributes,sizeof(attributes),&ctx));
    assert(!parse_panel_metadata(&ctx,&metadata));
    assert(!strcmp(metadata.manufacturer,"TST") && metadata.product==42 && metadata.serial==0x12345678);
    assert(!strcmp(metadata.name,"Synthetic Int"));
    /* Optional color/date fields are absent; timings come from the catalog. */
    struct apple_dcp d={.fw_compat=5,.has_panel=true,.panel={345,223},.connector=&d};
    __typeof__(*d.panel_identity) desktop={0};d.panel_identity=&desktop;
    struct dcp_display_mode modes[3]={0};
    for(unsigned i=0;i<3;i++)modes[i].mode=(struct drm_display_mode){.clock=i==0?1080005:i==1?540005:432000,
        .hdisplay=3456,.hsync_start=3500,.hsync_end=3550,.htotal=4000,
        .vdisplay=2160,.vsync_start=2165,.vsync_end=2170,.vtotal=2250,
        .flags=DRM_MODE_FLAG_PHSYNC|DRM_MODE_FLAG_PVSYNC,.type=i==0?DRM_MODE_TYPE_PREFERRED:0};
    modes[0].vrr=true;modes[0].min_vrr=24U<<16;modes[0].max_vrr=120U<<16;
    u64 generation=99;
    desktop.adt=(struct dcp_display_metadata){.manufacturer="APP",.product=0x616};
    /* A peak above the SDR range adds the HDR block: 1600 cd/m2 is code 160. */
    d.panel.peak_nits=1600;
    const struct drm_edid *hdr=dcp_panel_edid_snapshot(&d,modes,3,&generation);
    assert(!IS_ERR(hdr) && drm_edid_valid(hdr) && hdr->size==256);
    assert(!memcmp(hdr->cta,"\x02\x03\x0f\x00\xe3\x05\x80\x00\xe6\x06\x05\x01\xa0\xa0\x00",15));
    drm_edid_free(hdr);
    d.panel.peak_nits=0;
    const struct drm_edid *fallback=dcp_panel_edid_snapshot(&d,modes,3,&generation);
    assert(!IS_ERR(fallback) && drm_edid_valid(fallback) && fallback->size==128);
    if(argc==2) {
        char path[4096];snprintf(path,sizeof(path),"%s.adt-fallback",argv[1]);
        FILE *file=fopen(path,"wb");assert(file);assert(fwrite(&fallback->raw,128,1,file)==1);fclose(file);
    }
    drm_edid_free(fallback);
    bool (*handlers[600])(struct apple_dcp *,int,void *,void *)={
''' + bindings + r'''
    };
    for(unsigned form=0;form<3;form++) {
        desktop=(__typeof__(desktop)){0};notifications=0;
        struct dcp_dict_passthrough_req wire={0};u32 result=0;
        strcpy((char *)wire.key,form?"ProductAttributes":"DisplayAttributes");
        const u8 *data=form==2?minimal:form?product:attributes;
        size_t size=form==2?sizeof(minimal):form?sizeof(product):sizeof(attributes);
        memcpy(wire.value,data,size);
        unsigned tag=form?552:561;
        assert(iomfb_check_callback_v26_6_desktop(tag,sizeof(wire),4));
        assert(iomfb_check_callback_v26_6_desktop(tag,sizeof(wire),1));
        assert(!iomfb_check_callback_v26_6_desktop(tag,sizeof(wire)-1,4));
        u8 short_reply[2]={0,0xa5};
        assert(handlers[tag](&d,tag,short_reply,&wire));
        assert(short_reply[0]==1 && short_reply[1]==0xa5);
        assert(handlers[tag](&d,tag,&result,&wire) && result==1);
        u8 service[sizeof(wire)+4]={0};memcpy(service+4,&wire,sizeof(wire));
        assert(iomfb_check_callback_v26_6_desktop(413,sizeof(service),4));
        assert(!iomfb_check_callback_v26_6_desktop(413,sizeof(service)-1,4));
        assert(handlers[413](&d,413,&result,service) && result==1);
        assert(desktop.metadata_valid && notifications==1);
        assert(desktop.metadata.serial==(form==2?0:metadata.serial));
        assert(!strcmp(desktop.metadata.manufacturer,"TST"));
        if(form==2) {
            const struct drm_edid *minimal_edid=dcp_panel_edid_snapshot(&d,modes,3,&generation);
            assert(!IS_ERR(minimal_edid) && drm_edid_valid(minimal_edid));
            const u8 *m=(const void *)&minimal_edid->raw;
            assert(!memcmp(m+72+5,"TST 002a",8) && !memcmp(m+90+5,"unspecified",11));
            drm_edid_free(minimal_edid);
        }
        /* Null and unrelated setters preserve the existing ACK and identity. */
        wire.value_null=1;result=0;
        assert(handlers[tag](&d,tag,&result,&wire) && result==1 && notifications==1);
        wire.value_null=0;strcpy((char *)wire.key,"Unrelated");result=0;
        assert(handlers[tag](&d,tag,&result,&wire) && result==1 && notifications==1);
    }
    desktop=(__typeof__(desktop)){0};notifications=0;
    dcp_panel_metadata_replace(&d,&metadata);
    assert(notifications==1 && desktop.metadata_valid);
    const struct drm_edid *edid=dcp_panel_edid_snapshot(&d,modes,3,&generation);
    assert(!IS_ERR(edid) && drm_edid_valid(edid) && dcp_panel_metadata_current(&d,generation));
    const u8 *bytes=(const void *)&edid->raw;
    assert(bytes[8]==0x52 && bytes[9]==0x74 && bytes[10]==42 && !bytes[11]);
    assert(bytes[12]==0x78 && bytes[13]==0x56 && bytes[14]==0x34 && bytes[15]==0x12);
    assert(bytes[18]==1 && bytes[19]==4 && bytes[21]==35 && bytes[22]==22 && bytes[23]==0xff);
    assert(bytes[16]==0xff && bytes[17]==34);
    assert(((unsigned)bytes[33]<<2 | ((bytes[26]>>2)&3))==320);
    assert(((unsigned)bytes[34]<<2 | (bytes[26]&3))==337);
    assert(((unsigned)bytes[66] | (unsigned)(bytes[68] >> 4) << 8)==345);
    assert(((unsigned)bytes[67] | (unsigned)(bytes[68] & 15) << 8)==223);
    assert(bytes[72+3]==EDID_DETAIL_MONITOR_NAME && bytes[90+3]==EDID_DETAIL_MONITOR_SERIAL);
    assert(!memcmp(bytes+90+5,"12345678\n",9));
    if(argc==2) {FILE *file=fopen(argv[1],"wb");assert(file);assert(fwrite(bytes,128,1,file)==1);fclose(file);}
    assert(((unsigned)bytes[54]|(unsigned)bytes[55]<<8)==54001);
    assert(bytes[108+3]==EDID_DETAIL_MONITOR_RANGE && bytes[108+5]==24 && bytes[108+6]==120);
    assert(bytes[108+4] & DRM_EDID_RANGE_OFFSET_MAX_HFREQ);
    /* The encoded timing clock rounds to 10 kHz; the DCP catalog is unchanged. */
    assert(modes[1].mode.clock==540005 && modes[0].mode.clock==1080005);
    /* A real EDID replaces the fallback; invalid replacement preserves it. */
    u8 real[128];memcpy(real,bytes,128);real[12]^=1;real[127]^=1;
    dcp_panel_edid_receive(&d,real,sizeof(real));
    assert(desktop.raw_edid_size==128);
    u8 nested[148]={0};u32 word;
    word=0xd3;memcpy(nested,&word,4);word=0x01000001;memcpy(nested+4,&word,4);
    word=0x09000004;memcpy(nested+8,&word,4);memcpy(nested+12,"EDID",4);
    word=0x0a000080;memcpy(nested+16,&word,4);memcpy(nested+20,real,128);
    const u8 *nested_raw=NULL;size_t nested_size=0;
    assert(!parse_panel_edid_data(nested,sizeof(nested),&nested_raw,&nested_size));
    assert(nested_size==128 && !memcmp(nested_raw,real,128));
    dcp_panel_edid_receive(&d,nested,sizeof(nested));
    generation=desktop.generation;
    const struct drm_edid *copied=dcp_panel_edid_snapshot(&d,modes,3,&generation);
    assert(!IS_ERR(copied) && !memcmp(&copied->raw,real,128));drm_edid_free(copied);
    unsigned previous=notifications;
    real[127]^=1;dcp_panel_edid_receive(&d,real,sizeof(real));
    assert(notifications==previous);
    /* A checksum-valid block with no serial must not reintroduce null strings. */
    memset(real+12,0,4);memset(real+95,' ',13);real[127]=0;
    for(unsigned j=0;j<127;j++)real[127]-=real[j];
    assert(!dcp_panel_edid_has_identity(real));
    dcp_panel_edid_receive(&d,real,sizeof(real));assert(notifications==previous);
    desktop.raw_edid_size=0;notifications=1;
    drm_edid_free(edid);
    /* Repeated and partial dictionaries cannot trigger monitor/brightness restore loops. */
    for(unsigned repeat=0;repeat<32;repeat++) {
        dcp_panel_metadata_replace(&d,NULL);
        dcp_panel_metadata_replace(&d,&metadata);
        assert(dcp_panel_metadata_current(&d,generation) && notifications==1);
    }
    struct dcp_display_metadata partial={.manufacturer="TST",.product=42};
    dcp_panel_metadata_replace(&d,&partial);
    assert(dcp_panel_metadata_current(&d,generation) && notifications==1);
    assert(desktop.metadata.serial==metadata.serial);
    metadata.serial++;
    dcp_panel_metadata_replace(&d,&metadata);
    assert(!dcp_panel_metadata_current(&d,generation) && notifications==2);
    edid=dcp_panel_edid_snapshot(&d,modes,3,&generation);
    assert(!IS_ERR(edid) && dcp_panel_metadata_current(&d,generation));
    drm_edid_free(edid);
    /* Any firmware's internal panel takes its identity; no identity, nothing published. */
    desktop=(__typeof__(desktop)){0};d.fw_compat=2;dcp_panel_metadata_replace(&d,&metadata);
    assert(desktop.metadata_valid && !strcmp(desktop.metadata.manufacturer,"TST"));
    desktop=(__typeof__(desktop)){0};d.panel_identity=NULL;dcp_panel_metadata_replace(&d,&metadata);
    assert(!desktop.metadata_valid && !dcp_panel_metadata_current(&d,0));
    assert(PTR_ERR(dcp_panel_edid_snapshot(&d,modes,3,&generation))==-ENODEV);
    metadata.manufacturer[0]='?';assert(IS_ERR(dcp_panel_edid(&metadata,NULL,modes,3,345,223)));
    return 0;
}
'''

    return (unit, ((str(output / "desktop-panel-identity.edid"),),)) if output else unit


def powerlog_framing(root):
    import rate
    import struct
    f = rate.parser_fixture(root)
    # Use only the existing integer/string parser; no captured device data.
    raw = bytearray(struct.pack('<I', 0xd3))
    f['wire']({'mNits': 123000, 'iDAC': 42, 'timestamp': 1234,
               'logEvent': 'Display (Event Forward)'}, raw)
    afk = (root / 'drivers/gpu/drm/apple/afk.c').read_text()
    afkh = (root / 'drivers/gpu/drm/apple/afk.h').read_text()
    parser = (root / 'drivers/gpu/drm/apple/parser.c').read_text()
    header = (root / 'drivers/gpu/drm/apple/parser.h').read_text()
    system = (root / 'drivers/gpu/drm/apple/systemep.c').read_text()
    prefix = f['prefix'] + f['metadata'] + f['epoch'].replace('static inline ', 'static inline __attribute__((unused)) ') + f['header'] + f['edid_header'] + f['parser']
    prefix += block(header, 'struct dcp_system_ev_mnits {') + ';\n'
    prefix += '#define __le64 u64\n#define EPIC_SERVICE_CALL_MAGIC 0x69706378\n'
    prefix += block(afkh, 'struct epic_compact_sub_hdr {') + ';\n'
    prefix += block(afkh, 'struct epic_compact_desc {') + ';\n'
    prefix += block(afkh, 'struct epic_service_call {') + ' __attribute__((packed));\n'
    prefix += '#undef kfree\n#define kfree(p) free((void *)(p))\n#define pr_err(...) ((void)0)\n'
    prefix += block(parser, 'int parse_system_log_mnits(')
    prefix += block(parser, 'int parse_system_log_mnits_pairs(')
    prefix += r'''
#define DCP_FIRMWARE_H17P 1
#define DCP_FIRMWARE_V_26_6_DESKTOP 5
#define DCP_FIRMWARE_V_27_0 6
#define DCP_IOMFB_METHODS_H17G 7
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define dev_dbg(...) ((void)0)
#define dev_info(...) ((void)0)
#define dev_warn(...) ((void)0)
#define EPIC_SUBTYPE_STD_SERVICE 0xc0
enum epic_subtype { REPORT = 0xc0 };
struct work_struct { unsigned scheduled; };
static void __attribute__((unused)) schedule_work(struct work_struct *w) { w->scheduled++; }
struct apple_dcp {void *dev;int fw_compat;struct {int iomfb_method_profile;} hw;struct {u32 nits,maximum;void *bl_dev;} brightness;struct work_struct bl_register_wq;};
struct apple_epic_service;
struct apple_dcp_afkep {struct apple_dcp *dcp;unsigned endpoint;struct apple_epic_service *service;};
struct apple_epic_service_ops {int (*report)(struct apple_epic_service *,enum epic_subtype,const void *,size_t);};
struct apple_epic_service {struct apple_dcp_afkep *ep;struct apple_epic_service_ops *ops;unsigned channel,users;};
static unsigned hints;static u32 hint;
static bool __attribute__((unused)) dcp_backlight_active(struct apple_dcp *d) {(void)d;return false;}
static bool dcp_has_panel(struct apple_dcp *d) {(void)d;return true;}
static int dcp_backlight_takeover(struct apple_dcp *d,u32 mnits) {(void)d;hints++;hint=mnits;return 0;}
static u32 get_unaligned_le32(const void *p) {u32 v;memcpy(&v,p,4);return v;}
static struct apple_epic_service *afk_epic_find_service(struct apple_dcp_afkep *e,u16 id)
{assert(e->service->channel==id);return e->service;}
static struct apple_epic_service *afk_service_get(struct apple_epic_service *s) {s->users++;return s;}
static void afk_service_put(struct apple_epic_service *s) {assert(s->users);s->users--;}
static void afk_recv_handle_compact_init(struct apple_dcp_afkep *e,u16 id,const void *p,size_t n)
{(void)e;(void)id;(void)p;(void)n;assert(false);}
static void afk_recv_handle_compact_call(struct apple_dcp_afkep *e,u16 id,const void *p,size_t n)
{(void)e;(void)id;(void)p;(void)n;assert(false);}
static void afk_recv_handle_compact_reply(struct apple_dcp_afkep *e,u16 id,u8 t,const void *p,size_t n)
{(void)e;(void)id;(void)t;(void)p;(void)n;assert(false);}
static unsigned call_reports;static u32 call_magic,call_cmd;
static int call_report_cb(struct apple_epic_service *s,enum epic_subtype t,const void *p,size_t n)
{
    const struct epic_service_call *c=p;
    (void)s;(void)t;
    assert(n==sizeof(*c)+0x10);
    call_reports++;call_magic=get_unaligned_le32(&c->magic);call_cmd=get_unaligned_le32(&c->command);
    return 0;
}
'''
    functions = '\n'.join(block(source, name) for source, name in (
        (system, 'static int powerlog_report('),
        (afk, 'static bool afk_compact_report_dictionary('),
        (afk, 'static bool afk_compact_report_service_call('),
        (afk, 'static void afk_recv_compact_message(')))
    return prefix + '\n' + functions + '\nstatic const u8 report[]={'+','.join(map(str,raw))+'};\n' + r'''
int main(void)
{
    struct apple_dcp d={.fw_compat=5};
    struct apple_dcp_afkep ep={.dcp=&d,.endpoint=0x20};
    struct apple_epic_service_ops ops={.report=powerlog_report};
    struct apple_epic_service service={.ep=&ep,.ops=&ops,.channel=3};ep.service=&service;
    u8 wire[16+8+sizeof(report)]={0};
    struct epic_compact_sub_hdr *sub=(void *)(wire+8);sub->category=0;sub->type=0xc0;
    memcpy(wire+16,report,sizeof(report));
    afk_recv_compact_message(&ep,3,wire,16+sizeof(report));
    assert(hints==1 && hint==123000 && !service.users);
    /* Descriptor-wrapped reports keep their existing transport behavior. */
    memset(wire+16,0,8);memcpy(wire+24,report,sizeof(report));
    afk_recv_compact_message(&ep,3,wire,sizeof(wire));
    assert(hints==2 && hint==123000 && !service.users);
    /* The old unconditional descriptor removal loses the serialization header. */
    struct dcp_parse_ctx ctx;assert(parse(report+8,sizeof(report)-8,&ctx)==-EINVAL);
    /* Truncated data never creates a takeover hint. */
    memcpy(wire+16,report,8);
    afk_recv_compact_message(&ep,3,wire,23);
    assert(hints==2 && !service.users);
    /* Headerless pairs, after the descriptor, on the desktop firmware only. */
    struct dcp_system_ev_mnits mnits;
    assert(!parse_system_log_mnits_pairs(report+8,sizeof(report)-8,&mnits));
    assert(mnits.millinits==123000 && mnits.idac==42 && mnits.timestamp==1234);
    hint=0;memset(wire+16,0,8);memcpy(wire+24,report+8,sizeof(report)-8);
    afk_recv_compact_message(&ep,3,wire,sizeof(wire)-8);
    assert(hints==3 && hint==123000 && !service.users);
    d.fw_compat=DCP_FIRMWARE_H17P;
    afk_recv_compact_message(&ep,3,wire,sizeof(wire)-8);
    assert(hints==3 && !service.users);
    d.fw_compat=DCP_FIRMWARE_V_27_0;
    afk_recv_compact_message(&ep,3,wire,sizeof(wire)-8);
    assert(hints==3 && !service.users);
    d.fw_compat=DCP_FIRMWARE_V_26_6_DESKTOP;
    /* The first key without its value gives no reading. */
    afk_recv_compact_message(&ep,3,wire,24+12);
    assert(hints==3 && !service.users);
    /* Firmware 27 takes a framed report's hint as H17P does. */
    d.fw_compat=DCP_FIRMWARE_V_27_0;hint=0;
    memcpy(wire+16,report,sizeof(report));
    afk_recv_compact_message(&ep,3,wire,16+sizeof(report));
    assert(hints==4 && hint==123000 && !service.users);
    /* Bare and descriptor-wrapped EPIC service-call reports both preserve their header. */
    {
        u8 cwire[16+sizeof(struct epic_compact_desc)+sizeof(struct epic_service_call)+0x10]={0};
        struct epic_compact_sub_hdr *csub=(void *)(cwire+8);
        struct epic_service_call *ccall=(void *)(cwire+16);
        csub->category=0;csub->type=EPIC_SUBTYPE_STD_SERVICE;
        ops.report=call_report_cb;
        ccall->group=2;ccall->command=3;ccall->data_len=0x10;
        ccall->magic=EPIC_SERVICE_CALL_MAGIC;
        afk_recv_compact_message(&ep,3,cwire,16+sizeof(*ccall)+0x10);
        assert(call_reports==1 && call_magic==EPIC_SERVICE_CALL_MAGIC && call_cmd==3 && !service.users);
        memset(cwire+16,0,sizeof(cwire)-16);
        ccall=(void *)(cwire+16+sizeof(struct epic_compact_desc));
        ccall->group=2;ccall->command=4;ccall->data_len=0x10;
        ccall->magic=EPIC_SERVICE_CALL_MAGIC;
        afk_recv_compact_message(&ep,3,cwire,sizeof(cwire));
        assert(call_reports==2 && call_magic==EPIC_SERVICE_CALL_MAGIC && call_cmd==4 && !service.users);
    }
    return 0;
}
'''


def keys_and_reports(root):
    import display
    prefix = display.backlight(root).split('static void completed(')[0]
    prefix = prefix.replace('static inline ', 'static inline __attribute__((unused)) ')
    return prefix + r'''
static void report(struct dcp_backlight_state *s,u32 nits)
{
    struct dcp_backlight_state before=*s;
    assert(!dcp_bl_report(s,nits));
    assert(s->target==before.target && s->level==before.level && s->actual==before.actual);
    assert(s->fade_from==before.fade_from && s->fading==before.fading);
    assert(s->dirty==before.dirty && s->in_flight==before.in_flight && s->sequence==before.sequence);
    assert(s->report_valid && s->reported==nits);
}
static void step(struct dcp_backlight_state *s,u32 elapsed,u32 *id,u32 *previous,bool up)
{
    struct dcp_backlight_swap p;
    dcp_bl_fade_step(s,elapsed,250);
    report(s,(*id & 1)?50:550);
    if(!s->dirty)return;
    assert(!dcp_bl_swap_prepare(s,&p,(*id)++,true));
    report(s,(*id & 1)?0:600);
    if(*id & 1) {
        assert(!dcp_bl_swap_finish(s,&p,p.id,true,true));
        report(s,250);
        assert(dcp_bl_swap_finish(s,&p,p.id,false,true));
    } else {
        assert(!dcp_bl_swap_finish(s,&p,p.id,false,true));
        report(s,160);
        assert(dcp_bl_swap_finish(s,&p,p.id,true,true));
    }
    assert(up?s->actual>=*previous:s->actual<=*previous);
    *previous=s->actual;
}
int main(void)
{
    for(unsigned direction=0;direction<2;direction++) {
        bool up=!direction;u32 initial=up?100:600,previous=initial,id=1;
        struct dcp_backlight_state s;
        assert(!dcp_bl_init(&s,600,false,0,true,300));
        s.fade=true;s.preserve_inherited=true;
        assert(dcp_bl_report(&s,initial) && s.actual==initial && !s.dirty);
        report(&s,up?550:50); /* Only the first hint can establish takeover. */
        for(unsigned key=1;key<=10;key++) {
            u32 target=up?100+key*50:600-key*50;
            assert(!dcp_bl_request(&s,target,false,false));
            report(&s,up?50:550);
            /* Key repeat retargets before any fade can finish. */
            step(&s,16,&id,&previous,up);step(&s,32,&id,&previous,up);
            assert(s.target==target);
        }
        u32 last=up?600:100;
        for(u32 time=48;time<=256;time+=16)step(&s,time,&id,&previous,up);
        assert(s.target==last && s.actual==last && s.level==last && !s.fading && !s.dirty);
        for(unsigned late=0;late<64;late++)report(&s,late&1?0:600);
        assert(s.target==last && s.actual==last && !s.dirty);
    }
    return 0;
}
'''


def identity_keys(root):
    import rate, struct
    f = rate.parser_fixture(root)
    data = bytearray(struct.pack('<I', 0xd3))
    f['wire']({'ProductAttributes': {'ManufacturerID': 'TST', 'SerialNumber': 42},
               'Other': [True, 7]}, data)
    source = (root/'drivers/gpu/drm/apple/parser.c').read_text()
    unit = f['prefix'] + f['metadata'] + f['epoch'] + f['header'] + f['edid_header'] + f['parser']
    unit += block(source, 'static int parse_identity_tree(')
    unit += block(source, 'int parse_identity_keys(')
    unit += 'static const unsigned char blob[] = {' + ','.join(map(str,data)) + '};\n'
    return unit + r'''
static unsigned seen;
static void emit(void *data,const char *key,u32 len,u32 type,u32 size,u32 depth)
{
    (void)data;(void)size;assert(len && key && depth<5);
    assert(type==1 || type==2 || type==4 || type==9 || type==11);seen++;
}
int main(void)
{
    assert(!parse_identity_keys(blob,sizeof(blob),emit,NULL) && seen==7);
    for(unsigned i=0;i<sizeof(blob)-4;i++) {
        seen=0;assert(parse_identity_keys(blob,i,emit,NULL)<0);assert(seen<=7);
    }
    u32 huge[]={0xd3,(1U<<24)|0xffffff};
    assert(parse_identity_keys(huge,sizeof(huge),emit,NULL)==-E2BIG);
    return 0;
}
'''


def adt_identity(root):
    import struct
    def node(props, children=()):
        out=bytearray(struct.pack('<II',len(props),len(children)))
        for k,v in props.items():
            if isinstance(v,int):v=struct.pack('<I',v)
            out+=k.encode().ljust(32,b'\0')+struct.pack('<I',len(v))+v
            out+=b'\0'*(-len(out)%4)
        return bytes(out)+b''.join(children)
    panel=node({'product-id':42,'vendor-id':0x5274,'width-mm':345,'height-mm':223,
                'serial-number':b'SYNTHETIC-PANEL','name':b'panel\0'})
    # A synthetic potential headroom of 6.5; a nested node of that name is no
    # panel backlight and must not override it.
    backlight=node({'name':b'backlight\0','EDRPotentialHeadroom':0x68000},
                   [node({'name':b'backlight\0','EDRPotentialHeadroom':0x10000})])
    data=node({'name':b'device-tree\0'},[node({'name':b'product\0'},[panel]),
              node({'name':b'chosen\0','serial-number':b'PRIVATE-MACHINE'}),backlight])
    base=root/'drivers/gpu/drm/apple'
    source=(base/'dcp-panel-adt.c').read_text()
    source='\n'.join(l for l in source.splitlines() if not l.startswith('#include'))
    header=(base/'dcp-panel-adt.h').read_text()
    code=r'''
#include <assert.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
typedef uint8_t u8;typedef uint32_t u32;typedef int32_t s32;typedef int64_t s64;
#define U16_MAX UINT16_MAX
#define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
#define SZ_16M (16U<<20)
#define ALIGN(x,n) (((x)+(n)-1)&~((n)-1))
#define ERR_PTR(x) ((void *)(intptr_t)(x))
#define PTR_ERR(x) ((int)(intptr_t)(x))
#define IS_ERR(x) ((uintptr_t)(x)>(uintptr_t)-4096)
static u32 get_unaligned_le32(const void *p) {u32 n;memcpy(&n,p,4);return n;}
'''
    metadata=(base/'dcp-display-metadata.h').read_text()
    code+=block(metadata,'struct dcp_display_metadata {')+';\n'
    code+=block(metadata,'enum dcp_panel_gamut {')+';\n'
    code+=block(header,'struct dcp_panel_adt {')+';\n'+source
    code+='static const u8 fixture[]={'+','.join(map(str,data))+'};\n'
    return code+r'''
int main(void)
{
    struct dcp_panel_adt panel={.identity={.manufacturer="APP",.product=0x616}};
    assert(!dcp_panel_adt_parse(fixture,sizeof(fixture),&panel));
    assert(panel.found && panel.width_mm==345 && panel.height_mm==223);
    assert(!strcmp(panel.identity.manufacturer,"TST") && panel.identity.product==42);
    assert(panel.identity.serial && panel.identity.serial==dcp_adt_serial((const u8 *)"SYNTHETIC-PANEL",15));
    assert(panel.edr_potential_headroom==0x68000);
    for(unsigned n=0;n<sizeof(fixture);n++) {
        struct dcp_panel_adt old=panel;
        assert(dcp_panel_adt_parse(fixture,n,&panel)<0);
        assert(!memcmp(&old,&panel,sizeof(panel)));
    }
    u32 huge[]={5000,0};assert(dcp_panel_adt_parse(huge,sizeof(huge),&panel)==-E2BIG);
    assert(dcp_panel_adt_parse(NULL,0,&panel)==-EINVAL);
    return 0;
}
'''


def brightness_notifications(root):
    import display
    prefix=display.backlight(root).split('static void completed(')[0]
    prefix=prefix.replace('static inline ', 'static inline __attribute__((unused)) ')
    source=(root/'drivers/gpu/drm/apple/dcp_backlight.c').read_text()
    functions='\n'.join(block(source,name) for name in (
        'static void dcp_backlight_notify_work(', 'static void dcp_backlight_notify_release(', 'static void dcp_backlight_notify(',
        'static bool dcp_backlight_desktop(', 'void dcp_backlight_desktop_finish(',
        'void dcp_backlight_stop('))
    return prefix+r'''
#include <stddef.h>
#include <string.h>
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define DCP_FIRMWARE_V_26_6_DESKTOP 5
#define spin_lock_irqsave(p,f) do {assert(!*(p));*(p)=true;(f)=0;} while(0)
#define spin_unlock_irqrestore(p,f) do {assert(*(p));*(p)=false;(void)(f);} while(0)
#define container_of(p,t,m) ((t *)((char *)(p)-offsetof(t,m)))
#define KOBJ_CHANGE 1
#define system_dfl_wq NULL
#define msecs_to_jiffies(x) (x)
struct work_struct {int unused;};
struct backlight_device {struct {int kobj;} dev;struct {u32 brightness;} props;};
struct apple_dcp {
    int fw_compat;bool quiescing,crashed;
    struct {struct backlight_device *bl_dev;int fade_work;} brightness;
    struct {bool lock;struct dcp_backlight_state state;struct work_struct notify_work;int fade_work,retry_work;} backlight;
    struct {struct dcp_backlight_swap backlight;} *desktop;
};
static unsigned events,brightness,actual,kicks,cancelled,delayed_cancelled;
static void cancel_delayed_work_sync(int *work) {(void)work;delayed_cancelled++;}
static void cancel_work_sync(struct work_struct *w) {(void)w;cancelled++;}
static void kobject_uevent_env(int *k,int action,char **env)
{(void)k;assert(action==1 && !strcmp(env[0],"SOURCE=unknown") && !env[1]);events++;}
static void sysfs_notify(int *k,void *dir,const char *name)
{(void)k;assert(!dir);if(!strcmp(name,"brightness"))brightness++;else {assert(!strcmp(name,"actual_brightness"));actual++;}}
static void dcp_backlight_notify_work(struct work_struct *);
static void schedule_work(struct work_struct *w) {dcp_backlight_notify_work(w);}
static void dcp_backlight_kick(struct apple_dcp *d) {assert(!d->backlight.lock);kicks++;}
static void mod_delayed_work(void *q,int *w,unsigned delay) {(void)q;(void)w;assert(delay);}
'''+functions+r'''
int main(void)
{
    struct backlight_device bd={.props={500}};
    struct apple_dcp d={.fw_compat=5,.brightness={&bd}};
    __typeof__(*d.desktop) desktop={0};d.desktop=&desktop;
    assert(!dcp_bl_init(&d.backlight.state,600,true,100,false,0));
    assert(!dcp_bl_request(&d.backlight.state,500,false,false));
    u32 target=d.backlight.state.target;
    /* Accepted/completed presents update userspace once, duplicate callbacks do not. */
    assert(!dcp_bl_swap_prepare(&d.backlight.state,&desktop.backlight,7,true));
    dcp_backlight_desktop_finish(&d,8,false,true);assert(!events);
    dcp_backlight_desktop_finish(&d,7,true,true);assert(!events);
    dcp_backlight_desktop_finish(&d,7,false,true);assert(events==1 && brightness==1 && actual==1);
    dcp_backlight_desktop_finish(&d,7,false,true);assert(events==1);
    assert(d.backlight.state.target==target && bd.props.brightness==500);
    assert(!dcp_bl_request(&d.backlight.state,250,false,false));
    assert(!dcp_bl_swap_prepare(&d.backlight.state,&desktop.backlight,9,true));
    dcp_backlight_desktop_finish(&d,9,true,false);assert(events==1);
    d.quiescing=true;dcp_backlight_notify(&d);assert(events==1);
    d.quiescing=false;d.fw_compat=1;dcp_backlight_notify(&d);assert(events==1);
    dcp_backlight_stop(&d);assert(!cancelled && delayed_cancelled==3);
    d.fw_compat=5;dcp_backlight_notify_release(&d);
    assert(!d.brightness.bl_dev && cancelled==1);dcp_backlight_notify(&d);assert(events==1);
    dcp_backlight_stop(&d);assert(cancelled==2 && delayed_cancelled==6);
    return 0;
}
'''


def identity_trace_budget(root):
    source=(root/'drivers/gpu/drm/apple/iomfb.c').read_text()
    code=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
typedef uint32_t u32;
typedef struct {int value;} atomic_t;
static int atomic_read(const atomic_t *n) {return n->value;}
static bool atomic_add_unless(atomic_t *n,int add,int unless)
{if(n->value==unless)return false;n->value+=add;return true;}
#define READ_ONCE(x) (x)
#define min_t(t,a,b) ((t)(a)<(t)(b)?(t)(a):(t)(b))
#define DCP_FIRMWARE_V_26_6_DESKTOP 5
struct apple_dcp {int fw_compat;bool has_panel;void *dev;struct {atomic_t identity_log_count[4];} *desktop;};
static bool kms_debug;
#define DRM_UT_KMS 4
#define fallthrough __attribute__((fallthrough))
typedef uint8_t u8;
static bool drm_debug_enabled(int category) {assert(category==DRM_UT_KMS);return kms_debug;}
static unsigned messages;
static char output[256];
static bool dcp_has_panel(struct apple_dcp *d) {return d->has_panel;}
static void dev_info(void *dev,const char *fmt,...)
{
    (void)dev;va_list args;va_start(args,fmt);vsnprintf(output,sizeof(output),fmt,args);va_end(args);messages++;
}
typedef void (*dcp_identity_key_fn)(void *,const char *,u32,u32,u32,u32);
static int parse_identity_keys(const void *blob,size_t len,dcp_identity_key_fn emit,void *data)
{
    (void)blob;(void)len;
    for(unsigned i=0;i<70;i++)emit(data,"Key\n",4,9,25,0);
    return 0;
}
'''
    code+=block(source,'struct dcp_identity_trace {')+';\n'
    code+=block(source,'static void dcp_identity_key_log(')
    code+=block(source,'void dcp_trace_panel_property(')
    code+=block(source,'static void dcp_trace_panel_setter(')
    return code+r'''
int main(void)
{
    struct apple_dcp d={.fw_compat=5,.has_panel=true};
    __typeof__(*d.desktop) desktop={0};d.desktop=&desktop;
    const char secret[]="SYNTHETIC_VALUE_NOT_FOR_LOGS";
    dcp_trace_panel_property(&d,561,"Property",8,secret,sizeof(secret));assert(!messages);
    dcp_trace_panel_setter(&d,563,NULL,76);assert(!messages);
    kms_debug=true;d.fw_compat=1;
    dcp_trace_panel_property(&d,561,"Property",8,secret,sizeof(secret));assert(!messages);
    d.fw_compat=5;d.has_panel=false;
    dcp_trace_panel_property(&d,561,"Property",8,secret,sizeof(secret));assert(!messages);
    d.has_panel=true;
    u8 setter[4168]={0};strcpy((char *)setter,"ScalarKey");
    memcpy(setter+64,secret,sizeof(secret));setter[72]=0;
    dcp_trace_panel_setter(&d,563,setter,75);assert(!messages);
    dcp_trace_panel_setter(&d,563,setter,76);
    assert(strstr(output,"ScalarKey type=4 size=8") && !strstr(output,secret));
    dcp_trace_panel_setter(&d,565,setter,72);assert(strstr(output,"type=11"));
    dcp_trace_panel_setter(&d,567,setter,128);assert(strstr(output,"type=9"));
    setter[72]=1;dcp_trace_panel_setter(&d,563,setter,76);assert(strstr(output,"size=0"));
    memset(setter,0,sizeof(setter));strcpy((char *)setter+4,"ServiceKey");
    dcp_trace_panel_setter(&d,414,setter,80);assert(strstr(output,"ServiceKey type=4"));
    dcp_trace_panel_setter(&d,415,setter,76);assert(strstr(output,"ServiceKey type=11"));
    dcp_trace_panel_setter(&d,102,setter,68);dcp_trace_panel_setter(&d,104,setter,68);
    for(unsigned i=0;i<100;i++)dcp_trace_panel_property(&d,114,"Statistics",10,secret,sizeof(secret));
    assert(atomic_read(&desktop.identity_log_count[2])==128 && !atomic_read(&desktop.identity_log_count[0]) && !atomic_read(&desktop.identity_log_count[1]));
    unsigned before=messages;
    dcp_trace_panel_property(&d,552,"Panel",5,secret,sizeof(secret));assert(messages>before);
    dcp_trace_panel_property(&d,561,"Panel",5,secret,sizeof(secret));
    assert(atomic_read(&desktop.identity_log_count[0]) && atomic_read(&desktop.identity_log_count[1]));
    assert(strstr(output,"Key?") && !strstr(output,secret));
    for(unsigned i=0;i<100;i++) {
        dcp_trace_panel_property(&d,552,"Panel",5,secret,sizeof(secret));
        dcp_trace_panel_property(&d,561,"Panel",5,secret,sizeof(secret));
        dcp_trace_panel_property(&d,0,"Chunks",6,secret,sizeof(secret));
    }
    for(unsigned i=0;i<4;i++)assert(atomic_read(&desktop.identity_log_count[i])==128);
    before=messages;dcp_trace_panel_property(&d,561,"Panel",5,NULL,0);assert(messages==before);
    assert(messages<=520);
    return 0;
}
'''


def gamma(root):
    base = root / 'drivers/gpu/drm/apple'
    header = (base / 'iomfb.h').read_text()
    template = (base / 'iomfb_template.c').read_text()
    prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32, __le32;
typedef int64_t s64;
#define __packed __attribute__((packed))
#define le32_to_cpu(v) (v)
#define cpu_to_le32(v) (v)
#define div_s64(a, b) ((a) / (b))
'''
    prefix += next(line for line in header.splitlines()
                   if line.startswith('#define IOMFB_GAMMA_ENTRIES')) + '\n'
    for name in ('req', 'resp'):
        prefix += block(header, 'struct iomfb_gamma_table_' + name + ' {') + ';\n'
    prefix += r'''
struct drm_color_lut { u16 red, green, blue, reserved; };
struct dcp_desktop_state {
    struct iomfb_gamma_table_req *gamma_fw, *gamma_out;
    struct drm_color_lut gamma_lut[IOMFB_GAMMA_ENTRIES];
    bool gamma_user, gamma_fw_read, gamma_unusable;
};
struct apple_dcp { struct dcp_desktop_state *desktop; bool panel; void *dev; };
static unsigned warnings, debug, swaps, writes;
static bool dcp_has_panel(struct apple_dcp *dcp) { return dcp->panel; }
static void *memchr_inv(const void *p, int c, size_t n)
{
    const unsigned char *bytes = p;
    for (size_t i = 0; i < n; i++)
        if (bytes[i] != c) return (void *)(bytes + i);
    return NULL;
}
#define dev_warn(dev, ...) ((void)(dev), warnings++)
#define dev_dbg(dev, ...) ((void)(dev), debug++)
static void do_swap(struct apple_dcp *dcp, void *out, void *cookie)
{ assert(dcp && !out && !cookie); swaps++; }
static const int dcp_set_gamma_table_method;
static struct iomfb_gamma_table_req sent;
static void dcp_push(struct apple_dcp *dcp, bool oob, const int *method,
                     size_t in_size, size_t out_size, void *in,
                     void (*cb)(struct apple_dcp *, void *, void *), void *cookie)
{
    assert(!oob && method == &dcp_set_gamma_table_method);
    assert(in_size == sizeof(sent));
    assert(out_size == sizeof(struct iomfb_gamma_table_resp));
    struct iomfb_gamma_table_resp reply = {0};
    memcpy(&sent, in, sizeof(sent));
    writes++;
    cb(dcp, &reply, cookie);
}
'''
    production = '\n'.join(block(template, 'static ' + signature) for signature in (
        'u32 dcp_gamma_lookup(', 'void dcp_gamma_set_done(',
        'void dcp_gamma_set(', 'void dcp_gamma_read('))
    suffix = r'''
static struct iomfb_gamma_table_req fw, output;
static struct dcp_desktop_state state;
static struct apple_dcp dcp;
static struct iomfb_gamma_table_resp reply;
static void reset(bool panel)
{
    memset(&fw, 0xa5, sizeof(fw));
    memset(&output, 0, sizeof(output));
    memset(&state, 0, sizeof(state));
    memset(&reply, 0, sizeof(reply));
    state.gamma_fw = &fw;
    state.gamma_out = &output;
    dcp = (struct apple_dcp){ .desktop = &state, .panel = panel };
    warnings = debug = swaps = writes = 0;
}
static void increasing(void)
{
    /* Arbitrary synthetic scales: no assertion about the firmware encoding. */
    for (unsigned ch = 0; ch < 3; ch++)
        for (unsigned i = 0; i < IOMFB_GAMMA_ENTRIES; i++)
            reply.table[ch][i] = (ch + 1) * i * i;
}
static void rejected(bool warn)
{
    dcp_gamma_read(&dcp, &reply, NULL);
    assert(state.gamma_unusable && !state.gamma_fw_read);
    assert(warnings == warn && swaps == 1 && writes == 0);
    assert(debug == !warn);
    /* Failure must not turn the reply into a calibration base. */
    const unsigned char *bytes = (const void *)&fw;
    for (size_t i = 0; i < sizeof(fw); i++) assert(bytes[i] == 0xa5);
}
int main(void)
{
    _Static_assert(sizeof(reply) == 3088, "gamma response size");
    _Static_assert(offsetof(struct iomfb_gamma_table_resp, ret) == 3084,
                   "gamma return status offset");
    for (unsigned panel = 0; panel < 2; panel++) {
        reset(panel); reply.ret = 2; rejected(panel);
        reset(panel); reply.ret = 1; rejected(true);
        reset(panel); reply.ret = 0x100; increasing(); rejected(true);
        reset(panel); reply.ret = 2; increasing(); rejected(true);
        reset(panel); rejected(true); /* Successful but empty table. */
        for (unsigned ch = 0; ch < 3; ch++) {
            reset(panel); increasing();
            reply.table[ch][10] = reply.table[ch][9] - 1;
            rejected(true);
            reset(panel); increasing();
            memset(reply.table[ch], 0, sizeof(reply.table[ch]));
            rejected(true);
        }
        reset(panel);
        dcp_gamma_read(&dcp, NULL, NULL);
        assert(state.gamma_unusable && !state.gamma_fw_read);
        assert(warnings == 1 && swaps == 1 && writes == 0);

        reset(panel); increasing();
        dcp_gamma_read(&dcp, &reply, NULL);
        assert(state.gamma_fw_read && !state.gamma_unusable);
        assert(!warnings && writes == 1 && swaps == 1);
        assert(!sent.table_null);
        assert(!memcmp(sent.table, reply.table, sizeof(sent.table)));
        assert(!sent.padding[0] && !sent.padding[1] && !sent.padding[2]);

        /* Nonlinear, independent RGB channels; endpoints and interpolation. */
        state.gamma_user = true;
        for (unsigned i = 0; i < IOMFB_GAMMA_ENTRIES; i++) {
            state.gamma_lut[i].red = 0;
            state.gamma_lut[i].green = 0x7fff;
            state.gamma_lut[i].blue = 0xffff;
        }
        dcp_gamma_set(&dcp);
        assert(writes == 2 && swaps == 2 && !warnings);
        for (unsigned i = 0; i < IOMFB_GAMMA_ENTRIES; i++) {
            assert(sent.table[0][i] == 0);
            assert(sent.table[1][i] == 32767);
            assert(sent.table[2][i] == 196608);
        }
        state.gamma_user = false;
        dcp_gamma_set(&dcp);
        assert(writes == 3 && swaps == 3 && !warnings);
        assert(!memcmp(sent.table, reply.table, sizeof(sent.table)));
    }
    puts("gamma RPC status, admission, composition and restoration passed");
    return 0;
}
'''
    return prefix + production + suffix
