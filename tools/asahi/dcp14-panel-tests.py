#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Host checks for current14 internal-panel admission and boot geometry."""
from pathlib import Path
import argparse
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--old-source', type=Path)
args = parser.parse_args()
source = (root/'drivers/gpu/drm/apple/iomfb_v14_7.c').read_text()
board_header = root/'drivers/gpu/drm/apple/iomfb_v14_7_board.h'
board_types = board_header.read_text() if board_header.exists() else ''
cases = [('current', source)]
if args.old_source:
    cases.append(('old', args.old_source.read_text()))
with tempfile.TemporaryDirectory(prefix='dcp14-panels-') as directory:
    out = Path(directory)
    prefix=r'''#include <assert.h>
    #include <stdint.h>
    #include <stdbool.h>
    #include <stddef.h>
    #include <string.h>
    #include <errno.h>
    #include <stdio.h>
    typedef uint32_t u32;
    #define A(n) ((u32)'A'<<24|('0'+(n)/100)<<16|('0'+(n)/10%10)<<8|('0'+(n)%10))
    #define ARRAY_SIZE(a) (sizeof(a)/sizeof((a)[0]))
    #define ERR_PTR(e) ((void *)(intptr_t)(e))
    #define IS_ERR(p) ((uintptr_t)(p) >= (uintptr_t)-4095)
    #define dev_err(...) ((void)0)
    #define MAX_NOTCH_HEIGHT 100
    struct device_node {const char *compatible;};
    struct device {struct device_node *of_node;};
    static const char *machine;
    static bool of_device_is_compatible(const struct device_node *n,const char *c){return !strcmp(n->compatible,c);}
    static bool of_machine_is_compatible(const char*c){return !strcmp(machine,c);}
    static struct device_node fb={"simple-framebuffer"},dcp={"apple,t8122-dcp"};
    static u32 width,height,stride,notch,marker_value=1;
    static const char *image_uuid;
    static int of_property_read_string(struct device_node*n,const char*k,const char**v){*v=image_uuid;return 0;}
    #define dev_err_probe(d,e,...) (e)
    static struct device_node *of_find_compatible_node(void*a,void*b,const char*c){return &fb;}
    static int of_property_read_u32(struct device_node*n,const char*k,u32*v){
     if(!strcmp(k,"apple,t8122-handoff")||!strcmp(k,"apple,t6031-handoff"))*v=marker_value;
     else if(!strcmp(k,"width"))*v=width;
     else if(!strcmp(k,"height"))*v=height;
     else if(!strcmp(k,"stride"))*v=stride;
     else if(!strcmp(k,"apple,notch-height"))*v=notch;
     else return -EINVAL;
     return 0;
    }
    static void of_node_put(struct device_node*n){}
    '''
    for name,src in cases:
     end=src.index('struct dcp_v14_property {')
     if 'struct dcp_v14_board {' in src:
      records=src[src.index('struct dcp_v14_board {'):end]
     else:
      # The record type is in iomfb_v14_7_board.h; the records stay in the driver.
      records=board_types[board_types.index('struct dcp_v14_board {'):board_types.index('#endif')]
      records+=src[src.index('static const struct dcp_v14_board dcp_v14_board_t6030 = {'):end]
     defines='\n'.join(l for l in src.splitlines() if l.startswith('#define DCP_V14_') and 'FIRMWARE_UUID' in l)+'\n'
     sel_start='const struct dcp_v14_board *dcp_v14_board_select(' if 'const struct dcp_v14_board *dcp_v14_board_select(' in src else 'const struct dcp_v14_board *iomfb_v14_7_board('
     sel=src[src.index(sel_start):src.index('const char *iomfb_v14_7_board_name(')]
     geo=src[src.index('static int dcp_v14_geometry('):src.index('/* An external processor\'s firmware:')]
     driver='''struct apple_dcp_v14 {struct device *dev;const struct dcp_v14_board *board;u32 fb_width,fb_height,stride,notch_rows,panel_width,panel_height;};\n'''
     probe=src[src.index('int iomfb_v14_7_probe('):]
     uuid_gate=probe[probe.index('\tif (of_property_read_string(np, "apple,firmware-uuid"'):probe.index('\tif (!iommu_get_domain_for_dev(dev))')]
     gate='static int probe_gate(struct device *dev,const struct dcp_v14_board *board){struct device_node *np=dev->of_node;const char *uuid=NULL;u32 marker=0;'+uuid_gate+'return 0;}\n'
     tests=r'''int main(void){
     struct device dev={&dcp};machine="apple,j615";
     const struct dcp_v14_board *b=iomfb_v14_7_board(&dev);
    #ifdef OLD
     assert(IS_ERR(b));puts("old J615 board selection rejects: PASS");return 0;
    #else
     assert(!IS_ERR(b) && b && b->panel_width==2880 && b->panel_height==1864);
     assert(!strcmp(b->firmware_uuid,"90F849E1-B422-367E-B389-50246F8DEC47"));
     assert(!strcmp(b->handoff,"apple,t8122-handoff") && !b->promotion);
     image_uuid=b->firmware_uuid;assert(probe_gate(&dev,b)==0);
     image_uuid="C042E95C-B9D8-3F0E-94B3-582A08AA6FDD";assert(probe_gate(&dev,b)==-ENODEV);
     image_uuid="unknown";assert(probe_gate(&dev,b)==-ENODEV);
     image_uuid=b->firmware_uuid;marker_value=0;assert(probe_gate(&dev,b)==-ENODEV);marker_value=2;assert(probe_gate(&dev,b)==-ENODEV);marker_value=1;
     assert(b->ctm_set==A(421) && b->ctm_get==A(420));
     machine="apple,j613";const struct dcp_v14_board*j=iomfb_v14_7_board(&dev);
     assert(!IS_ERR(j)&&j->panel_width==2560 && j->panel_height==1664);
     machine="apple,j700";assert(IS_ERR(iomfb_v14_7_board(&dev)));
     /* J504 (14" M3 MacBook Pro): the J613 image, the J514 panel, 60 or 120 Hz. */
     machine="apple,j504";const struct dcp_v14_board*p14=iomfb_v14_7_board(&dev);
     assert(!IS_ERR(p14)&&p14&&!strcmp(p14->name,"J504")&&!strcmp(p14->debugfs,"dcp-j504"));
     assert(p14->panel_width==3024&&p14->panel_height==1964&&p14->promotion&&p14->log_bringup);
     assert(!strcmp(p14->firmware_uuid,"90F849E1-B422-367E-B389-50246F8DEC47"));
     assert(!strcmp(p14->handoff,"apple,t8122-handoff")&&p14->panel_nits&&!p14->swap_timestamps);
     assert(p14->ctm_set==A(421)&&p14->ctm_get==A(420));
     image_uuid=p14->firmware_uuid;assert(probe_gate(&dev,p14)==0);
     image_uuid="C042E95C-B9D8-3F0E-94B3-582A08AA6FDD";assert(probe_gate(&dev,p14)==-ENODEV);
     image_uuid="F493F03E-CC40-3D89-9A08-5F259BA2B246";assert(probe_gate(&dev,p14)==-ENODEV);
     image_uuid=NULL;assert(probe_gate(&dev,p14)==-ENODEV);
     image_uuid=p14->firmware_uuid;marker_value=0;assert(probe_gate(&dev,p14)==-ENODEV);marker_value=1;
     {struct apple_dcp_v14 q={.dev=&dev,.board=p14};width=3024;height=1890;stride=width*4;notch=74;
      assert(dcp_v14_geometry(&q)==0&&q.panel_height==1964&&q.notch_rows==74);
      height=1964;notch=0;assert(dcp_v14_geometry(&q)==0&&q.panel_height==1964);
      width=2560;height=1600;stride=width*4;notch=64;assert(dcp_v14_geometry(&q)==-EINVAL);}
     /* Other T8122 Macs (J433/J434 iMacs) have no record. */
     machine="apple,j433";assert(IS_ERR(iomfb_v14_7_board(&dev)));
     machine="apple,j434";assert(IS_ERR(iomfb_v14_7_board(&dev)));
     dcp.compatible="apple,t6030-dcp";machine="apple,j516s";assert(iomfb_v14_7_board(&dev)==&dcp_v14_board_t6030);
     dcp.compatible="apple,t8122-dcp";
     struct apple_dcp_v14 v={.dev=&dev,.board=b};width=2880;height=1800;stride=width*4;notch=64;
     assert(dcp_v14_geometry(&v)==0 && v.panel_height==1864);
     height=1864;assert(dcp_v14_geometry(&v)==0 && v.panel_height==1864);
     notch=0;assert(dcp_v14_geometry(&v)==0 && v.panel_height==1864);
     height=1800;assert(dcp_v14_geometry(&v)==-EINVAL);
     notch=64;height=1800;stride++;assert(dcp_v14_geometry(&v)==-EINVAL);stride--;
     width=2560;stride=width*4;assert(dcp_v14_geometry(&v)==-EINVAL);
     width=2880;stride=width*4;notch=101;assert(dcp_v14_geometry(&v)==-EINVAL);
     /* J516C (T6031): its own DCP image, the J516S panel. */
     dcp.compatible="apple,t6031-dcp";machine="apple,j516c";
     const struct dcp_v14_board*c=iomfb_v14_7_board(&dev);
     assert(!IS_ERR(c)&&c&&!strcmp(c->name,"J516C")&&c->panel_width==3456&&c->panel_height==2234&&c->promotion);
     assert(!strcmp(c->handoff,"apple,t6031-handoff")&&!c->ctm_set&&!c->ctm_get);
     image_uuid="F493F03E-CC40-3D89-9A08-5F259BA2B246";assert(probe_gate(&dev,c)==0);
     image_uuid="DDF38191-93B3-324A-BC8F-643006F5AC82";assert(probe_gate(&dev,c)==-ENODEV);
     image_uuid="90F849E1-B422-367E-B389-50246F8DEC47";assert(probe_gate(&dev,c)==-ENODEV);
     image_uuid="F493F03E-CC40-3D89-9A08-5F259BA2B24";assert(probe_gate(&dev,c)==-ENODEV);
     image_uuid=NULL;assert(probe_gate(&dev,c)==-ENODEV);
     image_uuid="F493F03E-CC40-3D89-9A08-5F259BA2B246";marker_value=0;assert(probe_gate(&dev,c)==-ENODEV);marker_value=1;
     struct apple_dcp_v14 m={.dev=&dev,.board=c};width=3456;height=2160;stride=width*4;notch=74;
     assert(dcp_v14_geometry(&m)==0&&m.panel_height==2234&&m.notch_rows==74);
     height=2234;assert(dcp_v14_geometry(&m)==0&&m.panel_height==2234);
     height=2160;notch=0;assert(dcp_v14_geometry(&m)==-EINVAL);
     notch=74;width=3024;stride=width*4;height=1890;assert(dcp_v14_geometry(&m)==-EINVAL);
     /* J514C is now admitted explicitly; its dimensions come from simplefb. */
     machine="apple,j514c";
     const struct dcp_v14_board *c14=iomfb_v14_7_board(&dev);
     assert(!IS_ERR(c14)&&c14==&dcp_v14_board_j514c&&c14!=c);
     assert(!strcmp(c14->name,"J514C")&&!strcmp(c14->debugfs,"dcp-j514c"));
     assert(!strcmp(c14->firmware_uuid,"F493F03E-CC40-3D89-9A08-5F259BA2B246"));
     assert(!strcmp(c14->handoff,"apple,t6031-handoff")&&c14->promotion&&c14->log_bringup);
     assert(!c14->panel_width&&!c14->panel_height&&!c14->ctm_set&&!c14->ctm_get);
     image_uuid=c14->firmware_uuid;assert(probe_gate(&dev,c14)==0);
     image_uuid="90F849E1-B422-367E-B389-50246F8DEC47";assert(probe_gate(&dev,c14)==-ENODEV);
     image_uuid=NULL;assert(probe_gate(&dev,c14)==-ENODEV);
     image_uuid=c14->firmware_uuid;marker_value=0;assert(probe_gate(&dev,c14)==-ENODEV);
     marker_value=2;assert(probe_gate(&dev,c14)==-ENODEV);marker_value=1;
     struct apple_dcp_v14 m14={.dev=&dev,.board=c14};
     width=3024;height=1890;stride=width*4;notch=74;
     assert(dcp_v14_geometry(&m14)==0&&m14.panel_width==3024&&m14.panel_height==1964);
     assert(m14.notch_rows==74);
     stride++;assert(dcp_v14_geometry(&m14)==-EINVAL);stride--;
     notch=MAX_NOTCH_HEIGHT+1;assert(dcp_v14_geometry(&m14)==-EINVAL);notch=74;
     width=0;stride=0;assert(dcp_v14_geometry(&m14)==-EINVAL);
     dcp.compatible="apple,t8122-dcp";assert(IS_ERR(iomfb_v14_7_board(&dev)));
     dcp.compatible="apple,t6031-dcp";machine="apple,j514s";assert(IS_ERR(iomfb_v14_7_board(&dev)));
     machine="apple,j613";assert(IS_ERR(iomfb_v14_7_board(&dev)));
     machine="apple,j516m";assert(IS_ERR(iomfb_v14_7_board(&dev)));
     dcp.compatible="apple,t6030-dcp";machine="apple,j516s";assert(iomfb_v14_7_board(&dev)==&dcp_v14_board_t6030);
     puts("current J615/J613/J504/T6030/J516C/J514C admission + hidden/full notch, wrong geometry controls: PASS");
    #endif
    }
    '''
     p=out/(name+'-panel-control.c');p.write_text(prefix+defines+records+driver+sel+geo+gate+tests)
     cmd=['cc','-std=gnu11','-Wall','-Werror','-Wno-unused-function']
     if name=='old':cmd.append('-DOLD')
     subprocess.run(cmd+[str(p),'-o',str(out/(name+'-panel-control'))],check=True)
     subprocess.run([str(out/(name+'-panel-control'))],check=True)
