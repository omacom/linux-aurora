#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Production native property builder with actual OF mutation and inversion bodies."""
from pathlib import Path
import re,subprocess,tempfile,os
R=Path(__file__).resolve().parents[2]
def read(n):
 p=R/n
 return p.read_text() if p.exists() else subprocess.check_output(['git','show','HEAD:'+n],cwd=R,text=True)
def fn(s,n):
 m=re.search(r'^(?:static )?(?:int|bool|void|struct property \*)\s*(?:__init )?'+n+r'\([^;]*?\)\s*\{',s,re.M|re.S);assert m,n
 i=m.end();depth=1
 while depth:depth+=(s[i]=='{')-(s[i]=='}');i+=1
 return s[m.start():i]
a=read('drivers/gpu/drm/apple/j613_dcp_activate.c');base=read('drivers/of/base.c');dyn=read('drivers/of/dynamic.c')
pre=r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#define __init
#define GFP_KERNEL 0
#define kzalloc_obj(x) zalloc(sizeof(x))
#define raw_spin_lock_irqsave(l,f) ((f)=0)
#define raw_spin_unlock_irqrestore(l,f) ((void)(f))
#define of_prop_cmp strcmp
#define __of_remove_property_sysfs(n,p) ((void)0)
#define __of_update_property_sysfs(n,p,o) ((void)0)
struct property { char *name; void *value; unsigned length; struct property *next; };
struct device_node { struct property *properties,*deadprops; };
struct of_changeset {int unused;};static struct of_changeset external_gate;
enum {OF_RECONFIG_ATTACH_NODE,OF_RECONFIG_DETACH_NODE,OF_RECONFIG_ADD_PROPERTY,OF_RECONFIG_REMOVE_PROPERTY,OF_RECONFIG_UPDATE_PROPERTY};
struct of_changeset_entry {int action;struct device_node *np;struct property *prop,*old_prop;};
static struct of_changeset_entry queued;static int queue_error,alloc_fail,alloc_calls,allocations;
static void *allocate(size_t n) { if (++alloc_calls==alloc_fail)return NULL;void *p=malloc(n);assert(p);allocations++;return p; }
static void *zalloc(size_t n){void *p=allocate(n);if(p)memset(p,0,n);return p;}
static char *kstrdup(const char *s,int f){(void)f;char *p=allocate(strlen(s)+1);if(p)strcpy(p,s);return p;}
static void *kmemdup(const void *s,size_t n,int f){(void)f;void *p=allocate(n);if(p)memcpy(p,s,n);return p;}
static void kfree(void *p){if(p){allocations--;free(p);}}
static int of_changeset_update_property(struct of_changeset *c,struct device_node *n,struct property *p){(void)c;if(queue_error)return queue_error;queued=(struct of_changeset_entry){OF_RECONFIG_UPDATE_PROPERTY,n,p,NULL};return 0;}
'''
code=pre+'\n'.join([fn(base,'__of_remove_property_from_list'),fn(base,'__of_update_property'),fn(base,'__of_remove_property'),fn(dyn,'__of_changeset_entry_invert'),fn(a,'j613_external_update')])+r'''
static void apply(void){assert(queued.action==OF_RECONFIG_UPDATE_PROPERTY);assert(!__of_update_property(queued.np,queued.prop,&queued.old_prop));}
static void revert(void){struct of_changeset_entry e;__of_changeset_entry_invert(&queued,&e);if(e.action==OF_RECONFIG_REMOVE_PROPERTY)assert(!__of_remove_property(e.np,e.prop));else{assert(e.action==OF_RECONFIG_UPDATE_PROPERTY);assert(!__of_update_property(e.np,e.prop,&e.old_prop));}}
int main(void){
 unsigned char oldbytes[4]={0,0,0,4},newbytes[4]={0,0,0,15};struct property old={.name="apple,min-state",.value=oldbytes,.length=4};struct device_node n={.properties=&old};
 assert(!j613_external_update(&n,"apple,min-state",newbytes,4));assert(n.properties==&old);apply();assert(n.properties->length==4&&!memcmp(n.properties->value,newbytes,4));revert();assert(n.properties==&old&&!memcmp(old.value,oldbytes,4));
 n=(struct device_node){0};assert(!j613_external_update(&n,"apple,min-state",newbytes,4));apply();revert();assert(!n.properties);
 n=(struct device_node){0};assert(!j613_external_update(&n,"apple,always-on",NULL,0));apply();assert(n.properties&&!n.properties->length);revert();assert(!n.properties);
 struct property boolean={.name="apple,always-on"};n=(struct device_node){.properties=&boolean};assert(!j613_external_update(&n,"apple,always-on",NULL,0));apply();revert();assert(n.properties==&boolean);
 old.value=newbytes;n=(struct device_node){.properties=&old};assert(!j613_external_update(&n,"apple,min-state",newbytes,4));apply();revert();assert(n.properties==&old&&!memcmp(n.properties->value,newbytes,4));
 for(int i=1;i<=3;i++){alloc_calls=0;alloc_fail=i;int before=allocations;assert(j613_external_update(&n,"apple,min-state",newbytes,4)==-ENOMEM);assert(allocations==before&&n.properties==&old);}
 alloc_fail=0;queue_error=-EIO;int before=allocations;assert(j613_external_update(&n,"apple,min-state",newbytes,4)==-EIO);assert(allocations==before&&n.properties==&old);
 puts("PASS actual native UPDATE builder and Linux OF apply/revert: existing/absent/equal u32, existing/absent bool, allocation/queue failures");
}
'''
with tempfile.TemporaryDirectory() as t:
 p=Path(t);(p/'test.c').write_text(code);subprocess.run([os.environ.get('CC','clang'),'-std=gnu11','-Wall','-Wextra','-Werror','-fsanitize=address,undefined',str(p/'test.c'),'-o',str(p/'test')],check=True);subprocess.run([str(p/'test')],check=True,env={**os.environ,'ASAN_OPTIONS':'detect_leaks=0'})
