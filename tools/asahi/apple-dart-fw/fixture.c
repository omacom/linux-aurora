#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stddef.h>
#include <stdio.h>
#include <errno.h>
typedef uint8_t u8;typedef uint32_t u32;typedef uint64_t u64;typedef uint64_t phys_addr_t;typedef uint64_t dma_addr_t;
#define BIT_ULL(n) (1ULL<<(n))
#define GENMASK_ULL(h,l) ((~0ULL>>(63-(h)))&(~0ULL<<(l)))
#define SZ_16K 16384
#define IOMMU_READ 1
#define IOMMU_WRITE 2
#define IOMMU_CACHE 4
#define APPLE_DART_PTE_VALID 1
#define APPLE_DART2 2
#define GFP_KERNEL 0
#define MEMREMAP_WB 0
#define READ_ONCE(x) (x)
#define WRITE_ONCE(x,v) ((x)=(v))
#define dma_wmb() ((void)0)
#define IS_ALIGNED(x,a) (((x)&((a)-1))==0)
#define PHYS_PFN(x) ((x)>>14)
#define ilog2(x) (63-__builtin_clzll(x))
#define check_mul_overflow(a,b,c) __builtin_mul_overflow(a,b,c)
#define check_add_overflow(a,b,c) __builtin_add_overflow(a,b,c)
#define ERR_PTR(x) ((void*)(intptr_t)(x))
#define IS_ERR(x) ((uintptr_t)(x)>=(uintptr_t)-4095)
#define PTR_ERR(x) ((intptr_t)(x))
struct apple_dart_stream_map;struct iommu_domain;
struct apple_dart_hw {int fmt;unsigned tcr_4level,ttbr_count;int(*invalidate_tlb)(struct apple_dart_stream_map*);};
#define DART_MAX_TTBR 4
#define DART_MAX_ROOT_ENTRIES 2048
#define U64_MAX UINT64_MAX
#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
static unsigned lock_depth;
#define spin_lock_irqsave(l,f) (assert(!lock_depth),lock_depth++)
#define spin_unlock_irqrestore(l,f) (assert(lock_depth==1),lock_depth--)
#define BIT_WORD(n) ((n)/64)
#define BIT_MASK(n) (1UL<<((n)%64))
#define test_bit(n,p) (!!((p)[BIT_WORD(n)]&BIT_MASK(n)))
#define atomic_long_read(p) (*(p))
#define DECLARE_BITMAP(n,b) unsigned long n[((b)+63)/64]
#define bitmap_zero(p,n) memset(p,0,(((n)+63)/64)*sizeof(unsigned long))
#define __set_bit(n,p) ((p)[BIT_WORD(n)]|=BIT_MASK(n))
#define __clear_bit(n,p) ((p)[BIT_WORD(n)]&=~BIT_MASK(n))
#define for_each_set_bit(n,p,b) for((n)=0;(n)<(b);(n)++) if(test_bit(n,p))
static size_t find_next_bit(unsigned long*p,size_t n,size_t a){for(;a<n;a++)if(test_bit(a,p))break;return a;}
static size_t find_next_zero_bit(unsigned long*p,size_t n,size_t a){for(;a<n;a++)if(!test_bit(a,p))break;return a;}
struct apple_dart_hw;struct device;
struct apple_dart {struct device*dev;unsigned pgsize,oas,fw_mirror,tcr,fw_handoff,ias,locked,num_streams,version;struct apple_dart_hw*hw;u64 dma_offset,locked_window[2];u64 *locked_ttbr[2][4],*locked_owned[2][4];struct apple_dart_fw_root*locked_fw[2][4];int lock,locked_retire_err[2];};
struct apple_dart_stream_map {struct apple_dart*dart;unsigned long sidmap[1];};
struct apple_dart_atomic_stream_map {struct apple_dart*dart;unsigned long sidmap[1];};
struct apple_dart_domain {struct apple_dart_atomic_stream_map stream_maps[2];unsigned long fw_sidmap[2][1];void*pgtbl_ops;};
struct apple_dart_master_cfg {struct apple_dart_stream_map stream_maps[2];};
#define for_each_stream_map(i,d,m) for((i)=0;(i)<2 && ((m)=&(d)->stream_maps[i])->dart;(i)++)
#define DART_T8110_TLB_CMD_OP_FLUSH_SID 1
static int commands,command_error;static u64 command_start[8],command_end[8];
static int apple_dart_t8110_hw_tlb_command_range(struct apple_dart_stream_map*m,int op,bool range,u64 lo,u64 hi){assert(op==1&&range&&commands<8);command_start[commands]=lo;command_end[commands++]=hi;return command_error;}
struct list_head {int count;};
struct iommu_resv_region {u64 start,length,dva;int prot,type;struct list_head list;};
struct device_node {int count;u32 value;bool t8140;};
struct iommu_device;struct device {struct device_node*of_node;struct apple_dart_master_cfg*cfg;struct iommu_device*iommu;};
static int of_device_is_compatible(struct device_node*np,const char*s){assert(!strcmp(s,"apple,t8140-dart"));return np->t8140;}
static int of_property_count_u32_elems(struct device_node*np,const char*s){assert(!strcmp(s,"apple,firmware-root-handoff"));return np->count;}
static int of_property_read_u32(struct device_node*np,const char*s,u32*v){assert(!strcmp(s,"apple,firmware-root-handoff"));*v=np->value;return np->count<1?-EINVAL:0;}
#define dev_iommu_priv_get(d) ((d)->cfg)
#define IOMMU_RESV_RESERVED 0
static struct iommu_resv_region regions[8];static int region_count,release_on_alloc;static struct apple_dart*retiring_dart;static u64*retired_table;
static struct iommu_resv_region*iommu_alloc_resv_region(u64 s,u64 l,int prot,int type,int g){assert(!lock_depth&&region_count<8&&type==IOMMU_RESV_RESERVED&&!prot);if(release_on_alloc){release_on_alloc=0;retiring_dart->locked_ttbr[0][0]=NULL;retiring_dart->locked_owned[0][0]=NULL;free(retired_table);retired_table=NULL;}struct iommu_resv_region*r=&regions[region_count++];r->start=s;r->length=l;r->type=type;return r;}
static void list_add_tail(struct list_head*l,struct list_head*h){h->count++;}
struct apple_dart_fw_root {u64*entry;u64*accepted;u64**leaf;};
#define DART_TCR(d,s) (s)
static unsigned apple_dart_readl(struct apple_dart*d,unsigned reg){return d->tcr;}
static u64 firmware_leaf[2048], ours_leaf[2048], root[2048];
static int ram_valid=1,reserved=1,map_failure,allocation_failure,change_root,map_calls,unmap_calls;
static int pfn_valid(unsigned long pfn){return ram_valid&&pfn>=4&&pfn<8;}
static unsigned long pfn_to_page(unsigned long pfn){return pfn;}
#define PageReserved(p) (reserved)
static void*allocation(size_t n){if(allocation_failure)return NULL;return calloc(1,n);}
#define kzalloc(n,g) allocation(n)
#define kcalloc(n,z,g) allocation((n)*(z))
#define kfree(p) free(p)
static void*memremap(u64 phys,size_t n,int type){assert(phys==0x14000&&n==SZ_16K);map_calls++;if(change_root)root[0]^=0x400;if(map_failure)return NULL;return firmware_leaf;}
static void memunmap(void*p){assert(p==firmware_leaf);unmap_calls++;}
static void*phys_to_virt(u64 phys){assert(phys==0x4000);return ours_leaf;}
static u64 compare(u64*p,u64 old,u64 next){u64 before=*p;if(before==old)*p=next;return before;}
#define cmpxchg(p,o,n) compare(p,o,n)
#define BITS_TO_LONGS(n) (((n)+63)/64)
#define bitmap_empty(p,n) (!((p)[0]&((n)>=64?~0UL:(1UL<<(n))-1)))
#define devm_memunmap(d,p) ((void)(p))
#define devm_kfree(d,p) ((void)(p))
#define to_dart_domain(d) ((struct apple_dart_domain*)(void*)(d))
static int pm_gets,pm_puts,errors_logged,pgtable_frees;
static int pm_runtime_resume_and_get(struct device*d){pm_gets++;return 0;}
static void pm_runtime_put(struct device*d){pm_puts++;}
#define dev_err_ratelimited(d,...) (errors_logged++,fprintf(stderr,__VA_ARGS__))
static void free_io_pgtable_ops(void*ops){assert(ops);pgtable_frees++;}
/* PRODUCTION */
#define IOMMU_RESV_DIRECT 1
#define IOMMU_RESV_DIRECT_RELAXABLE 2
#define IOMMU_RESV_TRANSLATED 3
#define LIST_HEAD(n) struct list_head n={0}
#define list_for_each_entry(e,h,m) for(int cursor=0;cursor<(h)->count&&((e)=&regions[cursor],1);cursor++)
#define WARN_ON_ONCE(x) (x)
#define ALIGN(x,a) (((x)+(a)-1)&~((a)-1))
#define __ffs(x) __builtin_ctzll(x)
struct iommu_domain {u64 pgsize_bitmap;struct apple_dart_domain*dart;};
struct iommu_device {int require_direct,require_translated;};
static int core_maps;
static bool iommu_is_dma_domain(struct iommu_domain*d){return true;}
static void iommu_get_resv_regions(struct device*d,struct list_head*h){h->count=region_count;}
static void iommu_put_resv_regions(struct device*d,struct list_head*h){}
static phys_addr_t iommu_iova_to_phys(struct iommu_domain*d,u64 i){return apple_dart_inherited_phys(d->dart,i);}
static int iommu_map(struct iommu_domain*d,u64 i,u64 p,size_t sz,int prot,int g){core_maps++;return apple_dart_check_fw_map(d->dart,i,p,sz,prot);}
/* CORE */
static u64 leaf(u64 phys){return GENMASK_ULL(51,40)|((phys>>4)&GENMASK_ULL(37,10))|1;}
int main(int argc,char**argv){
 assert(argc==2);const char*c=argv[1];struct apple_dart_hw hw={APPLE_DART2,1,1};struct apple_dart d={.pgsize=SZ_16K,.oas=42,.fw_mirror=0,.fw_handoff=1,.ias=42,.locked=1,.num_streams=1,.hw=&hw};
 u64 live[2]={0x1401,0},owned[2]={0,0},ours[2]={0x401,0};u64 entries[2]={0x1401,0},accepted[2]={0,0};u64*leaves[2]={firmware_leaf,NULL};struct apple_dart_fw_root fw={entries,accepted,leaves};
 firmware_leaf[0]=leaf(0x8000);ours_leaf[0]=leaf(0x8000);phys_addr_t phys;
 struct apple_dart_domain domain={0};domain.stream_maps[0].dart=&d;domain.fw_sidmap[0][0]=1;
 struct apple_dart_master_cfg cfg={0};cfg.stream_maps[0].dart=&d;cfg.stream_maps[0].sidmap[0]=1;
 struct iommu_device io={0};struct device_node node={1,1,true};struct device dev={&node,&cfg,&io};d.dev=&dev;struct list_head head={0};
 d.locked_ttbr[0][0]=root;static u64 shadow[2048];d.locked_owned[0][0]=shadow;d.locked_fw[0][0]=&fw;
 if(!strncmp(c,"legacy-",7)){
  d.fw_handoff=0;d.fw_mirror=1;root[0]=0x1401;
  if(!strcmp(c,"legacy-map-match"))
   assert(!apple_dart_check_fw_map(&domain,0,0x8000,SZ_16K,IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE));
  else if(!strcmp(c,"legacy-map-mismatch"))
   assert(apple_dart_check_fw_map(&domain,0,0xc000,SZ_16K,IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE)==-EBUSY);
  else if(!strcmp(c,"legacy-lookup"))assert(apple_dart_inherited_phys(&domain,0)==0x8000);
  else if(!strcmp(c,"legacy-snapshot")){
   reserved=0;struct apple_dart_fw_root*f=apple_dart_snapshot_fw_root(&d,0,root);
   assert(f&&!IS_ERR(f)&&f->entry[0]==0x1401);apple_dart_free_fw_root(&d,f);
  }else abort();
 }
 else if(!strncmp(c,"handoff-",8)){
  if(!strcmp(c,"handoff-missing"))node.count=-EINVAL;
  if(!strcmp(c,"handoff-duplicate"))node.count=2;
  if(!strcmp(c,"handoff-zero"))node.value=0;
  if(!strcmp(c,"handoff-unlocked"))d.locked=0;
  if(!strcmp(c,"handoff-other-soc"))node.t8140=false;
  assert(apple_dart_fw_handoff_enabled(&d)==!strcmp(c,"handoff-matched"));
 }
 else if(!strncmp(c,"core-",5)){
  root[0]=0x1401;struct iommu_domain io_domain={.pgsize_bitmap=SZ_16K,.dart=&domain};
  regions[0]=(struct iommu_resv_region){.start=0,.length=1ULL<<25,.type=IOMMU_RESV_RESERVED};
  regions[1]=(struct iommu_resv_region){.start=0x8000,.dva=0,.length=SZ_16K,.type=IOMMU_RESV_TRANSLATED};region_count=2;
  if(!strcmp(c,"core-hole"))firmware_leaf[0]=0;
  if(!strcmp(c,"core-changed-leaf"))firmware_leaf[0]=leaf(0xc000);
  /* A second DART of the domain whose root slot is vacant cannot inherit
   * the region, so the core maps it and the first DART accepts the copy. */
  static u64 vacant_root[2048],vacant_shadow[2048];struct apple_dart d2=d;
  if(!strcmp(c,"core-second-dart")){
   d2.locked_ttbr[0][0]=vacant_root;d2.locked_owned[0][0]=vacant_shadow;d2.locked_fw[0][0]=NULL;
   domain.stream_maps[1].dart=&d2;domain.fw_sidmap[1][0]=1;regions[1].prot=IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE;
  }
  int result=iommu_create_device_fw_mappings(&io_domain,&dev);
  assert(io.require_translated&&result==(!strcmp(c,"core-hole")?-EBUSY:0));
  assert(core_maps==(!strcmp(c,"core-hole")||!strcmp(c,"core-second-dart")?1:0));
 }
 else if(!strncmp(c,"range-",6)){
  u64 start=0,length=0;bool four=!strncmp(c,"range-four",10);u64 window=1ULL<<40;size_t slot=four?16:1;
  if(!strcmp(c,"range-offset"))d.dma_offset=1ULL<<40;
  if(!strcmp(c,"range-width"))slot=64;
  if(!strcmp(c,"range-four-width"))slot=64;
  if(!strcmp(c,"range-three-last"))slot=2047;
  if(!strcmp(c,"range-overflow"))window=UINT64_MAX;
  if(!strcmp(c,"range-underflow"))d.dma_offset=2ULL<<40;
  if(!strcmp(c,"range-four-last"))slot=63;
  bool ok=apple_dart_fw_slot_range(&d,window,slot,four,&start,&length);
  if(!strcmp(c,"range-four-width"))assert(!ok);
  else if(!strcmp(c,"range-width")){/* Three-level last slot still within42bits. */assert(ok);}
  else if(!strcmp(c,"range-overflow")||!strcmp(c,"range-underflow"))assert(!ok);
  else {assert(ok);assert(length==(four?1ULL<<36:1ULL<<25));assert(start==(four?(u64)slot<<36:(1ULL<<40)+((u64)slot<<25)-d.dma_offset));}
 }
 else if(!strncmp(c,"map-",4)){
  root[0]=0x1401;u64 addr=0;
  u64 paddr=0x8000;int prot=IOMMU_READ|IOMMU_WRITE|IOMMU_CACHE;
  if(!strcmp(c,"map-host")){addr=1ULL<<25;root[1]=0;}
  if(!strcmp(c,"map-owned"))shadow[0]=root[0];
  if(!strcmp(c,"map-unmatched"))d.fw_handoff=0;
  if(!strcmp(c,"map-attached")){domain.fw_sidmap[0][0]=0;domain.stream_maps[0].sidmap[0]=1;}
  /* Only a page the firmware leaf already maps the same way is repeated. */
  if(!strcmp(c,"map-hole"))firmware_leaf[0]=0;
  if(!strcmp(c,"map-wrong-physical"))paddr=0xc000;
  if(!strcmp(c,"map-wrong-permission"))prot&=~IOMMU_CACHE;
  if(!strcmp(c,"map-changed-root"))root[0]=0x1801;
  if(!strcmp(c,"map-four"))d.tcr=1;
  int expected=!strcmp(c,"map-host")||!strcmp(c,"map-owned")||!strcmp(c,"map-unmatched")||
   !strcmp(c,"map-firmware")||!strcmp(c,"map-attached")?0:-EBUSY;
  assert(apple_dart_check_fw_map(&domain,addr,paddr,SZ_16K,prot)==expected);
 }
 else if(!strncmp(c,"lookup-",7)){
  root[0]=0x1401;assert(apple_dart_inherited_phys(&domain,0)==0x8000);
  if(!strcmp(c,"lookup-changed-leaf")){firmware_leaf[0]=leaf(0xc000);assert(apple_dart_inherited_phys(&domain,0)==0xc000);}
  else if(!strcmp(c,"lookup-changed-root")){root[0]=0x1801;assert(!apple_dart_inherited_phys(&domain,0));}
  else if(!strcmp(c,"lookup-unmatched")){d.fw_handoff=0;assert(!apple_dart_inherited_phys(&domain,0));}
  else assert(!strcmp(c,"lookup-inherited"));
 }
 else if(!strncmp(c,"reserve-",8)){
  root[0]=0x1401;d.locked_window[0]=1ULL<<40;
  if(!strcmp(c,"reserve-teardown")){retired_table=calloc(2048,sizeof(u64));assert(retired_table);retired_table[0]=retired_table[5]=0x1401;d.locked_ttbr[0][0]=retired_table;release_on_alloc=1;retiring_dart=&d;}
  if(!strcmp(c,"reserve-unmatched"))d.fw_handoff=0;
  if(!strcmp(c,"reserve-owned"))shadow[0]=root[0];
  apple_dart_get_fw_resv_regions(&dev,&head);
  if(!strcmp(c,"reserve-unmatched")||!strcmp(c,"reserve-owned"))assert(!head.count);
  else if(!strcmp(c,"reserve-teardown")){assert(head.count==2&&regions[1].start==(1ULL<<40)+(5ULL<<25)&&!retired_table&&!release_on_alloc);}
  else {assert(head.count==1&&regions[0].start==(1ULL<<40)&&regions[0].length==(1ULL<<25)&&regions[0].type==IOMMU_RESV_RESERVED);}
 }
 else if(!strncmp(c,"invalidate-",11)){
  root[0]=0x1401;d.locked_window[0]=1ULL<<40;
  bool four=!strcmp(c,"invalidate-four");d.tcr=four;u64 span=four?1ULL<<36:1ULL<<25;
  if(!strcmp(c,"invalidate-changed-root"))root[0]=0x1801;
  struct apple_dart_stream_map stream={.dart=&d,.sidmap={1}};
  u64 base=four?0:1ULL<<40;
  if(!strcmp(c,"invalidate-error"))command_error=-ETIMEDOUT;
  assert(apple_dart_fw_invalidate(&stream,base,base+2*span-1)==(command_error?-ETIMEDOUT:0));
  assert(commands==1&&command_start[0]==base+span&&command_end[0]==base+2*span-1);
 }
 else if(!strncmp(c,"retire-",7)){
  /* A TLB timeout while retiring a root keeps the domain's page tables,
   * whether the domain free, a consumer release or an attach rollback
   * retired it first. */
  root[0]=0x1401;root[1]=shadow[1]=0x801;d.locked_window[0]=1ULL<<40;d.locked_fw[0][0]=NULL;d.version=0x0202;hw.invalidate_tlb=apple_dart_t8110_hw_invalidate_tlb;
  struct apple_dart_domain*dom=calloc(1,sizeof(*dom));assert(dom);dom->stream_maps[0].dart=&d;dom->stream_maps[0].sidmap[0]=1;dom->pgtbl_ops=&d;
  bool timeout=!!strstr(c,"timeout");if(timeout)command_error=-ETIMEDOUT;
  if(!strncmp(c,"retire-release",14)){
   struct apple_dart_master_cfg*owner=calloc(1,sizeof(*owner));assert(owner);*owner=cfg;dev.cfg=owner;
   apple_dart_release_device(&dev);assert(pm_gets==1&&pm_puts==1);
  }else if(!strcmp(c,"retire-rollback-timeout")){
   struct apple_dart_stream_map stream={.dart=&d,.sidmap={1}};
   assert(apple_dart_hw_unmap_locked_ttbr(&stream,0)==-ETIMEDOUT);
  }else assert(!strcmp(c,"retire-free")||!strcmp(c,"retire-free-timeout"));
  if(strncmp(c,"retire-free",11)){
   /* Retired first: the free sees no root left and issues no command. */
   assert(!d.locked_ttbr[0][0]&&commands==1&&d.locked_retire_err[0]==(timeout?-ETIMEDOUT:0));
   command_error=0;
  }
  apple_dart_domain_free((struct iommu_domain*)dom);
  assert(!d.locked_ttbr[0][0]&&root[0]==0x1401&&!root[1]&&!shadow[1]);
  assert(commands==1&&command_start[0]==(1ULL<<40)+(1ULL<<25)&&command_end[0]==(1ULL<<40)+(1ULL<<36)-1);
  assert(d.locked_retire_err[0]==(timeout?-ETIMEDOUT:0));
  assert(pgtable_frees==!timeout&&errors_logged==timeout);
 }
 else if(!strcmp(c,"mirror")){assert(!apple_dart_publish_root(&d,live,owned,ours,&fw,2));assert(live[0]==0x1401&&owned[0]==0);}
 else if(!strcmp(c,"changed-leaf")){assert(!apple_dart_publish_root(&d,live,owned,ours,&fw,2));firmware_leaf[0]=leaf(0xc000);assert(apple_dart_publish_root(&d,live,owned,ours,&fw,2)==-EBUSY);}
 else if(!strcmp(c,"wrong-permission")){firmware_leaf[0]|=4;assert(apple_dart_publish_root(&d,live,owned,ours,&fw,2)==-EBUSY);}
 else if(!strcmp(c,"wrong-physical")){firmware_leaf[0]=leaf(0xc000);assert(apple_dart_publish_root(&d,live,owned,ours,&fw,2)==-EBUSY);}
 else if(!strcmp(c,"malformed-leaf")){firmware_leaf[0]|=BIT_ULL(60);assert(apple_dart_publish_root(&d,live,owned,ours,&fw,2)==-EBUSY);}
 else if(!strcmp(c,"vacant")){ours[0]=0;ours[1]=0x801;assert(!apple_dart_publish_root(&d,live,owned,ours,&fw,2));assert(live[0]==0x1401&&live[1]==0x801&&owned[1]==0x801);}
 else if(!strcmp(c,"changed-owner")){live[0]=0x1801;assert(apple_dart_publish_root(&d,live,owned,ours,&fw,2)==-EBUSY);assert(!owned[0]);}
 else if(!strcmp(c,"retire")){owned[1]=0x801;live[1]=0xc01;apple_dart_retire_root(live,owned,2);assert(live[0]==0x1401&&live[1]==0xc01&&!owned[1]);}
 else if(!strcmp(c,"reserved-ram")){assert(apple_dart_fw_table_ram(&d,0x14000));}
 else if(!strcmp(c,"unreserved")){reserved=0;assert(!apple_dart_fw_table_ram(&d,0x14000));}
 else if(!strcmp(c,"mmio")){ram_valid=0;assert(!apple_dart_fw_table_ram(&d,0x14000));}
 else if(!strcmp(c,"misalignment")){assert(!apple_dart_fw_table_ram(&d,0x14001));}
 else if(!strcmp(c,"address-width")){d.oas=16;assert(!apple_dart_fw_table_ram(&d,0x14000));}
 else if(!strcmp(c,"root-encoding")){assert(!apple_dart_fw_table_decode(&d,0x1403,&phys));}
 else if(!strcmp(c,"leaf-as-table")){assert(!apple_dart_fw_table_decode(&d,leaf(0x14000),&phys));}
 else {
  root[0]=0x1401;
  if(!strcmp(c,"snapshot-unmatched"))d.fw_handoff=0;
  else if(!strcmp(c,"snapshot-four"))d.tcr=1;
  else if(!strcmp(c,"snapshot-malformed"))root[0]|=2;
  else if(!strcmp(c,"snapshot-unreserved"))reserved=0;
  else if(!strcmp(c,"snapshot-root-change"))change_root=1;
  else if(!strcmp(c,"snapshot-map-failure"))map_failure=1;
  else if(!strcmp(c,"snapshot-allocation-failure"))allocation_failure=1;
  else assert(!strcmp(c,"snapshot"));
  struct apple_dart_fw_root*f=apple_dart_snapshot_fw_root(&d,0,root);
  if(!strcmp(c,"snapshot-unmatched")||!strcmp(c,"snapshot-four")){assert(!f&&!map_calls);}
  else if(!strcmp(c,"snapshot")){assert(f&&!IS_ERR(f)&&f->entry[0]==0x1401);apple_dart_free_fw_root(&d,f);assert(unmap_calls==1);}
  else {assert(IS_ERR(f));int expected=allocation_failure||map_failure?-ENOMEM:change_root?-EBUSY:-EINVAL;assert(PTR_ERR(f)==expected);assert(map_calls==(change_root||map_failure));assert(unmap_calls==change_root);}
 }
 printf("PASS %s\n",c);return 0;
}
