// SPDX-License-Identifier: GPL-2.0-only
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stddef.h>
#include <errno.h>
#include <pthread.h>
#include <time.h>
#include <unistd.h>
#define TBNET_RING_SIZE 256
#define TBNET_FRAME_SIZE 4096
#define TBNET_RX_PAGE_ORDER (PAGE_SIZE==4096?1:0)
#define TBNET_RX_PAGE_SIZE (PAGE_SIZE<<TBNET_RX_PAGE_ORDER)
#define MAX_SKB_FRAGS 17
#define TBNET_LOGOUT_RETRIES 3
#define TBNET_THROTTLING 5
#define TBNET_MAX_MTU 65536
#define RING_DESC_CRC_ERROR 1
#define RING_DESC_BUFFER_OVERRUN 2
#define NUMA_NO_NODE -1
#define DMA_FROM_DEVICE 2
#define DMA_TO_DEVICE 1
#define PP_FLAG_DMA_MAP 1
#define PP_FLAG_DMA_SYNC_DEV 2
#define RING_FLAG_FRAME 1
#define RING_FLAG_E2E 2
#define TBNET_E2E 1
#define TBIP_PDF_FRAME_START 1
#define TBIP_PDF_FRAME_END 2
#define BIT(n) (1U<<(n))
#define container_of(ptr,type,member) ((type*)((char*)(ptr)-offsetof(type,member)))
#define le32_to_cpu(x) (x)
#define le16_to_cpu(x) (x)
#define trace_tbnet_free_frame(...) ((void)0)
#define trace_tbnet_alloc_rx_frame(...) ((void)0)
#define trace_tbnet_invalid_rx_ip_frame(...) ((void)0)
#define trace_tbnet_rx_ip_frame(...) ((void)0)
#define trace_tbnet_rx_skb(...) ((void)0)
#define netdev_dbg(...) ((void)0)
#define netdev_err(...) ((void)0)
#define netdev_warn(...) ((void)0)
#define lockdep_assert_held(...) ((void)0)
#define TB_MAX_FRAME_SIZE 4096
#define IS_ERR(x) ((intptr_t)(x)<0)
#define PTR_ERR(x) ((intptr_t)(x))
typedef uint16_t u16;
typedef unsigned int u32;
typedef uintptr_t dma_addr_t;
typedef pthread_mutex_t spinlock_t;
struct device { void *data; };
struct work_struct { void (*fn)(struct work_struct *); };
struct delayed_work { struct work_struct work; bool pending,running; };
struct tbnet;
struct net_device { struct tbnet *priv; bool carrier,running,attached,queue_on; };
struct napi_struct { int list_owner; bool enabled,owner,disabling,missed; struct tbnet *net; };
struct page_pool { int id,cpuid; struct { struct napi_struct *napi; } p; bool detached; unsigned int alive; };
struct page { struct page_pool *pool; int owner; char *mem; dma_addr_t dma; };
struct ring_frame { dma_addr_t buffer_phy; unsigned int size,flags; struct page *unused; };
struct tb_ring { int hop; bool running,is_tx,freed,masked; void (*start_poll)(void*); void *poll_data; struct work_struct work; struct ring_frame *queue[TBNET_RING_SIZE]; unsigned int queued,completed; };
struct tbnet_frame { struct ring_frame frame; struct page *page; struct net_device *dev; };
static inline size_t tb_ring_frame_size(const struct ring_frame *frame){return frame->size?frame->size:TB_MAX_FRAME_SIZE;}
struct tbnet_ring { unsigned int cons,prod; struct tb_ring *ring; struct page_pool *pool; struct tbnet_frame frames[TBNET_RING_SIZE]; };
struct thunderbolt_ip_frame_header { u32 frame_size; u16 frame_index,frame_id; u32 frame_count; };
struct skb_shared_info { unsigned int nr_frags; };
struct sk_buff { struct page *head,*frags[MAX_SKB_FRAGS]; struct skb_shared_info sh; bool recycle,freed; int protocol; unsigned int len; };
struct tb { void *nhi; };
struct tb_xdomain { struct tb *tb; };
struct tb_service { struct tbnet *data; int prtcstns; };
struct tbnet { struct net_device *dev; struct napi_struct napi; bool napi_enabled,rings_started; spinlock_t poll_lock; pthread_mutex_t connection_lock; struct sk_buff *skb; bool login_sent,login_received; int login_retries,local_transmit_path,remote_transmit_path; struct tbnet_ring rx_ring,tx_ring; struct tb_xdomain *xd; struct tb_service *svc; struct delayed_work rx_refill_work; struct work_struct connected_work,disconnect_work; struct thunderbolt_ip_frame_header rx_hdr; struct { unsigned int rx_packets,rx_bytes,rx_errors,rx_length_errors,rx_crc_errors,rx_over_errors,rx_missed_errors; } stats; int handler; };
struct page_pool_params { unsigned int order,pool_size,max_len,flags; int nid,dma_dir; struct device *dev; struct napi_struct *napi; struct net_device *netdev; };
static pthread_mutex_t gate=PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cv=PTHREAD_COND_INITIALIZER;
static int pause_at,paused,resume_poll,teardown_entered,completion_ok=1,alloc_limit=-1,queue_limit=-1,tx_error,path_error,in_hop_error,pool_error,build_error,invalid_frame,alloc_calls,rearm_calls,complete_calls,disable_calls,enable_calls,path_off_calls,path_on_calls,ring_stops,ring_frees,tx_unmaps,pool_puts,gro_count,pool_destroy_calls;
static bool tail_pending,setup_active,pool_alloc_active;
static int schedule_calls,retry_queues,retry_delay;
static bool cancel_waiting;
static int system_dfl_wq;
#define msecs_to_jiffies(x) (x)
#define INIT_DELAYED_WORK(w,f) ((w)->work.fn=(f))
static _Thread_local int heldmutex;
static struct page *pages[4096]; static int npages,pool_id;
static struct sk_buff *gro[1024];
static struct device dma_device;
static bool tbnet_e2e=true;
#define READ_ONCE(x) (x)
#define IS_ENABLED(x) 0
#define unlikely(x) (x)
static bool fake_softirq;
static bool in_softirq(void){return fake_softirq;}
static int smp_processor_id(void){return 0;}
static void schedule_work(struct work_struct *w){}
static bool queue_delayed_work(int wq,struct delayed_work *w,unsigned long delay){assert(delay==100);pthread_mutex_lock(&gate);bool new=!w->pending;if(new){w->pending=true;retry_queues++;retry_delay=delay;}pthread_mutex_unlock(&gate);return new;}
static void cancel_delayed_work_sync(struct delayed_work *w)
{
	pthread_mutex_lock(&gate);
	cancel_waiting = true;
	pthread_cond_broadcast(&cv);
	while (w->running)
		pthread_cond_wait(&cv, &gate);
	w->pending = false;
	cancel_waiting = false;
	pthread_mutex_unlock(&gate);
}
static void fire_retry(struct delayed_work *w){pthread_mutex_lock(&gate);assert(w->pending&&!w->running);w->pending=false;w->running=true;pthread_mutex_unlock(&gate);w->work.fn(&w->work);pthread_mutex_lock(&gate);w->running=false;pthread_cond_broadcast(&cv);pthread_mutex_unlock(&gate);}

static void nhi_ring_interrupt_mask(struct tb_ring *r,bool mask){r->masked=mask;}
static struct page *find_page(void *mem){for(int i=0;i<npages;i++)if(pages[i]->mem==mem)return pages[i];abort();}
static void pause_poll(int where){pthread_mutex_lock(&gate);if(pause_at==where){paused=where;pthread_cond_broadcast(&cv);while(!resume_poll)pthread_cond_wait(&cv,&gate);}pthread_mutex_unlock(&gate);}
static void mutex_lock(pthread_mutex_t *m){assert(!pthread_mutex_lock(m));heldmutex++;}
static void mutex_unlock(pthread_mutex_t *m){assert(heldmutex>0);heldmutex--;assert(!pthread_mutex_unlock(m));}
static void spin_lock(spinlock_t *m){mutex_lock(m);}static void spin_unlock(spinlock_t *m){mutex_unlock(m);}
static void spin_lock_bh(spinlock_t *m){mutex_lock(m);}static void spin_unlock_bh(spinlock_t *m){mutex_unlock(m);}
static struct tbnet *netdev_priv(struct net_device *d){return d->priv;}
static void netif_carrier_off(struct net_device *d){d->carrier=false;}static void netif_carrier_on(struct net_device *d){d->carrier=true;}static bool netif_carrier_ok(struct net_device *d){return d->carrier;}
static void netif_stop_queue(struct net_device *d){d->queue_on=false;}static void netif_start_queue(struct net_device *d){d->queue_on=true;}
static void stop_login(struct tbnet *n){assert(heldmutex==0);pthread_mutex_lock(&gate);if(setup_active){teardown_entered=1;pthread_cond_broadcast(&cv);}while(setup_active)pthread_cond_wait(&cv,&gate);pthread_mutex_unlock(&gate);}
static void start_login(struct tbnet *n){}static void cancel_work_sync(struct work_struct *w){}
static bool netif_running(struct net_device *d){return d->running;}static void netif_device_detach(struct net_device *d){d->attached=false;}static void netif_device_attach(struct net_device *d){d->attached=true;}
static struct tb_service *tb_to_service(struct device *d){return d->data;}static struct tbnet *tb_service_get_drvdata(struct tb_service *s){return s->data;}
static void tb_unregister_protocol_handler(void *h){}static void tb_register_protocol_handler(void *h){}
static void napi_enable(struct napi_struct *n){assert(!n->enabled);n->enabled=true;enable_calls++;}
static void napi_disable(struct napi_struct *n){pthread_mutex_lock(&gate);assert(n->enabled);n->disabling=true;disable_calls++;teardown_entered=1;pthread_cond_broadcast(&cv);while(n->owner)pthread_cond_wait(&cv,&gate);n->enabled=false;n->disabling=false;pthread_mutex_unlock(&gate);}
static bool napi_complete_done(struct napi_struct *n,int rx){complete_calls++;if(!completion_ok)return false;if(n->missed){n->missed=false;return false;}pthread_mutex_lock(&gate);n->owner=false;n->list_owner=-1;tail_pending=true;pthread_cond_broadcast(&cv);pthread_mutex_unlock(&gate);pause_poll(2);return true;}
static void tb_ring_poll_complete(struct tb_ring *r){assert(!r->freed&&r->running);rearm_calls++;r->masked=false;tail_pending=false;}
static struct device *tb_ring_dma_device(struct tb_ring *r){assert(r&&!r->freed);return &dma_device;}
static struct page_pool *page_pool_create(struct page_pool_params *p){assert(p->order==TBNET_RX_PAGE_ORDER&&p->pool_size==256&&p->max_len==4096&&p->dma_dir==DMA_FROM_DEVICE&&p->flags==(PP_FLAG_DMA_MAP|PP_FLAG_DMA_SYNC_DEV)&&p->napi==&p->netdev->priv->napi);if(pool_error)return(void*)(intptr_t)-ENOMEM;struct page_pool *x=calloc(1,sizeof(*x));x->id=++pool_id;x->cpuid=-1;x->p.napi=p->napi;return x;}
static struct page *page_pool_dev_alloc_pages(struct page_pool *p){alloc_calls++;assert(p&&!p->detached&&!pool_alloc_active);if(pause_at==5&&alloc_calls==3){pool_alloc_active=true;pause_poll(5);pool_alloc_active=false;}if(alloc_limit==0)return NULL;if(alloc_limit>0)alloc_limit--;for(int i=0;i<npages;i++)if(pages[i]->pool==p&&pages[i]->owner==0){pages[i]->owner=2;return pages[i];}struct page *x=calloc(1,sizeof(*x));x->pool=p;x->owner=2;x->mem=calloc(1,TBNET_RX_PAGE_SIZE);x->dma=(uintptr_t)x->mem;pages[npages++]=x;p->alive++;return x;}
static dma_addr_t page_pool_get_dma_addr(struct page *p){return p->dma;}
static void page_pool_put_full_page(struct page_pool *p,struct page *x,bool direct){assert(x&&x->pool==p&&x->owner!=0&&x->owner!=1);x->owner=0;pool_puts++;if(p->detached)p->alive--;}
static void page_pool_destroy(struct page_pool *p){assert(p&&!p->detached);pool_destroy_calls++;p->detached=true;for(int i=0;i<npages;i++)if(pages[i]->pool==p&&pages[i]->owner==0)p->alive--;}
static void __free_page(struct page *p){assert(!p->pool);p->owner=0;}
static void dma_unmap_page(struct device *d,dma_addr_t a,unsigned int size,int dir){assert(dir==DMA_TO_DEVICE&&size==4096&&a);tx_unmaps++;}
static void dma_sync_single_for_cpu(struct device *d,dma_addr_t a,unsigned int size,int dir){assert(dir==DMA_FROM_DEVICE&&size==4096);pause_poll(1);}
static int tb_ring_rx(struct tb_ring *r,struct ring_frame *f){if(queue_limit==0)return -ESHUTDOWN;if(queue_limit>0)queue_limit--;if(!r->running)return -ESHUTDOWN;struct tbnet_frame *tf=container_of(f,struct tbnet_frame,frame);assert(tf->page->owner==2);tf->page->owner=1;r->queue[r->queued++]=f;return 0;}
static void tb_ring_start(struct tb_ring *r){assert(!r->running&&!r->freed);r->running=true;}
static void tb_ring_stop(struct tb_ring *r){assert(r->running&&!tail_pending);assert(!r->is_tx||!r->freed);r->running=false;ring_stops++;for(unsigned int i=0;i<r->queued;i++){struct tbnet_frame *f=container_of(r->queue[i],struct tbnet_frame,frame);if(f->page&&f->page->owner==1)f->page->owner=2;}r->queued=0;r->completed=0;}
static void tb_ring_free(struct tb_ring *r){assert(!r->running&&!r->freed&&!tail_pending);r->freed=true;ring_frees++;}
static struct tb_ring *tb_ring_alloc_tx(void *n,int h,int size,int f){struct tb_ring *r=calloc(1,sizeof(*r));r->is_tx=true;r->hop=1;return r;}
static struct tb_ring *tb_ring_alloc_rx(void *n,int h,int size,int f,int tx,int sof,int eof,void (*poll)(void*),void *ctx){struct tb_ring *r=calloc(1,sizeof(*r));r->hop=2;r->start_poll=poll;r->poll_data=ctx;return r;}
static void tb_ring_throttling(struct tb_ring *r,int t){}
static struct ring_frame *tb_ring_poll(struct tb_ring *r){assert(r&&!r->freed);if(!r->completed)return NULL;struct ring_frame *f=r->queue[0];memmove(r->queue,r->queue+1,--r->queued*sizeof(*r->queue));r->completed--;container_of(f,struct tbnet_frame,frame)->page->owner=2;return f;}
static int tb_xdomain_alloc_out_hopid(struct tb_xdomain *x,int i){return 8;}static void tb_xdomain_release_out_hopid(struct tb_xdomain *x,int i){}
static int tb_xdomain_alloc_in_hopid(struct tb_xdomain *x,int i){return in_hop_error?-ENOMEM:i;}static void tb_xdomain_release_in_hopid(struct tb_xdomain *x,int i){}
static int tb_xdomain_disable_paths(struct tb_xdomain *x,int a,int b,int c,int d){path_off_calls++;return 0;}
static int tb_xdomain_enable_paths(struct tb_xdomain *x,int a,int b,int c,int d){path_on_calls++;pause_poll(4);return path_error;}
static int tbnet_logout_request(struct tbnet *n){return 0;}
static int tbnet_alloc_tx_buffers(struct tbnet *n){return tx_error;}
static void *page_address(struct page *p){assert(p&&p->owner==2);return p->mem;}
static struct sk_buff *build_skb(void *mem,unsigned int size){assert(size==TBNET_RX_PAGE_SIZE);if(build_error)return NULL;struct sk_buff *s=calloc(1,sizeof(*s));s->head=find_page(mem);s->head->owner=3;return s;}
static void skb_mark_for_recycle(struct sk_buff *s){s->recycle=true;}static void skb_reserve(struct sk_buff *s,int n){}static void skb_put(struct sk_buff *s,int n){s->len+=n;}
static struct skb_shared_info *skb_shinfo(struct sk_buff *s){return &s->sh;}
static void skb_add_rx_frag(struct sk_buff *s,int n,struct page *p,int off,int len,int total){pause_poll(3);assert(!s->freed&&s->recycle&&p->owner==2&&n<MAX_SKB_FRAGS);s->frags[n]=p;p->owner=3;s->sh.nr_frags++;s->len+=len;}
static int eth_type_trans(struct sk_buff *s,struct net_device *d){return 0;}
static void dev_kfree_skb_any(struct sk_buff *s){if(!s)return;assert(!s->freed&&s->recycle);s->freed=true;page_pool_put_full_page(s->head->pool,s->head,false);for(unsigned int i=0;i<s->sh.nr_frags;i++)page_pool_put_full_page(s->frags[i]->pool,s->frags[i],false);/* A retained wrapper records whether a second release was attempted. */}
static void napi_gro_receive(struct napi_struct *n,struct sk_buff *s){gro[gro_count++]=s;}
static void napi_schedule(struct napi_struct *n){pause_poll(6);pthread_mutex_lock(&gate);if(n->enabled&&!n->disabling){schedule_calls++;if(n->owner)n->missed=true;n->owner=true;n->list_owner=0;}pthread_mutex_unlock(&gate);}
