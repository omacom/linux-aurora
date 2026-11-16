#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only
"""Exercise Thunderbolt network refill recovery and NAPI teardown."""
from pathlib import Path
import argparse, hashlib, json, subprocess, re, tempfile, sys
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--ref', default='WORKTREE', help='Git source revision')
parser.add_argument('--out', type=Path, help='retain controls and receipt')
parser.add_argument('--case', action='append', help='run selected control')
parser.add_argument('--mutant', choices=['refill-count', 'no-retry', 'no-cancel', 'worker-allocation'])
args = parser.parse_args()
temporary = tempfile.TemporaryDirectory() if args.out is None else None
out = args.out or Path(temporary.name)
out.mkdir(parents=True, exist_ok=True)
wt = Path(__file__).resolve().parents[2]
ref = args.ref
source=(wt/'drivers/net/thunderbolt/main.c').read_text() if ref=='WORKTREE' else subprocess.check_output(['git','show',ref+':drivers/net/thunderbolt/main.c'],cwd=wt,text=True)
names=['tbnet_rx_refill_work','tbnet_free_buffers','tbnet_disable_napi','tbnet_tear_down','tbnet_available_buffers','tbnet_alloc_rx_buffers','tbnet_connect_failed_locked','tbnet_connected_work','tbnet_check_frame','tbnet_poll','tbnet_start_poll','tbnet_open','tbnet_stop','tbnet_shutdown','tbnet_suspend','tbnet_resume']
def extract(name,text=source):
 m=re.search(r'^static[^;\n]*\b'+name+r'\([^;]*?\n\{',text,re.M)
 if not m:
  assert name in ('tbnet_rx_refill_work','tbnet_connect_failed_locked');return ''
 a=m.start();b=m.end();n=1
 # Functions contain no braces in strings; avoid comment braces here.
 while n:
  if text[b]=='{':n+=1
  if text[b]=='}':n-=1
  b+=1
 return text[a:b]
nhi=subprocess.check_output(['git','show',ref+':drivers/thunderbolt/nhi.c'],cwd=wt,text=True) if ref!='WORKTREE' else (wt/'drivers/thunderbolt/nhi.c').read_text();core=subprocess.check_output(['git','show',ref+':net/core/page_pool.c'],cwd=wt,text=True) if ref!='WORKTREE' else (wt/'net/core/page_pool.c').read_text()
bodies={'__ring_interrupt':extract('__ring_interrupt',nhi),'page_pool_napi_local':extract('page_pool_napi_local',core)}
bodies.update({n:extract(n) for n in names})
original_bodies = dict(bodies)
if args.mutant == 'refill-count':
 bodies['tbnet_poll'] = bodies['tbnet_poll'].replace('cleaned_count = TBNET_RING_SIZE - tbnet_available_buffers', 'cleaned_count = tbnet_available_buffers')
if args.mutant == 'no-retry':
 bodies['tbnet_poll'] = bodies['tbnet_poll'].replace('if (tbnet_available_buffers(&net->rx_ring) < TBNET_RING_SIZE)', 'if (false)')
if args.mutant == 'no-cancel':
 bodies['tbnet_disable_napi'] = bodies['tbnet_disable_napi'].replace('cancel_delayed_work_sync(&net->rx_refill_work);', '')
if args.mutant == 'worker-allocation':
 bodies['tbnet_rx_refill_work'] = bodies['tbnet_rx_refill_work'].replace('napi_schedule(&net->napi);', 'page_pool_dev_alloc_pages(net->rx_ring.pool);\n\tnapi_schedule(&net->napi);')
# A source without a retry worker must not fire this callback.
if not bodies['tbnet_rx_refill_work']:
 bodies['tbnet_rx_refill_work'] = 'static void tbnet_rx_refill_work(struct work_struct *work) { abort(); }'

trailer=r'''
static void initialize(struct tbnet *n,struct net_device *d,struct tb_xdomain *x,struct tb_service *s){memset(n,0,sizeof(*n));memset(d,0,sizeof(*d));d->priv=n;d->running=true;d->attached=true;n->dev=d;n->xd=x;n->svc=s;s->data=n;n->napi.net=n;n->napi.list_owner=-1;pthread_mutex_init(&n->connection_lock,NULL);pthread_mutex_init(&n->poll_lock,NULL);INIT_DELAYED_WORK(&n->rx_refill_work,tbnet_rx_refill_work);assert(tbnet_open(d)==0);n->login_sent=n->login_received=true;n->remote_transmit_path=12;}
static void connect(struct tbnet *n){tbnet_connected_work(&n->connected_work);}
static void complete_frames(struct tbnet *n,int count,int total){assert(count<=(int)n->rx_ring.ring->queued);for(int i=0;i<count;i++){struct ring_frame *f=n->rx_ring.ring->queue[i];struct page *p=container_of(f,struct tbnet_frame,frame)->page;struct thunderbolt_ip_frame_header *h=(void*)p->mem;h->frame_size=64;h->frame_count=total;h->frame_index=i;f->size=80;f->flags=invalid_frame?RING_DESC_CRC_ERROR:0;}n->rx_ring.ring->completed=count;}
static void retry_poll(struct tbnet *n){assert(n->rx_refill_work.pending);fire_retry(&n->rx_refill_work);assert(n->napi.owner);assert(tbnet_poll(&n->napi,64)==0);}
static void exhaust(struct tbnet *n){alloc_limit=0;complete_frames(n,256,1);for(unsigned int i=0;i<256;i++){struct tbnet_frame *tf=container_of(n->rx_ring.ring->queue[i],struct tbnet_frame,frame);((struct thunderbolt_ip_frame_header*)tf->page->mem)->frame_index=0;}tbnet_start_poll(n);for(int i=0;i<4;i++)assert(tbnet_poll(&n->napi,64)==64);assert(tbnet_poll(&n->napi,64)==0);assert(n->rx_ring.prod-n->rx_ring.cons==0&&!n->rx_ring.ring->queued&&gro_count==256);assert(!n->napi.owner&&n->rx_refill_work.pending);}
static void *retry_thread(void *p){struct tbnet *n=p;fire_retry(&n->rx_refill_work);return NULL;}
static struct tbnet *thread_net;
static void *poll_thread(void *p){struct tbnet *n=p;pthread_mutex_lock(&gate);n->napi.owner=true;pthread_mutex_unlock(&gate);tbnet_poll(&n->napi,64);return NULL;}
static void *stop_thread(void *p){struct tbnet *n=p;tbnet_stop(n->dev);return NULL;}
static void *setup_thread(void *p){pthread_mutex_lock(&gate);setup_active=true;pthread_mutex_unlock(&gate);connect(p);pthread_mutex_lock(&gate);setup_active=false;pthread_cond_broadcast(&cv);pthread_mutex_unlock(&gate);return NULL;}
static void *tear_thread(void *p){tbnet_tear_down(p,false);return NULL;}
static void wait_paused(void){pthread_mutex_lock(&gate);while(!paused)pthread_cond_wait(&cv,&gate);pthread_mutex_unlock(&gate);}
static void wait_cancel(void)
{
	struct timespec deadline;

	assert(!clock_gettime(CLOCK_REALTIME, &deadline));
	deadline.tv_sec += 2;
	pthread_mutex_lock(&gate);
	while (!cancel_waiting)
		assert(!pthread_cond_timedwait(&cv, &gate, &deadline));
	assert(ring_stops == 0 && pool_destroy_calls == 0);
	pthread_mutex_unlock(&gate);
}
static void release_poll(void){pthread_mutex_lock(&gate);resume_poll=1;pthread_cond_broadcast(&cv);pthread_mutex_unlock(&gate);}
int main(int argc,char **argv){assert(argc==2);char *c=argv[1];struct tbnet n;struct net_device d;struct tb tb={0};struct tb_xdomain x={&tb};struct tb_service s={0};struct device dev={&s};
 if(!strcmp(c,"pool-create-failure")){pool_error=1;memset(&n,0,sizeof(n));memset(&d,0,sizeof(d));d.priv=&n;n.dev=&d;n.xd=&x;n.svc=&s;pthread_mutex_init(&n.connection_lock,NULL);assert(tbnet_open(&d)==-ENOMEM);assert(!n.rx_ring.pool&&!n.rx_ring.ring&&!n.tx_ring.ring&&ring_frees==2&&enable_calls==0);return 0;}
 initialize(&n,&d,&x,&s);
 if(!strcmp(c,"allocation-failure")){tb_ring_start(n.rx_ring.ring);alloc_limit=2;assert(tbnet_alloc_rx_buffers(&n,4)==-ENOMEM);assert(n.rx_ring.prod==2&&n.rx_ring.ring->queued==2&&pool_puts==0);alloc_limit=-1;assert(tbnet_alloc_rx_buffers(&n,2)==0&&n.rx_ring.prod==4);tb_ring_stop(n.rx_ring.ring);tbnet_free_buffers(&n.rx_ring);assert(pool_puts==4);return 0;}
 if(!strcmp(c,"enqueue-failure")){tb_ring_start(n.rx_ring.ring);queue_limit=2;assert(tbnet_alloc_rx_buffers(&n,4)==-ESHUTDOWN);assert(n.rx_ring.prod==2&&n.rx_ring.ring->queued==2&&pool_puts==1&&!n.rx_ring.frames[2].page);return 0;}
 if(!strcmp(c,"cold-failure-reconnect")){alloc_limit=2;connect(&n);assert(!d.carrier&&!n.napi.enabled&&!n.rings_started&&n.rx_ring.prod==0);int stops=ring_stops;tbnet_tear_down(&n,false);assert(ring_stops==stops&&disable_calls==0);alloc_limit=-1;n.login_sent=n.login_received=true;n.remote_transmit_path=12;connect(&n);assert(d.carrier&&n.napi.enabled);tbnet_stop(&d);assert(disable_calls==1);return 0;}
 if(!strcmp(c,"tx-failure")||!strcmp(c,"paths-failure")){if(c[0]=='t')tx_error=-ENOMEM;else path_error=-EIO;connect(&n);assert(!d.carrier&&!n.napi.enabled&&!n.rings_started&&n.rx_ring.prod==0&&pool_puts==256);tbnet_stop(&d);assert(disable_calls==(c[0]=='t'?0:1)&&ring_stops==2);return 0;}
 if(!strcmp(c,"early-initial-irq")||!strcmp(c,"early-reconnect-irq")){if(c[6]=='r'){connect(&n);tbnet_tear_down(&n,false);n.login_sent=n.login_received=true;n.remote_transmit_path=12;alloc_calls=0;}pause_at=5;pthread_t a;assert(!pthread_create(&a,NULL,setup_thread,&n));wait_paused();__ring_interrupt(n.rx_ring.ring);assert(n.rx_ring.ring->masked);if(n.napi.owner)tbnet_poll(&n.napi,64);assert(!n.napi.enabled&&schedule_calls==0);release_poll();assert(!pthread_join(a,NULL));assert(n.napi.enabled&&d.carrier&&n.rx_ring.prod==256&&!n.rx_ring.ring->masked);__ring_interrupt(n.rx_ring.ring);assert(n.napi.owner&&schedule_calls==1);pause_at=0;assert(tbnet_poll(&n.napi,64)==0);return 0;}
 if(!strcmp(c,"direct-recycle-admission")){fake_softirq=true;assert(n.rx_ring.pool->cpuid==-1&&n.napi.list_owner==-1&&!page_pool_napi_local(n.rx_ring.pool));connect(&n);tbnet_start_poll(&n);assert(page_pool_napi_local(n.rx_ring.pool));tbnet_poll(&n.napi,64);tbnet_tear_down(&n,false);assert(!n.napi.enabled&&n.napi.list_owner==-1&&!page_pool_napi_local(n.rx_ring.pool));return 0;}
 if(!strcmp(c,"setup-teardown-publication")){pause_at=4;pthread_t a,b;assert(!pthread_create(&a,NULL,setup_thread,&n));wait_paused();assert(!pthread_create(&b,NULL,tear_thread,&n));pthread_mutex_lock(&gate);while(!teardown_entered)pthread_cond_wait(&cv,&gate);pthread_mutex_unlock(&gate);release_poll();assert(!pthread_join(a,NULL)&&!pthread_join(b,NULL));assert(!d.carrier&&!d.queue_on&&!n.napi.enabled&&!n.rings_started);return 0;}
 connect(&n);assert(d.carrier);rearm_calls=0;
 if(!strcmp(c,"exhaustion-recovery")){exhaust(&n);int a=alloc_calls,irq=rearm_calls,sch=schedule_calls;alloc_limit=-1;retry_poll(&n);assert(alloc_calls==a+256&&n.rx_ring.prod-n.rx_ring.cons==256&&n.rx_ring.ring->queued==256&&!n.rx_refill_work.pending&&schedule_calls==sch+1&&rearm_calls==irq+1);return 0;}
 if(!strcmp(c,"empty-poll-refill")){exhaust(&n);n.rx_refill_work.pending=false;alloc_limit=-1;tbnet_start_poll(&n);assert(tbnet_poll(&n.napi,64)==0);assert(n.rx_ring.prod-n.rx_ring.cons==256&&!n.rx_refill_work.pending);return 0;}
 if(!strcmp(c,"bounded-repeated-failure")){exhaust(&n);int q=retry_queues;for(int i=0;i<5;i++){int a=alloc_calls;retry_poll(&n);assert(alloc_calls==a+1&&n.rx_refill_work.pending&&retry_queues==q+i+1&&retry_delay==100);}return 0;}
 if(!strcmp(c,"retry-coalesced")){exhaust(&n);int q=retry_queues;tbnet_start_poll(&n);assert(tbnet_poll(&n.napi,64)==0);assert(retry_queues==q&&n.rx_refill_work.pending);return 0;}
 if(!strcmp(c,"worker-only-schedules")){exhaust(&n);int a=alloc_calls;alloc_limit=-1;fire_retry(&n.rx_refill_work);assert(alloc_calls==a&&n.napi.owner&&n.rx_ring.ring->queued==0);tbnet_poll(&n.napi,64);return 0;}
 if(!strcmp(c,"partial-refill-recovery")){exhaust(&n);alloc_limit=3;retry_poll(&n);assert(n.rx_ring.prod-n.rx_ring.cons==3&&n.rx_refill_work.pending);alloc_limit=-1;retry_poll(&n);assert(n.rx_ring.prod-n.rx_ring.cons==256&&!n.rx_refill_work.pending);return 0;}
 if(!strcmp(c,"enqueue-retry-cleanup")){exhaust(&n);alloc_limit=-1;queue_limit=2;int puts=pool_puts;retry_poll(&n);assert(n.rx_ring.prod-n.rx_ring.cons==2&&pool_puts==puts+1&&!n.rx_ring.frames[n.rx_ring.prod&255].page&&n.rx_refill_work.pending);queue_limit=-1;retry_poll(&n);assert(n.rx_ring.prod-n.rx_ring.cons==256&&!n.rx_refill_work.pending);return 0;}
 if(!strcmp(c,"retry-full-budget")){alloc_limit=0;complete_frames(&n,1,1);tbnet_start_poll(&n);assert(tbnet_poll(&n.napi,1)==1&&n.rx_refill_work.pending&&n.napi.owner);return 0;}
 if(!strcmp(c,"retry-zero-budget")){exhaust(&n);int a=alloc_calls,q=retry_queues;fire_retry(&n.rx_refill_work);assert(tbnet_poll(&n.napi,0)==0&&alloc_calls==a&&retry_queues==q&&n.napi.owner);alloc_limit=-1;assert(tbnet_poll(&n.napi,64)==0&&n.rx_ring.prod-n.rx_ring.cons==256);return 0;}
 if(!strcmp(c,"retry-stop")||!strcmp(c,"retry-disconnect")||!strcmp(c,"retry-suspend")||!strcmp(c,"retry-shutdown")){exhaust(&n);int a=alloc_calls;if(!strcmp(c,"retry-stop"))tbnet_stop(&d);else if(!strcmp(c,"retry-suspend"))tbnet_suspend(&dev);else if(!strcmp(c,"retry-shutdown"))tbnet_shutdown(&s);else tbnet_tear_down(&n,false);assert(!n.rx_refill_work.pending&&!n.napi.enabled&&alloc_calls==a);assert(!n.rx_ring.ring||!n.rx_ring.ring->running);return 0;}
 if(!strcmp(c,"retry-reconnect")){exhaust(&n);tbnet_tear_down(&n,false);assert(!n.rx_refill_work.pending);alloc_limit=-1;n.login_sent=n.login_received=true;n.remote_transmit_path=12;connect(&n);assert(n.napi.enabled&&n.rx_ring.prod==256&&!n.rx_refill_work.pending);tbnet_stop(&d);return 0;}
 if(!strcmp(c,"retry-completion-tail")){alloc_limit=0;complete_frames(&n,1,2);pause_at=2;pthread_t a,b;assert(!pthread_create(&a,NULL,poll_thread,&n));wait_paused();assert(n.rx_refill_work.pending);assert(!pthread_create(&b,NULL,stop_thread,&n));usleep(30000);assert(ring_stops==0&&n.rx_refill_work.pending);release_poll();assert(!pthread_join(a,NULL)&&!pthread_join(b,NULL));assert(!n.rx_refill_work.pending&&ring_stops==2);return 0;}
 if(!strcmp(c,"retry-running-teardown")){exhaust(&n);pause_at=6;pthread_t a,b;assert(!pthread_create(&a,NULL,retry_thread,&n));wait_paused();assert(!pthread_create(&b,NULL,stop_thread,&n));wait_cancel();release_poll();assert(!pthread_join(a,NULL)&&!pthread_join(b,NULL));assert(!n.rx_refill_work.pending&&!n.rx_refill_work.running&&!n.napi.owner&&ring_stops==2&&pool_destroy_calls==1);return 0;}
 if(!strcmp(c,"retry-already-polling")){exhaust(&n);tbnet_start_poll(&n);fire_retry(&n.rx_refill_work);int rearm=rearm_calls;assert(n.napi.missed&&tbnet_poll(&n.napi,64)==0&&n.napi.owner&&n.rx_refill_work.pending&&rearm_calls==rearm);alloc_limit=-1;assert(tbnet_poll(&n.napi,64)==0&&!n.napi.owner&&n.rx_ring.prod-n.rx_ring.cons==256);return 0;}
 if(!strcmp(c,"retry-disabled-worker")){exhaust(&n);napi_disable(&n.napi);int a=alloc_calls,sch=schedule_calls;fire_retry(&n.rx_refill_work);assert(!n.napi.owner&&schedule_calls==sch&&alloc_calls==a);return 0;}
 if(!strcmp(c,"retry-counter-wrap")){n.rx_ring.cons=0xffffff00;n.rx_ring.prod=0;exhaust(&n);alloc_limit=-1;retry_poll(&n);assert(n.rx_ring.cons==0&&n.rx_ring.prod==256&&!n.rx_refill_work.pending);return 0;}
 if(!strcmp(c,"retry-startup-unwind")){tbnet_tear_down(&n,false);path_error=-EIO;alloc_limit=-1;n.login_sent=n.login_received=true;n.remote_transmit_path=12;connect(&n);assert(!n.rings_started&&!n.napi.enabled&&!n.rx_refill_work.pending&&n.rx_ring.prod==0);return 0;}

 if(!strcmp(c,"completion-false")){completion_ok=0;n.napi.owner=true;assert(tbnet_poll(&n.napi,64)==0);assert(complete_calls==1&&rearm_calls==0);return 0;}
 if(!strcmp(c,"budget-zero")){struct tbnet_frame *f=&n.rx_ring.frames[0];f->page->owner=3;f->page=NULL;n.rx_ring.cons++;int calls=alloc_calls;assert(tbnet_poll(&n.napi,0)==0);assert(alloc_calls==calls&&complete_calls==0&&rearm_calls==0);return 0;}
 if(!strcmp(c,"full-budget")){complete_frames(&n,1,1);n.napi.owner=true;assert(tbnet_poll(&n.napi,1)==1);assert(complete_calls==0&&rearm_calls==0&&gro_count==1&&gro[0]->recycle);return 0;}
 if(!strcmp(c,"live-partial-race")){complete_frames(&n,1,2);n.napi.owner=true;assert(tbnet_poll(&n.napi,64)==1);assert(n.skb);struct sk_buff *partial=n.skb;tail_pending=false;complete_frames(&n,1,2);((struct thunderbolt_ip_frame_header*)container_of(n.rx_ring.ring->queue[0],struct tbnet_frame,frame)->page->mem)->frame_index=1;pause_at=3;pthread_t a,b;assert(!pthread_create(&a,NULL,poll_thread,&n));wait_paused();assert(!pthread_create(&b,NULL,tear_thread,&n));usleep(30000);assert(!partial->freed&&ring_stops==0);release_poll();assert(!pthread_join(a,NULL)&&!pthread_join(b,NULL));assert(disable_calls==1&&!n.skb&&gro_count==1);return 0;}
 if(!strcmp(c,"completion-tail")||!strcmp(c,"popped-page")){pause_at=!strcmp(c,"completion-tail")?2:1;if(pause_at==1)complete_frames(&n,1,2);pthread_t a,b;assert(!pthread_create(&a,NULL,poll_thread,&n));wait_paused();assert(!pthread_create(&b,NULL,stop_thread,&n));usleep(30000);assert(ring_stops==0&&ring_frees==0&&pool_destroy_calls==0);release_poll();assert(!pthread_join(a,NULL)&&!pthread_join(b,NULL));assert(disable_calls==1&&ring_stops==2&&ring_frees==2&&pool_destroy_calls==1);return 0;}
 if(!strcmp(c,"invalid-frame")||!strcmp(c,"build-failure")){if(c[0]=='i')invalid_frame=1;else build_error=1;complete_frames(&n,1,1);n.napi.owner=true;assert(tbnet_poll(&n.napi,64)==0);assert(pool_puts==1&&!n.skb&&gro_count==0);return 0;}
 if(!strcmp(c,"partial-disconnect-reconnect")||!strcmp(c,"suspend-reconnect")||!strcmp(c,"shutdown-partial")||!strcmp(c,"deferred-stack-pages")){complete_frames(&n,2,3);n.napi.owner=true;assert(tbnet_poll(&n.napi,64)==2);assert(n.skb&&n.skb->recycle&&n.skb->sh.nr_frags==1);tail_pending=false;struct page_pool *old=n.rx_ring.pool;if(!strcmp(c,"deferred-stack-pages")){struct sk_buff *held=n.skb;n.skb=NULL;assert(tbnet_stop(&d)==0);assert(old->detached&&old->alive==2);dev_kfree_skb_any(held);assert(old->alive==0);assert(tbnet_open(&d)==0&&n.rx_ring.pool!=old);return 0;}
 if(c[0]=='s'&&c[1]=='u')tbnet_suspend(&dev);else if(c[0]=='s')tbnet_shutdown(&s);else tbnet_tear_down(&n,false);assert(!n.skb&&!n.napi.enabled&&pool_puts==258);int stopped=ring_stops;tbnet_tear_down(&n,false);assert(disable_calls==1&&ring_stops==stopped);
 if(!strcmp(c,"suspend-reconnect"))tbnet_resume(&dev);n.login_sent=n.login_received=true;n.remote_transmit_path=12;connect(&n);assert(n.napi.enabled&&d.carrier&&enable_calls==2);tbnet_stop(&d);assert(disable_calls==2);return 0;}
 if(!strcmp(c,"stop-after-disconnect")){tbnet_tear_down(&n,false);tbnet_stop(&d);assert(disable_calls==1&&ring_stops==2);return 0;}
 if(!strcmp(c,"tx-guard")){struct tbnet_frame *f=&n.tx_ring.frames[0];f->page=calloc(1,sizeof(struct page));f->page->owner=2;tbnet_free_buffers(&n.tx_ring);assert(tx_unmaps==0);f->page=calloc(1,sizeof(struct page));f->page->owner=2;f->frame.buffer_phy=4096;tbnet_free_buffers(&n.tx_ring);assert(tx_unmaps==1);return 0;}
 abort();
}
'''
cases=['direct-recycle-admission','early-initial-irq','early-reconnect-irq','setup-teardown-publication','pool-create-failure','allocation-failure','enqueue-failure','cold-failure-reconnect','tx-failure','paths-failure','completion-false','budget-zero','full-budget','live-partial-race','completion-tail','popped-page','invalid-frame','build-failure','partial-disconnect-reconnect','suspend-reconnect','shutdown-partial','deferred-stack-pages','stop-after-disconnect','tx-guard']
cases += ['exhaustion-recovery','empty-poll-refill','bounded-repeated-failure','retry-coalesced','worker-only-schedules','partial-refill-recovery','enqueue-retry-cleanup','retry-full-budget','retry-zero-budget','retry-stop','retry-disconnect','retry-suspend','retry-shutdown','retry-reconnect','retry-completion-tail','retry-running-teardown','retry-disabled-worker','retry-startup-unwind','retry-already-polling','retry-counter-wrap']
if args.case: cases = args.case
receipt={'mutant':args.mutant,'fixture_sha256':hashlib.sha256(Path(__file__).with_name('tbnet-refill-fixture.h').read_bytes()).hexdigest(),'driver_sha256':hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),'old_worker_adapter':not bool(original_bodies['tbnet_rx_refill_work']),'ref':subprocess.check_output(['git','rev-parse',ref],cwd=wt,text=True).strip() if ref!='WORKTREE' else ref,'source_sha256':hashlib.sha256(source.encode()).hexdigest(),'production_bodies':{k:hashlib.sha256(v.encode()).hexdigest() for k,v in original_bodies.items() if v},'limitations':'Exact production lifecycle/poll bodies, fake page_pool/DMA/NHI/NAPI/skb effects and pthread completion barriers. Exact production header validation also runs; generic page_pool internals, IRQ/MMIO, full netdev locks and hardware are not executed. Generic API source reviewed separately.','results':{}}
tag=ref[:12].lower()
probe = extract('tbnet_probe')
if original_bodies['tbnet_rx_refill_work']:
 assert probe.index('INIT_DELAYED_WORK(&net->rx_refill_work, tbnet_rx_refill_work);') < probe.index('register_netdev(dev)')
receipt['probe_sha256'] = hashlib.sha256(probe.encode()).hexdigest()
for page_size in [4096,16384]:
 code=out/(tag+'-'+str(page_size)+'.c');exe=code.with_suffix('')
 code.write_text('#define PAGE_SIZE '+str(page_size)+'\n'+(Path(__file__).with_name('tbnet-refill-fixture.h')).read_text()+'\n'+'\n\n'.join(v for v in bodies.values() if v)+'\n'+trailer)
 p=subprocess.run(['cc','-pthread','-std=gnu11','-O0','-g','-Werror=implicit-function-declaration',str(code),'-o',str(exe)],capture_output=True,text=True)
 (code.with_suffix('.compile.log')).write_text(p.stdout+p.stderr)
 if p.returncode:print(p.stderr);sys.exit(p.returncode)
 vals={}
 for case in cases:
  try:p=subprocess.run([str(exe),case],capture_output=True,text=True,timeout=5);vals[case]={'exit':p.returncode,'stderr':p.stderr}
  except subprocess.TimeoutExpired:vals[case]={'timeout':True}
 receipt['results'][str(page_size)]=vals
 path=out/(tag+'-controls.json');path.write_text(json.dumps(receipt,indent=2)+'\n')
print(json.dumps({k:{c:v.get('exit','timeout') for c,v in d.items()}for k,d in receipt['results'].items()},indent=2))
raise SystemExit(any(v.get('exit',1)!=0 for vals in receipt['results'].values() for v in vals.values()))
