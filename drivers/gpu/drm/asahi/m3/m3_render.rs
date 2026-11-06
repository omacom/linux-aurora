// SPDX-License-Identifier: GPL-2.0-only OR MIT
use kernel::prelude::*;
use crate::m3::m3_queue_layout as queue;
use crate::m3::m3_sync_layout as sync;
use crate::m3::m3_state_layout as state;
use crate::m3::m3_scene_layout as scene;
use crate::{m3::m3_pass::{Pass,ViewOwner},m3::m3_pass_layout::{self as pass_layout,Field as P,Space}};
use crate::{m3::m3_fragment_command as fragment, m3::m3_tiler_command as tiler, m3::m3_render_sequence as sequence, m3::m3_init_layout::Region};
use crate::{m3::m3_parameter_layout as parameter, m3::m3_pool_layout::GpuRegion};
use crate::{driver,mmu,pgtable::prot,m3::m3_memory::Buffer,m3::m3_shared_layout as s,m3::agx_uapi};
use crate::m3::m3_timeline as timeline;
use s::{PB_FIRST,PB_GROUPS,PB_BLOCKS_PER_GROUP,PB_BLOCK_SIZE};
const PB_BLOCKS:u32=(PB_GROUPS*PB_BLOCKS_PER_GROUP) as u32;
pub(crate) const PB_PAGES:u32=PB_BLOCKS*4;
// Serial execution permits retaining each client's views of the owned arenas.
// Bound the cache to four VMs including the active one; cache entries retain
// their ASID lease and all aliases until eviction after exact retirement.
struct CachedViews {
    aliases:KVec<mmu::KernelMapping>,
    views:KVec<(ViewOwner,mmu::KernelMapping)>,
    // Release mappings before returning their ASID lease (Rust field order).
    binding:mmu::VmBind,
}
pub(crate) struct Render {
    _aliases:KVec<mmu::KernelMapping>, objects:KVec<Buffer>,
    passes:KVec<Pass>, init_bm:Buffer,
    // All active mappings above must drop before their ASID binding.
    _binding:mmu::VmBind,
    stats: [Region;2], profile:super::m3_profile::Profile, sequence_bytes: KVec<u8>, tiler_bytes: KVec<u8>, fragment_bytes: KVec<u8>,
    last_progress_ns:i64,
    draw:u64, heads:[u16;2], cached_views:KVec<CachedViews>,
    slot:usize, batch_count:usize, early_count:u32,
    /// Measurement packets in the pass most recently appended to its queues.
    stage_timestamps:[bool;2],
    /// First pass slot of the current batch; its passes use base..base+batch_count mod SLOTS.
    base:usize,
    checkpoint:(u64,[u16;2],usize,usize,u32,usize),
    /// GPU clusters of the board: tiler accelerators (TPC storage) and preemption buffers.
    clusters:u32,
    /// The register lists of the SoC's GPU (`m3_soc::Soc::registers`).
    registers:crate::m3::m3_compute_layout::RegisterSet,
    /// Last: drops after the pass aliases.
    _lower_tlbi:crate::m3::m3_memory::LowerTlbi,
}
// Queue allocations belong to Render, not to a pass. Keep their fixed arenas
// and firmware/shared mapping permissions from the qualified allocation map.
// Scratch is GPU-visible firmware storage; it is not the application's VM.
struct QueueObjects {
    info: usize, state: usize, ring: usize, scratch: usize,
    tiler: bool, uuid: u32,
}
use s::{GPU_CONTEXT,JOB_LIST,BUFFER_MANAGER,PARAMETER_PAGE_LIST,
    PARAMETER_BLOCK_LIST,NOTIFIER,BLOCK_CONTROL,MANAGER_MISC};
const MANAGER_SCENE_COUNT: u32 = 48;
const TA_POOL_COUNTER: usize = 11;
const UMA_POOLS: [crate::m3::m3_pool::Owners; 2] = [
    crate::m3::m3_pool::Owners { backing: 0..8, pages: 8, table: 9, manager: 10, counter: 11, table_index: 0 },
    crate::m3::m3_pool::Owners { backing: 12..20, pages: 20, table: 21, manager: 22, counter: 23, table_index: 1 },
];
const FRAGMENT_POOL_COUNTER: usize = 23;
// Retained producer identifier for cross-pass dependencies, G15 / V14_8_3.
const DEPENDENCY_UUID: u32 = 250912;
const RENDER_QUEUES: [QueueObjects; 2] = [
    QueueObjects { info: s::TA_QUEUE, state: s::TA_POINTERS, ring: s::TA_RING,
        scratch: s::TA_SCRATCH, tiler: true, uuid: 0x7a0000 },
    QueueObjects { info: s::FRAGMENT_QUEUE, state: s::FRAGMENT_POINTERS, ring: s::FRAGMENT_RING,
        scratch: s::FRAGMENT_SCRATCH, tiler: false, uuid: 0x3d0000 },
];
impl Render {
    pub(crate) fn new(dev:&driver::AsahiDrmDevice,uat:&mmu::Uat,vm:&mmu::Vm,stats:[Region;2],clusters:u32,
        registers:crate::m3::m3_compute_layout::RegisterSet,profile:super::m3_profile::Profile)->Result<Self> {
        let lower_tlbi=crate::m3::m3_memory::LowerTlbi;
        let binding=uat.bind(vm)?;
        let mut objects=KVec::new();let mut aliases=KVec::new();
        for index in 0..s::COUNT {
            let a=s::allocation_for(profile,index).map_err(|_|EINVAL)?;
            let protection=prot::Prot::from_pte(a.access.pte());
            let b=match a.space {
                Space::ClientGpu=>Buffer::at_prot(dev,uat.kernel_vm(),Some((vm,a.address)),None,
                    a.size,prot::PROT_GPU_FW_SHARED_RW,protection)?,
                Space::Firmware=>Buffer::at_prot(dev,uat.kernel_vm(),
                    a.gpu_alias.map(|address|(uat.kernel_lower_vm(),address)),Some(a.address),
                    a.size,protection,prot::PROT_GPU_SHARED_RW)?,
            };
            let mut b=b;
            if a.gpu_alias.is_some() { aliases.push(b.map_gpu_view(vm)?,GFP_KERNEL)?; }
            objects.push(b,GFP_KERNEL)?;
        }
        // InitBM is shared, not part of a pass. Preserve its qualified mapping.
        let init_bm=Buffer::at_prot(dev,uat.kernel_vm(),None,Some(if profile==super::m3_profile::Profile::Extended {0xffff_fc24_4000_0000}
                else {0xfffffc200062bfe0}),
            sync::INIT_BM_SIZE,prot::PROT_FW_SHARED_RW,prot::PROT_GPU_SHARED_RW)?;
        let mut passes=KVec::with_capacity(pass_layout::SLOTS,GFP_KERNEL)?;
        for slot in 0..pass_layout::SLOTS {passes.push(Pass::new(dev,uat,vm,slot,clusters,profile,&mut aliases)?,GFP_KERNEL)?;}
        let mut sequence_bytes=KVec::new();
        sequence_bytes.resize(sequence::STORAGE,0u8,GFP_KERNEL)?;
        let mut tiler_bytes=KVec::new();
        tiler_bytes.resize(if profile==super::m3_profile::Profile::Extended {tiler::EXTENDED_SIZE} else {tiler::SIZE},0u8,GFP_KERNEL)?;
        let mut fragment_bytes=KVec::new();
        fragment_bytes.resize(if profile==super::m3_profile::Profile::Extended {fragment::EXTENDED_SIZE} else {fragment::SIZE},0u8,GFP_KERNEL)?;
        let mut job=Self{passes,init_bm,stats,profile,sequence_bytes,tiler_bytes,fragment_bytes,objects,_aliases:aliases,_binding:binding,last_progress_ns:0,draw:0,heads:[0;2],cached_views:KVec::new(),slot:0,batch_count:0,early_count:0,stage_timestamps:[true;2],base:0,checkpoint:(0,[0;2],0,0,0,0),clusters,registers,_lower_tlbi:lower_tlbi};
        // Lists and the matching manager are built before InitBM publication.
        job.initialize_parameter_buffer()?;
        for (index,pool) in UMA_POOLS.iter().enumerate() {
            if profile==super::m3_profile::Profile::Extended {
                pool.initialize_extended(&mut job.objects,33+index as u8,false)?;
            } else {pool.initialize(&mut job.objects)?;}
        }
        job.initialize_state()?;
        job.initialize_queues()?;
        dev_info!(dev.as_ref(),"M3 native render storage initialized, PB={} MiB\n",PB_BLOCKS as usize*PB_BLOCK_SIZE/(1024*1024));
        crate::m3::agx_memory::publish();crate::mem::sync();crate::mem::tlbi_all();crate::mem::sync();
        unsafe {core::arch::asm!("isb",options(nostack,preserves_flags))};
        Ok(job)
    }
    fn initialize_parameter_buffer(&mut self) -> Result {
        let region = |owner: usize| GpuRegion::new(self.objects[owner].gpu_va()?, self.objects[owner].size())
            .map_err(|_| EINVAL);
        let pages = region(PARAMETER_PAGE_LIST)?;
        let blocks = region(PARAMETER_BLOCK_LIST)?;
        let control = queue::FirmwareVa::new(self.objects[BLOCK_CONTROL].va()).map_err(|_| EINVAL)?;
        let counter = queue::FirmwareVa::new(self.objects[s::BM_COUNTER].va()).map_err(|_| EINVAL)?;
        let manager = parameter::Manager::new(pages, blocks, control, counter, PB_BLOCKS).map_err(|_| EINVAL)?;
        let extended=self.profile==super::m3_profile::Profile::Extended;
        let mut bytes = [0; parameter::Manager::EXTENDED_SIZE];
        let length=if extended {
            manager.encode_extended(queue::FirmwareVa::new(self.objects[PARAMETER_PAGE_LIST].va()).map_err(|_|EINVAL)?,
                queue::FirmwareVa::new(self.objects[PARAMETER_BLOCK_LIST].va()).map_err(|_|EINVAL)?,0,&mut bytes)
                .map_err(|_|EINVAL)?;
            parameter::Manager::EXTENDED_SIZE
        } else {manager.encode(&mut bytes).map_err(|_|EINVAL)?;parameter::MANAGER_SIZE};
        for group in 0..PB_GROUPS {
            let buffer = &self.objects[PB_FIRST + group];
            let backing = parameter::Backing::new(GpuRegion::new(buffer.gpu_va()?, buffer.size()).map_err(|_| EINVAL)?)
                .map_err(|_| EINVAL)?;
            for block in 0..PB_BLOCKS_PER_GROUP {
                self.objects[PARAMETER_BLOCK_LIST].write((group * PB_BLOCKS_PER_GROUP + block) * 8,
                    &backing.block(block).map_err(|_| EINVAL)?)?;
            }
            for page in 0..PB_BLOCKS_PER_GROUP * parameter::PAGES_PER_BLOCK {
                self.objects[PARAMETER_PAGE_LIST].u32((group * PB_BLOCKS_PER_GROUP * parameter::PAGES_PER_BLOCK + page) * 4,
                    backing.page(page).map_err(|_| EINVAL)?)?;
            }
        }
        self.objects[BUFFER_MANAGER].write(0, &bytes[..length])
    }
    fn initialize_state(&mut self) -> Result {
        let address = |owner: usize| queue::FirmwareVa::new(self.objects[owner].va()).map_err(|_| EINVAL);
        let notification = state::Notifier { threshold: address(s::EVENT_COUNT)?, context: self._binding.slot() };
        let list_owner = address(JOB_LIST)?;
        let mut context = [0; state::CONTEXT_SIZE];
        if self.profile==super::m3_profile::Profile::Extended {
            state::context_extended(&mut context).map_err(|_|EINVAL)?;
            self.objects[GPU_CONTEXT].write(0,&context[..state::EXTENDED_CONTEXT_SIZE])?;
            for engine in 0..2 {
                let value=state::Notifier {threshold:queue::FirmwareVa::new(self.objects[s::EVENT_COUNT].va()+engine as u64*8).map_err(|_|EINVAL)?,
                    context:self._binding.slot()};
                let mut notifier=[0;state::EXTENDED_NOTIFIER_SIZE];
                value.encode_extended(&mut notifier).map_err(|_|EINVAL)?;
                self.objects[NOTIFIER].write(engine*state::EXTENDED_NOTIFIER_SIZE,&notifier)?;
            }
        } else {
            state::context(&mut context).map_err(|_| EINVAL)?;
            self.objects[GPU_CONTEXT].write(0, &context)?;
            let mut notifier = [0; state::NOTIFIER_SIZE];
            notification.encode(&mut notifier).map_err(|_| EINVAL)?;
            self.objects[NOTIFIER].write(0, &notifier)?;
        }
        let mut list = [0; state::JOB_LIST_SIZE];
        state::job_list(&mut list, list_owner).map_err(|_| EINVAL)?;
        self.objects[JOB_LIST].write(0, &list)?;
        let mut manager = [0; state::MANAGER_STATE_SIZE];
        state::block_control(&mut manager, PB_BLOCKS, PB_BLOCKS).map_err(|_| EINVAL)?;
        self.objects[BLOCK_CONTROL].write(0, &manager)?;
        state::manager_misc(&mut manager).map_err(|_| EINVAL)?;
        self.objects[MANAGER_MISC].write(0, &manager)?;
        for (owner, value) in [(TA_POOL_COUNTER, 1), (FRAGMENT_POOL_COUNTER, 1),
            (s::EVENT_COUNT, 2), (s::BM_COUNTER, 1),
            (s::TA_STAMP, timeline::stamp(s::STAMP_TA - 256, 0)), (s::TA_FW_STAMP, timeline::stamp(s::STAMP_TA - 256, 0)),
            (s::FRAGMENT_STAMP, timeline::stamp(s::STAMP_FRAGMENT - 256, 0)), (s::FRAGMENT_FW_STAMP, timeline::stamp(s::STAMP_FRAGMENT - 256, 0))] {
            let mut bytes = [0; state::MANAGER_STATE_SIZE];
            let bytes = bytes.get_mut(..self.objects[owner].size()).ok_or(EINVAL)?;
            state::counter(bytes, value).map_err(|_| EINVAL)?;
            self.objects[owner].write(0, bytes)?;
        }
        if self.profile==super::m3_profile::Profile::Extended {
            self.objects[s::EVENT_COUNT].fill(0)?;
            self.objects[s::BM_COUNTER].fill(0)?;
        }
        Ok(())
    }
    fn initialize_queues(&mut self) -> Result {
        for owner in &RENDER_QUEUES {
            let address = |index: usize| queue::FirmwareVa::new(self.objects[index].va()).map_err(|_| EINVAL);
            let info = queue::QueueInfo {
                state: address(owner.state)?, ring: address(owner.ring)?,
                job_list: address(JOB_LIST)?, gpu_scratch: address(owner.scratch)?,
                context: address(GPU_CONTEXT)?, uuid: owner.uuid,
            };
            let seed_addresses=if owner.tiler {
                [self.init_bm.va(),self.passes[0].get(P::TilerCommand).va()]
            }else{[self.passes[0].get(P::TilerToFragment).va(),self.passes[0].get(P::FragmentCommand).va()]};
            let seeds=[queue::FirmwareVa::new(seed_addresses[0]).map_err(|_|EINVAL)?,
                queue::FirmwareVa::new(seed_addresses[1]).map_err(|_|EINVAL)?];
            let mut bytes=KVec::new();
            bytes.resize(if self.profile==super::m3_profile::Profile::Extended {
                queue::QueueInfo::EXTENDED_SIZE
            } else {queue::INFO_SIZE},0u8,GFP_KERNEL)?;
            if self.profile==super::m3_profile::Profile::Extended {
                info.encode_extended(address(owner.info)?,&mut bytes).map_err(|_|EINVAL)?;
            } else {info.encode(&mut bytes).map_err(|_|EINVAL)?;}
            self.objects[owner.info].write(0, &bytes)?;
            let mut state = [0; queue::STATE_SIZE];
            queue::RingState { capacity: queue::QUALIFIED_CAPACITY, done: 0, read: 0, write: 0 }
                .encode(&mut state).map_err(|_| EINVAL)?;
            self.objects[owner.state].write(0, &state)?;
            for index in 0..queue::QUALIFIED_CAPACITY {
                let entry = queue::ring_entry(index, queue::QUALIFIED_CAPACITY, &seeds).map_err(|_| EINVAL)?;
                self.objects[owner.ring].write(index as usize * 8, &entry)?;
            }
        }
        Ok(())
    }
    fn write_barrier(&mut self, owner: P, command: sync::Barrier) -> Result {
        let mut bytes=[0;sync::Barrier::EXTENDED_SIZE];
        let length=if self.profile==super::m3_profile::Profile::Extended {
            command.encode_extended(&mut bytes).map_err(|_|EINVAL)?;
            sync::Barrier::EXTENDED_SIZE
        } else {command.encode(&mut bytes).map_err(|_|EINVAL)?;sync::BARRIER_SIZE};
        self.passes[self.slot].get_mut(owner).write(0,&bytes[..length])
    }
    // Construct synchronization from the actual owners after resetting a pass,
    // before publishing its ring entries. InitBM is shared and dispatched only
    // for draw one; rewriting its bytes after retirement preserves the old path.
    fn prepare_sync(&mut self, early: bool, draw: u64,
        predecessor:Option<sync::CompletionDependency>) -> Result {
        let address = |owner: usize| queue::FirmwareVa::new(self.objects[owner].va()).map_err(|_| EINVAL);
        let manager = address(BUFFER_MANAGER)?;
        let ta_stamp = address(s::TA_FW_STAMP)?;
        let fs_stamp = address(s::FRAGMENT_FW_STAMP)?;
        let mut init = [0; sync::INIT_BM_SIZE];
        let value=sync::InitBufferManager { context: self._binding.slot(), slot: 0,
            block_count: PB_BLOCKS, manager, stamp: timeline::stamp(s::STAMP_TA - 256, 1) };
        if self.profile==super::m3_profile::Profile::Extended {
            value.encode_extended(&mut init).map_err(|_|EINVAL)?;
        } else {value.encode(&mut init).map_err(|_|EINVAL)?;}
        self.init_bm.write(0, &init)?;
        let ta_value = timeline::stamp(s::STAMP_TA - 256, draw);
        let fs_value = timeline::stamp(s::STAMP_FRAGMENT - 256, draw);
        let previous=predecessor.unwrap_or(sync::CompletionDependency {
            stamp:if early {ta_stamp} else {fs_stamp},
            value:timeline::stamp((if early {s::STAMP_TA} else {s::STAMP_FRAGMENT})-256,draw-1),
            event:u32::from(!early),
        });
        self.write_barrier(P::TilerToFragment, sync::Barrier {
            stamp: ta_stamp, wait_value: ta_value, event: 0, self_value: fs_value,
            uuid: 0x3d0000, dependency: sync::Dependency::TilerToFragment })?;
        self.write_barrier(P::TilerDependency, sync::Barrier {
            stamp: previous.stamp, wait_value: previous.value,
            event: previous.event, self_value: ta_value,
            uuid: DEPENDENCY_UUID, dependency: sync::Dependency::PreviousPass })?;
        self.write_barrier(P::FragmentDependency, sync::Barrier {
            stamp: fs_stamp, wait_value: timeline::stamp(s::STAMP_FRAGMENT - 256, draw - 1), event: 1, self_value: fs_value,
            uuid: DEPENDENCY_UUID, dependency: sync::Dependency::PreviousPass })
    }
    // Prepare each private command after its retired slot is reset. The
    // reusable host staging allocation is copied before the next slot is built.
    fn prepare_tiler(&mut self,r:agx_uapi::UapiRenderCommand,usc:u64,draw:u64)->Result {
        use crate::m3::m3_compute_layout::GpuVa;
        let pass=&self.passes[self.slot];
        let shared=|index:usize| queue::FirmwareVa::new(self.objects[index].va()).map_err(|_|EINVAL);
        let fw=|field:P| queue::FirmwareVa::new(pass.get(field).va()).map_err(|_|EINVAL);
        let gpu=|field:P| GpuVa::new(pass.get(field).gpu_va()?).map_err(|_|EINVAL);
        let seq=pass.get(P::TilerSequence);
        let command=pass.get(P::TilerCommand);
        let g=Self::geometry(r,self.clusters)?;
        if g.tpc_bytes>pass.get(P::Tpc).size() as u64
            || g.tilemap_bytes>pass.get(P::Tilemap).size() as u64 {return Err(E2BIG);}
        // The old per-slot register keeps its seed entry, independent of the
        // draw-dependent RenderScene reference. Do not change this policy here.
        let initial_scene_entry=queue::FirmwareVa::new(self.objects[s::BM_SCENES].va()
            +(self.slot as u64+1)*4).map_err(|_|EINVAL)?;
        let value=tiler::Command {
            context:self._binding.slot(),register_set:self.registers,
            notifier:shared(NOTIFIER)?,manager:shared(BUFFER_MANAGER)?,
            scene:fw(P::Scene)?,empty:if self.profile==super::m3_profile::Profile::Extended {queue::FirmwareVa::new(pass.get(P::Scene).va()+0x100).map_err(|_|EINVAL)?} else {shared(547)?},gpu_alias:GpuVa::new(command.gpu_va()?).map_err(|_|EINVAL)?,
            sequence:Region::new(seq.va(),seq.size()).map_err(|_|EINVAL)?,pool:shared(UMA_POOLS[0].manager)?,
            scratch:fw(P::TilerScratch)?,stamp:shared(s::TA_STAMP)?,fw_stamp:shared(s::TA_FW_STAMP)?,
            stamp_value:timeline::stamp(s::STAMP_TA - 256, draw),timestamps:[fw(P::TilerStart)?,fw(P::TilerEnd)?],
            user_timestamps:[fw(P::TilerUserStart)?,fw(P::TilerUserEnd)?],preemption:[gpu(P::Preemption0)?,gpu(P::Preemption1)?,gpu(P::Preemption2)?],
            tilemap:gpu(P::Tilemap)?,tpc:gpu(P::Tpc)?,tpc_bytes:g.tpc_bytes,heap:gpu(P::HeapMetadata)?,
            scene_user:gpu(P::SceneUser)?,initial_scene_entry,geometry:g,page_count:PB_PAGES,
            vdm:crate::m3::agx_render_state::compact(r.vdm_base).map_err(|_|EINVAL)?,
            multisample:r.multisample_control,ppp:r.ppp_control,process_empty_tiles:r.flags&2!=0,
        };
        if self.profile==super::m3_profile::Profile::Extended {
            value.encode_extended(tiler::Dispatch26 {incarnation:self._binding.generation(),serial:draw.checked_mul(2).ok_or(EOVERFLOW)?,
                peer_stamp:timeline::stamp(s::STAMP_FRAGMENT-256,draw),usc_window:usc,
                firmware_aux:queue::FirmwareVa::new(self.objects[UMA_POOLS[0].manager].va()+0x100).map_err(|_|EINVAL)?,
                scene_scratch:gpu(P::FragmentScratch)?},&mut self.tiler_bytes).map_err(|_|EINVAL)?;
        } else {value.encode(&mut self.tiler_bytes).map_err(|_|EINVAL)?;}
        self.passes[self.slot].get_mut(P::TilerCommand).write(0,&self.tiler_bytes)
    }
    fn prepare_sequences(&mut self,draw:u64)->Result {
        for fs in [false,true] {
            let pass=&self.passes[self.slot];
            let region=|field:P| {let b=pass.get(field);Region::new(b.va(),b.size()).map_err(|_|EINVAL)};
            let shared=|index:usize| queue::FirmwareVa::new(self.objects[index].va()).map_err(|_|EINVAL);
            let fw=|field:P| queue::FirmwareVa::new(pass.get(field).va()).map_err(|_|EINVAL);
            let target=if fs{P::FragmentSequence}else{P::TilerSequence};
            let notifier=&self.objects[NOTIFIER];
            let owners=sequence::Owners {
                command:region(if fs{P::FragmentCommand}else{P::TilerCommand})?,sequence:region(target)?,
                stats:self.stats[usize::from(fs)],notifier:Region::new(notifier.va()+
                    if fs && self.profile==super::m3_profile::Profile::Extended {0x200} else {0},
                    if self.profile==super::m3_profile::Profile::Extended {0x200} else {notifier.size()}).map_err(|_|EINVAL)?,
                queue:shared(if fs{s::FRAGMENT_QUEUE}else{s::TA_QUEUE})?,scene:fw(P::Scene)?,
                manager:shared(BUFFER_MANAGER)?,pool:shared(UMA_POOLS[usize::from(fs)].manager)?,
                fw_stamp:shared(if fs{s::FRAGMENT_FW_STAMP}else{s::TA_FW_STAMP})?,
                context:self._binding.slot(),stamp:timeline::stamp((if fs{s::STAMP_FRAGMENT}else{s::STAMP_TA}) - 256, draw),
            };
            let timed=self.stage_timestamps[usize::from(fs)];
            let length=if self.profile==super::m3_profile::Profile::Extended {
                let pool=&self.objects[UMA_POOLS[usize::from(fs)].manager];
                let auxiliary=super::m3_compute_layout::GpuVa::new(pool.gpu_va()?+if fs{0x200}else{0x100}).map_err(|_|EINVAL)?;
                let serial=draw.checked_mul(2).and_then(|v|v.checked_sub(u64::from(fs))).ok_or(EOVERFLOW)?;
                owners.encode_extended(fs,auxiliary,serial,&mut self.sequence_bytes).map_err(|_|EINVAL)?
            } else if fs {
                let b=pass.get(P::FragmentScratch);
                let scratch=GpuRegion::new(b.gpu_va()?,b.size()).map_err(|_|EINVAL)?;
                sequence::Fragment{owners,scratch,queue_command_count:timeline::previous(draw)}.encode_timestamps(&mut self.sequence_bytes,timed).map_err(|_|EINVAL)?
            }else{sequence::Tiler{owners,scratch:fw(P::TilerScratch)?}.encode_timestamps(&mut self.sequence_bytes,timed).map_err(|_|EINVAL)?};
            self.passes[self.slot].get_mut(target).write(0,&self.sequence_bytes)?;
            self.passes[self.slot].get_mut(if fs{P::FragmentCommand}else{P::TilerCommand}).u32(sequence::command_length_offset(fs),length as u32)?;
        }
        Ok(())
    }
    fn view_owner(&mut self,owner:ViewOwner)->&mut Buffer {
        match owner {ViewOwner::Shared(index)=>&mut self.objects[index],
            ViewOwner::Pass{slot,field}=>self.passes[slot].get_mut(field)}
    }
    fn patch_context(&mut self)->Result {
        let context=self._binding.slot();
        self.objects[NOTIFIER].u32(state::NOTIFIER_CONTEXT, context)?;
        if self.profile==super::m3_profile::Profile::Extended {
            self.objects[NOTIFIER].u32(0x200+state::NOTIFIER_CONTEXT,context)?;
        }
        Ok(())
    }
    /// Pass geometry; TPC storage is per tiler accelerator, one per GPU cluster.
    fn geometry(r:agx_uapi::UapiRenderCommand,clusters:u32)->Result<crate::m3::agx_render::Geometry> {
        use crate::m3::agx_render::{Geometry,Utile};
        let x=if r.utile_width==16 {Utile::Pixels16} else {Utile::Pixels32};
        let y=if r.utile_height==16 {Utile::Pixels16} else {Utile::Pixels32};
        let mut g=Geometry::new_layered(u32::from(r.width),u32::from(r.height),x,y,clusters,r.layers).map_err(|_|EINVAL)?;
        g.set_samples(r.samples).map_err(|_|EINVAL)?;
        Ok(g)
    }
    fn prepare_fragment(&mut self,r:agx_uapi::UapiRenderCommand,usc:u64,draw:u64)->Result {
        use crate::{m3::m3_compute_layout::GpuVa,m3::agx_render_state::{Program,DepthStencil}};
        let pass=&self.passes[self.slot];
        let shared=|index:usize| queue::FirmwareVa::new(self.objects[index].va()).map_err(|_|EINVAL);
        let fw=|field:P| queue::FirmwareVa::new(pass.get(field).va()).map_err(|_|EINVAL);
        let gpu=|field:P| GpuVa::new(pass.get(field).gpu_va()?).map_err(|_|EINVAL);
        let seq=pass.get(P::FragmentSequence);
        let command=pass.get(P::FragmentCommand);
        let geometry=Self::geometry(r,self.clusters)?;
        let mask=if r.flags&16!=0 {u64::MAX} else {u32::MAX as u64};
        let program=|p:agx_uapi::UapiProgram|->Result<Program> {Ok(Program {
            address:usc.checked_add(u64::from(p.usc&!63)).ok_or(EOVERFLOW)?,resources:p.resource_spec&mask,
        })};
        let tilebuffer_control=(u32::from(r.sample_size)*u32::from(r.utile_width)*u32::from(r.utile_height)*u32::from(r.samples)).div_ceil(2048)
            |(u32::from(r.samples.trailing_zeros() as u8)<<17);
        let tile_mode=0x280|u64::from(r.layers>1)|if r.flags&2!=0 {1<<16}else{0};
        let value=fragment::Command {
            context:self._binding.slot(),register_set:self.registers,
            sequence:Region::new(seq.va(),seq.size()).map_err(|_|EINVAL)?,
            notifier:queue::FirmwareVa::new(self.objects[NOTIFIER].va()+
                if self.profile==super::m3_profile::Profile::Extended {0x200} else {0}).map_err(|_|EINVAL)?,manager:shared(BUFFER_MANAGER)?,scene:fw(P::Scene)?,empty:if self.profile==super::m3_profile::Profile::Extended {queue::FirmwareVa::new(pass.get(P::Scene).va()+0x100).map_err(|_|EINVAL)?} else {shared(547)?},
            gpu_alias:GpuVa::new(command.gpu_va()?).map_err(|_|EINVAL)?,tilemap:gpu(P::Tilemap)?,
            heap:gpu(P::HeapMetadata)?,auxiliary:gpu(P::Auxiliary)?,scene_user:gpu(P::SceneUser)?,
            scratch:gpu(P::FragmentScratch)?,pool:shared(UMA_POOLS[1].manager)?,stamp:shared(s::FRAGMENT_STAMP)?,
            fw_stamp:shared(s::FRAGMENT_FW_STAMP)?,stamp_value:timeline::stamp(s::STAMP_FRAGMENT - 256, draw),
            timestamps:[fw(P::FragmentStart)?,fw(P::FragmentEnd)?],user_timestamps:[fw(P::FragmentUserStart)?,fw(P::FragmentUserEnd)?],
            state:fragment::State {
                geometry,multisample:r.multisample_control,merge_upper:[r.merge_upper_x,r.merge_upper_y],
                eot:program(r.end_of_tile)?,background:program(r.background)?,
                partial_eot:program(r.partial_end_of_tile)?,partial_background:program(r.partial_background)?,
                scissor:r.scissor_base,depth_bias:r.depth_bias_base,query:r.occlusion_query_base,
                isp_control:0xc000|(r.flags&(1<<18)),tilebuffer_control,mirror_tilebuffer_control:tilebuffer_control,
                tile_mode,mirror_tile_mode:tile_mode as u32,
                depth:DepthStencil{base:r.depth.base,stride:u64::from(r.depth.stride)},
                stencil:DepthStencil{base:r.stencil.base,stride:u64::from(r.stencil.stride)},
                depth_compression:DepthStencil{base:r.depth.compression_base,stride:u64::from(r.depth.compression_stride)},
                stencil_compression:DepthStencil{base:r.stencil.compression_base,stride:u64::from(r.stencil.compression_stride)},
                zls_control:r.zls_control,depth_dimensions:r.depth_dimensions,depth_clear:r.depth_clear,
                stencil_clear:r.stencil_clear,sample_size:u32::from(r.sample_size),process_empty_tiles:r.flags&2!=0,
            },
        };
        if self.profile==super::m3_profile::Profile::Extended {
            value.encode_extended(tiler::Dispatch26 {incarnation:self._binding.generation(),serial:draw.checked_mul(2).and_then(|v|v.checked_sub(1)).ok_or(EOVERFLOW)?,
                peer_stamp:0,usc_window:usc,
                firmware_aux:queue::FirmwareVa::new(self.objects[UMA_POOLS[1].manager].va()+0x200).map_err(|_|EINVAL)?,
                scene_scratch:gpu(P::FragmentScratch)?},&mut self.fragment_bytes).map_err(|_|EINVAL)?;
        } else {value.encode(&mut self.fragment_bytes).map_err(|_|EINVAL)?;}
        self.passes[self.slot].get_mut(P::FragmentCommand).write(0,&self.fragment_bytes)
    }
    pub(crate) fn set_attachments(&mut self,attachments:&[crate::m3::agx_attachments::Attachments;2])->Result {
        // Preserve the legacy renderer's existing packet format. The native
        // interface consumes separate attachment apertures for both stages.
        if self.profile!=super::m3_profile::Profile::Extended {return Ok(());}
        for (fragment,list) in attachments.iter().enumerate() {
            let field=if fragment==0 {P::TilerSequence}else{P::FragmentSequence};
            let buffer=self.passes[self.slot].get_mut(field);
            sequence::patch_extended_attachments(fragment!=0,list,|offset,data|buffer.write(offset,data))?;
        }
        crate::m3::agx_memory::publish();Ok(())
    }
    pub(crate) fn set_user_timestamps(&mut self,addresses:[[u64;2];2])->Result {
        if addresses.iter().enumerate().any(|(stage,pair)|*pair!=[0;2] && !self.stage_timestamps[stage]) {return Err(EINVAL);}
        for (stage,(command,sequence,offset,start,end)) in [
            (P::TilerCommand,P::TilerSequence,sequence::ta::USER_PAIR,sequence::ta::TIMESTAMP_START,sequence::ta::TIMESTAMP_END),
            (P::FragmentCommand,P::FragmentSequence,sequence::fragment::USER_PAIR,sequence::fragment::TIMESTAMP_START,sequence::fragment::TIMESTAMP_END)
        ].into_iter().enumerate() {
            let (offset,start,end)=if self.profile==super::m3_profile::Profile::Extended {
                if stage==0 {(0x8d8,0x1cc,0x21c)} else {(0xc20,0x1e4,0x234)}
            } else {(offset,start,end)};
            let pair=addresses[stage];
            if !self.stage_timestamps[stage] {continue;}
            self.passes[self.slot].get_mut(command).u64(offset,pair[0])?;
            self.passes[self.slot].get_mut(command).u64(offset+8,pair[1])?;
            let pointer=if pair==[0,0] {0} else {self.passes[self.slot].get_mut(command).va()+offset as u64};
            self.passes[self.slot].get_mut(sequence).u64(start+sequence::USER_TIMESTAMP_POINTER,pointer)?;
            self.passes[self.slot].get_mut(sequence).u64(end+sequence::USER_TIMESTAMP_POINTER,pointer)?;
        }
        crate::m3::agx_memory::publish();Ok(())
    }
    fn at(base:usize,index:usize)->usize {(base+index)%pass_layout::SLOTS}
    /// GPU time of the batch of `count` passes starting at pass slot `base`.
    pub(crate) fn gpu_ns(&mut self,base:usize,count:usize)->Result<u64> {
        let start=self.passes[base].get_mut(P::TilerStart).read_u64(0)?;
        let end=self.passes[Self::at(base,count-1)].get_mut(P::FragmentEnd).read_u64(0)?;
        self.profile.gpu_duration_ns(if start!=0 && end>=start {end-start} else {0}).ok_or(EOVERFLOW)
    }
    /// The batch's first tiler start and last fragment end, in 24 MHz GPU timestamp ticks.
    pub(crate) fn gpu_ticks(&mut self,base:usize,count:usize)->Result<(u64,u64)> {
        Ok((self.profile.profiler_ticks(self.passes[base].get_mut(P::TilerStart).read_u64(0)?),
            self.profile.profiler_ticks(self.passes[Self::at(base,count-1)].get_mut(P::FragmentEnd).read_u64(0)?)))
    }
    /// Read retained timestamps only after exact batch retirement. The first
    /// TA and final fragment delimit the batch in the shared 24 MHz counter.
    pub(crate) fn batch_gpu_span(&mut self)->Result<[u64;2]> {
        if self.batch_count == 0 {return Err(EINVAL);}
        let (start,end)=self.gpu_ticks(self.base,self.batch_count)?;
        Ok([start,end])
    }
    pub(crate) fn stage_ns(&mut self,base:usize,count:usize)->Result<[u64;3]> {
        let mut total=[0;3];
        for index in 0..count {
            let stages=self.slot_stage_ns(base,count,index)?;
            for i in 0..3 {total[i]+=stages[i];}
        }
        Ok(total)
    }
    pub(crate) fn slot_stage_ns(&mut self,base:usize,count:usize,index:usize)->Result<[u64;3]> {
        if index>=count {return Err(EINVAL);}
        let slot=Self::at(base,index);
        let duration=|a:u64,b:u64| self.profile.gpu_duration_ns(if a!=0 && b>=a {b-a} else {0}).unwrap_or(0);
        let fs_start=self.passes[slot].get_mut(P::FragmentStart).read_u64(0)?;
        let fs_end=self.passes[slot].get_mut(P::FragmentEnd).read_u64(0)?;
        let ta_start=self.passes[slot].get_mut(P::TilerStart).read_u64(0)?;
        let ta_end=self.passes[slot].get_mut(P::TilerEnd).read_u64(0)?;
        Ok([duration(ta_start,ta_end),duration(fs_start,fs_end),duration(ta_end,fs_start)])
    }
    pub(crate) fn early_count(&self)->u32 {self.early_count}
    pub(crate) fn heads(&self)->[u16;2] {self.heads}
    pub(crate) fn ordinal(&self)->u64 {self.draw}
    /// The current batch: first pass slot, pass count and last draw number.
    pub(crate) fn batch(&self)->(usize,usize,u64) {(self.base,self.batch_count,self.draw)}
    /// Whether draws 1..=`draw` have completed both stages (TA and fragment stamps, host and
    /// firmware copies, have reached `draw`'s values). Use signed modular ordering
    /// across stamp wrap; the runtime bounds outstanding passes to SLOTS.
    pub(crate) fn complete_through(&mut self,draw:u64)->Result<bool> {
        if draw==0 {return Ok(true);}
        for (stamp,fw,value) in [(s::TA_STAMP,s::TA_FW_STAMP,timeline::stamp(s::STAMP_TA-256,draw)),
            (s::FRAGMENT_STAMP,s::FRAGMENT_FW_STAMP,timeline::stamp(s::STAMP_FRAGMENT-256,draw))] {
            if !timeline::reached(self.objects[stamp].read_u32(0)?,value) || !timeline::reached(self.objects[fw].read_u32(0)?,value) {return Ok(false);}
        }
        Ok(true)
    }
    /// Whether any draw has been published (the first one initializes the buffer manager).
    pub(crate) fn started(&self)->bool {self.draw>0}
    pub(crate) fn first(&self)->bool {
        self.profile==super::m3_profile::Profile::Legacy && self.draw==self.batch_count as u64
    }
    /// Start a batch in pass slots `base..`. With `overlap`, earlier batches may still run: then
    /// the VM must already be bound, no storage may grow and the slots must not be in flight
    /// (the caller keeps the in-flight passes plus this batch within SLOTS).
    pub(crate) fn begin_batch(&mut self,dev:&driver::AsahiDrmDevice,uat:&mmu::Uat,vm:&mmu::Vm,
        commands:&[crate::m3::m3_submit::Command],base:usize,overlap:bool)->Result {
        if base>=pass_layout::SLOTS || (!overlap && !self.complete()?) {return Err(EBUSY);}
        if overlap {
            if !self._binding.matches(vm) {return Err(EBUSY);}
            for (index,&command) in commands.iter().enumerate() {
                let crate::m3::m3_submit::Command::Render{command:r,..}=command else {return Err(EINVAL);};
                let g=Self::geometry(r,self.clusters)?;
                let slot=Self::at(base,index);
                if g.tpc_bytes>self.passes[slot].get(P::Tpc).size() as u64
                    || g.tilemap_bytes>self.passes[slot].get(P::Tilemap).size() as u64 {return Err(EBUSY);}
            }
        }
        if commands.is_empty() || commands.len()>pass_layout::SLOTS {return Err(E2BIG);}
        self.draw.checked_add(commands.len() as u64).ok_or(EOVERFLOW)?;
        // Validate every pass before changing any queue or completion state.
        for &command in commands {
            let crate::m3::m3_submit::Command::Render{command:r,..}=command else {return Err(EINVAL);};
            Self::geometry(r,self.clusters)?;
            crate::m3::agx_render_state::compact(r.vdm_base).map_err(|_|EINVAL)?;
        }
        if !self._binding.matches(vm) {
            let next=if let Some(index)=self.cached_views.iter().position(|v|v.binding.matches(vm)) {
                self.cached_views.remove(index).map_err(|_|EIO)?
            } else {
                let binding=uat.bind(vm)?;
                let mut views=KVec::new();let mut aliases=KVec::new();
                for i in 0..s::COUNT {
                    let a=s::allocation_for(self.profile,i).map_err(|_|EINVAL)?;
                    if a.gpu_alias.is_some() {aliases.push(self.objects[i].map_gpu_view(vm)?,GFP_KERNEL)?;}
                    if a.space==Space::ClientGpu {
                        views.push((ViewOwner::Shared(i),self.objects[i].map_gpu_view(vm)?),GFP_KERNEL)?;
                    }
                }
                for (slot,pass) in self.passes.iter_mut().enumerate() {
                    for field in pass_layout::FIELDS {
                        let a=pass_layout::allocation_for(self.profile,slot,field,self.clusters).map_err(|_|EINVAL)?;
                        if a.space==Space::ClientGpu {views.push((ViewOwner::Pass{slot,field},pass.get_mut(field).map_gpu_view(vm)?),GFP_KERNEL)?;}
                        if a.gpu_alias.is_some() {aliases.push(pass.get_mut(field).map_gpu_view(vm)?,GFP_KERNEL)?;}
                    }
                }
                CachedViews{binding,aliases,views}
            };
            // Finish fallible allocations before replacing the current owner.
            let old_views=KVec::with_capacity(next.views.len(),GFP_KERNEL)?;
            self.cached_views.reserve(1,GFP_KERNEL)?;
            if self.cached_views.len()==3 {drop(self.cached_views.remove(0).map_err(|_|EIO)?);}
            let mut old=CachedViews {
                binding:core::mem::replace(&mut self._binding,next.binding),
                aliases:core::mem::replace(&mut self._aliases,next.aliases),
                views:old_views,
            };
            for (i,view) in next.views {
                old.views.push((i,self.view_owner(i).swap_gpu_view(view)?),GFP_KERNEL)?;
            }
            self.cached_views.push(old,GFP_KERNEL)?;
        }
        // Growth is legal only after exact retirement, and before append.
        // Cached VM mappings must be dropped before freeing the old backing.
        // Partial allocation failure leaves retired queue/stamp state intact;
        // already-grown buffers are reusable on the next submission.
        for (index,&command) in commands.iter().enumerate() {
            let slot=Self::at(base,index);
            let crate::m3::m3_submit::Command::Render{command:r,..}=command else {return Err(EINVAL);};
            let g=Self::geometry(r,self.clusters)?;
            for (field,base,bytes) in [(P::Tpc,s::TPC_LABEL,g.tpc_bytes),(P::Tilemap,s::TILEMAP_LABEL,g.tilemap_bytes)] {
                let size=usize::try_from(bytes).map_err(|_|EOVERFLOW)?;
                if size>self.passes[slot].get(field).size() {
                    let buffer=Buffer::tiler(dev,uat.kernel_vm(),vm,size)?;
                    self.cached_views.clear();
                    dev_info!(dev.as_ref(),"M3 render arena growth: slot={} object={} {} -> {} bytes GPU={:#x}\n",
                        slot,base,self.passes[slot].get(field).size(),size,buffer.gpu_va()?);
                    *self.passes[slot].get_mut(field)=buffer;
                }
            }
        }
        self.checkpoint=(self.draw,self.heads,self.batch_count,self.slot,self.early_count,self.base);
        self.batch_count=0;
        self.base=base;
        Ok(())
    }
    fn publish_counts(&mut self, draw:u64) -> Result {
        if self.profile==super::m3_profile::Profile::Extended {
            for offset in [0,8] {self.objects[s::EVENT_COUNT].u64(offset,draw)?;}
            for counter in [TA_POOL_COUNTER,FRAGMENT_POOL_COUNTER] {self.objects[counter].u64(0,draw)?;}
        } else {
            self.objects[s::BM_COUNTER].u32(0,draw as u32)?;
            self.objects[s::EVENT_COUNT].u32(0,timeline::events(draw,2))?;
        }
        Ok(())
    }

    /// Restore producer state only when no batch is in flight. Running firmware can
    /// consume append's WRITE updates without a new doorbell, so overlap cannot abort.
    pub(crate) fn abort_batch(&mut self)->Result {
        (self.draw,self.heads,self.batch_count,self.slot,self.early_count,self.base)=self.checkpoint;
        self.publish_counts(self.draw)?;
        for (stage,pointers) in [s::TA_POINTERS,s::FRAGMENT_POINTERS].into_iter().enumerate() {
            self.objects[pointers].u32(queue::WRITE,u32::from(self.heads[stage]))?;
        }
        crate::m3::agx_memory::publish();
        Ok(())
    }
    /// Append only into caller-reserved free pass slots. With overlap, WRITE updates
    /// are immediately visible to running firmware, even before a new doorbell.
    /// Each append selects disjoint host-mutated pass storage. A TA dependency
    /// retains full fragment-to-next-TA ordering for arbitrary resource hazards.
    pub(crate) fn append(&mut self,r:agx_uapi::UapiRenderCommand,usc:u64,coalesce_stamps:bool,timestamps:[bool;2])->Result {
        self.append_after(r,usc,coalesce_stamps,None,timestamps)
    }
    pub(crate) fn completion_dependency(&self)->Result<sync::CompletionDependency> {
        Ok(sync::CompletionDependency {
            stamp:queue::FirmwareVa::new(self.objects[s::FRAGMENT_FW_STAMP].va()).map_err(|_|EINVAL)?,
            value:timeline::stamp(s::STAMP_FRAGMENT-256,self.draw),event:1,
        })
    }
    pub(crate) fn append_after(&mut self,r:agx_uapi::UapiRenderCommand,usc:u64,
        coalesce_stamps:bool,predecessor:Option<sync::CompletionDependency>,timestamps:[bool;2])->Result {
        if predecessor.is_some_and(|p|p.event!=2) {return Err(EINVAL);}
        if self.batch_count>=pass_layout::SLOTS {
            pr_err!("M3 render batch limit: batch={} slots={}\n",
                self.batch_count,pass_layout::SLOTS);
            return Err(E2BIG);
        }
        self.slot=Self::at(self.base,self.batch_count);
        self.stage_timestamps=timestamps;
        // Retain the conservative stage-scope permission. Only a pass after the
        // first of its batch may overlap its predecessor's stages. Explicit
        // start timestamps keep the original full-order execution boundary.
        let early=predecessor.is_none() && self.batch_count>0 && crate::m3::m3_params::early_tiling()
            && r.flags&(1<<5)!=0 && r.vertex_timestamps.start.handle==0
            && r.fragment_timestamps.start.handle==0;
        if early {self.early_count+=1;}
        // Only reset per-draw objects. Pool allocators, PB lists, queues,
        // notification state and previous stamps retain firmware ownership.
        self.passes[self.slot].reset(&Self::geometry(r,self.clusters)?)?;
        self.init_bm.fill(0)?; // retain the prior shared InitBM reset before encoding
        self.draw=self.draw.checked_add(1).ok_or(EOVERFLOW)?;
        self.batch_count+=1;
        let draw=self.draw;
        self.publish_counts(draw)?;
        if self.profile==super::m3_profile::Profile::Extended {
            let pass=&self.passes[self.slot];
            let scratch=pass.get(P::FragmentScratch);
            let scratch_gpu=GpuRegion::new(scratch.gpu_va()?,scratch.size()).map_err(|_|EINVAL)?;
            let scratch_fw=queue::FirmwareVa::new(scratch.va()).map_err(|_|EINVAL)?;
            let list=pass.get(P::SceneUser);
            let list=GpuRegion::new(list.gpu_va()?,list.size()).map_err(|_|EINVAL)?;
            let metadata=pass.get(P::Scene);
            let owner=Region::new(metadata.va(),metadata.size()).map_err(|_|EINVAL)?;
            let mut bytes=[0;scene::EXTENDED_SIZE];
            scene::encode_extended(&mut bytes,owner,scratch_gpu,scratch_fw,list).map_err(|_|EINVAL)?;
            self.passes[self.slot].get_mut(P::Scene).write(0,&bytes)?;
        } else {
        let entry_offset = (self.draw % u64::from(MANAGER_SCENE_COUNT)) as usize * 4;
        let scene_list = &mut self.objects[s::BM_SCENES];
        if entry_offset + 4 > scene_list.size() { return Err(ERANGE); }
        let manager_entry = queue::FirmwareVa::new(scene_list.va().checked_add(entry_offset as u64)
            .ok_or(EOVERFLOW)?).map_err(|_| EINVAL)?;
        let manager_misc = queue::FirmwareVa::new(self.objects[MANAGER_MISC].va()).map_err(|_| EINVAL)?;
        let user_buffer = self.passes[self.slot].get_mut(P::SceneUser);
        let user_buffer = scene::UserBuffer::new(user_buffer.gpu_va()?, user_buffer.size()).map_err(|_| EINVAL)?;
        let mut bytes = [0; scene::SIZE];
        scene::Scene { manager_entry, manager_misc, user_buffer }.encode(&mut bytes).map_err(|_| EINVAL)?;
        self.passes[self.slot].get_mut(P::Scene).write(0, &bytes)?;
        }
        self.prepare_tiler(r,usc,draw)?;
        self.patch_context()?;
        self.prepare_fragment(r,usc,draw)?;
        self.prepare_sequences(draw)?;
        self.prepare_sync(early, draw, predecessor)?;
        // Encode before either ring WRITE exposes the command to firmware.
        // The caller retains every pass owner until the final flushed stamps,
        // queues, events and engine-idle checks retire this entire batch.
        if coalesce_stamps {
            self.passes[self.slot].get_mut(P::TilerCommand).u32(tiler::FLUSH_STAMPS,0)?;
            self.passes[self.slot].get_mut(P::FragmentCommand).u32(fragment::FLUSH_STAMPS,0)?;
        }
        let dep=P::TilerDependency;
        let fs_dep=P::FragmentDependency;
        let ta=self.passes[self.slot].get_mut(P::TilerCommand).va();
        let fragment=self.passes[self.slot].get_mut(P::FragmentCommand).va();
        let barrier=self.passes[self.slot].get_mut(P::TilerToFragment).va();
        let dependency=self.passes[self.slot].get(dep).va();
        let init=self.init_bm.va();
        let ta_commands=[init,dependency,ta];
        let fs_commands=[self.passes[self.slot].get(fs_dep).va(),barrier,fragment];
        for (stage,ring,pointers,addresses) in [
            (0,s::TA_RING,s::TA_POINTERS,&ta_commands[usize::from(draw!=1)..]),
            (1,s::FRAGMENT_RING,s::FRAGMENT_POINTERS,&fs_commands[usize::from(!early)..])] {
            for &address in addresses {
                self.objects[ring].u64(usize::from(self.heads[stage])*8,address)?;
                self.heads[stage]=(self.heads[stage]+1)%0x500;
            }
            crate::m3::agx_memory::publish();
            self.objects[pointers].u32(queue::WRITE,u32::from(self.heads[stage]))?;
        }
        crate::m3::agx_memory::publish();crate::mem::sync();crate::mem::tlbi_all();crate::mem::sync();Ok(())
    }
    /// Emitted only after both events, stamp/queue checks, clear faults,
    /// idle engines and consumed firmware pipes have been checked by Runtime.
    /// The existing TA/fragment descriptors request completion stamp flushes.
    pub(crate) fn progress(&mut self,dev:&driver::AsahiDrmDevice)->Result {
        // Keep summaries sparse at high throughput, but emit at least once a
        // second while completed batches advance. Batched draw counts can skip
        // multiples of 128; a count-only gate can starve the watchdog observer.
        if !crate::debug::debug_enabled(crate::debug::DebugFlags::SubmitTiming) {
            return Ok(());
        }
        let now = <kernel::time::Monotonic as kernel::time::ClockSource>::ktime_get();
        if self.last_progress_ns != 0 && now - self.last_progress_ns < 1_000_000_000
            && self.draw != 1 && self.draw % 128 != 0
            && !crate::debug::debug_enabled(crate::debug::DebugFlags::SubmitTiming) {
            return Ok(());
        }
        self.last_progress_ns = now;
        dev_info!(dev.as_ref(),"M3_RETIRED draw={} ta={:x}/{:x} fragment={:x}/{:x} queues={}/{} read={}/{} idle=1 cache_flush=1 events=2 faults=0 ordered_barrier=1 batch={} early={}\n",
            self.draw,self.objects[s::TA_STAMP].read_u32(0)?,self.objects[s::TA_FW_STAMP].read_u32(0)?,
            self.objects[s::FRAGMENT_STAMP].read_u32(0)?,self.objects[s::FRAGMENT_FW_STAMP].read_u32(0)?,
            self.objects[s::TA_POINTERS].read_u32(queue::DONE)?,self.objects[s::FRAGMENT_POINTERS].read_u32(queue::DONE)?,
            self.objects[s::TA_POINTERS].read_u32(queue::READ)?,self.objects[s::FRAGMENT_POINTERS].read_u32(queue::READ)?,self.batch_count,self.early_count);
        Ok(())
    }
    pub(crate) fn queues(&self)->[u64;2] {[self.objects[s::TA_QUEUE].va(),self.objects[s::FRAGMENT_QUEUE].va()]}
    /// Read retained command bytes only after exact batch retirement.
    pub(crate) fn stamp_flush_masks(&mut self,base:usize,count:usize)->Result<[u16;2]> {
        if base>=pass_layout::SLOTS || count==0 || count>pass_layout::SLOTS {return Err(EINVAL);}
        let mut masks=[0;2];
        for slot in 0..count {
            let pass=&mut self.passes[Self::at(base,slot)];
            for (stage,field,offset) in [(0,P::TilerCommand,tiler::FLUSH_STAMPS),
                (1,P::FragmentCommand,fragment::FLUSH_STAMPS)] {
                if pass.get_mut(field).read_u32(offset)?!=0 {masks[stage]|=1u16<<slot;}
            }
        }
        Ok(masks)
    }
    /// Read the fragment host and firmware stamps without changing retirement.
    pub(crate) fn fragment_stamp_parts(&mut self)->Result<[bool;2]> {
        let value=timeline::stamp(s::STAMP_FRAGMENT - 256,self.draw);
        Ok([self.objects[s::FRAGMENT_STAMP].read_u32(0)?==value,
            self.objects[s::FRAGMENT_FW_STAMP].read_u32(0)?==value])
    }
    /// Observe each completion condition across both render queues independently.
    pub(crate) fn completion_parts(&mut self)->Result<[bool;4]> {
        let mut done=[true;4];
        for (stamp,fw,pointers,value,head) in [(s::TA_STAMP,s::TA_FW_STAMP,s::TA_POINTERS,timeline::stamp(s::STAMP_TA - 256, self.draw),self.heads[0]),
            (s::FRAGMENT_STAMP,s::FRAGMENT_FW_STAMP,s::FRAGMENT_POINTERS,timeline::stamp(s::STAMP_FRAGMENT - 256, self.draw),self.heads[1])] {
            done[0]&=self.objects[stamp].read_u32(0)?==value;
            done[1]&=self.objects[fw].read_u32(0)?==value;
            done[2]&=self.objects[pointers].read_u32(queue::DONE)?==u32::from(head);
            done[3]&=self.objects[pointers].read_u32(queue::READ)?==u32::from(head);
        }
        Ok(done)
    }
    pub(crate) fn complete(&mut self)->Result<bool> {
        for (stamp,fw,pointers,value,head) in [(s::TA_STAMP,s::TA_FW_STAMP,s::TA_POINTERS,timeline::stamp(s::STAMP_TA - 256, self.draw),self.heads[0]),
            (s::FRAGMENT_STAMP,s::FRAGMENT_FW_STAMP,s::FRAGMENT_POINTERS,timeline::stamp(s::STAMP_FRAGMENT - 256, self.draw),self.heads[1])] {
            if self.objects[stamp].read_u32(0)?!=value || self.objects[fw].read_u32(0)?!=value
                || self.objects[pointers].read_u32(queue::DONE)?!=u32::from(head)
                || self.objects[pointers].read_u32(queue::READ)?!=u32::from(head) {return Ok(false);}
        }
        if self.profile==super::m3_profile::Profile::Extended && self.draw!=0 {
            for pool in UMA_POOLS {
                if self.objects[pool.manager].read_u64(0x54)?!=self.draw {return Ok(false);}
            }
        }
        Ok(true)
    }
    pub(crate) fn log(&mut self,dev:&driver::AsahiDrmDevice)->Result {
        // Retain allocator state as well as payloads: an exhausted parameter
        // pool can stall TA without producing an MMU fault.
        for (index,bytes) in [(BLOCK_CONTROL,64),(s::BM_COUNTER,64),(MANAGER_MISC,64),(BUFFER_MANAGER,172)] {
            for offset in (0..bytes).step_by(4) {
                let value=self.objects[index].read_u32(offset)?;
                dev_info!(dev.as_ref(),"M3_PB object={} offset={:#x} value={:#x}\n",index,offset,value);
            }
        }
        dev_info!(dev.as_ref(),"M3 fault render VM={} slot={} root={:#x} draw={} batch={}\n",self._binding.vm_id(),self._binding.slot(),self._binding.root(),self.draw,self.batch_count);
        for index in 0..self.batch_count {
            let slot=Self::at(self.base,index);
            let command=self.passes[slot].get_mut(P::TilerCommand);
            for index in 0..tiler::REGISTER_COUNT {
                let offset=tiler::REGISTERS+index*tiler::REGISTER_STRIDE;
                if command.read_u32(offset)?==0x1c880 {
                    dev_info!(dev.as_ref(),"M3 fault pass slot={} compact_vdm={:#x}\n",slot,command.read_u64(offset+4)?);
                }
            }
        }
        if crate::debug::debug_enabled(crate::debug::DebugFlags::VerboseFaults) {
        // Preserve historic diagnostic IDs while selecting named owners.
        for (index,bytes) in s::fault_buffers() {
            let b=match index {s::TILEMAP_LABEL=>self.passes[0].get_mut(P::Tilemap),
                s::HEAP_METADATA_LABEL=>self.passes[0].get_mut(P::HeapMetadata),_=>&mut self.objects[index]};
            for offset in (0..bytes).step_by(16) {
                let lo=b.read_u64(offset)?;
                let hi=b.read_u64(offset+8)?;
                if lo!=0 || hi!=0 {dev_info!(dev.as_ref(),"M3_BUFFER object={} offset={:#x} {:016x} {:016x}\n",index,offset,lo,hi);}
            }
        }
        }
        for (name,stamp,fw,pointers) in [("TA",s::TA_STAMP,s::TA_FW_STAMP,s::TA_POINTERS),
            ("fragment",s::FRAGMENT_STAMP,s::FRAGMENT_FW_STAMP,s::FRAGMENT_POINTERS)] {
            dev_info!(dev.as_ref(),"M3 {} stamps={:#x}/{:#x} done={} read={}\n",name,
                self.objects[stamp].read_u32(0)?,self.objects[fw].read_u32(0)?,
                self.objects[pointers].read_u32(queue::DONE)?,self.objects[pointers].read_u32(queue::READ)?);
        } Ok(())
    }
}
