// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Owned RTKit 2419 M3 compute graph, populated with a validated user CDM.
use kernel::prelude::*;
use crate::m3::m3_queue_layout as queue;
use crate::m3::m3_state_layout as state;
use crate::{m3::m3_compute_layout as command, m3::m3_compute_sequence as microsequence,
    m3::m3_pool_layout::GpuRegion, m3::m3_init_layout::Region};
use crate::{driver,mmu,m3::agx_memory::{self},m3::m3_memory::Buffer,m3::m3_compute_storage as storage};
use crate::m3::m3_timeline as timeline;
use storage::{NOTIFIER,PREEMPTION,GPU_CONTEXT,JOB_LIST,QUEUE_SCRATCH,TIMESTAMPS,
    TAIL_SCRATCH,MICROSEQUENCE_SCRATCH,Mapping};
const UMA_POOL:crate::m3::m3_pool::Owners=crate::m3::m3_pool::Owners {
    backing:storage::POOL_BACKING,pages:storage::POOL_PAGES,table:storage::POOL_TABLE,
    manager:storage::POOL,counter:storage::COUNTER,table_index:2,
};
// Retain at most four VM views, including the active one, as the render
// graph does. Game and compositor compute jobs otherwise remap the whole
// private arena every time they alternate. Each entry pins its VM binding.
struct CachedViews {
    command: mmu::KernelMapping,
    views: KVec<(usize, mmu::KernelMapping)>,
    // Release cached mappings before returning their ASID lease.
    binding: mmu::VmBind,
}
pub(crate) struct Compute {
    _client_command: mmu::KernelMapping,
    objects: KVec<Buffer>,
    command_bytes: KVec<u8>,
    sequence_bytes: KVec<u8>,
    _binding: mmu::VmBind,
    stats: Region,
    sequence: u64,
    last_progress_ns: i64,
    head: u16,
    cached_views: KVec<CachedViews>,
    slot: usize,
    batch_count: usize,
    checkpoint: (u64,u16,usize),
    /// Firmware-only barrier storage, allocated only for mixed publications.
    barriers: Option<Buffer>,
    /// The register lists of the SoC's GPU (`m3_soc::Soc::registers`).
    registers: command::RegisterSet,
    profile: super::m3_profile::Profile,
    /// Last: drops after the command aliases.
    _lower_tlbi: crate::m3::m3_memory::LowerTlbi,
}
impl Compute {
    pub(crate) fn new(dev: &driver::AsahiDrmDevice, uat: &mmu::Uat, vm: &mmu::Vm,
        stats: Region, control: crate::m3::agx_compute::Control,
        registers: command::RegisterSet, profile: super::m3_profile::Profile) -> Result<Self> {
        let lower_tlbi=crate::m3::m3_memory::LowerTlbi;
        let binding=uat.bind(vm)?;
        let mut objects=KVec::new();
        let mut client_command=None;
        let mut command_bytes = KVec::new();
        command_bytes.resize(if profile == super::m3_profile::Profile::Extended {
            command::EXTENDED_SIZE
        } else { command::SIZE }, 0u8, GFP_KERNEL)?;
        let mut sequence_bytes = KVec::new();
        sequence_bytes.resize(microsequence::STORAGE, 0u8, GFP_KERNEL)?;
        // A private compute aperture, disjoint from the retained render graph.
        // Address-bearing graph fields are constructed from their live owners.
        for index in 0..storage::COUNT {
            let a=storage::batch_allocation_for(profile,index).map_err(|_|EINVAL)?;
            let mut b=match a.mapping {
                Mapping::CommandAlias(va)=>Buffer::at(dev,uat.kernel_vm(),Some((uat.kernel_lower_vm(),va)),None,a.size,true)?,
                Mapping::Client(va)=>Buffer::at(dev,uat.kernel_vm(),Some((vm,va)),None,a.size,true)?,
                Mapping::FirmwareGpuShared=>Buffer::at(dev,uat.kernel_vm(),None,None,a.size,true)?,
                Mapping::FirmwareOnly=>Buffer::at(dev,uat.kernel_vm(),None,None,a.size,false)?,
            };
            if matches!(a.mapping,Mapping::CommandAlias(_)) {client_command=Some(b.map_gpu_view(vm)?);}
            objects.push(b,GFP_KERNEL)?;
        }
        Self::encode_command(&mut objects, &mut command_bytes, binding.slot(), control, 1, 0,
            registers, profile, binding.generation())?;
        Self::encode_sequence(&mut objects, &mut sequence_bytes, stats, binding.slot(), timeline::stamp(0xc1000000,1), 0, profile, 0)?;
        if profile == super::m3_profile::Profile::Extended {
            UMA_POOL.initialize_extended(&mut objects, 1, true)?;
        } else { UMA_POOL.initialize(&mut objects)?; }
        Self::initialize_state(&mut objects, binding.slot(), profile)?;
        // Queue references are firmware VAs from live allocations. In contrast
        // to render's unpublished seed entries, compute dispatches entry zero
        // immediately, so its initial producer cursor remains one.
        let address = |index: usize| queue::FirmwareVa::new(objects[index].va()).map_err(|_| EINVAL);
        let info = queue::QueueInfo { state: address(storage::QUEUE_STATE)?, ring: address(storage::RING)?,
            job_list: address(JOB_LIST)?, gpu_scratch: address(QUEUE_SCRATCH)?,
            context: address(GPU_CONTEXT)?, uuid: 0xc10000 };
        let command = address(storage::COMMAND)?;
        let mut bytes = KVec::new();
        bytes.resize(if profile == super::m3_profile::Profile::Extended {
            queue::QueueInfo::EXTENDED_SIZE
        } else { queue::INFO_SIZE }, 0u8, GFP_KERNEL)?;
        if profile == super::m3_profile::Profile::Extended {
            info.encode_extended(address(storage::QUEUE)?, &mut bytes).map_err(|_| EINVAL)?;
        } else { info.encode(&mut bytes).map_err(|_| EINVAL)?; }
        objects[storage::QUEUE].write(0, &bytes)?;
        let mut state = [0; queue::STATE_SIZE];
        queue::RingState { capacity: queue::QUALIFIED_CAPACITY, done: 0, read: 0, write: 1 }
            .encode(&mut state).map_err(|_| EINVAL)?;
        objects[storage::QUEUE_STATE].write(0, &state)?;
        for index in 0..queue::QUALIFIED_CAPACITY {
            let entry = queue::ring_entry(index, queue::QUALIFIED_CAPACITY, &[command]).map_err(|_| EINVAL)?;
            objects[storage::RING].write(index as usize * 8, &entry)?;
        }
        dev_info!(dev.as_ref(),"M3 compute graph: context={} queue={:#x} command={:#x} low={:#x} cdm={:#x}..{:#x} pool={:#x}\n",
            binding.slot(),objects[storage::QUEUE].va(),objects[storage::COMMAND].va(),
            objects[storage::COMMAND].gpu_va()?,control.base,control.end,objects[storage::POOL].va());
        // The M3 compute stream uses absolute IOTO addresses and does not
        // include the USC_EXEC_BASE_CP register.
        // Diagnostic only: read no further than the stream, and never fail the job over it.
        let mut stream=[0u8;48];
        let len=core::cmp::min(control.end.saturating_sub(control.base),stream.len() as u64) as usize;
        if vm.read_bytes(control.base,&mut stream[..len]).is_ok() {
            crate::cls_dev_dbg!(Compute,dev,"M3 client CDM bytes={:02x?}\n",&stream[..len]);
        }
        agx_memory::publish();
        // Finish the write-combining page-table stores before broadcasting
        // the invalidation.
        crate::mem::sync();crate::mem::tlbi_all();crate::mem::sync();
        unsafe {core::arch::asm!("isb",options(nostack,preserves_flags))};
        Ok(Self {_client_command:client_command.ok_or(EINVAL)?,objects,command_bytes,sequence_bytes,_binding:binding,stats,last_progress_ns:0,sequence:1,head:1,cached_views:KVec::new(),slot:0,batch_count:1,checkpoint:(0,0,0),barriers:None,registers,profile,_lower_tlbi:lower_tlbi})
    }
    fn encode_command(objects: &mut [Buffer], bytes: &mut [u8], context: u32,
        control: crate::m3::agx_compute::Control, sequence: u64, slot: usize,
        register_set: command::RegisterSet, profile: super::m3_profile::Profile, generation: u8) -> Result {
        let offset = |owner:usize| storage::slot_offset_for(profile,owner,slot).map_err(|_|EINVAL);
        let fw = |owner: usize| queue::FirmwareVa::new(objects[owner].va()+offset(owner)? as u64).map_err(|_| EINVAL);
        let preemption = &objects[PREEMPTION];
        let value = command::Command {
            context, counter: timeline::previous(sequence), notifier: fw(NOTIFIER)?,
            preemption: GpuRegion::new(preemption.gpu_va()?+offset(PREEMPTION)? as u64, storage::allocation_for(profile,PREEMPTION).map_err(|_|EINVAL)?.size).map_err(|_| EINVAL)?,
            cdm: command::GpuVa::new(control.base).map_err(|_| EINVAL)?,
            cdm_last: command::GpuVa::new(control.end.checked_sub(4).ok_or(EINVAL)?).map_err(|_| EINVAL)?,
            gpu_alias: command::GpuVa::new(objects[storage::COMMAND].gpu_va()?+offset(storage::COMMAND)? as u64).map_err(|_| EINVAL)?,
            microsequence: fw(storage::MICROSEQUENCE)?, microsequence_size:
                if profile == super::m3_profile::Profile::Extended { microsequence::Sequence::EXTENDED_LENGTH as u32 }
                else { microsequence::LENGTH as u32 },
            stamp: fw(storage::STAMP)?, fw_stamp: fw(storage::FW_STAMP)?,
            stamp_value: timeline::stamp(0xc1000000,sequence),
            timestamps: [fw(TIMESTAMPS[0])?, fw(TIMESTAMPS[1])?],
            pool: fw(storage::POOL)?, tail_scratch: fw(TAIL_SCRATCH)?, flush_stamps: false,
            register_set,
        };
        value.encode_with_usc(profile, generation, 0,
            if profile == super::m3_profile::Profile::Extended { control.usc_base } else { 0 }, bytes).map_err(|_| EINVAL)?;
        objects[storage::COMMAND].write(offset(storage::COMMAND)?, bytes)
    }
    fn encode_sequence(objects: &mut [Buffer], bytes: &mut [u8], stats: Region,
        context: u32, stamp_value: u32, slot: usize, profile: super::m3_profile::Profile,
        queue_sequence: u64) -> Result {
        let offset = |owner:usize| storage::slot_offset_for(profile,owner,slot).map_err(|_|EINVAL);
        let fw = |owner: usize| queue::FirmwareVa::new(objects[owner].va()+offset(owner)? as u64).map_err(|_| EINVAL);
        let region = |owner: usize| Region::new(objects[owner].va()+offset(owner)? as u64, storage::allocation_for(profile,owner).map_err(|_|EINVAL)?.size).map_err(|_| EINVAL);
        let value = microsequence::Sequence {
            command: region(storage::COMMAND)?, stats, notifier: region(NOTIFIER)?,
            queue: fw(storage::QUEUE)?, context,
            generation: if profile == super::m3_profile::Profile::Extended { 0 } else { 1 }, pool: fw(storage::POOL)?,
            scratch: [fw(MICROSEQUENCE_SCRATCH[0])?, fw(MICROSEQUENCE_SCRATCH[1])?,
                fw(MICROSEQUENCE_SCRATCH[2])?, fw(MICROSEQUENCE_SCRATCH[3])?, fw(MICROSEQUENCE_SCRATCH[4])?],
            fw_stamp: fw(storage::FW_STAMP)?, stamp_value,
        };
        if profile == super::m3_profile::Profile::Extended {
            let auxiliary = command::GpuVa::new(objects[TAIL_SCRATCH].gpu_va()? + offset(TAIL_SCRATCH)? as u64).map_err(|_| EINVAL)?;
            let context_store = GpuRegion::new(objects[MICROSEQUENCE_SCRATCH[1]].gpu_va()?
                + offset(MICROSEQUENCE_SCRATCH[1])? as u64, 0x40000).map_err(|_| EINVAL)?;
            value.encode_extended(auxiliary, context_store, queue_sequence, bytes).map_err(|_| EINVAL)?;
        } else { value.encode(bytes).map_err(|_| EINVAL)?; }
        objects[storage::MICROSEQUENCE].write(offset(storage::MICROSEQUENCE)?, bytes)
    }
    fn initialize_state(objects: &mut [Buffer], context_slot: u32, profile: super::m3_profile::Profile) -> Result {
        let address = |owner: usize| queue::FirmwareVa::new(objects[owner].va()).map_err(|_| EINVAL);
        let notification = state::Notifier { threshold: address(storage::EVENT)?, context: context_slot };
        let list_owner = address(JOB_LIST)?;
        let mut context = [0; state::CONTEXT_SIZE];
        if profile == super::m3_profile::Profile::Extended {
            state::context_extended(&mut context).map_err(|_| EINVAL)?;
        } else { state::context(&mut context).map_err(|_| EINVAL)?; }
        let context_size = if profile == super::m3_profile::Profile::Extended {
            state::EXTENDED_CONTEXT_SIZE
        } else { state::CONTEXT_SIZE };
        objects[GPU_CONTEXT].write(0, &context[..context_size])?;
        let mut notifier = KVec::new();
        notifier.resize(if profile == super::m3_profile::Profile::Extended {
            state::EXTENDED_NOTIFIER_SIZE
        } else { state::NOTIFIER_SIZE }, 0u8, GFP_KERNEL)?;
        if profile == super::m3_profile::Profile::Extended {
            notification.encode_extended(&mut notifier).map_err(|_| EINVAL)?;
        } else { notification.encode(&mut notifier).map_err(|_| EINVAL)?; }
        objects[NOTIFIER].write(0, &notifier)?;
        let mut list = [0; state::JOB_LIST_SIZE];
        state::job_list(&mut list, list_owner).map_err(|_| EINVAL)?;
        objects[JOB_LIST].write(0, &list)?;
        for (owner, value) in [(storage::COUNTER, 1), (storage::EVENT, 1), (storage::STAMP, timeline::stamp(0xc1000000,0))] {
            let mut bytes = [0; state::MANAGER_STATE_SIZE];
            let bytes = bytes.get_mut(..objects[owner].size()).ok_or(EINVAL)?;
            state::counter(bytes, value).map_err(|_| EINVAL)?;
            objects[owner].write(0, bytes)?;
        }
        // Fill the preemption buffer with 0xcc, including when a compute
        // command is submitted again.
        objects[PREEMPTION].fill(0xcc)
    }
    pub(crate) fn replay(&mut self, uat: &mmu::Uat, vm: &mmu::Vm,
        control: crate::m3::agx_compute::Control) -> Result {
        if !self.complete()? { return Err(EBUSY); }
        // All prior flushed stamps/events and engine idle were checked by
        // Runtime before entry. Keep pool/queue ownership, remap only client
        // views and patch the context when switching Vulkan processes.
        if !self._binding.matches(vm) {
            let next = if let Some(index) = self.cached_views.iter().position(|v| v.binding.matches(vm)) {
                self.cached_views.remove(index).map_err(|_| EIO)?
            } else {
                let binding = uat.bind(vm)?;
                let mut views = KVec::new();
                for i in 0..storage::COUNT {
                    if matches!(storage::allocation_for(self.profile,i).map_err(|_|EINVAL)?.mapping,Mapping::Client(_)) {
                        views.push((i,self.objects[i].map_gpu_view(vm)?),GFP_KERNEL)?;
                    }
                }
                let command = self.objects[storage::COMMAND].map_gpu_view(vm)?;
                CachedViews {binding, command, views}
            };
            // Complete allocations before moving the active views. Eviction
            // drops only an idle cached view; no arena storage is duplicated.
            let old_views = KVec::with_capacity(next.views.len(), GFP_KERNEL)?;
            self.cached_views.reserve(1, GFP_KERNEL)?;
            if self.cached_views.len() == 3 {
                drop(self.cached_views.remove(0).map_err(|_| EIO)?);
            }
            let mut old = CachedViews {
                binding: core::mem::replace(&mut self._binding, next.binding),
                command: core::mem::replace(&mut self._client_command, next.command),
                views: old_views,
            };
            for (i, view) in next.views {
                old.views.push((i, self.objects[i].swap_gpu_view(view)?), GFP_KERNEL)?;
            }
            self.cached_views.push(old, GFP_KERNEL)?;
        }
        self.checkpoint=(self.sequence,self.head,self.batch_count);
        self.batch_count=0;
        if let Err(error)=self.append(control) {
            self.abort_batch()?;
            return Err(error);
        }
        Ok(())
    }
    /// Append only while the previous batch is fully retired and this one
    /// remains unpublished. Firmware serializes each command, including its
    /// original completion flush, in this one compute queue.
    pub(crate) fn append(&mut self,control:crate::m3::agx_compute::Control)->Result {
        if self.batch_count>=storage::SLOTS {return Err(E2BIG);}
        self.slot=self.batch_count;
        for i in storage::RESET {
            let offset=storage::slot_offset_for(self.profile,i,self.slot).map_err(|_|EINVAL)?;
            let size=storage::allocation_for(self.profile,i).map_err(|_|EINVAL)?.size;
            self.objects[i].fill_range(offset,size,if i==PREEMPTION {0xcc} else {0})?;
        }
        self.objects[NOTIFIER].u32(state::NOTIFIER_CONTEXT,self._binding.slot())?;
        self.sequence=self.sequence.checked_add(1).ok_or(EOVERFLOW)?;
        Self::encode_command(&mut self.objects,&mut self.command_bytes,self._binding.slot(),control,self.sequence,self.slot,self.registers,self.profile,self._binding.generation())?;
        let stamp=self.stamp();
        Self::encode_sequence(&mut self.objects,&mut self.sequence_bytes,self.stats,self._binding.slot(),stamp,self.slot,self.profile,self.sequence - 1)?;
        self.write_counters()?;
        let address=self.objects[storage::COMMAND].va()+storage::slot_offset_for(self.profile,storage::COMMAND,self.slot).map_err(|_|EINVAL)? as u64;
        self.objects[storage::RING].u64(usize::from(self.head)*8,address)?;
        self.head=(self.head+1)%queue::QUALIFIED_CAPACITY as u16;
        self.batch_count+=1;
        self.objects[storage::QUEUE_STATE].u32(queue::WRITE,u32::from(self.head))?;
        agx_memory::publish();crate::mem::sync();crate::mem::tlbi_all();crate::mem::sync();
        Ok(())
    }
    /// Roll back producer state before any SubmitQueue publication. Old ring
    /// entries beyond the restored cursor are unreachable and get overwritten.
    pub(crate) fn abort_batch(&mut self)->Result {
        (self.sequence,self.head,self.batch_count)=self.checkpoint;
        self.slot=self.batch_count.saturating_sub(1);
        self.write_counters()?;
        self.objects[storage::QUEUE_STATE].u32(queue::WRITE,u32::from(self.head))?;
        agx_memory::publish();Ok(())
    }
    fn write_counters(&mut self) -> Result {
        for index in [storage::EVENT, storage::COUNTER] {
            if self.profile == super::m3_profile::Profile::Extended {
                self.objects[index].u64(0, self.sequence)?;
            } else { self.objects[index].u32(0, self.sequence as u32)?; }
        }
        Ok(())
    }

    fn offset(&self,owner:usize)->Result<usize> {
        storage::slot_offset_for(self.profile,owner,self.slot).map_err(|_|EINVAL)
    }
    /// Same cache-generation contract as M4: StartCompute and the persistent
    /// notifier must use the same tag before the firmware flushes user stamps.
    /// RTKit 2419: StartCompute.counter1 +0x34; JobMeta.flush_stamps +0x804.
    pub(crate) fn prepare_completion(&mut self,coalesce:bool)->Result {
        if coalesce && self.profile == super::m3_profile::Profile::Extended { return Err(EINVAL); }
        let sequence=self.offset(storage::MICROSEQUENCE)?;
        let command=self.offset(storage::COMMAND)?;
        self.objects[storage::MICROSEQUENCE].u32(sequence+microsequence::GENERATION,0)?;
        self.objects[storage::COMMAND].u32(command+if self.profile == super::m3_profile::Profile::Extended { 0x81c } else { command::FLUSH_STAMPS },u32::from(!coalesce))?;
        agx_memory::publish();Ok(())
    }
    pub(crate) fn set_user_timestamps(&mut self,addresses:[u64;2])->Result {
        let offset=self.offset(storage::COMMAND)?+if self.profile == super::m3_profile::Profile::Extended {
            0x844
        } else { command::USER_TIMESTAMPS };
        self.objects[storage::COMMAND].u64(offset,addresses[0])?;
        self.objects[storage::COMMAND].u64(offset+8,addresses[1])?;
        let pointer=if addresses==[0,0] {0} else {self.objects[storage::COMMAND].va()+offset as u64};
        let sequence=self.offset(storage::MICROSEQUENCE)?;
        for start in if self.profile == super::m3_profile::Profile::Extended { [0x1bc, 0x20c] }
            else { [microsequence::TIMESTAMP_START,microsequence::TIMESTAMP_END] } {
            self.objects[storage::MICROSEQUENCE].u64(sequence+start+microsequence::USER_TIMESTAMP_POINTER,pointer)?;
        }
        agx_memory::publish();Ok(())
    }
    pub(crate) fn set_attachments(&mut self,attachments:&crate::m3::agx_attachments::Attachments)->Result {
        let sequence=self.offset(storage::MICROSEQUENCE)?;
        self.objects[storage::MICROSEQUENCE].write(sequence+if self.profile == super::m3_profile::Profile::Extended {
            microsequence::Sequence::EXTENDED_ATTACHMENTS
        } else { microsequence::ATTACHMENTS },&attachments.0)?;
        self.objects[storage::MICROSEQUENCE].write(sequence+if self.profile == super::m3_profile::Profile::Extended {
            microsequence::Sequence::EXTENDED_HAS_ATTACHMENTS
        } else { microsequence::HAS_ATTACHMENTS },&[u8::from(attachments.count()!=0)])?;
        agx_memory::publish();Ok(())
    }
    fn stamp(&self) -> u32 { timeline::stamp(0xc1000000,self.sequence) }
    pub(crate) fn completion_dependency(&self)->Result<crate::m3::m3_sync_layout::CompletionDependency> {
        Ok(crate::m3::m3_sync_layout::CompletionDependency {
            stamp:queue::FirmwareVa::new(self.objects[storage::FW_STAMP].va()).map_err(|_|EINVAL)?,
            value:self.stamp(),event:2,
        })
    }
    pub(crate) fn reserve_barriers(&mut self,dev:&driver::AsahiDrmDevice,uat:&mmu::Uat)->Result {
        if self.barriers.is_none() {
            self.barriers=Some(Buffer::at(dev,uat.kernel_vm(),None,None,storage::SLOTS*64,false)?);
        }
        Ok(())
    }
    /// Insert a dependency before the last appended, still unpublished CDM.
    /// Its command ordinal and timestamp slot are unchanged; only the ring
    /// gains an entry. The whole batch must retire before this storage is reused.
    pub(crate) fn insert_dependency(&mut self,dependency:crate::m3::m3_sync_layout::CompletionDependency)->Result {
        if self.batch_count==0 || self.slot>=storage::SLOTS || dependency.event!=1 {
            return Err(EINVAL);
        }
        let value=self.stamp();
        let barriers=self.barriers.as_mut().ok_or(EINVAL)?;
        let offset=self.slot*64;
        let mut bytes=[0;crate::m3::m3_sync_layout::BARRIER_SIZE];
        crate::m3::m3_sync_layout::Barrier {
            stamp:dependency.stamp,wait_value:dependency.value,event:dependency.event,
            self_value:value,uuid:0xc10000,dependency:crate::m3::m3_sync_layout::Dependency::PreviousPass,
        }.encode(&mut bytes).map_err(|_|EINVAL)?;
        barriers.write(offset,&bytes)?;
        let address=barriers.va()+offset as u64;
        let (slots,next)=queue::insert_before_last(self.head,queue::QUALIFIED_CAPACITY).map_err(|_|EINVAL)?;
        let command=self.objects[storage::COMMAND].va()+self.offset(storage::COMMAND)? as u64;
        self.objects[storage::RING].u64(usize::from(slots[0])*8,address)?;
        self.objects[storage::RING].u64(usize::from(slots[1])*8,command)?;
        self.head=next;
        self.objects[storage::QUEUE_STATE].u32(queue::WRITE,u32::from(self.head))?;
        agx_memory::publish();crate::mem::sync();Ok(())
    }
    pub(crate) fn gpu_ns(&mut self)->Result<u64> {
        let mut total: u64 = 0;
        for slot in 0..self.batch_count {
            total = total.checked_add(self.slot_gpu_ns(slot)?).ok_or(EOVERFLOW)?;
        }
        Ok(total)
    }
    pub(crate) fn slot_gpu_ns(&mut self,slot:usize)->Result<u64> {
        if slot>=self.batch_count {return Err(EINVAL);}
        let start=self.objects[TIMESTAMPS[0]].read_u64(storage::slot_offset_for(self.profile,TIMESTAMPS[0],slot).map_err(|_|EINVAL)?)?;
        let end=self.objects[TIMESTAMPS[1]].read_u64(storage::slot_offset_for(self.profile,TIMESTAMPS[1],slot).map_err(|_|EINVAL)?)?;
        if start == 0 || end < start { return Ok(0); }
        self.profile.gpu_duration_ns(end - start).ok_or(EOVERFLOW)
    }
    /// A batch span includes gaps between its compute commands, unlike gpu_ns.
    /// Read only after stamps, queues, events and idle state prove retirement.
    pub(crate) fn batch_gpu_span(&mut self)->Result<[u64;2]> {
        if self.batch_count == 0 {return Err(EINVAL);}
        Ok([self.objects[TIMESTAMPS[0]].read_u64(storage::slot_offset_for(self.profile,TIMESTAMPS[0],0).map_err(|_|EINVAL)?)?,
            self.objects[TIMESTAMPS[1]].read_u64(storage::slot_offset_for(self.profile,TIMESTAMPS[1],self.batch_count-1).map_err(|_|EINVAL)?)?])
    }
    pub(crate) fn head(&self) -> u16 { self.head }
    pub(crate) fn ordinal(&self)->u64 {self.sequence}
    pub(crate) fn first(&self) -> bool {
        self.profile == super::m3_profile::Profile::Legacy && self.sequence == self.batch_count as u64
    }
    pub(crate) fn progress(&mut self, dev: &driver::AsahiDrmDevice) -> Result {
        // Report real completions by elapsed time as well as sequence count;
        // batches need not land on a multiple of 128.
        if !crate::debug::debug_enabled(crate::debug::DebugFlags::SubmitTiming) {
            return Ok(());
        }
        let now = <kernel::time::Monotonic as kernel::time::ClockSource>::ktime_get();
        if self.last_progress_ns != 0 && now - self.last_progress_ns < 1_000_000_000
            && self.sequence != 1 && self.sequence % 128 != 0
            && !crate::debug::debug_enabled(crate::debug::DebugFlags::SubmitTiming) {
            return Ok(());
        }
        self.last_progress_ns = now;
        dev_info!(dev.as_ref(), "M3_COMPUTE_RETIRED sequence={} context={} stamp={:#x}/{:#x} queue={}/{} head={} idle=1 cache_flush=1\n",
            self.sequence, self._binding.slot(), self.objects[storage::STAMP].read_u32(0)?,
            self.objects[storage::FW_STAMP].read_u32(0)?, self.objects[storage::QUEUE_STATE].read_u32(queue::DONE)?,
            self.objects[storage::QUEUE_STATE].read_u32(queue::READ)?, self.head);
        Ok(())
    }

    pub(crate) fn queue(&self)->u64 {self.objects[storage::QUEUE].va()}
    pub(crate) fn complete(&mut self)->Result<bool> {
        let stamped = self.objects[storage::STAMP].read_u32(0)? == self.stamp()
            && self.objects[storage::FW_STAMP].read_u32(0)? == self.stamp()
            && self.objects[storage::QUEUE_STATE].read_u32(queue::DONE)? == u32::from(self.head)
            && self.objects[storage::QUEUE_STATE].read_u32(queue::READ)? == u32::from(self.head);
        if !stamped { return Ok(false); }
        if self.profile == super::m3_profile::Profile::Extended {
            return Ok(self.objects[storage::POOL].read_u64(0x54)? == self.sequence);
        }
        Ok(true)
    }
    pub(crate) fn log(&mut self,dev:&driver::AsahiDrmDevice)->Result {
        for off in (0..512).step_by(128) {
            let mut raw=[0u8;128]; self.objects[storage::COMMAND].read(off,&mut raw)?;
            dev_info!(dev.as_ref(),"M3 post-command {:#x} {:02x?}\n",off,raw);
        }
        let stamp=self.objects[storage::STAMP].read_u32(0)?;
        let fw=self.objects[storage::FW_STAMP].read_u32(0)?;
        let done=self.objects[storage::QUEUE_STATE].read_u32(queue::DONE)?;
        let read=self.objects[storage::QUEUE_STATE].read_u32(queue::READ)?;
        dev_info!(dev.as_ref(),"M3 compute: stamps={:#x}/{:#x} done={} read={}\n",stamp,fw,done,read);
        let mut record=[0u8;0x84];self.objects[storage::POOL].read(0,&mut record)?;
        dev_info!(dev.as_ref(),"M3 compute pool={:02x?}\n",record);
        let mut roots=[(0,0);64];self._binding.vm().context_roots(&mut roots)?;
        dev_info!(dev.as_ref(),"M3 context {} roots={:x?}, bootstrap={:x?}\n",self._binding.slot(),roots[self._binding.slot() as usize],roots[0]);
        Ok(())
    }
}
