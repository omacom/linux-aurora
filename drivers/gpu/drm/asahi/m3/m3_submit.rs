// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Bounded M3 compute and graphics submissions through the common DRM scheduler/syncobjs.
use kernel::{c_str,dma_fence::*,drm::sched,prelude::*,sync::{Arc,Mutex},new_mutex,xarray};
use crate::{file,m3::agx_uapi,mmu,queue,m3::m3_drm::Shared};
use core::sync::atomic::{AtomicBool, Ordering};
#[derive(Default)]
pub(crate) struct Completion;
#[vtable]
impl FenceOps for Completion {
    fn get_driver_name<'a>(self:&'a FenceObject<Self>)->&'a CStr {c_str!("asahi")}
    fn get_timeline_name<'a>(self:&'a FenceObject<Self>)->&'a CStr {c_str!("m3-native")}
}
pub(crate) struct Destination { mapping: Arc<mmu::KernelMapping>, offset: usize }
impl Destination {
    fn resolve(objects: Pin<&xarray::XArray<KBox<file::Object>>>,
        timestamp: agx_uapi::UapiTimestamp) -> Result<Option<Self>> {
        if timestamp.handle == 0 { return Ok(None); }
        let guard = objects.lock();
        let object = guard.get(timestamp.handle.try_into()?).ok_or(ENOENT)?;
        match &*object {
            file::Object::TimestampBuffer(buffer) => {
                let mapping = buffer.firmware_mapping()?;
                let offset = timestamp.offset as usize;
                if offset & 7 != 0 || offset.checked_add(8).ok_or(EOVERFLOW)? > mapping.size() {
                    return Err(EINVAL);
                }
                Ok(Some(Self { mapping, offset }))
            }
        }
    }
    fn firmware_address(&self) -> u64 { self.mapping.iova() + self.offset as u64 }
}

#[derive(Copy,Clone)]
pub(crate) enum Command {
    Compute(crate::m3::agx_compute::Control),
    Render{command:agx_uapi::UapiRenderCommand,usc:u64},
}
pub(crate) struct Packet {
    pub(crate) vm:mmu::Vm,
    pub(crate) commands:KVec<Command>,
    timestamps:KVec<[Option<Destination>;4]>,
    pub(crate) attachments:KVec<crate::m3::agx_attachments::Attachments>,
    pub(crate) wide_visibility:KVec<bool>,
    completion:UserFence<Completion>,
    finish_claimed: AtomicBool,
    /// Set before a batch with any of this packet's commands can change firmware-visible state,
    /// and never cleared.
    published: AtomicBool,
    /// Queue urgency (3 = UAPI realtime .. 0 = UAPI low): more urgent packets are published first.
    pub(crate) urgency:u8,
    /// Who submitted the packet, for the failure log only.
    pub(crate) owner:Owner,
    vm_job:Pin<KBox<Mutex<Option<mmu::VmJobGuard>>>>,
}
/// The process and thread that submitted a packet, recorded in the submit ioctl. Only logged:
/// a renamed task leaves a stale name, which is harmless.
#[derive(Copy,Clone)]
pub(crate) struct Owner {pub(crate) pid:i32,process:[u8;16],thread:[u8;16]}
impl Owner {
    fn current()->Self {
        let task=kernel::current!();
        let leader=task.group_leader();
        Self {pid:leader.pid(),process:crate::m3::m3_client::task_comm(leader),
            thread:crate::m3::m3_client::task_comm(task)}
    }
    fn name(comm:&[u8;16])->&str {
        let len=comm.iter().position(|&b|b==0).unwrap_or(comm.len());
        core::str::from_utf8(&comm[..len]).unwrap_or("?")
    }
    pub(crate) fn process(&self)->&str {Self::name(&self.process)}
    pub(crate) fn thread(&self)->&str {Self::name(&self.thread)}
}
impl Packet {
    pub(crate) fn profile_id(&self)->usize {self as *const Self as usize}
    pub(crate) fn timestamp_addresses(&self,index:usize)->[u64;2] {
        core::array::from_fn(|i|self.timestamps[index][i].as_ref().map_or(0,Destination::firmware_address))
    }
    pub(crate) fn render_timestamp_addresses(&self,index:usize)->[[u64;2];2] {
        core::array::from_fn(|stage| core::array::from_fn(|i|
            self.timestamps[index][stage*2+i].as_ref().map_or(0,Destination::firmware_address)))
    }
    /// Called only after all published commands retired or before any publication.
    pub(crate) fn release_guard(&self) {drop(self.vm_job.lock().take());}
    /// Called before the first firmware-visible change of a batch that includes this packet.
    pub(crate) fn mark_published(&self) {self.published.store(true,Ordering::Relaxed);}
    /// Whether the firmware may have seen any of this packet's commands.
    pub(crate) fn may_be_published(&self)->bool {self.published.load(Ordering::Relaxed)}
    pub(crate) fn retired(&self) {self.release_guard();self.finish(Ok(()));}
    pub(crate) fn finish(&self,result:Result) {
        // Retirement, cancellation and a queued scheduler timeout can race.
        // Claim error/signal ownership once; this flag is NOT proof that the
        // fence is signaled or that hardware DMA has stopped.
        if self.finish_claimed.swap(true, Ordering::Relaxed) { return; }
        if let Err(e)=result {
            // A guard still held here belongs to work the firmware may still run: block
            // mapping changes before the error fence is visible (as G15 does), so userspace
            // can't unmap memory under it. It stays with the packet until the runtime drops
            // the packet, after the firmware has provably stopped.
            if let Some(guard)=self.vm_job.lock().as_ref() {guard.quarantine();}
            if let Some(status) = self.vm.status() { status.record(e); }
            self.completion.set_error(e);
        }
        // A scheduler job/fence and the runtime's last packet can outlive
        // hardware retirement. They must not keep the VM's active-job count
        // nonzero: GEM closes otherwise accumulate across completed frames.
        // Signalling an error is not retirement.
        crate::m3::agx_memory::publish();
        let profile=crate::debug::debug_enabled(crate::debug::DebugFlags::M3PassTiming);
        let before=if profile {crate::m3::m3_runtime::physical_counter()} else {0};
        self.completion.signal();
        if profile {
            let after=crate::m3::m3_runtime::physical_counter();
            let status=match result {Ok(())=>0,Err(e)=>e.to_errno()};
            pr_info!("M3_PACKET_FENCE vm={} packet={:#x} commands={} signal_before_tick={} signal_after_tick={} status={} counter_hz=24000000\n",
                self.vm.id(),self.profile_id(),self.commands.len(),before,after,status);
        }
    }
}
pub(crate) struct Job {shared:Shared,packet:Arc<Packet>}
/// Timeout state of a [`Job`], owned by the scheduler's timeout worker.
pub(crate) struct JobTimeout {shared:Shared,packet:Arc<Packet>}
impl sched::JobImpl for Job {
    type TimeoutData = JobTimeout;
    fn timeout_data(&self)->JobTimeout {
        JobTimeout {shared:self.shared.clone(),packet:self.packet.clone()}
    }
    fn run(job:&mut Self)->Result<Option<Fence>> {
        // Queue the packet and return its fence; the completion worker publishes and retires
        // it, so the scheduler can hand over further ready jobs meanwhile.
        if let Err(e)=crate::m3::m3_runtime::Runtime::submit(&job.shared,job.packet.clone()) {
            pr_err!("M3 scheduler: submission failed {:?}\n",e);
            // submit() failed before this packet could be published.
            job.packet.release_guard();job.packet.finish(Err(e));
        }
        Ok(Some(Fence::from_fence(&job.packet.completion)))
    }
    fn timed_out(job:&mut JobTimeout,_finished:bool)->sched::Status {
        // Retain the publishing branch's explicit diagnostic override.
        if !crate::m3::m3_params::timeout_nohang() {
            if let Some(r)=Option::as_mut(&mut *job.shared.lock()) {r.health().mark_failed();}
            crate::m3::m3_runtime::Runtime::withdraw(&job.shared,&job.packet);
            job.packet.finish(Err(ETIMEDOUT));return sched::Status::NoDevice;
        }
        let guard=job.shared.lock();
        let Some(runtime)=guard.as_ref() else {
            job.packet.finish(Err(ENODEV));
            return sched::Status::NoDevice;
        };
        // The scheduler's deadline covers the whole packet, including time queued behind
        // other batches. The completion worker owns each batch's bounded hardware-completion
        // checks. Reinsert a healthy packet instead of turning a late timeout into a global
        // device loss or corrupting an already signaled fence.
        if runtime.health().healthy() { return sched::Status::NoHang; }
        drop(guard);
        crate::m3::m3_runtime::Runtime::withdraw(&job.shared,&job.packet);
        job.packet.finish(Err(EIO));
        sched::Status::NoDevice
    }
    fn cancel(job:&mut Self) {
        crate::m3::m3_runtime::Runtime::withdraw(&job.shared,&job.packet);
        job.packet.finish(Err(ECANCELED));
    }
}
struct AddressSpace<'a>(&'a mmu::Vm);
impl agx_uapi::GpuAddressSpace for AddressSpace<'_> {
    fn covers(&self,address:u64,size:u64,access:agx_uapi::GpuAccess)->bool {
        let Some(end)=address.checked_add(size) else {return false;};
        let (r,w)=match access {agx_uapi::GpuAccess::Read=>(true,false),
            agx_uapi::GpuAccess::Write=>(false,true),agx_uapi::GpuAccess::ReadWrite=>(true,true)};
        address>=0x4000 && end<=(1u64<<42)-0x8000
            && !self.0.driver_range_overlaps(address..end) && self.0.covers_range(address,size,r,w)
    }
}
pub(crate) struct Queue {
    entity:sched::Entity<Job>,_scheduler:Arc<sched::Scheduler<Job>>,
    shared:Shared,vm:mmu::Vm,usc:u64,fences:FenceContexts,
    /// Whether the SoC advertises early tiling (`m3_soc::Features::fragment_dependency`).
    early_tiling:bool,
    urgency:u8,
}
impl Queue {
    pub(crate) fn new(shared:Shared,scheduler:Arc<sched::Scheduler<Job>>,vm:mmu::Vm,priority:u32,usc:u64,
        early_tiling:bool)->Result<Self> {
        agx_uapi::QueueUscWindow{base:usc,user_start:0x4000,user_end:(1u64<<42)-0x8000}.validate().map_err(|_|EINVAL)?;
        // file.rs passes REALTIME - UAPI priority: 3 is UAPI LOW, 0 is UAPI REALTIME.
        let urgency=3u8.saturating_sub(priority.min(3) as u8);
        let priority=match priority {3=>sched::Priority::Low,2=>sched::Priority::Normal,
            1=>sched::Priority::High,0=>sched::Priority::Kernel,_=>return Err(EINVAL)};
        Ok(Self {entity:sched::Entity::new(&scheduler,priority)?,_scheduler:scheduler,
            shared,vm,usc,fences:FenceContexts::new(1,c_str!("asahi_m3_queue"),kernel::static_lock_class!())?,
            early_tiling,urgency})
    }
}
impl queue::Queue for Queue {
    fn submit(&mut self,_reg_data:&crate::driver::AsahiRegData<'_>,_id:u64,mut syncs:KVec<file::SyncItem>,in_sync_count:usize,
        raw:&[u8],objects:Pin<&xarray::XArray<KBox<file::Object>>>)->Result {
        let vm_job=self.vm.retain_job()?;
        let mut parser=agx_uapi::UapiCommandParser::new(raw);let mut commands=KVec::new();let mut timestamps=KVec::new();let mut flushes=KVec::new();let mut visibility=KVec::new();
        while let Some(command)=parser.next_hardware().map_err(|_|EINVAL)? {
            if let agx_uapi::ParsedHardwareCommand::Render{payload,vertex_attachments,fragment_attachments,..}=command {
                if let Err(e)=validate_render(payload,self.usc,&AddressSpace(&self.vm),self.early_tiling) {
                    pr_err!("M3 render rejected {:?}: flags={:#x} size={}x{} layers={} samples={} utile={}x{} sample_size={} depth={:#x} stencil={:#x} zls={:#x} samplers={} heap={:#x}\n",e,payload.flags,payload.width,payload.height,payload.layers,payload.samples,payload.utile_width,payload.utile_height,payload.sample_size,payload.depth.base,payload.stencil.base,payload.zls_control,payload.sampler_count,payload.sampler_heap);
                    return Err(e);
                }
                agx_uapi::validate_attachments(&AddressSpace(&self.vm),&vertex_attachments).map_err(|_|EINVAL)?;
                agx_uapi::validate_attachments(&AddressSpace(&self.vm),&fragment_attachments).map_err(|_|EINVAL)?;
                commands.push(Command::Render{command:payload,usc:self.usc},GFP_KERNEL)?;
                timestamps.push([Destination::resolve(objects,payload.vertex_timestamps.start)?,
                    Destination::resolve(objects,payload.vertex_timestamps.end)?,
                    Destination::resolve(objects,payload.fragment_timestamps.start)?,
                    Destination::resolve(objects,payload.fragment_timestamps.end)?],GFP_KERNEL)?;
                flushes.push(crate::m3::agx_attachments::Attachments::EMPTY,GFP_KERNEL)?;
                visibility.push(false,GFP_KERNEL)?;
                continue;
            }
            let agx_uapi::ParsedHardwareCommand::Compute{mut payload,attachments,..}=command else {return Err(ENOTSUPP);};
            // Other engines and helpers need their own M3 firmware translation.
            let wide=kernel::uapi::drm_asahi_compute_flags_DRM_ASAHI_COMPUTE_WIDE_VISIBILITY as u32;
            if payload.flags & !wide !=0 || payload.sampler_count!=0 || payload.sampler_heap!=0 {
                pr_err!("M3 compute rejected: flags={:#x} samplers={} heap={:#x} attachments={} timestamps={}/{}\n",payload.flags,payload.sampler_count,payload.sampler_heap,attachments.as_slice().len(),payload.timestamps.start.handle,payload.timestamps.end.handle);
                return Err(ENOTSUPP);
            }
            visibility.push(payload.flags & wide !=0,GFP_KERNEL)?;
            // The common translator validates the original command/attachment
            // addresses; this M3 scheduling hint changes none of them.
            payload.flags &= !wide;
            flushes.push(encode_attachments(&self.vm,&attachments)?,GFP_KERNEL)?;
            let c=agx_uapi::translate_compute_command(payload,attachments,
                agx_uapi::QueueUscWindow{base:self.usc,user_start:0x4000,user_end:(1u64<<42)-0x8000},
                &AddressSpace(&self.vm)).map_err(|e| {
                    pr_err!("M3 compute translation rejected {:?}: stream={:#x}..{:#x} helper={:?}\n",e,payload.control_stream_base,payload.control_stream_end,payload.helper);
                    EINVAL
                })?;
            crate::m3::m3_client::check_compute_launches(&self.vm,&AddressSpace(&self.vm),c.control_stream_base,c.control_stream_end)?;
            timestamps.push([Destination::resolve(objects,payload.timestamps.start)?,
                Destination::resolve(objects,payload.timestamps.end)?,None,None],GFP_KERNEL)?;
            commands.push(Command::Compute(crate::m3::agx_compute::Control{base:c.control_stream_base,end:c.control_stream_end,usc_base:self.usc}),GFP_KERNEL)?;
        }
        parser.finish().map_err(|_|EINVAL)?;
        let packet=Arc::new(Packet{vm:self.vm.clone(),commands,timestamps,attachments:flushes,wide_visibility:visibility,
            completion:self.fences.new_fence(0,Completion)?.into(),
            finish_claimed:AtomicBool::new(false),published:AtomicBool::new(false),urgency:self.urgency,owner:Owner::current(),
            vm_job:KBox::pin_init(new_mutex!(Some(vm_job)),GFP_KERNEL)?},GFP_KERNEL)?;
        let mut job=self.entity.new_job(1,Job{shared:self.shared.clone(),packet})?;
        for sync in syncs.drain(0..in_sync_count) {if let Some(f)=sync.fence {job.add_dependency(f)?;}}
        let mut job=job.arm();let finished=job.fences().finished();job.push();
        for mut sync in syncs {if let Some(chain)=sync.chain_fence.take() {
            sync.syncobj.add_point(chain,&finished,sync.timeline_value);
        } else {sync.syncobj.replace_fence(Some(&finished));}}
        Ok(())
    }
}

fn encode_attachments(vm: &mmu::Vm, list: &agx_uapi::UapiAttachmentList)
    -> Result<crate::m3::agx_attachments::Attachments> {
    use crate::m3::agx_attachments::{Attachments, CAPACITY};
    use agx_uapi::{GpuAddressSpace, GpuAccess};
    if list.as_slice().len()>CAPACITY {return Err(E2BIG);}
    let mut ranges = [(0, 0); CAPACITY];
    for (range, entry) in ranges.iter_mut().zip(list.as_slice()) {
        *range = Attachments::range(entry.address, entry.size).map_err(|_| EINVAL)?;
        if !AddressSpace(vm).covers(range.0, range.1, GpuAccess::Write) {
            return Err(EINVAL);
        }
    }
    let mut out=Attachments::new(&ranges[..list.as_slice().len()]).map_err(|_| EINVAL)?;
    // G15S RTKit-2419 uses the legacy 0x17 cache attribute; retain the
    // M4 range normalization and ownership checks, not its 0x18 attribute.
    for i in 0..list.as_slice().len() {out.0[i*16+12..i*16+14].copy_from_slice(&0x17u16.to_le_bytes());}
    Ok(out)
}

fn validate_render(r: agx_uapi::UapiRenderCommand, usc: u64, space: &AddressSpace<'_>,
    early_tiling: bool) -> Result {
    use agx_uapi::{GpuAddressSpace, GpuAccess};
    if r.sample_size % 8 != 0 {return Err(ENOTSUPP);}
    let empty = agx_uapi::UapiHelperProgram { binary:0, config:0, data:0 };
    // Early tiling (render flag bit 5) only where it is advertised.
    let stage_scope=if early_tiling && crate::m3::m3_params::early_tiling() {1<<5} else {0};
    if r.flags & !(2|16|(1<<18)|stage_scope) != 0 || !matches!(r.samples, 1 | 2 | 4)
        || r.sampler_count != 0 || r.sampler_heap != 0
        || r.vertex_helper != empty || r.fragment_helper != empty
        || r.stencil.compression_base != 0 || r.stencil.compression_stride != 0
        || r.ppp_control & !0x203 != 0 { return Err(ENOTSUPP); }
    // Native 26A428 descriptors witness the ZLS metadata registers and mirrors
    // for single-sample, single-layer depth. Keep other layouts unqualified.
    if (r.depth.compression_base != 0 || r.stencil.compression_base != 0)
        && (r.samples != 1 || r.layers != 1) {return Err(ENOTSUPP);}
    // sample_size describes color tilebuffer storage. Depth/stencil-only
    // passes legitimately require zero bytes of color storage.
    if r.layers == 0 || r.layers > 2048 || r.width == 0 || r.height == 0 || r.width > 16384 || r.height > 16384
        || !matches!(r.utile_width,16|32) || !matches!(r.utile_height,16|32)
        || u32::from(r.sample_size)*u32::from(r.utile_width)*u32::from(r.utile_height)*u32::from(r.samples)>32768 {
        return Err(EINVAL);
    }
    crate::m3::agx_render_state::compact(r.vdm_base).map_err(|_| EINVAL)?;
    if r.vdm_base & 3 != 0 || !space.covers(r.vdm_base,4,GpuAccess::Read)
        || r.scissor_base == 0 || r.scissor_base & 7 != 0 || !space.covers(r.scissor_base,8,GpuAccess::Read) {
        return Err(EINVAL);
    }
    for (p,access) in [(r.depth_bias_base,GpuAccess::Read),(r.occlusion_query_base,GpuAccess::Write)] {
        if p != 0 && (p & 7 != 0 || !space.covers(p,8,access)) { return Err(EINVAL); }
    }
    for zls in [r.depth,r.stencil] { agx_uapi::validate_zls(space,zls,r.layers).map_err(|_| EINVAL)?; }
    let window = agx_uapi::QueueUscWindow { base:usc,user_start:0x4000,user_end:((1u64 << 42) - 2 * mmu::UAT_PGSZ as u64) };
    for p in [r.background,r.end_of_tile,r.partial_background,r.partial_end_of_tile] {
        window.full_program(space,p.usc).map_err(|_| EINVAL)?;
        if !space.covers(usc+u64::from(p.usc & !63),64,GpuAccess::Read) { return Err(EINVAL); }
    }
    Ok(())
}
