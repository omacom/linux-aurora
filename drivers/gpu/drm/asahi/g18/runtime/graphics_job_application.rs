// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Application lifetime around the H1 allocation graph. The fixture VM stays
//! unpublished storage; commands are rebuilt entirely from Mesa parameters.
use super::*;
use crate::{g18::uapi::application::G18ApplicationRender,g18::uapi::application_runtime::{self as g18_application_runtime,Frame}};

/// The VMs the audit's page walk read, as they were: the client VM and the
/// mapping epochs of the storage, client, kernel and kernel-lower VMs. A
/// client VM's id() comes from the device's 64-bit VM id counter and is never
/// given to another VM, so a VM created after one was freed never matches.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) struct WalkKey {
    client: u64,
    epochs: [u64; 4],
}

pub(crate) struct ApplicationJob {
    pub(crate) job: KBox<G18GraphicsJob>,
    storage: mmu::Vm,
    user_aliases: KVec<mmu::KernelMapping>,
    pub(crate) pool_aliases: KVec<mmu::KernelMapping>,
    guard: Option<mmu::T8140VmJobGuard>,
    scene: [u8;0x80],
    generation: u32,
    retired: bool,
    /// Frames of the current client VM, counting this one.
    client_frames: u32,
    /// The current render's adapter-derived private-memory selection.
    scratch: Option<crate::g18::uapi::graphics_scratch::G18RenderScratchSelection>,
    /// What the last page walk proved; None before the first.
    pub(crate) walked: Option<WalkKey>,
}

impl ApplicationJob {
    pub(crate) fn new(mut job: KBox<G18GraphicsJob>) -> Result<KBox<Self>> {
        let storage=job.vm().clone();
        let scene=job.parameter_buffer.application_scene(None)?;
        // Remove the fixture-only aliases before target growth. The original
        // payload and PB backing stay retained but no capture bytes execute.
        job.aliases.clear();
        Ok(KBox::new(Self {job,storage,user_aliases:KVec::new(),pool_aliases:KVec::new(),
            guard:None,scene,generation:0,retired:true,client_frames:0,scratch:None,walked:None},GFP_KERNEL)?)
    }
    pub(crate) fn frame(&self) -> Frame {Frame::new(self.generation).expect("admitted frame")}
    /// Grow the PB of the frame in flight for a firmware TVB grow request
    /// naming its slot and context. The client aliases join the client's
    /// other PB aliases; a later client maps every block afresh.
    pub(crate) fn grow_parameter_buffer(&mut self, dev: &AsahiDrmDevice, buffer_slot: u32,
        vm_slot: u32) -> Result<u32> {
        use crate::g18::uapi::render_policy::{PB_GROW_BLOCKS,PB_MAX_BLOCKS};
        if self.retired || buffer_slot!=self.job.parameter_buffer.command_fields().pb_slot
            || vm_slot!=u32::from(self.job.context_id()) {return Err(ENOENT);}
        self.user_aliases.reserve(2,GFP_KERNEL)?;
        let vm=self.job.vm().clone();
        let client=vm.memory_client().cloned();
        let (added,aliases)=self.job.parameter_buffer.grow(dev,&self.storage,&vm,client.as_ref(),
            PB_GROW_BLOCKS,PB_MAX_BLOCKS)?;
        for alias in aliases {let _=self.user_aliases.push_within_capacity(alias);}
        Ok(added)
    }
    /// The last frame kicked to both graphics queues (0 before the first).
    pub(crate) fn submitted_generation(&self) -> u32 {self.generation}
    pub(crate) fn quarantine(&mut self) {
        if let Some(guard)=&self.guard {guard.quarantine();}
        if let Some(status)=self.job.vm().status(){status.record(EIO);}
    }
    pub(crate) fn retired(&mut self) {self.retired=true;self.guard=None;}
    /// One of the first frames of its client: logged even when job logging
    /// is sparse, so every isolated client leaves a kick/retirement pair.
    pub(crate) fn traced(&self) -> bool {self.client_frames<=3}

    #[inline(never)]
    pub(crate) fn prepare(&mut self, dev: &AsahiDrmDevice, uat: &mmu::Uat,
        vm: &mmu::Vm, render: &G18ApplicationRender) -> Result {
        if !self.retired {return Err(EBUSY);}
        let frame=Frame::new(self.generation.checked_add(1).ok_or(EOVERFLOW)?).ok_or(EOVERFLOW)?;
        let plan=assembly::target_plan(&render.parameters).ok_or(EINVAL)?;
        // Growth retains each object's original IOVA. Reject a changed target
        // policy before touching mappings rather than encode unowned addresses.
        if self.job.targets.len()!=plan.offsets.len()
            || self.job.targets.iter().zip(plan.offsets)
                .any(|(target,offset)|target.iova()!=va::GRAPHICS_TARGET_OFFSET+offset) {
            return Err(EINVAL);
        }
        let guard=vm.retain_t8140_job()?;
        let new_client=!self.job.program.matches_vm(vm);
        let remap=self.generation==0 || new_client
            || self.job.targets.iter().zip(plan.sizes).any(|(target,size)|target.logical_size<size);
        if remap {
            self.job.program.rebind_retired(&self.storage)?;
            self.user_aliases.clear();self.pool_aliases.clear();
        }
        if self.generation==0 {
            let alias=self.job.graph.map_alias_at(uat.kernel_lower_vm(),va::GRAPHICS_JOB_LOW,
                mmu::PROT_GPU_SHARED_RW)?;
            self.job.aliases.push(alias,GFP_KERNEL)?;
        }
        for (index,target) in self.job.targets.iter_mut().enumerate() {
            let address=target.iova();
            target.regrow_wc_at(dev,&self.storage,address,plan.sizes[index])?;
            target.with_bytes_mut(|bytes| {
                bytes.fill(0);
                if index==2 {regs::initialize_g18_deflake(bytes,render.target.width,render.target.height).ok_or(EINVAL)?;}
                Ok(())
            })?;
            if remap {
                for alias in [address,va::COMPACT_BIAS+address] {
                    self.user_aliases.push(target.map_alias_at(vm,alias,mmu::PROT_GPU_SHARED_RW)?,GFP_KERNEL)?;
                }
            }
        }
        if remap {
            self.user_aliases.push(self.job.graph.map_alias_at(vm,va::GRAPHICS_JOB_LOW,
                mmu::PROT_GPU_SHARED_RW)?,GFP_KERNEL)?;
            let aliases=self.job.parameter_buffer.application_aliases(vm)?;
            for alias in aliases {self.user_aliases.push(alias,GFP_KERNEL)?;}
        }
        // Retract even an unchanged root before publish_h1's fresh lease
        // publication. Retain the mappings when the actual VM is unchanged.
        self.job.program.rebind_retired(vm)?;
        self.guard=Some(guard);self.generation=frame.generation;self.retired=false;
        self.client_frames=if new_client {1} else {self.client_frames.saturating_add(1)};
        if remap {
            crate::g18_kick_info!(dev.as_ref(),"G18_APP_VM frame={} vm={} context={} target={}x{} aliases={}\n",
                frame.generation,vm.id(),self.job.context_id(),render.target.width,render.target.height,self.user_aliases.len());
            for (target,name) in self.job.targets.iter().zip(assembly::TARGET_NAMES) {
                crate::g18_kick_info!(dev.as_ref(),"G18_APP_TARGET name={} va={:#x} bytes={:#x} context={}\n",
                    name,target.iova(),target.logical_size,self.job.context_id());
            }
        }
        self.job.parameters=render.parameters;self.job.controls=render.controls;
        self.scratch=render.scratch;
        assembly::apply_target_plan(&mut self.job.parameters,&plan);
        self.job.parameter_buffer.application_scene(Some(&self.scene))?;
        self.job.parameters.cycle=self.job.parameter_buffer.scene_cycle();
        self.job.parameters.record_index=self.job.parameter_buffer.metrics_record()|1;
        self.job.parameters.ta_hardware_buffer_id=self.job.parameter_buffer.command_fields().pb_slot;
        self.job.parameters.usc_flist_hardware_buffer_id=0;
        let p=&self.job.parameters;
        for address in [render.vdm_base,p.scissor_array,p.depth_bias_array,p.depth_buffer,p.depth_aux_buffer,
            p.load_pipeline,p.store_pipeline,p.partial_load_pipeline,p.partial_store_pipeline] {
            if address!=0 {vm.translate_iova(address)?;}
        }
        self.job.signals.with_bytes_mut(|out| {out[0xc000..0xc020].fill(0);Ok(())})?;
        self.job.encoded=None;
        self.encode_application(dev,frame)?;
        Ok(())
    }

    #[inline(never)]
    fn encode_application(&mut self, dev: &AsahiDrmDevice, frame: Frame) -> Result {
        let inputs=self.application_policy(frame)?;
        self.job.encode(dev,&inputs)
    }

    // Return the heap owner before entering the register builders. Otherwise
    // LLVM keeps the ~3.6 KiB command-input construction frame beneath them.
    #[inline(never)]
    fn application_policy(&mut self, frame: Frame) -> Result<KBox<cmd::G18RenderCommandInputs>> {
        let context=u32::from(self.job.context_id());
        let scratch=self.scratch;
        let job=&mut *self.job;
        let mut inputs=KBox::new(g18_application_runtime::inputs(&mut job.parameters,&mut job.controls,context,frame),GFP_KERNEL)?;
        // A render whose programs carry a thread-private frame takes both
        // launch pairs from its own plan; the TA/3D pool stays the owned one.
        // The adapter admitted it only at one sample, so no 4x override exists.
        if let Some(selection)=scratch {
            if job.controls.fragment_sample_control!=0 {return Err(EINVAL);}
            job.controls.ta.resource_word=selection.stages[0].resource_word();
            job.controls.fragment.resource_word=selection.stages[1].resource_word();
        }
        let pb=self.job.parameter_buffer.command_fields();
        inputs.ta.pb_slot=pb.pb_slot;inputs.ta.pm_1d4=pb.pm_1d4;
        inputs.ta.pb_state=pb.pb_state;inputs.fragment.pm_state=pb.pb_state;
        inputs.ta.pm_scene=pb.pm_scene;inputs.fragment.pm_scene=pb.pm_scene;
        inputs.ta.pm_b8=pb.pm_b8;inputs.fragment.pm_b8=pb.pm_b8;
        inputs.scheduler_record=self.job.scheduler_record()?;
        Ok(inputs)
    }

    pub(crate) fn pass(&self) -> Result<G18QueuedPass> {
        let mut pass=self.job.pass()?;pass.stamps=[self.frame().stamp();2];Ok(pass)
    }

    /// The VMs a page walk reads, as they are now; None while one is being
    /// changed.
    pub(crate) fn walk_key(&self, uat: &mmu::Uat) -> Option<WalkKey> {
        let mut epochs=[0;4];
        for (epoch,vm) in epochs.iter_mut().zip([&self.storage,self.job.vm(),uat.kernel_vm(),uat.kernel_lower_vm()]) {
            *epoch=vm.mapping_validation_epoch()?;
        }
        Some(WalkKey{client:self.job.vm().id(),epochs})
    }

    /// `walk`: also prove the job's mappings page by page (see walk_key()).
    pub(crate) fn audit(&mut self, dev: &AsahiDrmDevice, uat: &mmu::Uat,
        pb: &G18PbReadback, uma: &[u64;16], mut pages: usize, walk: bool) -> Result {
        let frame=self.frame();let pass=self.pass()?;
        if frame.generation==1 {self.job.queues.audit_before_pass(uat,&pass)?;}
        if pb.counter!=frame.generation || uma.iter().all(|v|*v==0)
            || u32::from_le_bytes(pb.state[8..12].try_into().map_err(|_|EINVAL)?)!=pass.context_id {return Err(EIO);}
        let scheduler=self.job.scheduler.snapshot(0)?;
        if scheduler.submitted!=frame.submitted()-2 || scheduler.completed!=frame.submitted()-2
            || scheduler.linked!=0 || self.job.stamps()?!=[frame.previous_stamp();2] {return Err(EBUSY);}
        if walk {
        for target in &self.job.targets {
            let pa=self.storage.translate_pages(target.iova(),target.logical_size)?;
            if self.job.vm().translate_pages(target.iova(),target.logical_size)?!=pa
                || self.job.vm().translate_pages(target.iova()+va::COMPACT_BIAS,target.logical_size)?!=pa {return Err(EFAULT);}
            pages+=pa.len();
        }
        pages+=self.job.parameter_buffer.audit_job_mappings(uat,self.job.vm(),va::COMPACT_BIAS)?;
        let pa=uat.kernel_vm().translate_pages(va::GRAPHICS_JOB_HIGH,va::GRAPHICS_JOB_BYTES)?;
        for vm in [uat.kernel_lower_vm(),self.job.vm()] {
            if vm.translate_pages(va::GRAPHICS_JOB_LOW,va::GRAPHICS_JOB_BYTES)?!=pa {return Err(EFAULT);}
        }
        pages+=pa.len();
        for (address,bytes) in [(va::GRAPHICS_SIGNALS,va::GRAPHICS_SIGNALS_BYTES),
            (va::GRAPHICS_SCHEDULER_START,0x8000),(va::PB_TABLE,0x4000),(va::UMA_TABLE,0x8000)] {
            pages+=uat.kernel_vm().translate_pages(address,bytes)?.len();
        }
        }
        let expected=self.job.encoded.as_ref().ok_or(EINVAL)?.digests;
        self.job.graph.with_bytes_mut(|out| {
            if crate::g18::fw::graphics_payload::fnv64_uncached(&out[..cmd::G18_TA_COMMAND_BYTES])!=expected[0]
                || crate::g18::fw::graphics_payload::fnv64_uncached(&out[0x4000..0x4000+cmd::G18_3D_COMMAND_BYTES])!=expected[1] {return Err(EIO);}Ok(())
        })?;
        if frame.generation<=3 {
            for (name,digest) in assembly::DIGEST_NAMES.iter().zip(expected) {
                dev_info!(dev.as_ref(),"G18_APP_DIGEST frame={} name={} fnv={:#018x}\n",frame.generation,name,digest);
            }
            dev_info!(dev.as_ref(),"G18_APP_AUDIT frame={} pb=resident uma=match translations={} previous_stamp={:#x}\n",frame.generation,pages,frame.previous_stamp());
        }
        if frame.generation==1 {
            self.job.parameter_buffer.print_h1_digests(dev,&pb.row)?;
            self.job.scheduler.print_render_digests(dev)?;
        }
        self.job.audited=true;Ok(())
    }

    pub(crate) fn publish(&mut self) -> Result<[u64;2]> {self.job.publish_h1()}
}
