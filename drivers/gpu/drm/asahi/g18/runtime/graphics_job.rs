// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Owned H1 graph, typed mapping checks and a single audited publication.
#[path = "graphics_job_application.rs"]
pub(crate) mod application;
use crate::{g18::buffer::object::MappedObject, g18::runtime::graphics_memory::G18GraphicsMemory,
    g18::queue::scheduler::{G18SchedulerMemory, G18SchedulerSnapshot},
    g18::queue::graphics::{G18GraphicsQueues, G18GraphicsQueueAddresses,
        G18ParameterBuffer, G18PbAddresses, G18PbPolicy, G18QueuedPass, G18PbReadback}};
use crate::{driver::AsahiDrmDevice, g18::fw::render_params::G17pRenderParameters,
    g18::address_map as va, g18::fw::render as regs, g18::fw::render_command as cmd,
    g18::fw::render_assembly::{self as assembly, EncodedJob, RenderVariant}, mmu};
use core::sync::atomic::{fence, Ordering};
use kernel::prelude::*;

pub(crate) struct G18GraphicsJob {
    // Payload's context lease retracts before the additional job mappings drop.
    program: G18GraphicsMemory,
    aliases: KVec<mmu::KernelMapping>,
    targets: KVec<MappedObject>,
    graph: MappedObject,
    signals: MappedObject,
    scheduler: G18SchedulerMemory,
    queues: G18GraphicsQueues,
    parameter_buffer: G18ParameterBuffer,
    pub parameters: G17pRenderParameters,
    pub controls: regs::G18RenderControls,
    variant: RenderVariant,
    encoded: Option<EncodedJob>,
    audited: bool,
}

impl G18GraphicsJob {
    #[inline(never)]
    pub(crate) fn from_payload(dev: &AsahiDrmDevice, uat: &mmu::Uat,
        profile: u32, variant: RenderVariant) -> Result<KBox<Self>> {
        if (variant == RenderVariant::GearsThreeFrames) != (profile == 3) { return Err(EINVAL); }
        let mut program = G18GraphicsMemory::load_payload(dev, uat, profile)?;
        let capture = match program.capture() {
            crate::g18::fw::graphics_payload::Capture::ColorA => regs::G18Capture::ColorA,
            crate::g18::fw::graphics_payload::Capture::ColorB => regs::G18Capture::ColorB,
            crate::g18::fw::graphics_payload::Capture::GearsG1 => regs::G18Capture::GearsG1,
            crate::g18::fw::graphics_payload::Capture::FragmentWitness => regs::G18Capture::FragmentWitness,
        };
        let (mut p, mut controls) = regs::g18_capture_parameters(capture);
        if crate::g18::params::g18_vdm_high_alias.value() == 1 {
            let source = p.encoder;
            p.encoder = program.alias_vdm_for_address_probe(source)?;
            dev_info!(dev.as_ref(), "G18_VDM_ADDRESS_CONTROL source={:#x} alias={:#x} compact={:#x} bytes=16384 contents=unchanged\n",
                source, va::GRAPHICS_HIGH_VDM, p.encoder);
            dev_info!(dev.as_ref(), "G18_RENDER_OBJECT name=vdm-address-control va={:#x} bytes={:#x} context={}\n",
                va::GRAPHICS_HIGH_VDM, va::PAGE, program.context_id());
        }
        if crate::g18::params::g18_ppp_high_alias.value() == 1 {
            program.alias_ppp_for_address_probe()?;
            dev_info!(dev.as_ref(), "G18_PPP_ADDRESS_CONTROL records=14 pages=4 contents=unchanged vdm_changes=ppp-pointers-only\n");
            for source in va::GRAPHICS_HIGH_PPP_PAGES {
                dev_info!(dev.as_ref(), "G18_RENDER_OBJECT name=ppp-address-control-{:x} va={:#x} bytes={:#x} context={}\n",
                    source, va::GRAPHICS_HIGH_PPP_BASE - 0x58000 + source, va::PAGE, program.context_id());
            }
        }
        if matches!(variant, RenderVariant::TaOnly | RenderVariant::Clear) {
            program.terminate_vdm(p.encoder)?;
        }
        if variant == RenderVariant::Clear { p.process_empty_tiles = true; }
        let plan = assembly::target_plan(&p).ok_or(EINVAL)?;
        let mut aliases = KVec::new();
        let mut targets = KVec::new();
        for i in 0..plan.sizes.len() {
            let address = va::GRAPHICS_TARGET_OFFSET + plan.offsets[i];
            let mut target = MappedObject::new_wc_at(dev, program.vm(), address,
                plan.sizes[i], mmu::PROT_GPU_SHARED_RW)?;
            if i == 2 {
                target.with_bytes_mut(|bytes|
                    regs::initialize_g18_deflake(bytes, p.width, p.height).ok_or(EINVAL))?;
            }
            aliases.push(target.map_alias_at(program.vm(), va::COMPACT_BIAS + address,
                mmu::PROT_GPU_SHARED_RW)?, GFP_KERNEL)?;
            targets.push(target, GFP_KERNEL)?;
        }
        assembly::apply_target_plan(&mut p, &plan);
        if crate::g18::params::g18_usc_statistics.value() != 0 {
            if variant != RenderVariant::Color || program.context_id() >= 64 {
                return Err(EINVAL);
            }
            // Original G18 setupSMAPPDesc: full GPU VA bits 6..42, context
            // in bits 56..61, enable bit 0. These are the separately owned,
            // zeroed target pages above, retained for the entire job lifetime.
            let encode = |address: u64| -> Result<u64> {
                if address & 63 != 0 || address & !0x7ff_ffff_ffc0 != 0 {
                    return Err(EINVAL);
                }
                Ok(address | (u64::from(program.context_id()) << 56) | 1)
            };
            controls.ta.usc_statistics = encode(p.ta_status)?;
            controls.fragment.usc_statistics = encode(p.fragment_status)?;
            dev_info!(dev.as_ref(), "G18_USC_STATISTICS context={} ta={:#x} fragment={:#x} backing=16384-each\n",
                program.context_id(), controls.ta.usc_statistics, controls.fragment.usc_statistics);
        }
        dev_info!(dev.as_ref(), "G18_RENDER_TARGET kind={} primary={:#x} tpc-allocation={:#x} native={:?}\n",
            crate::g18::uapi::render_policy::target_kind(), plan.sizes[0], plan.sizes[1], plan.native);
        let mut parameter_buffer = G18ParameterBuffer::new(dev, uat, program.vm(), G18PbAddresses {
            state_high: va::GRAPHICS_PB_HIGH, graph_high: va::PB_GRAPH,
            page_list_high: va::PB_PAGE_LIST_HIGH, page_list_low: va::PB_PAGE_LIST_LOW,
            page_list_row: va::COMPACT_BIAS + va::PB_PAGE_LIST_LOW,
            metrics_high: va::PB_METRICS_HIGH, metrics_low: va::PB_METRICS_LOW,
            scene_scratch_low: va::PB_SCENE_SCRATCH, blocks_low: va::PB_BLOCKS,
            discard_low: va::PB_DISCARD, user_view_delta: 0,
        }, G18PbPolicy { blocks: crate::g18::uapi::render_policy::PB_BLOCKS, capacity: crate::g18::uapi::render_policy::PB_CAPACITY, pm_154: 550, pm_15c: 1650,
            pm_1d4: 0, scene_count: 80, scene: 1, slot: 0, generation: 1 })?;
        parameter_buffer.alias_low_window(program.vm(), program.vm(), va::COMPACT_BIAS)?;
        p.cycle = parameter_buffer.scene_cycle();
        p.record_index = parameter_buffer.metrics_record() | 1;
        p.ta_hardware_buffer_id = 0;
        p.usc_flist_hardware_buffer_id = 0;
        let mut graph = MappedObject::new_wc_at(dev, uat.kernel_vm(), va::GRAPHICS_JOB_HIGH,
            va::GRAPHICS_JOB_BYTES, mmu::PROT_FW_PRIV_RW)?;
        for vm in [uat.kernel_lower_vm(), program.vm()] {
            aliases.push(graph.map_alias_at(vm, va::GRAPHICS_JOB_LOW,
                mmu::PROT_GPU_SHARED_RW)?, GFP_KERNEL)?;
        }
        let signals = MappedObject::new_wc_at(dev, uat.kernel_vm(), va::GRAPHICS_SIGNALS,
            va::GRAPHICS_SIGNALS_BYTES, mmu::PROT_FW_SHARED_RW)?;
        let mut scheduler = G18SchedulerMemory::new_render(dev, uat)?;
        scheduler.prepare(0, 0)?;
        let queues = G18GraphicsQueues::new(dev, uat, G18GraphicsQueueAddresses {
            entry_high: [va::GRAPHICS_ENTRY_HIGH, va::GRAPHICS_ENTRY_HIGH + 0x20000],
            entry_low: [va::GRAPHICS_ENTRY_LOW, va::GRAPHICS_ENTRY_LOW + 0x20000],
            graph: va::GRAPHICS_QUEUE_HIGH,
        })?;
        let mut job = KBox::init(try_init!(Self { program, aliases, targets, graph, signals, scheduler, queues, parameter_buffer,
            parameters: p, controls, variant, encoded: None, audited: false }), GFP_KERNEL)?;
        job.audit_mappings(uat)?;
        if job.program.output()?.1 != 65536 { return Err(EIO); }
        job.log_objects(dev);
        Ok(job)
    }

    pub(crate) fn vm(&self) -> &mmu::Vm { self.program.vm() }
    pub(crate) fn layout_only_payload(&self) -> bool { self.program.layout_only_payload() }
    pub(crate) fn context_id(&self) -> u8 { self.program.context_id() }
    pub(crate) fn scheduler_record(&self) -> Result<u64> { Ok(self.scheduler.binding(0)?.record) }
    pub(crate) fn queues(&mut self) -> &mut G18GraphicsQueues { &mut self.queues }
    pub(crate) fn parameter_buffer(&mut self) -> &mut G18ParameterBuffer { &mut self.parameter_buffer }
    #[inline(never)]
    pub(crate) fn encode_h1(&mut self, dev: &AsahiDrmDevice) -> Result {
        let inputs = self.h1_policy()?;
        self.encode(dev, &inputs)
    }

    #[inline(never)]
    fn h1_policy(&mut self) -> Result<KBox<cmd::G18RenderCommandInputs>> {
        let context = u32::from(self.context_id());
        let inputs = crate::g18::uapi::render_policy::h1_inputs(self.program.command(), context,
            &mut self.parameters, &mut self.controls);
        Ok(KBox::new(inputs, GFP_KERNEL)?)
    }

    pub(crate) fn pass(&self) -> Result<G18QueuedPass> {
        Ok(G18QueuedPass { command_high: [va::GRAPHICS_JOB_HIGH, va::GRAPHICS_JOB_HIGH+0x4000],
            context_id: u32::from(self.context_id()), scheduler_control: self.queues.scheduler_control(),
            scheduler_index: 0, uma: cmd::G18ConfigUma { state: va::GRAPHICS_UMA_STATE,
                generation: Some(0), slot: 0 }, parameter_buffer: Some(self.parameter_buffer.binding(10)),
            rce: self.encoded.as_ref().ok_or(EINVAL)?.rce, stamps: [0x100;2],
            event_generation: [0;2], ta_barrier_flag: crate::g18::uapi::render_policy::ta_barrier_flag() })
    }

    pub(crate) fn audit_h1(&mut self, dev: &AsahiDrmDevice, uat: &mmu::Uat,
        pb: &G18PbReadback, uma: &[u64;16]) -> Result {
        let pass = self.pass()?;
        self.queues.audit_before_pass(uat, &pass)?;
        if u32::from_le_bytes(pb.state[8..12].try_into().map_err(|_|EINVAL)?) != pass.context_id
            || pb.counter != 1 || uma.iter().all(|&v|v==0) { return Err(EIO); }
        let s = self.scheduler.snapshot(0)?;
        if s.submitted!=0 || s.completed!=0 || s.linked!=0 || self.program.output()?.1!=65536 { return Err(EBUSY); }
        for q in va::QUEUE_IDS { for bank in [0x4000,0x8000] {
            if self.signals.read_u32(va::GRAPHICS_SIGNALS+bank+u64::from(q)*4)?!=0 {return Err(EBUSY);}
        } }
        for off in [0xc000,0xc008,0xc010,0xc018] {
            if self.signals.read_u64(va::GRAPHICS_SIGNALS+off)?!=0 {return Err(EBUSY);}
        }
        let mut pages = self.audit_mappings(uat)?;
        pages += self.parameter_buffer.audit_job_mappings(uat,self.vm(),va::COMPACT_BIAS)?;
        // Command/RCE/target/PB/UMA graphs are owner-audited page by page.
        // Fixed firmware-only objects not covered by those owners:
        for (address,bytes) in [(va::GRAPHICS_SIGNALS,va::GRAPHICS_SIGNALS_BYTES),
            (va::GRAPHICS_SCHEDULER_START,0x8000),(va::PB_TABLE,0x4000),
            (va::UMA_TABLE,0x8000),(va::GRAPHICS_UMA_STATE,0x8000)] {
            pages += uat.kernel_vm().translate_pages(address,bytes)?.len();
        }
        // Captured shader/VDM/PPP pages are immutable except terminate/poison.
        // Validate all enabled direct register pointer inputs; bind/flag words
        // and shader offsets are not mistaken for full addresses.
        let p = &self.parameters;
        // The register carries a compact VDM offset, not a full GPU VA.
        // The low fixture alias used to hide this distinction in the audit.
        let vdm_address = va::COMPACT_BIAS.checked_add(p.encoder).ok_or(EINVAL)?;
        self.vm().translate_iova(vdm_address)?;
        pages += 1;
        for address in [p.tilemap,p.heapmeta,p.layermeta,p.tpc,p.deflake_1,p.deflake_2,p.deflake_3,
            p.ta_status,p.fragment_status,p.scissor_array,p.depth_bias_array,p.aux_fb,
            p.occlusion_query_base,p.depth_buffer,p.stencil_buffer,p.depth_aux_buffer,p.stencil_aux_buffer,
            p.sampler_array,p.cycle,p.record_index & !1,p.load_pipeline,p.store_pipeline,
            p.partial_load_pipeline,p.partial_store_pipeline] {
            if address!=0 { self.vm().translate_iova(address)?; pages+=1; }
        }
        let inputs=self.h1_policy()?;
        let f=&inputs.fragment;
        // Native descriptor copies of attachment/resource and shader pointers.
        // Other d0518 pairs are disabled in the admitted colour captures; the
        // last pair is a packed format word rather than a GPU pointer.
        for address in [f.d04e0,f.d04e8,f.d04f0,f.d0518[18][1],f.d0518[21][1],f.d0790[1],f.d07b0[1]] {
            if address!=0 {self.vm().translate_iova(address)?;pages+=1;}
        }
        for (index,pair) in f.d0518.iter().enumerate() {
            if ![18,21,22].contains(&index) && *pair!=[0;2] {return Err(ENOTSUPP);}
        }
        self.parameter_buffer.print_h1_digests(dev,&pb.row)?;
        self.scheduler.print_render_digests(dev)?;
        for (target,name) in self.targets.iter_mut().zip(assembly::TARGET_NAMES) {
            target.with_bytes_mut(|bytes| {
                dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name={} fnv={:#018x}\n",name,crate::g18::fw::graphics_payload::fnv64(bytes));Ok(())
            })?;
        }
        if self.variant==RenderVariant::Clear {
            let digest=self.program.terminated_digest(self.parameters.encoder)?;
            dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=terminated-vdm-page fnv={:#018x}\n",digest);
        }
        let expected=self.encoded.as_ref().ok_or(EINVAL)?.digests;
        self.graph.with_bytes_mut(|bytes| {
            let ta=&bytes[..cmd::G18_TA_COMMAND_BYTES];
            let fr=&bytes[assembly::FRAGMENT_OFFSET..assembly::FRAGMENT_OFFSET+cmd::G18_3D_COMMAND_BYTES];
            let f=crate::g18::fw::graphics_payload::fnv64;
            let actual=[f(ta),f(fr),f(&ta[0x60..0x60+91*12]),f(&fr[0xa0..0xa0+92*12]),
                f(&fr[0x7c0..0x7c0+17*12]),f(&fr[0xee0..0xee0+24*12]),f(&fr[0x1600..0x1600+12*12])];
            if actual!=expected {return Err(EIO);}
            for (name,digest) in assembly::DIGEST_NAMES.iter().zip(actual) {
                dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name={} fnv={:#018x}\n",name,digest);
            }
            Ok(())
        })?;
        let mut row=[0u8;128]; for (out,word) in row.chunks_exact_mut(8).zip(uma) {out.copy_from_slice(&word.to_le_bytes());}
        dev_info!(dev.as_ref(), "G18_RENDER_DIGEST name=uma-row fnv={:#018x}\n", crate::g18::fw::graphics_payload::fnv64(&row));
        dev_info!(dev.as_ref(), "G18_RENDER_AUDIT pb=match uma=match slots=0,1 scheduler=bound stamps=zero timestamps=zero poison=65536 translations={} policy=captured-M5-H1\n",pages);
        self.audited = true;
        Ok(())
    }

    pub(crate) fn publish_h1(&mut self) -> Result<[u64;2]> {
        if !self.audited {return Err(EINVAL);}
        self.program.publish()?;
        self.scheduler.publish_kick(0)?;
        self.scheduler.publish_kick(0)?;
        self.audited=false;
        Ok(self.program.roots())
    }

    pub(crate) fn stamps(&mut self) -> Result<[u32;2]> {
        Ok([self.signals.read_u32(va::GRAPHICS_SIGNALS+0x4000+9*4)?,
            self.signals.read_u32(va::GRAPHICS_SIGNALS+0x4000+10*4)?])
    }
    /// A firmware recovery ended this frame after its queues consumed it, so
    /// no stage will write its stamps. Publish them from the host; a stage
    /// that already wrote its own must have written exactly `stamp`.
    pub(crate) fn publish_recovered_stamps(&mut self, previous: u32, stamp: u32) -> Result {
        for slot in [9u64,10] {
            let address=va::GRAPHICS_SIGNALS+0x4000+slot*4;
            match self.signals.read_u32(address)? {
                value if value==stamp => {}
                value if value==previous => self.signals.write_u32(address,stamp)?,
                _ => return Err(EIO),
            }
        }
        fence(Ordering::SeqCst);
        Ok(())
    }
    pub(crate) fn pixels(&mut self) -> Result<([u32;8],u32,u32)> {
        self.program.verify_pixels(self.variant==RenderVariant::Clear)
    }
    pub(crate) fn capture_fields(&self) -> (cmd::G18RenderCommandInputs, usize) {
        let (mut inputs, open) = crate::g18::fw::render_passthrough::capture_fields(self.program.command());
        let pb = self.parameter_buffer.command_fields();
        inputs.ta.pb_slot = pb.pb_slot; inputs.ta.pm_1d4 = pb.pm_1d4;
        inputs.ta.pb_state = pb.pb_state; inputs.fragment.pm_state = pb.pb_state;
        inputs.ta.pm_scene = pb.pm_scene; inputs.fragment.pm_scene = pb.pm_scene;
        inputs.ta.pm_b8 = pb.pm_b8; inputs.fragment.pm_b8 = pb.pm_b8;
        (inputs, open)
    }

    /// Caller has allocated/retained PB and pool owners and filled all native
    /// fields. In particular a default resource word is not an admitted policy.
    #[inline(never)]
    pub(crate) fn encode(&mut self, _dev: &AsahiDrmDevice, inputs: &cmd::G18RenderCommandInputs) -> Result {
        if self.encoded.is_some() || self.controls.ta.resource_word == 0
            || self.controls.fragment.resource_word == 0
            || inputs.context_id != u32::from(self.context_id())
            || inputs.scheduler_record != self.scheduler_record()?
            || inputs.ta.pb_state == 0 || inputs.ta.pb_state != inputs.fragment.pm_state
            || inputs.ta.uma_state == 0 || inputs.ta.uma_state != inputs.fragment.uma_state {
            return Err(EINVAL);
        }
        let mut workspace = KBox::<assembly::Workspace>::new_uninit(GFP_KERNEL)?;
        // SAFETY: Workspace contains only integer scalars and integer arrays;
        // zero is valid for every field. Initialize the allocation in place.
        unsafe { core::ptr::write_bytes(workspace.as_mut_ptr(), 0, 1); }
        let mut workspace = unsafe { workspace.assume_init() };
        // Encode in cached memory and copy the commands to the write-combined
        // graph at once: the encoder and its digests read what they write.
        let mut commands = KVec::from_elem(0u8, cmd::G18_TA_COMMAND_BYTES + cmd::G18_3D_COMMAND_BYTES,
            GFP_KERNEL)?;
        let (ta, fragment) = commands.split_at_mut(cmd::G18_TA_COMMAND_BYTES);
        let encoded = assembly::encode_into(ta.try_into().map_err(|_| EINVAL)?,
            fragment.try_into().map_err(|_| EINVAL)?,
            &self.parameters, &self.controls, inputs, &mut workspace).map_err(|_| EINVAL)?;
        self.graph.with_bytes_mut(|bytes| {
            let (ta_page, rest) = bytes.split_at_mut(assembly::FRAGMENT_OFFSET);
            ta_page[..cmd::G18_TA_COMMAND_BYTES].copy_from_slice(ta);
            rest[..cmd::G18_3D_COMMAND_BYTES].copy_from_slice(fragment);
            Ok(())
        })?;
        self.encoded = Some(encoded);
        fence(Ordering::SeqCst);
        Ok(())
    }

    fn audit_mappings(&self, uat: &mmu::Uat) -> Result<usize> {
        let mut count = self.program.audit()?;
        for target in self.targets.iter() {
            let low = self.vm().translate_pages(target.iova(), target.logical_size)?;
            if self.vm().translate_pages(va::COMPACT_BIAS + target.iova(), target.logical_size)? != low {
                return Err(EFAULT);
            }
            count += low.len();
        }
        let pa = uat.kernel_vm().translate_pages(va::GRAPHICS_JOB_HIGH, va::GRAPHICS_JOB_BYTES)?;
        for vm in [uat.kernel_lower_vm(), self.vm()] {
            if vm.translate_pages(va::GRAPHICS_JOB_LOW, va::GRAPHICS_JOB_BYTES)? != pa { return Err(EFAULT); }
        }
        count += pa.len();
        Ok(count)
    }

    pub(crate) fn scheduler_snapshot(&mut self) -> Result<G18SchedulerSnapshot> { self.scheduler.snapshot(0) }
    pub(crate) fn output(&mut self) -> Result<(u64, u32)> { self.program.output() }

    fn log_objects(&self, dev: &AsahiDrmDevice) {
        for (i, target) in self.targets.iter().enumerate() {
            dev_info!(dev.as_ref(), "G18_RENDER_OBJECT name={} va={:#x} bytes={:#x} context={}\n",
                assembly::TARGET_NAMES[i], target.iova(), target.logical_size, self.context_id());
        }
        for (name, address, bytes, context) in [
            ("queue-state", va::GRAPHICS_QUEUE_HIGH, 0x8000, 0),
            ("ta-entry", va::GRAPHICS_ENTRY_HIGH, 0x20000, 0),
            ("3d-entry", va::GRAPHICS_ENTRY_HIGH + 0x20000, 0x20000, 0),
            ("ta-command", va::GRAPHICS_JOB_HIGH, cmd::G18_TA_COMMAND_BYTES, 0),
            ("3d-command", va::GRAPHICS_JOB_HIGH + 0x4000, cmd::G18_3D_COMMAND_BYTES, 0),
            ("signals", va::GRAPHICS_SIGNALS, va::GRAPHICS_SIGNALS_BYTES, 0),
            ("scheduler", va::GRAPHICS_SCHEDULER_START + 0x4000, 0x2400, 0),
            ("submitted", va::GRAPHICS_SCHEDULER_START, 0x90, 0),
            ("pb-table", va::PB_TABLE, 0x4000, 0), ("uma-table", va::UMA_TABLE, 0x8000, 0),
            ("pb-state", va::GRAPHICS_PB_HIGH, 0x4000, 0),
            ("pb-graph", va::PB_GRAPH, 0x8000, 0),
            ("pb-page-list", va::PB_PAGE_LIST_LOW, va::PB_PAGE_LIST_BYTES, self.context_id()),
            ("pb-metrics", va::PB_METRICS_LOW, 0x4000, self.context_id()),
            ("pb-scene-scratch", va::PB_SCENE_SCRATCH, 0x4000, self.context_id()),
            ("pb-discard", va::PB_DISCARD, 0x8000, self.context_id()),
            ("pb-blocks", va::PB_BLOCKS, crate::g18::uapi::render_policy::PB_BLOCKS as usize*0x80000, self.context_id()),
            ("uma-state", va::GRAPHICS_UMA_STATE, 0xa0, 0),
            ("uma-counter", va::GRAPHICS_UMA_COUNTER, 8, 0),
            ("uma-list", va::GRAPHICS_UMA_LIST, 0x2000, 0),
            ("pixels", crate::g18::fw::graphics_payload::OUTPUT, 0x40000, self.context_id()),
        ] {
            dev_info!(dev.as_ref(), "G18_RENDER_OBJECT name={} va={:#x} bytes={:#x} context={}\n",
                name, address, bytes, context);
        }
        for index in 0..crate::g18::uapi::render_policy::uma_blocks() {
            dev_info!(dev.as_ref(), "G18_RENDER_OBJECT name=uma-block-{} va={:#x} bytes=0x100000 context={}\n",
                index,crate::g18::uapi::render_policy::block_base()+u64::from(index)*va::POOL_BLOCK_STRIDE,self.context_id());
        }
    }
}
