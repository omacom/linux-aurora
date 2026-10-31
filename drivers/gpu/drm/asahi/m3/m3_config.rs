// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Owned RTKit 2419.140.12 configuration with named initialization references.
use kernel::prelude::*;
use crate::m3::m3_init_layout as init;
use crate::m3::m3_init_storage as storage;
use storage::{REGION_A,RUNTIME_POINTERS,HARDWARE_DATA,UNKNOWN_PAIR,FWLOG_PAYLOAD,
    UNKNOWN_C0,UNKNOWN_C1,UNKNOWN_C3,GLOBALS,GLOBALS_POWER,CONTROL_REGION,RUNTIME_FLAGS,
    FW_CONTROL_STATE,FW_CONTROL_RING};
use crate::{driver,m3::agx_memory::{self},m3::m3_memory::Buffer,mmu,pgtable::prot};
kernel::static_assert!(core::mem::size_of::<crate::fw::channels::RawFwLogMsg>() == init::fwlog::ENTRY_SIZE);
kernel::static_assert!(core::mem::size_of::<crate::fw::channels::RawFwLogPayloadMsg>() == init::fwlog::PAYLOAD_SIZE);
kernel::static_assert!(<crate::fw::channels::FwLogChannelState as crate::fw::channels::RxChannelState>::SUB_CHANNELS == init::fwlog::CHANNELS);

/// The root, runtime-pointer and firmware-control records, encoded for the owners
/// `region` describes. Config uploads them for its own allocations; the device-tree
/// InitData checks encode them for the fixed allocation table before GPU access.
pub(crate) struct Records {
    pub(crate) root: [u8; init::ROOT_SIZE],
    pub(crate) runtime: [u8; init::RUNTIME_SIZE],
    pub(crate) control: KVec<u8>,
}
impl Records {
    pub(crate) fn encode(region: impl Fn(usize) -> Result<init::Region>) -> Result<Self> {
        let root = init::Root { region_a: region(REGION_A)?, runtime: region(RUNTIME_POINTERS)?,
            globals: region(GLOBALS)?, new_a: region(RUNTIME_FLAGS)?,
            control: region(CONTROL_REGION)?, power: region(GLOBALS_POWER)? };
        let mut channels = [init::Channel { state: region(0)?, ring: region(1)? }; 17];
        for (index, channel) in channels.iter_mut().enumerate() {
            *channel = init::Channel { state: region(index * 2)?, ring: region(index * 2 + 1)? };
        }
        let runtime = init::RuntimePointers { hardware: region(HARDWARE_DATA)?,
            unknown_pair: region(UNKNOWN_PAIR)?, fwlog_payload: region(FWLOG_PAYLOAD)?,
            unknown_c0: region(UNKNOWN_C0)?, unknown_c1: region(UNKNOWN_C1)?,
            unknown_c3: region(UNKNOWN_C3)?, channels };
        let control = init::ControlRegion { channel: init::Channel {
            state: region(FW_CONTROL_STATE)?, ring: region(FW_CONTROL_RING)? } };
        let mut root_bytes = [0; init::ROOT_SIZE];
        root.encode(&mut root_bytes).map_err(|_| EINVAL)?;
        let mut runtime_bytes = [0; init::RUNTIME_SIZE];
        runtime.encode(&mut runtime_bytes).map_err(|_| EINVAL)?;
        // The control/status region is 50 KiB; use temporary heap storage,
        // never a kernel-stack array. All three owners precede firmware startup.
        let mut control_bytes = KVec::new();
        control_bytes.resize(init::CONTROL_SIZE, 0u8, GFP_KERNEL)?;
        control.encode(&mut control_bytes).map_err(|_| EINVAL)?;
        Ok(Self { root: root_bytes, runtime: runtime_bytes, control: control_bytes })
    }
}

pub(crate) struct Config {
    objects: KVec<Buffer>,
    profile: super::m3_profile::Profile,
    control_generation: u32,
    io_descriptors: Option<KVec<u8>>,
    _iomaps: KVec<mmu::KernelMapping>,
    pub(crate) completed_events: u64,
    pstates: crate::m3::m3_adt_config::PstateWatch,
    thermal: crate::m3::m3_thermal::Governor,
    request_top: u32,
    request_value: u32,
    request_checked: bool,
    request_check_at: Option<kernel::time::Instant<kernel::time::Monotonic>>,
    request_writes: u64,
    request_observed: u32,
}
impl Config {

    pub(crate) fn new(dev: &driver::AsahiDrmDevice, uat: &mmu::Uat, firmware: &crate::m3::m3_firmware::Firmware,
        contents: &crate::m3::m3_adt_config::Contents) -> Result<Self> {
        contents.check_for_upload()?;
        if let Some(inputs) = &contents.extended {
            if firmware.profile() != super::m3_profile::Profile::Extended { return Err(EINVAL); }
            let owners = ExtendedStorage::new(dev, uat, &inputs.power, &inputs.sram_uv, inputs.bases)?;
            let thermal = crate::m3::m3_thermal::Governor::new(dev.as_ref().as_ref(), &contents.pstates)?;
            return Ok(Self { objects: owners.objects, _iomaps: owners.iomaps,
                profile: firmware.profile(), control_generation: 0, completed_events: 0,
                io_descriptors: Some(owners.descriptors),
                pstates: crate::m3::m3_adt_config::PstateWatch::new(contents.pstates), thermal,
                request_top: 8, request_value: 0, request_checked: false, request_check_at: None,
                request_writes: 0, request_observed: 0 });
        }
        let mut objects = KVec::new();
        let mut iomaps = KVec::new();
        for index in 0..storage::COUNT {
            let mut a=storage::allocation(index).map_err(|_|EINVAL)?;
            let image=contents.initial(index,&a)?;
            if a.initial!=storage::Initial::Zero && image.len()!=a.size {return Err(EINVAL);}
            // The SoC may place the HwData object elsewhere and have it mapped cacheable
            // (`Soc::hwdata_object`); the records and the upload check use the live address.
            let object=if index==HARDWARE_DATA {contents.hwdata_object} else {None};
            if let Some(o)=object {a.address=o.address;}
            let mut b=match object {
                Some(o) if o.cached=>Buffer::at_prot(dev,uat.kernel_vm(),None,Some(a.address),a.size,
                    prot::PROT_FW_PRIV_RW,prot::PROT_GPU_SHARED_RW)?,
                _=>Buffer::at(dev,uat.kernel_vm(),None,Some(a.address),a.size,a.gpu_shared)?,
            };
            b.write(0,image)?;objects.push(b,GFP_KERNEL)?;
        }
        for io in contents.iomaps {
            // Read-write unless the contents say otherwise (only the T8122 start experiment).
            let protection=if contents.read_only_slots==0 {prot::PROT_FW_MMIO_RW}
                else {crate::m3::t8122_start::mmio_prot(contents.read_only_slots,io.slot)};
            let map=uat.kernel_vm().map_io_m3(io.address,io.physical.try_into()?,io.size,protection)?;
            let owned=init::Region::new(map.iova(),io.size).map_err(|_|EINVAL)?;
            let field=io.pointer_field().map_err(|_|EINVAL)?;
            objects[HARDWARE_DATA].u64(field,io.pointer(owned).map_err(|_|EINVAL)?)?;
            iomaps.push(map,GFP_KERNEL)?;
        }
        // This field is a physical address owned by the reserved GPU region,
        // unlike the firmware virtual references in the IOMapping records.
        objects[HARDWARE_DATA].u64(storage::GPU_REGION_PHYSICAL,firmware.resources.regions[0].base)?;
        Self::initialize_records(&mut objects)?;
        // The root record carries the G15 14.8.3 InitData version, which is every T6030
        // image's. Only the T8122 start experiment gives the firmware another one
        // (`asahi.t8122_initdata_version`).
        if firmware.initdata_magic!=crate::m3::m3_firmware::G15_V14_8_3_INITDATA {
            objects[storage::ROOT].u64(0,firmware.initdata_magic)?;
        }
        // Vertex/tessellation loops need not retire a primitive at every
        // firmware progress poll. The inherited interval of 10 falsely
        // declares a valid 35.7 ms TA job stuck on J514S. Keep progress
        // detection enabled, with a finite interval of 100 firmware polls.
        // This is independent of the host job bound and 60-second watchdog.
        // RTKit 2419 copies Globals+0x980 into its TA poll threshold at
        // text+0x6514; text+0xd710 checks it before sampling engine progress.
        const TA_PROGRESS_INTERVAL: usize = 0x980;
        if objects[GLOBALS].read_u32(TA_PROGRESS_INTERVAL)? != 10 { return Err(EINVAL); }
        objects[GLOBALS].u32(TA_PROGRESS_INTERVAL, 100)?;
        dev_info!(dev.as_ref(), "M3: firmware TA progress-check interval=100\n");
        // RTKit 2419 +2de48..2dec8 accepts user timestamp stores only
        // within the 64 MiB arena at HwDataB+28. Match map_timestamp().
        objects[HARDWARE_DATA].u64(0x28,agx_memory::TIMESTAMP_RANGE.start)?;
        // RTKit 2419 HwDataA follows the 0x4580-byte HwDataB. Request the
        // boot performance state through firmware (asahi.m3_boot_pstate) and cap
        // the states it may use (asahi.m3_max_pstate), retaining the voltage
        // tables, power/thermal ceilings and minimum operating state.
        // The bring-up image initialized actual/target and every base to 1
        // (338 MHz), so short desktop jobs never left that startup state.
        // The uploaded table is the SoC's (`Soc::pstates`): its highest state at +0xb58 and that
        // state's frequency in the table at +0xb5c (8 and 1380 MHz on T6030).
        let (states, top_mhz) = contents.table;
        if objects[HARDWARE_DATA].read_u32(0xb58)? != states
            || objects[HARDWARE_DATA].read_u32(0xb5c + states as usize * 4)? != top_mhz
        {
            return Err(EINVAL);
        }
        let (hwdata, globals) = objects.split_at_mut(GLOBALS);
        crate::m3::m3_adt_config::publish_pstates(dev.as_ref().as_ref(), &mut hwdata[HARDWARE_DATA], &mut globals[0], &contents.pstates)?;
        // Firmware ktrace is for explicit detailed diagnostics. Normal games
        // still drain error/events, but need not generate trace records for
        // every firmware operation on the command-processing core.
        objects[RUNTIME_FLAGS].u32(0,u32::from(crate::debug::debug_enabled(
            crate::debug::DebugFlags::SubmitTiming)))?;
        crate::m3::m3_adt_config::check_upload(dev.as_ref().as_ref(), firmware, contents.iomaps, &mut objects)?;
        agx_memory::publish();
        let thermal = crate::m3::m3_thermal::Governor::new(dev.as_ref().as_ref(), &contents.pstates)?;
        Ok(Self { objects, profile: firmware.profile(), control_generation: 0, io_descriptors: None,
            _iomaps: iomaps, completed_events: 0,
            pstates: crate::m3::m3_adt_config::PstateWatch::new(contents.pstates), thermal,
            request_top:contents.table.0,request_value:0,request_checked:false,
            request_check_at:None,request_writes:0,request_observed:0 })
    }
    fn initialize_records(objects: &mut [Buffer]) -> Result {
        let records = Records::encode(|index: usize| {
            init::Region::new(objects[index].va(), objects[index].size()).map_err(|_| EINVAL)
        })?;
        objects[storage::ROOT].write(0, &records.root)?;
        objects[RUNTIME_POINTERS].write(0, &records.runtime)?;
        objects[CONTROL_REGION].write(0, &records.control)
    }
    pub(crate) fn publish_io_table(&self,
        pdev: &kernel::platform::Device<kernel::device::Core<'_>>,
        firmware: &crate::m3::m3_firmware::Firmware) -> Result {
        if let Some(descriptors) = &self.io_descriptors {
            firmware.publish_io_table(pdev, descriptors)?;
        }
        Ok(())
    }

    pub(crate) fn global_table_views(&mut self, vm: &mmu::Vm) -> Result<KVec<mmu::KernelMapping>> {
        let mut mappings = KVec::new();
        if self.profile == super::m3_profile::Profile::Extended {
            for index in [super::m3_init_26::GLOBAL_POOL, super::m3_init_26::GLOBAL_PARAMETERS] {
                mappings.push(self.objects[index].map_gpu_view(vm)?, GFP_KERNEL)?;
            }
        }
        Ok(mappings)
    }

    pub(crate) fn root(&self) -> u64 { self.objects[storage::ROOT].va() }
    pub(crate) fn log_globals_debug(&mut self,dev:&driver::AsahiDrmDevice)->Result {
        let value=self.objects[GLOBALS].read_u32(crate::m3::m3_globals_debug::OFFSET)?;
        dev_info!(dev.as_ref(),"M3_GLOBALS_DEBUG off={} value={} request_level={}\n",
            u8::from(crate::m3::m3_params::globals_debug_off()),value,crate::m3::m3_params::consistent_perf());
        Ok(())
    }
    pub(crate) fn performance_request(&mut self,chip:u32,busy:bool)->Result {
        // Native controller inputs supersede the legacy fixed-request interface.
        if self.profile == super::m3_profile::Profile::Extended { return Ok(()); }
        let Some(value)=crate::m3::m3_consistent_perf::request(chip,
            crate::m3::m3_params::consistent_perf(),busy,self.request_top) else {return Ok(());};
        if value==0 && self.request_value==0 {return Ok(());}
        let now=kernel::time::Instant::<kernel::time::Monotonic>::now();
        let elapsed=self.request_check_at.map(|last|(now-last).as_nanos());
        if !crate::m3::m3_consistent_perf::refresh_due(value,self.request_value,elapsed) {
            return Ok(());
        }
        type P=crate::m3::g15_raw::raw::PowerCtlBlock;
        let at=core::mem::offset_of!(P,consistent_perf_state);
        if !self.request_checked {
            let mut flag=[0];
            self.objects[GLOBALS_POWER].read(core::mem::offset_of!(P,perf_ctrl_override_active),&mut flag)?;
            if flag[0]!=0 || self.objects[GLOBALS_POWER].read_u32(at)?!=0 {return Err(EINVAL);}
            self.request_checked=true;
        }
        let observed=self.objects[GLOBALS_POWER].read_u32(at)?;
        self.request_observed=observed;
        if observed!=value {
            self.objects[GLOBALS_POWER].packed_request(at,value)?;
            agx_memory::publish();
            self.request_writes=self.request_writes.saturating_add(1);
        }
        self.request_value=value;
        self.request_check_at=Some(now);
        Ok(())
    }
    /// Move the runtime cap by temperature (the thermal limit), then check the performance
    /// state the firmware reports against the ceiling and the runtime cap. Called after boot and
    /// after every job, by the job thread.
    pub(crate) fn check_pstate(&mut self, dev: &driver::AsahiDrmDevice, device: &crate::m3::m3_device::Device, what: &str) -> Result {
        if self.profile == super::m3_profile::Profile::Extended {
            let state = device.pstate_register()? & 15;
            if state > self.request_top {
                dev_err!(dev.as_ref(), "M3: native performance state {} exceeds {} ({})\n", state, self.request_top, what);
                return Err(ERANGE);
            }
            return Ok(());
        }
        let now = kernel::time::Instant::<kernel::time::Monotonic>::now();
        if let Some(cap) = self.thermal.update(dev.as_ref().as_ref(), now) {
            crate::m3::m3_adt_config::publish_runtime_cap(&mut self.objects[GLOBALS], cap)?;
        }
        let allowed = self.thermal.allowed(now);
        let (over, peak) = self.pstates.check(dev.as_ref().as_ref(), &mut self.objects[HARDWARE_DATA], || device.pstate_register(), what, allowed)?;
        self.thermal.note_check(dev.as_ref().as_ref(), now, over, peak)?;
        self.thermal.busy(now);
        Ok(())
    }
    pub(crate) fn stats_region(&self) -> Result<init::Region> {
        let owner = &self.objects[if self.profile == super::m3_profile::Profile::Extended {
            UNKNOWN_C3
        } else { HARDWARE_DATA }];
        init::Region::new(owner.va(), owner.size()).map_err(|_| EINVAL)
    }

    pub(crate) fn log_active_power(&mut self, dev: &driver::AsahiDrmDevice,
                                  device: &crate::m3::m3_device::Device,
                                  sequence: u32) -> Result {
        if self.profile == super::m3_profile::Profile::Extended {
            let state = device.pstate_register()? & 15;
            let performance = &mut self.objects[super::m3_init_26::PERFORMANCE_CONTROL];
            let actual = performance.read_u32(0x2c)?;
            let target = performance.read_u32(0x30)?;
            let hardware = performance.read_u32(0x38)?;
            dev_info!(dev.as_ref(), "M3_NATIVE_POWER sample={} state={} actual={} target={} hardware={}\n",
                sequence, state, actual, target, hardware);
            return Ok(());
        }
        let cap = self.thermal.allowed(kernel::time::Instant::<kernel::time::Monotonic>::now());
        self.pstates.log_active(dev.as_ref().as_ref(), &mut self.objects[HARDWARE_DATA],
                               device.pstate_register()?, cap, sequence)?;
        type P=crate::m3::g15_raw::raw::PowerCtlBlock;
        type G=crate::m3::g15_raw::raw::GlobalsG15V14_8_3;
        let request=self.objects[GLOBALS_POWER].read_u32(core::mem::offset_of!(P,consistent_perf_state))?;
        let cap_word=self.objects[GLOBALS].read_u32(core::mem::offset_of!(G,perf_state_cap))?;
        let power_if=core::mem::offset_of!(G,power_if_target);
        let target1=self.objects[GLOBALS].read_u32(power_if+4)?;
        let target2=self.objects[GLOBALS].read_u32(power_if+8)?;
        let mut override_active=[0];
        self.objects[GLOBALS_POWER].read(core::mem::offset_of!(P,perf_ctrl_override_active),&mut override_active)?;
        dev_info!(dev.as_ref(),"M3_PERF_REQUEST sample={} level={} request={} cap_word={} target1={} target2={} writes={} observed={} override_active={}\n",
            sequence,crate::m3::m3_params::consistent_perf(),request,cap_word,target1,target2,
            self.request_writes,self.request_observed,override_active[0]);
        Ok(())
    }

    pub(crate) fn render_pb(&mut self) -> Result {
        if self.profile == super::m3_profile::Profile::Extended { return Err(EOPNOTSUPP); }
        self.objects[UNKNOWN_C1].write(0,&storage::parameter_buffer(crate::m3::m3_render::PB_PAGES).map_err(|_|EINVAL)?)
    }
    pub(crate) fn submit_queue(&mut self, kind:usize, queue:u64, head:u16,event:u8,new:bool)->Result {
        if kind>2 {return Err(EINVAL);}
        let state=kind*2;let ring=state+1;
        let w=self.objects[state].read_u32(0x20)?;
        if w>=256 || self.objects[state].read_u32(0)?!=w { return Err(EBUSY); }
        let mut message=[0u8;0x18];
        message[8..16].copy_from_slice(&queue.to_le_bytes());
        message[16..20].copy_from_slice(&(kind as u32).to_le_bytes());
        message[20..22].copy_from_slice(&head.to_le_bytes());
        message[22]=event;message[23]=u8::from(new); // New queue; caller selected the event slot.
        self.objects[ring].write(w as usize*0x18,&message)?;
        agx_memory::publish();
        self.objects[state].u32(0x20,(w+1)%256)?;
        agx_memory::publish();
        Ok(())
    }
    /// Whether the firmware has consumed every message on pipe `kind` (0 TA, 1 3D, 2 compute).
    pub(crate) fn pipe_free(&mut self, kind:usize) -> Result<bool> {
        if kind>2 {return Err(EINVAL);}
        Ok(self.objects[kind*2].read_u32(0)? == self.objects[kind*2].read_u32(0x20)?)
    }
    pub(crate) fn pipes_idle(&mut self) -> Result<bool> {
        for state in [0,2,4] {
            if self.objects[state].read_u32(0)? != self.objects[state].read_u32(0x20)? {
                return Ok(false);
            }
        }
        Ok(true)
    }
    pub(crate) fn ready(&mut self) -> Result<bool> {
        if self.profile == super::m3_profile::Profile::Extended {
            return Ok(self.objects[RUNTIME_FLAGS].read_u32(0x14)? == 1);
        }
        // NewRegionB+4 is an input initialized to 1, not an acknowledgement.
        // These are firmware-written transitions present in compute-15's
        // pre-submit snapshot; both start at zero in the exported schema.
        Ok(self.objects[RUNTIME_POINTERS].read_u32(0x314)? == 0xabcdabcd
            && self.objects[RUNTIME_FLAGS].read_u32(4)? == 1)
    }
    pub(crate) fn log_ready(&mut self, dev: &driver::AsahiDrmDevice) -> Result {
        if self.profile == super::m3_profile::Profile::Extended {
            let ready = self.objects[RUNTIME_FLAGS].read_u32(0x14)?;
            let consumed = self.objects[CONTROL_REGION].read_u32(0xeae8)?;
            dev_info!(dev.as_ref(), "M3: extended firmware ready={} control-generation={}/{}\n",
                ready, consumed, self.control_generation);
            return Ok(());
        }
        let magic=self.objects[RUNTIME_POINTERS].read_u32(0x314)?;
        let state=self.objects[RUNTIME_POINTERS].read_u32(0x2fc)?;
        let a=self.objects[RUNTIME_FLAGS].read_u32(4)?;
        let b=self.objects[RUNTIME_FLAGS].read_u32(0x14)?;
        let c=self.objects[CONTROL_REGION].read_u32(0x45c4)?;
        dev_info!(dev.as_ref(),"M3 firmware readiness: magic={:#x} runtime={} newA={}/{} newB={}\n",magic,state,a,b,c);
        Ok(())
    }
    /// The firmware's halt counter and its halted and resume flags.
    pub(crate) fn halt_state(&mut self) -> Result<(u64,u32,u32)> {
        if self.profile == super::m3_profile::Profile::Extended { return Err(EOPNOTSUPP); }
        let control=&mut self.objects[CONTROL_REGION];
        Ok((control.read_u64(0x4580)?,control.read_u32(0x4590)?,control.read_u32(0x45a0)?))
    }
    /// Let a halted firmware continue: clear halted, then set resume. These
    /// words keep the halt/resume layout of earlier Apple GPU firmware.
    pub(crate) fn resume_halted(&mut self) -> Result {
        if self.profile == super::m3_profile::Profile::Extended { return Err(EOPNOTSUPP); }
        let control=&mut self.objects[CONTROL_REGION];
        control.u32(0x4590,0)?;
        agx_memory::publish();
        control.u32(0x45a0,1)?;
        agx_memory::publish();
        Ok(())
    }
    /// Read-only snapshot of the firmware's recovery words in the control
    /// region (InitData+0xb0): the halt counter and the halted/resume flags
    /// at +0x4580/+0x4590/+0x45a0, the firmware's own recovery information
    /// at +0x40a0, and the details it leaves for a recovery event at +0x44a0.
    /// Nothing is acknowledged, cleared or resumed here.
    pub(crate) fn log_recovery_state(&mut self, dev: &driver::AsahiDrmDevice, events: u64) -> Result {
        if self.profile == super::m3_profile::Profile::Extended {
            for (owner, start, length) in [(super::m3_init_26::RECOVERY_ACTIVITY, 0, 0x60),
                (CONTROL_REGION, 0x4ebc, 0x100)] {
                for offset in (start..start + length).step_by(32) {
                    let mut bytes = [0; 32];
                    self.objects[owner].read(offset, &mut bytes)?;
                    dev_err!(dev.as_ref(), "M3: firmware recovery owner={} offset={:#x} bytes={:02x?}\n",
                        owner, offset, bytes);
                }
            }
            return Ok(());
        }
        let control=&mut self.objects[CONTROL_REGION];
        let halt_count=control.read_u64(0x4580)?;
        let halted=control.read_u32(0x4590)?;
        let resume=control.read_u32(0x45a0)?;
        let info=[control.read_u32(0x40a0)?,control.read_u32(0x40a4)?];
        let mut packet=[0u8;0x14];
        control.read(0x44a0,&mut packet)?;
        dev_info!(dev.as_ref(),"M3 firmware recovery state: halt_count={} halted={} resume={} info={:#x}/{:#x} packet={:02x?} completed_events={} event_messages={}\n",
            halt_count,halted,resume,info[0],info[1],packet,self.completed_events,events);
        Ok(())
    }
    /// Read-only: log the event ring entries from the host's read index to the firmware's write
    /// index (at most 16), as `drain` reads them. Nothing is consumed or acknowledged, and the
    /// completion count is not changed.
    pub(crate) fn log_event_ring(&mut self, dev: &driver::AsahiDrmDevice) -> Result {
        let size = if self.profile == super::m3_profile::Profile::Extended { 0x48 } else { 0x38 };
        const COUNT: usize = 256;
        let r = self.objects[storage::EVENT*2].read_u32(0)? as usize;
        let w = self.objects[storage::EVENT*2].read_u32(0x20)? as usize;
        dev_info!(dev.as_ref(), "M3 event ring: read={} write={}\n", r, w);
        if r >= COUNT || w >= COUNT { return Err(EIO); }
        let mut index = r;
        for _ in 0..16 {
            if index == w { break; }
            let mut raw = [0u8; 0x48];
            self.objects[storage::EVENT*2+1].read(index * size, &mut raw[..size])?;
            dev_info!(dev.as_ref(), "M3 event ring entry {} kind={} bytes={:02x?}\n",
                index, u32::from_le_bytes([raw[0], raw[1], raw[2], raw[3]]), raw);
            index = (index + 1) % COUNT;
        }
        Ok(())
    }
    pub(crate) fn control(&mut self, opcode: u32) -> Result<u32> {
        if !matches!(opcode, 0x13 | 9) { return Err(EINVAL); }
        let w = self.objects[storage::DEVICE_CONTROL*2].read_u32(0x20)?;
        let r = self.objects[storage::DEVICE_CONTROL*2].read_u32(0)?;
        if w >= 256 || r != w { return Err(EBUSY); }
        let mut bytes = [0; 0x38];
        bytes[..4].copy_from_slice(&opcode.to_le_bytes());
        self.objects[storage::DEVICE_CONTROL*2+1].write(w as usize * 0x38, &bytes)?;
        agx_memory::publish();
        let next = (w + 1) % 256;
        self.objects[storage::DEVICE_CONTROL*2].u32(0x20, next)?;
        agx_memory::publish();
        Ok(next)
    }
    /// Publish an extended-format synchronous control. The caller rings the
    /// doorbell after this method and waits for both retirement cursors.
    pub(crate) fn control_extended(&mut self, action: super::m3_channel_format::Action) -> Result<u32> {
        use super::m3_channel_format as format;
        if self.profile != super::m3_profile::Profile::Extended { return Err(EINVAL); }
        let state = storage::DEVICE_CONTROL * 2;
        let cursors = [self.objects[state].read_u32(0)?,
            self.objects[state].read_u32(0x10)?, self.objects[state].read_u32(0x20)?];
        let cursor = format::idle_cursor(cursors).map_err(|error| {
            if error == init::Error::Bounds { EIO } else { EBUSY }
        })?;
        let consumed = self.objects[CONTROL_REGION].read_u32(0xeae8)?;
        let generation = action.generation(self.control_generation, consumed).map_err(|_| EOVERFLOW)?;
        let mut record = [0; format::CONTROL_BYTES];
        action.encode(&mut record).map_err(|_| EINVAL)?;
        self.objects[state + 1].write(cursor as usize * format::CONTROL_BYTES, &record)?;
        if generation != 0 {
            self.objects[GLOBALS].write(0xe41, &generation.to_le_bytes())?;
        }
        agx_memory::publish();
        let next = (cursor + 1) % format::SLOTS;
        self.objects[state].u32(0x20, next)?;
        agx_memory::publish();
        self.control_generation = generation;
        Ok(next)
    }

    pub(crate) fn control_done(&mut self, next: u32) -> Result<bool> {
        Ok(self.objects[storage::DEVICE_CONTROL*2].read_u32(0)? == next && self.objects[storage::DEVICE_CONTROL*2].read_u32(0x10)? == next)
    }
    pub(crate) fn drain(&mut self, dev: &driver::AsahiDrmDevice) -> Result {
        if self.profile == super::m3_profile::Profile::Extended {
            return self.drain_extended(dev);
        }
        // Event, trace, stats and six independent firmware log rings.
        for (state, ring, size, count, channels) in [(storage::EVENT*2,storage::EVENT*2+1,0x38,256,1),
            (storage::TRACE*2,storage::TRACE*2+1,0x38,512,1),
            (storage::STATS*2,storage::STATS*2+1,0x40,256,1),
            (storage::FIRMWARE_LOG*2,storage::FIRMWARE_LOG*2+1,init::fwlog::ENTRY_SIZE,init::fwlog::SLOTS,init::fwlog::CHANNELS)] {
            for channel in 0..channels {
                let base = channel * 0x30;
                let mut r = self.objects[state].read_u32(base)? as usize;
                let w = self.objects[state].read_u32(base+0x20)? as usize;
                if r >= count || w >= count { return Err(EIO); }
                let previous_read = r;
                while r != w {
                    if (state==storage::TRACE*2 && self.completed_events<4) || state==storage::FIRMWARE_LOG*2 {
                        let mut raw=[0u8;0xd8];
                        self.objects[ring].read((channel*count+r)*size,&mut raw[..size])?;
                        dev_info!(dev.as_ref(),"M3 fw-channel {} {:02x?}\n",state,&raw[..size]);
                        if state == storage::FIRMWARE_LOG*2 && self.objects[ring].read_u32((channel*count+r)*size)? == 2 {
                            let index = self.objects[ring].read_u64((channel*count+r)*size+8)?;
                            let offset = init::fwlog::payload_offset(channel,index).map_err(|_| EIO)?;
                            let mut payload = [0u8;init::fwlog::PAYLOAD_SIZE];
                            self.objects[FWLOG_PAYLOAD].read(offset,&mut payload)?;
                            dev_info!(dev.as_ref(),"M3 fwlog payload {} {:02x?}\n",channel,payload);
                        }
                    }
                    if state == storage::EVENT*2 {
                        let kind = self.objects[ring].read_u32(r*size)?;
                        if kind == 1 {
                            let flags=self.objects[ring].read_u64(r*size+4)?;
                            self.completed_events+=u64::from((flags&7).count_ones());
                        }
                        if kind == 0 || kind == 4 {
                            let mut raw=[0u8;0x38];self.objects[ring].read(r*size,&mut raw)?;
                            dev_err!(dev.as_ref(), "M3 firmware error event {} bytes={:02x?}\n", kind,raw);
                            return Err(EIO);
                        }
                    }
                    r = (r+1) % count;
                }
                // Publish actual consumption only, as RxChannel::get does.
                // Empty polls need no write to firmware's shared state line.
                if r != previous_read { self.objects[state].u32(base, r as u32)?; }
            }
        }
        Ok(())
    }
    fn drain_extended(&mut self, dev: &driver::AsahiDrmDevice) -> Result {
        use super::m3_channel_format as format;
        // Snapshot each producer once. New arrivals remain for the next drain;
        // a corrupt cursor is rejected before reading its ring allocation.
        for (channel, capacity, threads) in [(storage::FIRMWARE_LOG, 256, 9),
            (storage::EVENT, 256, 1), (storage::TRACE, 512, 1), (storage::STATS, 256, 1)] {
            let state = channel * 2;
            for thread in 0..threads {
                let at = thread * 0x30;
                let mut read = self.objects[state].read_u32(at)?;
                let write = self.objects[state].read_u32(at + 0x20)?;
                if read >= capacity || write >= capacity { return Err(EIO); }
                let before = read;
                while read != write {
                    let offset = (thread * capacity as usize + read as usize) * format::RECEIVE_BYTES;
                    let mut record = [0; format::RECEIVE_BYTES];
                    self.objects[state + 1].read(offset, &mut record)?;
                    let kind = u32::from_le_bytes(record[..4].try_into().map_err(|_| EIO)?);
                    if channel == storage::FIRMWARE_LOG {
                        let slot = u32::from_le_bytes(record[8..12].try_into().map_err(|_| EIO)?);
                        let payload = format::payload_offset(thread, slot).map_err(|_| EIO)?;
                        if crate::debug::debug_enabled(crate::debug::DebugFlags::KTraceCh) {
                            let mut text = [0; 200];
                            self.objects[FWLOG_PAYLOAD].read(payload + 0x10, &mut text)?;
                            let len = text.iter().position(|value| *value == 0).unwrap_or(text.len());
                            dev_info!(dev.as_ref(), "M3 firmware log[{}]: {}\n", thread,
                                core::str::from_utf8(&text[..len]).unwrap_or("<non-UTF8>"));
                        }
                    } else if channel == storage::EVENT {
                        if kind != 1 {
                            dev_err!(dev.as_ref(), "M3 firmware event {}: {:02x?}\n", kind, record);
                            return Err(EIO);
                        }
                        let flags = u32::from_le_bytes(record[4..8].try_into().map_err(|_| EIO)?);
                        self.completed_events = self.completed_events.saturating_add((flags & 7).count_ones().into());
                    } else if (channel == storage::TRACE && kind != 5)
                        || (channel == storage::STATS && kind == 0x16) {
                        dev_err!(dev.as_ref(), "M3 firmware channel {} record {}: {:02x?}\n", channel, kind, record);
                        return Err(EIO);
                    }
                    read = (read + 1) % capacity;
                }
                if read != before { self.objects[state].u32(at, read)?; }
            }
        }
        agx_memory::publish();
        Ok(())
    }

}

/// All extended initialization owners, retained as one construction transaction.
/// Nothing is published to RTKit by this constructor. A failed allocation drops
/// its mappings before backing storage and leaves no root visible to firmware.
pub(crate) struct ExtendedStorage {
    pub(crate) objects: KVec<Buffer>,
    pub(crate) iomaps: KVec<mmu::KernelMapping>,
    pub(crate) descriptors: KVec<u8>,
    pub(crate) maximum_khz: u32,
}

impl ExtendedStorage {
    pub(crate) fn new(
        dev: &driver::AsahiDrmDevice,
        uat: &mmu::Uat,
        pwr: &super::hw::PwrConfig,
        sram_uv: &[u32],
        bases: super::m3_mmio_26::RegisterBases,
    ) -> Result<Self> {
        use super::{m3_init_26 as plan, m3_mmio_26 as io};
        let mut objects = KVec::new();
        for index in 0..plan::COUNT {
            let entry = plan::allocation(index).map_err(|_| EINVAL)?;
            let alias = match index {
                plan::GLOBAL_POOL => Some((uat.kernel_lower_vm(), 0x70_0000_0000)),
                plan::GLOBAL_PARAMETERS => Some((uat.kernel_lower_vm(), 0x70_0000_8000)),
                _ => None,
            };
            let object = if plan::cached(index) {
                Buffer::at_prot(dev, uat.kernel_vm(), alias, Some(entry.address), entry.size,
                    prot::PROT_FW_PRIV_RW, prot::PROT_GPU_SHARED_RW)?
            } else {
                Buffer::at(dev, uat.kernel_vm(), alias, Some(entry.address), entry.size, entry.gpu_shared)?
            };
            objects.push(object, GFP_KERNEL)?;
        }
        let mut descriptors = KVec::new();
        descriptors.resize(io::SLOTS * io::RECORD_BYTES, 0, GFP_KERNEL)?;
        let mut iomaps = KVec::new();
        for slot in 0..io::SLOTS {
            let mut region = None;
            if let Some(area) = io::area(slot, bases).map_err(|_| EINVAL)? {
                let address = 0xffff_fc23_0000_0000 + slot as u64 * 0x10_0000;
                let access = if area.writable() { prot::PROT_FW_MMIO_RW } else { prot::PROT_FW_MMIO_RO };
                for (physical, bytes, offset) in area.banks() {
                    if bytes == 0 { continue; }
                    let mapping = uat.kernel_vm().map_io_m3(address + offset as u64,
                        physical.try_into()?, bytes, access)?;
                    iomaps.push(mapping, GFP_KERNEL)?;
                }
                region = Some(init::Region::new(address, area.mapped_bytes()).map_err(|_| EINVAL)?);
            }
            io::encode(slot, bases, region,
                &mut descriptors[slot * io::RECORD_BYTES..(slot + 1) * io::RECORD_BYTES])
                .map_err(|_| EINVAL)?;
        }
        // Runtime+0x18 references Fender SRAM, rather than anonymous DMA RAM.
        // Retain its normal shared mapping independently of its MMIO view.
        let scratch_physical = bases.gpu.checked_add(0xd6_0000).ok_or(EOVERFLOW)?;
        let scratch = uat.kernel_vm().map_io_m3(0xffff_fc22_0000_0000,
            scratch_physical.try_into()?, 0x8000, prot::PROT_FW_SHARED_RW)?;
        let scratch_region = init::Region::new(scratch.iova(), 0x8000).map_err(|_| EINVAL)?;
        iomaps.push(scratch, GFP_KERNEL)?;
        let mut regions = [init::Region::new(0xffff_fc21_0000_0000, 1).map_err(|_| EINVAL)?; plan::COUNT];
        for (index, object) in objects.iter().enumerate() {
            regions[index] = init::Region::new(object.va(), object.size()).map_err(|_| EINVAL)?;
        }
        let graph = plan::PointerGraph { owners: &regions, scratch: scratch_region,
            pool_alias: objects[plan::GLOBAL_POOL].gpu_va()?,
            parameter_alias: objects[plan::GLOBAL_PARAMETERS].gpu_va()? };
        let mut root = [0; 0xc8];
        let mut runtime = [0; plan::RUNTIME_SIZE];
        let mut control = KVec::new();
        control.resize(plan::allocation(CONTROL_REGION).map_err(|_| EINVAL)?.size, 0, GFP_KERNEL)?;
        graph.root(&mut root).map_err(|_| EINVAL)?;
        graph.runtime(&mut runtime).map_err(|_| EINVAL)?;
        graph.firmware_control(&mut control).map_err(|_| EINVAL)?;
        let images = super::m3_power_26::PowerImages::new(pwr, sram_uv,
            objects[plan::SENSOR_VALUE].va(), objects[plan::RETENTION_STATE].va(),
            agx_memory::TIMESTAMP_RANGE.start, &descriptors)?;
        objects[storage::ROOT].write(0, &root)?;
        objects[RUNTIME_POINTERS].write(0, &runtime)?;
        objects[CONTROL_REGION].write(0, &control)?;
        objects[HARDWARE_DATA].write(0, &images.hardware)?;
        objects[plan::PERFORMANCE_CONTROL].write(0, &images.performance)?;
        objects[GLOBALS_POWER].write(0, &images.power)?;
        objects[GLOBALS].write(0, &images.globals)?;
        // Fragment scheduler entries start unassigned; these are input sentinels.
        objects[UNKNOWN_C1].u32(0xc18, u32::MAX)?;
        objects[UNKNOWN_C1].u32(0xc30, u32::MAX)?;
        agx_memory::publish();
        Ok(Self { objects, iomaps, descriptors, maximum_khz: images.maximum_khz })
    }
}
