// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Owned M3 register apertures. The platform
//! power domain owns TVM/PMGR and stays on across probe/remove. Firmware and
//! queues must be stopped before this object is released.

use kernel::{
    bindings, c_str,
    device::Core,
    devres::Devres,
    error::to_result,
    io::{
        mem::IoMem,
        Io,
        IoBase,
    },
    platform,
    prelude::*,
    sync::aref::ARef,
    time::{delay::fsleep, Delta, Instant, Monotonic},
};

use crate::m3::m3_firmware::Firmware;
use crate::m3::m3_soc::Soc;

const ASC_CPU_CONTROL: usize = 0x44;
const ASC_CPU_RUN: u32 = 1 << 4;

pub(crate) struct Device {
    dev: ARef<platform::Device>,
    asc: Devres<IoMem<'static, 0x4000>>,
    sgx: Devres<IoMem<'static>>,
    firmware: Firmware,
    core_mask: u64,
    soc: &'static Soc,
    power_vote: bool,
}

impl Device {
    /// `experiment` is the T8122 start experiment's values, on an armed T8122 only: it decides
    /// the SGX setup write, which the T8122 table lacks.
    pub(crate) fn new(pdev: &platform::Device<Core<'_>>, firmware: Firmware, soc: &'static Soc,
        experiment: Option<&crate::m3::t8122_start::Experiment>) -> Result<Self> {
        // Map ASC before taking a vote. The larger SGX aperture contains
        // this control window and the separate mailbox provider, so it is
        // mapped without a conflicting claim over those child resources.
        // Only the 16 KiB control window is used. Some device trees describe
        // the ASC block as a larger range that also covers the mailbox, which
        // the mailbox driver has already claimed, so do not claim the range.
        let asc = pdev.io_request_by_name(c_str!("asc"))
            .ok_or(EINVAL)?
            .iomap_sized::<0x4000>()?
            .into_devres()?;
        dev_info!(pdev.as_ref(), "M3: ASC aperture mapped\n");
        let sgx = pdev.io_request_by_name(c_str!("sgx"))
            .ok_or(EINVAL)?
            .iomap()?
            .into_devres()?;
        dev_info!(pdev.as_ref(), "M3: SGX aperture mapped\n");
        let pmp = crate::m3::m3_board::has_pmp_link(pdev);
        let t8122 = crate::m3::t8122_start::is_t8122(soc);
        if pmp && !t8122 {
            dev_err!(pdev.as_ref(), "M3: optional apple,pmp GPU link is unsupported\n");
            return Err(ENOTSUPP);
        }
        if t8122 && !pmp {
            dev_err!(pdev.as_ref(), "M3 G15G: current14 GPU requires its PMP supplier link\n");
            return Err(ENODEV);
        }
        if pmp {
            // SAFETY: the consumer is live; the managed link pins its declared PMP supplier.
            to_result(unsafe { bindings::apple_pmp_link_device(pdev.as_ref().as_raw()) })?;
        } else {
            dev_info!(pdev.as_ref(), "M3: no apple,pmp link; GPU power is left to its power domain\n");
        }
        if asc.access(pdev.as_ref())?.as_view().read32(ASC_CPU_CONTROL) & ASC_CPU_RUN != 0 {
            dev_err!(
                pdev.as_ref(),
                "M3 {}: ASC already running; refusing to take ownership\n",
                soc.gpu_name
            );
            return Err(EBUSY);
        }
        let mut device = Self {
            dev: pdev.into(),
            asc,
            sgx,
            firmware,
            core_mask: 0,
            soc,
            power_vote: false,
        };

        if pmp {
            // SAFETY: the linked current14 T8122 PMP owns AGX logical device 5. The bridge
            // waits for firmware readiness and the power-command acknowledgement.
            // An unacknowledged request may still have enabled AGX. Keep ownership so a
            // failed acquisition releases it only after the ASC-stopped check in Drop.
            device.power_vote = true;
            to_result(unsafe { bindings::apple_pmp_set_device_power(0x0f, 5, 1) })?;
            dev_info!(pdev.as_ref(), "M3 G15G: PMP GPU device 5 power acknowledged\n");
        }

        let registers = device.sgx.access(pdev.as_ref())?.as_view();
        let version = registers.try_read32(0xd04000)?;
        let counts = registers.try_read32(0xd04010)?;
        // The core-enable words: the second only on a SoC with more than 32 core slots.
        let mut core_mask = u64::from(registers.try_read32(0xe01500)?);
        if crate::m3::m3_board::core_slots(soc) > 32 {
            core_mask |= u64::from(registers.try_read32(0xe01504)?) << 32;
        }
        dev_info!(pdev.as_ref(), "M3 {}: power acknowledged, version={:#010x} counts={:#010x} core-mask={:#x}, firmware={}\n",
            soc.gpu_name, version, counts, core_mask, device.firmware.version());
        // These words identify the qualified configuration of this SoC. The
        // fused core mask varies with SKU; absent cores must never be enabled.
        if version & soc.id.version_mask != soc.id.version
            || counts & soc.id.counts_mask != soc.id.counts
            || !crate::m3::m3_board::core_mask_valid(soc, core_mask)
        {
            return Err(ENODEV);
        }
        let setup = match experiment {
            Some(e) => {
                match e.sgx_setup() {
                    Some((offset, value)) => dev_warn!(pdev.as_ref(),
                        "M3 G15G start: SGX setup write: SGX+{:#x} = {:#x} (asahi.t8122_sgx_setup=t6030)\n", offset, value),
                    None => dev_info!(pdev.as_ref(),
                        "M3 G15G start: no SGX setup write (asahi.t8122_sgx_setup=none)\n"),
                }
                e.sgx_setup()
            }
            None => Some(soc.sgx_setup.ok_or(ENODEV)?),
        };
        if let Some((setup_offset, setup_value)) = setup {
            registers.try_write32(setup_value, setup_offset)?;
        }
        device.core_mask = core_mask;
        Ok(device)
    }

    pub(crate) fn core_mask(&self) -> u64 { self.core_mask }

    /// The SoC table this device was admitted with.
    pub(crate) fn soc(&self) -> &'static Soc { self.soc }

    pub(crate) fn check_idle(&self)->Result {
        self.check_idle_parts(true,true)
    }
    /// [`Self::check_idle`] with its engine-busy (`busy`) and fault-bank (`faults`) register
    /// accesses selectable (`asahi.m3_retire_mmio`); with neither it touches no GPU register.
    pub(crate) fn check_idle_parts(&self,busy:bool,faults:bool)->Result {
        if !busy && !faults {return Ok(());}
        let sgx_guard=self.sgx.try_access().ok_or(ENODEV)?;
        let sgx=sgx_guard.as_view();
        if busy && (sgx.try_read64(0xc020)? | sgx.try_read64(0xc120)?) & 1 != 0 {
            return Err(EBUSY);
        }
        if !faults {return Ok(());}
        let selector=sgx.try_read64(0xd800)?;
        let mut fault=0;
        // One fault bank (selected through 0xd800) per GPU cluster: banks 0 and 1 on the
        // two-cluster T6030, bank 0 alone on the single-cluster T8122. Never select a bank past
        // the SoC's clusters.
        for bank in 0..u64::from(self.soc.clusters) {
            sgx.try_write64(bank,0xd800)?;fault|=sgx.try_read64(0xd8c0)?;
        }
        sgx.try_write64(selector,0xd800)?;
        if fault!=0 {return Err(EIO);} Ok(())
    }

    /// The raw performance-state register (SGX+0xe01000).
    pub(crate) fn pstate_register(&self) -> Result<u32> {
        self.sgx.try_access().ok_or(ENODEV)?.as_view().try_read32(0xe01000)
    }

    /// Read-only snapshot. Do not select or acknowledge fault banks here;
    /// the firmware owns those registers until recovery is ported.
    pub(crate) fn log_engine_state(&self, vm: &crate::mmu::Vm) -> Result {
        if self.firmware.profile() == super::m3_profile::Profile::Extended {
            // Adaptive clock gating can leave individual engine banks inaccessible
            // even while the GPU performance-state register says powered up.
            dev_info!(self.dev.as_ref(), "M3: adaptive firmware fault evidence uses retained metadata; engine banks may be gated\n");
            return Ok(());
        }
        let sgx_guard = self.sgx.try_access().ok_or(ENODEV)?;
        let sgx = sgx_guard.as_view();
        let pstate = sgx.try_read32(0xe01000)? & 0xf;
        if pstate == 0 {
            dev_info!(self.dev.as_ref(), "M3 {}: GPU is powered down; skipping engine/MMU register snapshot\n", self.soc.gpu_name);
            return Ok(());
        }
        dev_info!(self.dev.as_ref(), "M3 {}: reading engine/MMU snapshot at pstate={}\n", self.soc.gpu_name, pstate);
        let selector=sgx.try_read64(0xd800)?;
        for bank in [0,1,0x100,0x40000,0x40001,0x80000,0xc0000] {
            sgx.try_write64(bank,0xd800)?;
            let address=sgx.try_read64(0xd8c8)?;
            let fault=sgx.try_read64(0xd8c0)?;
            dev_info!(self.dev.as_ref(),"M3 bank {} fault={:#x} address_word={:#x} address={:#x}\n",bank,fault,address,address<<6);
            if fault&1!=0 {
                dev_info!(self.dev.as_ref(),"M3 fault mapping bank={} address={:#x} translation={:?}\n",bank,address<<6,vm.translate_iova(address<<6));
            }
        }
        sgx.try_write64(selector,0xd800)?;
        // Read-only per-core service/debug snapshot, using the same
        // core-selector protocol as the idle check.
        let service=sgx.try_read32(0xa010)?;
        let debug=sgx.try_read32(0xa000)?;
        let mut cores=u64::from(sgx.try_read32(0xe01500)?);
        if crate::m3::m3_board::core_slots(self.soc) > 32 {
            cores |= u64::from(sgx.try_read32(0xe01504)?) << 32;
        }
        for core in 0..crate::m3::m3_board::core_slots(self.soc).min(64) {
            if cores & (1u64<<core)==0 {continue;}
            sgx.try_write32(core,0xa010)?;
            sgx.try_write32(core,0xa000)?;
            dev_info!(self.dev.as_ref(),"M3_USC core={} VDM={:#x}/{:#x} PDM={:#x}/{:#x} CDM={:#x}/{:#x}\n",
                core,sgx.try_read64(0xa088)?,sgx.try_read64(0xa090)?,
                sgx.try_read64(0xa068)?,sgx.try_read64(0xa070)?,
                sgx.try_read64(0xa0a8)?,sgx.try_read64(0xa0b0)?);
        }
        sgx.try_write32(service,0xa010)?;
        sgx.try_write32(debug,0xa000)?;
        let mut mmu = [0u32; 13];
        for (i, word) in mmu.iter_mut().enumerate() { *word = sgx.try_read32(0xd08000 + i * 4)?; }
        dev_info!(self.dev.as_ref(), "M3 {}: GPU MMU configuration={:x?}\n", self.soc.gpu_name, mmu);
        for offset in [0xc000,0xc008,0xc010,0xc018,0xc020,0xc028,0xc030,0xc038,0xc060,0xc068,0xc070,0xc078,0xc088,0xc090,0xc098,0xc0a0,0xc0a8,0xc0b0, 0xc040, 0xc048, 0xc050, 0xc058, 0xc080, 0xc120, 0xc140, 0xc148,
            0xd8c0, 0xd8c8] {
            dev_info!(self.dev.as_ref(), "M3 {}: SGX +{:#x}={:#018x}\n", self.soc.gpu_name, offset, sgx.try_read64(offset)?);
        }
        Ok(())

    }

    pub(crate) fn firmware(&self) -> &Firmware { &self.firmware }

    pub(crate) fn require_stopped(&self, pdev: &platform::Device<Core<'_>>) -> Result {
        if self.asc.access(pdev.as_ref())?.as_view().read32(ASC_CPU_CONTROL) & ASC_CPU_RUN != 0 {
            return Err(EBUSY);
        }
        Ok(())
    }

    pub(crate) fn start_asc(&self, pdev: &platform::Device<Core<'_>>) -> Result {
        self.require_stopped(pdev)?;
        let asc = self.asc.access(pdev.as_ref())?.as_view();
        asc.write32(asc.read32(ASC_CPU_CONTROL) | ASC_CPU_RUN, ASC_CPU_CONTROL);
        Ok(())
    }

    pub(crate) fn stop_asc(&self) -> Result {
        let asc_guard = self.asc.try_access().ok_or(ENODEV)?;
        let asc = asc_guard.as_view();
        asc.write32(asc.read32(ASC_CPU_CONTROL) & !ASC_CPU_RUN, ASC_CPU_CONTROL);
        let start = Instant::<Monotonic>::now();
        while asc.read32(ASC_CPU_CONTROL) & ASC_CPU_RUN != 0 {
            if start.elapsed() >= Delta::from_millis(100) { return Err(ETIMEDOUT); }
            fsleep(Delta::from_micros(10));
        }
        Ok(())
    }

}

impl Drop for Device {
    fn drop(&mut self) {
        if !self.power_vote {
            return;
        }
        // ASC and firmware queues must be stopped before the inner GPU vote is relinquished.
        match self.asc.try_access() {
            Some(asc) if asc.as_view().read32(ASC_CPU_CONTROL) & ASC_CPU_RUN == 0 => {}
            _ => {
                dev_err!(self.dev.as_ref(), "M3 G15G: cannot prove ASC stopped; retaining PMP vote\n");
                return;
            }
        }
        // SAFETY: the managed consumer link keeps the PMP bound through device removal.
        if let Err(error) = to_result(unsafe { bindings::apple_pmp_set_device_power(0x0f, 5, 0) }) {
            dev_err!(self.dev.as_ref(), "M3 G15G: PMP GPU power release failed: {:?}\n", error);
        }
    }
}
