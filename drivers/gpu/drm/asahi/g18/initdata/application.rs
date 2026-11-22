// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::*;
use crate::g18::uapi::application_runtime::Frame;
use crate::g18::runtime::graphics_job::application::ApplicationJob;

impl G18FirmwareMemory {
    pub(crate) fn application_aliases(&mut self, vm: &mmu::Vm) -> Result<KVec<mmu::KernelMapping>> {
        self.uma_pool.application_aliases(&self.uat,vm)
    }

    pub(crate) fn audit_application(&mut self, dev: &AsahiDrmDevice, job: &mut ApplicationJob) -> Result {
        let frame=job.frame();let context=u32::from(job.job.context_id());
        let pb=job.job.parameter_buffer().verify_resident(&mut self.pbdesc,context,frame.generation)?;
        let uma=self.uma_pool.verify_application_descriptor(&mut self.uma,j873_sma_device_words(2,6)?,frame.generation==1)?;
        // The page walks prove this job's mappings in the VMs they read. A VM's
        // mapping epoch advances on every unmap and remap (an addition leaves
        // every proven range mapped), so with the same client VM and the same
        // epochs the last walk still holds and is not repeated. Everything else
        // is checked on every frame.
        let key=job.walk_key(&self.uat);
        let walk=key.is_none() || key!=job.walked;
        let pages=if walk {self.uma_pool.audit_render_mappings(&self.uat,job.job.vm())?} else {0};
        job.audit(dev,&self.uat,&pb,&uma,pages,walk)?;
        if walk {job.walked=key;}
        if frame.generation==1 {
            let state=self.uma_pool.render_state()?;let mut row=[0u8;128];
            for (out,word) in row.chunks_exact_mut(8).zip(uma) {out.copy_from_slice(&word.to_le_bytes());}
            for (name,bytes) in [("uma-state",state.as_slice()),("uma-row",row.as_slice())] {
                dev_info!(dev.as_ref(),"G18_RENDER_DIGEST name={} fnv={:#018x}\n",name,crate::g18::fw::graphics_payload::fnv64(bytes));
            }
        }
        self.completion_masks=[0;2];Ok(())
    }

    pub(crate) fn publish_application_outer(&mut self, records: &[[u8;24];2], frame: Frame) -> Result<u32> {
        for (index,record) in records.iter().enumerate() {
            let qid=if index==0 {10} else {9};
            if record[22]!=qid || record[23]!=1 || record[16..20]!=2u32.to_le_bytes()
                || record[20..22]!=(frame.inner_after() as u16).to_le_bytes()
                || record[8..16]!=(crate::g18::address_map::GRAPHICS_QUEUE_HIGH+if qid==9 {0} else {0x100}).to_le_bytes() {return Err(EINVAL);}
        }
        self.publish_application_records(records)
    }

    /// Graphics and compute share this outer ring but have independent inner
    /// sequences. Only publish from a fully retired observed outer position.
    pub(crate) fn publish_application_records(&mut self, records: &[[u8;24]]) -> Result<u32> {
        if records.is_empty() || records.len()>2 || self.control_counters()?!=self.control_retired() {
            return Err(EINVAL);
        }
        let counters=self.compute_outer_counters()?;let before=counters[0];
        if before>=256 || counters!=[before;3] {return Err(EBUSY);}
        for record in records {
            let address=u64::from_le_bytes(record[8..16].try_into().map_err(|_|EINVAL)?);
            let expected=match record[22] {
                9=>crate::g18::address_map::GRAPHICS_QUEUE_HIGH,
                10=>crate::g18::address_map::GRAPHICS_QUEUE_HIGH+0x100,
                11=>crate::g18::address_map::COMPUTE_GRAPH,
                _=>return Err(EINVAL),
            };
            if address!=expected || record[..8]!=[0;8] || record[23]!=1
                || record[16..20]!=2u32.to_le_bytes()
                || u16::from_le_bytes(record[20..22].try_into().map_err(|_|EINVAL)?)>=256 {
                return Err(EINVAL);
            }
        }
        let after=(before+records.len() as u32)%256;
        self.shared.with_bytes_mut(|out| {
            for (index,record) in records.iter().enumerate() {
                let start=self.layouts.work_rings[8]+((before as usize+index)%256)*24;
                out.get_mut(start..start+24).ok_or(EINVAL)?.copy_from_slice(record);
            } Ok(())
        })?;
        if self.compute_outer_counters()?!=[before;3] {return Err(EBUSY);}
        fence(Ordering::SeqCst);
        self.private.write_u32(PRIVATE_BASE+(init::G18G_26A428_PRIVATE_STATE_GRIDS[0]
            +init::WORK_STATE_GRID_OFFSETS[8]+0x20) as u64,after)?;
        fence(Ordering::SeqCst);Ok(after)
    }

    pub(crate) fn publish_application_compute_outer(&mut self, inner: u32) -> Result<u32> {
        if inner>=256 {return Err(EINVAL);}
        let mut record=[0;24];
        record[8..16].copy_from_slice(&crate::g18::address_map::COMPUTE_GRAPH.to_le_bytes());
        record[16..20].copy_from_slice(&2u32.to_le_bytes());
        record[20..22].copy_from_slice(&(inner as u16).to_le_bytes());
        record[22]=11;record[23]=1;
        self.publish_application_records(&[record])
    }
}
