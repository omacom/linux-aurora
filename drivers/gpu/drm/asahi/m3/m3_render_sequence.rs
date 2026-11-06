// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::{m3_init_layout::{Error, Region}, m3_queue_layout::FirmwareVa,
    m3_pool_layout::GpuRegion};
pub(crate) const STORAGE: usize = 0x1000;
pub(crate) const USER_TIMESTAMP_POINTER: usize = 0x24;
pub(crate) mod ta {
    pub(crate) const TIMESTAMP_START: usize = 0x1cc;
    pub(crate) const TIMESTAMP_END: usize = 0x20c;
    pub(crate) const FINALIZE: usize = 0x248;
    pub(crate) const LENGTH: usize = 0x2d8;
    pub(crate) const POOL: usize = 0x18c;
    pub(crate) const COMMAND_TAIL: usize = 0x8a8;
    pub(crate) const USER_PAIR: usize = COMMAND_TAIL + 0x18;
    pub(crate) const REGISTERS: usize = 0x40;
    pub(crate) const STRUCT3: usize = 0x784;
    pub(crate) const SAMPLER_ARRAY: usize = 0x84c;
    pub(crate) const SCRATCH_STATE: usize = 0x890;
    pub(crate) const STATS: usize = 0x1884;
    pub(crate) const UUID: u32 = 0x7a0000;
}
pub(crate) mod fragment {
    pub(crate) const TIMESTAMP_START: usize = 0x1dc;
    pub(crate) const TIMESTAMP_END: usize = 0x21c;
    pub(crate) const FINALIZE: usize = 0x258;
    pub(crate) const LENGTH: usize = 0x310;
    pub(crate) const POOL: usize = 0x1a4;
    pub(crate) const COMMAND_TAIL: usize = 0xbf0;
    pub(crate) const USER_PAIR: usize = COMMAND_TAIL + 0x18;
    pub(crate) const REGISTERS: usize = 0x80;
    pub(crate) const BUSY: usize = 0xb70;
    pub(crate) const STRUCT6: usize = 0xb74;
    // Start.struct7 and Finalize.job_meta refer to this prefix, not +0xbac.
    pub(crate) const STRUCT7: usize = 0xba0;
    pub(crate) const SCRATCH_FLAG: usize = 0xa58;
    pub(crate) const SCRATCH_STATE: usize = 0xbd8;
    pub(crate) const STATS: usize = 0x24c8;
    pub(crate) const UUID: u32 = 0x3d0000;
}
fn word(out: &mut [u8], off: usize, value: u32) { out[off..off+4].copy_from_slice(&value.to_le_bytes()); }
fn pointer(out: &mut [u8], off: usize, value: u64) { out[off..off+8].copy_from_slice(&value.to_le_bytes()); }
/// The stage's work command owns its sequence-length field.
pub(crate) fn command_length_offset(fragment:bool)->usize {
    if fragment {super::m3_command_layout::FRAGMENT_SEQUENCE_LENGTH}
    else {super::m3_command_layout::TILER_SEQUENCE_LENGTH}
}
/// Omit measurement packets while retaining engine-idle, finalization and End.
/// Finalize's relative restart displacement follows its compacted location.
fn without_timestamps(out:&mut[u8],start:usize,finalize:usize,length:usize,restart:usize)->usize {
    let next=start+4;
    let removed=finalize-next;
    out.copy_within(finalize..length,next);
    word(out,start,1);
    word(out,next+restart,(-(next as i32)) as u32);
    let length=length-removed;
    out[length..STORAGE].fill(0);
    length
}
/// Resources common to both stages; command, sequence and scene are per-pass.
/// Stats and notifier belong to Config / Render for the entire submission.
#[derive(Clone, Copy)]
pub(crate) struct Owners {
    pub(crate) command: Region,
    pub(crate) sequence: Region,
    pub(crate) stats: Region,
    pub(crate) notifier: Region,
    pub(crate) queue: FirmwareVa,
    pub(crate) scene: FirmwareVa,
    pub(crate) manager: FirmwareVa,
    pub(crate) pool: FirmwareVa,
    pub(crate) fw_stamp: FirmwareVa,
    pub(crate) context: u32,
    pub(crate) stamp: u32,
}
struct Timestamps { time:u64, pair:u64, end:u64, complete:u64 }
impl Timestamps {
    fn new(command:Region, tail:usize)->Result<Self,Error> {
        Ok(Self {time:command.at(tail,8)?,pair:command.at(tail+8,16)?,
            end:command.at(tail+16,8)?,complete:command.at(tail+0x58,8)?})
    }
    fn encode(&self,out:&mut[u8],start:usize,end:usize,queue:FirmwareVa,uuid:u32) {
        for (offset,opcode,selected) in [(start,0x80000003,self.pair),(end,3,self.end)] {
            word(out,offset,opcode);
            for (field,value) in [(4,self.time),(0xc,self.pair),(0x14,selected),
                (0x1c,queue.get()),(0x2c,self.complete)] {pointer(out,offset+field,value);}
            word(out,offset+0x34,uuid);
        }
        word(out,end-4,1); // G15 WaitForIdle2, between the timestamp packets
    }
}
pub(crate) struct Tiler {pub(crate) owners:Owners,pub(crate) scratch:FirmwareVa}
impl Tiler {
    /// Validate every referenced subobject before changing output bytes.
    pub(crate) fn encode(&self,out:&mut[u8])->Result<(),Error> {
        self.encode_timestamps(out,true).map(|_|())
    }
    pub(crate) fn encode_timestamps(&self,out:&mut[u8],timed:bool)->Result<usize,Error> {
        use ta::*;
        let o=self.owners;
        if out.len()<STORAGE {return Err(Error::Size);}
        if o.context>=64 {return Err(Error::Address);}
        let registers=o.command.at(REGISTERS,128*12)?;
        let state=o.command.at(STRUCT3,0x10c)?;
        let sampler=o.command.at(SAMPLER_ARRAY,8)?;
        let scratch_state=o.command.at(SCRATCH_STATE,0x18)?;
        let timestamps=Timestamps::new(o.command,COMMAND_TAIL)?;
        let request=o.command.at(COMMAND_TAIL+0x50,8)?;
        let stats=o.stats.at(STATS,8)?;
        let notifier=o.notifier.at(0xa8,8)?;
        o.sequence.at(0,STORAGE)?;
        let pool_field=o.sequence.at(POOL,8)?;
        let bytes=&mut out[..STORAGE];bytes.fill(0);
        word(bytes,0,5); // StartTA
        for (field,value) in [(0x14,registers),(0x1c,o.manager.get()),(0x24,o.scene.get()),
            (0x2c,stats),(0x34,o.queue.get()),(0x3c,sampler),(0x64,state),
            (0x6c,scratch_state),(POOL,o.pool.get()),(0x1a4,self.scratch.get()),
            (0x1bc,notifier)] {pointer(bytes,field,value);}
        word(bytes,0x44,o.context);
        pointer(bytes,0x78,u64::from(UUID)<<32); // retained unk_178_0; StartTA.uuid stays zero
        pointer(bytes,0x1b4,1); // g15_unk_28
        timestamps.encode(bytes,TIMESTAMP_START,TIMESTAMP_END,o.queue,UUID);
        word(bytes,FINALIZE,6); // FinalizeTA
        for (field,value) in [(4,o.scene.get()),(0xc,o.manager.get()),(0x14,stats),
            (0x1c,o.queue.get()),(0x24,sampler),(0x34,state),(0x44,o.fw_stamp.get()),
            (0x74,pool_field),(0x84,request)] {pointer(bytes,FINALIZE+field,value);}
        word(bytes,FINALIZE+0x2c,o.context);
        word(bytes,FINALIZE+0x40,UUID);
        word(bytes,FINALIZE+0x4c,o.stamp);
        word(bytes,FINALIZE+0x7c,(-(FINALIZE as i32)) as u32);
        word(bytes,LENGTH-4,0x40000002); // End, qualified completion flag
        Ok(if timed {LENGTH} else {without_timestamps(bytes,TIMESTAMP_START,FINALIZE,LENGTH,0x7c)})
    }
}
pub(crate) struct Fragment {
    pub(crate) owners:Owners,
    /// Unlike the tiler scratch reference, this is in the client GPU space.
    pub(crate) scratch:GpuRegion,
    pub(crate) queue_command_count:u32,
}
impl Fragment {
    pub(crate) fn encode(&self,out:&mut[u8])->Result<(),Error> {
        self.encode_timestamps(out,true).map(|_|())
    }
    pub(crate) fn encode_timestamps(&self,out:&mut[u8],timed:bool)->Result<usize,Error> {
        use fragment::*;
        let o=self.owners;
        if out.len()<STORAGE {return Err(Error::Size);}
        if o.context>=64 || self.scratch.size()<0x10000 {return Err(Error::Address);}
        let command=o.command.at(0,0xc73)?;
        let registers=o.command.at(REGISTERS,128*12)?;
        let busy=o.command.at(BUSY,4)?;
        let struct6=o.command.at(STRUCT6,0x2c)?;
        let struct7=o.command.at(STRUCT7,0x34)?;
        let flag=o.command.at(SCRATCH_FLAG,4)?;
        let state=o.command.at(SCRATCH_STATE,0x18)?;
        let timestamps=Timestamps::new(o.command,COMMAND_TAIL)?;
        let request=o.command.at(COMMAND_TAIL+0x50,8)?;
        let stats=o.stats.at(STATS,8)?;
        let notifier=o.notifier.at(0xa8,8)?;
        o.sequence.at(0,STORAGE)?;
        let pool_field=o.sequence.at(POOL,8)?;
        let bytes=&mut out[..STORAGE];bytes.fill(0);
        word(bytes,0,7); // Start3D
        for (field,value) in [(0x14,registers),(0x1c,o.scene.get()),(0x24,stats),
            (0x2c,busy),(0x34,struct6),(0x3c,struct7),(0x44,o.queue.get()),
            (0x4c,command),(0x74,flag),(0x7c,state),(POOL,o.pool.get()),
            (0x1bc,self.scratch.base()),(0x1d4,notifier)] {pointer(bytes,field,value);}
        word(bytes,0x54,o.context);
        word(bytes,0x58,1); // unk_50
        pointer(bytes,0x68,u64::from(self.queue_command_count));
        word(bytes,0x98,UUID);
        timestamps.encode(bytes,TIMESTAMP_START,TIMESTAMP_END,o.queue,UUID);
        word(bytes,FINALIZE,8); // Finalize3D
        word(bytes,FINALIZE+4,UUID);
        for (field,value) in [(0xc,o.fw_stamp.get()),(0x1c,o.scene.get()),
            (0x24,o.manager.get()),(0x34,stats),(0x3c,struct7),(0x44,busy),
            (0x4c,o.queue.get()),(0x54,command),(0x64,flag),(0x9c,pool_field),
            (0xac,request)] {pointer(bytes,FINALIZE+field,value);}
        word(bytes,FINALIZE+0x14,o.stamp);
        pointer(bytes,FINALIZE+0x2c,1); // unk_2c
        pointer(bytes,FINALIZE+0x5c,u64::from(o.context)); // firmware u64, unlike Start3D
        word(bytes,FINALIZE+0xa4,(-(FINALIZE as i32)) as u32);
        word(bytes,FINALIZE+0xa8,1); // unk_98
        word(bytes,LENGTH-4,0x40000002);
        Ok(if timed {LENGTH} else {without_timestamps(bytes,TIMESTAMP_START,FINALIZE,LENGTH,0xa4)})
    }
}

impl Owners {
    pub(crate) fn encode_extended(&self, fragment:bool,
        auxiliary:super::m3_compute_layout::GpuVa, serial:u64, output:&mut[u8]) -> Result<usize,Error> {
        if self.context>=64 || serial==0 || self.stamp==0 {return Err(Error::Address);}
        let command=self.command.at(0,if fragment {0xca0}else{0x940})?;
        let sequence=self.sequence.at(0,STORAGE)?;
        let activity=self.stats.at(0,8)?;
        let scene_tail=self.scene.get().checked_add(0x54).ok_or(Error::Address)?;
        let (clock,pair,aux_time,elapsed)=if fragment {(0xc08,0xc10,0xc70,0xb84)}else{(0x8c0,0x8c8,0x928,0x83c)};
        let first=if fragment {0x1e4}else{0x1cc};
        let finalize=if fragment {0x280}else{0x268};
        let pool=if fragment {0x1b0}else{0x190};
        let length=if fragment {0x370}else{0x300};
        let uuid=if fragment {fragment::UUID}else{ta::UUID};
        let bytes=output.get_mut(..STORAGE).ok_or(Error::Size)?;
        bytes.fill(0);
        word(bytes,0,if fragment {7}else{5});
        pointer(bytes,0x14,command+if fragment {0x80}else{0x40});
        pointer(bytes,pool,self.pool.get());
        pointer(bytes,pool+0x18,auxiliary.get());
        pointer(bytes,pool+0x28,serial);
        word(bytes,pool+0x30,u32::from(fragment));
        if fragment {
            pointer(bytes,0x1c,self.scene.get());
            pointer(bytes,0x24,activity);
            pointer(bytes,0x2c,command+0xb70);
            pointer(bytes,0x34,scene_tail);
            pointer(bytes,0x3c,command+0xbb0);
            pointer(bytes,0x44,command+0xbb4);
            pointer(bytes,0x4c,self.queue.get());
            pointer(bytes,0x54,command);
            word(bytes,0x5c,self.context);
            word(bytes,0x60,1);
            pointer(bytes,0x7c,command+0xa58);
            pointer(bytes,0x84,command+0xbf0);
            word(bytes,0xa0,uuid);
        } else {
            pointer(bytes,0x1c,self.manager.get());
            pointer(bytes,0x24,self.scene.get());
            pointer(bytes,0x2c,activity);
            pointer(bytes,0x34,self.queue.get());
            pointer(bytes,0x3c,command+0x85c);
            word(bytes,0x44,self.context);
            word(bytes,0x48,1);
            pointer(bytes,0x64,command+0x784);
            pointer(bytes,0x6c,command+0x8a8);
            word(bytes,0x7c,uuid);
        }
        for (at,opcode,selected) in [(first,0x8000_0003,pair),(first+0x50,3,pair+8)] {
            word(bytes,at,opcode);
            pointer(bytes,at+4,command+clock);
            pointer(bytes,at+0xc,command+pair);
            pointer(bytes,at+0x14,command+selected);
            pointer(bytes,at+0x1c,self.queue.get());
            pointer(bytes,at+0x2c,command+aux_time);
            pointer(bytes,at+0x34,command+elapsed);
            pointer(bytes,at+0x44,uuid.into());
        }
        word(bytes,first+0x4c,1);
        word(bytes,finalize,if fragment {8}else{6});
        if fragment {
            word(bytes,finalize+4,uuid);
            pointer(bytes,finalize+0xc,self.fw_stamp.get());
            word(bytes,finalize+0x14,self.stamp);
            pointer(bytes,finalize+0x1c,self.scene.get());
            pointer(bytes,finalize+0x24,self.manager.get());
            word(bytes,finalize+0x2c,1);
            pointer(bytes,finalize+0x30,activity);
            pointer(bytes,finalize+0x38,command+0xbb0);
            pointer(bytes,finalize+0x40,command+0xbb4);
            pointer(bytes,finalize+0x48,command+0xb70);
            pointer(bytes,finalize+0x50,self.queue.get());
            pointer(bytes,finalize+0x58,command);
            word(bytes,finalize+0x60,self.context);
            pointer(bytes,finalize+0x64,command+0xa58);
            pointer(bytes,finalize+0x98,sequence+pool as u64);
            word(bytes,finalize+0xa0,(-(finalize as i32))as u32);
            pointer(bytes,finalize+0xa5,command+0xc60);
            word(bytes,finalize+0xad,1);
            bytes[finalize+0xb1]=1;
            // J613 terminates this record after the 16-bit event field.
            bytes[finalize+0xb2..finalize+0xb4].copy_from_slice(&1u16.to_le_bytes());
            word(bytes,finalize+0xb4,0x4000_0002);
        } else {
            pointer(bytes,finalize+4,self.scene.get());
            pointer(bytes,finalize+0xc,self.manager.get());
            pointer(bytes,finalize+0x14,activity);
            pointer(bytes,finalize+0x1c,self.queue.get());
            pointer(bytes,finalize+0x24,command+0x85c);
            word(bytes,finalize+0x2c,self.context);
            pointer(bytes,finalize+0x34,command+0x784);
            word(bytes,finalize+0x40,uuid);
            pointer(bytes,finalize+0x48,self.fw_stamp.get());
            word(bytes,finalize+0x50,self.stamp);
            pointer(bytes,finalize+0x78,sequence+pool as u64);
            word(bytes,finalize+0x80,(-(finalize as i32))as u32);
            pointer(bytes,finalize+0x85,command+0x918);
            bytes[finalize+0x8d]=1;
            word(bytes,finalize+0x94,0x4000_0002);
        }
        Ok(length)
    }
}

#[cfg(test)]
mod native_tests {
    use super::*;
    #[test] fn j613_fragment_ends_before_the_m4_tail_and_keeps_relative_restart() {
        let fw=|index:u64|FirmwareVa::new(0xffff_fc25_2000_0000+index*0x4000).unwrap();
        let o=Owners {command:Region::new(fw(0).get(),0xca0).unwrap(),sequence:Region::new(fw(1).get(),STORAGE).unwrap(),
            stats:Region::new(fw(2).get(),0x1000).unwrap(),notifier:Region::new(fw(3).get(),0x200).unwrap(),
            queue:fw(4),scene:fw(5),manager:fw(6),pool:fw(7),fw_stamp:fw(8),context:3,stamp:0x3d000100};
        let mut output=[0xa5;STORAGE+8];
        let length=o.encode_extended(true,super::super::m3_compute_layout::GpuVa::new(0x72_3000_0200).unwrap(),1,&mut output).unwrap();
        assert_eq!(length,0x370);
        let word=|at|u32::from_le_bytes(output[at..at+4].try_into().unwrap());
        assert_eq!(word(0x1e4),0x8000_0003);
        assert_eq!(word(0x230),1);
        assert_eq!(word(0x234),3);
        assert_eq!(word(0x280),8);
        assert_eq!(word(0x320),(-0x280i32)as u32);
        assert_eq!(word(0x334),0x4000_0002);
        assert_eq!(&output[STORAGE..],&[0xa5;8]);
    }
}

/// Native start records reserve one packed attachment list per engine. The
/// matching finalizer flag enables completion visibility for that same list.
pub(crate) const fn extended_attachment_offsets(fragment:bool)->(usize,usize) {
    if fragment {(0xa8,0x324)} else {(0x88,0x2ec)}
}

/// Send both pieces through the same bounded writer used for firmware storage.
#[cfg(any(test, CONFIG_DRM_ASAHI_M3_T8122))]
pub(crate) fn patch_extended_attachments<E>(fragment:bool,
    attachments:&super::agx_attachments::Attachments,
    mut write:impl FnMut(usize,&[u8])->Result<(),E>)->Result<(),E> {
    let (start,flag)=extended_attachment_offsets(fragment);
    write(start,&attachments.0)?;
    write(flag,&[u8::from(attachments.count()!=0)])
}

#[cfg(test)]
mod attachment_tests {
    use super::*;
    use super::super::agx_attachments::{Attachments,SIZE};
    #[test] fn each_engine_keeps_its_list_and_can_clear_a_previous_list() {
        for (fragment,start,flag,pool) in [(false,0x88,0x2ec,0x190),(true,0xa8,0x324,0x1b0)] {
            let mut bytes=[0xa5;STORAGE];
            let list=Attachments::new(&[(0x100_0040,0x180),(0x200_0000,0x4000)]).unwrap();
            let mut apply=|list:&Attachments|patch_extended_attachments(fragment,list,|at,data|{
                bytes.get_mut(at..at+data.len()).ok_or(Error::Size)?.copy_from_slice(data);Ok::<(),Error>(())
            });
            apply(&list).unwrap();
            assert_eq!(&bytes[start..start+SIZE],&list.0);
            assert_eq!(bytes[flag],1);
            assert_eq!(&bytes[pool..pool+8],&[0xa5;8]);
            patch_extended_attachments(fragment,&Attachments::EMPTY,|at,data|{
                bytes[at..at+data.len()].copy_from_slice(data);Ok::<(),Error>(())
            }).unwrap();
            assert!(bytes[start..start+SIZE].iter().all(|b|*b==0));
            assert_eq!(bytes[flag],0);
            assert_eq!(bytes[start-1],0xa5);
            assert_eq!(bytes[start+SIZE],0xa5);
        }
    }
}
