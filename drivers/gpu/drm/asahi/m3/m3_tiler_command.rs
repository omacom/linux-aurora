// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::{agx_render::Geometry, m3_compute_layout::{GpuVa,RegisterSet},
    m3_init_layout::{Error,Region}, m3_queue_layout::FirmwareVa,
    m3_render_sequence::ta};
pub(crate) const SIZE:usize=0x92b;
pub(crate) const EXTENDED_SIZE:usize=0x940;
pub(crate) const EXTENDED_FLUSH:usize=0x894;
pub(crate) struct Dispatch26 {
    pub(crate) incarnation:u8,
    pub(crate) serial:u64,
    pub(crate) peer_stamp:u32,
    pub(crate) usc_window:u64,
    pub(crate) firmware_aux:FirmwareVa,
    pub(crate) scene_scratch:GpuVa,
}

pub(crate) const REGISTERS:usize=ta::REGISTERS;
/// The registers G15G does not take; the others keep the G15S order.
pub(crate) const G15G_OMIT:[u32;14]=[0x1c041,0x1c9d0,0x1c948,0x1c888,0x1c890,0x1c918,0x1c089,0x1c9e0,0x16c41,0x1ca40,0x1c9a8,0x1c920,0xa5a1,0xd419];
pub(crate) const REGISTER_COUNT:usize=72;
pub(crate) const REGISTER_STRIDE:usize=12;
pub(crate) const STAMP_VALUE:usize=0x780;
pub(crate) const STRUCT3_STAMP:usize=ta::STRUCT3+0xf0;
pub(crate) const FLUSH_STAMPS:usize=ta::STRUCT3+0xfc;
pub(crate) const CONTEXT:usize=0xc;
pub(crate) const SEQUENCE_LENGTH:usize=0x778;
fn word(out:&mut[u8],off:usize,value:u32) {out[off..off+4].copy_from_slice(&value.to_le_bytes());}
fn pointer(out:&mut[u8],off:usize,value:u64) {out[off..off+8].copy_from_slice(&value.to_le_bytes());}
fn compact(address:u64)->Result<u64,Error> {
    super::agx_render_state::compact(address).map_err(|_|Error::Address)
}
pub(crate) struct Command {
    pub(crate) context:u32,
    /// The register list of the GPU: G15S's, or G15G's without [`G15G_OMIT`].
    pub(crate) register_set:RegisterSet,
    pub(crate) notifier:FirmwareVa,
    pub(crate) manager:FirmwareVa,
    pub(crate) scene:FirmwareVa,
    pub(crate) empty:FirmwareVa,
    pub(crate) gpu_alias:GpuVa,
    pub(crate) sequence:Region,
    pub(crate) pool:FirmwareVa,
    pub(crate) scratch:FirmwareVa,
    pub(crate) stamp:FirmwareVa,
    pub(crate) fw_stamp:FirmwareVa,
    pub(crate) stamp_value:u32,
    pub(crate) timestamps:[FirmwareVa;2],
    pub(crate) user_timestamps:[FirmwareVa;2],
    pub(crate) preemption:[GpuVa;3],
    pub(crate) tilemap:GpuVa,
    pub(crate) tpc:GpuVa,
    pub(crate) tpc_bytes:u64,
    pub(crate) heap:GpuVa,
    pub(crate) scene_user:GpuVa,
    /// Qualified register uses the initial scene-list entry for every draw,
    /// even while RenderScene advances. Preserve this separate binding.
    pub(crate) initial_scene_entry:FirmwareVa,
    pub(crate) geometry:Geometry,
    pub(crate) page_count:u32,
    /// Already encoded, validated compact client VDM pointer. Zero is valid
    /// only for host fixture construction before the application patch.
    pub(crate) vdm:u64,
    pub(crate) multisample:u64,
    pub(crate) ppp:u32,
    pub(crate) process_empty_tiles:bool,
}
impl Command {
    /// All address checks precede mutation. Zero all reserved fields, then
    /// emit semantic registers and explicit packed command/tail fields.
    pub(crate) fn encode(&self,out:&mut[u8])->Result<(),Error> {
        self.encode_format(out, None)
    }
    pub(crate) fn encode_extended(&self, dispatch: Dispatch26, out: &mut [u8]) -> Result<(),Error> {
        self.encode_format(out, Some(dispatch))
    }
    fn encode_format(&self, out: &mut [u8], dispatch: Option<Dispatch26>) -> Result<(),Error> {
        let extended = dispatch.is_some();
        let size = if extended { EXTENDED_SIZE } else { SIZE };
        if out.len()<size {return Err(Error::Size);}
        if let Some(native) = &dispatch {
            if native.incarnation == 0 || native.serial == 0 || native.peer_stamp == 0 || native.usc_window & 0xffff_ffff != 0
                || native.usc_window >= 1 << 42 || self.register_set != RegisterSet::G15G {
                return Err(Error::Address);
            }
        }
        if self.context>=64 || self.page_count==0 || self.tpc_bytes==0 {return Err(Error::Address);}
        let registers=self.gpu_alias.get().checked_add(REGISTERS as u64).ok_or(Error::Address)?;
        GpuVa::new(self.gpu_alias.get().checked_add(((size-1)&!3) as u64).ok_or(Error::Address)?)
            .map_err(|_|Error::Address)?;
        let sequence_size = if extended { 0x300 } else { ta::LENGTH };
        let seq=self.sequence.at(0,sequence_size)?;
        let tilemap=compact(self.tilemap.get())?;
        let tpc=compact(self.tpc.get())?;
        let heap=compact(self.heap.get())?;
        let preemption=[compact(self.preemption[0].get())?,compact(self.preemption[1].get())?,compact(self.preemption[2].get())?];
        let user=compact(self.scene_user.get().checked_add(if extended {0} else {0x40}).ok_or(Error::Address)?)?;
        let g=&self.geometry;
        let layers=g.layers;
        let layer_metadata=compact(self.heap.get().checked_add(if layers>1 {0x200}else{0}).ok_or(Error::Address)?)?;
        let layer_mode=if layers>1 {0xc000 | if self.process_empty_tiles{0x2000}else{0x1000} | u64::from(layers-1)}else{0x8000};
        if self.vdm>=0x80_0000_0000 || self.vdm&3!=0 {return Err(Error::Address);}
        let values:[(u32,u64);REGISTER_COUNT]=[
            (0x1748,1),(0x10141,0x200),(0x1c039,tilemap),(0x1c9c8,tilemap),
            (0x1c041,0),(0x1c9d0,0),(0x1c0a1,tpc),
            (0x1c031,heap|1<<63),(0x1c9c0,heap|1<<63),
            (0x1c051,0x003a0012006b0003),(0x1c061,1),
            (0x10149,g.utile_config),(0x10139,self.multisample),
            (0x10111,preemption[0]),(0x1c9b0,preemption[0]),
            (0x10119,preemption[1]),(0x1c9b8,preemption[1]),
            (0x1c958,1),(0x1c950,preemption[2]|1<<50),(0x1c930,0),
            (0x1c880,self.vdm),(0x1c898,0),(0x1c948,0),(0x1c888,0),
            (0x1c890,1),(0x1c918,0),(0x1c079,layer_metadata),(0x1c9d8,layer_metadata),
            (0x1c089,0),(0x1c9e0,0),(0x16c41,0),(0x1ca40,0),
            (0x1c9a8,u64::from(self.page_count)),(0x1c920,0),(0x10151,0),
            (0x1c199,0),(0x1c1a1,0),(0x1c1a9,0),(0x1c1b1,0),(0x1c1b9,0),
            (0x1c8f8,0x8860),(0x1c0b1,u64::from(g.region_stride)),(0x1c850,u64::from(g.region_stride)),
            (0x10131,0x88),(0x10121,2|u64::from(self.ppp&0x201)),
            (0x10129,u64::from(g.pixels)),(0x101b9,u64::from(g.screen)),
            (0x1c069,u64::from(g.x_blocks)),(0x1c071,u64::from(g.y_blocks)),
            (0x1c081,u64::from(g.blocks)),(0x1c0a9,u64::from(g.tpc_stride)),
            (0x10171,0x100),(0x10169,layer_mode),(0x12099,0x1c),(0x101e1,0x1c),
            (0x1c9e8,0),(0x1a099,0),(0x1a0a1,0),(0x1a069,0),(0x1a071,0),
            (0x1a0c9,0),(0x1a0d1,0),(0x101c9,0),(0xd471,0),(0x1a0f1,8),
            (0x10799,0xff0000),(0xa5a1,0x7f00400020),(0xd419,0x200000001),
            (0x1c830,0),(0x1ca30,user),(0x16c39,user),
            (0x1c910,0xa0_00000000|(self.initial_scene_entry.get()&0xf_ffffffff)|1),
        ];
        let bytes=&mut out[..size];bytes.fill(0); // TA opcode is zero
        pointer(bytes,4,dispatch.as_ref().map_or(1, |native| native.serial));
        word(bytes,CONTEXT,self.context);
        for (offset,value) in [(if extended {0x10} else {0x14},self.notifier.get()),
            (if extended {0x20} else {0x24},self.manager.get()),
            (if extended {0x28} else {0x2c},self.scene.get()),
            (if extended {0x30} else {0x34},self.empty.get()),(0x740,registers),
            (0x760,self.tpc.get()),(0x768,self.tpc_bytes),(0x770,seq)] {pointer(bytes,offset,value);}
        let g15g=self.register_set==RegisterSet::G15G;
        let mut count=0;
        let mut emit = |register, value| {
            word(bytes, REGISTERS + count * REGISTER_STRIDE, register);
            pointer(bytes, REGISTERS + count * REGISTER_STRIDE + 4, value);
            count += 1;
        };
        for (register, mut value) in values {
            if g15g && G15G_OMIT.contains(&register) { continue; }
            if extended && matches!(register, 0x12099 | 0x1c898) { continue; }
            if let Some(native) = &dispatch {
                if register == 0x101e1 {
                    emit(0xa309, 0);
                    emit(0x1c8e0, u64::MAX);
                    emit(0x1c8e8, u64::MAX);
                    emit(0x1c898, 0);
                }
                if register == 0x1c910 {
                    let address = native.scene_scratch.get();
                    let escaped = (address & (1 << 42)) >> 3;
                    let bias = if escaped == 0 { 0x70_0000_0000 } else { 0 };
                    value = ((address + bias) & 0x7f_ffff_fffe) | escaped | 1;
                }
            }
            emit(register, value);
        }
        if let Some(native) = &dispatch {
            emit(0xa5a1, 0x60_0040_0020);
            emit(0xd419, 0x2_0000_0001);
            for register in [0x1ca10, 0x14a1, 0xa349] { emit(register, ta::UUID.into()); }
            emit(0x10061, native.usc_window);
        }
        bytes[0x748..0x74a].copy_from_slice(&(count as u16).to_le_bytes());
        bytes[0x74a..0x74c].copy_from_slice(&((count*REGISTER_STRIDE) as u16).to_le_bytes());
        word(bytes,SEQUENCE_LENGTH,sequence_size as u32);
        word(bytes,0x77c,1); // fragment event slot
        if let Some(native) = dispatch {
            word(bytes, STAMP_VALUE, native.peer_stamp);
            pointer(bytes, 0x7a8, preemption[0]);
            pointer(bytes, 0x878, self.stamp.get());
            pointer(bytes, 0x880, self.fw_stamp.get());
            word(bytes, 0x888, self.stamp_value);
            word(bytes, EXTENDED_FLUSH, 1);
            word(bytes, 0x898, ta::UUID);
            pointer(bytes, 0x8c8, self.timestamps[0].get());
            pointer(bytes, 0x8d0, self.timestamps[1].get());
            pointer(bytes, 0x8d8, self.user_timestamps[0].get());
            pointer(bytes, 0x8e0, self.user_timestamps[1].get());
            pointer(bytes, 0x8fe, self.pool.get());
            bytes[0x906] = 1;
            pointer(bytes, 0x90f, native.firmware_aux.get());
            bytes[0x917] = native.incarnation;
        } else {
        word(bytes,STAMP_VALUE,self.stamp_value);
        pointer(bytes,ta::STRUCT3+0x24,preemption[0]);
        word(bytes,ta::STRUCT3+0xbc,0x67); // retained producer encoder_id
        word(bytes,ta::STRUCT3+0xc4,u32::MAX); // unk_53c
        word(bytes,ta::STRUCT3+0xd0,1); // sampler_max
        pointer(bytes,ta::STRUCT3+0xe0,self.stamp.get());
        pointer(bytes,ta::STRUCT3+0xe8,self.fw_stamp.get());
        word(bytes,STRUCT3_STAMP,self.stamp_value);
        word(bytes,FLUSH_STAMPS,1); // flush completion stamps
        word(bytes,ta::STRUCT3+0x100,ta::UUID);
        for (offset,value) in [(8,self.timestamps[0].get()),(16,self.timestamps[1].get()),
            (0x18,self.user_timestamps[0].get()),(0x20,self.user_timestamps[1].get()),
            (0x2e,self.pool.get()),(0x37,0xad0000),(0x3f,0x1600000),
            (0x47,self.scratch.get())] {pointer(bytes,ta::COMMAND_TAIL+offset,value);}
        bytes[ta::COMMAND_TAIL+0x36]=1; // unk_846
        bytes[ta::COMMAND_TAIL+0x4f]=1; // unk_85f
        }
        Ok(())
    }
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    fn command() -> Command {
        let fw=|index:u64|FirmwareVa::new(0xffff_fc25_1000_0000+index*0x4000).unwrap();
        let gpu=|index:u64|GpuVa::new(0x10_0100_0000+index*0x4000).unwrap();
        Command {context:3,register_set:RegisterSet::G15G,notifier:fw(0),manager:fw(1),scene:fw(2),empty:fw(3),
            gpu_alias:GpuVa::new(0x72_1000_0000).unwrap(),sequence:Region::new(fw(4).get(),0x1000).unwrap(),
            pool:fw(5),scratch:fw(6),stamp:fw(7),fw_stamp:fw(8),stamp_value:0x7a000100,
            timestamps:[fw(9),fw(10)],user_timestamps:[fw(11),fw(12)],preemption:[gpu(0),gpu(1),gpu(2)],
            tilemap:gpu(3),tpc:gpu(4),tpc_bytes:0x4000,heap:gpu(5),scene_user:gpu(6),
            initial_scene_entry:fw(13),geometry:Geometry::new(64,64,super::super::agx_render::Utile::Pixels32,
                super::super::agx_render::Utile::Pixels32,1).unwrap(),page_count:128,
            vdm:0x100000,multisample:0,ppp:0,process_empty_tiles:false}
    }
    #[test] fn native_tiler_keeps_peer_completion_and_usc_window_explicit() {
        let value=command();let mut bytes=[0xa5;EXTENDED_SIZE+8];
        value.encode_extended(Dispatch26 {incarnation:9,serial:2,peer_stamp:0x3d000100,
            usc_window:1<<32,firmware_aux:value.scratch,
            scene_scratch:GpuVa::new(0x10_0200_0000).unwrap()},&mut bytes).unwrap();
        let word=|at|u32::from_le_bytes(bytes[at..at+4].try_into().unwrap());
        assert_eq!(word(0x780),0x3d000100);
        assert_eq!(word(0x888),value.stamp_value);
        assert_eq!(word(0x894),1);
        assert_eq!(word(0x748)&0xffff,66);
        assert_eq!(bytes[0x917],9);
        assert_eq!(&bytes[EXTENDED_SIZE..],&[0xa5;8]);
        let registers=&bytes[REGISTERS..REGISTERS+66*REGISTER_STRIDE];
        assert!(!registers.chunks_exact(12).any(|r|u32::from_le_bytes(r[..4].try_into().unwrap())==0x12099));
        let usc=registers.chunks_exact(12).find(|r|u32::from_le_bytes(r[..4].try_into().unwrap())==0x10061).unwrap();
        assert_eq!(u64::from_le_bytes(usc[4..].try_into().unwrap()),1<<32);
    }
}
