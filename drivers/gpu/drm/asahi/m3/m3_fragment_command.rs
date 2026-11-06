// SPDX-License-Identifier: GPL-2.0-only OR MIT
use super::{agx_render::Geometry, agx_render_state::{Program,DepthStencil},
    m3_compute_layout::{GpuVa,RegisterSet}, m3_init_layout::{Error,Region},
    m3_queue_layout::FirmwareVa, m3_render_sequence::fragment};
pub(crate) const SIZE:usize=0xc73;
pub(crate) const EXTENDED_SIZE:usize=0xca0;
pub(crate) const EXTENDED_FLUSH:usize=0xbdc;
pub(crate) const REGISTERS:usize=fragment::REGISTERS;
/// The registers G15G does not take; the others keep the G15S order.
pub(crate) const G15G_OMIT:[u32;4]=[0x160a8,0x160b8,0xa5a9,0xd429];
pub(crate) const REGISTER_COUNT:usize=79;
pub(crate) const REGISTER_STRIDE:usize=12;
pub(crate) const PARAMETERS:usize=0x790;
pub(crate) const META:usize=0xbac;
pub(crate) const SEQUENCE_LENGTH:usize=0x1c;
pub(crate) const STAMP_VALUE:usize=META+0x10;
pub(crate) const FLUSH_STAMPS:usize=META+0x1c;
fn word(out:&mut[u8],off:usize,value:u32) {out[off..off+4].copy_from_slice(&value.to_le_bytes());}
fn pointer(out:&mut[u8],off:usize,value:u64) {out[off..off+8].copy_from_slice(&value.to_le_bytes());}
/// The initial producer had different register/mirror values for tilebuffer
/// and tile mode. Keep those fields independent; the live adapter supplies
/// the same current value to both, exactly as the previous patch path did.
pub(crate) struct State {
    pub(crate) geometry:Geometry,
    pub(crate) multisample:u64,
    pub(crate) merge_upper:[u32;2],
    pub(crate) eot:Program,
    pub(crate) background:Program,
    pub(crate) partial_eot:Program,
    pub(crate) partial_background:Program,
    pub(crate) scissor:u64,
    pub(crate) depth_bias:u64,
    pub(crate) query:u64,
    pub(crate) isp_control:u32,
    pub(crate) tilebuffer_control:u32,
    pub(crate) mirror_tilebuffer_control:u32,
    pub(crate) tile_mode:u64,
    pub(crate) mirror_tile_mode:u32,
    pub(crate) depth:DepthStencil,
    pub(crate) stencil:DepthStencil,
    // Paired ZLS metadata address/stride. Native 26A428 compressed depth
    // witness: 153c1/153c9 and partial mirror +1e0; independent of image data.
    pub(crate) depth_compression:DepthStencil,
    pub(crate) stencil_compression:DepthStencil,
    pub(crate) zls_control:u64,
    pub(crate) depth_dimensions:u32,
    pub(crate) depth_clear:u32,
    pub(crate) stencil_clear:u32,
    pub(crate) sample_size:u32,
    pub(crate) process_empty_tiles:bool,
}
pub(crate) struct Command {
    pub(crate) context:u32,
    /// The register list of the GPU: G15S's, or G15G's without [`G15G_OMIT`].
    pub(crate) register_set:RegisterSet,
    pub(crate) sequence:Region,
    pub(crate) notifier:FirmwareVa,
    pub(crate) manager:FirmwareVa,
    pub(crate) scene:FirmwareVa,
    pub(crate) empty:FirmwareVa,
    pub(crate) gpu_alias:GpuVa,
    pub(crate) tilemap:GpuVa,
    pub(crate) heap:GpuVa,
    pub(crate) auxiliary:GpuVa,
    pub(crate) scene_user:GpuVa,
    pub(crate) scratch:GpuVa,
    pub(crate) pool:FirmwareVa,
    pub(crate) stamp:FirmwareVa,
    pub(crate) fw_stamp:FirmwareVa,
    pub(crate) stamp_value:u32,
    pub(crate) timestamps:[FirmwareVa;2],
    pub(crate) user_timestamps:[FirmwareVa;2],
    pub(crate) state:State,
}
impl Command {
    /// Check references before touching output. The caller owns validation of
    /// client resource mappings; this layer serializes their semantic state.
    pub(crate) fn encode(&self,out:&mut[u8])->Result<(),Error> {
        self.encode_format(out,None)
    }
    pub(crate) fn encode_extended(&self, dispatch:super::m3_tiler_command::Dispatch26,
        out:&mut[u8])->Result<(),Error> {
        self.encode_format(out,Some(dispatch))
    }
    fn encode_format(&self,out:&mut[u8],dispatch:Option<super::m3_tiler_command::Dispatch26>)->Result<(),Error> {
        let extended=dispatch.is_some();
        let size=if extended {EXTENDED_SIZE} else {SIZE};
        if out.len()<size {return Err(Error::Size);}
        if let Some(native)=&dispatch {
            if native.incarnation==0 || native.serial==0 || native.usc_window&0xffff_ffff!=0
                || native.usc_window>=1<<42 || self.register_set!=RegisterSet::G15G {return Err(Error::Address);}
        }
        if self.context>=64 {return Err(Error::Address);}
        let sequence_size=if extended {0x370} else {fragment::LENGTH};
        let seq=self.sequence.at(0,sequence_size)?;
        let alias=self.gpu_alias.get();
        GpuVa::new(alias.checked_add(((size-1)&!3) as u64).ok_or(Error::Address)?).map_err(|_|Error::Address)?;
        let registers=alias+REGISTERS as u64;
        let s=&self.state;let g=&s.geometry;
        let layer_metadata=self.heap.get().checked_add(if g.layers>1 {0x200}else{0}).ok_or(Error::Address)?;
        GpuVa::new(layer_metadata).map_err(|_|Error::Address)?;
        let user=super::agx_render_state::compact(self.scene_user.get().checked_add(if extended {0} else {0x40}).ok_or(Error::Address)?)
            .map_err(|_|Error::Address)? & !15;
        let width=(g.pixels&0xffff)+1;let height=(g.pixels>>16)+1;
        let tx=(width+31)/32;let ty=(height+31)/32;
        let native_tiles=221201|(u64::from(g.layers>1)<<32)|(u64::from(tx-1)<<44)
            |(u64::from(ty-1)<<53)|137438953472|((g.utile_config&61440)<<28);
        let values:[(u32,u64);REGISTER_COUNT]=[
            (0x1739,1),(0x10009,g.utile_config),(0x15379,s.eot.resources),(0x15381,s.eot.address),
            (0x15369,s.background.resources),(0x15371,s.background.address),
            (0x15131,u64::from(s.merge_upper[0])),(0x15139,u64::from(s.merge_upper[1])),
            (0x100a1,0),(0x15069,0),(0x15071,0),(0x16058,0),(0x10019,s.multisample),
            (0x100b1,u64::from(g.isp_blocks)),(0x16030,u64::from(g.isp_blocks)),
            (0x100d9,u64::from(g.screen)),(0x10791,0xff0200),(0x16098,self.heap.get()),
            (0x15109,s.scissor),(0x15101,s.depth_bias),(0x15021,u64::from(s.isp_control)),
            (0x15211,(u64::from(height)<<32)|u64::from(width)),(0x15049,0x100000),
            (0x10051,u64::from(s.tilebuffer_control)),(0x15321,u64::from(s.depth_dimensions)),
            (0x15301,u64::from(s.depth_clear)),(0x15309,u64::from(s.stencil_clear)),
            (0x15311,s.query&!15),(0x15319,s.zls_control),(0x15349,0x04040404),(0x15351,0),
            (0x15329,s.depth.base),(0x15331,s.depth.base),(0x15339,s.stencil.base),(0x15341,s.stencil.base),
            (0x15231,0),(0x15221,0),(0x15239,0),(0x15229,0),
            (0x15401,s.depth.stride),(0x15421,s.depth.stride),(0x15409,s.stencil.stride),(0x15429,s.stencil.stride),
            (0x153c1,s.depth_compression.base),(0x15411,s.depth_compression.stride),
            (0x153c9,s.depth_compression.base),(0x15431,s.depth_compression.stride),
            (0x153d1,s.stencil_compression.base),(0x15419,s.stencil_compression.stride),
            (0x153d9,s.stencil_compression.base),(0x15439,s.stencil_compression.stride),
            (0x16429,self.tilemap.get()),(0x16060,layer_metadata),(0x16431,u64::from(4*g.region_stride)<<24),
            (0x10039,s.tile_mode),(0x16020,0),(0x16451,0),(0x15359,0),(0x100b8,0x8860),
            (0x16461,self.auxiliary.get()),(0x16090,self.auxiliary.get()),(0x120a1,0x1c),(0x101e9,0x1c),
            (0x160a8,0),(0x16068,native_tiles),(0x160b8,0),
            (0x1a0a9,0),(0x1a0b1,0),(0x1a079,0),(0x1a081,0),(0x1a0d9,0),(0x1a0e1,0),
            (0x101c1,0),(0xd469,0),(0x1a0f9,8),(0x1c838,0),(0x1ca28,user),
            (0xa5a9,0x6000400020),(0xd429,0x200000001),
        ];
        let bytes=&mut out[..size];bytes.fill(0);
        word(bytes,0,1); // fragment opcode; work counter remains zero
        word(bytes,0xc,self.context);
        for (offset,value) in [(0x14,seq),(0x20,self.notifier.get()),(0x28,self.manager.get()),
            (0x30,self.scene.get()),(0x38,self.empty.get()),(0x40,self.tilemap.get()),(0x780,registers)] {pointer(bytes,offset,value);}
        word(bytes,SEQUENCE_LENGTH,sequence_size as u32);
        if extended {
            pointer(bytes,4,dispatch.as_ref().map_or(0,|native|native.serial));
            word(bytes,0x50,1);
            word(bytes,0x54,g.isp_blocks);
            word(bytes,0x60,g.x_blocks);
            word(bytes,0x64,g.y_blocks);
            word(bytes,0x68,g.blocks);
            word(bytes,0x78,g.tiles);
        } else {
            pointer(bytes,0x48,0x88);word(bytes,0x50,1);
            bytes[0x54..0x56].copy_from_slice(&((g.y_blocks>>18) as u16).to_le_bytes());
            bytes[0x56..0x58].copy_from_slice(&((g.x_blocks>>18) as u16).to_le_bytes());
            word(bytes,0x68,s.merge_upper[0]);word(bytes,0x6c,s.merge_upper[1]);
            pointer(bytes,0x78,u64::from(g.tiles));
        }
        let g15g=self.register_set==RegisterSet::G15G;
        let mut count=0;
        let mut emit=|register,value| {
            word(bytes,REGISTERS+count*REGISTER_STRIDE,register);
            pointer(bytes,REGISTERS+count*REGISTER_STRIDE+4,value);
            count+=1;
        };
        for (register,mut value) in values {
            if extended {
                if matches!(register,0x160b8|0x120a1|0x1c838|0x1ca28) {continue;}
                if register==0x10791 {emit(0xa301,0);}
                if register==0x16068 {value=(value&!0x3ffff)|0x3717f;}
                if matches!(register,0x15109|0x15101) {value&=!3;}
            } else if g15g && G15G_OMIT.contains(&register) {continue;}
            emit(register,value);
        }
        if let Some(native)=&dispatch {
            for register in [0x160e0,0x1499,0xa341] {emit(register,fragment::UUID.into());}
            emit(0x1c838,0);
            emit(0x1ca28,user);
            emit(0x10069,native.usc_window);
        }
        bytes[0x788..0x78a].copy_from_slice(&(count as u16).to_le_bytes());
        bytes[0x78a..0x78c].copy_from_slice(&((count*REGISTER_STRIDE) as u16).to_le_bytes());
        // Start3DJobParameters3: mirror records use their actual packed widths.
        for (offset,value) in [(0,s.depth_bias),(0x10,s.scissor),(0x20,s.query),
            (0x168,if extended {s.background.resources}else{s.partial_background.resources}),
            (0x170,if extended {s.background.address}else{s.partial_background.address}),
            (0x198,s.partial_background.resources),(0x1a0,s.partial_background.address),
            (0x1a8,s.zls_control),(0x1b8,s.depth.base),(0x1c0,s.depth.stride),
            (0x1c8,s.depth_compression.stride),(0x1e0,s.depth_compression.base),
            (0x1d0,s.depth.base),(0x1d8,s.depth.base),
            (0x1e8,s.stencil.base),(0x1f0,s.stencil.stride),(0x200,s.stencil.base),(0x208,s.stencil.base),
            (0x1f8,s.stencil_compression.stride),(0x210,s.stencil_compression.base),
            (0x26c,if extended {s.eot.address}else{s.partial_eot.address}),(0x28c,s.partial_eot.address),(0x2c0,u64::from(s.depth_dimensions))] {
            pointer(bytes,PARAMETERS+offset+if extended && offset<=0x20 {0x10}else{0},value);
        }
        for (offset,value) in [(0x1b0,0x04040404),(0x228,s.mirror_tilebuffer_control),
            (0x230,s.isp_control),(0x238,width),(0x23c,height),(0x240,0x100000),
            (0x248,s.mirror_tile_mode),(0x264,if extended {s.eot.resources as u32}else{s.partial_eot.resources as u32}),(0x284,s.partial_eot.resources as u32),
            (0x298,s.depth_clear),(0x29c,s.stencil_clear),(0x2a0,s.sample_size)] {word(bytes,PARAMETERS+offset,value);}
        if let Some(native)=dispatch {
            pointer(bytes,0xbc0,self.stamp.get());
            pointer(bytes,0xbc8,self.fw_stamp.get());
            word(bytes,0xbd0,self.stamp_value);
            word(bytes,0xbd4,1);
            word(bytes,EXTENDED_FLUSH,1);
            word(bytes,0xbe0,fragment::UUID);
            pointer(bytes,0xc10,self.timestamps[0].get());
            pointer(bytes,0xc18,self.timestamps[1].get());
            pointer(bytes,0xc20,self.user_timestamps[0].get());
            pointer(bytes,0xc28,self.user_timestamps[1].get());
            pointer(bytes,0xc46,self.pool.get());
            bytes[0xc4e]=1;
            pointer(bytes,0xc57,native.firmware_aux.get());
            bytes[0xc5f]=native.incarnation;
            bytes[0xc84]=1;
        } else {
        word(bytes,0xb88,0x67); // EncoderParams.encoder_id
        word(bytes,0xb90,u32::MAX); // EncoderParams.unk_mask
        word(bytes,0xb94,u32::from(s.process_empty_tiles));
        word(bytes,0xb98,1); // retained no_clear_pipeline_textures
        pointer(bytes,META,self.stamp.get());pointer(bytes,META+8,self.fw_stamp.get());
        word(bytes,STAMP_VALUE,self.stamp_value);word(bytes,META+0x14,1); // event slot
        word(bytes,FLUSH_STAMPS,1); // flush completion stamps
        word(bytes,META+0x20,fragment::UUID);word(bytes,0xbe8,1); // unk_buf2.unk_10
        for (offset,value) in [(8,self.timestamps[0].get()),(16,self.timestamps[1].get()),
            (0x18,self.user_timestamps[0].get()),(0x20,self.user_timestamps[1].get()),
            (0x2e,self.pool.get()),(0x37,0xad0000),(0x3f,0x1600000),(0x47,self.scratch.get())] {
            pointer(bytes,fragment::COMMAND_TAIL+offset,value);
        }
        bytes[fragment::COMMAND_TAIL+0x36]=1;bytes[fragment::COMMAND_TAIL+0x4f]=1;
        }
        Ok(())
    }
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    #[test]
    fn native_fragment_uses_distinct_program_mirrors_and_completion_tail() {
        let fw=|n:u64|FirmwareVa::new(0xffff_fc25_3000_0000+n*0x4000).unwrap();
        let gpu=|n:u64|GpuVa::new(0x10_0300_0000+n*0x4000).unwrap();
        let program=|n:u64|Program{address:0x1_0000_0000+n*0x1000,resources:n};
        let surface=DepthStencil{base:0,stride:0};
        let command=Command {context:3,register_set:RegisterSet::G15G,
            sequence:Region::new(fw(0).get(),0x1000).unwrap(),notifier:fw(1),manager:fw(2),scene:fw(3),empty:fw(4),
            gpu_alias:GpuVa::new(0x72_1000_4000).unwrap(),tilemap:gpu(0),heap:gpu(1),auxiliary:gpu(2),scene_user:gpu(3),
            scratch:gpu(4),pool:fw(5),stamp:fw(6),fw_stamp:fw(7),stamp_value:0x3d000100,
            timestamps:[fw(8),fw(9)],user_timestamps:[fw(10),fw(11)],
            state:State {geometry:Geometry::new(64,64,super::super::agx_render::Utile::Pixels32,
                super::super::agx_render::Utile::Pixels32,1).unwrap(),multisample:0,merge_upper:[0;2],
                eot:program(1),background:program(2),partial_eot:program(3),partial_background:program(4),
                scissor:0,depth_bias:0,query:0,isp_control:0xc000,tilebuffer_control:1,mirror_tilebuffer_control:1,
                tile_mode:0x280,mirror_tile_mode:0x280,depth:surface,stencil:surface,depth_compression:surface,
                stencil_compression:surface,zls_control:0,depth_dimensions:0,depth_clear:0,stencil_clear:0,
                sample_size:4,process_empty_tiles:false}};
        let dispatch=||super::super::m3_tiler_command::Dispatch26 {incarnation:9,serial:1,peer_stamp:0,
            usc_window:1<<32,firmware_aux:fw(12),scene_scratch:gpu(4)};
        let mut bytes=[0xa5;EXTENDED_SIZE+8];
        command.encode_extended(dispatch(),&mut bytes).unwrap();
        let word=|at|u32::from_le_bytes(bytes[at..at+4].try_into().unwrap());
        let pointer=|at|u64::from_le_bytes(bytes[at..at+8].try_into().unwrap());
        assert_eq!(word(0xbd0),command.stamp_value);
        assert_eq!(word(EXTENDED_FLUSH),1);
        assert_eq!(pointer(0x900),command.state.background.address);
        assert_eq!(pointer(0x930),command.state.partial_background.address);
        assert_eq!(bytes[0xc5f],9);
        assert_eq!(&bytes[EXTENDED_SIZE..],&[0xa5;8]);
        let mut short=[0xa5;EXTENDED_SIZE-1];
        assert!(command.encode_extended(dispatch(),&mut short).is_err());
        assert!(short.iter().all(|b|*b==0xa5));
    }
}
