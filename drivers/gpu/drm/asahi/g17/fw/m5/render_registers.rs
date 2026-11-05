// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! M5 register adaptation over the shared render-pass/resource lowering.
#![allow(dead_code)]
use kernel::prelude::*;
use super::super::{kick::RegisterWrite, render::{RenderArgs, RenderPass, M5RegisterPrograms as RegisterPrograms}};
use super::render::M5RenderCommandInputs;
const DEFLAKE_STATE_OFFSET: u64 = 0x980;
const DEFLAKE_AUX_OFFSET: u64 = 0xc0;

fn utile_config(p: &RenderPass) -> u64 {
    (u64::from(p.utile_width / 16) << 12) | (u64::from(p.utile_height / 16) << 14)
        | u64::from(p.samples_log2)
}
#[derive(Copy, Clone)]
pub(crate) struct M5RenderStageControls {
    pub(crate) scs: crate::g17::render_resources::StageScratch,

    pub(crate) resource_flags: u64,

    pub(crate) resource_control: u64,

    pub(crate) ptc_max: u32,
    pub(crate) work_key: u64,
}

#[derive(Copy, Clone)]
pub(crate) struct M5RenderControls {
    /// Encoded event-command state for the initial fragment register list.
    pub(crate) fragment_event_control: u64,
    pub(crate) ta: M5RenderStageControls,
    pub(crate) fragment: M5RenderStageControls,
    pub(crate) ta_tile_control: u64,

    pub(crate) ta_dispatch_control: u64,
    pub(crate) fragment_raster_control: u64,

    pub(crate) fragment_tile_flags: u64,

    pub(crate) store_pipeline_2_bind: u64,
    pub(crate) store_pipeline_2: u64,
    pub(crate) partial_store_pipeline_2_bind: u64,
    pub(crate) partial_store_pipeline_2: u64,
    pub(crate) partial_tile_flags: u64,
    pub(crate) aux_fb_flags: u64,
    pub(crate) aux_fb_control: u64,
}

struct RegisterWriter<'a> {
    values: &'a mut [RegisterWrite],
    count: usize,
}

impl RegisterWriter<'_> {
    fn push(&mut self, number: u32, value: u64) -> Result {
        let slot = self.values.get_mut(self.count)
            .ok_or(EINVAL)?;
        *slot = RegisterWrite { reg: number, value };
        self.count += 1;
        Ok(())
    }
    fn finish(self) -> Result {
        if self.count != self.values.len() {
            return Err(EINVAL);
        }
        Ok(())
    }
}

fn value(registers: &[RegisterWrite], number: u32) -> Result<u64> {
    registers
        .iter()
        .find(|r| r.reg == number)
        .map(|r| r.value)
        .ok_or(EINVAL)
}

/// The event-command state of the initial fragment register list: 0 when
/// disabled, 3 for kind 1, otherwise 1.
pub(crate) const fn fragment_event_control(enabled: bool, kind: u32) -> u64 {
    if !enabled { 0 } else if kind == 1 { 3 } else { 1 }
}

const fn ptc_control(maximum: u32) -> u64 {
    let maximum = if maximum == 0 { 0x60 } else { maximum };
    ((maximum as u64) << 32) | 0x400020
}

fn tile_state(p: &RenderPass, flags: u64) -> Result<u64> {
    let tiles_x = (p.width as u64 + 31) / 32;
    let tiles_y = (p.height as u64 + 31) / 32;
    if !(1..=1024).contains(&tiles_x) || !(1..=1024).contains(&tiles_y) {
        return Err(EINVAL);
    }

    let geometry_mask = (0xfffff_u64 << 42) | (0xf_u64 << 38) | (1_u64 << 32);
    if flags & geometry_mask != 0 {
        return Err(EINVAL);
    }
    Ok(flags
        | ((tiles_x - 1) << 42)
        | ((tiles_y - 1) << 52)
        | ((utile_config(p) & 0xf000) << 26)
        | if p.layers > 1 { 1_u64 << 32 } else { 0 })
}

fn tile_blocks(pixels: u32) -> u64 {
    let tiles = (pixels as u64 + 31) / 32;
    let first = (((tiles + 3) / 4) + 3) & !3;

    (3 * first) | ((2 * first) << 10) | (first << 20)
}

pub(crate) fn build_ta_registers_into(
    p: &RenderPass,
    controls: &M5RenderControls,
    free_list_slot: u32,
    out: &mut [RegisterWrite; 82],
    common: &[RegisterWrite; 73],
) -> Result {
    if controls.ta_dispatch_control & 1 != 0 {
        return Err(EINVAL);
    }

    tile_state(p, controls.fragment_tile_flags)?;
    let mut out = RegisterWriter { values: out, count: 0 };
    for mut register in common.iter().copied() {
        match register.reg {
            0x0a309 | 0x0a5a1 | 0x0d419 => continue,
            // A fixed word of the TA program. Sample locations are bound by
            // their own register.
            0x10131 => register.value = 0x88,
            0x1c8f8 => {
                out.push(0x0d499, controls.ta.resource_flags)?;
                out.push(0x0d419, controls.ta.resource_control)?;
                out.push(0x0a5a1, ptc_control(controls.ta.ptc_max))?;
                register.value = controls.ta_tile_control;
            }
            0x10171 => {
                for (number, source) in [(0x1ca50, 0x101b9), (0x1ca58, 0x10149), (0x1ca60, 0x10141)]
                {
                    out.push(number, value(common, source)?)?;
                }
                out.push(0x1ca70, tile_blocks(p.width))?;
                out.push(0x1ca78, tile_blocks(p.height))?;
                out.push(0x1ca68, value(common, 0x1c081)?)?;
            }
            0x1c069 => register.value = tile_blocks(p.width),
            0x1c071 => register.value = tile_blocks(p.height),
            0x1c898 => {
                if register.value > 1 {
                    return Err(EINVAL);
                }
                register.value |= controls.ta_dispatch_control;
            }
            0x101c9 => register.value = controls.ta.scs.scratch.size_word(),
            0xd471 => register.value = u64::from(controls.ta.scs.auxiliary_allocation),
            0x1a0f1 => register.value = controls.ta.scs.scratch.stack_word(),
            0x1a099 => register.value = controls.ta.scs.intersection.words()[0],
            0x1a0a1 => register.value = controls.ta.scs.intersection.words()[1],
            0x1a069 => register.value = controls.ta.scs.intersection.words()[2],
            0x1a071 => register.value = controls.ta.scs.intersection.words()[3],
            0x1a0c9 => register.value = controls.ta.scs.intersection.words()[4],
            0x1a0d1 => register.value = controls.ta.scs.intersection.words()[5],
            0x10799 => {
                register.value = controls.ta.scs.scratch.window_word()
                    | (register.value & (1 << 25));
                out.push(0x1a149, controls.ta.scs.scratch.driver_word())?;
            },
            0x10209 => {
                out.push(0x0a739, 0)?;
                register.value = controls.ta.work_key;
            }
            0x1c9f0 | 0x14320 => register.value = controls.ta.work_key,
            0x14308 => register.value = free_list_slot as i32 as i64 as u64,
            _ => {}
        }
        out.push(register.reg, register.value)?;
        if register.reg == 0x01740 {
            out.push(0x0a739, 0)?;
        }
    }
    out.finish()
}

pub(crate) fn build_fragment_registers_into(
    p: &RenderPass,
    controls: &M5RenderControls,
    free_list_slot: u32,
    out: &mut [RegisterWrite; 90],
    common: &[RegisterWrite; 89],
) -> Result {
    let geometry = tile_state(p, controls.fragment_tile_flags)?;
    let mut out = RegisterWriter { values: out, count: 0 };
    let mut dispatch_writes = 0;
    let mut tile_writes = 0;
    for mut register in common.iter().copied() {
        match register.reg {
            0x0a301 | 0x15109 | 0x15101 | 0x160a8 | 0x0a5a9 | 0x0d429 => continue,
            0x100b1 => register.reg = 0x16469,
            0x100d9 => register.reg = 0x16471,
            0x100b8 => register.reg = 0x16138,
            0x10791 => {
                let scratch = controls.fragment.scs.scratch;
                register.value = if scratch.enabled() {
                    (controls.fragment_raster_control | (1 << 24)) & 0x200001000204
                        | (scratch.window_word() & 0xff0000)
                        | (controls.fragment_raster_control & (1 << 25))
                } else { controls.fragment_raster_control };
            },
            0x101c1 => register.value = controls.fragment.scs.scratch.size_word(),
            0xd469 => register.value = u64::from(controls.fragment.scs.auxiliary_allocation),
            0x1a0f9 => register.value = controls.fragment.scs.scratch.stack_word(),
            0x1a0a9 => register.value = controls.fragment.scs.intersection.words()[0],
            0x1a0b1 => register.value = controls.fragment.scs.intersection.words()[1],
            0x1a079 => register.value = controls.fragment.scs.intersection.words()[2],
            0x1a081 => register.value = controls.fragment.scs.intersection.words()[3],
            0x1a0d9 => register.value = controls.fragment.scs.intersection.words()[4],
            0x1a0e1 => register.value = controls.fragment.scs.intersection.words()[5],
            0x15321 => {
                out.push(0x0d491, controls.fragment.resource_flags)?;
                out.push(0x0d429, controls.fragment.resource_control)?;
                out.push(0x0a5a9, ptc_control(controls.fragment.ptc_max))?;
            }
            0x16068 => {
                tile_writes += 1;
                if tile_writes == 1 {
                    register.value = geometry;
                }
            }
            0x16020 => {
                dispatch_writes += 1;
                if dispatch_writes == 3 {
                    if register.value != 0 {
                        return Err(EINVAL);
                    }
                    continue;
                }
            }
            0x160e0 => out.push(0x1a151, controls.fragment.scs.scratch.driver_word())?,
            0x10211 => {
                out.push(0x0a741, controls.fragment_event_control)?;
                register.value = controls.fragment.work_key;
            }
            0x10420 => register.value = controls.fragment.work_key,
            0x14048 => register.value = free_list_slot as i32 as i64 as u64,
            _ => {}
        }
        out.push(register.reg, register.value)?;
        if register.reg == 0x15381 {
            out.push(0x153b1, controls.store_pipeline_2_bind)?;
            out.push(0x153b9, controls.store_pipeline_2)?;
        }
        if register.reg == 0x01731 {
            out.push(0x0a741, 0)?;
        }
    }
    if dispatch_writes != 3 || tile_writes != 2 {
        return Err(EINVAL);
    }
    out.finish()
}

#[derive(Copy, Clone)]
pub(crate) struct FragmentDispatch {
    pub(crate) words: [u64; 2],
}

impl FragmentDispatch {

    pub(crate) fn initial(self, descriptor_flag: u8) -> u64 {
        if descriptor_flag & 1 != 0 { return 0x20; }
        ((self.words[0] >> 27) & 0x1f_ffff_f860)
            | (self.words[1].wrapping_shl(2) & 0x3_ffff_f87c)
            | ((self.words[1] >> 32) << 7)
    }
}

#[derive(Copy, Clone)]
pub(crate) struct TaExtra {
    pub(crate) gtp1: u64,
    pub(crate) gtp2: u64,
    pub(crate) gtp4: u64,
    pub(crate) gtp5: u64,
    pub(crate) gtp6: u64,
    pub(crate) gtp7: u64,
    pub(crate) segment_control: u64,
    pub(crate) segment_index: u64,
    pub(crate) min_free_tvb_pages: u64,
    pub(crate) scs_bracket: u8,
    pub(crate) matrix_bracket: u8,
}

pub(crate) struct TaProgram {
    registers: [RegisterWrite; 97],
    length: usize,
}

impl TaProgram {
    pub(crate) const fn empty() -> Self {
        Self { registers: [RegisterWrite {reg: 0, value: 0}; 97], length: 0 }
    }
    pub(crate) fn registers(&self) -> &[RegisterWrite] { &self.registers[..self.length] }
    pub(crate) fn set_lifecycle(&mut self, value: u64) {
        for r in &mut self.registers[..self.length] {
            if matches!(r.reg, 0x1ca10 | 0x14a1 | 0xa349) {
                r.value = value;
            }
        }
    }
    fn push(&mut self, number: u32, value: u64) -> Result {
        let slot = self.registers.get_mut(self.length).ok_or(EINVAL)?;
        *slot = RegisterWrite { reg: number, value };
        self.length += 1;
        Ok(())
    }
}

pub(crate) fn tiling_with_scratch_into(common: &[RegisterWrite], e: TaExtra,
    mesh_storage: u64, out: &mut TaProgram) -> Result {
    out.length = 0;
    if e.scs_bracket & 1 != 0 { out.push(0x17e1, 1)?; }
    if e.matrix_bracket & 1 != 0 { out.push(0x17f1, 1)?; }
    let mut deflake = false;
    for r in common.iter().copied() {
        if r.reg == 0x1748 { continue; }
        if r.reg == 0x1740 {
            if e.scs_bracket & 1 != 0 { out.push(0x17d9, 1)?; }
            if e.matrix_bracket & 1 != 0 { out.push(0x17e9, 1)?; }
            continue;
        }
        out.push(r.reg, r.value)?;
        let register = r.reg;
        match register {
            // In a mesh-object TA program the mesh rendering buffer register
            // follows the scratch-window register.
            0x10799 if mesh_storage != 0 => out.push(0x1ca48, mesh_storage)?,
            0x1c9c8 => { out.push(0x1c041, e.gtp1)?; out.push(0x1c9d0, e.gtp1)?; }
            0x1c880 if !deflake => {
                deflake = true;
                for (n,v) in [(0x1c948,e.gtp4),(0x1c888,e.gtp5),
                    (0x1c890,e.segment_control),(0x1c918,e.segment_index)] { out.push(n,v)?; }
            }
            0x1c9d8 => {
                for (n,v) in [(0x1c089,e.gtp2),(0x1c9e0,e.gtp2),
                    (0x16c41,e.gtp6 & !0x7f),(0x1ca40,e.gtp6 & !0x7f),
                    (0x1c9a8,e.min_free_tvb_pages),(0x1c920,e.gtp7 & !0x3f)] { out.push(n,v)?; }
            }
            _ => {}
        }
    }
    if out.length != 92 + usize::from(e.scs_bracket & 1) * 2 + usize::from(e.matrix_bracket & 1) * 2
        + usize::from(mesh_storage != 0) {
        return Err(EINVAL);
    }
    Ok(())
}

pub(crate) fn fragment_main_with_scratch_into(p: &RenderPass, common: &[RegisterWrite],
    dispatch: FragmentDispatch, descriptor_flag: u8, scs: bool, out: &mut [RegisterWrite]) -> Result {

    let mut n = 0;
    let mut tile = 0;
    for mut r in common.iter().copied() {

        if r.reg == 0x10051 {
            r.value = isp_sample_control(p);
        }
        if r.reg == 0x16068 {
            if tile == 2 { return Err(EINVAL); }
            out[n] = RegisterWrite { reg: 0x160a8,
                value: if tile == 0 { dispatch.initial(descriptor_flag) } else { 0x20 } };
            n += 1;
            tile += 1;
        }
        let slot = out.get_mut(n).ok_or(EINVAL)?;
        *slot = r;
        n += 1;
        // The SCS controls bracket the fragment main list after entry and exit.
        if scs && matches!(r.reg, 0x1739 | 0x1731) {
            *out.get_mut(n).ok_or(EINVAL)? = RegisterWrite {
                reg: if r.reg == 0x1739 { 0x17e1 } else { 0x17d9 }, value: 1 };
            n += 1;
        }
    }
    if n != out.len() || tile != 2 { return Err(EINVAL); }
    Ok(())
}

/// ISP sample allocation, in the M5 register's units.
pub(crate) fn isp_sample_control(p: &RenderPass) -> u64 {
    let samples = 1u32 << p.samples_log2;
    if samples <= 1 { return 0; }
    let sample_bytes = (u32::from(p.sample_size).max(8) + 7) & !7;
    let groups = sample_bytes.div_ceil(32);
    let units = (u32::from(p.utile_width) >> 3) * (u32::from(p.utile_height) >> 2) * groups;
    let value = match samples { 2 => units.max(8) >> 3, 4 => units >> 1, 8 => units, _ => 0 };
    let value = if samples == 2 && p.utile_width == 16 && p.utile_height == 16 { value } else { value >> 1 };
    u64::from(value & 0x1f) << 16
}

fn partial<const N: usize>(common: &[RegisterWrite], insert_after: u32,
    c: &M5RenderControls, load: bool, out: &mut [RegisterWrite; N]) -> Result {
    let mut w = RegisterWriter { values: out, count: 0 };
    for r in common.iter().copied() {
        if r.reg == 0x15359 { continue; }
        w.push(r.reg, r.value | if r.reg == 0x10039 { c.partial_tile_flags } else { 0 })?;
        if r.reg == insert_after {
            w.push(0x153b1, if load { c.store_pipeline_2_bind } else { c.partial_store_pipeline_2_bind })?;
            w.push(0x153b9, if load { c.store_pipeline_2 } else { c.partial_store_pipeline_2 })?;
        }
    }
    w.finish()
}

/// Caller-owned assembly scratch. All resource addresses come from RenderArgs.
pub(crate) struct Workspace {
    common: RegisterPrograms,
    base: [RegisterWrite; 90],
    pub(crate) ta: TaProgram,
    pub(crate) fragment: [RegisterWrite; 94],
    pub(crate) fragment_count: usize,
    pub(crate) store: [RegisterWrite; 17],
    pub(crate) resume: [RegisterWrite; 24],
    pub(crate) load: [RegisterWrite; 12],
    // Assemble in normal cached memory. Firmware command mappings on M5
    // are write-combined and must not be used for header read/modify/write.
    pub(crate) encoded_ta: [u8; super::render::M5_TA_COMMAND_BYTES],
    pub(crate) encoded_fragment: [u8; super::render::M5_3D_COMMAND_BYTES],
}
// SAFETY: Every field consists only of integer storage.
unsafe impl Zeroable for Workspace {}
impl Workspace {
    #[inline(never)]
    pub(crate) fn write(&mut self, args: &RenderArgs<'_>, controls: &M5RenderControls,
        extra: TaExtra, dispatch: FragmentDispatch, inputs: &M5RenderCommandInputs) -> Result {
        self.common.write_m5(args)?;
        let r = args.resources;
        let p = args.pass;
        // The TA's two deflake sub-buffers sit at fixed offsets from the
        // deflake base (DEFLAKE_STATE_OFFSET, DEFLAKE_AUX_OFFSET).
        let base = r.deflake_va.checked_sub(0x10_0000_0000).ok_or(EINVAL)?;
        let mut vdm_seen = false;
        for reg in &mut self.common.ta {
            let register = reg.reg;
            reg.value = match register {
                0x1c880 if vdm_seen => base.checked_add(DEFLAKE_STATE_OFFSET).ok_or(EINVAL)?,
                0x10111 | 0x1c9b0 => base.checked_add(DEFLAKE_STATE_OFFSET).ok_or(EINVAL)?,
                0x10119 | 0x1c9b8 => base.checked_add(DEFLAKE_AUX_OFFSET).ok_or(EINVAL)?,
                0x1c079 | 0x1c9d8 if p.layers == 1 => r.heapmeta_va.checked_sub(0x10_0000_0000).ok_or(EINVAL)?,
                _ => reg.value,
            };
            if reg.reg == 0x1c880 { vdm_seen = true; }
        }
        for reg in &mut self.common.fragment {
            let register = reg.reg;
            reg.value = match register {
                0x16060 if p.layers == 1 => r.heapmeta_va,
                0x15021 => controls.aux_fb_flags,
                0x15049 => controls.aux_fb_control,
                // Main tile-control bits 17..19 carry the empty-tile and
                // depth/stencil attachment action flags. The ordinary M5
                // profile has one stored colour attachment. An unstored ZLS
                // aspect or a pass that skips empty
                // tiles needs its corresponding action bit. Partial phases
                // clear these bits independently and retain their own policy.
                0x10039 => {
                    let skip_empty = !p.process_empty_tiles;
                    reg.value
                        | (u64::from(skip_empty && inputs.fragment.empty_tile_flags[3] & 1 == 0) << 17)
                        | (u64::from(p.depth.va != 0
                            && (p.zls_control & (1 << 15) != 0 || p.zls_control & (1 << 19) == 0)) << 18)
                        | (u64::from(p.stencil.va != 0
                            && (p.zls_control & (1 << 14) != 0 || p.zls_control & (1 << 18) == 0)) << 19)
                }
                _ => reg.value,
            };
        }
        let mut c = *controls;
        c.ta.work_key = super::super::work_key(r.context_id, args.ta.state_word);
        c.fragment.work_key = super::super::work_key(r.context_id, args.fragment.state_word);
        let ta: &mut [RegisterWrite;82] = (&mut self.base[..82]).try_into().map_err(|_| EINVAL)?;
        build_ta_registers_into(p, &c, r.free_list_slot, ta, &self.common.ta)?;
        let mut extra = extra;
        extra.scs_bracket |= u8::from(c.ta.scs.scratch.enabled());
        tiling_with_scratch_into(ta, extra, r.mesh_storage, &mut self.ta)?;
        self.ta.set_lifecycle((u64::from(inputs.ta.predecessor_id) << 32) | u64::from(inputs.ta.completion.key as u32));
        c.fragment_tile_flags = (c.fragment_tile_flags & !0x7ffff) | 0x36011
            | (u64::from(inputs.fragment.empty_tile_flags[3] & 1) << 18);
        build_fragment_registers_into(p, &c, r.free_list_slot, &mut self.base, &self.common.fragment)?;
        self.fragment_count = 92 + usize::from(c.fragment.scs.scratch.enabled()) * 2;
        fragment_main_with_scratch_into(p, &self.base, dispatch, inputs.fragment.empty_tile_flags[3],
            c.fragment.scs.scratch.enabled(), &mut self.fragment[..self.fragment_count])?;
        for reg in &mut self.fragment {
            if matches!(reg.reg, 0x160e0 | 0x1499 | 0xa341) {
                reg.value = (u64::from(inputs.fragment.predecessor_id) << 32) | u64::from(inputs.fragment.completion.key as u32);
            }
        }
        partial(&self.common.store, 0x15381, &c, false, &mut self.store)?;
        // The ordinary SPM store takes its own empty-tile flag, independent
        // of main empty-tile processing and of the SPM load's flag. Stored
        // ZLS surfaces still force empty-tile processing in the store.
        for register in &mut self.store {
            if register.reg == 0x10039 {
                let empty = inputs.fragment.empty_tile_flags[1] & 1 != 0
                    || p.depth.va != 0 || p.stencil.va != 0;
                register.value = (register.value & !(1 << 16))
                    | (u64::from(empty) << 16);
            }
        }
        partial(&self.common.resume, 0x15381, &c, false, &mut self.resume)?;
        // The load phase uses the primary normal store binding.
        // Normal EOT values are in the main array, before any adaptation.
        c.store_pipeline_2_bind = self.common.fragment[2].value << 32;
        c.store_pipeline_2 = self.common.fragment[3].value;
        partial(&self.common.load, 0x15309, &c, true, &mut self.load)?;
        // The SPM load phase takes its empty-tile policy from its own flag,
        // which the fragment command also carries. It differs from the main
        // and partial-store policy for multisample resolve passes.
        for register in &mut self.load {
            if register.reg == 0x10039 {
                register.value = (register.value & !(1 << 16))
                    | (u64::from(inputs.fragment.empty_tile_flags[2] & 1) << 16);
            }
        }
        Ok(())
    }
}

/// Populate command-side copies from the same pass that produces the registers.
/// Queue identity, completion sequencing and resource budgets remain caller-owned.
pub(crate) fn apply_pass(args: &RenderArgs<'_>, c: &M5RenderControls,
    i: &mut M5RenderCommandInputs) -> Result {
    let p=args.pass;let r=args.resources;
    if p.width==0 || p.width>32768 || p.height==0 || p.height>32768
        || p.layers==0 || p.layers>2048 || r.context_id==0 || r.context_id>63 || p.samples_log2>3
        || !matches!((p.utile_width,p.utile_height),(32,32)|(32,16)|(16,16)) { return Err(EINVAL); }
    let usc=|v:u32|p.usc_base.checked_add(u64::from(v&!7)).ok_or(EINVAL);
    let spec=|v:u64|if p.rsrc_spec_hi {v}else{v&u64::from(u32::MAX)};
    // Check every address before changing command inputs.
    let bg=usc(p.bg.offset)?;let pbg=usc(p.partial_bg.offset)?;
    let eot=usc(p.eot_offset)?;let peot=usc(p.partial_eot_offset)?;
    let deflake=r.deflake_va.checked_sub(0x10_0000_0000)
        .and_then(|offset|offset.checked_add(DEFLAKE_STATE_OFFSET)).ok_or(EINVAL)?;
    i.context_id=u32::from(r.context_id);
    i.ta.tpc_va=r.tpc_va;i.ta.tpc_size=r.tpc_size;i.ta.deflake_state_va=deflake;
    let f=&mut i.fragment;
    f.empty_tile_flags[0]=u8::from(p.process_empty_tiles);
    // The admitted ordinary single-sample profile has one stored colour
    // attachment. Its clear-intent flag follows empty-tile intent; a LOAD
    // pass uses zero here, while partial-store ZLS addresses can
    // independently force empty tiles in the register program.
    if p.samples_log2 == 0 {
        f.empty_tile_flags[1] = u8::from(p.process_empty_tiles);
        f.empty_tile_flags[2] = 0;
    } else {
        // Multisample stored-colour CLEAR and LOAD passes both process empty
        // tiles in the SPM load. Only CLEAR sets the store flag; the BG
        // resource specification publishes that intent in bit 6. An empty
        // seed pass has no BG or empty-tile intent.
        f.empty_tile_flags[1] = u8::from(p.process_empty_tiles && p.bg.rsrc_spec & (1 << 6) != 0);
        f.empty_tile_flags[2] = u8::from(p.process_empty_tiles);
    }
    f.depth_bias_va=p.depth_bias_va;f.scissor_va=p.scissor_va;f.occlusion_query_va=p.occlusion_query_va;
    f.binding_pairs[18]=[spec(p.bg.rsrc_spec),bg];
    f.binding_pairs[21]=[spec(p.partial_bg.rsrc_spec),pbg];f.binding_pairs[22][0]=p.zls_control;
    f.depth_dimensions=p.depth_dimensions;f.depth_va=p.depth.va;f.depth_va_pair=[p.depth.va;2];
    f.stencil_va=p.stencil.va;f.stencil_va_pair=[p.stencil.va;2];
    f.depth_stride=p.depth.stride;f.stencil_stride=p.stencil.stride;
    f.depth_compression_va=p.depth.compression_va;f.depth_compression_stride=p.depth.compression_stride;
    f.stencil_compression_va=p.stencil.compression_va;f.stencil_compression_stride=p.stencil.compression_stride;
    f.sample_aux_flags=[isp_sample_control(p),c.aux_fb_flags];
    f.size_aux_control=[u64::from(p.width)|(u64::from(p.height)<<32),c.aux_fb_control];
    f.eot_program=[spec(p.eot_rsrc_spec)<<32,eot];
    f.partial_eot_program=[spec(p.partial_eot_rsrc_spec)<<32,peot];
    f.zs_clear_values=u64::from(p.depth_clear)|(u64::from(p.stencil_clear|0x300)<<32);
    f.tilemap_va=r.tilemap_va;f.msaa_control=p.msaa_control;f.sample_count=1u32<<p.samples_log2;
    let tx=p.width.div_ceil(32);let ty=p.height.div_ceil(32);
    let mx=tx.div_ceil(4).next_multiple_of(4);let my=ty.div_ceil(4).next_multiple_of(4);
    f.utile_grid=((mx*(32/u32::from(p.utile_width)))<<16)|(my*(32/u32::from(p.utile_height)));
    f.merge_bounds[1]=u64::from(p.merge_upper_x)|(u64::from(p.merge_upper_y)<<32);
    f.tile_count=tx*ty;
    // The fragment command copies userspace's imageblock allocation (the
    // sample size), not a tile count: 8 for one RGBA8 target at any size,
    // 16/32/64 for two/four/eight RGBA8 targets and 16 for RGBA32F.
    f.sample_size=u32::from(p.sample_size);
    Ok(())
}
