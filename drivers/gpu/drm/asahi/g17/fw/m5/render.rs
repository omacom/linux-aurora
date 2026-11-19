// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! M5 tiling and fragment command encoders, with shared kick bindings.

#![allow(dead_code)]

use super::super::kick::{KickTimestamp, RegisterWrite, RegisterArrayBinding};

use super::render_tail::{self as tail, RenderStage, RenderTail};

pub(crate) const M5_TA_COMMAND_BYTES: usize = 0x990;
pub(crate) const M5_3D_COMMAND_BYTES: usize = 0x2240;
pub(crate) const M5_TA_SELECTOR: u32 = 0;
pub(crate) const M5_3D_SELECTOR: u32 = 1;

pub(crate) const M5_TA_PROGRAM_OFFSET: u32 = 0x60;

pub(crate) const M5_3D_PROGRAM_OFFSETS: [u32; 4] = [0xa0, 0x7c0, 0xee0, 0x1600];

const PROGRAM_AREA: usize = 0x700;
const REGISTER_BYTES: usize = 12;
pub(crate) const M5_PROGRAM_CAPACITY: usize = PROGRAM_AREA / REGISTER_BYTES;

const TA_DISPATCH_REGISTER: u32 = 0x1c898;

const FRAGMENT_TILE_REGISTER: u32 = 0x10038;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum M5RenderCommandError {
    ProgramTooLong,
    ProgramOffset,
    RceCount,
    MissingDispatchMirror,
    AddressOverflow,
    InvalidCompletionState,
    InvalidUsageHint,
    Kick,
    SharedRing,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct M5ProgramRef<'a> {
    pub registers: &'a [RegisterWrite],

    pub rce_count: u16,
    pub command_offset: u32,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct M5FragmentProgramRefs<'a> {

    pub main: M5ProgramRef<'a>,

    pub main_mid_count: u16,
    pub store: M5ProgramRef<'a>,
    pub resume: M5ProgramRef<'a>,
    pub load: M5ProgramRef<'a>,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct M5StageCompletion {

    pub stamp_addresses: [u64; 2],

    pub stamp: u32,

    pub queue_index: u32,

    pub ordinal: u32,

    pub has_wait: bool,

    pub completion: u64,

    pub key: u64,
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(crate) struct M5HardwareStamp {
    pub stamp: u32,

    pub entries: u32,

    pub generation: u64,

    pub base: u64,
}

pub(crate) fn m5_hardware_stamp(p: &M5HardwareStamp) -> Result<u64, M5RenderCommandError> {
    if p.entries == 0 {
        return Err(M5RenderCommandError::InvalidCompletionState);
    }
    let value = (u64::from(p.stamp >> 8) | p.generation << 24).wrapping_sub(1);
    let quotient = (value / u64::from(p.entries)) as u32;
    let remainder = (value as u32).wrapping_sub(quotient.wrapping_mul(p.entries));
    let slot = (p.base as u32 & 0xff) + (remainder & 0xff);
    let carry = slot / p.entries;
    let index = (slot - carry * p.entries) & 0xff;
    let phase = ((p.base >> 8) as u32)
        .wrapping_add(quotient)
        .wrapping_add(carry);
    Ok(u64::from(index) | u64::from(phase) << 8)
}

pub(crate) fn m5_render_completion(
    p: &M5HardwareStamp,
    pass: u8,
) -> Result<u64, M5RenderCommandError> {
    Ok(m5_hardware_stamp(p)? | u64::from(pass) << 40)
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct M5TaLinked3d {

    pub queue_index: u32,

    pub stamp: u32,

    pub flag: u8,

    pub completion: u64,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct M5TaPolicy {
    pub policy_a: u32,
    pub policy_b: u8,
    pub policy_c: u32,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct M5ExtraAddresses {

    pub count: u32,
    pub addresses: [u64; 4],

    pub policy: u8,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct M5TaCommandFields {

    pub sequence: u64,
    pub completion: M5StageCompletion,

    pub pb_slot: u32,

    pub ta_word_a: u32,

    pub pb_state: u64,

    pub pm_scene: u64,

    pub scene_end_va: u64,
    pub ta_word_b: u32,
    pub ta_word_e: u32,
    pub context_token: u64,
    pub tail_descriptor: u64,
    pub resolved_tail_value: u64,
    pub tpc_va: u64,
    pub tpc_size: u64,
    pub linked_3d: M5TaLinked3d,
    pub ta_word_c: u64,
    pub ta_word_d: u64,
    pub deflake_state_va: u64,
    pub stage_flag_a: u8,
    pub predecessor_id: u32,

    pub stage_word_a: u32,
    pub stage_word_b: u32,
    pub timestamp_va: u64,

    pub diagnostic_ta_timestamp_upper: u64,
    pub ta_pair_a: [u64; 2],

    pub timestamp_counts: [u32; 2],

    pub ta_word_f: u64,
    /// The shifted usage hint: the seven bytes of the tail's descriptor value after
    /// its first byte. The byte after them starts the resolved usage pointer.
    pub shifted_usage_hint: u64,

    pub uma_state: u64,

    pub tail_flag: u8,
    pub pb_growth_request: u8,
    pub shader_write_class: u8,
    pub policy: M5TaPolicy,
    pub extra: M5ExtraAddresses,
}

#[derive(Clone, Copy, Debug, Default)]
pub(crate) struct M53dCommandFields {
    pub sequence: u64,
    pub completion: M5StageCompletion,

    pub pass: u8,
    pub context_token: u64,
    pub tail_descriptor: u64,
    pub resolved_tail_value: Option<u64>,
    pub fragment_word_l: u32,

    pub fragment_flag_a: bool,

    pub short_partial_programs: u8,

    pub tail_flag: u8,

    pub pm_state: u64,

    pub pm_scene: u64,

    pub scene_end_va: u64,
    pub tilemap_va: u64,
    pub msaa_control: u64,
    pub fragment_word_a: u32,
    pub sample_count: u32,
    pub utile_grid: u32,
    pub merge_bounds: [u64; 2],
    pub fragment_word_b: u64,
    pub tile_count: u32,
    pub depth_bias_va: u64,
    pub scissor_va: u64,
    pub occlusion_query_va: u64,
    pub fragment_word_c: u64,

    pub binding_pairs: [[u64; 2]; 23],
    pub depth_dimensions: u64,
    pub depth_va: u64,
    pub depth_va_pair: [u64; 2],
    pub stencil_va: u64,
    pub stencil_va_pair: [u64; 2],
    pub fragment_word_d: u64,
    pub fragment_word_e: u64,
    pub depth_stride: u64,
    pub stencil_stride: u64,
    pub depth_compression_va: u64,
    pub depth_compression_stride: u64,
    pub stencil_compression_va: u64,
    pub stencil_compression_stride: u64,
    pub sample_aux_flags: [u64; 2],
    pub size_aux_control: [u64; 2],
    pub eot_program: [u64; 2],
    pub partial_eot_program: [u64; 2],
    pub fragment_word_f: u64,
    pub fragment_word_g: u32,
    pub fragment_word_h: u64,
    pub fragment_word_i: u32,
    pub zs_clear_values: u64,
    pub fragment_word_j: u64,
    pub fragment_word_k: u32,
    pub sample_size: u32,
    pub fragment_pass_flag: u8,
    pub fragment_word_n: u32,
    pub fragment_flag_c: u8,
    pub stage_flag_a: u8,
    pub predecessor_id: u32,

    pub stage_word_a: u32,
    pub stage_word_b: u32,
    pub fragment_pair_a: [u64; 2],

    pub timestamp_counts: [u32; 2],

    pub fragment_word_m: u64,

    pub uma_state: u64,

    pub timestamp_va: [u64; 2],

    pub empty_tile_flags: [u8; 4],
    pub fragment_flag_b: u8,
    pub extra: M5ExtraAddresses,
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct M5RenderCommandInputs {

    pub context_id: u32,

    pub scheduler_record: u64,
    pub ta: M5TaCommandFields,
    pub fragment: M53dCommandFields,
}

impl Default for M5RenderCommandInputs {
    fn default() -> Self {

        Self {
            context_id: 0,
            scheduler_record: 0,
            ta: M5TaCommandFields { stage_word_b: u32::MAX, ..Default::default() },
            fragment: M53dCommandFields { stage_word_b: u32::MAX, ..Default::default() },
        }
    }
}

fn put8(out: &mut [u8], offset: usize, value: u8) {
    out[offset] = value;
}

fn put16(out: &mut [u8], offset: usize, value: u16) {
    out[offset..offset + 2].copy_from_slice(&value.to_le_bytes());
}

fn put32(out: &mut [u8], offset: usize, value: u32) {
    out[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn put64(out: &mut [u8], offset: usize, value: u64) {
    out[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

fn put128(out: &mut [u8], offset: usize, value: [u64; 2]) {
    put64(out, offset, value[0]);
    put64(out, offset + 8, value[1]);
}

fn read32(out: &[u8], offset: usize) -> u32 {
    u32::from_le_bytes(out[offset..offset + 4].try_into().unwrap())
}

fn completion_tail(out: &mut [u8], base: usize, c: &M5StageCompletion) {
    put64(out, base, c.stamp_addresses[0]);
    put64(out, base + 8, c.stamp_addresses[1]);
    put32(out, base + 0x10, c.stamp);
    put32(out, base + 0x14, c.queue_index);
    put32(out, base + 0x18, c.ordinal);
    put32(out, base + 0x1c, u32::from(c.has_wait));
}

/// Fill publication-time completion values without replacing the retained
/// resource, lifecycle or register-program fields of either command.
pub(crate) fn link_completions(
    ta: &mut [u8; M5_TA_COMMAND_BYTES],
    fragment: &mut [u8; M5_3D_COMMAND_BYTES],
    kicks: [KickTimestamp; 2],
    pass: u8,
    fragment_busy: bool,
) {
    put32(ta, 0x7a4, u32::from(fragment_busy));
    put64(ta, 0x7a8, kicks[1].completion_value(pass) & 0xff_ffff_ffff);
    put8(ta, 0x7b0, pass);
    put8(ta, 0x7b1, pass);
    put64(ta, 0x8c6, kicks[0].completion_value(pass));
    put8(fragment, 0x90, pass);
    put64(fragment, 0x2160, kicks[1].completion_value(pass));
}

/// Bind the command-state slot in each engine's completion tail.
pub(crate) fn bind_work_slot(
    ta: &mut [u8; M5_TA_COMMAND_BYTES],
    fragment: &mut [u8; M5_3D_COMMAND_BYTES],
    slot: u32,
) -> Result<(), M5RenderCommandError> {
    if slot >= 4 { return Err(M5RenderCommandError::InvalidCompletionState); }
    put32(ta, 0x8be, slot);
    put32(fragment, 0x2158, slot);
    Ok(())
}

fn extra_tail(out: &mut [u8], stage: RenderStage, e: &M5ExtraAddresses) {
    tail::encode_extra(out, stage, &tail::ExtraAddresses {
        count: e.count, policy: e.policy, addresses: e.addresses,
    }).expect("fixed render command extent");
}

fn check_program(p: &M5ProgramRef<'_>, offset: u32) -> Result<(), M5RenderCommandError> {
    if p.command_offset != offset {
        return Err(M5RenderCommandError::ProgramOffset);
    }
    if p.registers.len() > M5_PROGRAM_CAPACITY {
        return Err(M5RenderCommandError::ProgramTooLong);
    }
    if usize::from(p.rce_count) > p.registers.len() || p.rce_count > 0x7f {
        return Err(M5RenderCommandError::RceCount);
    }
    Ok(())
}

fn place_program(out: &mut [u8], low_alias: u64, p: &M5ProgramRef<'_>) -> Result<u64, M5RenderCommandError> {
    let offset = p.command_offset as usize;
    let address = low_alias
        .checked_add(u64::from(p.command_offset))
        .ok_or(M5RenderCommandError::AddressOverflow)?;
    put16(out, offset + PROGRAM_AREA + 8, 0);
    put64(out, offset + PROGRAM_AREA, address);
    for (index, register) in p.registers.iter().enumerate() {
        let entry = offset + index * REGISTER_BYTES;
        let header = (read32(out, entry) & 0xfffc0006) | (register.reg & 0x3fff9);
        put32(out, entry, header);
        put64(out, entry + 4, register.value);
    }
    put16(out, offset + PROGRAM_AREA + 8, p.registers.len() as u16);
    Ok(address)
}

/// The TA command's external-write class: whether the render's graphics
/// programs declare writes to memory, as the compiler reports it.
pub(crate) const fn shader_write_class(declared: bool) -> u8 {
    // Forwarded as declared; the TA policy word is not inferred from it.
    declared as u8
}

/// The shifted usage hint of a TA command, by programmed mesh class. The hint
/// covers only the seven bytes before the resolved usage pointer: what looks
/// like a high "pass rotation" bit in a wider read is that pointer's first
/// byte, not part of the hint. Other ordinary vertex programs have different
/// hints.
pub(crate) fn mesh_class_usage_hint(mesh_class: u8) -> u64 {
    match mesh_class {
        0 => 0xc0c0,
        1 => 0x10ac0,
        _ => 0x12b40,
    }
}

pub(crate) fn encode_m5_ta_command(
    out: &mut [u8; M5_TA_COMMAND_BYTES],
    low_alias: u64,
    i: &M5RenderCommandInputs,
    ta: &M5ProgramRef<'_>,
) -> Result<[Option<RegisterArrayBinding>; 4], M5RenderCommandError> {
    check_program(ta, M5_TA_PROGRAM_OFFSET)?;
    let dispatch = ta
        .registers
        .iter()
        .find(|r| r.reg & !1 == TA_DISPATCH_REGISTER)
        .ok_or(M5RenderCommandError::MissingDispatchMirror)?
        .value;
    low_alias.checked_add(u64::from(ta.command_offset))
        .ok_or(M5RenderCommandError::AddressOverflow)?;
    let t = &i.ta;
    let descriptor_value = tail::tiling_descriptor_value(t.tail_descriptor, t.shifted_usage_hint)
        .ok_or(M5RenderCommandError::InvalidUsageHint)?;
    put32(out, 0, M5_TA_SELECTOR);
    put64(out, 4, t.sequence);
    put32(out, 0xc, i.context_id);
    tail::encode(out, RenderStage::Tiling, &RenderTail {
        descriptor_value, resolved_value: Some(t.resolved_tail_value),
        trailing_value: t.context_token,
    }).expect("fixed TA command extent");
    put32(out, 0x8d6, t.ta_word_e);
    put32(out, 0x48, t.ta_word_b);
    put32(out, 0x18, t.pb_slot);
    put32(out, 0x1c, t.ta_word_a);
    put64(out, 0x20, t.pb_state);
    put64(out, 0x28, t.pm_scene);
    put64(out, 0x30, t.scene_end_va);
    put64(out, 0x10, i.scheduler_record);
    put64(out, 0x780, t.tpc_va);
    put64(out, 0x788, t.tpc_size);
    put32(out, 0x79c, t.linked_3d.queue_index);
    put32(out, 0x7a0, t.linked_3d.stamp);
    put32(out, 0x7a4, u32::from(t.linked_3d.flag & 1));
    put64(out, 0x7a8, t.linked_3d.completion & 0xff_ffff_ffff);
    let pass = (t.linked_3d.completion >> 40) as u8;
    put8(out, 0x7b1, pass);
    put8(out, 0x7b0, pass);
    put64(out, 0x7b6, t.ta_word_c);
    put64(out, 0x7be, t.ta_word_d);
    put64(out, 0x7d6, t.deflake_state_va);
    put128(out, 0x7de, [0, 0]);
    put64(out, 0x7ee, 0);
    put32(out, 0x7b2, 0);
    put32(out, 0x86a, u32::from(t.stage_flag_a & 1));
    put32(out, 0x86e, t.predecessor_id);
    put32(out, 0x872, t.stage_word_a);
    put32(out, 0x876, t.stage_word_b);
    put128(out, 0x87a, [0, 0]);
    put128(out, 0x90e, t.ta_pair_a);
    put32(out, 0x91e, t.timestamp_counts[0]);
    put32(out, 0x922, t.timestamp_counts[1]);
    put64(out, 0x926, t.ta_word_f);
    put64(out, 0x92e, t.uma_state);
    put8(out, 0x936, t.tail_flag & 1);
    put64(out, 0x8fe, t.timestamp_va);
    put64(out, 0x906, t.diagnostic_ta_timestamp_upper);
    put32(out, 0x892, u32::from(t.pb_growth_request & 1));
    put32(out, 0x89a, u32::from(t.shader_write_class & 1));
    put32(out, 0x89e, u32::from(m5_ta_policy_bit(&t.policy, t.shader_write_class)));
    put32(out, 0x8a2, 0);
    put64(out, 0x8f6, 0);
    completion_tail(out, 0x8a6, &t.completion);
    put64(out, 0x8c6, t.completion.completion);
    put64(out, 0x8ce, t.completion.key);

    put32(out, 0x38, 0);
    put16(out, 0x3c, 0);
    put32(out, 0x40, 0);
    let address = place_program(out, low_alias, ta)?;
    put16(out, 0x38, ta.rce_count);
    put16(out, 0x3a, ta.registers.len() as u16);
    put16(out, 0x3c, ta.registers.len() as u16);
    put64(out, 0x85e, dispatch);
    extra_tail(out, RenderStage::Tiling, &t.extra);
    let empty = None;
    Ok([Some(RegisterArrayBinding::new(address, ta.rce_count as u8).map_err(|_| M5RenderCommandError::ProgramOffset)?), empty, empty, empty])
}

pub(crate) fn encode_m5_3d_command(
    out: &mut [u8; M5_3D_COMMAND_BYTES],
    low_alias: u64,
    i: &M5RenderCommandInputs,
    f: &M5FragmentProgramRefs<'_>,
) -> Result<[Option<RegisterArrayBinding>; 4], M5RenderCommandError> {
    let programs = [f.main, f.store, f.resume, f.load];
    for (program, offset) in programs.iter().zip(M5_3D_PROGRAM_OFFSETS) {
        check_program(program, offset)?;
        low_alias.checked_add(u64::from(program.command_offset))
            .ok_or(M5RenderCommandError::AddressOverflow)?;
    }
    for partial in &programs[1..] {
        if usize::from(partial.rce_count) != partial.registers.len() {
            return Err(M5RenderCommandError::RceCount);
        }
    }
    if f.main_mid_count < f.main.rce_count || usize::from(f.main_mid_count) > f.main.registers.len() {
        return Err(M5RenderCommandError::RceCount);
    }
    let tile = f
        .main
        .registers
        .iter()
        .find(|r| r.reg & !1 == FRAGMENT_TILE_REGISTER)
        .ok_or(M5RenderCommandError::MissingDispatchMirror)?
        .value;
    let t = &i.fragment;
    put32(out, 0, M5_3D_SELECTOR);
    put64(out, 4, t.sequence);
    put32(out, 0xc, i.context_id);
    tail::encode(out, RenderStage::Fragment, &RenderTail {
        descriptor_value: t.tail_descriptor, resolved_value: t.resolved_tail_value,
        trailing_value: t.context_token,
    }).expect("fixed fragment command extent");
    put32(out, 0x2170, t.fragment_word_l);
    put32(out, 0x2174, u32::from(t.fragment_flag_a));
    put32(out, 0x2100, u32::from(t.short_partial_programs & 1));
    put32(out, 0x20f8, 0);

    put128(out, 0x2178, [0, 0]);
    put64(out, 0x2188, 0);
    put8(out, 0x21d0, t.tail_flag & 1);
    put64(out, 0x20, i.scheduler_record);
    put64(out, 0x28, t.pm_state);
    put64(out, 0x30, t.pm_scene);
    put64(out, 0x38, t.scene_end_va);
    put32(out, 0x58, t.fragment_word_a);
    put64(out, 0x40, t.tilemap_va);
    put64(out, 0x48, t.msaa_control);
    put32(out, 0x50, t.sample_count);
    put32(out, 0x54, t.utile_grid);
    put128(out, 0x60, t.merge_bounds);
    put64(out, 0x70, t.fragment_word_b);
    put32(out, 0x78, t.tile_count);
    put64(out, 0x1d30, t.scissor_va);
    put64(out, 0x1d20, t.depth_bias_va);
    put64(out, 0x1d40, t.occlusion_query_va);
    for (index, pair) in t.binding_pairs.iter().enumerate() {
        put128(out, 0x1d58 + index * 16, *pair);
    }
    put64(out, 0x1ec8, t.depth_va);
    put64(out, 0x1ed0, t.depth_stride);
    put64(out, 0x1ed8, t.depth_compression_stride);
    put64(out, 0x1f28, t.fragment_word_d);
    put128(out, 0x1ee0, t.depth_va_pair);
    put64(out, 0x1ef0, t.depth_compression_va);
    put64(out, 0x1ef8, t.stencil_va);
    put64(out, 0x1f00, t.stencil_stride);
    put64(out, 0x1f08, t.stencil_compression_stride);
    put64(out, 0x1f30, t.fragment_word_e);
    put128(out, 0x1f10, t.stencil_va_pair);
    put64(out, 0x1f20, t.stencil_compression_va);
    put64(out, 0x1f68, t.fragment_word_f);
    put32(out, 0x1f70, t.fragment_word_g);
    put128(out, 0x1f48, t.size_aux_control);
    put128(out, 0x1f74, t.eot_program);
    put64(out, 0x1f88, t.fragment_word_h);
    put32(out, 0x1f90, t.fragment_word_i);
    put128(out, 0x1f94, t.partial_eot_program);
    put64(out, 0x1fa8, t.zs_clear_values);
    put32(out, 0x1fb0, t.sample_size);
    put128(out, 0x1f38, t.sample_aux_flags);
    put64(out, 0x1d50, t.fragment_word_c);
    put64(out, 0x1fc0, t.fragment_word_j);
    put32(out, 0x1fc8, t.fragment_word_k);
    put64(out, 0x1fd0, t.depth_dimensions);
    out[0x1fdc..0x20dc].fill(0);
    put32(out, 0x20dc, 0);
    put128(out, 0x20e0, [0, 0]);
    put32(out, 0x1fd8, 0);
    put32(out, 0x20f0, 0);
    put8(out, 0x90, t.pass);
    put32(out, 0x20fc, u32::from(t.fragment_pass_flag & 1));
    put32(out, 0x2209, t.fragment_word_n);
    put8(out, 0x220d, t.fragment_flag_c);
    put32(out, 0x2104, u32::from(t.stage_flag_a & 1));
    put32(out, 0x2108, t.predecessor_id);
    put32(out, 0x210c, t.stage_word_a);
    put32(out, 0x2110, t.stage_word_b);
    put128(out, 0x2114, [0, 0]);
    put128(out, 0x21a8, t.fragment_pair_a);
    put32(out, 0x21b8, t.timestamp_counts[0]);
    put32(out, 0x21bc, t.timestamp_counts[1]);
    put64(out, 0x21c0, t.fragment_word_m);
    put64(out, 0x21c8, t.uma_state);
    put128(out, 0x2198, t.timestamp_va);
    completion_tail(out, 0x2140, &t.completion);
    put64(out, 0x2160, t.completion.completion);
    put64(out, 0x2190, 0);
    for (index, flag) in t.empty_tile_flags.iter().enumerate() {
        put32(out, 0x2124 + index * 4, u32::from(flag & 1));
    }
    put32(out, 0x2138, u32::from(t.fragment_flag_b & 1));
    put64(out, 0x2168, t.completion.key);
    extra_tail(out, RenderStage::Fragment, &t.extra);

    put32(out, 0x80, 0);
    put16(out, 0x84, 0);
    put32(out, 0x88, 0);
    let mut rce = [None; 4];
    for (slot, program) in rce.iter_mut().zip(programs.iter()) {
        let address = place_program(out, low_alias, program)?;
        *slot = Some(RegisterArrayBinding::new(address, program.rce_count as u8).map_err(|_| M5RenderCommandError::ProgramOffset)?);
    }
    put64(out, 0x1f58, tile);
    put64(out, 0x1f60, 0);
    put16(out, 0x80, f.main.rce_count);
    put16(out, 0x82, f.main_mid_count);
    put16(out, 0x84, f.main_mid_count);
    put16(out, 0x88, f.main.registers.len() as u16);
    Ok(rce)
}

fn m5_ta_policy_bit(p: &M5TaPolicy, shader_write_class: u8) -> bool {
    if p.policy_a == 0 || !(1..=2).contains(&p.policy_b) || p.policy_c == 3 {
        return false;
    }
    match p.policy_a {
        1 | 4 => shader_write_class & 1 == 0,
        3 => true,
        _ => false,
    }
}
