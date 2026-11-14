// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Assemble an M5 render pair from shared render resources.
use kernel::prelude::*;
use super::{render_registers as regs, render as cmd};
use super::render_registers::{Workspace, M5RenderControls};
use super::super::{render::RenderArgs, kick::{RegisterArrayBinding, RegisterWrite}};

pub(crate) const FRAGMENT_OFFSET: usize = 0x4000;
pub(crate) const COMMAND_STORAGE_BYTES: usize = 0x8000;

pub(crate) struct EncodedRender {
    pub(crate) rce: [[Option<RegisterArrayBinding>; 4]; 2],
    pub(crate) register_counts: [u16; 5],
    pub(crate) kick_flags: super::super::kick::StageKickFlags,
}

fn commands(storage: &mut [u8]) -> Result<(
    &mut [u8; cmd::M5_TA_COMMAND_BYTES],
    &mut [u8; cmd::M5_3D_COMMAND_BYTES],
)> {
    if storage.len() < COMMAND_STORAGE_BYTES { return Err(EINVAL); }
    let (ta, fragment) = storage.split_at_mut(FRAGMENT_OFFSET);
    Ok(((&mut ta[..cmd::M5_TA_COMMAND_BYTES]).try_into().map_err(|_| EINVAL)?,
        (&mut fragment[..cmd::M5_3D_COMMAND_BYTES]).try_into().map_err(|_| EINVAL)?))
}

pub(crate) fn link_completions(storage: &mut [u8],
    kicks: [super::super::kick::KickTimestamp; 2], pass: u8, fragment_busy: bool) -> Result {
    let (ta, fragment) = commands(storage)?;
    cmd::link_completions(ta, fragment, kicks, pass, fragment_busy);
    Ok(())
}

pub(crate) fn bind_work_slot(storage: &mut [u8], slot: u32) -> Result {
    let (ta, fragment) = commands(storage)?;
    cmd::bind_work_slot(ta, fragment, slot).map_err(|_| EINVAL)
}

#[inline(never)]
pub(crate) fn encode_into(storage: &mut [u8], low: u64,
    args: &RenderArgs<'_>, controls: &M5RenderControls,
    extra: regs::TaExtra, dispatch: regs::FragmentDispatch,
    inputs: &cmd::M5RenderCommandInputs, workspace: &mut Workspace)
    -> Result<EncodedRender> {
    if storage.len() < COMMAND_STORAGE_BYTES { return Err(EINVAL); }
    let fragment_low = low.checked_add(FRAGMENT_OFFSET as u64).ok_or(EINVAL)?;
    fragment_low.checked_add(cmd::M5_3D_COMMAND_BYTES as u64).ok_or(EINVAL)?;
    if inputs.context_id != u32::from(args.resources.context_id) { return Err(EINVAL); }
    workspace.write(args, controls, extra, dispatch, inputs)?;
    let short_partial = [RegisterWrite::new(0x160a8, 0x20), RegisterWrite::new(0x16068, 0x4_0000)];
    let (store, resume, load): (&[_], &[_], &[_]) = if inputs.fragment.short_partial_programs & 1 != 0 {
        (&short_partial, &short_partial, &short_partial)
    } else { (&workspace.store, &workspace.resume, &workspace.load) };
    let partial_counts = [store.len() as u16, resume.len() as u16, load.len() as u16];
    let count = workspace.ta.registers().len() as u16;
    let ta_program = cmd::M5ProgramRef { registers: workspace.ta.registers(),
        rce_count: count - 3, command_offset: cmd::M5_TA_PROGRAM_OFFSET };
    let view = |registers, rce_count, command_offset| cmd::M5ProgramRef {
        registers, rce_count, command_offset };
    let fragment_programs = cmd::M5FragmentProgramRefs {
        main: view(&workspace.fragment[..workspace.fragment_count],
            workspace.fragment_count as u16 - 4, 0xa0),
        main_mid_count: workspace.fragment_count as u16 - 2,
        store: view(store, partial_counts[0], 0x7c0),
        resume: view(resume, partial_counts[1], 0xee0),
        load: view(load, partial_counts[2], 0x1600),
    };
    let ta = &mut workspace.encoded_ta;
    let fragment = &mut workspace.encoded_fragment;
    ta.fill(0); fragment.fill(0);
    let fragment_rce = cmd::encode_m5_3d_command(fragment, fragment_low, inputs, &fragment_programs).map_err(|_| EINVAL)?;
    let ta_rce = cmd::encode_m5_ta_command(ta, low, inputs, &ta_program).map_err(|_| EINVAL)?;
    let (ta_storage, fragment_storage) = storage.split_at_mut(FRAGMENT_OFFSET);
    ta_storage[..cmd::M5_TA_COMMAND_BYTES].copy_from_slice(ta);
    fragment_storage[..cmd::M5_3D_COMMAND_BYTES].copy_from_slice(fragment);
    // A kick sets header bit 61 for a stage whose register program has the
    // SCS bracket and bit 63 for its matrix bracket, from the same flags that
    // gate those brackets. The fragment matrix bracket is never emitted here.
    let kick_flags = super::super::kick::StageKickFlags {
        scratch: [extra.scs_bracket & 1 != 0 || controls.ta.scs.scratch.enabled(),
            controls.fragment.scs.scratch.enabled()],
        matrix: [extra.matrix_bracket & 1 != 0, false],
    };
    Ok(EncodedRender { rce: [ta_rce, fragment_rce],
        register_counts: [count, workspace.fragment_count as u16, partial_counts[0], partial_counts[1], partial_counts[2]],
        kick_flags })
}
