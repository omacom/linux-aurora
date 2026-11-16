// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Safety checks for clients of the M3 (G15) render node.
//!
//! Client admission: the M3 GPU runs AGX3 shaders and command streams. Userspace written for
//! the AGX2 GPUs (G13/G14) reads the M3's parameters as those of a G14 and would submit AGX2
//! streams, which fault the GPU; the M3 backends cannot recover from a fault, so one such client
//! stops the GPU for the rest of the boot. Userspace that drives the M3 knows the AGX3 fields
//! of `drm_asahi_params_global` (`usc_generation`, `gpu_hal_generation`) and asks GET_PARAMS
//! for at least the structure up to them; older userspace asks for less. On G15 a file therefore
//! has to ask for those fields before it can create a VM. Every path to the GPU needs a VM, so a
//! refused client can still query the parameters but never reaches the hardware.

use core::sync::atomic::{
    AtomicU32,
    Ordering, //
};

use kernel::{
    prelude::*,
    uapi,
};

use crate::{
    m3::agx_uapi::{
        GpuAccess,
        GpuAddressSpace, //
    },
    mmu,
};

/// GET_PARAMS size an AGX3-aware client asks for: the global parameter structure up to the end
/// of `gpu_hal_generation`. The optional tail fields that follow it are not part of the check,
/// so growing the structure does not refuse userspace built against the shorter one.
const AGX3_PARAMS_SIZE: u64 =
    (core::mem::offset_of!(uapi::drm_asahi_params_global, gpu_hal_generation)
        + core::mem::size_of::<u32>()) as u64;
const _: () = assert!(AGX3_PARAMS_SIZE == 600);

/// Refused clients that are logged per boot; later refusals only count.
const REFUSALS_LOGGED: u32 = 16;
static REFUSALS: AtomicU32 = AtomicU32::new(0);

/// Whether a file may create a VM on the M3 runtime: it must have asked GET_PARAMS for at least
/// [`AGX3_PARAMS_SIZE`] bytes (so it knows this GPU runs AGX3 command streams), and pass
/// VM_CREATE flags 0 (legacy HAL). `size` is the largest GET_PARAMS size the file asked for.
pub(crate) fn admit_vm(device: &kernel::device::Device, size: u64, flags: u32) -> Result {
    {
        // The positive acknowledgement of the command ABI (7.1.12 `g16_profile::client_admitted`):
        // GET_PARAMS coverage, and VM_CREATE flags 0 on the legacy-HAL M3 runtime.
        if size >= AGX3_PARAMS_SIZE && flags == 0 {
            return Ok(());
        }
        let refusals = REFUSALS.fetch_add(1, Ordering::Relaxed).saturating_add(1);
        if refusals <= REFUSALS_LOGGED {
            let task = kernel::current!();
            let comm = task_comm(task);
            let len = comm.iter().position(|&b| b == 0).unwrap_or(comm.len());
            let name = core::str::from_utf8(&comm[..len]).unwrap_or("?");
            let last = if refusals == REFUSALS_LOGGED {
                " (further refusals are not logged)"
            } else {
                ""
            };
            dev_info!(
                device,
                "M3: refusing a VM for {}[{}]: it asked GET_PARAMS for {} of the {} parameter bytes, so it cannot know this GPU runs AGX3 command streams; M3-aware userspace is needed{}\n",
                name,
                task.group_leader().pid(),
                size,
                AGX3_PARAMS_SIZE,
                last
            );
        }
        Err(Error::from_errno(-(kernel::bindings::EOPNOTSUPP as i32)))
    }
}

/// The name of `task`, for log lines only.
pub(crate) fn task_comm(task: &kernel::task::Task) -> [u8; 16] {
    let mut name = [0u8; 16];
    // SAFETY: `comm` is a fixed-size array inside the live task. It is read byte by byte with
    // volatile loads because a concurrent rename may change it; a torn name is only printed.
    unsafe {
        let comm = core::ptr::addr_of!((*task.as_ptr()).comm).cast::<u8>();
        for (i, byte) in name.iter_mut().enumerate() {
            *byte = comm.add(i).read_volatile();
        }
    }
    name
}

/// Bytes at the start of a compute control stream whose launch records are checked.
const LAUNCH_WINDOW: usize = 4096;
/// Bytes that must be readable at a launch's program pointer. The pointer is 64-byte aligned,
/// so this never crosses a page.
const PROGRAM_BYTES: u64 = 64;
/// CDM record types (bits 31:29 of a record's first word).
const CDM_LAUNCH: u32 = 0;
const CDM_BARRIER: u32 = 3;

/// Refuse a compute command whose control stream starts with a launch the GPU cannot fetch.
///
/// Walks the records at the start of the stream: launches (40 bytes direct, 36 or 24 bytes
/// indirect, told apart by bits 28:27) and barriers (4 bytes). The walk ends at the first link,
/// return, terminator or unknown record, at the end of the stream and after `LAUNCH_WINDOW`
/// bytes. Each launch names its program at `(word1[31:22] << 38) | (word2 << 6)`; that address,
/// and the one read with only word2's low 26 bits when VA[37:32] is set, must be mapped
/// readable in the submitting VM, outside the driver's own mappings, or the command is refused
/// with EINVAL before it is queued. Without the check the GPU would take a translation fault on
/// the fetch, which the M3 backends cannot recover from.
///
/// This catches malformed or stale pointers, not a hostile client: userspace can rewrite the
/// stream after the check.
pub(crate) fn check_compute_launches<A: GpuAddressSpace>(
    vm: &mmu::Vm,
    space: &A,
    base: u64,
    end: u64,
) -> Result {
    let len = usize::try_from(end.checked_sub(base).ok_or(EINVAL)?)
        .unwrap_or(usize::MAX)
        .min(LAUNCH_WINDOW);
    let mut stream = KVec::from_elem(0u8, len, GFP_KERNEL)?;
    if let Err(error) = read_through_cache(vm, base, &mut stream) {
        pr_err!(
            "M3 compute rejected: control stream {:#x} cannot be read ({:?})\n",
            base,
            error
        );
        return Err(EINVAL);
    }
    let word = |offset: usize| {
        u32::from_le_bytes([
            stream[offset],
            stream[offset + 1],
            stream[offset + 2],
            stream[offset + 3],
        ])
    };
    let mut offset = 0;
    let mut launch = 0u32;
    while offset + 4 <= len {
        let head = word(offset);
        match head >> 29 {
            CDM_LAUNCH => {
                if offset + 12 > len {
                    break;
                }
                let high = u64::from(word(offset + 4) >> 22) << 38;
                let low = u64::from(word(offset + 8));
                // Whether the GPU takes VA[37:32] from word2's top six bits is not
                // established, so the program must be mapped under both readings. They
                // agree whenever those bits are clear, as in every working launch.
                for program in [high | (low << 6), high | ((low & 0x3ff_ffff) << 6)] {
                    if !space.covers(program, PROGRAM_BYTES, GpuAccess::Read) {
                        pr_err!(
                            "M3 compute rejected: launch {} at {:#x} names program {:#x}, which is not mapped readable in this VM\n",
                            launch,
                            base + offset as u64,
                            program
                        );
                        return Err(EINVAL);
                    }
                }
                launch += 1;
                offset += match (head >> 27) & 3 {
                    0 => 40,
                    1 => 36,
                    2 => 24,
                    _ => break,
                };
            }
            CDM_BARRIER => offset += 4,
            _ => break,
        }
    }
    Ok(())
}

/// Read `out.len()` bytes at `iova` in `vm` through the kernel's linear map.
///
/// Userspace normally writes command streams through a write-combined mapping, which does not
/// update cache lines the kernel's cacheable alias may hold from an earlier read or a
/// speculative fetch. Each line is cleaned and invalidated before it is read, so the bytes
/// come from memory, as the GPU will fetch them. Cleaning never discards data: a dirty line from
/// a cached mapping is written back first.
///
/// `Vm::read_mapped_bytes` holds the VM execution lock from translation until the copy, so the
/// pages cannot be unmapped and freed while they are cleaned and read.
fn read_through_cache(vm: &mmu::Vm, iova: u64, out: &mut [u8]) -> Result {
    // 7.2's `Vm::read_user` cleans and invalidates each line to the point of coherency before
    // the copy, under the VM execution lock.
    vm.read_user(iova, out)
}

/// Clean and invalidate the data cache lines covering `start..start + len` to the point of
/// coherency, completing before later loads.
fn clean_invalidate(start: usize, len: usize) {
    let ctr: u64;
    // SAFETY: CTR_EL0 is readable at EL1 and has no side effects.
    unsafe {
        core::arch::asm!("mrs {ctr}, ctr_el0", ctr = out(reg) ctr, options(nomem, nostack, preserves_flags));
    }
    // DminLine: log2 of the smallest data cache line, in 4-byte words.
    let line = 4usize << ((ctr >> 16) & 0xf);
    let end = start.saturating_add(len);
    let mut address = start & !(line - 1);
    // SAFETY: the range lies inside one page of the linear map that the caller has mapped.
    // Clean-and-invalidate writes dirty lines back before dropping them, so no data is lost.
    unsafe {
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
        while address < end {
            core::arch::asm!("dc civac, {address}", address = in(reg) address, options(nostack, preserves_flags));
            address += line;
        }
        core::arch::asm!("dsb sy", options(nostack, preserves_flags));
    }
}
