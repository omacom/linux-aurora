// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Stream admission gates (safety nets for userspace encoders).
//!
//! VDM: the root stream's first word is 0x21000960. CDM walk from `cdm_ctrl_stream_base` through
//! links until TERMINATE (0x40000000), at most 64 links and [`CDM_MAX_BYTES`] of records (a
//! longer or cyclic stream is rejected): barriers 0x60000160, 0x60001560, 0x600215fe, 0x600201fe;
//! link 0x2000XXXX; direct launch (40 B, the G15 layout), indirect launches 36 B
//! (`w0 & 0xff7fffff == 0x08080000`) and 24 B (0x10080000); every program VA inside the queue's
//! 4 GiB USC window, 64-aligned, first 64 bytes readable; an indirect launch's 12 argument bytes
//! readable. A program or argument page found good is not checked again in the same walk, and a
//! walk checks at most [`CDM_MAX_PAGE_CHECKS`] pages. Client memory is read through the VM's
//! page tables with a point-of-coherency clean+invalidate of the lines ([`mmu::Vm::read_user`]:
//! the GPU may have written it). `g15_stream_gate=0` disables both gates (encoder bring-up only).

use core::sync::atomic::{AtomicU32, Ordering};
use kernel::prelude::*;

use super::{command::cover, command::USC_WINDOW, params};
use crate::mmu;

const VDM_ROOT: u32 = 0x21000960;
const CDM_TERMINATE: u32 = 0x40000000;
const CDM_BARRIERS: [u32; 4] = [0x60000160, 0x600215fe, 0x60001560, 0x600201fe];
const CDM_LINK: u32 = 0x20000000;
const CDM_DIRECT: u32 = 0x00080000;
const CDM_INDIRECT_SIZES: u32 = 0x08080000;
const CDM_INDIRECT: u32 = 0x10080000;
const CDM_MAX_LINKS: u32 = 64;
/// The stream block the encoder links to when one fills (Mesa's 64 KiB control-stream blocks).
const CDM_BLOCK_BYTES: u64 = 0x10000;
/// Stream bytes walked at most: the root block and one block per admitted link. A stream of
/// that size holds every record the encoder can write before the link budget (a record count
/// does not: the 1024 tessellated draws of one multi-draw write more than 8192).
const CDM_MAX_BYTES: u64 = (CDM_MAX_LINKS as u64 + 1) * CDM_BLOCK_BYTES;
/// Page checks (a program's or an indirect launch's arguments, each under the VM lock) a walk
/// makes at most. The last page found good is not checked again, but the encoder's indirect
/// arguments move to a new page every few launches: a stream at the link budget makes about
/// 8100. Twice that leaves room; without a bound, a stream of distinct pages would make one per
/// 24-byte launch, about 177000 in the byte budget.
const CDM_MAX_PAGE_CHECKS: u32 = 16384;
/// Rejections logged per boot.
const MAX_REPORTS: u32 = 32;

static REPORTS: AtomicU32 = AtomicU32::new(0);

/// Logs a refused record and refuses it.
fn reject(what: &str, va: u64, w: &[u32]) -> Error {
    if REPORTS.fetch_add(1, Ordering::Relaxed) < MAX_REPORTS {
        let word = |i: usize| w.get(i).copied().unwrap_or(0);
        pr_warn!(
            "G15: rejecting {} record at {:#x}: {:08x} {:08x} {:08x} {:08x}\n",
            what,
            va,
            word(0),
            word(1),
            word(2),
            word(3)
        );
    }
    EINVAL
}

/// Up to 256 stream bytes read at once.
struct Window {
    va: u64,
    len: usize,
    bytes: [u8; 256],
}

impl Window {
    /// The `count` words at `va`, reading whole spans of up to 256 bytes (not past the page
    /// unless the record crosses it). Unreadable memory: EFAULT.
    fn words(&mut self, vm: &mut mmu::VmReadGuard, va: u64, count: usize) -> Result<[u32; 10]> {
        let n = count * 4;
        let cached = self.len != 0
            && va >= self.va
            && va
                .checked_add(n as u64)
                .is_some_and(|end| end <= self.va + self.len as u64);
        if !cached {
            let in_page = mmu::UAT_PGSZ - (va as usize & mmu::UAT_PGMSK);
            let len = n.max(self.bytes.len().min(in_page));
            self.len = 0;
            vm.read_user(va, &mut self.bytes[..len])
                .map_err(|_| EFAULT)?;
            self.va = va;
            self.len = len;
        }
        let start = (va - self.va) as usize;
        let mut words = [0u32; 10];
        for (index, word) in words.iter_mut().take(count).enumerate() {
            let at = start + index * 4;
            let mut le = [0u8; 4];
            le.copy_from_slice(&self.bytes[at..at + 4]);
            *word = u32::from_le_bytes(le);
        }
        Ok(words)
    }
}

/// The pages a walk has found good and the page checks it has made.
struct Checked {
    program: Option<u64>,
    args: Option<u64>,
    count: u32,
}

/// One page check of `[start, end)` under the walk's page budget: ENOSPC once the budget is
/// spent.
fn check(vm: &mut mmu::VmReadGuard, start: u64, end: u64, count: &mut u32) -> Result {
    if *count >= CDM_MAX_PAGE_CHECKS {
        return Err(ENOSPC);
    }
    *count += 1;
    cover(vm, start, end, true, false)
}

/// A launch's program: inside the queue's 4 GiB USC window, 64-aligned, first block readable.
/// The last proven program page is not checked again.
fn program(vm: &mut mmu::VmReadGuard, usc: u64, address: u64, checked: &mut Checked) -> Result {
    if address < usc || address - usc >= USC_WINDOW || address & 63 != 0 {
        return Err(EINVAL);
    }
    // 64-aligned: the block never crosses its page.
    let page = address & !(mmu::UAT_PGMSK as u64);
    if checked.program == Some(page) {
        return Ok(());
    }
    check(vm, address, address + 64, &mut checked.count)?;
    checked.program = Some(page);
    Ok(())
}

/// An indirect launch's 12 argument bytes: readable. The last proven argument page is not
/// checked again; arguments that cross a page are checked each time.
fn arguments(vm: &mut mmu::VmReadGuard, address: u64, checked: &mut Checked) -> Result {
    let end = address.checked_add(12).ok_or(EINVAL)?;
    let page = address & !(mmu::UAT_PGMSK as u64);
    let within = (end - 1) & !(mmu::UAT_PGMSK as u64) == page;
    if within && checked.args == Some(page) {
        return Ok(());
    }
    check(vm, address, end, &mut checked.count)?;
    if within {
        checked.args = Some(page);
    }
    Ok(())
}

/// VDM admission: a normal root stream opens with exactly the G15 barrier.
pub(crate) fn vdm(vm: &mut mmu::VmReadGuard, va: u64) -> Result {
    if !params::stream_gate() {
        return Ok(());
    }
    let mut bytes = [0u8; 4];
    vm.read_user(va, &mut bytes).map_err(|_| EFAULT)?;
    let word = u32::from_le_bytes(bytes);
    if word != VDM_ROOT {
        return Err(reject("VDM root", va, &[word]));
    }
    Ok(())
}

/// CDM admission: walks the stream from its base through no-return links until TERMINATE,
/// admitting only the known G15S records: direct launch (40), indirect launch (36/24), barriers
/// (4), link (8).
pub(crate) fn cdm(vm: &mut mmu::VmReadGuard, usc: u64, base: u64) -> Result {
    if !params::stream_gate() {
        return Ok(());
    }
    let mut window = Window {
        va: 0,
        len: 0,
        bytes: [0; 256],
    };
    let mut checked = Checked {
        program: None,
        args: None,
        count: 0,
    };
    let mut va = base;
    let mut links = 0;
    let mut walked = 0;
    while walked < CDM_MAX_BYTES {
        let w = window.words(vm, va, 1)?;
        if CDM_BARRIERS.contains(&w[0]) {
            va += 4;
            walked += 4;
            continue;
        }
        if w[0] == CDM_TERMINATE {
            return Ok(());
        }
        if w[0] & 0xffff0000 == CDM_LINK {
            let w = window.words(vm, va, 2)?;
            links += 1;
            if links > CDM_MAX_LINKS {
                return Err(reject("CDM link budget", va, &w));
            }
            va = (u64::from(w[0] & 0xffff) << 32) | u64::from(w[1]);
            if va & 3 != 0 {
                return Err(reject("CDM link", va, &w));
            }
            walked += 8;
            continue;
        }
        if w[0] == CDM_DIRECT {
            let w = window.words(vm, va, 10)?;
            let address = (u64::from(w[1] >> 16) << 32) | (u64::from(w[2]) << 6);
            if w[1] & 0xffff != 0 || w[3] != 0x40 || (w[2] >> 26) != ((w[1] >> 16) & 0x3f) {
                return Err(reject("CDM direct launch", va, &w));
            }
            match program(vm, usc, address, &mut checked) {
                Err(error) if error == EINVAL => return Err(reject("CDM program", va, &w)),
                Err(error) if error == ENOSPC => {
                    return Err(reject("CDM page check budget", va, &w))
                }
                result => result?,
            }
            va += 40;
            walked += 40;
            continue;
        }
        let kind = w[0] & 0xff7fffff;
        if kind == CDM_INDIRECT_SIZES || kind == CDM_INDIRECT {
            let sizes = kind == CDM_INDIRECT_SIZES;
            let w = window.words(vm, va, if sizes { 9 } else { 6 })?;
            let address = (u64::from(w[1] >> 22) << 38) | (u64::from(w[2]) << 6);
            let indirect = (u64::from(w[4] & 0xffff) << 32) | u64::from(w[5]);
            let bad_size = |size: u32| size == 0 || size > 0x7ff;
            if w[3] != (0x40000000 | ((address >> 40) & 0xff) as u32)
                || w[4] >> 16 != 0
                || indirect & 3 != 0
                || (sizes && (bad_size(w[6]) || bad_size(w[7]) || bad_size(w[8])))
            {
                return Err(reject("CDM indirect launch", va, &w));
            }
            let result = program(vm, usc, address, &mut checked)
                .and_then(|()| arguments(vm, indirect, &mut checked));
            match result {
                Err(error) if error == EINVAL => return Err(reject("CDM indirect", va, &w)),
                Err(error) if error == ENOSPC => {
                    return Err(reject("CDM page check budget", va, &w))
                }
                result => result?,
            }
            let size = if sizes { 36 } else { 24 };
            va += size;
            walked += size;
            continue;
        }
        return Err(reject("CDM", va, &w));
    }
    Err(reject("CDM byte budget", va, &[]))
}
