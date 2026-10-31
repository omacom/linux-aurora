// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Persistent G15/J514S render storage. These owners outlive every pass and
//! retain firmware allocator/queue state across submissions and VM switches.
//! Index values are private storage order, retained also as diagnostic labels.
//! Fixed arenas and permissions are the qualified contract, not generic G15 ABI.
use super::m3_pass_layout::{Allocation,Access,Space};
pub(crate) const COUNT:usize=560;
pub(crate) const GPU_CONTEXT:usize=24;
pub(crate) const NOTIFIER:usize=25;
pub(crate) const EVENT_COUNT:usize=26;
pub(crate) const JOB_LIST:usize=27;
pub(crate) const BLOCK_CONTROL:usize=28;
pub(crate) const BM_COUNTER:usize=29;
pub(crate) const MANAGER_MISC:usize=30;
pub(crate) const PARAMETER_PAGE_LIST:usize=31;
pub(crate) const PARAMETER_BLOCK_LIST:usize=32;
pub(crate) const BM_SCENES:usize=33;
pub(crate) const BUFFER_MANAGER:usize=34;
pub(crate) const PB_FIRST:usize=35;
pub(crate) const PB_GROUPS:usize=512;
pub(crate) const PB_BLOCKS_PER_GROUP:usize=4;
pub(crate) const PB_BLOCK_SIZE:usize=0x20000;
pub(crate) const PB_GROUP_SIZE:usize=PB_BLOCK_SIZE*PB_BLOCKS_PER_GROUP;
pub(crate) const PB_BASE:u64=0x78_0000_0000; // Runtime::new_vm reserved arena.
pub(crate) const LEGACY_EMPTY:usize=PB_FIRST+PB_GROUPS;
pub(crate) const FRAGMENT_QUEUE:usize=548;
pub(crate) const FRAGMENT_POINTERS:usize=549;
pub(crate) const FRAGMENT_RING:usize=550;
pub(crate) const FRAGMENT_SCRATCH:usize=551;
pub(crate) const TA_QUEUE:usize=552;
pub(crate) const TA_POINTERS:usize=553;
pub(crate) const TA_RING:usize=554;
pub(crate) const TA_SCRATCH:usize=555;
pub(crate) const TA_STAMP:usize=556;
pub(crate) const TA_FW_STAMP:usize=557;
pub(crate) const FRAGMENT_STAMP:usize=558;
pub(crate) const FRAGMENT_FW_STAMP:usize=559;
// Established initial completion domains; advance by 256 per retired draw.
pub(crate) const STAMP_TA:u32=0x7a000100;
pub(crate) const STAMP_FRAGMENT:u32=0x3d000100;
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) enum Error {Index}

/// Return the storage contract, with no initial byte image or implicit patch
/// table. Buffer::at_prot zeroes fresh backing; named constructors fill state.
/// Firmware sizes include packed tails; Buffer preserves the VA's 16-KiB
/// offset while rounding its backing allocation and mapping to whole pages.
pub(crate) fn allocation(index:usize)->Result<Allocation,Error> {
    use Access::*;use Space::*;
    let (address,size,space,access)=match index {
        // Two persistent UMA pools: eight 4-MiB backings plus page/table lists.
        0..=7=>(0x10080000000+index as u64*0x408000,0x400000,ClientGpu,GpuUncachedRw),
        8=>(0x10082040000,0x400000,ClientGpu,GpuUncachedRw),
        9=>(0x1008244b000,0x1000,ClientGpu,GpuUncachedRw),
        10=>(0xfffffc207001bf7c,132,Firmware,GpuFirmwareUncachedRw),
        11=>(0xfffffc2070023fc0,64,Firmware,GpuFirmwareUncachedRw),
        12..=19=>(0x10082450000+(index-12) as u64*0x408000,0x400000,ClientGpu,GpuUncachedRw),
        20=>(0x10084490000,0x400000,ClientGpu,GpuUncachedRw),
        21=>(0x1008489b000,0x1000,ClientGpu,GpuUncachedRw),
        22=>(0xfffffc207002bf7c,132,Firmware,GpuFirmwareUncachedRw),
        23=>(0xfffffc2070033fc0,64,Firmware,GpuFirmwareUncachedRw),
        GPU_CONTEXT=>(0xfffffc200040bfc0,64,Firmware,FirmwareUncachedRw),
        NOTIFIER=>(0xfffffc200044fef8,264,Firmware,FirmwareUncachedRw),
        EVENT_COUNT=>(0xfffffc2000493ffc,4,Firmware,FirmwareUncachedRw),
        JOB_LIST=>(0xfffffc204089bfe8,24,Firmware,FirmwareUncachedRw),
        BLOCK_CONTROL=>(0xfffffc20408dffc0,64,Firmware,FirmwareUncachedRw),
        BM_COUNTER=>(0xfffffc2040923fc0,64,Firmware,FirmwareUncachedRw),
        MANAGER_MISC=>(0xfffffc2040967fc0,64,Firmware,FirmwareUncachedRw),
        PARAMETER_PAGE_LIST=>(0x1074000000,163840,ClientGpu,GpuFirmwareCachedRw),
        PARAMETER_BLOCK_LIST=>(0x100e0000000,32768,ClientGpu,GpuFirmwareCachedRw),
        BM_SCENES=>(0xfffffc207003bf40,192,Firmware,GpuFirmwareUncachedRw),
        BUFFER_MANAGER=>(0xfffffc20004d7f54,172,Firmware,FirmwareUncachedRw),
        PB_FIRST..LEGACY_EMPTY=>(PB_BASE+(index-PB_FIRST) as u64*PB_GROUP_SIZE as u64,PB_GROUP_SIZE,ClientGpu,GpuCachedRw),
        // Unresolved legacy buffer: preserve allocation and zero contents.
        LEGACY_EMPTY=>(0xfffffc2070043fc0,64,Firmware,GpuFirmwareUncachedRw),
        FRAGMENT_QUEUE=>(0xfffffc200051bf4c,180,Firmware,FirmwareUncachedRw),
        FRAGMENT_POINTERS=>(0xfffffc20409abfa0,96,Firmware,FirmwareUncachedRw),
        FRAGMENT_RING=>(0xfffffc200055d800,10240,Firmware,FirmwareUncachedRw),
        FRAGMENT_SCRATCH=>(0xfffffc2070048000,262144,Firmware,GpuFirmwareUncachedRw),
        TA_QUEUE=>(0xfffffc20005a3f4c,180,Firmware,FirmwareUncachedRw),
        TA_POINTERS=>(0xfffffc20409effa0,96,Firmware,FirmwareUncachedRw),
        TA_RING=>(0xfffffc20005e5800,10240,Firmware,FirmwareUncachedRw),
        TA_SCRATCH=>(0xfffffc207008c000,262144,Firmware,GpuFirmwareUncachedRw),
        TA_STAMP=>(0xfffffc20700d3ffc,4,Firmware,GpuFirmwareUncachedRw),
        TA_FW_STAMP=>(0xfffffc20700dbffc,4,Firmware,GpuFirmwareUncachedRw),
        FRAGMENT_STAMP=>(0xfffffc20700e3ffc,4,Firmware,GpuFirmwareUncachedRw),
        FRAGMENT_FW_STAMP=>(0xfffffc20700ebffc,4,Firmware,GpuFirmwareUncachedRw),
        _=>return Err(Error::Index),
    };
    Ok(Allocation{address,size,space,access,gpu_alias:None})
}

// Historical labels remain stable for fault-log decoders. They do not index
// the persistent owners: Render resolves them to named pass-zero resources.
pub(crate) const TPC_LABEL:usize=572;
pub(crate) const TILEMAP_LABEL:usize=573;
pub(crate) const HEAP_METADATA_LABEL:usize=574;
pub(crate) fn fault_buffers()->impl Iterator<Item=(usize,usize)> {
    core::iter::once((PARAMETER_PAGE_LIST,256))
        .chain((PB_FIRST..PB_FIRST+PB_GROUPS).map(|i|(i,256)))
        .chain([(TILEMAP_LABEL,1280),(HEAP_METADATA_LABEL,256)])
}

pub(crate) fn allocation_for(profile: super::m3_profile::Profile, index: usize) -> Result<Allocation,Error> {
    let mut owner = allocation(index)?;
    if profile == super::m3_profile::Profile::Legacy { return Ok(owner); }
    if owner.space == Space::Firmware {
        owner.address = 0xffff_fc24_0000_0000 + index as u64 * 0x10_0000;
    }
    match index {
        8 | 20 => owner.size = 0x20000,
        9 | 21 => { owner.address &= !0x3fff; owner.size = 0x4000; }
        10 | 22 => {
            owner.size = 0x4000;
            owner.gpu_alias = Some(0x72_3000_0000 + u64::from(index == 22) * 0x4000);
        }
        GPU_CONTEXT => owner.size = 0x38,
        NOTIFIER => owner.size = 0x400,
        EVENT_COUNT => owner.size = 16,
        BUFFER_MANAGER => owner.size = 0xc0,
        FRAGMENT_QUEUE | TA_QUEUE => owner.size = 0x24c0,
        _ => {}
    }
    Ok(owner)
}
