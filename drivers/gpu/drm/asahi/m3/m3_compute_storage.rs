// SPDX-License-Identifier: GPL-2.0-only OR MIT
pub(crate) const COUNT:usize=50;
pub(crate) const LEGACY_PAGE:usize=0;
pub(crate) const POOL_BACKING:core::ops::Range<usize>=1..23;
pub(crate) const POOL_PAGES:usize=23;
pub(crate) const POOL_TABLE:usize=24;
pub(crate) const POOL:usize=25;
pub(crate) const COUNTER:usize=26;
pub(crate) const GPU_CONTEXT:usize=27;
pub(crate) const NOTIFIER:usize=28;
pub(crate) const EVENT:usize=29;
pub(crate) const JOB_LIST:usize=30;
pub(crate) const QUEUE:usize=31;
pub(crate) const QUEUE_STATE:usize=32;
pub(crate) const RING:usize=33;
pub(crate) const QUEUE_SCRATCH:usize=34;
pub(crate) const STAMP:usize=35;
pub(crate) const FW_STAMP:usize=36;
pub(crate) const TIMESTAMPS:[usize;2]=[37,38];
pub(crate) const PREEMPTION:usize=41;
pub(crate) const COMMAND:usize=42;
pub(crate) const TAIL_SCRATCH:usize=43;
pub(crate) const MICROSEQUENCE:usize=44;
pub(crate) const MICROSEQUENCE_SCRATCH:[usize;5]=[45,46,47,48,49];
// These owners alone reset after exact retirement; allocator/queue/notifier
// state survives. Preemption retains the qualified 0xcc initialization pattern.
pub(crate) const RESET:core::ops::Range<usize>=37..COUNT;
/// Each queued command owns its mutable timestamps, preemption, command,
/// microsequence and scratch. Only retired batches may reuse these slots.
pub(crate) const SLOTS:usize=16;
pub(crate) fn slot_offset(index:usize,slot:usize)->Result<usize,InvalidOwner> {
    let a=allocation(index)?;
    if slot>=SLOTS {return Err(InvalidOwner);}
    Ok(if RESET.contains(&index) {((a.size+0x3fff)&!0x3fff)*slot} else {0})
}
pub(crate) fn batch_allocation(index:usize)->Result<Allocation,InvalidOwner> {
    let mut a=allocation(index)?;
    a.size+=slot_offset(index,SLOTS-1)?;
    Ok(a)
}
const CLIENT_BASE:u64=0x71_0000_0000;
const POOL_STRIDE:u64=0x404000; // Four MiB plus a 16-KiB guard.
const POOL_TABLE_VA:u64=CLIENT_BASE+23*POOL_STRIDE+0x3000;
const PREEMPTION_VA:u64=CLIENT_BASE+23*POOL_STRIDE+0x8000;
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) enum Mapping {
    FirmwareOnly,
    FirmwareGpuShared,
    Client(u64),
    CommandAlias(u64),
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) struct Allocation {pub(crate) size:usize,pub(crate) mapping:Mapping}
#[derive(Debug,PartialEq,Eq)]
pub(crate) struct InvalidOwner;
pub(crate) fn allocation(index:usize)->Result<Allocation,InvalidOwner> {
    use Mapping::*;
    let (size,mapping)=match index {
        LEGACY_PAGE=>(0x4000,FirmwareGpuShared),
        1..=POOL_PAGES=>(0x400000,Client(CLIENT_BASE+(index-1) as u64*POOL_STRIDE)),
        POOL_TABLE=>(0x1000,Client(POOL_TABLE_VA)),
        POOL=>(132,FirmwareGpuShared),
        COUNTER=>(64,FirmwareGpuShared),
        GPU_CONTEXT=>(64,FirmwareOnly),
        NOTIFIER=>(264,FirmwareOnly),
        EVENT=>(4,FirmwareOnly),
        JOB_LIST=>(24,FirmwareOnly),
        QUEUE=>(180,FirmwareOnly),
        QUEUE_STATE=>(96,FirmwareOnly),
        RING=>(10240,FirmwareOnly),
        QUEUE_SCRATCH=>(262144,FirmwareGpuShared),
        STAMP|FW_STAMP=>(4,FirmwareGpuShared),
        37|38=>(8,FirmwareGpuShared),
        39|40=>(8,FirmwareOnly),
        PREEMPTION=>(65536,Client(PREEMPTION_VA)),
        COMMAND=>(2195,CommandAlias(0x73_0000_3760)),
        TAIL_SCRATCH=>(65536,FirmwareGpuShared),
        MICROSEQUENCE=>(4096,FirmwareOnly),
        45..=49=>(65536,FirmwareGpuShared),
        _=>return Err(InvalidOwner),
    };
    Ok(Allocation{size,mapping})
}

/// Firmware timestamp writes follow the GPU visibility barrier and therefore
/// retain their own host flush. Every batch ends with an unconditional flush.
pub(crate) fn coalesce_stamp_flush(wide:bool,timestamps:[u64;2],last:bool)->bool {
    wide && timestamps==[0,0] && !last
}

pub(crate) fn allocation_for(profile: super::m3_profile::Profile, index: usize)
    -> Result<Allocation, InvalidOwner> {
    let mut owner = allocation(index)?;
    if profile == super::m3_profile::Profile::Legacy { return Ok(owner); }
    match index {
        // The initial allocator has eight 4-MiB payload owners. Retain the
        // historical indices without allocating unused 4-MiB payloads.
        9..=22 => { owner.size = 0x4000; owner.mapping = Mapping::FirmwareOnly; }
        POOL_PAGES => owner.size = 0x20000,
        POOL_TABLE => { owner.size = 0x4000; owner.mapping = Mapping::Client(POOL_TABLE_VA & !0x3fff); }
        POOL => owner.size = 0xa0,
        GPU_CONTEXT => owner.size = 0x38,
        NOTIFIER => owner.size = 0x200,
        EVENT => owner.size = 8,
        QUEUE => owner.size = 0x24c0,
        PREEMPTION => owner.size = 0x4000,
        COMMAND => { owner.size = 0x8b0; owner.mapping = Mapping::CommandAlias(0x73_0000_0000); }
        TAIL_SCRATCH => { owner.size = 0x4000; owner.mapping = Mapping::Client(0x72_0100_0000); }
        46 => { owner.size = 0x40000; owner.mapping = Mapping::Client(0x72_0000_0000); }
        45 | 47..=49 => { owner.size = 0x4000; owner.mapping = Mapping::FirmwareOnly; }
        _ => {}
    }
    Ok(owner)
}

pub(crate) fn slot_offset_for(profile: super::m3_profile::Profile, index: usize, slot: usize)
    -> Result<usize, InvalidOwner> {
    if slot >= SLOTS { return Err(InvalidOwner); }
    let size = allocation_for(profile, index)?.size;
    Ok(if RESET.contains(&index) { ((size + 0x3fff) & !0x3fff) * slot } else { 0 })
}

pub(crate) fn batch_allocation_for(profile: super::m3_profile::Profile, index: usize)
    -> Result<Allocation, InvalidOwner> {
    let mut owner = allocation_for(profile, index)?;
    owner.size += slot_offset_for(profile, index, SLOTS - 1)?;
    Ok(owner)
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    use super::super::m3_profile::Profile;
    #[test] fn extended_commands_do_not_straddle_another_slots_page() {
        let command = allocation_for(Profile::Extended, COMMAND).unwrap();
        let Mapping::CommandAlias(base) = command.mapping else { panic!("missing command alias") };
        assert_eq!(base & 0x3fff, 0);
        for slot in 0..SLOTS - 1 {
            let start = slot_offset_for(Profile::Extended, COMMAND, slot).unwrap();
            let next = slot_offset_for(Profile::Extended, COMMAND, slot + 1).unwrap();
            assert!(start + command.size <= next);
        }
        assert_eq!(batch_allocation_for(Profile::Extended, QUEUE).unwrap().size, 0x24c0);
        assert!(slot_offset_for(Profile::Extended, COMMAND, SLOTS).is_err());
    }
    #[test] fn legacy_plan_is_byte_for_byte_the_same_for_every_owner_and_slot() {
        for index in 0..COUNT {
            assert_eq!(allocation_for(Profile::Legacy, index), allocation(index));
            for slot in 0..SLOTS {
                assert_eq!(slot_offset_for(Profile::Legacy, index, slot), slot_offset(index, slot));
            }
            assert_eq!(batch_allocation_for(Profile::Legacy, index), batch_allocation(index));
        }
    }
}
