// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! M5 meanings of fields in the shared channel configuration record.
use kernel::prelude::*;
use super::super::queue::{Policy,QueueConfig,QueueConfigArgs,QueueConfigTarget};

/// The firmware's channel priority profiles, including the independent QoS
/// weight selected at queue registration. A single owner has a full share.
pub(crate) fn scheduling_policy(priority: u32) -> Result<Policy> {
    match priority {
        0 => Ok(Policy::new(0, 1, 15, 8192)),
        1 => Ok(Policy::new(1, 0, 20, 8192)),
        2 => Ok(Policy::new(2, 2, 4, 8192)),
        3 => Ok(Policy::new(3, 0, 20, 8192)),
        _ => Err(EINVAL),
    }
}

/// M5's channel-state record. The scheduler pointer sits earlier than in
/// the Neo record; the policy and owner fields also moved.
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub(crate) struct QueueRecord {
    pointers_va: u64,
    items_va: u64,
    job_list_va: u64,
    reserved_18: [u8; 0xc],
    sentinel_24: u32,
    priority: [u32; 2],
    reserved_30: [u8; 6],
    sentinel_36: u16,
    reserved_38: u32,
    policy: u32,
    sentinel_40: u32,
    owner_pid: u32,
    reserved_48: [u8; 0x28],
    scheduler_va: u64,
    reserved_78: [u8; 8],
}
impl QueueRecord {
    fn scheduling_words(policy: Policy) -> Result<([u8; 6], u16, u32)> {
        match (policy.priority(), policy.policy()) {
            (0, 1) => Ok(([0, 0, 0xff, 0xff, 0xff, 0xff], u16::MAX, 1)),
            (1, 0) => Ok(([0, 0, 0, 0, 0xff, 0xff], u16::MAX, 0)),
            (2, 2) => Ok(([0; 6], u16::MAX, 0)),
            (3, 0) => Ok(([0; 6], 0, 0)),
            _ => Err(EINVAL),
        }
    }
    /// The scheduling words follow the queue's priority class.
    pub(crate) fn new(args: &super::super::queue::QueueRecordArgs) -> Result<Self> {
        let (reserved_30, sentinel, reserved_38) = Self::scheduling_words(args.policy)?;
        Ok(Self {
            pointers_va: args.pointers_va, items_va: args.items_va,
            job_list_va: args.job_list_va, reserved_18: [0; 0xc],
            sentinel_24: u32::MAX, priority: [u32::from(args.policy.priority()); 2], reserved_30,
            sentinel_36: sentinel, reserved_38, policy: u32::from(args.policy.policy()),
            sentinel_40: u32::MAX, owner_pid: args.owner_pid,
            reserved_48: [0; 0x28], scheduler_va: args.scheduler_va,
            reserved_78: [0; 8],
        })
    }
    /// Every host-owned identity word already holds what set_compute_owner
    /// would store (so that update would not change the record).
    pub(crate) fn names_owner(&self, work_head: u64, scheduler: u64,
        owner_pid: u32, policy: super::super::queue::Policy) -> Result<bool> {
        let (reserved_30, sentinel, reserved_38) = Self::scheduling_words(policy)?;
        let (job_list, scheduler_va, pid, priority) =
            (self.job_list_va, self.scheduler_va, self.owner_pid, self.priority);
        let (r30, s36, r38, current) = (self.reserved_30, self.sentinel_36, self.reserved_38, self.policy);
        Ok(job_list == work_head && scheduler_va == scheduler && pid == owner_pid
            && priority == [u32::from(policy.priority()); 2] && r30 == reserved_30
            && s36 == sentinel && r38 == reserved_38 && current == u32::from(policy.policy()))
    }
    /// Replace only host-owned identity fields after the old owner retires.
    pub(crate) fn set_compute_owner(&mut self, work_head: u64, scheduler: u64,
        owner_pid: u32, policy: super::super::queue::Policy) -> Result {
        let (reserved_30, sentinel, reserved_38) = Self::scheduling_words(policy)?;
        // Changing a channel's priority invalidates this word when its hardware
        // class changes. Retained physical queues can change class on reuse.
        if self.priority[0] != u32::from(policy.priority()) {
            self.sentinel_40 = u32::MAX;
        }
        self.job_list_va = work_head;
        self.scheduler_va = scheduler;
        self.owner_pid = owner_pid;
        self.priority = [u32::from(policy.priority()); 2];
        self.reserved_30 = reserved_30;
        self.sentinel_36 = sentinel;
        self.reserved_38 = reserved_38;
        self.policy = u32::from(policy.policy());
        Ok(())
    }
}
const _: () = {
    assert!(core::mem::size_of::<QueueRecord>() == 0x80);
    assert!(core::mem::offset_of!(QueueRecord, policy) == 0x3c);
    assert!(core::mem::offset_of!(QueueRecord, scheduler_va) == 0x70);
};

pub(crate) struct ConfigProfile {
    pub(crate) soft_fault_enabled:bool,
    pub(crate) extended:bool,
    pub(crate) parameter_manager:Option<u32>,
}
pub(crate) fn config(args:&QueueConfigArgs,p:ConfigProfile)->Result<QueueConfig> {
    if p.parameter_manager.is_some() && !matches!(args.target,QueueConfigTarget::Tiling(Some(_))) {
        return Err(EINVAL);
    }
    let mut out=QueueConfig::new(args)?;
    out.unk_66=u16::from(p.soft_fault_enabled);
    out.unk_68[2..6].copy_from_slice(&u32::from(p.extended).to_le_bytes());
    if let Some(manager)=p.parameter_manager {out.fragment_qid=manager;}
    Ok(out)
}

/// M5 publishes the current kick with the common tag-16 wire shape. All
/// reserved bytes are zero; the descriptor is reached through the shared ring's entry.
#[repr(C, packed)]
#[derive(Clone, Copy)]
pub(crate) struct Notification {
    record: super::super::queue::KickPredecessor,
    reserved: [u8; 6],
}
pub(crate) fn notification(qid:u8, current:super::super::kick::KickTimestamp)->Result<Notification> {
    Ok(Notification{record:super::super::queue::KickPredecessor::new(qid,current)?,reserved:[0;6]})
}
const _: () = assert!(core::mem::size_of::<Notification>() == 24);

/// Host-port transaction for an M5 render pair. The FIFO is a register pair
/// with a 16-byte stride, selecting slot 9. Queue validity
/// is asserted high then low, after its configuration payload. An existing
/// configuration is invalidated in the same order before replacement.
#[derive(Clone, Copy)]
pub(crate) struct HardwareSubmission {
    writes: [(usize,u64);16],
    count: usize,
}
impl HardwareSubmission {
    pub(crate) fn advance_pair(fragment:u8,tiling:u8)->Result<Self> {
        if fragment==tiling {return Err(EINVAL);}
        let mut writes=[(0,0);16];
        for (n,qid) in [fragment,tiling].into_iter().enumerate() {
            writes[n*2..n*2+2].copy_from_slice(&[
                (0x4090,Self::configuration_tag(qid)?+0x10),(0x4098,1)]);
        }
        Ok(Self{writes,count:4})
    }
    /// Configure the compute queue at first use, then publish only new kicks.
    pub(crate) fn compute(qid:u8,va:u64,priority:u8,configure:bool)->Result<Self> {
        use super::super::queue::{Policy,DataMaster};
        const ADDRESS_MASK:u64=0x0000_07ff_ffff_ffe0;
        let tag=Self::configuration_tag(qid)?;
        if priority>=Policy::PRIORITIES || va & !ADDRESS_MASK != 0 {return Err(EINVAL);}
        let mut writes=[(0,0);16];
        let mut count=0;
        if configure {
            let (low,high)=if qid<64 {(1u64<<qid,0)}else{(0,1u64<<(qid-64))};
            let completion=(3*(4-u64::from(priority))+0x3fe).wrapping_shl(54);
            let payload=((DataMaster::Compute as u64+1)<<60)|va|completion;
            writes[..6].copy_from_slice(&[(0x21080,high),(0x21078,low),
                (0x4090,tag),(0x4098,payload),(0x21070,high),(0x21068,low)]);
            count=6;
        }
        writes[count..count+2].copy_from_slice(&[(0x4090,tag+0x10),(0x4098,1)]);
        count+=2;
        Ok(Self{writes,count})
    }

    pub(crate) fn new(fragment:(u8,u64),tiling:(u8,u64),priority:u8,configured:bool)->Result<Self> {
        use super::super::{kick::QID_MAX,queue::{Policy,DataMaster}};
        const ADDRESS_MASK:u64=0x0000_07ff_ffff_ffe0;
        if fragment.0==tiling.0 || priority>=Policy::PRIORITIES {return Err(EINVAL);}
        let mut writes=[(0,0);16];
        let mut count=0;
        for ((qid,va),engine) in [(fragment,DataMaster::Fragment),(tiling,DataMaster::Tiling)] {
            if qid>QID_MAX || va & !ADDRESS_MASK != 0 {return Err(EINVAL);}
            let completion=(3*(4-u64::from(priority))+0x3fe).wrapping_shl(54);
            let tag=0x0020_0800_0002_1008 | (u64::from(qid)<<44);
            let payload=((engine as u64+1)<<60)|va|completion;
            let (low,high)=if qid<64 {(1u64<<qid,0)}else{(0,1u64<<(qid-64))};
            if configured {
                writes[count..count+2].copy_from_slice(&[(0x21080,high),(0x21078,low)]);
                count+=2;
            }
            writes[count..count+4].copy_from_slice(&[(0x4090,tag),(0x4098,payload),(0x21070,high),(0x21068,low)]);
            count+=4;
        }
        for (qid,_) in [fragment,tiling] {
            writes[count..count+2].copy_from_slice(&[(0x4090,Self::configuration_tag(qid)?+0x10),(0x4098,1)]);
            count+=2;
        }
        Ok(Self{writes,count})
    }
    pub(crate) fn configuration_tag(qid:u8)->Result<u64> {
        if qid>super::super::kick::QID_MAX {return Err(EINVAL);}
        Ok(0x0020_0800_0002_1008 | (u64::from(qid)<<44))
    }
    pub(crate) fn writes(&self)->&[(usize,u64)] {&self.writes[..self.count]}
}

/// M5's scheduler release uses the shared identity and pointer layout.
pub(crate) const SCHEDULER_RELEASE_OPCODE: u32 = 0x13;
