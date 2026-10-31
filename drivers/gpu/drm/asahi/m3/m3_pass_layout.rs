// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Qualified G15/J514S sixteen-pass allocation contract. The strides describe
//! the existing fixed arenas; they are not permission to relocate or overlap
//! firmware-owned storage. Legacy unused allocations remain until separately
//! qualified for removal. No generated object indices appear in this layout.
pub(crate) const SLOTS:usize=16;
pub(crate) const RESET_COUNT:usize=32;
pub(crate) const COUNT:usize=34;
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
#[repr(usize)]
pub(crate) enum Field {
    FragmentStart,FragmentEnd,TilerStart,TilerEnd,
    FragmentUserStart,FragmentUserEnd,TilerUserStart,TilerUserEnd,
    Auxiliary,Preemption0,Preemption1,Preemption2,Tpc,Tilemap,HeapMetadata,
    LegacyClusterTilemap,LegacyClusterMetadata0,LegacyClusterMetadata1,
    LegacyClusterMetadata2,LegacyClusterMetadata3,LegacySceneUser,
    Scene,SceneUser,TilerToFragment,FragmentCommand,LegacyFragmentTail,
    FragmentSequence,FragmentScratch,TilerCommand,LegacyTilerTail,
    TilerSequence,TilerScratch,TilerDependency,FragmentDependency,
}
pub(crate) const FIELDS:[Field;COUNT]=[
    Field::FragmentStart,Field::FragmentEnd,Field::TilerStart,Field::TilerEnd,
    Field::FragmentUserStart,Field::FragmentUserEnd,Field::TilerUserStart,Field::TilerUserEnd,
    Field::Auxiliary,Field::Preemption0,Field::Preemption1,Field::Preemption2,Field::Tpc,Field::Tilemap,Field::HeapMetadata,
    Field::LegacyClusterTilemap,Field::LegacyClusterMetadata0,Field::LegacyClusterMetadata1,
    Field::LegacyClusterMetadata2,Field::LegacyClusterMetadata3,Field::LegacySceneUser,
    Field::Scene,Field::SceneUser,Field::TilerToFragment,Field::FragmentCommand,Field::LegacyFragmentTail,
    Field::FragmentSequence,Field::FragmentScratch,Field::TilerCommand,Field::LegacyTilerTail,
    Field::TilerSequence,Field::TilerScratch,Field::TilerDependency,Field::FragmentDependency,
];
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) enum Space {Firmware,ClientGpu}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) enum Access {GpuFirmwareCachedRw,GpuFirmwareUncachedRw,FirmwareUncachedRw,GpuCachedRw,GpuUncachedRw}
impl Access {
    /// Exact original PTE attributes, applied in the allocation's own space.
    /// Names follow pgtable.rs AP/UXN/PXN and MEMATTR definitions; bit 3 is
    /// the uncached memory attribute, not a read-only access flag.
    pub(crate) const fn pte(self)->u64 {
        match self {
            Self::GpuFirmwareCachedRw=>0xe0000000000000,
            Self::GpuFirmwareUncachedRw=>0xe0000000000008,
            Self::FirmwareUncachedRw=>0xc0000000000048,
            Self::GpuCachedRw=>0xc0000000000080,
            Self::GpuUncachedRw=>0xc0000000000088,
        }
    }
}
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) struct Allocation {
    pub(crate) address:u64,
    pub(crate) space:Space,
    pub(crate) size:usize,
    pub(crate) access:Access,
    /// Command GPU alias in the lower kernel root, additionally mapped in the
    /// active client VM. The command's canonical address remains firmware VA.
    pub(crate) gpu_alias:Option<u64>,
}
#[derive(Debug,PartialEq,Eq)]
pub(crate) enum Error {Slot,Address,Clusters}
/// GPU clusters of the qualified layout (G15S, two clusters).
pub(crate) const QUALIFIED_CLUSTERS:u32=2;
/// The most GPU clusters the layout holds: four (G15C, T6031). Only the three preemption buffers
/// grow with the clusters. Their start addresses stay the qualified ones, so with more than two
/// clusters each grows past the end of its 16 KiB page into the next page of the same pass slot:
/// slot `s` (stride 0x118000) holds, with four clusters,
/// - Preemption0: 0x1006083580 + s * 0x118000, 0x1500 bytes (pages 0x1006080000..0x1006088000);
/// - Preemption1: 0x100609bb00 + s * 0x118000, 0xa00 bytes (pages 0x1006098000..0x10060a0000);
/// - Preemption2: 0x10060b3f80 + s * 0x118000, 0x100 bytes (pages 0x10060b0000..0x10060b8000).
///
/// Each stays below the next field of its slot (Preemption1, Preemption2, TPC at 0x10060c8000),
/// and every pass slot stays inside the 128 MiB render-graph aperture at 0x1000000000 that
/// userspace reserves for the M3 runtime (`m3_runtime::Runtime::new_vm`), so no userspace
/// reservation changes with the cluster count.
pub(crate) const MAX_CLUSTERS:u32=4;
/// Per-cluster sizes of the three tiler preemption buffers: the qualified sizes (2688, 1280,
/// 128) are two clusters of the T6030 preempt1/2/3 sizes. The start addresses are kept, so
/// fewer clusters use a prefix of the same page.
pub(crate) const PREEMPTION_PER_CLUSTER:[usize;3]=[0x540,0x280,0x40];
// Two clusters give exactly the qualified T6030 sizes.
const _:()=assert!(PREEMPTION_PER_CLUSTER[0]*2==2688 && PREEMPTION_PER_CLUSTER[1]*2==1280
    && PREEMPTION_PER_CLUSTER[2]*2==128);
/// The qualified two-cluster allocation: [`board_allocation`] with [`QUALIFIED_CLUSTERS`].
#[allow(dead_code)]
pub(crate) fn allocation(slot:usize,field:Field)->Result<Allocation,Error> {
    board_allocation(slot,field,QUALIFIED_CLUSTERS)
}
/// Preserve all fixed virtual addresses, sizes, permission bits and aliases.
/// The caller controls allocation order and lifetime; the layout owns no memory.
/// Only the preemption buffers scale with the GPU cluster count (1 to [`MAX_CLUSTERS`]); every
/// other allocation is the qualified two-cluster one.
pub(crate) fn board_allocation(slot:usize,field:Field,clusters:u32)->Result<Allocation,Error> {
    use Field::*;
    use Space::*;
    use Access::*;
    if slot>=SLOTS {return Err(Error::Slot);}
    if clusters==0 || clusters>MAX_CLUSTERS {return Err(Error::Clusters);}
    let preemption=|i:usize|PREEMPTION_PER_CLUSTER[i]*clusters as usize;
    let (base,stride,size,space,access,alias)=match field {
        FragmentStart=>(0xfffffc20700f3ff8,0x74000,8,Firmware,GpuFirmwareUncachedRw,None),
        FragmentEnd=>(0xfffffc20700fbff8,0x74000,8,Firmware,GpuFirmwareUncachedRw,None),
        TilerStart=>(0xfffffc2070103ff8,0x74000,8,Firmware,GpuFirmwareUncachedRw,None),
        TilerEnd=>(0xfffffc207010bff8,0x74000,8,Firmware,GpuFirmwareUncachedRw,None),
        FragmentUserStart=>(0xfffffc2071003ff8,0x20000,8,Firmware,FirmwareUncachedRw,None),
        FragmentUserEnd=>(0xfffffc207100bff8,0x20000,8,Firmware,FirmwareUncachedRw,None),
        TilerUserStart=>(0xfffffc2071013ff8,0x20000,8,Firmware,FirmwareUncachedRw,None),
        TilerUserEnd=>(0xfffffc207101bff8,0x20000,8,Firmware,FirmwareUncachedRw,None),
        Auxiliary=>(0x100848a0000,0x98000,131072,ClientGpu,GpuUncachedRw,None),
        Preemption0=>(0x1006083580,0x118000,preemption(0),ClientGpu,GpuCachedRw,None),
        Preemption1=>(0x100609bb00,0x118000,preemption(1),ClientGpu,GpuCachedRw,None),
        Preemption2=>(0x10060b3f80,0x118000,preemption(2),ClientGpu,GpuCachedRw,None),
        Tpc=>(0x10060c8000,0x118000,262144,ClientGpu,GpuCachedRw,None),
        Tilemap=>(0x1006118000,0x118000,98304,ClientGpu,GpuCachedRw,None),
        HeapMetadata=>(0x1006143000,0x118000,4096,ClientGpu,GpuCachedRw,None),
        LegacyClusterTilemap=>(0x100848c8000,0x98000,196608,ClientGpu,GpuUncachedRw,None),
        LegacyClusterMetadata0=>(0x10084903ff8,0x98000,8,ClientGpu,GpuUncachedRw,None),
        LegacyClusterMetadata1=>(0x1008490b380,0x98000,3200,ClientGpu,GpuUncachedRw,None),
        LegacyClusterMetadata2=>(0x10084913b00,0x98000,1280,ClientGpu,GpuUncachedRw,None),
        LegacyClusterMetadata3=>(0x1008491bfa0,0x98000,96,ClientGpu,GpuUncachedRw,None),
        LegacySceneUser=>(0x10084920000,0x98000,65536,ClientGpu,GpuUncachedRw,None),
        Scene=>(0xfffffc2070113f80,0x74000,128,Firmware,GpuFirmwareUncachedRw,None),
        SceneUser=>(0x1006158000,0x118000,65536,ClientGpu,GpuCachedRw,None),
        TilerToFragment=>(0xfffffc200066ffc0,0xcc000,62,Firmware,FirmwareUncachedRw,None),
        FragmentCommand=>(0xfffffc207011b380,0x74000,3187,Firmware,GpuFirmwareUncachedRw,Some(0x7200003380)),
        LegacyFragmentTail=>(0xfffffc2070120000,0x74000,65536,Firmware,GpuFirmwareUncachedRw,None),
        FragmentSequence=>(0xfffffc20006b3000,0xcc000,4096,Firmware,FirmwareUncachedRw,None),
        FragmentScratch=>(0x1006178000,0x118000,65536,ClientGpu,GpuCachedRw,None),
        TilerCommand=>(0xfffffc20701376c0,0x74000,2347,Firmware,GpuFirmwareUncachedRw,Some(0x72000136c0)),
        LegacyTilerTail=>(0xfffffc207013c000,0x74000,65536,Firmware,GpuFirmwareUncachedRw,None),
        TilerSequence=>(0xfffffc20006f7000,0xcc000,4096,Firmware,FirmwareUncachedRw,None),
        TilerScratch=>(0xfffffc2070150000,0x74000,65536,Firmware,GpuFirmwareUncachedRw,None),
        TilerDependency=>(0xfffffc200132ffc0,0x44000,62,Firmware,FirmwareUncachedRw,None),
        FragmentDependency=>(0xfffffc200176ffc0,0x44000,62,Firmware,FirmwareUncachedRw,None),
    };
    let address=base+(slot as u64)*stride;
    let gpu_alias=alias.map(|base|base+(slot as u64)*0x20000);
    // Both roots must retain matching 16-KiB page offsets for coherent backing.
    if gpu_alias.is_some_and(|alias|alias&0x3fff!=address&0x3fff) {return Err(Error::Address);}
    Ok(Allocation {address,space,size,access,gpu_alias})
}

#[cfg(test)]
mod tests {
    use super::*;

    fn span(a:&Allocation)->(u64,u64) {(a.address,a.address+a.size as u64)}

    #[test]
    fn one_and_two_clusters_are_the_qualified_layout() {
        for slot in 0..SLOTS {
            for field in FIELDS {
                let two=board_allocation(slot,field,2).unwrap();
                assert_eq!(allocation(slot,field).unwrap(),two);
                let one=board_allocation(slot,field,1).unwrap();
                assert_eq!(one.address,two.address);
                match field {
                    Field::Preemption0|Field::Preemption1|Field::Preemption2=>assert_eq!(one.size*2,two.size),
                    _=>assert_eq!(one,two),
                }
            }
        }
        // The qualified sizes and the page ends they reach.
        let p=|f|board_allocation(0,f,2).unwrap();
        assert_eq!(span(&p(Field::Preemption0)),(0x1006083580,0x1006084000));
        assert_eq!(span(&p(Field::Preemption1)),(0x100609bb00,0x100609c000));
        assert_eq!(span(&p(Field::Preemption2)),(0x10060b3f80,0x10060b4000));
    }

    #[test]
    fn four_clusters_only_grow_the_preemption_buffers() {
        let p=|s,f|board_allocation(s,f,4).unwrap();
        assert_eq!(span(&p(0,Field::Preemption0)),(0x1006083580,0x1006084a80));
        assert_eq!(span(&p(0,Field::Preemption1)),(0x100609bb00,0x100609c500));
        assert_eq!(span(&p(0,Field::Preemption2)),(0x10060b3f80,0x10060b4080));
        assert_eq!(span(&p(15,Field::Preemption0)),(0x1006083580+15*0x118000,0x1006084a80+15*0x118000));
        for slot in 0..SLOTS {
            for field in FIELDS {
                let four=p(slot,field);
                let two=board_allocation(slot,field,2).unwrap();
                assert_eq!(four.address,two.address);
                assert_eq!(four.gpu_alias,two.gpu_alias);
                assert_eq!(four.access,two.access);
                match field {
                    Field::Preemption0|Field::Preemption1|Field::Preemption2=>assert_eq!(four.size,2*two.size),
                    _=>assert_eq!(four.size,two.size),
                }
            }
        }
        assert_eq!(board_allocation(0,Field::Tpc,5),Err(Error::Clusters));
        assert_eq!(board_allocation(0,Field::Tpc,0),Err(Error::Clusters));
    }

    #[test]
    fn four_cluster_client_pages_never_overlap() {
        // Every client-GPU allocation of every slot, as the 16 KiB pages it is mapped with.
        let mut pages=Vec::new();
        for slot in 0..SLOTS {
            for field in FIELDS {
                let a=board_allocation(slot,field,MAX_CLUSTERS).unwrap();
                if a.space!=Space::ClientGpu {continue;}
                let start=a.address&!0x3fff;
                let end=(a.address+a.size as u64+0x3fff)&!0x3fff;
                pages.push((start,end,slot,field));
            }
        }
        pages.sort_by_key(|p|(p.0,p.1));
        for w in pages.windows(2) {
            assert!(w[0].1<=w[1].0,"{:?} overlaps {:?}",w[0],w[1]);
        }
        // The render-graph slots stay inside the aperture userspace reserves.
        for &(start,end,_,field) in &pages {
            if (0x10_0000_0000..0x10_0800_0000).contains(&start) {
                assert!(end<=0x10_0800_0000,"{:?}",field);
            }
        }
    }
}

pub(crate) fn allocation_for(profile: super::m3_profile::Profile, slot:usize, field:Field, clusters:u32)
    -> Result<Allocation,Error> {
    let mut owner = board_allocation(slot,field,clusters)?;
    if profile == super::m3_profile::Profile::Legacy { return Ok(owner); }
    if owner.space == Space::Firmware {
        owner.address = 0xffff_fc25_0000_0000 + (slot * COUNT + field as usize) as u64 * 0x10_0000;
    }
    match field {
        Field::Scene => owner.size = 0x400,
        Field::TilerCommand | Field::FragmentCommand => {
            let fragment = field == Field::FragmentCommand;
            owner.size = if fragment {0xca0} else {0x940};
            owner.gpu_alias = Some(0x72_1000_0000 + slot as u64 * 0x4_0000
                + u64::from(fragment) * 0x4000);
        }
        Field::TilerToFragment | Field::TilerDependency | Field::FragmentDependency => owner.size = 0x40,
        _ => {}
    }
    Ok(owner)
}

#[cfg(test)]
mod extended_tests {
    use super::*;
    #[test] fn extended_command_aliases_keep_whole_disjoint_pages() {
        for slot in 0..SLOTS {
            for field in [Field::TilerCommand,Field::FragmentCommand] {
                let owner = allocation_for(super::super::m3_profile::Profile::Extended,slot,field,1).unwrap();
                assert_eq!(owner.address & 0x3fff,0);
                assert_eq!(owner.gpu_alias.unwrap() & 0x3fff,0);
                assert!(owner.size <= 0x4000);
            }
        }
    }
}
