// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! G15S / RTKit 2419 initialization storage layout.
//! Config owns every DMA object and MMIO mapping until firmware stops. Fixed
//! VAs, packed offsets and permissions follow the qualified setup. Owners marked
//! Hardware, Globals or Power receive the images m3_adt_config generates from the
//! device tree; every other owner starts zeroed.
use crate::m3_init_layout::{Error,Region,fwlog};
pub(crate) const COUNT:usize=49;
pub(crate) const ROOT:usize=36;
pub(crate) const REGION_A:usize=37;
pub(crate) const RUNTIME_POINTERS:usize=38;
pub(crate) const HARDWARE_DATA:usize=39;
pub(crate) const UNKNOWN_PAIR:usize=40;
pub(crate) const FWLOG_PAYLOAD:usize=41;
pub(crate) const UNKNOWN_C0:usize=42;
pub(crate) const UNKNOWN_C1:usize=43;
pub(crate) const UNKNOWN_C3:usize=44;
pub(crate) const GLOBALS:usize=45;
pub(crate) const GLOBALS_POWER:usize=46;
pub(crate) const CONTROL_REGION:usize=47;
pub(crate) const RUNTIME_FLAGS:usize=48;
pub(crate) const DEVICE_CONTROL:usize=12;
pub(crate) const EVENT:usize=13;
pub(crate) const FIRMWARE_LOG:usize=14;
pub(crate) const TRACE:usize=15;
pub(crate) const STATS:usize=16;
pub(crate) const FW_CONTROL_STATE:usize=34;
pub(crate) const FW_CONTROL_RING:usize=35;
/// Initial contents of an owner: zeroed, or one of the three images generated
/// from the device tree (m3_adt_config). Driver-owned references are filled
/// before firmware boot.
#[derive(Clone,Copy,Debug,PartialEq,Eq)]
pub(crate) enum Initial {Zero,Hardware,Globals,Power}
#[derive(Clone,Copy,Debug)]
pub(crate) struct Allocation {
    pub(crate) address:u64,pub(crate) size:usize,
    /// Additional GPU access to the firmware-root mapping; no client GPU alias.
    pub(crate) gpu_shared:bool,pub(crate) initial:Initial,
}
pub(crate) fn allocation(index:usize)->Result<Allocation,Error> {
    use Initial::*;
    let (address,size,gpu_shared,initial)=match index {
        // Twelve host submission channels: TA/fragment/compute at four priorities.
        0..=23=>if index%2==0 {
            (0xfffffc2040003fd0+(index/2) as u64*0x44000,48,false,Zero)
        }else{(0xfffffc2000002800+(index/2) as u64*0x44000,6144,false,Zero)},
        24=>(0xfffffc2040333fd0,48,false,Zero), // Device control state/ring.
        25=>(0xfffffc2000330800,14336,false,Zero),
        26=>(0xfffffc2040377fd0,48,false,Zero), // Events.
        27=>(0xfffffc20403b8800,14336,false,Zero),
        28=>(0xfffffc20403ffee0,288,false,Zero), // Six firmware-log channels.
        29=>(0xfffffc2040443000,fwlog::RING_BYTES,false,Zero),
        30=>(0xfffffc20404d7fd0,48,false,Zero), // Trace.
        31=>(0xfffffc2040519000,28672,false,Zero),
        32=>(0xfffffc2040563fd0,48,false,Zero), // Statistics.
        33=>(0xfffffc20405a4000,16384,false,Zero),
        FW_CONTROL_STATE=>(0xfffffc20405ebfd0,48,false,Zero),
        FW_CONTROL_RING=>(0xfffffc2000376c00,5120,false,Zero),
        ROOT=>(0xfffffc204062ff40,192,false,Zero),
        REGION_A=>(0xfffffc2040670000,16384,false,Zero),
        RUNTIME_POINTERS=>(0xfffffc20406b7b4d,1203,false,Zero),
        HARDWARE_DATA=>(0xfffffc20406fb5fc,35332,false,Hardware),
        UNKNOWN_PAIR=>(0xfffffc2040747f00,256,false,Zero),
        // Keep the log payload above the fixed render control/queue pages.
        FWLOG_PAYLOAD=>(0xfffffc2040a40000,fwlog::PAYLOAD_BYTES,false,Zero),
        UNKNOWN_C0=>(0xfffffc2070003000,4096,true,Zero),
        UNKNOWN_C1=>(0xfffffc2070008000,16384,true,Zero),
        UNKNOWN_C3=>(0xfffffc2070010000,16384,true,Zero),
        GLOBALS=>(0xfffffc20407cc204,15868,false,Globals),
        GLOBALS_POWER=>(0xfffffc204081370c,2292,false,Power),
        CONTROL_REGION=>(0xfffffc20003bbc30,50128,false,Zero),
        RUNTIME_FLAGS=>(0xfffffc2040857fc0,64,false,Zero),
        _=>return Err(Error::Bounds),
    };
    Ok(Allocation{address,size,gpu_shared,initial})
}
/// Hardware B IOMapping entries are packed 32-byte records. Physical address,
/// size, range size and flags come from the generated HwData image; Config
/// supplies the owned firmware virtual address at +8. Slots are HwDataB
/// IO-mapping table slots. offset preserves a subpage PA.
#[derive(Clone,Copy,Debug)]
pub(crate) struct IoMap {
    pub(crate) slot:usize,pub(crate) physical:u64,pub(crate) size:usize,
    pub(crate) address:u64,pub(crate) offset:usize,
}
impl IoMap {
    pub(crate) fn pointer_field(self)->Result<usize,Error> {
        if self.slot>=31 {return Err(Error::Bounds);}
        Ok(0x640+self.slot*32+8)
    }
    /// The mapping must fit both the CPU physical range and the firmware VA range.
    fn validate(self) -> Result<(), Error> {
        self.pointer_field()?;
        if self.size == 0 || self.offset >= self.size
            || (self.physical | self.address | self.size as u64) & 0x3fff != 0
            || self.physical.checked_add(self.size as u64).is_none_or(|end| end > 1 << 42)
        {
            return Err(Error::Address);
        }
        Region::new(self.address, self.size)?;
        Ok(())
    }

    /// Check the subpage physical range without wrapping either endpoint.
    pub(crate) fn covers(self, physical: u64, total: u32) -> bool {
        total != 0
            && self.physical.checked_add(self.offset as u64) == Some(physical)
            && physical.checked_add(u64::from(total)).is_some_and(|end| {
                self.physical.checked_add(self.size as u64).is_some_and(|limit| end <= limit)
            })
    }

    pub(crate) fn pointer(self,owned:Region)->Result<u64,Error> {owned.at(self.offset,1)}
}
/// Validate every mapping before any GPU access, rejecting duplicate slots and overlapping VAs.
pub(crate) fn validate_iomaps(iomaps: &[IoMap]) -> Result<(), Error> {
    if iomaps.is_empty() {
        return Err(Error::Size);
    }
    for (i, io) in iomaps.iter().enumerate() {
        io.validate()?;
        for other in &iomaps[..i] {
            if io.slot == other.slot
                || (io.address < other.address + other.size as u64
                    && other.address < io.address + io.size as u64)
            {
                return Err(Error::Address);
            }
        }
        for index in 0..COUNT {
            let owner = allocation(index)?;
            let start = owner.address & !0x3fff;
            let end = owner.address.checked_add(owner.size as u64)
                .and_then(|end| end.checked_add(0x3fff)).ok_or(Error::Address)? & !0x3fff;
            if io.address < end && start < io.address + io.size as u64 {
                return Err(Error::Address);
            }
        }
    }
    Ok(())
}

/// Firmware VA of the first runtime IO map.
pub(crate) const IOMAP_BASE: u64 = 0xfffffc2068000000;

/// The runtime IO maps of `mappings` (HwDataB slot, physical address, total size, element size,
/// writable): each map is the 16 KiB pages that hold its mapping, with the subpage offset kept,
/// and starts at the end of the previous map plus a 16 KiB gap, the first at `base`.
pub(crate) const fn pack_iomaps<const N: usize>(
    mappings: &[(usize, u64, u32, u32, bool); N],
    base: u64,
) -> [IoMap; N] {
    let mut maps = [IoMap { slot: 0, physical: 0, size: 0, address: 0, offset: 0 }; N];
    let mut address = base;
    let mut i = 0;
    while i < N {
        let (slot, physical, total, _, _) = mappings[i];
        let offset = (physical & 0x3fff) as usize;
        let size = (offset + total as usize + 0x3fff) & !0x3fff;
        maps[i] = IoMap { slot, physical: physical & !0x3fff, size, address, offset };
        address += size as u64 + 0x4000;
        i += 1;
    }
    maps
}

/// Whether two IO-map tables are the same, entry by entry.
pub(crate) const fn same_iomaps(a: &[IoMap], b: &[IoMap]) -> bool {
    if a.len() != b.len() {
        return false;
    }
    let mut i = 0;
    while i < a.len() {
        let (x, y) = (a[i], b[i]);
        if x.slot != y.slot || x.physical != y.physical || x.size != y.size
            || x.address != y.address || x.offset != y.offset
        {
            return false;
        }
        i += 1;
    }
    true
}

// HWDataB.gpu_region_base refers to the physical GPU reserved region (TTBs),
// not an MMIO firmware VA. Resources validates its ownership and alignment.
pub(crate) const GPU_REGION_PHYSICAL:usize=0xb44;
/// Unknown C1's qualified parameter-buffer descriptor. The first word is
/// retained verbatim; the second word supplies the 22-bit page count. This is
/// the previous render_pb initialization with its page-count patch combined.
pub(crate) fn parameter_buffer(pages:u32)->Result<[u8;16],Error> {
    if pages>0x3fffff {return Err(Error::Bounds);}
    let mut bytes=[0;16];bytes[..4].copy_from_slice(&0x07400000u32.to_le_bytes());
    bytes[4..8].copy_from_slice(&pages.to_le_bytes());Ok(bytes)
}

#[cfg(test)]
mod iomap_tests {
    use super::*;

    /// A valid IO-map table (the former T6030 runtime's): the page-aligned CPU
    /// physical block and the fixed firmware VA of each slot.
    const T6030_IOMAPS:[IoMap;15]=[
        IoMap { slot:0, physical:0x290d00000, size:0x144000, address:0xfffffc2068000000, offset:0x0 },
        IoMap { slot:1, physical:0x20e100000, size:0x4000, address:0xfffffc2068148000, offset:0x1000 },
        IoMap { slot:2, physical:0x351014000, size:0x4000, address:0xfffffc2068150000, offset:0x0 },
        IoMap { slot:3, physical:0x290000000, size:0x20000, address:0xfffffc2068158000, offset:0x0 },
        IoMap { slot:7, physical:0x3502bc000, size:0x4000, address:0xfffffc206817c000, offset:0x0 },
        IoMap { slot:9, physical:0x290e08000, size:0x8000, address:0xfffffc2068184000, offset:0x0 },
        IoMap { slot:11, physical:0x220000000, size:0x12c000, address:0xfffffc2068190000, offset:0x0 },
        IoMap { slot:12, physical:0x35104c000, size:0x4000, address:0xfffffc20682c0000, offset:0x0 },
        IoMap { slot:18, physical:0x3503d0000, size:0x4000, address:0xfffffc20682c8000, offset:0x0 },
        IoMap { slot:19, physical:0x3503c0000, size:0x4000, address:0xfffffc20682d0000, offset:0x0 },
        IoMap { slot:20, physical:0x3503d8000, size:0x4000, address:0xfffffc20682d8000, offset:0x0 },
        IoMap { slot:23, physical:0x293000000, size:0x400000, address:0xfffffc20682e0000, offset:0x0 },
        IoMap { slot:25, physical:0x30945c000, size:0x4000, address:0xfffffc20686e4000, offset:0x0 },
        IoMap { slot:26, physical:0x350280000, size:0x8000, address:0xfffffc20686ec000, offset:0x0 },
        IoMap { slot:29, physical:0x290e5c000, size:0x4000, address:0xfffffc20686f8000, offset:0x0 },
    ];

    #[test]
    fn t6030_maps_and_relocated_physical_blocks_are_valid() {
        assert_eq!(validate_iomaps(&T6030_IOMAPS), Ok(()));
        let mut maps = T6030_IOMAPS;
        for io in &mut maps {
            io.physical += 0x40000;
        }
        assert_eq!(validate_iomaps(&maps), Ok(()));
    }

    #[test]
    fn rejects_duplicate_slots_overlapping_vas_and_initdata_aliases() {
        let mut maps = T6030_IOMAPS;
        maps[1].slot = maps[0].slot;
        assert!(validate_iomaps(&maps).is_err());
        maps = T6030_IOMAPS;
        maps[1].address = maps[0].address + 0x4000;
        assert!(validate_iomaps(&maps).is_err());
        maps = T6030_IOMAPS;
        maps[1].address = allocation(HARDWARE_DATA).unwrap().address & !0x3fff;
        assert!(validate_iomaps(&maps).is_err());
        assert!(validate_iomaps(&[]).is_err());
    }

    #[test]
    fn packing_keeps_subpage_offsets_and_leaves_a_page_between_maps() {
        let maps = pack_iomaps(
            &[
                (0, 0x2_90d0_0000, 0x10_4000, 0x10_4000, true),
                (1, 0x2_0e10_1000, 1, 1, false),
                (10, 0x2_90d0_d000, 0x1000, 0x1000, true),
                (11, 0x2_2000_0000, 0xb_0000, 0x5_8000, true),
            ],
            IOMAP_BASE,
        );
        assert_eq!(validate_iomaps(&maps), Ok(()));
        let expect = [
            (0, 0x2_90d0_0000, 0x10_4000, IOMAP_BASE, 0),
            (1, 0x2_0e10_0000, 0x4000, IOMAP_BASE + 0x10_8000, 0x1000),
            (10, 0x2_90d0_c000, 0x4000, IOMAP_BASE + 0x11_0000, 0x1000),
            (11, 0x2_2000_0000, 0xb_0000, IOMAP_BASE + 0x11_8000, 0),
        ];
        for (io, (slot, physical, size, address, offset)) in maps.iter().zip(expect) {
            assert_eq!((io.slot, io.physical, io.size, io.address, io.offset),
                (slot, physical, size, address, offset));
        }
        assert!(maps[1].covers(0x2_0e10_1000, 1));
        assert!(maps[2].covers(0x2_90d0_d000, 0x1000));
        assert!(same_iomaps(&maps, &maps));
        assert!(!same_iomaps(&maps, &maps[..3]));
        let mut moved = maps;
        moved[3].address += 0x4000;
        assert!(!same_iomaps(&maps, &moved));
    }

    #[test]
    fn rejects_invalid_ranges_and_preserves_subpage_offsets() {
        for edit in 0..6 {
            let mut maps = T6030_IOMAPS;
            match edit {
                0 => maps[0].slot = 31,
                1 => maps[0].size = 0,
                2 => maps[0].offset = maps[0].size,
                3 => maps[0].physical = (1 << 42) - 0x4000,
                4 => maps[0].address = u64::MAX & !0x3fff,
                _ => maps[0].physical += 1,
            }
            assert!(validate_iomaps(&maps).is_err());
        }
        let io = T6030_IOMAPS[1];
        assert!(io.covers(io.physical + 0x1000, 1));
        assert!(!io.covers(io.physical, 1));
        assert!(!io.covers(io.physical + 0x1000, 0x4000));
        assert!(!io.covers(io.physical + 0x1000, 0));
        let overflow = IoMap { physical: u64::MAX - 0x1000, ..io };
        assert!(!overflow.covers(u64::MAX, 1));
    }
}
