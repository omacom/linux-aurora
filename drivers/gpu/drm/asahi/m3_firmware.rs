// SPDX-License-Identifier: GPL-2.0-only OR MIT


use crate::m3_resources as agx_resources;

/// A GPU firmware image the runtime can identify.
#[derive(Debug)]
pub(crate) struct KnownImage {
    /// Human-readable identity, for logs.
    pub(crate) name: &'static str,
    uuid: Option<[u8; 16]>,
    stkg_sha256: Option<[u8; 32]>,
    /// The validated InitData magic (InitData+0), or `None` until runtime admission is supported.
    pub(crate) initdata_magic: Option<u64>,
}

impl KnownImage {
    /// Runtime admission requires a validated InitData version and all recorded identities.
    pub(crate) fn accepts_runtime(&self, uuid: Option<[u8; 16]>, digest: &[u8; 32]) -> bool {
        self.initdata_magic.is_some()
            && (self.uuid.is_some() || self.stkg_sha256.is_some())
            && self.uuid.map_or(true, |known| uuid == Some(known))
            && self.stkg_sha256.map_or(true, |known| known == *digest)
    }
}

/// The G15 InitData version of firmware-compat 14.8.3, the version of the T8122 image.
pub(crate) const G15_V14_8_3_INITDATA: u64 = 0x0c08_e21e_8380_0490;

/// The image-info UUID of the T8122 C0 firmware-compat 14.8.3 image.
const T8122_C0_UUID: [u8; 16] = [
    0xdf, 0x69, 0x7f, 0x05, 0xf6, 0xb5, 0x33, 0xef, 0xa1, 0x37, 0x61, 0xc4, 0xa7, 0x3d, 0x66, 0x6a,
];

/// T8122 C0, firmware-compat 14.8.3: the image the macOS 14.8.3 system firmware loads on the
/// M3 MacBook Airs. It takes the G15 14.8.3 InitData version.
pub(crate) static KNOWN_IMAGES_T8122: [KnownImage; 1] = [KnownImage {
    name: "T8122 C0 firmware-compat 14.8.3",
    uuid: Some(T8122_C0_UUID),
    stkg_sha256: None,
    initdata_magic: Some(G15_V14_8_3_INITDATA),
}];

/// The same T8122 image for the start experiment (`t8122_start`, only with
/// `asahi.t8122_start=1`), where `asahi.t8122_initdata_version` may replace the version.
pub(crate) static KNOWN_IMAGES_T8122_EXPERIMENT: [KnownImage; 1] = [KnownImage {
    name: "T8122 C0 firmware-compat 14.8.3 (start experiment)",
    uuid: Some(T8122_C0_UUID),
    stkg_sha256: None,
    initdata_magic: Some(G15_V14_8_3_INITDATA),
}];

const IMAGE_INFO_OFFSET: usize = 0x4200;
/// Size of the identifying part of the image-info header.
const IMAGE_INFO_SIZE: usize = 0x38;
/// First word of the header: a branch over it (`b +0x44`).
const IMAGE_INFO_BRANCH: u32 = 0x1400_0011;
/// Header magic, "uuid".
const IMAGE_INFO_MAGIC: u32 = 0x6469_7575;

/// Parsed image-info header: UUID, patchbay offset and size, TEXT size.
pub(crate) struct ImageInfo {
    pub(crate) uuid: [u8; 16],
    pub(crate) patchbay: core::ops::Range<usize>,
    pub(crate) text_size: usize,
}

pub(crate) fn image_info(text: &[u8]) -> Option<ImageInfo> {
    let hdr = text.get(IMAGE_INFO_OFFSET..IMAGE_INFO_OFFSET + IMAGE_INFO_SIZE)?;
    let word = |i: usize| u32::from_le_bytes([hdr[i], hdr[i + 1], hdr[i + 2], hdr[i + 3]]);
    if word(0) != IMAGE_INFO_BRANCH || word(4) != IMAGE_INFO_MAGIC || word(8) != 5 {
        return None;
    }
    let mut uuid = [0u8; 16];
    uuid.copy_from_slice(&hdr[0x14..0x24]);
    let start = word(0x2c) as usize;
    let end = start.checked_add(word(0x30) as usize)?;
    if word(0x34) as usize != text.len() {
        return None;
    }
    text.get(start..end)?;
    Some(ImageInfo {
        uuid,
        patchbay: start..end,
        text_size: word(0x34) as usize,
    })
}

/// The layout of a loaded GPU firmware the runtime accepts: segment sizes and VAs, and the
/// per-boot words of the text segment that identification ignores.
#[derive(Debug)]
pub(crate) struct Layout {
    pub(crate) text_size: u64,
    pub(crate) data_size: u64,
    pub(crate) vas: [u64; 2],
    /// Offset of the 8-byte tag that precedes the 8 per-boot bytes zeroed before hashing.
    pub(crate) entropy_tag_offset: usize,
    pub(crate) entropy_tag: [u8; 8],
    /// The firmware version, for logs.
    pub(crate) version: &'static str,
}

/// T8122 firmware-compat 14.8.3 segment sizes, virtual addresses and entropy-tag bounds.
pub(crate) static T8122_LAYOUT: Layout = Layout {
    text_size: 0x64000,
    data_size: 0xb0000,
    vas: [0xffff_fc00_0000_0000, 0xffff_fc00_0006_4000],
    entropy_tag_offset: 0x5d54a,
    entropy_tag: *b"GKTS\x08\x00\x00\x00",
    version: "RTKit-2419.140.12.release",
};

fn normalize_boot_entropy(layout: &Layout, text: &mut [u8]) -> bool {
    let tag = layout.entropy_tag_offset;
    if text.len() != layout.text_size as usize {
        return false;
    }
    let Some(end) = tag.checked_add(16) else { return false; };
    let Some(entropy) = text.get_mut(tag..end) else { return false; };
    if entropy[..8] != layout.entropy_tag {
        return false;
    }
    entropy[8..].fill(0);
    true
}

#[derive(Debug)]
pub(crate) struct Firmware {
    pub(crate) resources: agx_resources::Resources,
    /// The InitData version validated for the identified image.
    pub(crate) initdata_magic: u64,
    /// The accepted layout.
    pub(crate) layout: &'static Layout,
}

impl Firmware {
    pub(crate) const fn version(&self) -> &'static str {
        self.layout.version
    }

    fn matching_layout(layout: &Layout, resources: &agx_resources::Resources) -> bool {
        resources.regions[4].size == layout.text_size
            && resources.regions[5].size == layout.data_size
            && resources.firmware_vas == layout.vas
    }
}

#[cfg(not(test))]
pub(crate) fn identify_loaded(
    pdev: &kernel::platform::Device<kernel::device::Core>,
    soc: &'static crate::m3_soc::Soc,
    resources: agx_resources::Resources,
    experiment: Option<&crate::t8122_start::Experiment>,
) -> kernel::error::Result<Firmware> {
    use kernel::{
        bindings, c_str,
        io::mem::{Mem, MemFlag},
        prelude::*,
    };

    let layout = soc.firmware.ok_or(ENODEV)?;
    if !Firmware::matching_layout(layout, &resources) {
        dev_err!(pdev.as_ref(), "M3 {}: unsupported firmware segment layout\n", soc.gpu_name);
        return Err(ENODEV);
    }
    let node = pdev.as_ref().of_node().ok_or(ENODEV)?;
    let text = agx_resources::reserved_resource(&node, c_str!("fw-text"))?;
    if text.start() != resources.regions[4].base || text.size() != layout.text_size {
        return Err(EINVAL);
    }
    let mapping = unsafe { Mem::try_new(text, MemFlag::WB.into()) }?;
    let bytes = unsafe { core::slice::from_raw_parts(mapping.ptr(), mapping.size()) };
    let mut canonical = KVec::new();
    canonical.extend_from_slice(bytes, GFP_KERNEL)?;
    if !normalize_boot_entropy(layout, &mut canonical) {
        return Err(ENODEV);
    }
    let mut digest = [0u8; 32];
    // SAFETY: the initialized private copy and distinct digest live for the
    // synchronous SHA-256 call. Normalization never writes loaded firmware.
    unsafe { bindings::sha256(canonical.as_ptr(), canonical.len(), digest.as_mut_ptr()) };
    // The T8122 start experiment identifies the same image, with its own InitData version.
    let images = match experiment {
        Some(_) => &KNOWN_IMAGES_T8122_EXPERIMENT[..],
        None => soc.images,
    };
    let image = crate::m3_board::identify(pdev.as_ref(), bytes, &digest, images).ok_or(ENODEV)?;
    let initdata_magic = match experiment {
        Some(experiment) => experiment.initdata_version(),
        None => image.initdata_magic.ok_or(ENODEV)?,
    };
    Ok(Firmware { resources, initdata_magic, layout })
}

#[cfg(test)]
mod tests {
    use super::*;
    use agx_resources::{Region, Resources};

    /// A second layout, with other segment sizes than the T8122's (the former T6030 one).
    static OTHER_LAYOUT: Layout = Layout {
        text_size: 0x5c000,
        data_size: 0x114000,
        vas: [0xffff_fc00_0000_0000, 0xffff_fc00_0005_c000],
        entropy_tag_offset: 0x58e88,
        entropy_tag: *b"GKTS\x08\x00\x00\x00",
        version: "RTKit-2419.140.12.release",
    };

    const TEXT_SHA256: [u8; 32] = [
        0x11, 0xe4, 0x9f, 0x75, 0xb, 0x67, 0x1a, 0x2b, 0x13, 0xd0, 0x92, 0xdb, 0x1c, 0xbc, 0xa3,
        0xa8, 0x77, 0x55, 0x86, 0xd6, 0xb1, 0xe3, 0xaa, 0xe8, 0xa6, 0xa4, 0x64, 0xa7, 0x80, 0x2,
        0x93, 0xd5,
    ];

    /// Images of another chip: one identified by its text digest, one by its UUID.
    static KNOWN_IMAGES: [KnownImage; 2] = [
        KnownImage {
            name: "digest-identified image",
            uuid: None,
            stkg_sha256: Some(TEXT_SHA256),
            initdata_magic: Some(G15_V14_8_3_INITDATA),
        },
        KnownImage {
            name: "uuid-identified image",
            uuid: Some([
                0xdb, 0xf3, 0x7c, 0x40, 0xea, 0xd5, 0x37, 0x60, 0x94, 0x41, 0x99, 0x50,
                0x18, 0x78, 0x29, 0x55,
            ]),
            stkg_sha256: None,
            initdata_magic: Some(G15_V14_8_3_INITDATA),
        },
    ];

    fn image_header(uuid: [u8; 16]) -> Vec<u8> {
        let mut text = vec![0; OTHER_LAYOUT.text_size as usize];
        let size = text.len() as u32;
        for (offset, value) in [
            (0, IMAGE_INFO_BRANCH), (4, IMAGE_INFO_MAGIC), (8, 5),
            (0x2c, size - 32), (0x30, 32), (0x34, size),
        ] {
            header_word(&mut text, offset, value);
        }
        text[IMAGE_INFO_OFFSET + 0x14..IMAGE_INFO_OFFSET + 0x24].copy_from_slice(&uuid);
        text
    }

    fn header_word(text: &mut [u8], offset: usize, value: u32) {
        let start = IMAGE_INFO_OFFSET + offset;
        text[start..start + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn admits_text(image: &KnownImage, text: &[u8], digest: &[u8; 32]) -> bool {
        image.accepts_runtime(image_info(text).map(|info| info.uuid), digest)
    }

    #[test]
    fn uuid_admission_requires_header_bounds_and_exact_text_size() {
        let image = &KNOWN_IMAGES[1];
        let text = image_header(image.uuid.unwrap());
        assert!(admits_text(image, &text, &[0; 32]));
        let size = text.len() as u32;
        for (offset, value) in [
            (0x34, 0), (0x34, size - 1), (0x34, size + 1),
            (0x2c, size), (0x2c, u32::MAX), (0x30, u32::MAX),
        ] {
            let mut bad = text.clone();
            header_word(&mut bad, offset, value);
            assert!(!admits_text(image, &bad, &[0; 32]), "offset {offset:#x}, {value:#x}");
        }
        let info = image_info(&text).unwrap();
        assert_eq!(info.patchbay, text.len() - 32..text.len());
        assert_eq!(info.text_size, text.len());
    }

    #[test]
    fn uuid_admission_rejects_bad_header_words_and_truncation() {
        let image = &KNOWN_IMAGES[1];
        let text = image_header(image.uuid.unwrap());
        for offset in [0, 4, 8] {
            let mut bad = text.clone();
            header_word(&mut bad, offset, 0);
            assert!(!admits_text(image, &bad, &[0; 32]));
        }
        for len in [0, IMAGE_INFO_OFFSET, IMAGE_INFO_OFFSET + IMAGE_INFO_SIZE - 1, text.len() - 1] {
            assert!(!admits_text(image, &text[..len], &[0; 32]));
        }
    }

    #[test]
    fn runtime_admission_requires_every_recorded_identity() {
        let image = KnownImage {
            name: "test", uuid: Some([1; 16]), stkg_sha256: Some([2; 32]),
            initdata_magic: Some(3),
        };
        let text = image_header([1; 16]);
        assert!(admits_text(&image, &text, &[2; 32]));
        assert!(!admits_text(&image, &text, &[0; 32]));
        assert!(!admits_text(&image, &image_header([0; 16]), &[2; 32]));
        assert!(!image.accepts_runtime(None, &[2; 32]));
        let anonymous = KnownImage { uuid: None, stkg_sha256: None, ..image };
        assert!(!admits_text(&anonymous, &text, &[2; 32]));
    }

    #[test]
    fn digest_only_admission_preserves_exact_digest_requirement() {
        let image = &KNOWN_IMAGES[0];
        assert!(admits_text(image, &[], &TEXT_SHA256));
        let mut wrong = TEXT_SHA256;
        wrong[0] ^= 1;
        assert!(!admits_text(image, &[], &wrong));
    }

    #[test]
    fn t8122_admission_requires_qualified_initdata_and_matching_image_identity() {
        let image = &KNOWN_IMAGES_T8122[0];
        assert_eq!(image.initdata_magic, Some(G15_V14_8_3_INITDATA));
        let text = image_header(image.uuid.unwrap());
        assert_eq!(image_info(&text).unwrap().uuid, image.uuid.unwrap());
        assert!(admits_text(image, &text, &[0; 32]));

        // A qualified version does not admit an unknown or another chip's image.
        for wrong_uuid in [[0; 16], KNOWN_IMAGES[1].uuid.unwrap()] {
            assert!(!admits_text(image, &image_header(wrong_uuid), &[0; 32]));
        }
        assert!(!image.accepts_runtime(None, &[0; 32]));

        // Matching identity still cannot admit an unqualified InitData layout.
        let unqualified = KnownImage {
            name: image.name, uuid: image.uuid, stkg_sha256: image.stkg_sha256,
            initdata_magic: None,
        };
        assert!(!admits_text(&unqualified, &text, &[0; 32]));
        let unidentified = KnownImage {
            name: "unknown", uuid: None, stkg_sha256: None,
            initdata_magic: image.initdata_magic,
        };
        assert!(!admits_text(&unidentified, &text, &[0; 32]));
    }

    #[test]
    fn t8122_experiment_record_identifies_the_same_image_with_the_g15_version() {
        let image = &KNOWN_IMAGES_T8122_EXPERIMENT[0];
        assert_eq!(image.uuid, KNOWN_IMAGES_T8122[0].uuid);
        assert_eq!(image.stkg_sha256, None);
        assert_eq!(image.initdata_magic, Some(G15_V14_8_3_INITDATA));
        assert!(KNOWN_IMAGES.iter().all(|i| i.initdata_magic == Some(G15_V14_8_3_INITDATA)));
        let text = image_header(image.uuid.unwrap());
        assert!(admits_text(image, &text, &[0; 32]));
        assert!(!admits_text(image, &image_header(KNOWN_IMAGES[1].uuid.unwrap()), &[0; 32]));
        assert!(!admits_text(&KNOWN_IMAGES[1], &text, &[0; 32]));
    }

    #[test]
    fn each_layout_rejects_the_other_chips_segments() {
        for (layout, other) in [(&OTHER_LAYOUT, &T8122_LAYOUT), (&T8122_LAYOUT, &OTHER_LAYOUT)] {
            let mut regions = [Region { base: 0, size: 0 }; 6];
            regions[4].size = layout.text_size;
            regions[5].size = layout.data_size;
            let mut resources = Resources { regions, firmware_vas: layout.vas };
            assert!(Firmware::matching_layout(layout, &resources));
            assert!(!Firmware::matching_layout(other, &resources));
            resources.firmware_vas[1] += 0x4000;
            assert!(!Firmware::matching_layout(layout, &resources));
        }
    }

    #[test]
    fn entropy_normalization_changes_only_the_eight_boot_bytes() {
        for layout in [&OTHER_LAYOUT, &T8122_LAYOUT] {
            let mut text = vec![0xa5; layout.text_size as usize];
            let tag = layout.entropy_tag_offset;
            text[tag..tag + 8].copy_from_slice(&layout.entropy_tag);
            let before = text.clone();
            assert!(normalize_boot_entropy(layout, &mut text));
            assert_eq!(&text[..tag + 8], &before[..tag + 8]);
            assert_eq!(&text[tag + 8..tag + 16], &[0; 8]);
            assert_eq!(&text[tag + 16..], &before[tag + 16..]);
        }
    }

    #[test]
    fn bad_entropy_bounds_tags_and_lengths_do_not_mutate_text() {
        for offset in [8, 16, usize::MAX] {
            let layout = Layout { text_size: 16, entropy_tag_offset: offset, ..T8122_LAYOUT };
            let mut text = vec![0xa5; 16];
            assert!(!normalize_boot_entropy(&layout, &mut text));
            assert_eq!(text, vec![0xa5; 16]);
        }
        let layout = Layout { text_size: 16, entropy_tag_offset: 0, ..T8122_LAYOUT };
        let mut text = vec![0xa5; 16];
        assert!(!normalize_boot_entropy(&layout, &mut text));
        assert_eq!(text, vec![0xa5; 16]);
        text.pop();
        assert!(!normalize_boot_entropy(&layout, &mut text));
        assert_eq!(text, vec![0xa5; 15]);
    }
}
