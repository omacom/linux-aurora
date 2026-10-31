// SPDX-License-Identifier: GPL-2.0-only OR MIT
//! Offline tests of the production parser. All addresses are synthetic.
#![allow(dead_code)]

#[path = "../../drivers/gpu/drm/asahi/m3/t8122_admission.rs"]
mod admission;
#[path = "../../drivers/gpu/drm/asahi/m3/m3_profile.rs"]
mod m3_profile;
#[path = "../../drivers/gpu/drm/asahi/m3/agx_uapi.rs"]
mod uapi;
use uapi::*;

fn frame(kind: u16, payload: &[u8]) -> Vec<u8> {
    let mut bytes = Vec::new();
    for word in [
        kind,
        payload.len() as u16,
        UAPI_BARRIER_NONE,
        UAPI_BARRIER_NONE,
    ] {
        bytes.extend_from_slice(&word.to_le_bytes());
    }
    bytes.extend_from_slice(payload);
    bytes
}

fn parse(bytes: &[u8]) -> Result<u32, UapiTranslateError> {
    let mut parser = UapiCommandParser::new(bytes);
    while parser.next_hardware()?.is_some() {}
    parser.finish()
}

// Each offset below is checked against offsetof of the corresponding C member.
fn layout() -> &'static std::collections::BTreeMap<String, (usize, usize)> {
    static LAYOUT: std::sync::OnceLock<std::collections::BTreeMap<String, (usize, usize)>> =
        std::sync::OnceLock::new();
    LAYOUT.get_or_init(|| {
        let path = std::env::var("ASAHI_UAPI_LAYOUT").expect("C layout path");
        std::fs::read_to_string(path)
            .unwrap()
            .lines()
            .map(|line| {
                let words: Vec<_> = line.split_whitespace().collect();
                (
                    words[0].to_string(),
                    (words[1].parse().unwrap(), words[2].parse().unwrap()),
                )
            })
            .collect()
    })
}

fn field(raw: &[u8], name: &str, offset: usize, width: usize, actual: u64) {
    assert_eq!(layout()[name], (offset, width), "C offset/width for {name}");
    let mut expected = [0u8; 8];
    for (i, byte) in expected[..width].iter_mut().enumerate() {
        *byte = raw.get(offset + i).copied().unwrap_or(0);
    }
    assert_eq!(actual, u64::from_le_bytes(expected), "decoded {name}");
}

fn check_render(p: UapiRenderCommand, raw: &[u8]) {
    field(raw, "cmd_render.flags", 0, 4, p.flags as u64);
    field(
        raw,
        "cmd_render.isp_zls_pixels",
        4,
        4,
        p.depth_dimensions as u64,
    );
    field(
        raw,
        "cmd_render.vdm_ctrl_stream_base",
        8,
        8,
        p.vdm_base as u64,
    );
    field(
        raw,
        "cmd_render.vertex_helper.binary",
        16,
        4,
        p.vertex_helper.binary as u64,
    );
    field(
        raw,
        "cmd_render.vertex_helper.cfg",
        20,
        4,
        p.vertex_helper.config as u64,
    );
    field(
        raw,
        "cmd_render.vertex_helper.data",
        24,
        8,
        p.vertex_helper.data as u64,
    );
    field(
        raw,
        "cmd_render.fragment_helper.binary",
        32,
        4,
        p.fragment_helper.binary as u64,
    );
    field(
        raw,
        "cmd_render.fragment_helper.cfg",
        36,
        4,
        p.fragment_helper.config as u64,
    );
    field(
        raw,
        "cmd_render.fragment_helper.data",
        40,
        8,
        p.fragment_helper.data as u64,
    );
    field(
        raw,
        "cmd_render.isp_scissor_base",
        48,
        8,
        p.scissor_base as u64,
    );
    field(
        raw,
        "cmd_render.isp_dbias_base",
        56,
        8,
        p.depth_bias_base as u64,
    );
    field(
        raw,
        "cmd_render.isp_oclqry_base",
        64,
        8,
        p.occlusion_query_base as u64,
    );
    field(raw, "cmd_render.depth.base", 72, 8, p.depth.base as u64);
    field(
        raw,
        "cmd_render.depth.comp_base",
        80,
        8,
        p.depth.compression_base as u64,
    );
    field(raw, "cmd_render.depth.stride", 88, 4, p.depth.stride as u64);
    field(
        raw,
        "cmd_render.depth.comp_stride",
        92,
        4,
        p.depth.compression_stride as u64,
    );
    field(raw, "cmd_render.stencil.base", 96, 8, p.stencil.base as u64);
    field(
        raw,
        "cmd_render.stencil.comp_base",
        104,
        8,
        p.stencil.compression_base as u64,
    );
    field(
        raw,
        "cmd_render.stencil.stride",
        112,
        4,
        p.stencil.stride as u64,
    );
    field(
        raw,
        "cmd_render.stencil.comp_stride",
        116,
        4,
        p.stencil.compression_stride as u64,
    );
    field(raw, "cmd_render.zls_ctrl", 120, 8, p.zls_control as u64);
    field(
        raw,
        "cmd_render.ppp_multisamplectl",
        128,
        8,
        p.multisample_control as u64,
    );
    field(
        raw,
        "cmd_render.sampler_heap",
        136,
        8,
        p.sampler_heap as u64,
    );
    field(raw, "cmd_render.ppp_ctrl", 144, 4, p.ppp_control as u64);
    field(raw, "cmd_render.width_px", 148, 2, p.width as u64);
    field(raw, "cmd_render.height_px", 150, 2, p.height as u64);
    field(raw, "cmd_render.layers", 152, 2, p.layers as u64);
    field(
        raw,
        "cmd_render.sampler_count",
        154,
        2,
        p.sampler_count as u64,
    );
    field(
        raw,
        "cmd_render.utile_width_px",
        156,
        1,
        p.utile_width as u64,
    );
    field(
        raw,
        "cmd_render.utile_height_px",
        157,
        1,
        p.utile_height as u64,
    );
    field(raw, "cmd_render.samples", 158, 1, p.samples as u64);
    field(
        raw,
        "cmd_render.sample_size_B",
        159,
        1,
        p.sample_size as u64,
    );
    field(
        raw,
        "cmd_render.isp_merge_upper_x",
        160,
        4,
        p.merge_upper_x as u64,
    );
    field(
        raw,
        "cmd_render.isp_merge_upper_y",
        164,
        4,
        p.merge_upper_y as u64,
    );
    field(raw, "cmd_render.bg.usc", 168, 4, p.background.usc as u64);
    field(
        raw,
        "cmd_render.bg.rsrc_spec",
        172,
        4,
        (p.background.resource_spec & 0xffffffff) as u64,
    );
    field(
        raw,
        "cmd_render.bg_rsrc_spec_hi",
        248,
        4,
        (p.background.resource_spec >> 32) as u64,
    );
    field(raw, "cmd_render.eot.usc", 176, 4, p.end_of_tile.usc as u64);
    field(
        raw,
        "cmd_render.eot.rsrc_spec",
        180,
        4,
        (p.end_of_tile.resource_spec & 0xffffffff) as u64,
    );
    field(
        raw,
        "cmd_render.bg_eot_rsrc_spec_hi",
        240,
        4,
        (p.end_of_tile.resource_spec >> 32) as u64,
    );
    field(
        raw,
        "cmd_render.partial_bg.usc",
        184,
        4,
        p.partial_background.usc as u64,
    );
    field(
        raw,
        "cmd_render.partial_bg.rsrc_spec",
        188,
        4,
        (p.partial_background.resource_spec & 0xffffffff) as u64,
    );
    field(
        raw,
        "cmd_render.bg_partial_rsrc_spec_hi",
        252,
        4,
        (p.partial_background.resource_spec >> 32) as u64,
    );
    field(
        raw,
        "cmd_render.partial_eot.usc",
        192,
        4,
        p.partial_end_of_tile.usc as u64,
    );
    field(
        raw,
        "cmd_render.partial_eot.rsrc_spec",
        196,
        4,
        (p.partial_end_of_tile.resource_spec & 0xffffffff) as u64,
    );
    field(
        raw,
        "cmd_render.bg_eot_partial_rsrc_spec_hi",
        244,
        4,
        (p.partial_end_of_tile.resource_spec >> 32) as u64,
    );
    field(
        raw,
        "cmd_render.isp_bgobjdepth",
        200,
        4,
        p.depth_clear as u64,
    );
    field(
        raw,
        "cmd_render.isp_bgobjvals",
        204,
        4,
        p.stencil_clear as u64,
    );
    field(
        raw,
        "cmd_render.ts_vtx.start.handle",
        208,
        4,
        p.vertex_timestamps.start.handle as u64,
    );
    field(
        raw,
        "cmd_render.ts_vtx.start.offset",
        212,
        4,
        p.vertex_timestamps.start.offset as u64,
    );
    field(
        raw,
        "cmd_render.ts_vtx.end.handle",
        216,
        4,
        p.vertex_timestamps.end.handle as u64,
    );
    field(
        raw,
        "cmd_render.ts_vtx.end.offset",
        220,
        4,
        p.vertex_timestamps.end.offset as u64,
    );
    field(
        raw,
        "cmd_render.ts_frag.start.handle",
        224,
        4,
        p.fragment_timestamps.start.handle as u64,
    );
    field(
        raw,
        "cmd_render.ts_frag.start.offset",
        228,
        4,
        p.fragment_timestamps.start.offset as u64,
    );
    field(
        raw,
        "cmd_render.ts_frag.end.handle",
        232,
        4,
        p.fragment_timestamps.end.handle as u64,
    );
    field(
        raw,
        "cmd_render.ts_frag.end.offset",
        236,
        4,
        p.fragment_timestamps.end.offset as u64,
    );
}

fn check_compute(p: UapiComputeCommand, raw: &[u8]) {
    field(raw, "cmd_compute.flags", 0, 4, p.flags as u64);
    field(
        raw,
        "cmd_compute.sampler_count",
        4,
        4,
        p.sampler_count as u64,
    );
    field(
        raw,
        "cmd_compute.cdm_ctrl_stream_base",
        8,
        8,
        p.control_stream_base as u64,
    );
    field(
        raw,
        "cmd_compute.cdm_ctrl_stream_end",
        16,
        8,
        p.control_stream_end as u64,
    );
    field(
        raw,
        "cmd_compute.sampler_heap",
        24,
        8,
        p.sampler_heap as u64,
    );
    field(
        raw,
        "cmd_compute.helper.binary",
        32,
        4,
        p.helper.binary as u64,
    );
    field(raw, "cmd_compute.helper.cfg", 36, 4, p.helper.config as u64);
    field(raw, "cmd_compute.helper.data", 40, 8, p.helper.data as u64);
    field(
        raw,
        "cmd_compute.ts.start.handle",
        48,
        4,
        p.timestamps.start.handle as u64,
    );
    field(
        raw,
        "cmd_compute.ts.start.offset",
        52,
        4,
        p.timestamps.start.offset as u64,
    );
    field(
        raw,
        "cmd_compute.ts.end.handle",
        56,
        4,
        p.timestamps.end.handle as u64,
    );
    field(
        raw,
        "cmd_compute.ts.end.offset",
        60,
        4,
        p.timestamps.end.offset as u64,
    );
}

fn fixture() -> Vec<u8> {
    std::fs::read(std::env::var("ASAHI_UAPI_FIXTURE").expect("C fixture path")).unwrap()
}

#[test]
fn c_header_producer_matches_production_decoder() {
    let offsets = layout();
    assert_eq!(offsets["header"].0, UAPI_COMMAND_HEADER_SIZE);
    assert_eq!(offsets["attachment"].0, UAPI_ATTACHMENT_SIZE);
    assert_eq!(offsets["timestamps"].0, 16);
    assert_eq!(offsets["cmd_render.ts_frag_end_copies"].0, UAPI_RENDER_SIZE);
    assert_eq!(offsets["cmd_compute.ts_end_copies"].0, UAPI_COMPUTE_SIZE);
    let bytes = fixture();
    let render_offset = 2 * UAPI_COMMAND_HEADER_SIZE + UAPI_ATTACHMENT_SIZE;
    let render_size = u16::from_le_bytes(
        bytes[render_offset - 6..render_offset - 4]
            .try_into()
            .unwrap(),
    ) as usize;
    let compute_offset = render_offset + render_size + UAPI_COMMAND_HEADER_SIZE;
    let mut parser = UapiCommandParser::new(&bytes);
    let Some(ParsedHardwareCommand::Render {
        index,
        header,
        payload,
        vertex_attachments,
        fragment_attachments,
    }) = parser.next_hardware().unwrap()
    else {
        panic!("render missing")
    };
    assert_eq!(index, 2);
    assert_eq!(
        header,
        UapiCommandHeader {
            command_type: 0,
            size: render_size as u16,
            render_barrier: UAPI_BARRIER_NONE,
            compute_barrier: UAPI_BARRIER_NONE,
        }
    );
    check_render(payload, &bytes[render_offset..render_offset + render_size]);
    assert_eq!(vertex_attachments.count, 1);
    let attachment = vertex_attachments.as_slice()[0];
    field(
        &bytes[8..32],
        "attachment.pointer",
        0,
        8,
        attachment.address,
    );
    field(&bytes[8..32], "attachment.size", 8, 8, attachment.size);
    assert!(fragment_attachments.as_slice().is_empty());
    let Some(ParsedHardwareCommand::Compute {
        index,
        header,
        payload,
        attachments,
    }) = parser.next_hardware().unwrap()
    else {
        panic!("compute missing")
    };
    assert_eq!(index, 3);
    assert_eq!(
        header,
        UapiCommandHeader {
            command_type: 1,
            size: (bytes.len() - compute_offset) as u16,
            render_barrier: UAPI_BARRIER_NONE,
            compute_barrier: UAPI_BARRIER_NONE,
        }
    );
    check_compute(payload, &bytes[compute_offset..]);
    assert!(attachments.as_slice().is_empty());
    assert!(parser.next_hardware().unwrap().is_none());
    assert_eq!(parser.finish(), Ok(2));
}

#[test]
fn short_payloads_zero_extend_but_missing_bytes_are_rejected() {
    // Every length includes cuts within each multi-byte field, for both kinds.
    for (kind, size) in [(0, 256), (1, 64)] {
        let supplied: Vec<u8> = (0..size).map(|i| (i % 254 + 1) as u8).collect();
        for length in 0..=size {
            let raw = &supplied[..length];
            let bytes = frame(kind, raw);
            let mut parser = UapiCommandParser::new(&bytes);
            match parser.next_hardware().unwrap().unwrap() {
                ParsedHardwareCommand::Render { payload, .. } => check_render(payload, raw),
                ParsedHardwareCommand::Compute { payload, .. } => check_compute(payload, raw),
            }
            assert!(parser.next_hardware().unwrap().is_none());
            assert_eq!(parser.finish(), Ok(1));
            for cut in 1..bytes.len() {
                assert!(parse(&bytes[..cut]).is_err());
            }
        }
    }
    assert_eq!(parse(&[]), Err(UapiTranslateError::HardwareCommandCount));
}

#[test]
fn named_optional_timestamp_copies_are_refused() {
    for (kind, name) in [
        (0, "cmd_render.ts_frag_end_copies"),
        (1, "cmd_compute.ts_end_copies"),
    ] {
        let (offset, size) = layout()[name];
        let mut raw = vec![0; offset + size];
        assert_eq!(parse(&frame(kind, &raw)), Ok(1));
        for i in offset..raw.len() {
            raw[i] = 1;
            assert_eq!(
                parse(&frame(kind, &raw)),
                Err(UapiTranslateError::PayloadTail),
                "{name} byte {i}"
            );
            raw[i] = 0;
        }
    }
}

#[test]
fn unsupported_nonzero_extensions_never_disappear() {
    for (kind, size) in [(0, UAPI_RENDER_SIZE), (1, UAPI_COMPUTE_SIZE)] {
        for extra in [1, 8, 64, 128] {
            let mut payload = vec![0; size + extra];
            assert_eq!(parse(&frame(kind, &payload)), Ok(1));
            for offset in size..payload.len() {
                payload[offset] = 1;
                assert_eq!(
                    parse(&frame(kind, &payload)),
                    Err(UapiTranslateError::PayloadTail)
                );
                payload[offset] = 0;
            }
        }
    }
    assert_eq!(
        parse(&frame(0xffff, &[])),
        Err(UapiTranslateError::CommandType)
    );
}

#[test]
fn attachment_shapes_counts_and_reserved_fields_are_checked() {
    for kind in 2..=4 {
        for size in 1..UAPI_ATTACHMENT_SIZE {
            assert_eq!(
                parse(&frame(kind, &vec![0; size])),
                Err(UapiTranslateError::AttachmentPayloadSize)
            );
        }
        let mut bytes = frame(kind, &vec![0; UAPI_ATTACHMENT_SIZE * UAPI_MAX_ATTACHMENTS]);
        bytes.extend(frame(1, &[]));
        assert_eq!(parse(&bytes), Ok(1));
        assert_eq!(
            parse(&frame(
                kind,
                &vec![0; UAPI_ATTACHMENT_SIZE * (UAPI_MAX_ATTACHMENTS + 1)]
            )),
            Err(UapiTranslateError::AttachmentCount)
        );
        for offset in 16..24 {
            let mut payload = [0; UAPI_ATTACHMENT_SIZE];
            payload[offset] = 1;
            assert_eq!(
                parse(&frame(kind, &payload)),
                Err(UapiTranslateError::AttachmentFields)
            );
        }
        let mut bytes = frame(kind, &[]);
        bytes[4..6].copy_from_slice(&0u16.to_le_bytes());
        assert_eq!(parse(&bytes), Err(UapiTranslateError::AttachmentBarrier));
    }
}

#[test]
fn attachment_lists_reach_their_commands() {
    let entry = |pointer: u64, size: u64| {
        let mut raw = [0u8; UAPI_ATTACHMENT_SIZE];
        raw[..8].copy_from_slice(&pointer.to_le_bytes());
        raw[8..16].copy_from_slice(&size.to_le_bytes());
        raw
    };
    let vertex = entry(0x1_2345_6000, 0x1_0000_0040).to_vec();
    let fragment = [
        entry(0x2_0000_8000, 0x80),
        entry(0x3_0000_c000, 0x2_0000_0100),
    ]
    .concat();
    let compute = entry(0x4_0000_1000, 0x5_0000_0010).to_vec();
    let check = |list: UapiAttachmentList, raw: &[u8]| {
        assert_eq!(list.as_slice().len(), raw.len() / UAPI_ATTACHMENT_SIZE);
        for (attachment, raw) in list.as_slice().iter().zip(raw.chunks(UAPI_ATTACHMENT_SIZE)) {
            field(raw, "attachment.pointer", 0, 8, attachment.address);
            field(raw, "attachment.size", 8, 8, attachment.size);
        }
    };

    // Lists persist until replaced; an empty list clears only its own kind.
    let mut bytes = frame(2, &vertex);
    bytes.extend(frame(3, &fragment));
    bytes.extend(frame(4, &compute));
    bytes.extend(frame(0, &[]));
    bytes.extend(frame(1, &[]));
    bytes.extend(frame(3, &[]));
    bytes.extend(frame(0, &[]));
    let mut parser = UapiCommandParser::new(&bytes);
    for expected_fragment in [&fragment[..], &[]] {
        let Some(ParsedHardwareCommand::Render {
            vertex_attachments,
            fragment_attachments,
            ..
        }) = parser.next_hardware().unwrap()
        else {
            panic!("render missing")
        };
        check(vertex_attachments, &vertex);
        check(fragment_attachments, expected_fragment);
        if !expected_fragment.is_empty() {
            let Some(ParsedHardwareCommand::Compute { attachments, .. }) =
                parser.next_hardware().unwrap()
            else {
                panic!("compute missing")
            };
            check(attachments, &compute);
        }
    }
    assert!(parser.next_hardware().unwrap().is_none());
    assert_eq!(parser.finish(), Ok(3));
}

#[test]
fn dependencies_count_hardware_commands_and_bound_the_submit() {
    for field in [4, 6] {
        let mut bytes = frame(1, &[]);
        bytes[field..field + 2].copy_from_slice(&1u16.to_le_bytes());
        assert_eq!(parse(&bytes), Err(UapiTranslateError::BarrierFuture));
    }
    let mut bytes = frame(2, &[]);
    bytes.extend(frame(0, &[]));
    let mut compute = frame(1, &[]);
    compute[4..6].copy_from_slice(&1u16.to_le_bytes());
    bytes.extend(compute);
    assert_eq!(parse(&bytes), Ok(2));
    let command = frame(1, &[]);
    assert_eq!(
        parse(&command.repeat(UAPI_MAX_HARDWARE_COMMANDS as usize)),
        Ok(UAPI_MAX_HARDWARE_COMMANDS)
    );
    assert_eq!(
        parse(&command.repeat(UAPI_MAX_HARDWARE_COMMANDS as usize + 1)),
        Err(UapiTranslateError::HardwareCommandCount)
    );
}

struct Region {
    start: u64,
    end: u64,
    writable: bool,
}
impl GpuAddressSpace for Region {
    fn covers(&self, address: u64, size: u64, access: GpuAccess) -> bool {
        let Some(end) = address.checked_add(size) else {
            return false;
        };
        address >= self.start && end <= self.end && (access == GpuAccess::Read || self.writable)
    }
}

#[test]
fn attachment_validation_rejects_readonly_zero_and_wrapped_ranges() {
    let mut list = UapiAttachmentList::EMPTY;
    list.count = 1;
    let mut region = Region {
        start: 0x4000,
        end: 0x8000,
        writable: false,
    };
    list.entries[0] = UapiAttachment {
        address: 0x4000,
        size: 0x4000,
    };
    assert_eq!(
        validate_attachments(&region, &list),
        Err(UapiTranslateError::AddressRange)
    );
    region.writable = true;
    assert_eq!(validate_attachments(&region, &list), Ok(()));
    for (address, size) in [(0x4000, 0), (0x4000, 0x4001), (u64::MAX, 2)] {
        list.entries[0] = UapiAttachment { address, size };
        assert_eq!(
            validate_attachments(&region, &list),
            Err(UapiTranslateError::AddressRange)
        );
    }
}

#[test]
fn usc_windows_reject_misalignment_overflow_and_unmapped_programs() {
    let queue = QueueUscWindow {
        base: USC_WINDOW_SIZE,
        user_start: 0x4000,
        user_end: 2 * USC_WINDOW_SIZE,
    };
    assert_eq!(queue.validate(), Ok(()));
    assert_eq!(
        QueueUscWindow {
            base: queue.base + 1,
            ..queue
        }
        .validate(),
        Err(UapiTranslateError::QueueUscAlignment)
    );
    assert_eq!(
        QueueUscWindow {
            base: u64::MAX & !(USC_WINDOW_SIZE - 1),
            ..queue
        }
        .validate(),
        Err(UapiTranslateError::QueueUscRange)
    );
    assert_eq!(
        QueueUscWindow {
            user_end: queue.user_end - 1,
            ..queue
        }
        .validate(),
        Err(UapiTranslateError::QueueUscRange)
    );
    let region = Region {
        start: queue.base + 8,
        end: queue.base + 12,
        writable: false,
    };
    assert_eq!(queue.full_program(&region, 9), Ok(queue.base + 8));
    assert_eq!(
        queue.full_program(&region, 0),
        Err(UapiTranslateError::ProgramOffset)
    );
    assert_eq!(
        queue.full_program(&region, 16),
        Err(UapiTranslateError::AddressRange)
    );
}

#[test]
fn unsupported_firmware_cannot_reach_the_existing_profile() {
    let identity = (1, 1, 0x07022000, [0xff, 0, 0]);
    assert!(admission::validate_identity(
        &[14, 8, 3],
        identity.0,
        identity.1,
        identity.2,
        identity.3
    )
    .is_some());
    for version in [&[26, 6, 2][..], &[14, 8][..], &[14, 8, 3, 0][..], &[][..]] {
        assert!(admission::validate_identity(
            version, identity.0, identity.1, identity.2, identity.3
        )
        .is_none());
    }
}

#[test]
fn firmware_profile_agrees_with_the_actual_c_header() {
    assert_eq!(
        layout()["profile.params_required"].0 as u64,
        m3_profile::PARAMS_REQUIRED
    );
    assert_eq!(layout()["profile.vm_create_size"].0, 24);
    assert_eq!(
        layout()["profile.hal200"].0 as u32,
        m3_profile::Profile::Extended.hal()
    );
    assert_eq!(layout()["profile.features"].0, 0x0a);
}
