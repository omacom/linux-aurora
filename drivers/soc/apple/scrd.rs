// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

//! SEP credential endpoint (SCRD, EP 0x0a) wire protocol.

use crate::proto::*;
use crate::sks::SKS_AUTH_TOKEN_LEN;
use kernel::prelude::*;
use kernel::soc::apple::mailbox::Message;

pub(crate) const SCRD_ACM_HANDLE_LEN: usize = SKS_AUTH_TOKEN_LEN;

pub(crate) const SCRD_REQUEST_COMMAND: u8 = 1;

const SCRD_MAGIC: [u8; 4] = *b"DRCS";

const SCRD_CMD_INITIALIZE: u8 = 0x0a;
const SCRD_CMD_CONTEXT_CREATE_TRACKED: u8 = 0x24;
const SCRD_CMD_CONTEXT_EXTERNALIZE: u8 = 0x13;
const SCRD_CMD_CONTEXT_DELETE: u8 = 0x02;
const SCRD_CMD_VERIFY_POLICY: u8 = 0x03;

const SCRD_INIT_LOG_LEVEL: u8 = 0x28;

const SCRD_POLICY_TOUCHID_ENROLLMENT: &[u8] = b"TouchIdEnrollment";

const SCRD_MAX_LOGICAL: usize = 64;
static_assert!(
    8 + SCRD_ACM_HANDLE_LEN + SCRD_POLICY_TOUCHID_ENROLLMENT.len() + 1 + 1 + 8 <= SCRD_MAX_LOGICAL
);

pub(crate) struct ScrdCommand {
    request: u8,
    payload: [u8; SCRD_MAX_LOGICAL],
    payload_len: usize,
    name: &'static CStr,
}

impl ScrdCommand {
    pub(crate) fn request(&self) -> u8 {
        self.request
    }
    pub(crate) fn payload(&self) -> &[u8] {
        &self.payload[..self.payload_len]
    }
    pub(crate) fn name(&self) -> &'static CStr {
        self.name
    }
}

fn scrd_header(out: &mut [u8; SCRD_MAX_LOGICAL], command: u8, byte5: u8, version: u8) {
    out[0..4].copy_from_slice(&SCRD_MAGIC);
    out[4] = command;
    out[5] = byte5;
    out[6] = 0;
    out[7] = version;
}

pub(crate) fn scrd_initialize() -> ScrdCommand {
    let mut payload = [0u8; SCRD_MAX_LOGICAL];
    scrd_header(&mut payload, SCRD_CMD_INITIALIZE, SCRD_INIT_LOG_LEVEL, 0);
    ScrdCommand {
        request: SCRD_REQUEST_COMMAND,
        payload,
        payload_len: 8,
        name: c"SCRD_INITIALIZE",
    }
}

pub(crate) fn scrd_context_create_tracked(session_uid: i32) -> ScrdCommand {
    let mut payload = [0u8; SCRD_MAX_LOGICAL];
    scrd_header(&mut payload, SCRD_CMD_CONTEXT_CREATE_TRACKED, 0, 1);
    payload[8..12].copy_from_slice(&session_uid.to_le_bytes());
    ScrdCommand {
        request: SCRD_REQUEST_COMMAND,
        payload,
        payload_len: 12,
        name: c"SCRD_CONTEXT_CREATE",
    }
}

pub(crate) fn scrd_context_externalize(handle: &[u8; SCRD_ACM_HANDLE_LEN]) -> ScrdCommand {
    let mut payload = [0u8; SCRD_MAX_LOGICAL];
    scrd_header(&mut payload, SCRD_CMD_CONTEXT_EXTERNALIZE, 0, 1);
    payload[8..8 + SCRD_ACM_HANDLE_LEN].copy_from_slice(handle);
    ScrdCommand {
        request: SCRD_REQUEST_COMMAND,
        payload,
        payload_len: 8 + SCRD_ACM_HANDLE_LEN,
        name: c"SCRD_EXTERNALIZE",
    }
}

pub(crate) fn scrd_context_delete(handle: &[u8; SCRD_ACM_HANDLE_LEN]) -> ScrdCommand {
    let mut payload = [0u8; SCRD_MAX_LOGICAL];
    scrd_header(&mut payload, SCRD_CMD_CONTEXT_DELETE, 0, 1);
    payload[8..8 + SCRD_ACM_HANDLE_LEN].copy_from_slice(handle);
    ScrdCommand {
        request: SCRD_REQUEST_COMMAND,
        payload,
        payload_len: 8 + SCRD_ACM_HANDLE_LEN,
        name: c"SCRD_CONTEXT_DELETE",
    }
}

pub(crate) fn scrd_verify_touchid_enrollment(handle: &[u8; SCRD_ACM_HANDLE_LEN]) -> ScrdCommand {
    let mut payload = [0u8; SCRD_MAX_LOGICAL];
    scrd_header(&mut payload, SCRD_CMD_VERIFY_POLICY, 0, 1);
    let mut n = 8;
    payload[n..n + SCRD_ACM_HANDLE_LEN].copy_from_slice(handle);
    n += SCRD_ACM_HANDLE_LEN;
    payload[n..n + SCRD_POLICY_TOUCHID_ENROLLMENT.len()]
        .copy_from_slice(SCRD_POLICY_TOUCHID_ENROLLMENT);
    n += SCRD_POLICY_TOUCHID_ENROLLMENT.len();
    // then NUL, preflight(1), u32 flags, u32 param count -- all zero
    n += 1 + 1 + 4 + 4;
    ScrdCommand {
        request: SCRD_REQUEST_COMMAND,
        payload,
        payload_len: n,
        name: c"SCRD_VERIFY_POLICY",
    }
}

pub(crate) struct ScrdReply {
    pub(crate) request: u8,
    pub(crate) response_size: u16,
    pub(crate) status: i32,
}

pub(crate) fn decode_scrd_reply(msg: &Message) -> ScrdReply {
    let b = msg.msg0.to_le_bytes();
    ScrdReply {
        request: b[1],
        response_size: u16::from_le_bytes([b[2], b[3]]),
        status: i32::from_le_bytes([b[4], b[5], b[6], b[7]]),
    }
}

pub(crate) fn encode_scrd(request: u8, len: usize) -> Message {
    let len = (len & 0xffff) as u16;
    let len = len.to_le_bytes();
    Message {
        msg0: u64::from_le_bytes([EP_SCRD, request, len[0], len[1], 0, 0, 0, 0]),
        msg1: 0,
    }
}

static_assert!(EP_SCRD == 0x0a);
static_assert!(SCRD_CMD_INITIALIZE == 0x0a);
static_assert!(SCRD_CMD_CONTEXT_CREATE_TRACKED == 0x24);
static_assert!(SCRD_CMD_CONTEXT_EXTERNALIZE == 0x13);
static_assert!(SCRD_CMD_VERIFY_POLICY == 0x03);
