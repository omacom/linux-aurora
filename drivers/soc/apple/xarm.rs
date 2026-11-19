// SPDX-License-Identifier: GPL-2.0-only OR MIT
// Copyright 2026 Dj

use crate::store::crc16_ccitt_false;
use crate::xarm::{XarmReply as Reply, XarmRequest as Request};
use crate::xart_store::{Key, Store, MAX_VALUE};
use kernel::prelude::*;
use kernel::soc::apple::mailbox::Message;

pub(crate) use crate::proto::EP_XARM;

const OP_ROOT_READ: u8 = 0x00;
const OP_ROOT_WRITE: u8 = 0x01;
const OP_SESSION_READ: u8 = 0x05;
const OP_SESSION_WRITE: u8 = 0x06;
const OP_SESSION_DELETE: u8 = 0x07;
pub(crate) const OP_QUERY_PROTECTED: u8 = 0x0e;
const OP_GET_OS_UUID: u8 = 0x13;
const OP_NOTIFY_DISABLE_FIRST: u8 = 0x1d;
const OP_NOTIFY_DISABLE_LAST: u8 = 0x1f;

pub(crate) const STATUS_OK: u8 = 0x00;
const STATUS_UNAVAILABLE: u8 = 0x02;
pub(crate) const STATUS_FAILED: u8 = 0x16;

const ROOT_TYPE_BASE: u8 = 1;
const SESSION_TYPE_BASE: u8 = 3;

/// Root read prefixes its value with seventeen zero bytes.
const ROOT_READ_PREFIX: usize = 17;

/// `0x01` rejects when `(length >> 4) >= 0x7ff`.
const ROOT_WRITE_MAX: usize = 0x7fef;

impl Reply {
    fn ok(req: &Request, length: u16) -> Reply {
        Reply {
            tag: req.tag,
            status: STATUS_OK,
            length,
            args: [0; 3],
        }
    }

    fn fail(req: &Request, status: u8) -> Reply {
        Reply {
            tag: req.tag,
            status,
            length: 0,
            args: [0; 3],
        }
    }
}

pub(crate) fn is_silent(opcode: u8) -> bool {
    (OP_NOTIFY_DISABLE_FIRST..=OP_NOTIFY_DISABLE_LAST).contains(&opcode)
}

pub(crate) fn needs_buffers(opcode: u8) -> bool {
    opcode != OP_QUERY_PROTECTED
}

fn root_key(args: &[u8; 3]) -> Key {
    Key::root(ROOT_TYPE_BASE + (args[0] & 1))
}

fn session_type(args: &[u8; 3]) -> u8 {
    SESSION_TYPE_BASE + (args[0] & 1)
}

fn expected_crc(args: &[u8; 3]) -> u16 {
    u16::from_le_bytes([args[1], args[2]])
}

fn uuid_from(bytes: &[u8]) -> [u8; 16] {
    let mut uuid = [0u8; 16];
    uuid.copy_from_slice(&bytes[..16]);
    uuid
}

pub(crate) struct Serviced {
    pub(crate) reply: Reply,
    pub(crate) reply_bytes: usize,
}

/// Requests that change the store: a refusal of one means the enclave's state
/// was not persisted.
pub(crate) fn is_write(opcode: u8) -> bool {
    matches!(opcode, OP_ROOT_WRITE | OP_SESSION_WRITE | OP_SESSION_DELETE)
}

pub(crate) fn service(
    req: &Request,
    outbound: &[u8],
    inbound: &mut [u8],
    store: &mut Store,
    protected_data: bool,
    os_uuid: Option<[u8; 16]>,
) -> Serviced {
    match req.opcode {
        OP_ROOT_READ => {
            let key = root_key(&req.args);
            let value = match store.read(&key) {
                Ok(v) => v,
                Err(_) => {
                    return Serviced {
                        reply: Reply::fail(req, STATUS_FAILED),
                        reply_bytes: 0,
                    }
                }
            };

            let Some(value) = value else {
                return Serviced {
                    reply: Reply::ok(req, 0),
                    reply_bytes: 0,
                };
            };
            let total = ROOT_READ_PREFIX + value.len();
            if total > inbound.len() {
                return Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                };
            }
            inbound[..ROOT_READ_PREFIX].fill(0);
            inbound[ROOT_READ_PREFIX..total].copy_from_slice(&value);

            Serviced {
                reply: Reply::ok(req, total as u16),
                reply_bytes: total,
            }
        }

        OP_ROOT_WRITE => {
            if (req.length as usize) > ROOT_WRITE_MAX || outbound.len() > MAX_VALUE {
                return Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                };
            }
            if crc16_ccitt_false(outbound) != expected_crc(&req.args) {
                return Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                };
            }
            match store.write(&root_key(&req.args), outbound) {
                Ok(()) => Serviced {
                    reply: Reply::ok(req, 0),
                    reply_bytes: 0,
                },
                Err(_) => Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                },
            }
        }

        OP_SESSION_READ => {
            if outbound.len() != 16 {
                return Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                };
            }
            let key = Key::new(session_type(&req.args), uuid_from(outbound));
            match store.read(&key) {
                Ok(Some(value)) => {
                    if value.len() > inbound.len() {
                        return Serviced {
                            reply: Reply::fail(req, STATUS_FAILED),
                            reply_bytes: 0,
                        };
                    }
                    inbound[..value.len()].copy_from_slice(&value);
                    // Only a read that hands the enclave the record counts,
                    // for the lockers the guard lets it rewrite under
                    // xart_guard_pins.
                    store.note_enclave_read(&key);
                    Serviced {
                        reply: Reply::ok(req, value.len() as u16),
                        reply_bytes: value.len(),
                    }
                }
                Ok(None) => Serviced {
                    reply: Reply::ok(req, 0),
                    reply_bytes: 0,
                },
                Err(_) => Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                },
            }
        }

        OP_SESSION_WRITE => {
            if outbound.len() < 16 {
                return Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                };
            }
            if crc16_ccitt_false(outbound) != expected_crc(&req.args) {
                return Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                };
            }
            let key = Key::new(session_type(&req.args), uuid_from(outbound));
            match store.write(&key, &outbound[16..]) {
                Ok(()) => Serviced {
                    reply: Reply::ok(req, 0),
                    reply_bytes: 0,
                },
                Err(_) => Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                },
            }
        }

        OP_SESSION_DELETE => {
            if outbound.len() < 16 {
                return Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                };
            }
            let key = Key::new(session_type(&req.args), uuid_from(outbound));
            match store.delete(&key) {
                Ok(true) => Serviced {
                    reply: Reply::ok(req, 0),
                    reply_bytes: 0,
                },
                Ok(false) => Serviced {
                    reply: Reply::ok(req, 0),
                    reply_bytes: 0,
                },
                Err(_) => Serviced {
                    reply: Reply::fail(req, STATUS_FAILED),
                    reply_bytes: 0,
                },
            }
        }

        OP_QUERY_PROTECTED => {
            let mut reply = Reply::ok(req, 0);
            reply.args[0] = u8::from(protected_data);
            Serviced {
                reply,
                reply_bytes: 0,
            }
        }

        OP_GET_OS_UUID => match os_uuid {
            Some(uuid) => {
                if inbound.len() < 16 {
                    return Serviced {
                        reply: Reply::fail(req, STATUS_FAILED),
                        reply_bytes: 0,
                    };
                }
                inbound[..16].copy_from_slice(&uuid);
                // Reply length must echo the request, not the 16 flushed bytes; setting 16 is wrong.
                Serviced {
                    reply: Reply::ok(req, req.length),
                    reply_bytes: 16,
                }
            }
            None => Serviced {
                reply: Reply::fail(req, STATUS_UNAVAILABLE),
                reply_bytes: 0,
            },
        },

        _ => Serviced {
            reply: Reply::fail(req, STATUS_FAILED),
            reply_bytes: 0,
        },
    }
}

pub(crate) struct XarmRequest {
    pub(crate) tag: u8,
    pub(crate) opcode: u8,
    pub(crate) length: u16,
    pub(crate) args: [u8; 3],
}

pub(crate) struct XarmReply {
    pub(crate) tag: u8,
    pub(crate) status: u8,
    pub(crate) length: u16,
    pub(crate) args: [u8; 3],
}

pub(crate) fn decode_xarm(msg: &Message) -> XarmRequest {
    let b = msg.msg0.to_le_bytes();
    XarmRequest {
        tag: b[1],
        opcode: b[2],
        length: u16::from_le_bytes([b[3], b[4]]),
        args: [b[5], b[6], b[7]],
    }
}

pub(crate) fn encode_xarm_reply(reply: &XarmReply) -> Message {
    let len = reply.length.to_le_bytes();
    Message {
        msg0: u64::from_le_bytes([
            EP_XARM,
            reply.tag,
            reply.status,
            len[0],
            len[1],
            reply.args[0],
            reply.args[1],
            reply.args[2],
        ]),
        msg1: 0,
    }
}

static_assert!(EP_XARM == 0x13);
