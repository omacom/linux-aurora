// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Host-only ordering of timestamp writes, independent of scheduler credits.
//!
//! Links point from earlier logical render ordinals to later ones. They retain
//! only public output state, never a packet, physical render lease or VM job pin.
//! All link storage is allocated before acceptance. Completion neither allocates
//! nor waits for another command; the last prerequisite publishes a ready result.

use super::Output;
use crate::g17::{dependency::inherited_error, status::VmStatus};
use kernel::{new_mutex, prelude::*, sync::{Arc, Mutex}};

type Links = Option<KBox<Link>>;

struct Link {
    target: Arc<Publication>,
    result: Result,
    next: Links,
    // Reuse an already-consumed link as a traversal frame instead of recursing
    // or allocating when publication unlocks a long chain of ready successors.
    resume: Links,
}

struct State {
    remaining: usize,
    result: Option<Result<[u64; 4]>>,
    predecessor_error: Option<Error>,
    claimed: bool,
    published: Option<Result>,
    successors: Links,
}

impl State {
    fn claim(&mut self) -> Option<Result<[u64; 4]>> {
        if self.remaining != 0 || self.claimed {
            return None;
        }
        let result = self.result.take()?;
        self.claimed = true;
        Some(match (result, self.predecessor_error) {
            (Ok(_), Some(error)) => Err(error),
            (result, _) => result,
        })
    }
}

#[pin_data]
pub(crate) struct Publication {
    output: Output,
    status: Arc<VmStatus>,
    #[pin]
    state: Mutex<State>,
}

impl Publication {
    pub(super) fn new(output: Output, status: Arc<VmStatus>) -> Result<Arc<Self>> {
        Arc::pin_init(
            pin_init!(Self {
                output,
                status,
                state <- new_mutex!(State {
                    // Acceptance installs all links before releasing this sentinel.
                    remaining: 1,
                    result: None,
                    predecessor_error: None,
                    claimed: false,
                    published: None,
                    successors: None,
                }, "G17 host timestamp publication"),
            }),
            GFP_KERNEL,
        )
    }

    pub(super) fn output(&self) -> &Output {
        &self.output
    }

    /// One terminal result, offered separately from its physical-retirement proof.
    pub(super) fn finish(&self, result: Result<[u64; 4]>) {
        let ready = {
            let mut state = self.state.lock();
            if state.claimed || state.result.is_some() {
                return;
            }
            state.result = Some(result);
            state.claim()
        };
        if let Some(result) = ready {
            Self::drain(self.publish(result));
        }
    }

    fn publish(&self, result: Result<[u64; 4]>) -> Links {
        if let Err(error) = result {
            // Inherited failure is visible to the client without inventing a
            // permanent admission failure in this command's VM.
            if error != ECANCELED {
                self.status.report_failure(error);
            }
        }
        self.output.publish(result);
        let result = result.map(|_| ());
        let mut links = {
            let mut state = self.state.lock();
            // A late link is acknowledged only after timestamp writes, the public
            // fence and aggregate membership have all been published off this lock.
            state.published = Some(result);
            state.successors.take()
        };
        let mut next = links.as_mut();
        while let Some(link) = next {
            link.result = result;
            next = link.next.as_mut();
        }
        links
    }

    fn predecessor_done(&self, result: Result) -> Links {
        let ready = {
            let mut state = self.state.lock();
            debug_assert!(state.remaining != 0);
            if let Err(error) = result {
                if state.predecessor_error.is_none() {
                    state.predecessor_error = Some(inherited_error(error));
                }
            }
            state.remaining -= 1;
            state.claim()
        };
        ready.and_then(|result| self.publish(result))
    }

    fn add_successor(&self, mut link: KBox<Link>) {
        let published = {
            let mut state = self.state.lock();
            match state.published {
                Some(result) => result,
                None => {
                    link.next = state.successors.take();
                    state.successors = Some(link);
                    return;
                }
            }
        };
        Self::drain(link.target.predecessor_done(published));
    }

    /// Iterative depth-first notification, O(nodes + links), with no stack growth.
    fn drain(mut pending: Links) {
        let mut frames: Links = None;
        loop {
            if let Some(mut link) = pending.take() {
                pending = link.next.take();
                let children = link.target.predecessor_done(link.result);
                if children.is_some() {
                    link.resume = pending.take();
                    link.next = frames.take();
                    frames = Some(link);
                    pending = children;
                }
            } else if let Some(mut frame) = frames.take() {
                frames = frame.next.take();
                pending = frame.resume.take();
            } else {
                break;
            }
        }
    }
}

/// Temporary backwards references exist only during fallible setup. Installed
/// links retain the successor only, so closing a queue cannot form an Arc cycle.
pub(crate) struct Prepared {
    target: Arc<Publication>,
    links: KVec<(Arc<Publication>, KBox<Link>)>,
}

impl Prepared {
    pub(crate) fn new<'a>(
        target: Arc<Publication>,
        predecessors: impl Iterator<Item = &'a Arc<Publication>> + Clone,
    ) -> Result<Self> {
        let mut links = KVec::with_capacity(predecessors.clone().count(), GFP_KERNEL)?;
        for predecessor in predecessors {
            let link = KBox::new(Link {
                target: target.clone(),
                result: Ok(()),
                next: None,
                resume: None,
            }, GFP_KERNEL)?;
            links.push((predecessor.clone(), link), GFP_KERNEL)?;
        }
        Ok(Self { target, links })
    }

    /// Called after all fallible scheduler setup and before any GPU publication.
    pub(crate) fn activate(self) {
        {
            let mut state = self.target.state.lock();
            debug_assert!(state.remaining == 1 && !state.claimed);
            // The fully allocated link vector bounds this count below usize::MAX.
            state.remaining += self.links.len();
        }
        for (predecessor, link) in self.links {
            predecessor.add_successor(link);
        }
        Publication::drain(self.target.predecessor_done(Ok(())));
    }
}
