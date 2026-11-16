// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Whether a pair's retained render groups, retired by firmware without executing, are
//! published again. Only the whole set is replayed, so no replayed group runs after a live
//! successor, and work a recovery blamed is never run again.

/// One retained group, front first.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) struct Group {
    /// Published, not failed and not classified guilty by a recovery.
    pub(crate) innocent: bool,
    /// A recovery blamed it, whatever class it kept (an earlier recovery may have spared it):
    /// it may have caused the fault.
    pub(crate) blamed: bool,
    /// Carries the strict abandonment witness.
    pub(crate) abandoned: bool,
    /// Already republished the maximum number of times.
    pub(crate) exhausted: bool,
    /// Retired by firmware after executing in full.
    pub(crate) ran: bool,
    /// Retired by firmware unexecuted, but never to run again: a recovery interrupted it after
    /// its tiler started, so it may have run partly, or a command of another queue waited on it,
    /// and that wait ended when firmware retired the group, before its output existed.
    pub(crate) lost: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(crate) enum Decision {
    Replay,
    /// A group may still execute: look again later.
    Wait,
    /// Never replay this set; the render timeout settles it.
    Refuse,
    /// Firmware has retired every group, but a dropped group cannot run again in order: a later
    /// group already ran, or the dropped group had started or had a consumer. Fail the set now.
    Fail,
}

/// Groups are read in order and the first one that decides stops the walk, so a caller can
/// observe the abandonment lazily. An exhausted front is replayed: the replay fails it, and its
/// successors, which consume its output, with it. An exhausted successor would break the order.
///
/// After a restart the firmware retires the groups it had started on a pair without running
/// them, while groups it had not yet started run as usual. Once such a later group has run, a
/// replay of the dropped ones would run them after it; a group that had started cannot run
/// twice; and a group another queue waited on may already have been read before it ran. None of
/// these is replayed: the whole set fails at once instead of waiting for the render timeout. A
/// group that ran ahead of every dropped one is left to in-order retirement.
pub(crate) fn decide(groups: impl Iterator<Item = Group>) -> Decision {
    let mut front_exhausted = None;
    let mut successor_exhausted = false;
    let mut dropped = false;
    let mut unordered = false;
    for group in groups {
        if !group.innocent || group.blamed {
            return Decision::Refuse;
        }
        if group.ran {
            if !dropped {
                return Decision::Wait;
            }
            unordered = true;
            continue;
        }
        if group.lost {
            dropped = true;
            unordered = true;
            continue;
        }
        if !group.abandoned {
            return Decision::Wait;
        }
        dropped = true;
        match front_exhausted {
            None => front_exhausted = Some(group.exhausted),
            Some(_) => successor_exhausted |= group.exhausted,
        }
    }
    if unordered {
        return Decision::Fail;
    }
    match front_exhausted {
        Some(true) => Decision::Replay,
        Some(false) if !successor_exhausted => Decision::Replay,
        _ => Decision::Refuse,
    }
}

#[cfg_attr(not(test), kernel::macros::kunit_tests(asahi_g17_render_replay))]
mod tests {
    use super::*;
    fn group(innocent: bool, abandoned: bool, exhausted: bool) -> Group {
        Group { innocent, blamed: false, abandoned, exhausted, ran: false, lost: false }
    }
    const OK: Group =
        Group { innocent: true, blamed: false, abandoned: true, exhausted: false, ran: false, lost: false };
    const BLAMED: Group = Group { blamed: true, ..OK };
    const RAN: Group = Group { abandoned: false, ran: true, ..OK };
    const LOST: Group = Group { abandoned: false, lost: true, ..OK };
    #[test]
    fn every_group_abandoned_and_innocent_is_replayed() {
        assert_eq!(decide([OK].into_iter()), Decision::Replay);
        assert_eq!(decide([OK, OK, OK].into_iter()), Decision::Replay);
    }
    #[test]
    fn nothing_retained_is_never_replayed() {
        assert_eq!(decide([].into_iter()), Decision::Refuse);
    }
    #[test]
    fn guilty_work_is_never_replayed() {
        assert_eq!(decide([group(false, true, false)].into_iter()), Decision::Refuse);
        assert_eq!(decide([OK, group(false, true, false)].into_iter()), Decision::Refuse);
    }
    #[test]
    fn blamed_work_is_never_replayed_even_when_spared() {
        // An earlier recovery spared it, so it still counts as innocent; the blame refuses it.
        assert_eq!(decide([BLAMED].into_iter()), Decision::Refuse);
        assert_eq!(decide([BLAMED, OK].into_iter()), Decision::Refuse);
        assert_eq!(decide([OK, BLAMED].into_iter()), Decision::Refuse);
        // Also once the replay limit is reached, where an innocent front alone is replayed.
        let exhausted = Group { exhausted: true, ..BLAMED };
        assert_eq!(decide([exhausted].into_iter()), Decision::Refuse);
    }
    #[test]
    fn a_live_group_waits_and_stops_the_walk() {
        assert_eq!(decide([OK, group(true, false, false)].into_iter()), Decision::Wait);
        // Nothing after the live group is looked at, guilty or not.
        assert_eq!(
            decide([group(true, false, false), group(false, true, false)].into_iter()),
            Decision::Wait
        );
        let mut seen = 0;
        let groups = [group(true, false, false), OK, OK].into_iter().inspect(|_| seen += 1);
        assert_eq!(decide(groups), Decision::Wait);
        assert_eq!(seen, 1);
    }
    #[test]
    fn an_exhausted_successor_refuses_an_exhausted_front_replays() {
        assert_eq!(decide([OK, group(true, true, true)].into_iter()), Decision::Refuse);
        assert_eq!(decide([group(true, true, true), OK].into_iter()), Decision::Replay);
        assert_eq!(
            decide([group(true, true, true), group(true, true, true)].into_iter()),
            Decision::Replay
        );
    }
    #[test]
    fn a_group_that_ran_after_dropped_ones_fails_the_set() {
        // Seen after restarts: the started groups were dropped, the queued ones ran.
        assert_eq!(decide([OK, OK, RAN, RAN].into_iter()), Decision::Fail);
        assert_eq!(decide([OK, RAN, OK].into_iter()), Decision::Fail);
        // Also with an exhausted front, which a replay would only fail later.
        assert_eq!(decide([group(true, true, true), RAN].into_iter()), Decision::Fail);
    }
    #[test]
    fn a_group_that_ran_first_is_left_to_retirement() {
        assert_eq!(decide([RAN].into_iter()), Decision::Wait);
        assert_eq!(decide([RAN, OK].into_iter()), Decision::Wait);
    }
    #[test]
    fn a_started_dropped_group_fails_the_set() {
        assert_eq!(decide([OK, LOST].into_iter()), Decision::Fail);
        assert_eq!(decide([LOST, OK].into_iter()), Decision::Fail);
    }
    #[test]
    fn failing_still_waits_for_live_groups_and_refuses_guilty_ones() {
        assert_eq!(decide([OK, RAN, group(true, false, false)].into_iter()), Decision::Wait);
        assert_eq!(decide([OK, RAN, group(false, true, false)].into_iter()), Decision::Refuse);
        assert_eq!(decide([LOST, BLAMED].into_iter()), Decision::Refuse);
    }
}
