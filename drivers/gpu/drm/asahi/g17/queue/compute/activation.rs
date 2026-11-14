// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! A retained M5 channel belongs to its current priority ring, not every ring it
//! has previously used. A priority change must submit it to the new ring again,
//! including when a later owner returns to an earlier priority.

pub(super) struct Publication {
    pub(super) first: bool,
    /// Refresh firmware's per-QID scheduling state on a retained class change.
    pub(super) refresh: bool,
    pub(super) next_started: u8,
}

impl Publication {
    pub(super) fn new(started: u8, priority: u8, m5: bool) -> Self {
        let ring = 1u8 << priority;
        Self {
            first: started & ring == 0,
            refresh: m5 && started != 0 && started & ring == 0,
            next_started: if m5 { ring } else { started | ring },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Publication;

    #[test]
    fn first_publication_activates() {
        for priority in 0..4 {
            let publication = Publication::new(0, priority, true);
            assert!(publication.first);
            assert!(!publication.refresh);
            assert_eq!(publication.next_started, 1 << priority);
        }
    }

    #[test]
    fn same_priority_keeps_fast_path() {
        for priority in 0..4 {
            let publication = Publication::new(1 << priority, priority, true);
            assert!(!publication.first);
            assert!(!publication.refresh);
            assert_eq!(publication.next_started, 1 << priority);
        }
    }

    #[test]
    fn returning_to_old_priority_reactivates() {
        for old in 0..4 {
            for new in 0..4 {
                if old == new { continue; }
                let changed = Publication::new(1 << old, new, true);
                assert!(changed.first);
                assert!(changed.refresh);
                assert_eq!(changed.next_started, 1 << new);
                let returned = Publication::new(changed.next_started, old, true);
                assert!(returned.first);
                assert!(returned.refresh);
                assert_eq!(returned.next_started, 1 << old);
            }
        }
    }

    #[test]
    fn non_m5_preserves_historical_membership() {
        let changed = Publication::new(1 << 2, 3, false);
        assert!(changed.first);
        assert!(!changed.refresh);
        assert_eq!(changed.next_started, (1 << 2) | (1 << 3));
        let returned = Publication::new(changed.next_started, 2, false);
        assert!(!returned.first);
        assert!(!returned.refresh);
        assert_eq!(returned.next_started, changed.next_started);
    }
}
