// SPDX-License-Identifier: GPL-2.0-only OR MIT

//! Admission of commands beside a command that traces rays, on firmware that restarts the GPU
//! or fails a compute command when another client's compute or ray-traversal command arrives
//! beside one ([`super::fw::Variant::traversal_exclusion`]). A client is a VM.
//!
//! - Commands that trace rays run for one client at a time; the first client refused goes next.
//! - A compute command that traces no rays waits while another client's ray-traversal command
//!   is admitted and unfinished. The first that waited keeps other clients' further
//!   ray-traversal commands out until it has finished, which bounds its wait.
//!
//! The same firmware loses or mixes up mesh output when two clients' renders with mesh shading
//! run on the GPU together, so the scheduler admits those renders like commands that trace
//! rays (`runtime/traversal.rs`, `aurora.g17_mesh_exclusion`).
//!
//! Other renders never wait. Only arrivals are ordered: a command already on its way to the GPU
//! is not recalled. The state is plain data over job handles, so it builds on the host; the
//! scheduler side is `runtime/traversal.rs`.

/// Ray-traversal commands of one client admitted at a time; with more, it waits for one to end.
const HELD: usize = 16;

/// Admission state over job handles `H`. `done` says whether a handle's job has ended; ended
/// jobs are forgotten first.
pub(crate) struct Admission<H> {
    /// The client of the admitted ray-traversal commands; meaningful while one is held.
    client: u64,
    /// The admitted ray-traversal commands that had not ended when last looked at.
    held: [Option<H>; HELD],
    /// The first ray-traversal command refused for another client's: its client is next.
    next: Option<(u64, H)>,
    /// The first compute command that had to wait, and its client: it runs before another
    /// client's ray-traversal command is admitted again.
    turn: Option<(u64, H)>,
}

impl<H: Clone + PartialEq> Admission<H> {
    pub(crate) fn new() -> Self {
        Self { client: 0, held: core::array::from_fn(|_| None), next: None, turn: None }
    }

    /// Whether anything is admitted or waiting.
    pub(crate) fn busy(&self) -> bool {
        self.next.is_some() || self.turn.is_some() || self.held.iter().any(Option::is_some)
    }

    fn sweep(&mut self, done: &impl Fn(&H) -> bool) {
        for slot in self.held.iter_mut() {
            if slot.as_ref().is_some_and(done) {
                *slot = None;
            }
        }
        // A refused command that ended without its turn (cancelled, or failed before it ran).
        if self.next.as_ref().is_some_and(|(_, job)| done(job)) {
            self.next = None;
        }
        if self.turn.as_ref().is_some_and(|(_, job)| done(job)) {
            self.turn = None;
        }
    }

    fn running(&self) -> Option<&H> {
        self.held.iter().flatten().next()
    }

    /// A command `mine` of `client` that traces rays. `None`: it is admitted and held until
    /// its job ends. `Some(job)`: wait for that job to end, then ask again.
    pub(crate) fn traversal(&mut self, client: u64, mine: &H, done: impl Fn(&H) -> bool) -> Option<H> {
        self.sweep(&done);
        if self.held.iter().flatten().any(|job| job == mine) {
            // Asked again after another wait: already admitted.
            return None;
        }
        if let Some((owner, compute)) = &self.turn {
            if *owner != client {
                // Another client's compute command waited: it runs first.
                return Some(compute.clone());
            }
        }
        let foreign = self.running().filter(|_| self.client != client).cloned();
        if let Some((owner, command)) = &self.next {
            if *owner != client {
                // Another client is next: behind the running work, then behind its command.
                return Some(foreign.unwrap_or_else(|| command.clone()));
            }
        }
        if let Some(running) = foreign {
            if self.next.is_none() {
                self.next = Some((client, mine.clone()));
            }
            return Some(running);
        }
        match self.held.iter_mut().find(|slot| slot.is_none()) {
            Some(slot) => {
                *slot = Some(mine.clone());
                self.client = client;
                // Only this client's own refused command can be in line here.
                self.next = None;
                None
            }
            // HELD commands of this client are in flight already: behind one of them.
            None => self.running().cloned(),
        }
    }

    /// A compute command `mine` of `client` that traces no rays. `None`: it may go on.
    /// `Some(job)`: wait for another client's ray-traversal job to end, then ask again.
    pub(crate) fn compute(&mut self, client: u64, mine: &H, done: impl Fn(&H) -> bool) -> Option<H> {
        self.sweep(&done);
        let running = self.running().filter(|_| self.client != client).cloned()?;
        if self.turn.is_none() {
            self.turn = Some((client, mine.clone()));
        }
        Some(running)
    }
}

#[cfg_attr(not(test), kernel::macros::kunit_tests(asahi_g17_traversal_admission))]
mod tests {
    use super::*;

    const A: u64 = 1;
    const B: u64 = 2;
    const C: u64 = 3;

    /// Jobs are numbers; a job has ended when its bit is set in `ended`.
    fn ended(mask: u64) -> impl Fn(&u32) -> bool {
        move |job| mask & (1 << job) != 0
    }

    #[test]
    fn one_client_s_traversal_commands_run_together() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert_eq!(gate.traversal(A, &2, ended(0)), None);
        // Asking again changes nothing.
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert!(gate.busy());
    }

    #[test]
    fn another_client_s_traversal_command_waits_for_the_running_one() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert_eq!(gate.traversal(B, &2, ended(0)), Some(1));
        // Still running: still refused, on the same job.
        assert_eq!(gate.traversal(B, &2, ended(0)), Some(1));
        // The running command ended: the refused client is admitted.
        assert_eq!(gate.traversal(B, &2, ended(1 << 1)), None);
    }

    #[test]
    fn the_first_refused_client_goes_before_the_holder_s_next_command() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert_eq!(gate.traversal(B, &2, ended(0)), Some(1));
        // The holder's next command is behind B's refused command, not beside its own work.
        assert_eq!(gate.traversal(A, &3, ended(0)), Some(2));
        // A third client is behind the running work.
        assert_eq!(gate.traversal(C, &4, ended(0)), Some(1));
        // A's first command ended: A still waits for B's command, which is admitted next.
        assert_eq!(gate.traversal(A, &3, ended(1 << 1)), Some(2));
        assert_eq!(gate.traversal(B, &2, ended(1 << 1)), None);
        // And A is refused for B's running command, first in line again.
        assert_eq!(gate.traversal(A, &3, ended(1 << 1)), Some(2));
        assert_eq!(gate.traversal(A, &3, ended(1 << 1 | 1 << 2)), None);
    }

    #[test]
    fn a_refused_command_that_ended_gives_up_its_place() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert_eq!(gate.traversal(B, &2, ended(0)), Some(1));
        // B's command was cancelled: A's next command is not held behind it.
        assert_eq!(gate.traversal(A, &3, ended(1 << 2)), None);
    }

    #[test]
    fn a_client_with_the_limit_in_flight_waits_for_one_of_its_own() {
        let mut gate = Admission::<u32>::new();
        for job in 0..HELD as u32 {
            assert_eq!(gate.traversal(A, &job, ended(0)), None);
        }
        assert_eq!(gate.traversal(A, &40, ended(0)), Some(0));
        assert_eq!(gate.traversal(A, &40, ended(1 << 5)), None);
    }

    #[test]
    fn other_clients_compute_waits_for_a_traversal_command() {
        let mut gate = Admission::<u32>::new();
        // Nothing traces rays: compute is never held, and nothing is remembered.
        assert_eq!(gate.compute(B, &9, ended(0)), None);
        assert!(!gate.busy());
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        // The traversal client's own compute commands go on.
        assert_eq!(gate.compute(A, &2, ended(0)), None);
        // Another client's waits for the traversal command.
        assert_eq!(gate.compute(B, &3, ended(0)), Some(1));
        assert_eq!(gate.compute(C, &4, ended(0)), Some(1));
        // The traversal command ended: both go on.
        assert_eq!(gate.compute(C, &4, ended(1 << 1)), None);
        assert_eq!(gate.compute(B, &3, ended(1 << 1)), None);
    }

    #[test]
    fn a_compute_command_that_waited_runs_before_more_traversal_work() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert_eq!(gate.compute(B, &2, ended(0)), Some(1));
        // The traversal client's next command is behind B's compute command.
        assert_eq!(gate.traversal(A, &3, ended(0)), Some(2));
        // The first traversal command ended: the compute command is admitted and runs, and
        // the traversal client keeps waiting for it.
        assert_eq!(gate.compute(B, &2, ended(1 << 1)), None);
        assert_eq!(gate.traversal(A, &3, ended(1 << 1)), Some(2));
        // The compute command ended: traversal work is admitted again.
        assert_eq!(gate.traversal(A, &3, ended(1 << 1 | 1 << 2)), None);
        // Compute of B waits again, and has the turn again.
        assert_eq!(gate.compute(B, &4, ended(1 << 1 | 1 << 2)), Some(3));
    }

    #[test]
    fn a_waiting_compute_command_does_not_hold_its_own_client_s_traversal() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert_eq!(gate.compute(B, &2, ended(0)), Some(1));
        // B's own traversal command is refused for A's running one, not for B's compute.
        assert_eq!(gate.traversal(B, &3, ended(0)), Some(1));
        assert_eq!(gate.traversal(B, &3, ended(1 << 1)), None);
    }

    #[test]
    fn a_waiting_compute_command_that_ended_gives_up_its_turn() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert_eq!(gate.compute(B, &2, ended(0)), Some(1));
        // The compute command was cancelled while it waited.
        assert_eq!(gate.traversal(A, &3, ended(1 << 2)), None);
        // Both traversal commands ended: B's next compute command goes on, nothing is left.
        assert_eq!(gate.compute(B, &5, ended(1 << 1 | 1 << 2 | 1 << 3)), None);
        assert!(!gate.busy());
    }

    #[test]
    fn a_waiting_compute_command_and_a_refused_client_both_get_their_turn() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        // B's traversal command is first in line, C's compute command waited.
        assert_eq!(gate.traversal(B, &2, ended(0)), Some(1));
        assert_eq!(gate.compute(C, &3, ended(0)), Some(1));
        // A's next command is behind C's compute command.
        assert_eq!(gate.traversal(A, &4, ended(0)), Some(3));
        // A's first command ended. C's compute command runs; both traversal clients wait for it.
        let first = 1 << 1;
        assert_eq!(gate.compute(C, &3, ended(first)), None);
        assert_eq!(gate.traversal(B, &2, ended(first)), Some(3));
        assert_eq!(gate.traversal(A, &4, ended(first)), Some(3));
        // It ended: B, first in line, is admitted; A is behind B's command.
        let compute = first | 1 << 3;
        assert_eq!(gate.traversal(A, &4, ended(compute)), Some(2));
        assert_eq!(gate.traversal(B, &2, ended(compute)), None);
        assert_eq!(gate.traversal(A, &4, ended(compute)), Some(2));
        // B's command ended: A is admitted, and nothing else is left.
        assert_eq!(gate.traversal(A, &4, ended(compute | 1 << 2)), None);
        assert_eq!(gate.compute(C, &5, ended(compute | 1 << 2 | 1 << 4)), None);
        assert!(!gate.busy());
    }

    #[test]
    fn everything_ended_leaves_nothing_behind() {
        let mut gate = Admission::<u32>::new();
        assert_eq!(gate.traversal(A, &1, ended(0)), None);
        assert_eq!(gate.traversal(B, &2, ended(0)), Some(1));
        assert_eq!(gate.compute(C, &3, ended(0)), Some(1));
        assert_eq!(gate.compute(C, &3, ended(0b1110)), None);
        assert!(!gate.busy());
    }
}
