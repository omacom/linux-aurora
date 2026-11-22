G17 repeated-timeout containment
===============================

The 73f audit found workload-reachable host-liveness hazards. The journal
alone cannot attribute the observed machine reset to one of them; no hardware
reproduction is claimed here.

* Compute replay called Packet::check_dependencies with the device mutex
  held. That routine used MAX_SCHEDULE_TIMEOUT. If an input needed completion
  service taking the same mutex, the event worker could deadlock. Replay now
  checks statuses without sleeping and fails an unexpectedly unmet dependency.
  Initial publication waits off-lock in 10 ms cancellation/status-check slices,
  with one absolute 2 s budget (10 s for a selected long submission).
* G17 mailbox receive drained an indefinitely replenished FIFO under rx_lock
  with IRQs disabled. G17 now uses a oneshot threaded receive IRQ, consumes at
  most 64 messages per pass, then sleeps 1-2 ms off-lock with the IRQ masked
  when its budget is exhausted. The level condition retriggers without
  discarding messages. Explicit receive polling has the same finite budget.
* G17 mailbox transmit without a send-empty IRQ could busy-poll for 500 ms
  under tx_lock with IRQs disabled. Non-atomic G17 callers now sleep off-lock
  between checks; an atomic G17 send polls for at most 1 ms. All sleeping
  retries share a single 500 ms deadline, including contention between senders.
  Removal, cookie, provider lifecycle and PM ownership gates are retained.
* Scheduler failure callbacks no longer synchronously join preparation leases
  and RTKit receive work. They mark the runtime lost and queue the event worker.
  Recovery joins preparation for at most 2 s. Runtime stop halts both firmware
  producers first and joins preparation for at most 2 s before detachment.
  If a builder remains stalled, its stopped firmware, graphs and reservations
  remain owned in Shared::state. Timeout is never an ownership witness. That
  path does not re-arm a polling loop; a later queued completion/owner return
  may finish the stopped teardown. Explicit device shutdown still joins every
  borrower before returning or releasing provider resources.
* Borrowed render graphs and off-lock queued retirements previously renewed
  the watchdog without incrementing a render deferral counter. Both engines
  now charge deferrals, and render transaction grace ends after eight polls.
  Existing three replay credits remain finite and are never reset here.
* Idle compute-pool releases have no scheduler job once their client is lost.
  Record a timestamp on each exact release publication; a missing consumer
  witness after 10 s stops the runtime while retaining the backing. Parked
  render replays similarly expire after 10 s rather than polling forever when
  admission or publication cannot proceed. Wakes/new clients do not refresh
  either transaction timestamp; exact completion clears it.
* A per-device burst limit stops the runtime on the eighth repeated scheduler
  timeout in a 30 s window or the eighth restart handshake in a 30 s window.
  A command's first expiry is charged to its own client, whose work it fails
  (or spares behind another VM's), and does not count: otherwise one process
  could spend the budget with one hung command per queue. Only an expiry of a
  command whose earlier settlement did not end it counts. Likewise a restart
  is counted only once it is classified, and not when it is charged to the VMs
  it finds guilty: each found guilty for the first time, which costs its client
  the device, or blamed again for no more restarts than the commands it held
  then. A restart without a guilty VM, or one that keeps blaming a VM beyond
  that, counts. VM changes, new publications, recovery generations and replay
  do not reset those counters; terminal loss latches. The 30 s window holds
  the nine two-second expirations of a stalled deferred timeout that cannot
  complete its normal per-client settlement: its first, then the eight that
  count. This limits host recovery load even when each individual handshake
  succeeds. Legitimate long jobs only consume this budget after their
  independent bounded execution allowance expires.

Lock order remains stop -> state and state -> preparation wait/reclaim. All
preparation joins and deferred fence completion/destruction run outside state.
Runtime stop uses try_lock for the stop owner, avoiding a recursive join from
an off-lock fence callback. Receive callbacks only retain/copy ingress and
queue work; no firmware ring is polled while an IRQ spinlock is held without
a finite iteration/time bound on the G17 paths. RX acknowledgement is still
level-sensitive, and TX publication/removal remains serialized by tx_lock.

The existing control backpressure timer has 1,000 finite retries; its event
probes do not consume or reset that budget. Event-ring ingress is bounded by
EVENT_SLOTS; trace/statistics consume a snapshot producer. Render retirement
loops remove one retained tracker per successful iteration (or mark one as
accounted); compute replay consumes a retained replay per iteration. Shared
capacity deadlines remain absolute and failed fences do not release mappings
or return physical reservations. The storm limits additionally bound
repeated recoveries and timeouts across those paths.

KUnit exercises 64 consecutive watchdog timeouts with recovery pending under
real state/preparation mutexes, a retained lease that must time out rather
than be freed, late lease return with failed admission still closed, restart
storms, the published-expiry record and nonblocking replay status checks.
The mailbox tests use normal RAM
as never-empty RX and never-draining TX registers, invoking the production
poll/send functions for 64 RX passes and eight atomic TX timeouts plus a
sleeping TX timeout/removal control. The isolated Asahi scenario runs with
lockdep, PROVE_LOCKING and panic_on_warn. Software tests cannot establish
that physical fabric/firmware or the original reset is fixed; separately
controller-gated Neo qualification remains necessary.
