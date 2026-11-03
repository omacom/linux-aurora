// Unit test and simulation of pmp-agx-vote.h: the policy that decides when the T6001 AGX request (PS-REQ bit 9) is set and cleared.
// A fake clock, a fake PMP bit and a fake timer drive the same transition functions the kernel glue calls. No kernel, no hardware.
#include <limits.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#include "pmp-agx-vote.h"

struct sim {
	struct agx_vote v;
	uint32_t now;
	bool bit;            // the PMP's PS-REQ bit 9
	int sets, clears;    // register writes that tried to set / clear it
	int fail_sets;       // the next N set attempts are not acknowledged
	bool armed;
	uint32_t timer_at;
	int begin_in_write;  // begin this many operations while the register write of the next reconcile is in progress
};

static void arm(struct sim *s, uint32_t delay_ms)
{
	uint32_t at = s->now + delay_ms;
	if (!s->armed || (int32_t)(at - s->timer_at) < 0) {
		s->armed = true;
		s->timer_at = at;
	}
}

static void begin(struct sim *s);

// What the kernel work function does.
static void reconcile(struct sim *s)
{
	enum agx_vote_action act = agx_vote_decide(&s->v, s->now);
	int ret = 0;

	s->armed = false;
	if (act == AGX_VOTE_SET) {
		s->sets++;
		if (s->fail_sets > 0) {
			s->fail_sets--;
			ret = -110;
			s->bit = false;      // glue force-clears a request that was not acknowledged
		} else {
			s->bit = true;
		}
	} else if (act == AGX_VOTE_CLEAR) {
		s->clears++;
		s->bit = false;
	}
	while (s->begin_in_write > 0) {
		s->begin_in_write--;
		begin(s);
	}
	if (act != AGX_VOTE_NONE)
		agx_vote_applied(&s->v, act, ret, s->now);
	uint32_t next = agx_vote_next(&s->v, s->now);
	if (next)
		arm(s, next);
}

static void begin(struct sim *s)
{
	if (agx_vote_begin(&s->v))
		arm(s, 0);
}

static void end(struct sim *s)
{
	uint32_t d = agx_vote_end(&s->v, s->now);
	if (d)
		arm(s, d);
}

static void advance_to(struct sim *s, uint32_t t)
{
	while (s->armed && (int32_t)(s->timer_at - t) <= 0) {
		if ((int32_t)(s->timer_at - s->now) > 0)
			s->now = s->timer_at;
		reconcile(s);
	}
	s->now = t;
}

static struct sim fresh(bool enabled, unsigned long idle_ms, uint32_t start)
{
	struct sim s = {.now = start};
	s.v.enabled = enabled;
	s.v.idle_ms = idle_ms;
	return s;
}

#define CHECK(cond) do { if (!(cond)) { fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #cond); exit(1); } } while (0)

int main(void)
{
	// 1. parameter off: nothing is ever written, however much the GPU runs
	{
		struct sim s = fresh(false, 100, 1000);
		for (int i = 0; i < 50; i++) { begin(&s); advance_to(&s, s.now + 4); end(&s); advance_to(&s, s.now + 7); }
		advance_to(&s, s.now + 1000);
		CHECK(s.sets == 0 && s.clears == 0 && !s.bit);
	}
	// 2. one operation: set at once, kept for idle_ms, then cleared
	{
		struct sim s = fresh(true, 100, 1000);
		begin(&s); advance_to(&s, 1000);
		CHECK(s.bit && s.sets == 1);
		advance_to(&s, 1004); end(&s);
		advance_to(&s, 1103); CHECK(s.bit);              // 99 ms after the end: still voted
		advance_to(&s, 1104); CHECK(!s.bit && s.clears == 1);   // 100 ms after the end: dropped
	}
	// 3. decode-like bursts every 10.58 ms for one second: one set, no clear until the idle time after the last burst
	{
		struct sim s = fresh(true, 100, 5000);
		uint32_t last_end = 0;
		for (int i = 0; i < 95; i++) {
			uint32_t t0 = 5000 + (uint32_t)(i * 10.58);
			advance_to(&s, t0); begin(&s);
			advance_to(&s, t0 + 4); end(&s); last_end = s.now;
		}
		CHECK(s.sets == 1 && s.clears == 0 && s.bit);
		advance_to(&s, last_end + 99); CHECK(s.bit);
		advance_to(&s, last_end + 100); CHECK(!s.bit && s.clears == 1 && s.sets == 1);
	}
	// 4. work arrives while the drop is pending: the drop is cancelled, then rescheduled from the new end
	{
		struct sim s = fresh(true, 100, 0);
		begin(&s); advance_to(&s, 0); end(&s);
		advance_to(&s, 50); begin(&s);
		advance_to(&s, 100); CHECK(s.bit && s.clears == 0);
		advance_to(&s, 120); end(&s);
		advance_to(&s, 219); CHECK(s.bit);
		advance_to(&s, 220); CHECK(!s.bit && s.clears == 1 && s.sets == 1);
	}
	// 5. a set that is not acknowledged is cleared again, not retried in a hot loop, and retried after the backoff
	{
		struct sim s = fresh(true, 100, 0);
		s.fail_sets = 1;
		begin(&s); advance_to(&s, 0);
		CHECK(s.sets == 1 && !s.bit && !s.v.voted && s.v.failures == 1);
		advance_to(&s, AGX_VOTE_RETRY_MS - 1); CHECK(s.sets == 1);
		advance_to(&s, AGX_VOTE_RETRY_MS); CHECK(s.sets == 2 && s.bit && s.v.voted && s.v.failures == 0);
	}
	// 6. an end without a begin is ignored: no underflow, the next begin still works
	{
		struct sim s = fresh(true, 100, 0);
		end(&s); end(&s);
		CHECK(s.v.users == 0 && !s.armed);
		begin(&s); advance_to(&s, 0);
		CHECK(s.v.users == 1 && s.bit);
	}
	// 7. the parameter is switched off while voted and busy: cleared at the next reconcile, even with work in flight
	{
		struct sim s = fresh(true, 100, 0);
		begin(&s); advance_to(&s, 0); CHECK(s.bit);
		s.v.enabled = false; arm(&s, 0); advance_to(&s, 1);
		CHECK(!s.bit && s.clears == 1 && !s.v.voted);
		s.v.enabled = true; arm(&s, 0); advance_to(&s, 2);       // switched on again with work still in flight
		CHECK(s.bit && s.sets == 2);
	}
	// 8. zero idle time still waits one tick (no clear inside the end call's own tick)
	{
		struct sim s = fresh(true, 0, 0);
		begin(&s); advance_to(&s, 0); end(&s);
		CHECK(s.armed && s.bit);
		advance_to(&s, 1); CHECK(!s.bit);
	}
	// 9. the millisecond clock wraps: idle_at + idle_ms overflows, the decision stays right
	{
		struct sim s = fresh(true, 100, UINT32_MAX - 50);
		begin(&s); advance_to(&s, s.now); end(&s);
		advance_to(&s, s.now + 99); CHECK(s.bit);
		advance_to(&s, s.now + 1); CHECK(!s.bit && s.clears == 1);
	}
	// 10. release (GPU driver unbind): a held vote is cleared once, at once, and the idle timer is not needed afterwards
	{
		struct sim s = fresh(true, 100, 0);
		begin(&s); advance_to(&s, 0); end(&s);
		CHECK(s.bit && s.armed);
		if (agx_vote_release(&s.v)) { s.clears++; s.bit = false; }
		CHECK(!s.bit && s.clears == 1 && !s.v.voted);
		advance_to(&s, 500);                       // the pending timer fires and finds nothing to do
		CHECK(s.clears == 1 && !s.armed);
		CHECK(!agx_vote_release(&s.v));            // a second release writes nothing
	}
	// 11. w7X's reproduction: jiffies_to_msecs is 32 bits wide, so a vote whose last operation ended at 0xFFFFFFF0 must still clear
	// 100 ms later, after the clock has wrapped to 0x54 (a 64-bit compare of idle_at + idle_ms never decided it)
	{
		struct sim s = fresh(true, 100, 0xFFFFFFF0u);
		begin(&s); advance_to(&s, s.now); end(&s);
		CHECK(s.v.idle_at == 0xFFFFFFF0u);
		advance_to(&s, 0xFFFFFFF0u + 99u); CHECK(s.bit && s.now == 0x53u);
		advance_to(&s, 0x54u); CHECK(!s.bit && s.clears == 1);
		CHECK(agx_vote_next(&s.v, s.now) == 0);
	}
	// 12. a retry deadline across the same wrap
	{
		struct sim s = fresh(true, 100, 0xFFFFFC00u);
		s.fail_sets = 1;
		begin(&s); advance_to(&s, s.now);
		CHECK(s.sets == 1 && !s.bit);
		advance_to(&s, 0xFFFFFC00u + AGX_VOTE_RETRY_MS - 1u); CHECK(s.sets == 1);
		advance_to(&s, 0xFFFFFC00u + AGX_VOTE_RETRY_MS); CHECK(s.sets == 2 && s.bit);
	}
	// 13. the shipped default hold: w7X measured gaps of up to 111.5 ms inside one 9B generation (ctx 8192, once per process), over the old
	// 100 ms hold. The default must ride through that gap with one vote, and still drop the vote after a long idle.
	{
		struct sim s = fresh(true, AGX_VOTE_IDLE_MS_DEFAULT, 1000);
		CHECK(AGX_VOTE_IDLE_MS_DEFAULT >= 250);
		begin(&s); advance_to(&s, s.now); end(&s);
		advance_to(&s, s.now + 112);               // the 111.5 ms interval rounded up
		begin(&s); advance_to(&s, s.now); end(&s);
		CHECK(s.sets == 1 && s.clears == 0 && s.bit);
		advance_to(&s, s.now + AGX_VOTE_IDLE_MS_DEFAULT - 1); CHECK(s.bit);
		advance_to(&s, s.now + 1); CHECK(!s.bit && s.clears == 1);
	}
	// 14. review finding 2: an operation begins while the clear is being written (voted is still true, so begin does not kick). After the clear is
	// applied the vote must be set again without waiting for another operation.
	{
		struct sim s = fresh(true, 100, 0);
		begin(&s); advance_to(&s, 0); end(&s);
		CHECK(s.bit && s.armed);
		s.begin_in_write = 1;
		advance_to(&s, 100);                               // the idle hold expires: reconcile decides CLEAR, begin lands inside the write
		CHECK(s.v.users == 1);
		advance_to(&s, 101);                               // the owed set runs
		CHECK(s.bit && s.v.voted && s.sets == 2 && s.clears == 1);
		// the header alone, as the reviewer reproduced it: decide (CLEAR), begin, applied, next
		struct agx_vote v = {.enabled = true, .voted = true, .idle_ms = 100, .idle_at = 0};
		CHECK(agx_vote_decide(&v, 100) == AGX_VOTE_CLEAR);
		CHECK(!agx_vote_begin(&v));
		agx_vote_applied(&v, AGX_VOTE_CLEAR, 0, 100);
		CHECK(v.users == 1 && !v.voted);
		CHECK(agx_vote_next(&v, 100) == 1);                // not 0: a set is owed
		CHECK(agx_vote_decide(&v, 101) == AGX_VOTE_SET);
	}
	// 15. review finding 3: after release (GPU unbind or report removal) no begin may vote again, until the GPU probes again
	{
		struct sim s = fresh(true, 100, 0);
		begin(&s); advance_to(&s, 0);
		CHECK(s.bit && s.v.users == 1);
		if (agx_vote_release(&s.v)) { s.clears++; s.bit = false; }
		CHECK(!s.bit && s.v.stopped && s.clears == 1);
		end(&s); begin(&s); advance_to(&s, 50);            // an ioctl already past the unplug check starts an operation
		CHECK(!s.bit && s.sets == 1 && !s.armed);
		CHECK(agx_vote_decide(&s.v, 50) == AGX_VOTE_NONE);
		CHECK(agx_vote_next(&s.v, 50) == 0);
		// a work item that was queued before the release finds nothing to set
		arm(&s, 0); advance_to(&s, 51);
		CHECK(!s.bit && s.sets == 1);
		// the GPU probes again: the vote is allowed, and the operation still in flight gets its set
		CHECK(agx_vote_resume(&s.v));
		arm(&s, 0); advance_to(&s, 52);
		CHECK(s.bit && s.sets == 2 && !s.v.stopped);
	}
	// 16. review finding 4: system sleep. The vote is withdrawn before suspend and re-decided on resume: with work still in flight it is set
	// again (the PMP may have dropped the word), with an idle GPU nothing is written.
	{
		struct sim s = fresh(true, 250, 0);
		begin(&s); advance_to(&s, 0);
		CHECK(s.bit);
		if (agx_vote_release(&s.v)) { s.clears++; s.bit = false; }     // suspend prepare
		CHECK(!s.bit && s.v.stopped && s.clears == 1);
		if (agx_vote_resume(&s.v)) arm(&s, 0);                         // resume
		advance_to(&s, 10);
		CHECK(s.bit && s.sets == 2 && s.v.voted);
		end(&s);
		if (agx_vote_release(&s.v)) { s.clears++; s.bit = false; }     // a second sleep while idle inside the hold
		CHECK(!s.bit && s.clears == 2);
		CHECK(!agx_vote_resume(&s.v));                                 // idle: nothing is owed
		advance_to(&s, 1000);
		CHECK(!s.bit && s.sets == 2 && s.clears == 2);
		begin(&s); advance_to(&s, 1001);                               // the next operation after resume votes as usual
		CHECK(s.bit && s.sets == 3);
	}
	// 17. disabled and stopped never set, whatever begins
	{
		struct sim s = fresh(false, 100, 0);
		s.v.stopped = true;
		begin(&s); advance_to(&s, 1);
		CHECK(!s.bit && s.sets == 0 && !s.armed);
		CHECK(!agx_vote_resume(&s.v));                                 // disabled: resume owes nothing
	}
	// 18. mutants that survived the first review: begin() must itself refuse while stopped (not only decide()), a failed set must schedule the retry
	// deadline and not the 1 ms owed-set kick (a 1 kHz spin for the whole 1000 ms window otherwise), and agx_vote_refuse() must stop votes.
	{
		struct agx_vote v = {.enabled = true, .idle_ms = 100};
		agx_vote_refuse(&v);
		CHECK(v.stopped && !v.voted);
		CHECK(!agx_vote_begin(&v));                            // a user, but no kick
		CHECK(agx_vote_decide(&v, 0) == AGX_VOTE_NONE && agx_vote_next(&v, 0) == 0);
		CHECK(agx_vote_resume(&v));                            // work in flight: the reconcile step must run
		struct agx_vote w = {.enabled = true, .idle_ms = 100};
		CHECK(agx_vote_begin(&w));
		agx_vote_applied(&w, AGX_VOTE_SET, -110, 500);          // the set was not acknowledged
		CHECK(w.failures == 1 && w.retry_at == 500 + AGX_VOTE_RETRY_MS);
		CHECK(agx_vote_next(&w, 500) == AGX_VOTE_RETRY_MS);      // the retry deadline, not 1 ms
		CHECK(agx_vote_next(&w, 500 + AGX_VOTE_RETRY_MS - 1) == 1);
	}
	printf("AGX-VOTE-SELFCHECK-OK\n");
	return 0;
}
