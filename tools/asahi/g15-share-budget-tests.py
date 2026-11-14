#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Run the actual G15 compute share budget bodies on host services.

The harness extracts the policy functions is_contended(), share_inflight(), share_cap(),
ewma_update() and ticks_ns() and Engine::share_budget() from g15/engine.rs, and
share_budget_ns() and share_window_ns() from g15/params.rs, with the parameter defaults read
from asahi_core.rs. Module parameters and the engine are models. It runs them for: a cap of
at least one command while a compute slot is free and never more than the free slots; all
free slots without a budget; one command with no estimate yet; the budget over the mean,
clamped; one flight while contended; the window boundary; EWMA seeding and convergence; the
24 MHz tick conversion; parameter clamps; and budget 0 turning the policy off.

Source checks: publish_prefix_into() caps the member loop and the horizon at the budget's
cap (horizon stop 4) and hands it to horizon_ready(), with share_cap() wired to the pass's
budget and the queue's mean; publish() notes the contention at the hold of another VM's
job and holds behind a flight with computes only, through share_inflight() wired to the
budget and the newest flight's computes (render-only flights keep MAX_INFLIGHT);
stat_retire() feeds the EWMA through ticks_ns() over valid intervals only; and the
asahi-g15/stats lines keep their existing formats, in order, with the "share:" line
appended after them, rendered and parsed. With --mutations, the wiring checks are also run
against copies of engine.rs with each call site disconnected, which must fail them, and
mutants of the compiled bodies must die at their intended assertions.

Not shown: kernel, firmware or GPU execution, end-to-end scheduling, or timing.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import subprocess

ROOT = Path(__file__).resolve().parents[2]
G15 = "drivers/gpu/drm/asahi/g15"
ENGINE = f"{G15}/engine.rs"
POLL = f"{G15}/poll.rs"
PARAMS = f"{G15}/params.rs"
CORE = "drivers/gpu/drm/asahi/asahi_core.rs"


def extract(text, marker):
    start = text.index(marker)
    opening = text.index("{", start)
    depth, end = 1, opening + 1
    while depth:
        depth += (text[end] == "{") - (text[end] == "}")
        end += 1
    return text[start:end]


PRELUDE = r'''
#![allow(dead_code)]
mod module_parameters {
    use std::sync::atomic::{AtomicU32, Ordering};
    pub struct Param(AtomicU32);
    impl Param {
        // A copy that outlives the call, as the kernel's SetOnce value does.
        pub fn value(&self) -> &u32 { Box::leak(Box::new(self.0.load(Ordering::SeqCst))) }
        pub fn set(&self, v: u32) { self.0.store(v, Ordering::SeqCst) }
    }
    #[allow(non_upper_case_globals)]
    pub static g15_share_budget_us: Param = Param(AtomicU32::new(__BUDGET_DEFAULT__));
    #[allow(non_upper_case_globals)]
    pub static g15_share_window_ms: Param = Param(AtomicU32::new(__WINDOW_DEFAULT__));
}
mod params {
    use crate::module_parameters;
    __PARAMS__
}
struct Engine { contended_ns: u64 }
impl Engine {
__ENGINE__
}
__POLICY__
const MS: u64 = 1_000_000;
// The cap as specified, in wider arithmetic: min(free, max(1, budget / avg)); free without a
// budget; min(free, 1) without an estimate.
fn reference_cap(free: u32, budget: u64, avg: u64) -> u32 {
    if budget == 0 { return free; }
    let fit: u128 = if avg == 0 { 1 } else { (budget as u128 / avg as u128).max(1) };
    fit.min(free as u128) as u32
}
fn main() {
    // share_cap over slots, budgets and means.
    let budgets = [0u64, 1, 100_000, 4 * MS, 1000 * MS, u64::MAX];
    let means = [0u64, 1, 250_000, 500_000, 4 * MS, 7 * MS, 300 * MS, u64::MAX];
    for free in 0..=16u32 {
        for &budget in budgets.iter() {
            for &avg in means.iter() {
                let cap = share_cap(free, budget, avg);
                assert!(cap <= free, "cap must not exceed the free slots: {free} {budget} {avg} -> {cap}");
                assert!(free == 0 || cap >= 1, "cap >= 1 while a slot is free: {free} {budget} {avg} -> {cap}");
                if budget == 0 {
                    assert_eq!(cap, free, "no budget: all free slots");
                } else if avg == 0 {
                    assert_eq!(cap, free.min(1), "no estimate: one command");
                } else {
                    assert_eq!(cap, reference_cap(free, budget, avg), "budget over the mean, clamped to 1..free");
                }
            }
        }
    }
    // The cases of linux-aurora #387 at the default budget.
    assert_eq!(share_cap(16, 4 * MS, 7 * MS), 1, "12B model commands (~7 ms): one per prefix");
    assert_eq!(share_cap(16, 4 * MS, 4 * MS), 1, "1B model commands (~4 ms): one per prefix");
    assert_eq!(share_cap(16, 4 * MS, 500_000), 8, "0.5 ms commands: eight per prefix");
    assert_eq!(share_cap(16, 4 * MS, 250_000), 16, "short commands: every slot");
    assert_eq!(share_cap(3, 4 * MS, 250_000), 3, "budget over the mean, clamped to 1..free");

    // share_inflight.
    for max in 1..=2usize {
        assert_eq!(share_inflight(true, max), 1, "one flight while contended");
        assert_eq!(share_inflight(false, max), max, "all flights uncontended");
    }

    // is_contended.
    assert!(!is_contended(50, 0, 100), "never held: not contended");
    assert!(!is_contended(u64::MAX, 0, u64::MAX), "never held: not contended");
    assert!(is_contended(1000, 1000, 100), "contended at the hold");
    assert!(is_contended(1099, 1000, 100), "contended within the window");
    assert!(!is_contended(1100, 1000, 100), "the window ends at window_ns");
    assert!(!is_contended(5000, 1000, 100), "the window ends at window_ns");
    assert!(!is_contended(1000, 1000, 0), "a zero window is never contended");
    assert!(is_contended(u64::MAX, u64::MAX - 5, 10), "contended within the window");

    // ewma_update.
    assert_eq!(ewma_update(0, 7 * MS), 7 * MS, "the first sample seeds the mean");
    assert_eq!(ewma_update(5 * MS, 0), 5 * MS, "a zero sample leaves the mean");
    assert_eq!(ewma_update(0, 0), 0, "a zero sample leaves the mean");
    assert_eq!(ewma_update(7 * MS, 7 * MS), 7 * MS, "a steady sample keeps the mean");
    assert_eq!(ewma_update(u64::MAX, u64::MAX), u64::MAX, "the mean does not overflow");
    for (from, to) in [(MS, 7 * MS), (50 * MS, 500_000), (300 * MS, 7 * MS), (1, 24)] {
        let mut p = from;
        for step in 1..=400 {
            let next = ewma_update(p, to);
            assert!(next.abs_diff(to) <= p.abs_diff(to), "the mean moves towards the sample");
            p = next;
            if step == 8 {
                // Weight 1/8: (7/8)^8 = 0.34 of the distance left after eight samples.
                assert!(p.abs_diff(to) * 2 < from.abs_diff(to) || from.abs_diff(to) < 16,
                        "eight samples move the mean more than half way: {from} -> {to}: {p}");
            }
        }
        assert!(p.abs_diff(to) < 8, "the mean converges to a steady sample: {from} -> {to}: {p}");
    }
    // A long prefill command, then short ones: the cap stays one while the mean decays, then
    // opens again (450 us: 8 per 4 ms anywhere in the EWMA's fixed-point band).
    let mut p = ewma_update(0, 300 * MS);
    assert_eq!(share_cap(16, 4 * MS, p), 1, "the cap follows the mean");
    for _ in 0..400 {
        p = ewma_update(p, 450_000);
    }
    assert_eq!(share_cap(16, 4 * MS, p), 8, "the cap follows the mean back down");

    // ticks_ns.
    assert_eq!(ticks_ns(24), 1000, "24 MHz ticks in ns");
    assert_eq!(ticks_ns(3), 125, "24 MHz ticks in ns");
    assert_eq!(ticks_ns(168_000), 7 * MS, "24 MHz ticks in ns");
    assert_eq!(ticks_ns(24_000_000), 1000 * MS, "24 MHz ticks in ns");
    assert_eq!(ticks_ns(u64::MAX), u64::MAX / 3, "tick conversion saturates");

    // Parameters: the defaults of asahi_core.rs, then the clamps.
    use module_parameters::{g15_share_budget_us as BUDGET, g15_share_window_ms as WINDOW};
    assert_eq!(params::share_budget_ns(), 4 * MS, "default budget 4000 us");
    assert_eq!(params::share_window_ns(), 500 * MS, "default window 500 ms");
    BUDGET.set(0);
    assert_eq!(params::share_budget_ns(), 0, "budget 0 is off");
    BUDGET.set(1);
    assert_eq!(params::share_budget_ns(), 100_000, "budget clamped");
    BUDGET.set(u32::MAX);
    assert_eq!(params::share_budget_ns(), 1000 * MS, "budget clamped");
    WINDOW.set(0);
    assert_eq!(params::share_window_ns(), MS, "window clamped");
    WINDOW.set(u32::MAX);
    assert_eq!(params::share_window_ns(), 10_000 * MS, "window clamped");
    BUDGET.set(__BUDGET_DEFAULT__);
    WINDOW.set(__WINDOW_DEFAULT__);

    // Engine::share_budget: the budget only within the window after a hold.
    let t = 1000 * MS;
    let mut e = Engine { contended_ns: 0 };
    assert_eq!(e.share_budget(t), 0, "never held: no budget");
    e.contended_ns = t;
    assert_eq!(e.share_budget(t + 50 * MS), 4 * MS, "contended: the budget");
    // Keystroke redraws at ~10 Hz stay inside the default window.
    assert_eq!(e.share_budget(t + 100 * MS), 4 * MS, "contended: the budget");
    assert_eq!(e.share_budget(t + 500 * MS - 1), 4 * MS, "contended: the budget");
    assert_eq!(e.share_budget(t + 500 * MS), 0, "the budget ends with the window");
    // Budget 0: whatever the contention, every cap and the flight count are as without it.
    BUDGET.set(0);
    assert_eq!(e.share_budget(t + 1), 0, "budget 0 turns the policy off");
    for free in 0..=16u32 {
        for &avg in means.iter() {
            let share = e.share_budget(t + 1);
            assert_eq!(share_cap(free, share, avg), free, "budget 0 turns the policy off");
            assert_eq!(share_inflight(share != 0, 2), 2, "budget 0 turns the policy off");
        }
    }
    println!("PASS: production share budget bodies: cap 1..free, no-budget and no-estimate caps, one flight while contended, window boundary, EWMA seeding/convergence, tick conversion, parameter clamps, budget 0 off");
}
'''

# The asahi-g15/stats format literals before the share budget, in order. Scripts parse them:
# they stay exactly as they are, and new lines come after them.
EXISTING_STATS = [
    "compute", "render", "mixed",
    "{}: count {} prefixes {} busy_us {} max_us {} first_us {} last_us {}",
    "gaps {} gap_us {} yields {} overlaps {}",
    "jobs {} job_cmds {} job_max {} early {} recoveries {} hang_kills {} clustered {}",
    "ct_sync {} slot_sync {} starved_us {} prep_us {} grouped {} pass_sync {}",
    "type0: fs2c {} c2ta {}",
    "horizon:", " {}", " stop {}/{}/{}/{} behind {} hold {}/{}/{} ahead {}",
    "stamps: compute_left {} compute_margin {} render_left {} render_margin {}",
    "stage: ta_us {} fs_us {} cdm_us {} span_us {}",
    "prof: prefixes {} select_us {} encode_us {} claim_us {} write_us {} post_us {} polls {} "
    "drain_us {} retire_us {} poll_us {} flushes {} flush_us {}",
]
SHARE_LINE = "share: flights {} capped {} hz_budget {} hold_vm {} budget_us {} window_ms {}"
SHARE_ARGS = ["s.share_flights", "s.share_capped", "s.hz_stop[4]", "s.hold[3]",
              "params::share_budget_ns() / 1000", "params::share_window_ns() / 1_000_000"]


def need(cond, message):
    if not cond:
        raise SystemExit("source check failed: " + message)


def source_checks(engine, poll):
    need("pub(crate) hz_stop: [u64; 5]," in engine, "hz_stop has a fifth reason")
    need("pub(crate) hold: [u64; 4]," in engine, "hold has a fourth reason")
    prefix = extract(engine, "    fn publish_prefix_into(&mut self, job: &Arc<Job>, f: &mut Flight) -> Result {")
    need("computes == free_c" not in prefix, "the member loop stops at the cap, not the free slots")
    need(prefix.count("if computes == cap_c {") == 1, "the member loop stops at the cap")
    need(prefix.count("self.horizon_ready(job, computes, cap_c, max)") == 1, "horizon_ready gets the cap")
    slots = re.search(r"if computes \+ n2 > free_c \{\s*hz_stop = Some\(2\);", prefix)
    budget = re.search(r"if computes \+ n2 > cap_c \{\s*hz_stop = Some\(4\);", prefix)
    need(slots is not None and budget is not None and slots.start() < budget.start(),
         "the horizon checks the slots (stop 2) before the budget (stop 4)")
    need(prefix.count("hz_stop = Some(4);") == 1, "one budget stop")
    need(prefix.index("let cap_c = share_cap(") < prefix.index("while end < count {"), "the cap precedes the member loop")
    need(re.search(r"let cap_c = share_cap\(\s*free_c,\s*self\.engine\.share_ns,\s*"
                   r"job\.queue\.avg_compute_ns\.load\(Ordering::Relaxed\),?\s*\);", prefix) is not None,
         "share_cap takes the free slots, the pass's budget and the queue's mean")
    need(prefix.count("if self.engine.share_ns != 0 && computes != 0 {") == 1,
         "the share stats count prefixes with computes under the budget")
    need(prefix.count("capped = cap_c < free_c;") == 1 and prefix.count("capped = true;") == 1,
         "the member loop and the horizon mark a prefix the cap cut short")
    publish = extract(engine, "    pub(crate) fn publish(&mut self, done: &mut Completions) -> Result {")
    need(publish.index("self.engine.share_ns = self.engine.share_budget(now_ns());") < publish.index("while "),
         "the budget is set once per pass, before the loop")
    need(re.search(r"if other_vm \{\s*(//[^\n]*\s*)*self\.engine\.contended_ns = now_ns\(\);\s*self\.engine\.stats\.hold\[3\] \+= 1;",
                   publish) is not None, "the hold of another VM's job notes the contention")
    need(publish.count("let max_inflight = tuning::MAX_INFLIGHT.clamp(1, sizes::FLIGHTS as u32) as usize;") == 1,
         "the full flight count is the tuning constant")
    tail = publish[publish.index("if let Some(tail) = self.engine.flights.last() {"):]
    gate = re.search(r"let inflight = share_inflight\(\s*self\.engine\.share_ns != 0 && tail\.computes != 0,\s*"
                     r"max_inflight,?\s*\);", tail)
    need(gate is not None, "share_inflight takes the budget and the newest flight's computes")
    need(tail.count("self.engine.flights.len() >= inflight") == 2 and tail.count(">= max_inflight") == 0,
         "the hold and its count use the gated flight limit")
    need(gate.start() < tail.index("self.engine.flights.len() >= inflight"), "the gate precedes the hold")
    retire = extract(poll, "    fn stat_retire(&mut self, f: &Flight) {")
    need(re.search(r"if cdm_n != 0 \{[^}]*ticks_ns\(cdm / cdm_n\)[^}]*ewma_update\(avg\.load\(Ordering::Relaxed\), sample\)",
                   retire, re.S) is not None, "the EWMA is fed in ns over valid intervals only")


def stats_checks(poll):
    body = extract(poll, "    pub(crate) fn write_stats(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {")
    literals = re.findall(r'"((?:[^"\\]|\\.)*)"', body)
    need(literals[:len(EXISTING_STATS)] == EXISTING_STATS,
         "the existing stats lines keep their formats and order: " + repr(literals[:len(EXISTING_STATS)]))
    added = literals[len(EXISTING_STATS):]
    need(SHARE_LINE in added, "the share line comes after the existing ones: " + repr(added))
    call = body[body.rindex("writeln!(", 0, body.index(SHARE_LINE)):]
    call = call[:call.index("\n        )")]
    args = [a.strip() for a in call.split(SHARE_LINE + '",', 1)[1].split(",\n") if a.strip()]
    need(args == SHARE_ARGS, "the share line's values: " + repr(args))
    need(SHARE_LINE.count("{}") == len(SHARE_ARGS), "one value per share key")
    # Render the horizon and share lines with distinct numbers and parse them back.
    numbers = iter(range(1, 1000))
    share = re.sub(r"\{\}", lambda _: str(next(numbers)), SHARE_LINE)
    need(share.startswith("share: "), "the share line is keyed")
    pairs = re.findall(r"(\w+) (\d+)", share[len("share: "):])
    need([k for k, _ in pairs] == ["flights", "capped", "hz_budget", "hold_vm", "budget_us", "window_ms"],
         "share keys: " + repr(pairs))
    return {"share_line": share}


# Production call sites disconnected in a copy of engine.rs: (old, new, the source check that
# must fail). The policy bodies are compiled below; these are the wiring the host cannot run.
WIRING_MUTANTS = {
    "cap-budget-disconnected": ("            free_c,\n            self.engine.share_ns,\n",
                                "            free_c,\n            0,\n",
                                "share_cap takes the free slots, the pass's budget and the queue's mean"),
    "inflight-contention-disconnected": ("self.engine.share_ns != 0 && tail.computes != 0,", "false,",
                                         "share_inflight takes the budget and the newest flight's computes"),
    "inflight-gate-dropped": ("self.engine.share_ns != 0 && tail.computes != 0,", "self.engine.share_ns != 0,",
                              "share_inflight takes the budget and the newest flight's computes"),
    "hold-ignores-gate": ("stats.hold[0] += (self.engine.flights.len() >= inflight) as u64;",
                          "stats.hold[0] += (self.engine.flights.len() >= max_inflight) as u64;",
                          "the hold and its count use the gated flight limit"),
    "share-stats-count-renders": ("if self.engine.share_ns != 0 && computes != 0 {", "if self.engine.share_ns != 0 {",
                                  "the share stats count prefixes with computes under the budget"),
    "capped-never-marked": ("                    capped = true;\n", "",
                            "the member loop and the horizon mark a prefix the cap cut short"),
}

# Mutants of the compiled policy bodies: (old, new, the assertion they must die at).
RUNTIME_MUTANTS = {
    "cap-may-be-zero": ("fit.max(1).min(free as u64) as u32", "fit.min(free as u64) as u32", "cap >= 1 while a slot is free"),
    "cap-ignores-slots": ("fit.max(1).min(free as u64) as u32", "fit.max(1) as u32", "cap must not exceed the free slots"),
    "no-estimate-admits-all": ("let fit = if avg_ns == 0 { 1 }", "let fit = if avg_ns == 0 { free as u64 }", "no estimate: one command"),
    "budget-zero-caps": ("    if budget_ns == 0 {\n        return free;\n    }\n", "", "no budget: all free slots"),
    "cap-rounds-up": ("budget_ns / avg_ns };", "budget_ns.div_ceil(avg_ns) };", "budget over the mean, clamped to 1..free"),
    "inflight-ignores-contention": ("        max.min(1)\n", "        max\n", "one flight while contended"),
    "never-held-contended": ("since != 0 && now.saturating_sub(since) < window_ns", "now.saturating_sub(since) < window_ns", "never held: not contended"),
    "window-inclusive": ("now.saturating_sub(since) < window_ns", "now.saturating_sub(since) <= window_ns", "the window ends at window_ns"),
    "ewma-unseeded": ("    if prev == 0 {\n        return sample;\n    }\n", "", "the first sample seeds the mean"),
    "ewma-takes-zero": ("    if sample == 0 {\n        return prev;\n    }\n", "", "a zero sample leaves the mean"),
    "ewma-ignores-sample": ("(prev - prev / 8).saturating_add(sample / 8)", "prev", "eight samples move the mean more than half way"),
    "ticks-as-us": ("ticks.saturating_mul(125) / 3", "ticks / 24", "24 MHz ticks in ns"),
    "budget-unclamped": ("us => us.clamp(100, 1_000_000) * 1000", "us => us * 1000", "budget clamped"),
    "window-unclamped": ("clamp(1, 10_000) as u64 * 1_000_000", "max(1) as u64 * 1_000_000", "window clamped"),
    "budget-without-window": ("if budget != 0 && is_contended(now, self.contended_ns, params::share_window_ns()) {",
                              "if budget != 0 {", "never held: no budget"),
}


def wiring_mutants(engine, poll):
    """Runs the source checks against each wiring mutant; returns their failure messages."""
    failed = {}
    for name, (old, new, message) in WIRING_MUTANTS.items():
        if engine.count(old) != 1:
            raise RuntimeError(f"{name}: unique mutation anchor count {engine.count(old)}")
        try:
            source_checks(engine.replace(old, new), poll)
        except SystemExit as e:
            failed[name] = str(e)
            if message not in str(e):
                raise RuntimeError(f"{name}: failed the wrong check: {e}")
        else:
            raise RuntimeError(f"{name}: survived the source checks")
    return failed


def runtime_mutants(code):
    """The compiled variants of `code` and the assertion message each must die with."""
    variants, expected = {}, {}
    for name, (old, new, message) in RUNTIME_MUTANTS.items():
        if code.count(old) != 1:
            raise RuntimeError(f"{name}: unique mutation anchor count {code.count(old)}")
        variants[name] = code.replace(old, new)
        expected[name] = message
    return variants, expected


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mutations", action="store_true")
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=True)
    src = {name: (ROOT / name).read_text() for name in [ENGINE, POLL, PARAMS, CORE]}
    engine, poll = src[ENGINE], src[POLL]
    source_checks(engine, poll)
    rendered = stats_checks(poll)
    wiring = wiring_mutants(engine, poll) if args.mutations else {}
    defaults = {}
    for param in ("g15_share_budget_us", "g15_share_window_ms"):
        m = re.search(rf"{param}: u32 \{{\s*default: (\d+),", src[CORE])
        need(m is not None, param + " default not found")
        defaults[param] = m.group(1)
    policy = "\n".join(extract(engine, marker) for marker in [
        "pub(crate) fn is_contended(", "pub(crate) fn share_inflight(", "pub(crate) fn share_cap(",
        "pub(crate) fn ewma_update(", "pub(crate) fn ticks_ns("])
    params = "\n".join(extract(src[PARAMS], marker) for marker in [
        "pub(crate) fn share_budget_ns() -> u64 {", "pub(crate) fn share_window_ns() -> u64 {"])
    code = (PRELUDE.replace("__BUDGET_DEFAULT__", defaults["g15_share_budget_us"])
            .replace("__WINDOW_DEFAULT__", defaults["g15_share_window_ms"])
            .replace("__PARAMS__", params)
            .replace("__ENGINE__", extract(engine, "    fn share_budget(&self, now: u64) -> u64 {"))
            .replace("__POLICY__", policy))
    variants = {"control": code}
    expected = {}
    if args.mutations:
        mutants, expected = runtime_mutants(code)
        variants.update(mutants)
    variants["control-after"] = code
    receipt = {"scope": __doc__, "stats": rendered, "defaults": defaults, "wiring": wiring,
               "sources": {name: hashlib.sha256(text.encode()).hexdigest() for name, text in src.items()},
               "variants": {}}
    for name, source in variants.items():
        path = args.output / (name + ".rs")
        path.write_text(source)
        binary = args.output / name
        built = subprocess.run([os.environ.get("RUSTC", "rustc"), "--edition=2021", str(path), "-o", str(binary)],
                               capture_output=True, text=True)
        (args.output / (name + ".build.log")).write_text(built.stdout + built.stderr)
        if built.returncode:
            raise RuntimeError(f"{name}: NOBUILD, not a runtime kill: {built.stderr}")
        ran = subprocess.run([str(binary.resolve())], capture_output=True, text=True, timeout=30)
        receipt["variants"][name] = {"exit": ran.returncode, "stdout": ran.stdout, "stderr": ran.stderr,
                                     "intended_failure": name in expected and expected[name] in ran.stderr}
        if name.startswith("control") and ran.returncode:
            raise RuntimeError(ran.stdout + ran.stderr)
        if name in expected and (ran.returncode == 0 or expected[name] not in ran.stderr):
            raise RuntimeError(f"{name}: did not die at intended runtime assertion: {ran.stderr}")
    (args.output / "result.json").write_text(json.dumps(receipt, indent=2) + "\n")
    print(json.dumps(receipt, indent=2))


if __name__ == "__main__":
    main()
