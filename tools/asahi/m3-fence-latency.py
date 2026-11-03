#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Join existing M3 timing logs. Input must be one boot, captured without gaps."""
import collections
import json
import math
import re
import sys

milestones = {}
packets = collections.defaultdict(list)
fences = []
invalid = collections.Counter()
for line in open(sys.argv[1]):
    fields = {k: int(v, 0) for k, v in re.findall(r'(\w+)=(-?0x[0-9a-fA-F]+|-?\d+)', line)}
    if 'M3_BATCH_MILESTONE ' in line:
        key = (fields['vm'], fields['publication_tick'])
        if key in milestones:
            invalid['duplicate_milestone'] += 1
        milestones[key] = fields
    elif 'M3_BATCH_PACKET ' in line:
        packets[(fields['vm'], fields['packet'])].append((fields['publication_tick'], fields['command']))
    elif 'M3_PACKET_FENCE ' in line:
        fences.append(fields)
values = []
previous_signal = {}
for f in sorted(fences, key=lambda f: f['signal_before_tick']):
    identity = (f['vm'], f['packet'])
    previous = previous_signal.get(identity, 0)
    previous_signal[identity] = f['signal_after_tick']
    if f['status'] != 0:
        invalid['error_fence'] += 1
        continue
    linked = packets.get((f['vm'], f['packet']), [])
    # Pointer reuse is disambiguated by the completed fence's physical counter.
    candidates = [(tick, command) for tick, command in linked if previous < tick <= f['signal_before_tick'] and command == f['commands'] - 1]
    if not candidates:
        invalid['missing_final_command'] += 1
        continue
    final_tick, _ = max(candidates)
    batch = milestones.get((f['vm'], final_tick))
    if not batch or batch['mixed'] or batch['render_end_tick'] <= 0:
        invalid['missing_or_non_render_end'] += 1
        continue
    end = batch['render_end_tick']
    before, after = f['signal_before_tick'], f['signal_after_tick']
    if not (final_tick <= end <= before <= after):
        invalid['invalid_counter_order'] += 1
        continue
    # GPU execution end is earlier than full coherent retirement; this includes
    # trailing witnesses and dispatch/retirement, rather than claiming an IRQ is done.
    values.append(((before - end) / 24, (after - end) / 24))

def percentiles(data):
    ordered = sorted(data)
    return {str(p): ordered[max(0, math.ceil(len(ordered) * p / 100) - 1)] for p in (50, 95, 99, 100)} if ordered else {}

print(json.dumps({'render_fences': len(values), 'invalid': invalid,
                  'gpu_end_to_signal_before_us': percentiles([a for a, b in values]),
                  'gpu_end_to_signal_after_us': percentiles([b for a, b in values]),
                  'note': '24 MHz physical counter; interval brackets signal(); includes trailing witnesses. Not presentation latency.'}, indent=2))
if not values:
    sys.exit('No valid render fences: do not report latency from this capture')
