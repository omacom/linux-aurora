#!/usr/bin/env python3
# SPDX-License-Identifier: GPL-2.0-only OR MIT
"""Read device-wide firmware load without confusing missing data with zero load."""
import argparse
import json
from pathlib import Path
import time


def snapshot(path):
    try:
        text = path.read_text()
    except BlockingIOError:
        # A snapshot can lose its bounded retries to the firmware writer.
        # Preserve an unavailable sample instead of terminating a monitor.
        return {'device': str(path.parent.resolve()), 'source': 'firmware',
                'load_valid': False, 'busy_percent': None,
                'error': 'statistics snapshot busy'}
    fields = dict(line.split(None, 1) for line in text.splitlines()
                  if len(line.split(None, 1)) == 2)
    if fields.get('version') != '2' or fields.get('source') != 'firmware':
        raise ValueError(f'{path}: unsupported statistics format')
    valid = fields.get('load_valid') == '1'
    return {
        'device': str(path.parent.resolve()),
        'source': 'firmware',
        'load_valid': valid,
        'busy_percent': int(fields['busy_percent']) if valid else None,
        'firmware_percentages': [int(fields[f'util{i}']) for i in range(1, 5)] if valid else None,
        'gpu_powered': fields['gpu_powered'] == '1' if fields['gpu_power_known'] == '1' else None,
        'samples': int(fields['samples']),
        'sample_age_seconds': int(fields['sample_age_ns']) / 1e9 if int(fields['samples']) else None,
        'invalid_samples': int(fields['invalid_samples']),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--path', type=Path)
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--interval', type=float, default=1.0)
    args = parser.parse_args()
    if args.count < 1 or args.interval <= 0:
        parser.error('count and interval must be positive')
    paths = [args.path] if args.path else sorted(Path('/sys/class/drm').glob('renderD*/device/agx_stats'))
    if not paths:
        parser.error('no GPU statistics attribute found')
    for index in range(args.count):
        print(json.dumps([snapshot(path) for path in paths]), flush=True)
        if index + 1 != args.count:
            time.sleep(args.interval)


if __name__ == '__main__':
    main()
