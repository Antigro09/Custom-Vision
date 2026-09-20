#!/usr/bin/env python3
"""Summarize posted host-side vision JSONL without hiding failed observations."""
import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path

import numpy as np


def _group():
    return dict(records=0, connected=0, errors=Counter(), samples=defaultdict(list),
                tags=0, valid_poses=0, invalid_poses=Counter(), invalid_fields=Counter(),
                invalid_measurements=Counter(), last_dropped=None, max_dropped=None,
                initial_dropped=None, observed_dropped_increments=0, drop_counter_resets=0)


def _finite_nonnegative(value):
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value) and value >= 0)


def summarize(path, warmup=30):
    if isinstance(warmup, bool) or not isinstance(warmup, int) or warmup < 0:
        raise ValueError('warmup must be a nonnegative integer')
    groups = defaultdict(_group)
    malformed = 0
    with Path(path).open() as stream:
        for line in stream:
            try:
                p = json.loads(line)
                if (not isinstance(p, dict) or not isinstance(p.get('pipeline'), str)
                        or not p['pipeline'] or not isinstance(p.get('boot_id', 'unknown'), str)
                        or not isinstance(p.get('connected'), bool)):
                    raise ValueError()
            except (ValueError, TypeError):
                malformed += 1
                continue
            g = groups[(p['pipeline'], p.get('boot_id', 'unknown'))]
            g['records'] += 1
            if not p['connected']:
                g['errors'][str(p.get('error') or 'disconnected')] += 1
                continue
            g['connected'] += 1
            # Camera readers can restart within one boot. A maximum is not the
            # total dropped frames and a missing counter is not an observed zero.
            if 'dropped_frames' in p:
                dropped = p['dropped_frames']
                if isinstance(dropped, int) and not isinstance(dropped, bool) and dropped >= 0:
                    previous = g['last_dropped']
                    if previous is None:
                        g['initial_dropped'] = dropped
                    elif dropped >= previous:
                        g['observed_dropped_increments'] += dropped - previous
                    else:
                        g['drop_counter_resets'] += 1
                        g['observed_dropped_increments'] += dropped
                    g['last_dropped'] = dropped
                    g['max_dropped'] = dropped if g['max_dropped'] is None else max(dropped, g['max_dropped'])
                else:
                    g['invalid_fields']['dropped_frames'] += 1
            # Quality/error accounting includes warmup. Only timing/FPS samples
            # exclude the first N connected records, independently per boot/camera.
            detections = p.get('detections', [])
            if not isinstance(detections, list):
                g['invalid_fields']['detections'] += 1
                detections = []
            for d in detections:
                if not isinstance(d, dict):
                    g['invalid_fields']['detection'] += 1
                    continue
                if 'id' not in d:  # Object detections have class_id, not tag ID.
                    continue
                if isinstance(d['id'], bool) or not isinstance(d['id'], int) or d['id'] < 0:
                    g['invalid_fields']['tag_id'] += 1
                    continue
                g['tags'] += 1
                if d.get('pose_valid') is True:
                    g['valid_poses'] += 1
                else:
                    g['invalid_poses'][str(d.get('pose_invalid_reason') or 'unknown')] += 1
            if g['connected'] <= warmup:
                continue
            values = {k: p.get(k) for k in ('fps', 'detector_ms', 'processing_ms',
                                           'localization_ms', 'queue_ms', 'latency_ms') if k in p}
            native = p.get('native_timings')
            if native is not None and not isinstance(native, dict):
                g['invalid_fields']['native_timings'] += 1
                native = {}
            for k in ('detect_ms', 'pose_ms', 'pose_kernel_ms', 'preprocess_ms'):
                if native is not None and k in native:
                    values['native_' + k] = native[k]
            for k, v in values.items():
                if _finite_nonnegative(v):
                    g['samples'][k].append(v)
                else:
                    g['invalid_measurements'][k] += 1
    result = []
    for (name, boot), g in groups.items():
        samples = g.pop('samples')
        for field in ('errors', 'invalid_poses', 'invalid_fields', 'invalid_measurements'):
            g[field] = dict(g[field])
        g.update(pipeline=name, boot_id=boot,
                 warmup_connected_frames_excluded=min(warmup, g['connected']),
                 measured_connected_frames=max(0, g['connected'] - warmup),
                 measurements={k: dict(samples=len(v), p50=float(np.percentile(v, 50)),
                                      p95=float(np.percentile(v, 95)), p99=float(np.percentile(v, 99)),
                                      maximum=float(max(v))) for k, v in samples.items()})
        result.append(g)
    return dict(
        note='Host-read-to-publication timing, not exposure-to-robot latency. Logged FPS is a smoothed runtime measurement, not reconstructed camera throughput. Logging adds overhead.',
        quality_counts_scope='All connected records, including warmup; errors include shutdown and all disconnected records.',
        dropped_frames_scope='Observed counter increments after the first logged counter, including values after detected resets. Initial counter values are separate; losses before logging or between counter resets cannot be reconstructed.',
        malformed_lines=malformed, pipelines=result)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('log', type=Path)
    parser.add_argument('--warmup', type=int, default=30)
    args = parser.parse_args(argv)
    if args.warmup < 0:
        parser.error('warmup must be nonnegative')
    print(json.dumps(summarize(args.log, args.warmup), indent=2, allow_nan=False))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
