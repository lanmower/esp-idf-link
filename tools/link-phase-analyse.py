#!/usr/bin/env python3
import bisect
import json
import math
import statistics
import sys

SEGMENT_NS = 5e9
STEP_PULSES = 0.05
LOCK_STDEV_US = 1000.0


def wrap(x):
    return x - math.floor(x + 0.5)


def circular_centre(fracs):
    sin_mean = sum(math.sin(2 * math.pi * f) for f in fracs) / len(fracs)
    cos_mean = sum(math.cos(2 * math.pi * f) for f in fracs) / len(fracs)
    return math.atan2(sin_mean, cos_mean) / (2 * math.pi), math.hypot(sin_mean, cos_mean)


def pulse_rows(samples, raw_path):
    raw = json.load(open(raw_path, encoding='utf-8'))
    f8 = [t for t, name in raw if name == 'F8']
    mono = [s[0] for s in samples]
    offsets = sorted(s[1] - s[0] / 1000.0 for s in samples)
    offset = offsets[len(offsets) // 2]
    rows = []
    for m in f8:
        i = bisect.bisect_right(mono, m) - 1
        if i < 0 or i + 1 >= len(samples):
            continue
        s = samples[i]
        lf = m / 1000.0 + offset
        tempo = s[3]
        x = (s[2] + (lf - s[1]) * tempo / 60e6) * 24
        rows.append((m, x % 1.0, tempo))
    return f8, rows


def report_tempo(tempo_key, group):
    centre, r = circular_centre([f for _, f, _ in group])
    pulse_us = 60e6 / (tempo_key * 24)
    late = [wrap(f - centre) * pulse_us for _, f, _ in group]
    absolute = sorted(abs(x) for x in late)
    t0 = min(m for m, _, _ in group)
    segments = {}
    for m, f, _ in group:
        segments.setdefault(int((m - t0) / SEGMENT_NS), []).append(f)
    seg_centres = [round(circular_centre(v)[0], 3) for _, v in sorted(segments.items())]
    steps = sum(1 for a, b in zip(seg_centres, seg_centres[1:]) if abs(wrap(b - a)) > STEP_PULSES)
    stdev = statistics.pstdev(late)
    verdict = 'LOCKED' if stdev < LOCK_STDEV_US and steps == 0 else 'NOT LOCKED'
    print('tempo %d: n=%d centre_frac=%+.3f R=%.4f stdev_us=%.1f p99_abs_us=%.1f max_abs_us=%.1f '
          'segments=%d steps=%d verdict=%s' % (
              tempo_key, len(group), centre, r, stdev,
              absolute[int(0.99 * (len(absolute) - 1))], absolute[-1],
              len(segments), steps, verdict))
    print('  segment_centres_frac=%s' % seg_centres)


def analyse(samples_path, raw_path):
    samples = json.load(open(samples_path, encoding='utf-8'))['samples']
    f8, rows = pulse_rows(samples, raw_path)
    print('f8=%d analysed=%d' % (len(f8), len(rows)))
    groups = {}
    for row in rows:
        groups.setdefault(round(row[2]), []).append(row)
    for key in sorted(groups):
        report_tempo(key, groups[key])


if __name__ == '__main__':
    analyse(sys.argv[1], sys.argv[2])
