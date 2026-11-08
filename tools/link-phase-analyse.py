#!/usr/bin/env python3
import bisect
import json
import statistics
import sys


def analyse(samples_path, raw_path):
    samples = json.load(open(samples_path, encoding='utf-8'))['samples']
    raw = json.load(open(raw_path, encoding='utf-8'))
    f8 = [t for t, name in raw if name == 'F8']
    mono = [s[0] for s in samples]
    offsets = sorted(s[1] - s[0] / 1000.0 for s in samples)
    offset = offsets[len(offsets) // 2]
    tempos = {}
    for m in f8:
        i = bisect.bisect_right(mono, m) - 1
        if i < 0 or i + 1 >= len(samples):
            continue
        s = samples[i]
        lf = m / 1000.0 + offset
        tempo = s[3]
        beat_f = s[2] + (lf - s[1]) * tempo / 60e6
        k = round(beat_f * 24)
        t_grid = s[1] + (k / 24.0 - s[2]) * 60e6 / tempo
        tempos.setdefault(round(tempo), {'late': []})['late'].append(lf - t_grid)
    for key in sorted(tempos):
        late = tempos[key]['late']
        absolute = sorted(abs(x) for x in late)
        print('tempo %d: n=%d mean_us=%.1f stdev_us=%.1f p99_us=%.1f max_us=%.1f over_1ms=%d' % (
            key, len(late), statistics.fmean(late), statistics.pstdev(late),
            absolute[int(0.99 * (len(absolute) - 1))], absolute[-1],
            sum(1 for x in absolute if x > 1000)))


if __name__ == '__main__':
    analyse(sys.argv[1], sys.argv[2])
