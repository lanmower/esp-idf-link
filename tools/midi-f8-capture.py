#!/usr/bin/env python3
import argparse
import json
import os
import select
import statistics
import time

NAMES = {0xF8: 'F8', 0xFA: 'FA', 0xFB: 'FB', 0xFC: 'FC'}


def capture(path, seconds):
    fd = os.open(path, os.O_RDONLY)
    events = []
    f2 = None
    end_ns = time.perf_counter_ns() + int(seconds * 1e9)
    try:
        while time.perf_counter_ns() < end_ns:
            ready, _, _ = select.select([fd], [], [], 0.25)
            if not ready:
                continue
            data = os.read(fd, 1)
            now = time.perf_counter_ns()
            if not data:
                continue
            b = data[0]
            if b >= 0xF8:
                if b in NAMES:
                    events.append((now, NAMES[b], None))
                continue
            if b == 0xF2:
                f2 = []
                continue
            if f2 is not None:
                if b < 0x80:
                    f2.append(b)
                    if len(f2) == 2:
                        events.append((now, 'F2', f2[0] | (f2[1] << 7)))
                        f2 = None
                    continue
                f2 = None
    finally:
        os.close(fd)
    return events


def analyse(events, seconds, bpm):
    f8 = [t for t, name, _ in events if name == 'F8']
    ideal_ns = 60e9 / (bpm * 24)
    expected = bpm * 24 / 60 * seconds
    intervals = [b - a for a, b in zip(f8, f8[1:])]
    errors = [d - ideal_ns for d in intervals]
    abs_sorted = sorted(abs(e) for e in errors)
    p99 = abs_sorted[int(0.99 * (len(abs_sorted) - 1))] if abs_sorted else 0.0
    gaps = sum(1 for d in intervals if d > 1.5 * ideal_ns)
    report = {
        'seconds': seconds,
        'bpm': bpm,
        'f8_count': len(f8),
        'f8_expected': round(expected, 1),
        'interval_count': len(intervals),
        'ideal_us': round(ideal_ns / 1e3, 3),
        'mean_err_us': round(statistics.fmean(errors) / 1e3, 3) if errors else None,
        'stdev_err_us': round(statistics.pstdev(errors) / 1e3, 3) if errors else None,
        'max_abs_err_us': round(abs_sorted[-1] / 1e3, 3) if abs_sorted else None,
        'p99_abs_err_us': round(p99 / 1e3, 3),
        'gaps_over_1p5_ideal': gaps,
        'non_clock': [],
    }
    seen = 0
    t0 = f8[0] if f8 else 0
    for t, name, value in events:
        if name == 'F8':
            seen += 1
        else:
            report['non_clock'].append({'name': name, 'value': value, 'after_f8': seen, 't_s': round((t - t0) / 1e9, 3)})
    return report


def floor(samples, period_ns):
    stamps = []
    nxt = time.perf_counter_ns()
    for _ in range(samples):
        nxt += period_ns
        delay = (nxt - time.perf_counter_ns()) / 1e9
        if delay > 0:
            time.sleep(delay)
        stamps.append(time.perf_counter_ns())
    errs = [b - a - period_ns for a, b in zip(stamps, stamps[1:])]
    absolute = sorted(abs(e) for e in errs)
    return {
        'samples': samples,
        'period_us': round(period_ns / 1e3, 3),
        'mean_err_us': round(statistics.fmean(errs) / 1e3, 3),
        'stdev_err_us': round(statistics.pstdev(errs) / 1e3, 3),
        'max_abs_err_us': round(absolute[-1] / 1e3, 3),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dev', default='/dev/snd/midiC0D0')
    parser.add_argument('--seconds', type=float, default=10.0)
    parser.add_argument('--bpm', type=float, default=120.0)
    parser.add_argument('--out', required=True)
    parser.add_argument('--floor', type=int, default=0)
    parser.add_argument('--raw', default=None)
    args = parser.parse_args()
    result = {}
    if args.floor:
        result['floor'] = floor(args.floor, 60e9 / (args.bpm * 24))
    events = capture(args.dev, args.seconds)
    if args.raw:
        with open(args.raw, 'w', encoding='utf-8') as fh:
            json.dump([[t, name] for t, name, _ in events], fh)
    result['capture'] = analyse(events, args.seconds, args.bpm)
    with open(args.out, 'w', encoding='utf-8') as fh:
        json.dump(result, fh, indent=1)
    print(json.dumps(result, indent=1))


if __name__ == '__main__':
    main()
