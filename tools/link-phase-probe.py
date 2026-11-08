#!/usr/bin/env python3
import argparse
import bisect
import json
import math
import os
import select
import statistics
import sys
import threading
import time


def _us(value):
    return value.total_seconds() * 1e6 if hasattr(value, "total_seconds") else float(value)


def wrap(x):
    return x - math.floor(x + 0.5)


def circular_centre(fracs):
    sin_mean = sum(math.sin(2 * math.pi * f) for f in fracs) / len(fracs)
    cos_mean = sum(math.cos(2 * math.pi * f) for f in fracs) / len(fracs)
    return math.atan2(sin_mean, cos_mean) / (2 * math.pi)


def reader(fd, f8, stop):
    while not stop.is_set():
        ready, _, _ = select.select([fd], [], [], 0.05)
        if not ready:
            continue
        data = os.read(fd, 1)
        now = time.monotonic_ns()
        if data and data[0] == 0xF8:
            f8.append(now)


def probe(args):
    sys.setswitchinterval(1e-4)
    sys.path.insert(0, args.pylink)
    import asyncio
    import aalink
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    link = aalink.Link(args.bpm, loop)
    link.enabled = True
    fd = None if args.no_midi else os.open(args.dev, os.O_RDONLY)
    samples = []
    f8 = []
    stop = threading.Event()
    thread = threading.Thread(target=reader, args=(fd, f8, stop), daemon=True)
    if not args.no_midi:
        thread.start()
    start = time.monotonic_ns()
    end = start + int(args.seconds * 1e9)
    set_done = args.set_tempo is None
    try:
        while time.monotonic_ns() < end:
            now = time.monotonic_ns()
            if not set_done and now - start >= args.set_tempo_at * 1e9:
                link.tempo = args.set_tempo
                set_done = True
            lt = _us(link.time)
            beat = link.beat
            tempo = link.tempo
            peers = link.num_peers
            samples.append([time.monotonic_ns(), lt, beat, tempo, peers])
            time.sleep(args.sample_us / 1e6)
    finally:
        stop.set()
        if not args.no_midi:
            thread.join(timeout=2)
        if fd is not None:
            os.close(fd)
    return {'samples': samples, 'f8': list(f8), 'seconds': args.seconds}


def analyse(data):
    samples = data['samples']
    f8 = data['f8']
    mono = [s[0] for s in samples]
    offsets = sorted(s[1] - s[0] / 1000.0 for s in samples)
    offset = offsets[len(offsets) // 2]
    peers_seen = sorted(set(s[4] for s in samples))
    fracs = []
    tempos = []
    for m in f8:
        i = bisect.bisect_right(mono, m) - 1
        if i < 0:
            continue
        s = samples[i]
        lf = m / 1000.0 + offset
        tempo = s[3]
        beat_f = s[2] + (lf - s[1]) * tempo / 60e6
        fracs.append((beat_f * 24) % 1.0)
        tempos.append(tempo)
    centre = circular_centre(fracs) if fracs else 0.0
    late = [wrap(f - centre) * 60e6 / (t * 24) for f, t in zip(fracs, tempos)]
    absolute = sorted(abs(x) for x in late)
    return {
        'f8_count': len(f8),
        'analysed': len(late),
        'peers_seen': peers_seen,
        'offset_us': round(offset, 1),
        'mean_late_us': round(statistics.fmean(late), 2) if late else None,
        'stdev_late_us': round(statistics.pstdev(late), 2) if late else None,
        'p99_abs_late_us': round(absolute[int(0.99 * (len(absolute) - 1))], 2) if absolute else None,
        'max_abs_late_us': round(absolute[-1], 2) if absolute else None,
        'tempo_min': min(tempos) if tempos else None,
        'tempo_max': max(tempos) if tempos else None,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--dev', default='/dev/snd/midiC0D0')
    parser.add_argument('--pylink', default='.gm/pylink')
    parser.add_argument('--seconds', type=float, default=20.0)
    parser.add_argument('--bpm', type=float, default=120.0)
    parser.add_argument('--sample-us', type=int, default=2000)
    parser.add_argument('--set-tempo', type=float, default=None)
    parser.add_argument('--set-tempo-at', type=float, default=10.0)
    parser.add_argument('--no-midi', action='store_true')
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    data = probe(args)
    result = analyse(data)
    with open(args.out, 'w', encoding='utf-8') as fh:
        json.dump({'result': result, 'samples': data['samples'], 'f8': data['f8']}, fh)
    print(json.dumps(result, indent=1))


if __name__ == '__main__':
    main()
