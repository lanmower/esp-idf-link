#!/usr/bin/env python3
"""Impose a Link beat-origin shift from this host, the way a first loop does.

Records host samples (mono_ns, beat, tempo, peers) every 2 ms and the time of the
forced origin, so a DIN capture on the same CLOCK_MONOTONIC can be aligned to it.
"""
import argparse
import asyncio
import json
import os
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--pylink', default='.gm/pylink')
    parser.add_argument('--bpm', type=float, default=120.0)
    parser.add_argument('--seconds', type=float, default=90.0)
    parser.add_argument('--shift-at', type=float, default=25.0)
    parser.add_argument('--shift-beats', type=float, default=5.5)
    parser.add_argument('--join-wait', type=float, default=20.0)
    parser.add_argument('--out', required=True)
    args = parser.parse_args()
    sys.path.insert(0, args.pylink)
    import aalink
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    link = aalink.Link(args.bpm, loop)
    link.enabled = True
    samples = []
    shift = None
    start = time.monotonic_ns()
    end = start + int(args.seconds * 1e9)
    joined_at = None
    while time.monotonic_ns() < end:
        now = time.monotonic_ns()
        elapsed = (now - start) / 1e9
        if joined_at is None and link.num_peers > 0:
            joined_at = elapsed
        if shift is None and elapsed >= args.shift_at and (joined_at is not None or elapsed >= args.join_wait):
            before = link.beat
            target = before + args.shift_beats
            link.force_beat(target)
            after = link.beat
            shift = {'mono_ns': time.monotonic_ns(), 'beat_before': before, 'target': target,
                     'beat_after': after, 'peers_at_shift': link.num_peers}
        samples.append([time.monotonic_ns(), link.beat, link.tempo, link.num_peers])
        time.sleep(0.002)
    link.enabled = False
    with open(args.out, 'w', encoding='utf-8') as fh:
        json.dump({'shift': shift, 'joined_at_s': joined_at, 'samples': samples}, fh)
    print(json.dumps({'shift': shift, 'joined_at_s': joined_at, 'n': len(samples)}))


if __name__ == '__main__':
    main()
