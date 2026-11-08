#!/usr/bin/env python3
"""Measure the offset between MIDI F8 clock pulses and audible click onsets.

Records a 16-bit stereo WAV with arecord (or analyses --wav), finds click onsets on
one channel (peak-hold envelope, rising edge, 200 ms refractory), pairs each onset
with the nearest F8 timestamp and prints offset = click - F8 in ms (positive: the
click lags the F8): N, mean, stdev and max-abs over the paired clicks.

F8 timestamps must share the WAV's time origin (seconds from the first sample).
For a real 24 ppqn stream pass only beat pulses (every 24th F8): the tool does not
know pulse indices, so a dense list pairs clicks with neighbouring pulses.

Examples:
  python3 -I tools/midi-click-jitter.py --dev plughw:0,0 --chan 2 --dur 30 --f8 f8.txt
  python3 -I tools/midi-click-jitter.py --wav cap.wav --chan 2 --f8 -
  python3 -I tools/midi-click-jitter.py --selftest
"""

import argparse
import array
import bisect
import math
import os
import random
import statistics
import subprocess
import sys
import time
import wave

RATE = 48000
REFRACTORY_S = 0.200
PEAK_HOLD_S = 0.002
MIN_CLICKS = 8
DEFAULT_OUT = "/tmp/claude-1000/-config-workspace-esp-idf-link/ce34f795-a6c6-4874-97ff-1912e486c879/scratchpad"


def read_channel(path, chan):
    with wave.open(path, "rb") as w:
        if w.getsampwidth() != 2:
            sys.exit("midi-click-jitter: need 16-bit PCM WAV")
        nch, rate = w.getnchannels(), w.getframerate()
        raw = w.readframes(w.getnframes())
    if not 0 <= chan < nch:
        sys.exit("midi-click-jitter: channel %d out of range for %d-channel WAV" % (chan + 1, nch))
    samples = array.array("h")
    samples.frombytes(raw)
    if sys.byteorder == "big":
        samples.byteswap()
    return samples[chan::nch], rate


def find_onsets(samples, rate):
    mags = [abs(v) for v in samples]
    if not mags or max(mags) == 0:
        return []
    peak = max(mags)
    noise = statistics.median(mags[::7])
    thr = max(0.1 * peak, 8.0 * noise)
    decay = math.exp(-1.0 / (PEAK_HOLD_S * rate))
    onsets, env, above, last = [], 0.0, False, -1e9
    for i, m in enumerate(mags):
        env = max(m, env * decay)
        if env >= thr:
            t = i / rate
            if not above and t - last >= REFRACTORY_S:
                onsets.append(t)
                last = t
            above = True
        else:
            above = False
    return onsets


def read_f8(path):
    fh = sys.stdin if path == "-" else open(path, "r")
    try:
        vals = [float(s) for s in (line.strip() for line in fh) if s and not s.startswith("#")]
    finally:
        if fh is not sys.stdin:
            fh.close()
    return sorted(vals)


def pair_offsets(onsets, f8s, max_match_s):
    offsets, unmatched = [], 0
    for c in onsets:
        j = bisect.bisect_left(f8s, c)
        cands = [f8s[k] for k in (j - 1, j) if 0 <= k < len(f8s)]
        best = min(cands, key=lambda f: abs(c - f)) if cands else None
        if best is not None and abs(c - best) <= max_match_s:
            offsets.append((c - best) * 1000.0)
        else:
            unmatched += 1
    return offsets, unmatched


def stats(offsets):
    n = len(offsets)
    if n == 0:
        return 0, float("nan"), float("nan"), float("nan")
    sd = statistics.stdev(offsets) if n > 1 else 0.0
    return n, statistics.fmean(offsets), sd, max(abs(o) for o in offsets)


def record(dev, dur, out_path):
    cmd = ["arecord", "-q", "-D", dev, "-t", "wav", "-f", "S16_LE", "-r", str(RATE),
           "-c", "2", "-d", str(max(1, int(math.ceil(dur)))), out_path]
    try:
        subprocess.run(cmd, check=True)
    except (OSError, subprocess.CalledProcessError) as e:
        sys.exit("midi-click-jitter: arecord failed: %s" % e)


def clamp16(v):
    return max(-32768, min(32767, int(round(v))))


def selftest(out_dir):
    rng = random.Random(1)
    n = int(10.5 * RATE)
    true_clicks = [0.5 + 0.5 * k for k in range(20)]
    click = [0.0] * n
    for t in true_clicks:
        n0 = int(round(t * RATE))
        for k in range(int(0.03 * RATE)):
            if n0 + k < n:
                click[n0 + k] += 16384.0 * math.sin(2 * math.pi * 1000.0 * k / RATE) * math.exp(-k / (0.005 * RATE))
    frames = array.array("h", [0]) * (2 * n)
    for i in range(n):
        frames[2 * i] = clamp16(rng.gauss(0, 2000))
        frames[2 * i + 1] = clamp16(rng.gauss(0, 65) + click[i])
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, "selftest-click-jitter.wav")
    if sys.byteorder == "big":
        frames.byteswap()
    with wave.open(path, "wb") as w:
        w.setnchannels(2)
        w.setsampwidth(2)
        w.setframerate(RATE)
        w.writeframes(frames.tobytes())
    jit = [rng.gauss(0, 0.0004) for _ in true_clicks]
    jit_mean = statistics.fmean(jit)
    f8 = [t - 0.0015 + (j - jit_mean) for t, j in zip(true_clicks, jit)]
    samples, rate = read_channel(path, 1)
    onsets = find_onsets(samples, rate)
    offsets, unmatched = pair_offsets(onsets, sorted(f8), 0.010)
    n_pairs, mean, sd, mx = stats(offsets)
    print("selftest wav=%s" % path)
    print("clicks_detected=%d paired=%d unmatched=%d" % (len(onsets), n_pairs, unmatched))
    print("N=%d mean_ms=%.4f stdev_ms=%.4f max_abs_ms=%.4f" % (n_pairs, mean, sd, mx))
    ok = abs(mean - 1.5) <= 0.1 and abs(sd - 0.4) <= 0.3 and n_pairs >= MIN_CLICKS
    print("SELFTEST mean_ms=%.4f (want 1.5 +/-0.1) stdev_ms=%.4f (want 0.4 +/-0.3): %s" % (mean, sd, "OK" if ok else "BAD"))
    return ok


def main():
    ap = argparse.ArgumentParser(description="MIDI F8 vs click onset offset (ms).")
    ap.add_argument("--selftest", action="store_true", help="synthesise a WAV and check recovery")
    ap.add_argument("--dev", default="plughw:0,0", help="ALSA capture device (arecord -D)")
    ap.add_argument("--chan", type=int, default=1, help="1-based channel on the capture device (1 or 2)")
    ap.add_argument("--dur", type=float, default=30.0, help="capture seconds")
    ap.add_argument("--wav", help="analyse this 16-bit WAV instead of recording")
    ap.add_argument("--f8", help="F8 timestamps in seconds, one per line, or - for stdin")
    ap.add_argument("--max-match-ms", type=float, default=10.0, help="reject pairs further apart than this")
    ap.add_argument("--out-dir", default=DEFAULT_OUT, help="where recorded WAVs go (keep out of the repo)")
    args = ap.parse_args()

    if args.selftest:
        ok = selftest(args.out_dir)
        print("JITTER-TOOL: %s" % ("PASS" if ok else "FAIL"))
        return 0 if ok else 1
    if args.f8 is None:
        ap.error("--f8 is required outside --selftest")

    wav = args.wav
    if wav is None:
        os.makedirs(args.out_dir, exist_ok=True)
        wav = os.path.join(args.out_dir, "click-jitter-%d.wav" % int(time.time()))
        record(args.dev, args.dur, wav)
    samples, rate = read_channel(wav, args.chan - 1)
    onsets = find_onsets(samples, rate)
    offsets, unmatched = pair_offsets(onsets, read_f8(args.f8), args.max_match_ms / 1000.0)
    n, mean, sd, mx = stats(offsets)
    print("wav=%s channel=%d" % (wav, args.chan))
    print("clicks_detected=%d paired=%d unmatched=%d" % (len(onsets), n, unmatched))
    print("N=%d mean_ms=%.4f stdev_ms=%.4f max_abs_ms=%.4f" % (n, mean, sd, mx))
    verdict = "PASS" if n >= MIN_CLICKS else "FAIL"
    print("JITTER-TOOL: %s (N=%d, need >=%d)" % (verdict, n, MIN_CLICKS))
    return 0 if verdict == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
