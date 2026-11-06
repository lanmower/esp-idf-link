import argparse
import datetime as dt
import hashlib
import json
import math
import os
import re
import statistics
import sys
from pathlib import Path

DEFAULT_INPUT = "/config/workspace/mc-420/.gm/witness-out/telemetry-1hz-20261105T195938Z.jsonl"
DEFAULT_REPORT = "/config/workspace/esp-idf-link/.gm/witness-out/phrase-telemetry-report-20261105.txt"
SELF_TEST_REPORT = "/config/workspace/esp-idf-link/.gm/witness-out/phrase-telemetry-self-test-20261105.txt"
SELF_TEST_DATA = "/config/workspace/esp-idf-link/.gm/witness-out/phrase-telemetry-self-test-data-20261105.jsonl"
ESP_STATUS_FILE = "/config/workspace/esp-idf-link/.gm/witness-out/esp-status-phrase-20261105.jsonl"
ESP_META = "/config/workspace/esp-idf-link/.gm/witness-out/esp-status-phrase-20261105.meta.json"
ESP_EXTENSION_META = "/config/workspace/esp-idf-link/.gm/witness-out/esp-status-phrase-20261105.extension.json"
FALLBACK_WITNESS = "/config/workspace/esp-idf-link/.gm/witness-out/phrase-telemetry-420-complete.json"
MAIN_H = Path(__file__).resolve().parent.parent / "main" / "main.h"

ACCEPTANCE_SOURCE = "AGENTS.md Live state 2026-10-23 acceptance terms"
ACCEPTANCE = {
    "retention_min_pct": 95.0,
    "bpm_spread_max": 0.05,
    "phase_err_abs_max": 0.03,
    "eff_speed_spread_max": 0.005,
    "gap_pct_max": 10.0,
}
COMPLETION_CRITERION = "line count stable >=90 s AND last sample index >=1790"
HOST_KEY_SHA256 = "SHA256:aABHFBNqjNwZCEToVnx7ca4KpwHYg7xhrWhWxd4fn3A"
SUBSCRIBER_NOTE = "The ESP status port is a subscriber hit: each poll raises ESP LCLK/TTMP broadcasts for a 15 s window and does not perturb the Link session."
CONTROL_TELEMETRY_DEFAULT = "/config/workspace/mc-420/.gm/witness-out/control-window-1hz-20261105T203434Z.jsonl"
CONTROL_MARKERS_DEFAULT = "/config/workspace/mc-420/.gm/witness-out/telemetry-control-20261105.markers.json"
CONTROL_ESP_FILE = ".gm/witness-out/esp-status-control-20261105.jsonl"
CONTROL_ESP_META = ".gm/witness-out/esp-status-control-20261105.meta.json"
CONTROL_ESP_PID = ".gm/witness-out/esp-status-control-20261105.pid"
CONTROL_REPORT = ".gm/witness-out/phrase-control-report-20261105.txt"
FINAL_WITNESS = ".gm/witness-out/phrase-telemetry-final-witness-20261105.json"
DEFAULT_LIMITATIONS = "Cross-tree DIN transport behaviour (mc-420 MidiClock SPP/Stop bytes and the enableStartStopSync documentation) is NOT visible in 1 Hz 4445 telemetry; it is excluded from the telemetry-based score and judged separately."
NO_PERTURBATIONS_SENT = "NO_PERTURBATIONS_SENT"


def load_constants():
    result = {"LINK_QUANTUM": 16.0, "PHRASE_BEATS": 64.0, "source": "fallback constants"}
    try:
        text = MAIN_H.read_text(encoding="utf-8", errors="replace")
        quantum = re.search(r"#define\s+LINK_QUANTUM\s+([0-9]*\.?[0-9]+)", text)
        phrase = re.search(r"#define\s+PHRASE_BEATS\s+([0-9]*\.?[0-9]+)", text)
        if quantum and phrase:
            result = {
                "LINK_QUANTUM": float(quantum.group(1)),
                "PHRASE_BEATS": float(phrase.group(1)),
                "source": str(MAIN_H),
            }
    except OSError:
        pass
    return result


def truthy(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)):
        return value != 0
    if isinstance(value, str):
        return value.strip().lower() in ("true", "1", "yes", "ok")
    return False


def parse_timestamp(value):
    if value is None:
        return None
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        return float(value)
    text = str(value).strip()
    if not text:
        return None
    if text.endswith("Z"):
        text = text[:-1] + "+00:00"
    fraction = re.match(r"^(.*\.)(\d+)(.*)$", text)
    if fraction and len(fraction.group(2)) > 6:
        text = fraction.group(1) + fraction.group(2)[:6] + fraction.group(3)
    try:
        parsed = dt.datetime.fromisoformat(text)
        if parsed.tzinfo is None:
            parsed = parsed.replace(tzinfo=dt.timezone.utc)
        return parsed.timestamp()
    except ValueError:
        try:
            return float(text)
        except ValueError:
            return None


def coerce_reply(raw):
    if isinstance(raw, dict):
        return raw
    if isinstance(raw, str):
        try:
            value = json.loads(raw)
        except (ValueError, TypeError):
            return None
        return value if isinstance(value, dict) else None
    return None


def load_rows(path):
    rows = []
    parse_errors = []
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line_no, line in enumerate(handle, 1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                obj = json.loads(stripped)
            except (ValueError, TypeError) as exc:
                parse_errors.append({"line": line_no, "error": repr(exc)})
                rows.append({
                    "line_no": line_no,
                    "i": len(rows),
                    "t": None,
                    "epoch": None,
                    "ok": False,
                    "reply": None,
                    "gap": True,
                    "gap_reason": "line_unparsed",
                    "source_keys": [],
                    "reply_raw_type": None,
                })
                continue
            if isinstance(obj, dict) and "marker" in obj and "i" not in obj:
                continue
            if not isinstance(obj, dict):
                parse_errors.append({"line": line_no, "error": "top level is not an object"})
                rows.append({
                    "line_no": line_no,
                    "i": len(rows),
                    "t": None,
                    "epoch": None,
                    "ok": False,
                    "reply": None,
                    "gap": True,
                    "gap_reason": "line_not_object",
                    "source_keys": [],
                    "reply_raw_type": None,
                })
                continue
            try:
                index = int(obj.get("i", len(rows)))
            except (TypeError, ValueError):
                index = len(rows)
            t_value = obj.get("t")
            epoch = parse_timestamp(t_value)
            ok = truthy(obj.get("ok", False))
            reply_raw = obj.get("reply") if ok else None
            reply = coerce_reply(reply_raw) if ok else None
            gap_reason = None
            if not ok:
                gap_reason = "ok_false"
            elif reply_raw is None:
                gap_reason = "reply_missing"
            elif reply is None:
                gap_reason = "reply_unparsed"
            rows.append({
                "line_no": line_no,
                "i": index,
                "t": t_value,
                "epoch": epoch,
                "ok": ok,
                "reply": reply,
                "gap": reply is None,
                "gap_reason": gap_reason,
                "source_keys": sorted(obj.keys()),
                "reply_raw_type": type(reply_raw).__name__ if reply_raw is not None else None,
            })
    epochs = [row["epoch"] for row in rows if row["epoch"] is not None]
    start_epoch = min(epochs) if epochs else None
    for row in rows:
        if row["epoch"] is not None and start_epoch is not None:
            row["t_s"] = row["epoch"] - start_epoch
        else:
            row["t_s"] = float(row["i"])
    return rows, parse_errors, start_epoch


def as_number(value):
    if isinstance(value, bool):
        return None
    if isinstance(value, (int, float)):
        return float(value)
    if isinstance(value, str):
        try:
            return float(value)
        except ValueError:
            return None
    return None


def nested_value(reply, *keys):
    current = reply
    for key in keys:
        if not isinstance(current, dict) or key not in current:
            return None
        current = current[key]
    return current


def nested_number(reply, *keys):
    return as_number(nested_value(reply, *keys))


def numeric_stats(values):
    if not values:
        return None
    ordered = sorted(float(v) for v in values)
    count = len(ordered)
    p95_index = min(count - 1, int(round(0.95 * (count - 1))))
    return {
        "count": count,
        "min": round(ordered[0], 6),
        "max": round(ordered[-1], 6),
        "mean": round(statistics.fmean(ordered), 6),
        "median": round(statistics.median(ordered), 6),
        "p95": round(ordered[p95_index], 6),
        "stdev": round(statistics.pstdev(ordered) if count > 1 else 0.0, 6),
        "spread": round(ordered[-1] - ordered[0], 6),
    }


def collect_numbers(valid_rows, keys):
    values = []
    for row in valid_rows:
        value = nested_number(row["reply"], *keys)
        if value is not None:
            values.append(value)
    return values


def sample_period_seconds(valid_rows):
    diffs = []
    for previous, current in zip(valid_rows, valid_rows[1:]):
        delta = current["t_s"] - previous["t_s"]
        if delta > 0:
            diffs.append(delta)
    return round(statistics.median(diffs), 6) if diffs else 1.0


def analyze_gaps(rows, sample_period, burst_threshold):
    clusters = []
    current_cluster = None
    for row in rows:
        if row["gap"]:
            if current_cluster is None:
                current_cluster = {
                    "start_i": row["i"],
                    "end_i": row["i"],
                    "start_t": row["t"],
                    "end_t": row["t"],
                    "start_s": row["t_s"],
                    "end_s": row["t_s"],
                    "length": 1,
                    "reasons": {row["gap_reason"]: 1},
                }
            else:
                current_cluster["end_i"] = row["i"]
                current_cluster["end_t"] = row["t"]
                current_cluster["end_s"] = row["t_s"]
                current_cluster["length"] += 1
                reason = row["gap_reason"]
                current_cluster["reasons"][reason] = current_cluster["reasons"].get(reason, 0) + 1
        elif current_cluster is not None:
            current_cluster["duration_s"] = round(current_cluster["end_s"] - current_cluster["start_s"] + sample_period, 6)
            clusters.append(current_cluster)
            current_cluster = None
    if current_cluster is not None:
        current_cluster["duration_s"] = round(current_cluster["end_s"] - current_cluster["start_s"] + sample_period, 6)
        clusters.append(current_cluster)
    gap_count = sum(1 for row in rows if row["gap"])
    reasons = {}
    for row in rows:
        if row["gap"]:
            reasons[row["gap_reason"]] = reasons.get(row["gap_reason"], 0) + 1
    bursts = [cluster for cluster in clusters if cluster["length"] >= burst_threshold]
    return {
        "gap_count": gap_count,
        "gap_pct": round(100.0 * gap_count / len(rows), 6) if rows else None,
        "clusters": clusters,
        "cluster_count": len(clusters),
        "burst_threshold": burst_threshold,
        "bursts": bursts,
        "burst_count": len(bursts),
        "reasons": reasons,
    }


def analyze_monotonic(valid_rows):
    counters = {}
    for label, keys in (
        ("resample_fold_samples", ("resample_fold", "samples")),
        ("resample_chain_samples", ("resample_chain", "samples")),
    ):
        resets = []
        previous = None
        previous_row = None
        first = None
        last = None
        for row in valid_rows:
            value = nested_number(row["reply"], *keys)
            if value is None:
                continue
            if first is None:
                first = value
            if previous is not None and value < previous:
                resets.append({
                    "from_i": previous_row["i"],
                    "to_i": row["i"],
                    "from": previous,
                    "to": value,
                    "delta": round(value - previous, 6),
                })
            previous = value
            previous_row = row
            last = value
        counters[label] = {
            "first": first,
            "last": last,
            "monotonic": len(resets) == 0,
            "backward_steps": len(resets),
            "resets": resets,
            "interpretation": "a backwards step means an aloop restart",
        }
    beat_backwards = []
    previous_beat = None
    previous_row = None
    beat_first = None
    beat_last = None
    for row in valid_rows:
        beat = nested_number(row["reply"], "beat_mark", "beat")
        if beat is None:
            continue
        if beat_first is None:
            beat_first = beat
        if previous_beat is not None and beat < previous_beat - 1e-6:
            beat_backwards.append({
                "from_i": previous_row["i"],
                "to_i": row["i"],
                "from": previous_beat,
                "to": beat,
                "delta": round(beat - previous_beat, 6),
            })
        previous_beat = beat
        previous_row = row
        beat_last = beat
    return {
        "counters": counters,
        "beat_mark_beat": {
            "first": beat_first,
            "last": beat_last,
            "monotonic": len(beat_backwards) == 0,
            "backward_steps": len(beat_backwards),
            "backwards": beat_backwards,
        },
    }


def grid_distance_to_boundary(value, quantum):
    if value is None:
        return None
    wrapped = value % quantum
    return min(wrapped, quantum - wrapped)


def phase_distance_to_phrase(value, phrase_beats):
    if value is None:
        return None
    wrapped = value % phrase_beats
    return min(wrapped, phrase_beats - wrapped)


def analyze_boundaries(valid_rows, sample_period, constants, args):
    quantum = constants["LINK_QUANTUM"]
    phrase_beats = constants["PHRASE_BEATS"]
    phase_jump_threshold = args.phase_jump_threshold
    alignment_tolerance = args.boundary_alignment_tolerance
    spacing_tolerance = args.spacing_tolerance_seconds
    reset_alignment_tolerance = max(2.0, alignment_tolerance)
    anomalies = []
    boundaries = []
    resets = []
    grid_mismatches = []
    phase_jumps = []
    spacings = []
    bpms = collect_numbers(valid_rows, ("link", "bpm"))
    nominal_bpm = round(statistics.median(bpms), 6) if bpms else 120.0
    expected_seconds = phrase_beats * 60.0 / nominal_bpm if nominal_bpm > 0 else None
    expected_samples = expected_seconds / sample_period if expected_seconds and sample_period > 0 else None
    for previous, current in zip(valid_rows, valid_rows[1:]):
        g_previous = nested_number(previous["reply"], "grid_beat_index")
        g_current = nested_number(current["reply"], "grid_beat_index")
        beat_previous = nested_number(previous["reply"], "beat_mark", "beat")
        beat_current = nested_number(current["reply"], "beat_mark", "beat")
        mp_previous = nested_number(previous["reply"], "master_phase_beats")
        mp_current = nested_number(current["reply"], "master_phase_beats")
        rec_previous = nested_number(previous["reply"], "recorded_beats")
        rec_current = nested_number(current["reply"], "recorded_beats")
        if None in (g_previous, g_current, beat_previous, beat_current):
            continue
        beat_delta = beat_current - beat_previous
        if beat_delta < -1e-6:
            anomalies.append({
                "type": "backwards_beat",
                "from_i": previous["i"],
                "to_i": current["i"],
                "from_beat": beat_previous,
                "to_beat": beat_current,
                "delta": round(beat_delta, 6),
            })
        grid_delta_mod = (g_current - g_previous) % quantum
        if grid_delta_mod == 0 and beat_delta >= quantum / 2.0:
            grid_delta_mod = quantum
        wraps = 0 if beat_delta <= 0 else round((beat_delta - grid_delta_mod) / quantum)
        if wraps < 0:
            wraps = 0
        implied_grid_delta = grid_delta_mod + quantum * wraps
        grid_mismatch = abs(implied_grid_delta - beat_delta)
        if grid_mismatch > phase_jump_threshold:
            record = {
                "from_i": previous["i"],
                "to_i": current["i"],
                "beat_delta": round(beat_delta, 6),
                "implied_grid_delta": round(implied_grid_delta, 6),
                "mismatch_beats": round(grid_mismatch, 6),
                "threshold_beats": phase_jump_threshold,
            }
            grid_mismatches.append(record)
            anomalies.append({"type": "grid_phase_jump", **record})
        master_active = (
            (mp_previous is not None and mp_previous > 0.5)
            or (mp_current is not None and mp_current > 0.5)
            or (rec_previous is not None and rec_previous > 0)
            or (rec_current is not None and rec_current > 0)
        )
        if beat_delta > 0:
            wrap_beat = (math.floor(beat_previous / quantum) + 1) * quantum
            while wrap_beat <= beat_current + 1e-9 and len(resets) < 10000:
                wrap_fraction = (wrap_beat - beat_previous) / beat_delta
                wrap_t = previous["t_s"] + wrap_fraction * (current["t_s"] - previous["t_s"])
                expected_grid_at_wrap = (g_previous + (wrap_beat - beat_previous)) % quantum
                grid_residual = min(expected_grid_at_wrap, quantum - expected_grid_at_wrap)
                wrap_mod = wrap_beat % phrase_beats
                phrase_distance = min(wrap_mod, phrase_beats - wrap_mod)
                record = {
                    "type": "grid_quantum_wrap",
                    "from_i": previous["i"],
                    "to_i": current["i"],
                    "wrap_beat": wrap_beat,
                    "wrap_t_s": round(wrap_t, 6),
                    "phrase_distance_beats": round(phrase_distance, 6),
                    "grid_residual_beats": round(grid_residual, 6),
                    "grid_mismatch_beats": round(grid_mismatch, 6),
                    "observed_grid_decrease": g_current < g_previous,
                }
                resets.append(record)
                if phrase_distance <= alignment_tolerance and (grid_residual > reset_alignment_tolerance or grid_mismatch > alignment_tolerance):
                    reset_anomaly = dict(record)
                    reset_anomaly["type"] = "reset_not_64_aligned"
                    anomalies.append(reset_anomaly)
                wrap_beat += quantum
        if master_active and mp_previous is not None and mp_current is not None:
            loop_length = phrase_beats
            if rec_current is not None and rec_current > 0:
                loop_length = rec_current
            elif rec_previous is not None and rec_previous > 0:
                loop_length = rec_previous
            mp_delta = mp_current - mp_previous
            expected_wrap = mp_previous > mp_current and mp_current < max(1.0, alignment_tolerance) and loop_length > 0
            if expected_wrap:
                mp_mod = beat_current % phrase_beats
                mp_distance = min(mp_mod, phrase_beats - mp_mod)
                record = {
                    "type": "master_phase_reset",
                    "from_i": previous["i"],
                    "to_i": current["i"],
                    "beat": beat_current,
                    "from_phase": mp_previous,
                    "to_phase": mp_current,
                    "phrase_distance_beats": round(mp_distance, 6),
                    "loop_length_beats": loop_length,
                }
                resets.append(record)
                if mp_distance > alignment_tolerance:
                    reset_anomaly = dict(record)
                    reset_anomaly["type"] = "reset_not_64_aligned"
                    anomalies.append(reset_anomaly)
            elif abs(mp_delta - beat_delta) > phase_jump_threshold:
                record = {
                    "from_i": previous["i"],
                    "to_i": current["i"],
                    "phase_delta": round(mp_delta, 6),
                    "beat_delta": round(beat_delta, 6),
                    "mismatch_beats": round(abs(mp_delta - beat_delta), 6),
                    "threshold_beats": phase_jump_threshold,
                }
                phase_jumps.append(record)
                anomalies.append({"type": "phase_jump", **record})
        if beat_delta > 0:
            phrase_previous = math.floor(beat_previous / phrase_beats)
            phrase_current = math.floor(beat_current / phrase_beats)
            for phrase_number in range(phrase_previous + 1, phrase_current + 1):
                boundary_beat = phrase_number * phrase_beats
                fraction = (boundary_beat - beat_previous) / beat_delta
                if fraction < 0 or fraction > 1:
                    continue
                boundary_t = previous["t_s"] + fraction * (current["t_s"] - previous["t_s"])
                boundary_sample = previous["i"] + fraction * (current["i"] - previous["i"])
                previous_grid_distance = grid_distance_to_boundary(g_previous, quantum)
                current_grid_distance = grid_distance_to_boundary(g_current, quantum)
                grid_distance = min(previous_grid_distance, current_grid_distance)
                mp_distance = None
                if master_active and mp_previous is not None and mp_current is not None:
                    mp_distance = min(
                        phase_distance_to_phrase(mp_previous, phrase_beats),
                        phase_distance_to_phrase(mp_current, phrase_beats),
                    )
                aligned = grid_distance <= alignment_tolerance and (mp_distance is None or mp_distance <= alignment_tolerance)
                record = {
                    "phrase_number": phrase_number,
                    "boundary_beat": boundary_beat,
                    "t_s": round(boundary_t, 6),
                    "sample": round(boundary_sample, 6),
                    "from_i": previous["i"],
                    "to_i": current["i"],
                    "grid_beat_index_prev": g_previous,
                    "grid_beat_index_cur": g_current,
                    "grid_distance_beats": round(grid_distance, 6),
                    "master_phase_distance_beats": round(mp_distance, 6) if mp_distance is not None else None,
                    "master_active": master_active,
                    "aligned": aligned,
                }
                boundaries.append(record)
                if not aligned:
                    anomalies.append({"type": "phrase_boundary_misalignment", **record})
    for previous, current in zip(boundaries, boundaries[1:]):
        seconds = current["t_s"] - previous["t_s"]
        samples = current["sample"] - previous["sample"]
        seconds_error = seconds - expected_seconds if expected_seconds is not None else None
        samples_error = samples - expected_samples if expected_samples is not None else None
        outlier = (
            (seconds_error is not None and abs(seconds_error) > spacing_tolerance)
            or (samples_error is not None and abs(samples_error) > 1.0)
        )
        record = {
            "from_phrase": previous["phrase_number"],
            "to_phrase": current["phrase_number"],
            "seconds": round(seconds, 6),
            "samples": round(samples, 6),
            "expected_seconds": round(expected_seconds, 6) if expected_seconds is not None else None,
            "expected_samples": round(expected_samples, 6) if expected_samples is not None else None,
            "seconds_error": round(seconds_error, 6) if seconds_error is not None else None,
            "samples_error": round(samples_error, 6) if samples_error is not None else None,
            "outlier": outlier,
        }
        spacings.append(record)
        if outlier:
            anomalies.append({"type": "phrase_boundary_spacing_outlier", **record})
    spacing_seconds = [item["seconds"] for item in spacings]
    spacing_samples = [item["samples"] for item in spacings]
    return {
        "constants": constants,
        "thresholds": {
            "phase_jump_beats": phase_jump_threshold,
            "boundary_alignment_beats": alignment_tolerance,
            "reset_alignment_beats": reset_alignment_tolerance,
            "spacing_tolerance_seconds": spacing_tolerance,
        },
        "sample_period_seconds": sample_period,
        "nominal_bpm": nominal_bpm,
        "expected_seconds": round(expected_seconds, 6) if expected_seconds is not None else None,
        "expected_samples": round(expected_samples, 6) if expected_samples is not None else None,
        "boundaries": boundaries,
        "boundary_count": len(boundaries),
        "resets": resets,
        "grid_mismatches": grid_mismatches,
        "phase_jumps": phase_jumps,
        "spacings": spacings,
        "spacing_stats": {
            "seconds": numeric_stats(spacing_seconds),
            "samples": numeric_stats(spacing_samples),
            "max_abs_seconds_error": round(max(abs(item["seconds_error"]) for item in spacings if item["seconds_error"] is not None), 6) if spacings else None,
        },
        "anomalies": anomalies,
        "beat_mark_note": "beat_mark is not a boundary flag; beat_mark.beat is used only as a continuous absolute-beat clock, while grid_beat_index and master_phase_beats validate boundary alignment.",
    }


def analyze_core_busy(valid_rows):
    per_core = {}
    all_values = []
    for row in valid_rows:
        busy = row["reply"].get("core_busy")
        if not isinstance(busy, list):
            continue
        for index, raw in enumerate(busy):
            value = as_number(raw)
            if value is None:
                continue
            per_core.setdefault(str(index), []).append(value)
            all_values.append(value)
    return {
        "per_core": {key: numeric_stats(values) for key, values in sorted(per_core.items())},
        "overall": numeric_stats(all_values),
    }


def pad_mark_valid(row):
    age = nested_number(row["reply"], "beat_pad_mark", "age_ms")
    seq = nested_number(row["reply"], "beat_pad_mark", "seq")
    index = nested_number(row["reply"], "beat_pad_mark", "index")
    return age is not None and age >= 0 and seq is not None and seq > 0 and index is not None and index >= 0


def analyze_jitter(valid_rows):
    def series(keys, row_filter=None):
        values = []
        for row in valid_rows:
            if row_filter is not None and not row_filter(row):
                continue
            value = nested_number(row["reply"], *keys)
            if value is not None:
                values.append(value)
        return numeric_stats(values)
    result = {
        "beat_mark_late_ms_sawtooth": series(("beat_mark", "late_ms")),
        "beat_mark_ms_to_next": series(("beat_mark", "ms_to_next")),
        "beat_pad_mark_late_ms": series(("beat_pad_mark", "late_ms"), pad_mark_valid),
        "metro_late_last_us": series(("metro", "late_last_us")),
        "metro_late_worst_us": series(("metro", "late_worst_us")),
        "metro_late_mean_us": series(("metro", "late_mean_us")),
        "metro_late_rms_us": series(("metro", "late_rms_us")),
    }
    result["beat_mark_late_ms_interpretation"] = "sawtooth position indicator (0.3..500 ms, mean about 250 ms), not lateness and not a timing error"
    return result


def count_anomaly_types(anomalies):
    counts = {}
    for anomaly in anomalies:
        kind = anomaly["type"]
        counts[kind] = counts.get(kind, 0) + 1
    return counts


def score_acceptance(metrics, gaps, post60):
    terms = []
    retention = post60["retention_pct"]
    retention_pass = retention is not None and post60["reply_count"] > 0 and retention >= ACCEPTANCE["retention_min_pct"]
    terms.append({
        "term": "post_t_plus_60s_peer_retention_among_replies",
        "value": retention,
        "threshold": ">=95.0%",
        "pass": bool(retention_pass),
        "result": "PASS" if retention_pass else "FAIL",
        "detail": "replies=" + str(post60["reply_count"]) + " peers>=1=" + str(post60["peers_ge1"]) + " peers=0=" + str(post60["peers0"]),
    })
    bpm_spread = metrics["bpm"]["spread"] if metrics["bpm"] else None
    bpm_pass = bpm_spread is not None and bpm_spread <= ACCEPTANCE["bpm_spread_max"]
    terms.append({
        "term": "bpm_spread",
        "value": bpm_spread,
        "threshold": "<=0.05",
        "pass": bool(bpm_pass),
        "result": "PASS" if bpm_pass else "FAIL",
        "detail": "max-min of link.bpm among replies",
    })
    phase_err = metrics["max_abs_phase_err_beats"]
    phase_pass = phase_err is not None and phase_err <= ACCEPTANCE["phase_err_abs_max"]
    terms.append({
        "term": "max_abs_phase_err_beats",
        "value": phase_err,
        "threshold": "<=0.03",
        "pass": bool(phase_pass),
        "result": "PASS" if phase_pass else "FAIL",
        "detail": "max |link.phase_err_beats| among replies",
    })
    eff_spread = metrics["eff_speed"]["spread"] if metrics["eff_speed"] else None
    eff_pass = eff_spread is not None and eff_spread <= ACCEPTANCE["eff_speed_spread_max"]
    terms.append({
        "term": "eff_speed_spread",
        "value": eff_spread,
        "threshold": "<=0.005",
        "pass": bool(eff_pass),
        "result": "PASS" if eff_pass else "FAIL",
        "detail": "max-min of eff_speed among replies",
    })
    gap_pct = gaps["gap_pct"]
    gap_pass = gap_pct is not None and gap_pct <= ACCEPTANCE["gap_pct_max"]
    terms.append({
        "term": "sample_gaps",
        "value": gap_pct,
        "threshold": "<=10.0%",
        "pass": bool(gap_pass),
        "result": "PASS" if gap_pass else "FAIL",
        "detail": "no reply / ok:false / missing or unparsed reply count divided by total samples",
    })
    overall = "PASS" if all(term["pass"] for term in terms) else "FAIL"
    return {
        "source": ACCEPTANCE_SOURCE,
        "thresholds": ACCEPTANCE,
        "terms": terms,
        "overall": overall,
    }


def analyze(rows, args, constants, parse_errors=None):
    if parse_errors is None:
        parse_errors = []
    valid_rows = [row for row in rows if not row["gap"]]
    sample_period = sample_period_seconds(valid_rows)
    gaps = analyze_gaps(rows, sample_period, args.gap_burst_threshold)
    monotonic = analyze_monotonic(valid_rows)
    boundaries = analyze_boundaries(valid_rows, sample_period, constants, args)
    core_busy = analyze_core_busy(valid_rows)
    jitter = analyze_jitter(valid_rows)
    bpm = numeric_stats(collect_numbers(valid_rows, ("link", "bpm")))
    eff_speed = numeric_stats(collect_numbers(valid_rows, ("eff_speed",)))
    phase_errs = [abs(value) for value in collect_numbers(valid_rows, ("link", "phase_err_beats"))]
    max_abs_phase_err = round(max(phase_errs), 6) if phase_errs else None
    xruns = collect_numbers(valid_rows, ("xruns",))
    peers_values = collect_numbers(valid_rows, ("link", "peers"))
    peers_histogram = {}
    for value in peers_values:
        key = str(int(value))
        peers_histogram[key] = peers_histogram.get(key, 0) + 1
    peers0_count = sum(1 for value in peers_values if value == 0)
    post_rows = [row for row in rows if row["t_s"] >= 60.0]
    post_valid = [row for row in post_rows if not row["gap"]]
    post_peers = [nested_number(row["reply"], "link", "peers") for row in post_valid]
    post_peers = [value for value in post_peers if value is not None]
    post_peers_ge1 = sum(1 for value in post_peers if value >= 1)
    post_peers0 = sum(1 for value in post_peers if value == 0)
    post60 = {
        "reply_count": len(post_peers),
        "peers_ge1": post_peers_ge1,
        "peers0": post_peers0,
        "retention_pct": round(100.0 * post_peers_ge1 / len(post_peers), 6) if post_peers else None,
        "no_reply_count": sum(1 for row in post_rows if row["gap"]),
    }
    anomalies = []
    for burst in gaps["bursts"]:
        anomalies.append({
            "type": "gap_burst",
            "start_i": burst["start_i"],
            "end_i": burst["end_i"],
            "length": burst["length"],
            "threshold": args.gap_burst_threshold,
        })
    for label, counter in monotonic["counters"].items():
        for reset in counter["resets"]:
            anomalies.append({"type": "resample_counter_reset", "counter": label, **reset})
    for backwards in monotonic["beat_mark_beat"]["backwards"]:
        anomalies.append({"type": "backwards_beat", **backwards})
    anomalies.extend(boundaries["anomalies"])
    metrics = {
        "total_samples": len(rows),
        "reply_count": len(valid_rows),
        "no_reply_count": gaps["gap_count"],
        "peers0_count": peers0_count,
        "peers_histogram": peers_histogram,
        "parse_error_count": len(parse_errors),
        "parse_errors": parse_errors,
        "first_i": rows[0]["i"] if rows else None,
        "last_i": rows[-1]["i"] if rows else None,
        "first_t": rows[0]["t"] if rows else None,
        "last_t": rows[-1]["t"] if rows else None,
        "span_seconds": round(rows[-1]["t_s"] - rows[0]["t_s"], 6) if rows else None,
        "sample_period_seconds": sample_period,
        "bpm": bpm,
        "eff_speed": eff_speed,
        "max_abs_phase_err_beats": max_abs_phase_err,
        "xruns": {
            "start": xruns[0] if xruns else None,
            "end": xruns[-1] if xruns else None,
            "min": min(xruns) if xruns else None,
            "max": max(xruns) if xruns else None,
            "delta": round(xruns[-1] - xruns[0], 6) if xruns else None,
        },
        "core_busy": core_busy,
        "jitter": jitter,
        "post60": post60,
    }
    acceptance = score_acceptance(metrics, gaps, post60)
    return {
        "metrics": metrics,
        "gaps": gaps,
        "monotonic": monotonic,
        "boundaries": boundaries,
        "acceptance": acceptance,
        "anomalies": anomalies,
        "anomaly_counts": count_anomaly_types(anomalies),
    }


def generate_synthetic_rows():
    base = dt.datetime.fromisoformat("2026-11-05T20:00:00+00:00")
    rows = []
    previous_fold = None
    previous_chain = None
    for index in range(160):
        beat = 25600.0 + 2.0 * index
        timestamp = (base + dt.timedelta(seconds=index)).isoformat(timespec="milliseconds").replace("+00:00", "Z")
        grid = int(beat) % 16
        beat_mark_beat = beat
        master_phase = beat % 64.0
        fold = 1000000 + index * 38400
        chain = 1000000 + index * 38400
        peers = 1
        if index == 32:
            grid = 3
        if index == 50:
            peers = 0
        if index == 70:
            master_phase = 0.2
        if index == 100:
            beat_mark_beat = beat - 5.0
        if index == 110:
            master_phase = (beat % 64.0) + 10.0
        if index == 120:
            fold = previous_fold - 1000
            chain = previous_chain - 1000
        if 140 <= index <= 144:
            rows.append({"i": index, "t": timestamp, "ok": False})
            previous_fold = fold
            previous_chain = chain
            continue
        reply = {
            "core_busy": [0, 30 + index % 5, 0, 0],
            "xruns": 0,
            "link": {"synced": True, "bpm": 120.0, "peers": peers, "playing": True, "phase_err_beats": 0.0},
            "eff_speed": 1.0,
            "resample_fold": {"sum": 0.0, "samples": fold},
            "resample_chain": {"sum": 0.0, "samples": chain},
            "grid_beat_index": grid,
            "master_phase_beats": master_phase,
            "master_len_samples": 0.0,
            "recorded_beats": 64.0,
            "beat_mark": {
                "valid": True,
                "beat": beat_mark_beat,
                "index": int(beat) % 16,
                "ms_to_next": 500.0 - (beat % 1.0) * 500.0,
                "late_ms": (beat % 1.0) * 500.0,
            },
            "beat_pad_mark": {"seq": 0, "index": -1, "late_ms": 0.0, "age_ms": -1, "bpm": 0.0},
        }
        rows.append({
            "i": index,
            "t": timestamp,
            "ok": True,
            "reply": json.dumps(reply, separators=(",", ":")),
        })
        previous_fold = fold
        previous_chain = chain
    return rows


def write_self_test_data(rows, data_path=SELF_TEST_DATA):
    path = Path(data_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as handle:
        for row in rows:
            handle.write(json.dumps(row, separators=(",", ":")) + "\n")


def write_self_test_report(result, report_path=SELF_TEST_REPORT, data_path=SELF_TEST_DATA):
    path = Path(report_path)
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [
        "PHRASE TELEMETRY SYNTHETIC SELF-TEST",
        "RESULT: " + ("PASS" if result["passed"] else "FAIL"),
        "GENERATED_UTC: " + result["generated_utc"],
        "GENERATED_DATA: " + data_path,
        "REQUIRED_FLAGS: " + ", ".join(result["required_flags"]),
        "FLAG_COUNTS: " + json.dumps(result["flag_counts"], sort_keys=True),
        "ASSERTIONS:",
        json.dumps(result["assertions"], indent=2, sort_keys=True),
        "NO_REPLY_VS_PEERS0:",
        json.dumps(result["no_reply_vs_peers0"], indent=2, sort_keys=True),
        "BOUNDARY_ANOMALIES:",
        json.dumps(result["boundary_anomalies"], indent=2, sort_keys=True),
    ]
    if result.get("error"):
        lines.append("ERROR: " + str(result["error"]))
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def run_self_test(args, constants):
    rows = generate_synthetic_rows()
    data_path, report_path = SELF_TEST_DATA, SELF_TEST_REPORT
    try:
        write_self_test_data(rows, data_path)
    except OSError:
        import tempfile
        scratch = tempfile.mkdtemp(prefix="phrase-self-test-")
        data_path = str(Path(scratch) / "data.jsonl")
        report_path = str(Path(scratch) / "report.txt")
        write_self_test_data(rows, data_path)
    loaded, parse_errors, start_epoch = load_rows(data_path)
    analysis = analyze(loaded, args, constants, parse_errors)
    flags = set(analysis["anomaly_counts"].keys())
    required_flags = [
        "backwards_beat",
        "phrase_boundary_misalignment",
        "resample_counter_reset",
        "gap_burst",
        "reset_not_64_aligned",
        "phase_jump",
    ]
    assertions = [{"flag": flag, "present": flag in flags} for flag in required_flags]
    no_reply_vs_peers0 = {
        "no_reply_count": analysis["metrics"]["no_reply_count"],
        "peers0_count": analysis["metrics"]["peers0_count"],
        "distinct": analysis["metrics"]["no_reply_count"] > 0 and analysis["metrics"]["peers0_count"] > 0,
    }
    result = {
        "passed": all(item["present"] for item in assertions) and no_reply_vs_peers0["distinct"],
        "generated_rows": len(rows),
        "required_flags": required_flags,
        "assertions": assertions,
        "flag_counts": analysis["anomaly_counts"],
        "no_reply_vs_peers0": no_reply_vs_peers0,
        "boundary_anomalies": [item for item in analysis["anomalies"] if item["type"].startswith("phrase_") or item["type"].startswith("reset_")][:20],
        "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        "report": report_path,
    }
    try:
        write_self_test_report(result, report_path, data_path)
    except OSError:
        import tempfile
        report_path = str(Path(tempfile.mkdtemp(prefix="phrase-self-test-")) / "report.txt")
        result["report"] = report_path
        write_self_test_report(result, report_path, data_path)
    if not result["passed"]:
        raise AssertionError("synthetic self-test missing required flags")
    return result


def load_json_if_exists(path):
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (OSError, ValueError):
        return None


def input_context(path, rows):
    try:
        stat = os.stat(path)
        size = stat.st_size
        mtime = dt.datetime.fromtimestamp(stat.st_mtime, dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z")
        digest = hashlib.sha256(Path(path).read_bytes()).hexdigest()
    except OSError:
        size = None
        mtime = None
        digest = None
    first_valid = next((row for row in rows if not row["gap"]), None)
    reply_keys = sorted(first_valid["reply"].keys()) if first_valid else []
    nested_keys = {}
    if first_valid:
        for key in ("link", "beat_mark", "beat_pad_mark", "resample_fold", "resample_chain", "loopers", "groove"):
            value = first_valid["reply"].get(key)
            nested_keys[key] = sorted(value.keys()) if isinstance(value, dict) else type(value).__name__
    return {
        "path": path,
        "size_bytes": size,
        "mtime_utc": mtime,
        "sha256": digest,
        "first_top_level_keys": rows[0]["source_keys"] if rows else [],
        "first_reply_raw_type": rows[0]["reply_raw_type"] if rows else None,
        "first_valid_reply_keys": reply_keys,
        "first_valid_nested_keys": nested_keys,
        "parser_rules": [
            "ok:false or missing ok is a gap",
            "missing, empty, or unparsed reply is a gap",
            "reply may be a JSON string or an object",
            "no telemetry value is imputed for a gap",
        ],
        "analyzer_network_reads": False,
    }


def completion_context(path, rows, args):
    now = dt.datetime.now(dt.timezone.utc).timestamp()
    last_i = rows[-1]["i"] if rows else None
    last_t = rows[-1]["t"] if rows else None
    epochs = [row["epoch"] for row in rows if row["epoch"] is not None]
    last_epoch = max(epochs) if epochs else None
    last_age = now - last_epoch if last_epoch is not None else None
    try:
        file_age = now - os.path.getmtime(path)
    except OSError:
        file_age = None
    fallback = load_json_if_exists(FALLBACK_WITNESS)
    fallback_complete = bool(fallback and fallback.get("status") == "fallback_complete")
    proxy_complete = (
        last_i is not None
        and last_i >= 1790
        and last_age is not None
        and last_age >= 90
        and file_age is not None
        and file_age >= 90
    )
    closure = load_json_if_exists(FINAL_WITNESS)
    return {
        "status": args.status,
        "final_requires": "420 completion signal relayed by the coordinator",
        "completion_signal": args.completion_signal,
        "completion_criterion": COMPLETION_CRITERION,
        "line_count": len(rows),
        "last_sample_index": last_i,
        "last_sample_timestamp": last_t,
        "last_sample_age_seconds": round(last_age, 3) if last_age is not None else None,
        "file_mtime_age_seconds": round(file_age, 3) if file_age is not None else None,
        "single_read_stability_proxy": bool(proxy_complete),
        "fallback_witness": fallback,
        "self_detect_complete": bool(fallback_complete or proxy_complete),
        "promoted_to_final": args.status == "FINAL" and bool(args.completion_signal.strip()),
        "witness_closure": closure,
        "witness_closed": bool(closure and closure.get("status") == "CLOSED"),
    }


def esp_context(args):
    meta = load_json_if_exists(ESP_META)
    extension = load_json_if_exists(ESP_EXTENSION_META)
    baseline_result = analyze_esp_status_file(ESP_STATUS_FILE)
    baseline_rows, baseline_events, baseline_parse_errors = load_esp_status_rows(ESP_STATUS_FILE)
    baseline_part = [row for row in baseline_rows if row.get("i", 0) < 76]
    extension_part = [row for row in baseline_rows if row.get("i", 0) >= 76]
    continuation_result = {
        "baseline_samples": len(baseline_part),
        "baseline_replies": sum(1 for row in baseline_part if row.get("ok")),
        "baseline_no_reply": sum(1 for row in baseline_part if not row.get("ok")),
        "extension_samples": len(extension_part),
        "extension_replies": sum(1 for row in extension_part if row.get("ok")),
        "extension_no_reply": sum(1 for row in extension_part if not row.get("ok")),
        "extension_no_reply_details": [row for row in extension_part if not row.get("ok")][:10],
        "first_baseline_t": baseline_part[0].get("timestamp") if baseline_part else None,
        "last_baseline_t": baseline_part[-1].get("timestamp") if baseline_part else None,
        "first_extension_t": extension_part[0].get("timestamp") if extension_part else None,
        "last_extension_t": extension_part[-1].get("timestamp") if extension_part else None,
        "events": baseline_events,
        "parse_error_count": len(baseline_parse_errors),
    }
    control_meta = load_json_if_exists(CONTROL_ESP_META)
    control_result = analyze_esp_status_file(args.control_esp)
    pid = None
    pid_alive = False
    try:
        pid = open(CONTROL_ESP_PID, "r", encoding="utf-8").read().strip()
        pid_alive = os.path.exists("/proc/" + pid)
    except OSError:
        pass
    return {
        "status_file": ESP_STATUS_FILE,
        "baseline": meta,
        "no_restart_continuation": extension,
        "eta_baseline_end_utc": meta.get("expected_end_utc") if meta else "2026-11-05T20:27:54Z",
        "eta_extension_end_utc": extension.get("extension_end_utc") if extension else "2026-11-05T20:30:30Z",
        "baseline_result": baseline_result,
        "continuation_result": continuation_result,
        "control_window": {
            "meta": control_meta,
            "status_file": args.control_esp,
            "result": control_result,
            "pid": pid,
            "pid_alive": pid_alive,
            "no_fixed_end_before_utc": "2026-11-05T21:15:00Z",
        },
        "command": "node /config/workspace/mc-420/test/hardware/ssh-run.js 192.168.137.132 \"printf 'P' | nc -u -w1 192.168.4.19 20813\"",
        "host_key_sha256": HOST_KEY_SHA256,
        "host_key_verified_out_of_band": True,
        "known_hosts_modified": False,
        "subscriber_note": SUBSCRIBER_NOTE,
        "read_only": True,
    }


def compact_summary(result):
    analysis = result["analysis"]
    metrics = analysis["metrics"]
    boundaries = analysis["boundaries"]
    acceptance = analysis["acceptance"]
    anomaly_counts = analysis["anomaly_counts"]
    phrase_anomaly = any(kind in anomaly_counts for kind in (
        "phrase_boundary_misalignment",
        "reset_not_64_aligned",
        "phrase_boundary_spacing_outlier",
    ))
    baseline_esp = result["esp_sampling"]["baseline_result"]
    control_esp = result["esp_sampling"]["control_window"]["result"]
    return {
        "ok": True,
        "status": result["status"],
        "input": result["input"],
        "report": result["report"],
        "sample_count": metrics["total_samples"],
        "reply_count": metrics["reply_count"],
        "no_reply_count": metrics["no_reply_count"],
        "peers0_count": metrics["peers0_count"],
        "gap_pct": analysis["gaps"]["gap_pct"],
        "gap_clusters": analysis["gaps"]["cluster_count"],
        "post60_retention_pct": metrics["post60"]["retention_pct"],
        "bpm_spread": metrics["bpm"]["spread"] if metrics["bpm"] else None,
        "max_abs_phase_err_beats": metrics["max_abs_phase_err_beats"],
        "eff_speed_spread": metrics["eff_speed"]["spread"] if metrics["eff_speed"] else None,
        "acceptance": {term["term"]: term["result"] for term in acceptance["terms"]},
        "overall_verdict": acceptance["overall"],
        "phrase_boundaries": {
            "count": boundaries["boundary_count"],
            "expected_seconds": boundaries["expected_seconds"],
            "mean_seconds": boundaries["spacing_stats"]["seconds"]["mean"] if boundaries["spacing_stats"]["seconds"] else None,
            "max_abs_seconds_error": boundaries["spacing_stats"]["max_abs_seconds_error"],
        },
        "phrase_boundary_anomaly_found": phrase_anomaly,
        "anomaly_counts": anomaly_counts,
        "self_test_passed": result["self_test"]["passed"],
        "self_test_report": result["self_test"]["report"],
        "completeness": {
            "criterion": COMPLETION_CRITERION,
            "self_detect_complete": result["completion"]["self_detect_complete"],
            "promoted_to_final": result["completion"]["promoted_to_final"],
            "witness_closed": result["completion"]["witness_closed"],
        },
        "esp_sampling": {
            "file": ESP_STATUS_FILE,
            "eta_baseline_end_utc": result["esp_sampling"]["eta_baseline_end_utc"],
            "eta_extension_end_utc": result["esp_sampling"]["eta_extension_end_utc"],
            "baseline_samples": baseline_esp.get("sample_count"),
            "baseline_replies": baseline_esp.get("reply_count"),
            "baseline_no_reply": baseline_esp.get("no_reply_count"),
            "control_file": result["esp_sampling"]["control_window"]["status_file"],
            "control_sampler_live": result["esp_sampling"]["control_window"]["pid_alive"],
        },
        "control_window": {
            "status": result["control_window"]["status"],
            "report": result["control_report"],
            "telemetry": result["control_window"]["telemetry_path"],
            "perturbation_scoring": result["control_window"]["perturbation_scoring"],
            "first_ltmp_marker": result["control_window"]["first_ltmp_marker"],
            "perturbations_sent": result["control_window"]["control_window_summary"]["perturbations_sent"],
            "end_utc": result["control_window"]["control_window_summary"]["end_utc"],
        },
    }


def write_full_report(path, result):
    analysis = result["analysis"]
    metrics = analysis["metrics"]
    lines = []
    add = lines.append
    add("PHRASE TELEMETRY ANALYSIS REPORT")
    add("STATUS: " + result["status"])
    add("WITNESS: " + ("CLOSED" if result["completion"]["witness_closed"] else "OPEN"))
    add("GENERATED_UTC: " + result["generated_utc"])
    add("INPUT: " + result["input"])
    add("REPORT: " + result["report"])
    add("ACCEPTANCE_SOURCE: " + ACCEPTANCE_SOURCE)
    add("")
    add("INTERIM NUMBERS" if result["status"] == "INTERIM" else "NUMBERS")
    add(json.dumps(result["compact"], indent=2, sort_keys=True))
    add("")
    add("COMPLETENESS")
    add(json.dumps(result["completion"], indent=2, sort_keys=True))
    add("")
    add("CONSTANTS AND THRESHOLDS")
    add(json.dumps({
        "constants": result["constants"],
        "acceptance": ACCEPTANCE,
        "boundary_thresholds": analysis["boundaries"]["thresholds"],
        "gap_burst_threshold": analysis["gaps"]["burst_threshold"],
    }, indent=2, sort_keys=True))
    add("")
    add("INPUT SHAPE AND PARSER RULES")
    add(json.dumps(result["input_context"], indent=2, sort_keys=True))
    add("")
    add("ACCEPTANCE AGAINST AGENTS.md")
    for term in analysis["acceptance"]["terms"]:
        add(term["result"] + " " + term["term"] + ": value=" + str(term["value"]) + " threshold=" + term["threshold"] + " detail=" + term["detail"])
    add("OVERALL VERDICT: " + analysis["acceptance"]["overall"])
    add("")
    add("NO REPLY VERSUS PEERS=0")
    add("no_reply_count=" + str(metrics["no_reply_count"]))
    add("peers0_count=" + str(metrics["peers0_count"]))
    add("peers_histogram=" + json.dumps(metrics["peers_histogram"], sort_keys=True))
    add("post60=" + json.dumps(metrics["post60"], sort_keys=True))
    add("")
    add("GAPS")
    add(json.dumps(analysis["gaps"], indent=2, sort_keys=True))
    add("")
    add("MONOTONICITY")
    add(json.dumps(analysis["monotonic"], indent=2, sort_keys=True))
    add("")
    add("DISTRIBUTIONS")
    add(json.dumps({
        "bpm": metrics["bpm"],
        "eff_speed": metrics["eff_speed"],
        "max_abs_phase_err_beats": metrics["max_abs_phase_err_beats"],
        "xruns": metrics["xruns"],
        "core_busy": metrics["core_busy"],
        "jitter": metrics["jitter"],
    }, indent=2, sort_keys=True))
    add("NOTE: beat_mark.late_ms is a sawtooth position indicator (0.3..500 ms, mean about 250 ms), NOT lateness; it is not treated as a timing error.")
    add("")
    add("PHRASE BOUNDARIES")
    add("beat_mark is NOT a boundary flag; boundaries are inferred from absolute beat crossing PHRASE_BEATS multiples and validated with grid_beat_index and master_phase_beats.")
    add("expected_seconds=" + str(analysis["boundaries"]["expected_seconds"]))
    add("expected_samples=" + str(analysis["boundaries"]["expected_samples"]))
    add("nominal_bpm=" + str(analysis["boundaries"]["nominal_bpm"]))
    add("sample_period_seconds=" + str(analysis["boundaries"]["sample_period_seconds"]))
    add("boundaries=" + json.dumps(analysis["boundaries"]["boundaries"][:80], indent=2, sort_keys=True))
    add("spacings=" + json.dumps(analysis["boundaries"]["spacings"][:80], indent=2, sort_keys=True))
    add("spacing_stats=" + json.dumps(analysis["boundaries"]["spacing_stats"], indent=2, sort_keys=True))
    add("resets=" + json.dumps(analysis["boundaries"]["resets"][:100], indent=2, sort_keys=True))
    add("grid_mismatches=" + json.dumps(analysis["boundaries"]["grid_mismatches"][:100], indent=2, sort_keys=True))
    add("phase_jumps=" + json.dumps(analysis["boundaries"]["phase_jumps"][:100], indent=2, sort_keys=True))
    add("")
    add("ANOMALIES")
    add("counts=" + json.dumps(analysis["anomaly_counts"], sort_keys=True))
    add(json.dumps(analysis["anomalies"][:200], indent=2, sort_keys=True))
    add("")
    add("SYNTHETIC SELF-TEST")
    add("RESULT: " + ("PASS" if result["self_test"]["passed"] else "FAIL"))
    add(json.dumps(result["self_test"], indent=2, sort_keys=True))
    add("")
    add("LIVE ESP STATUS SAMPLING")
    add(json.dumps(result["esp_sampling"], indent=2, sort_keys=True))
    add(SUBSCRIBER_NOTE)
    add("")
    add("CONTROL WINDOW")
    add(json.dumps({
        "status": result["control_window"]["status"],
        "telemetry": result["control_window"]["telemetry_path"],
        "esp": result["control_window"]["esp_path"],
        "report": result["control_report"],
        "markers": result["control_window"]["marker_order"],
        "perturbation_scoring": result["control_window"]["perturbation_scoring"],
        "first_ltmp_marker": result["control_window"]["first_ltmp_marker"],
        "pre_ltmp_baseline_status": result["control_window"]["pre_ltmp_baseline"]["status"],
        "control_window_summary": result["control_window"]["control_window_summary"],
        "phase_scores_empty": len(result["control_window"]["phases"]) == 0,
        "spp_context": result["control_window"]["spp_context"],
        "tempo_only_window": result["control_window"]["tempo_only_window"],
        "lrst": result["control_window"]["lrst"],
        "final_bpm_restored": result["control_window"]["final_bpm_restored"],
    }, indent=2, sort_keys=True))
    add("")
    add("LIMITATIONS")
    add(result["limitations"])
    add("")
    add("WITNESS CLOSURE")
    add(json.dumps(result["witness_closure"], indent=2, sort_keys=True))
    add("")
    add("END OF REPORT")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def load_esp_status_rows(path):
    rows = []
    events = []
    parse_errors = []
    if not path or not os.path.exists(path):
        return rows, events, parse_errors
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line_no, line in enumerate(handle, 1):
            stripped = line.strip()
            if not stripped:
                continue
            try:
                obj = json.loads(stripped)
            except (ValueError, TypeError) as exc:
                parse_errors.append({"line": line_no, "error": repr(exc)})
                continue
            if not isinstance(obj, dict):
                parse_errors.append({"line": line_no, "error": "top level is not an object"})
                continue
            if "event" in obj:
                events.append(obj)
                continue
            obj["epoch"] = parse_timestamp(obj.get("timestamp"))
            rows.append(obj)
    return rows, events, parse_errors


def analyze_esp_status_file(path):
    if not path or not os.path.exists(path):
        return {"path": path, "exists": False}
    rows, events, parse_errors = load_esp_status_rows(path)
    replies = [row for row in rows if row.get("ok") and isinstance(row.get("reply"), dict)]
    no_reply = [row for row in rows if not row.get("ok")]
    def series(*keys):
        values = []
        for row in replies:
            value = nested_number(row["reply"], *keys)
            if value is not None:
                values.append(value)
        return values
    beat_values = series("beat")
    phase_values = series("phase")
    bpm_values = series("bpm")
    quantum_values = series("quantum")
    peers_values = series("peers")
    metro_late_last = series("metro", "late_last_us")
    metro_late_worst = series("metro", "late_worst_us")
    metro_late_mean = series("metro", "late_mean_us")
    metro_late_rms = series("metro", "late_rms_us")
    metro_fired = series("metro", "fired")
    phase_residuals = []
    for row in replies:
        beat = nested_number(row["reply"], "beat")
        phase = nested_number(row["reply"], "phase")
        quantum = nested_number(row["reply"], "quantum")
        if None in (beat, phase, quantum) or quantum <= 0:
            continue
        expected = beat % quantum
        signed = (phase - expected + quantum / 2.0) % quantum - quantum / 2.0
        phase_residuals.append(abs(signed))
    beat_backwards = []
    beat_delta_errors = []
    for previous, current in zip(replies, replies[1:]):
        beat_previous = nested_number(previous["reply"], "beat")
        beat_current = nested_number(current["reply"], "beat")
        epoch_previous = previous.get("epoch")
        epoch_current = current.get("epoch")
        bpm = nested_number(current["reply"], "bpm")
        if None in (beat_previous, beat_current):
            continue
        delta = beat_current - beat_previous
        if delta < -1e-6:
            beat_backwards.append({"from_i": previous.get("i"), "to_i": current.get("i"), "delta": round(delta, 6)})
        if epoch_previous is not None and epoch_current is not None and epoch_current > epoch_previous and bpm is not None and bpm > 0:
            expected = (epoch_current - epoch_previous) * bpm / 60.0
            beat_delta_errors.append(abs(delta - expected))
    bpm_transitions = []
    previous_bpm = None
    for row in replies:
        bpm = nested_number(row["reply"], "bpm")
        if bpm is not None and previous_bpm is not None and bpm != previous_bpm:
            bpm_transitions.append({"i": row.get("i"), "timestamp": row.get("timestamp"), "from": previous_bpm, "to": bpm})
        if bpm is not None:
            previous_bpm = bpm
    metro_spikes = []
    for row in replies:
        late_last = nested_number(row["reply"], "metro", "late_last_us")
        if late_last is not None and late_last > args_metro_spike_threshold_us():
            metro_spikes.append({"i": row.get("i"), "timestamp": row.get("timestamp"), "late_last_us": late_last})
    return {
        "path": path,
        "exists": True,
        "line_count": len(rows) + len(events),
        "sample_count": len(rows),
        "event_count": len(events),
        "events": events,
        "reply_count": len(replies),
        "no_reply_count": len(no_reply),
        "no_reply_details": no_reply[:20],
        "parse_error_count": len(parse_errors),
        "first_timestamp": rows[0].get("timestamp") if rows else None,
        "last_timestamp": rows[-1].get("timestamp") if rows else None,
        "first_elapsed_s": rows[0].get("elapsed_s") if rows else None,
        "last_elapsed_s": rows[-1].get("elapsed_s") if rows else None,
        "beat": numeric_stats(beat_values),
        "phase": numeric_stats(phase_values),
        "bpm": numeric_stats(bpm_values),
        "quantum": numeric_stats(quantum_values),
        "peers": numeric_stats(peers_values),
        "metro_late_last_us": numeric_stats(metro_late_last),
        "metro_late_worst_us": numeric_stats(metro_late_worst),
        "metro_late_mean_us": numeric_stats(metro_late_mean),
        "metro_late_rms_us": numeric_stats(metro_late_rms),
        "metro_fired": numeric_stats(metro_fired),
        "metro_fired_delta": round(metro_fired[-1] - metro_fired[0], 6) if metro_fired else None,
        "phase_residual_abs_beats": numeric_stats(phase_residuals),
        "beat_backwards": beat_backwards,
        "beat_delta_error_abs_beats": numeric_stats(beat_delta_errors),
        "bpm_transitions": bpm_transitions,
        "metro_spikes": metro_spikes,
        "host_key_sha256": HOST_KEY_SHA256,
        "host_key_verified_out_of_band": True,
        "known_hosts_modified": False,
        "subscriber_note": SUBSCRIBER_NOTE,
    }


def args_metro_spike_threshold_us():
    return 1000.0


def load_control_markers(path):
    markers = []
    if not path or not os.path.exists(path):
        return markers
    with open(path, "r", encoding="utf-8", errors="replace") as handle:
        for line in handle:
            stripped = line.strip()
            if not stripped:
                continue
            try:
                obj = json.loads(stripped)
            except (ValueError, TypeError):
                continue
            if isinstance(obj, dict) and "marker" in obj:
                markers.append({
                    "name": str(obj.get("marker")),
                    "t": obj.get("t"),
                    "epoch": parse_timestamp(obj.get("t")),
                    "raw": obj,
                })
    return markers


def nearest_reply_value(rows, epoch, keys):
    if epoch is None:
        return None, None
    best_value = None
    best_distance = None
    for row in rows:
        row_epoch = row.get("epoch")
        if row_epoch is None or not isinstance(row.get("reply"), dict):
            continue
        value = nested_number(row["reply"], *keys)
        if value is None:
            continue
        distance = abs(row_epoch - epoch)
        if best_distance is None or distance < best_distance:
            best_distance = distance
            best_value = value
    return best_value, best_distance


def marker_beat_snapshot(marker, esp_rows, telemetry_rows):
    esp_beat, esp_distance = nearest_reply_value(esp_rows, marker["epoch"], ("beat",))
    telemetry_beat, telemetry_distance = nearest_reply_value(telemetry_rows, marker["epoch"], ("beat_mark", "beat"))
    esp_bpm, _ = nearest_reply_value(esp_rows, marker["epoch"], ("bpm",))
    telemetry_bpm, _ = nearest_reply_value(telemetry_rows, marker["epoch"], ("link", "bpm"))
    esp_phase, _ = nearest_reply_value(esp_rows, marker["epoch"], ("phase",))
    telemetry_phase_err, _ = nearest_reply_value(telemetry_rows, marker["epoch"], ("link", "phase_err_beats"))
    return {
        "marker": marker["name"],
        "t": marker["t"],
        "epoch": marker["epoch"],
        "esp_beat": esp_beat,
        "esp_beat_sample_distance_s": round(esp_distance, 6) if esp_distance is not None else None,
        "telemetry_beat_mark_beat": telemetry_beat,
        "telemetry_beat_sample_distance_s": round(telemetry_distance, 6) if telemetry_distance is not None else None,
        "esp_bpm": esp_bpm,
        "telemetry_bpm": telemetry_bpm,
        "esp_phase": esp_phase,
        "telemetry_phase_err_beats": telemetry_phase_err,
    }


def control_phase_ranges(markers, last_epoch):
    ranges = []
    for index, marker in enumerate(markers):
        start = marker["epoch"] if marker["epoch"] is not None else 0.0
        end = last_epoch if last_epoch is not None else start + 60.0
        if index + 1 < len(markers) and markers[index + 1]["epoch"] is not None:
            end = markers[index + 1]["epoch"]
        ranges.append({
            "name": marker["name"],
            "start_epoch": start,
            "end_epoch": end,
            "start_t": marker["t"],
            "end_t": markers[index + 1]["t"] if index + 1 < len(markers) else None,
        })
    return ranges


def score_control_phase(phase, esp_rows, telemetry_rows, spike_threshold):
    def in_phase(row):
        epoch = row.get("epoch")
        return epoch is not None and phase["start_epoch"] <= epoch < phase["end_epoch"]
    esp_in = [row for row in esp_rows if in_phase(row)]
    telemetry_in = [row for row in telemetry_rows if in_phase(row)]
    esp_replies = [row for row in esp_in if row.get("ok") and isinstance(row.get("reply"), dict)]
    telemetry_replies = [row for row in telemetry_in if not row.get("gap") and isinstance(row.get("reply"), dict)]
    telemetry_gaps = [row for row in telemetry_in if row.get("gap")]
    def esp_series(*keys):
        return [nested_number(row["reply"], *keys) for row in esp_replies if nested_number(row["reply"], *keys) is not None]
    def telemetry_series(*keys):
        return [nested_number(row["reply"], *keys) for row in telemetry_replies if nested_number(row["reply"], *keys) is not None]
    esp_beat = esp_series("beat")
    esp_phase_residuals = []
    for row in esp_replies:
        beat = nested_number(row["reply"], "beat")
        phase_value = nested_number(row["reply"], "phase")
        quantum = nested_number(row["reply"], "quantum")
        if None in (beat, phase_value, quantum) or quantum <= 0:
            continue
        expected = beat % quantum
        signed = (phase_value - expected + quantum / 2.0) % quantum - quantum / 2.0
        esp_phase_residuals.append(abs(signed))
    esp_beat_backwards = []
    esp_beat_delta_errors = []
    for previous, current in zip(esp_replies, esp_replies[1:]):
        beat_previous = nested_number(previous["reply"], "beat")
        beat_current = nested_number(current["reply"], "beat")
        epoch_previous = previous.get("epoch")
        epoch_current = current.get("epoch")
        bpm = nested_number(current["reply"], "bpm")
        if None in (beat_previous, beat_current):
            continue
        delta = beat_current - beat_previous
        if delta < -1e-6:
            esp_beat_backwards.append({"from_i": previous.get("i"), "to_i": current.get("i"), "delta": round(delta, 6)})
        if epoch_previous is not None and epoch_current is not None and epoch_current > epoch_previous and bpm is not None and bpm > 0:
            expected = (epoch_current - epoch_previous) * bpm / 60.0
            esp_beat_delta_errors.append(abs(delta - expected))
    telemetry_beat_backwards = []
    previous_beat = None
    previous_row = None
    for row in telemetry_replies:
        beat = nested_number(row["reply"], "beat_mark", "beat")
        if beat is None:
            continue
        if previous_beat is not None and beat < previous_beat - 1e-6:
            telemetry_beat_backwards.append({"from_i": previous_row["i"], "to_i": row["i"], "delta": round(beat - previous_beat, 6)})
        previous_beat = beat
        previous_row = row
    phase_errs = [abs(value) for value in telemetry_series("link", "phase_err_beats")]
    peers = telemetry_series("link", "peers")
    peers0_count = sum(1 for value in peers if value == 0)
    peers_transitions = []
    previous_peers = None
    for row in telemetry_replies:
        value = nested_number(row["reply"], "link", "peers")
        if value is None:
            continue
        if previous_peers is not None and value != previous_peers:
            peers_transitions.append({"i": row["i"], "from": previous_peers, "to": value})
        previous_peers = value
    metro_late_last = esp_series("metro", "late_last_us")
    metro_late_worst = esp_series("metro", "late_worst_us")
    metro_fired = esp_series("metro", "fired")
    metro_spikes = [
        {"i": row["i"], "timestamp": row.get("timestamp"), "late_last_us": value}
        for row, value in zip(esp_replies, metro_late_last)
        if value > spike_threshold
    ]
    max_phase_residual = max(esp_phase_residuals) if esp_phase_residuals else None
    max_phase_err = max(phase_errs) if phase_errs else None
    continuity_pass = (
        len(esp_beat_backwards) == 0
        and len(telemetry_beat_backwards) == 0
        and (max_phase_residual is None or max_phase_residual <= 1.0)
        and (max_phase_err is None or max_phase_err <= 0.03)
        and len(telemetry_gaps) == 0
    )
    return {
        "name": phase["name"],
        "start_t": phase["start_t"],
        "end_t": phase["end_t"],
        "start_epoch": phase["start_epoch"],
        "end_epoch": phase["end_epoch"],
        "esp_reply_count": len(esp_replies),
        "esp_no_reply_count": sum(1 for row in esp_in if not row.get("ok")),
        "telemetry_reply_count": len(telemetry_replies),
        "telemetry_gap_count": len(telemetry_gaps),
        "bpm": numeric_stats(esp_series("bpm")),
        "phase": numeric_stats(esp_series("phase")),
        "beat": numeric_stats(esp_beat),
        "phase_residual_abs_beats": numeric_stats(esp_phase_residuals),
        "beat_delta_error_abs_beats": numeric_stats(esp_beat_delta_errors),
        "esp_beat_backwards": esp_beat_backwards,
        "telemetry_beat_backwards": telemetry_beat_backwards,
        "telemetry_bpm": numeric_stats(telemetry_series("link", "bpm")),
        "telemetry_phase_err_abs": numeric_stats(phase_errs),
        "telemetry_eff_speed": numeric_stats(telemetry_series("eff_speed")),
        "telemetry_peers0_count": peers0_count,
        "telemetry_peers_transitions": peers_transitions,
        "metro_late_last_us": numeric_stats(metro_late_last),
        "metro_late_worst_us": numeric_stats(metro_late_worst),
        "metro_fired_delta": round(metro_fired[-1] - metro_fired[0], 6) if metro_fired else None,
        "metro_spike_count": len(metro_spikes),
        "metro_spikes": metro_spikes[:20],
        "beat_phase_continuity": "PASS" if continuity_pass else "FAIL",
    }


def tempo_only_window(markers, esp_rows, telemetry_rows, spike_threshold, window_seconds=60.0):
    candidates = [
        marker for marker in markers
        if "tempo" in marker["name"].lower() and "phase" not in marker["name"].lower()
    ]
    if not candidates:
        return {"status": "pending", "reason": "no tempo-only LTMP marker yet", "window_seconds": window_seconds}
    marker = candidates[0]
    center = marker["epoch"]
    if center is None:
        return {"status": "pending", "reason": "tempo-only marker has no parseable timestamp"}
    start = center - window_seconds / 2.0
    end = center + window_seconds / 2.0
    window_rows = [row for row in esp_rows if row.get("epoch") is not None and start <= row["epoch"] <= end]
    replies = [row for row in window_rows if row.get("ok") and isinstance(row.get("reply"), dict)]
    metro = []
    for row in replies:
        late_last = nested_number(row["reply"], "metro", "late_last_us")
        late_worst = nested_number(row["reply"], "metro", "late_worst_us")
        fired = nested_number(row["reply"], "metro", "fired")
        if None not in (late_last, late_worst, fired):
            metro.append({"i": row.get("i"), "timestamp": row.get("timestamp"), "late_last_us": late_last, "late_worst_us": late_worst, "fired": fired})
    if not metro:
        return {"status": "insufficient_data", "reason": "no ESP metro replies in window", "marker": marker["name"], "window_seconds": window_seconds}
    first = metro[0]
    last = metro[-1]
    late_last_values = [item["late_last_us"] for item in metro]
    late_worst_values = [item["late_worst_us"] for item in metro]
    telemetry_window = [row for row in telemetry_rows if row.get("epoch") is not None and start <= row["epoch"] <= end and not row.get("gap")]
    telemetry_bpm = [nested_number(row["reply"], "link", "bpm") for row in telemetry_window if nested_number(row["reply"], "link", "bpm") is not None]
    telemetry_phase_err = [abs(nested_number(row["reply"], "link", "phase_err_beats")) for row in telemetry_window if nested_number(row["reply"], "link", "phase_err_beats") is not None]
    telemetry_eff = [nested_number(row["reply"], "eff_speed") for row in telemetry_window if nested_number(row["reply"], "eff_speed") is not None]
    rearm_candidate = max(late_last_values) > spike_threshold or last["late_worst_us"] > first["late_worst_us"]
    return {
        "status": "complete",
        "marker": marker["name"],
        "marker_t": marker["t"],
        "window_seconds": window_seconds,
        "window_start_epoch": start,
        "window_end_epoch": end,
        "esp_reply_count": len(replies),
        "metro_first": first,
        "metro_last": last,
        "late_last_us_delta": round(last["late_last_us"] - first["late_last_us"], 6),
        "late_worst_us_delta": round(last["late_worst_us"] - first["late_worst_us"], 6),
        "fired_delta": round(last["fired"] - first["fired"], 6),
        "late_last_us_max": max(late_last_values),
        "late_worst_us_max": max(late_worst_values),
        "telemetry_bpm": numeric_stats(telemetry_bpm),
        "telemetry_phase_err_abs": numeric_stats(telemetry_phase_err),
        "telemetry_eff_speed": numeric_stats(telemetry_eff),
        "rearm_defect_candidate": rearm_candidate,
    }


def lrst_peers(markers, telemetry_rows, esp_rows, window_seconds=120.0):
    candidates = [marker for marker in markers if "lrst" in marker["name"].lower() or "reset" in marker["name"].lower()]
    if not candidates:
        return {"status": "pending", "reason": "no LRST marker yet", "window_seconds": window_seconds}
    marker = candidates[0]
    center = marker["epoch"]
    if center is None:
        return {"status": "pending", "reason": "LRST marker has no parseable timestamp"}
    start = center - window_seconds / 2.0
    end = center + window_seconds / 2.0
    def transitions(rows, keys):
        series = []
        for row in rows:
            epoch = row.get("epoch")
            if epoch is None or not (start <= epoch <= end):
                continue
            reply = row.get("reply")
            if not isinstance(reply, dict):
                continue
            value = nested_number(reply, *keys)
            if value is not None:
                series.append((epoch, value, row.get("i")))
        drops = []
        returns = []
        for previous, current in zip(series, series[1:]):
            if previous[1] >= 1 and current[1] == 0:
                drops.append({"t_epoch": current[0], "i": current[2], "from": previous[1], "to": current[1]})
            if previous[1] == 0 and current[1] >= 1:
                returns.append({"t_epoch": current[0], "i": current[2], "from": previous[1], "to": current[1]})
        return {"drops": drops, "returns": returns, "series_count": len(series)}
    telemetry_result = transitions(telemetry_rows, ("link", "peers"))
    esp_result = transitions(esp_rows, ("peers",))
    observed = bool(telemetry_result["drops"] and telemetry_result["returns"])
    return {
        "status": "complete" if observed else "in_progress",
        "marker": marker["name"],
        "marker_t": marker["t"],
        "window_seconds": window_seconds,
        "telemetry": telemetry_result,
        "esp": esp_result,
        "peers_drop_observed": bool(telemetry_result["drops"] or esp_result["drops"]),
        "peers_return_observed": bool(telemetry_result["returns"] or esp_result["returns"]),
        "outage_seconds": round(telemetry_result["returns"][0]["t_epoch"] - telemetry_result["drops"][0]["t_epoch"], 6) if telemetry_result["drops"] and telemetry_result["returns"] else None,
    }


def final_bpm_restored(markers, telemetry_rows, esp_rows):
    end_markers = [marker for marker in markers if "end" in marker["name"].lower() or "restore" in marker["name"].lower()]
    last_telemetry_bpm = None
    for row in reversed(telemetry_rows):
        if not row.get("gap") and isinstance(row.get("reply"), dict):
            last_telemetry_bpm = nested_number(row["reply"], "link", "bpm")
            if last_telemetry_bpm is not None:
                break
    last_esp_bpm = None
    for row in reversed(esp_rows):
        if row.get("ok") and isinstance(row.get("reply"), dict):
            last_esp_bpm = nested_number(row["reply"], "bpm")
            if last_esp_bpm is not None:
                break
    restored = last_telemetry_bpm == 120.0 and last_esp_bpm == 120.0
    return {
        "status": "complete" if end_markers else "pending",
        "reason": "waiting for end or restore marker" if not end_markers else "end marker present",
        "last_telemetry_bpm": last_telemetry_bpm,
        "last_esp_bpm": last_esp_bpm,
        "target_bpm": 120.0,
        "result": "PASS" if restored else "FAIL",
    }


def spp_context(markers, esp_rows, snapshots):
    threshold = 16383.75
    start_esp_beat = snapshots[0]["esp_beat"] if snapshots else None
    sampler_first = next((row for row in esp_rows if row.get("ok") and isinstance(row.get("reply"), dict)), None)
    sampler_start_beat = nested_number(sampler_first["reply"], "beat") if sampler_first else None
    sampler_start_t = sampler_first.get("timestamp") if sampler_first else None
    order = [marker["name"] for marker in markers]
    tempo_phase = [marker["name"] for marker in markers if "tempo" in marker["name"].lower() and "phase" in marker["name"].lower()]
    tempo_only = [marker["name"] for marker in markers if "tempo" in marker["name"].lower() and "phase" not in marker["name"].lower()]
    beat_reset_observed = any(
        snapshot["esp_beat"] is not None and snapshot["esp_beat"] < 1000.0
        for snapshot in snapshots[1:]
    )
    return {
        "spp_cast_threshold_beats": threshold,
        "esp_beat_at_control_start": start_esp_beat,
        "esp_beat_at_sampler_start": sampler_start_beat,
        "esp_sampler_start_t": sampler_start_t,
        "above_old_firmware_spp_threshold": start_esp_beat is not None and start_esp_beat > threshold,
        "note": "A tempo+phase LTMP resets beat to 0 via forceBeatAtTime; a tempo-only LTMP does not. Record which order 420 uses and the beat value at each marker.",
        "marker_order": order,
        "tempo_phase_markers": tempo_phase,
        "tempo_only_markers": tempo_only,
        "beat_at_markers": snapshots,
        "beat_reset_observed_after_a_marker": beat_reset_observed,
    }


def control_baseline_summary(esp_rows, telemetry_rows, first_ltmp_epoch):
    def before_first_ltmp(row):
        epoch = row.get("epoch")
        return epoch is not None and (first_ltmp_epoch is None or epoch < first_ltmp_epoch)
    esp_replies = [row for row in esp_rows if before_first_ltmp(row) and row.get("ok") and isinstance(row.get("reply"), dict)]
    telemetry_replies = [row for row in telemetry_rows if before_first_ltmp(row) and not row.get("gap") and isinstance(row.get("reply"), dict)]
    def series(rows, *keys):
        return [nested_number(row["reply"], *keys) for row in rows if nested_number(row["reply"], *keys) is not None]
    return {
        "status": "unscored_pre_ltmp_baseline",
        "first_ltmp_epoch": first_ltmp_epoch,
        "esp_reply_count": len(esp_replies),
        "telemetry_reply_count": len(telemetry_replies),
        "esp_bpm": numeric_stats(series(esp_replies, "bpm")),
        "esp_beat": numeric_stats(series(esp_replies, "beat")),
        "esp_metro_late_last_us": numeric_stats(series(esp_replies, "metro", "late_last_us")),
        "esp_metro_late_worst_us": numeric_stats(series(esp_replies, "metro", "late_worst_us")),
        "esp_metro_fired": numeric_stats(series(esp_replies, "metro", "fired")),
        "telemetry_bpm": numeric_stats(series(telemetry_replies, "link", "bpm")),
        "telemetry_phase_err_abs": numeric_stats([abs(value) for value in series(telemetry_replies, "link", "phase_err_beats")]),
        "telemetry_eff_speed": numeric_stats(series(telemetry_replies, "eff_speed")),
        "note": "Everything before the first LTMP marker is baseline and is not scored as a perturbation. No event is assumed.",
    }


def analyze_control_window(args, constants):
    telemetry_path = args.control_telemetry
    esp_path = args.control_esp
    telemetry_exists = os.path.exists(telemetry_path)
    esp_exists = os.path.exists(esp_path)
    esp_rows, esp_events, esp_parse_errors = load_esp_status_rows(esp_path)
    esp_summary = analyze_esp_status_file(esp_path)
    markers = load_control_markers(telemetry_path)
    telemetry_rows = []
    telemetry_parse_errors = []
    telemetry_analysis = None
    if telemetry_exists:
        telemetry_rows, telemetry_parse_errors, start_epoch = load_rows(telemetry_path)
        telemetry_analysis = analyze(telemetry_rows, args, constants, telemetry_parse_errors)
    epochs = [row.get("epoch") for row in esp_rows if row.get("epoch") is not None]
    epochs.extend(row.get("epoch") for row in telemetry_rows if row.get("epoch") is not None)
    last_epoch = max(epochs) if epochs else None
    ranges = control_phase_ranges(markers, last_epoch)
    snapshots = [marker_beat_snapshot(marker, esp_rows, telemetry_rows) for marker in markers]
    ltmp_markers = [marker for marker in markers if "ltmp" in marker["name"].lower() or "tempo" in marker["name"].lower()]
    first_ltmp = ltmp_markers[0] if ltmp_markers else None
    first_ltmp_epoch = first_ltmp["epoch"] if first_ltmp else None
    pre_ltmp_baseline = control_baseline_summary(esp_rows, telemetry_rows, first_ltmp_epoch)
    if first_ltmp:
        scoring_ranges = [phase for phase in ranges if phase["start_epoch"] >= first_ltmp_epoch]
        phase_scores = [score_control_phase(phase, esp_rows, telemetry_rows, args.metro_spike_threshold_us) for phase in scoring_ranges]
    else:
        phase_scores = []
    tempo_window = tempo_only_window(markers, esp_rows, telemetry_rows, args.metro_spike_threshold_us)
    lrst = lrst_peers(markers, telemetry_rows, esp_rows)
    restored = final_bpm_restored(markers, telemetry_rows, esp_rows)
    spp = spp_context(markers, esp_rows, snapshots)
    end_markers = [marker for marker in markers if "end" in marker["name"].lower() or "restore" in marker["name"].lower()]
    lrst_markers = [marker for marker in markers if "lrst" in marker["name"].lower() or "reset" in marker["name"].lower()]
    no_perturbations = bool(end_markers and not ltmp_markers and not lrst_markers)
    start_marker = markers[0] if markers else None
    end_marker = end_markers[0] if end_markers else None
    if no_perturbations:
        status = NO_PERTURBATIONS_SENT
        perturbation_scoring = "no_perturbations_sent"
        tempo_window = {
            "status": "not_applicable_no_perturbations_sent",
            "reason": "no LTMP was sent",
            "window_seconds": 60.0,
        }
        lrst = {
            "status": "not_applicable_no_perturbations_sent",
            "reason": "no LRST was sent",
            "peers_drop_observed": False,
            "peers_return_observed": False,
        }
        restored = {
            "status": "not_applicable_no_perturbations_sent",
            "restore_needed": False,
            "last_telemetry_bpm": restored.get("last_telemetry_bpm"),
            "last_esp_bpm": restored.get("last_esp_bpm"),
            "target_bpm": 120.0,
            "result": "PASS" if restored.get("last_telemetry_bpm") == 120.0 and restored.get("last_esp_bpm") == 120.0 else "FAIL",
        }
    elif not telemetry_exists:
        status = "pending"
        perturbation_scoring = "pending_telemetry"
    elif end_markers:
        status = "complete"
        perturbation_scoring = "active" if first_ltmp else "complete_without_ltmp_marker"
    elif first_ltmp:
        status = "in_progress"
        perturbation_scoring = "active"
    else:
        status = "pending_perturbations"
        perturbation_scoring = "pending_first_ltmp_marker"
    control_window_summary = {
        "status": status,
        "start_marker": start_marker["name"] if start_marker else None,
        "start_utc": start_marker["t"] if start_marker else None,
        "end_marker": end_marker["name"] if end_marker else None,
        "end_utc": end_marker["t"] if end_marker else None,
        "end_note": end_marker["raw"].get("note") if end_marker else None,
        "ltmp_count": len(ltmp_markers),
        "lrst_count": len(lrst_markers),
        "perturbations_sent": 0 if no_perturbations else len(ltmp_markers) + len(lrst_markers),
        "telemetry_sample_count": len(telemetry_rows),
        "telemetry_marker_count": len(markers),
        "esp_sample_count": esp_summary.get("sample_count"),
        "esp_reply_count": esp_summary.get("reply_count"),
        "esp_no_reply_count": esp_summary.get("no_reply_count"),
        "note": "420 stood down the control window; no LTMP and no LRST were sent; the clean 30-min 1 Hz baseline is the live result." if no_perturbations else "Control window status is not final.",
    }
    control_meta = load_json_if_exists(CONTROL_ESP_META)
    pid = None
    pid_alive = False
    try:
        pid = open(CONTROL_ESP_PID, "r", encoding="utf-8").read().strip()
        pid_alive = os.path.exists("/proc/" + pid)
    except OSError:
        pass
    return {
        "status": status,
        "telemetry_path": telemetry_path,
        "esp_path": esp_path,
        "telemetry_exists": telemetry_exists,
        "esp_exists": esp_exists,
        "control_start_utc": markers[0]["t"] if markers else None,
        "markers": markers,
        "marker_order": [marker["name"] for marker in markers],
        "esp_sampler": {
            "meta": control_meta,
            "pid": pid,
            "pid_alive": pid_alive,
            "no_fixed_end_before_utc": "2026-11-05T21:15:00Z",
            "stop_marker": ".gm/witness-out/esp-status-control-20261105.stop",
        },
        "esp_summary": esp_summary,
        "telemetry_summary": {
            "exists": telemetry_exists,
            "analysis": telemetry_analysis,
            "parse_error_count": len(telemetry_parse_errors),
        },
        "spp_context": spp,
        "beat_at_markers": snapshots,
        "phases": phase_scores,
        "pre_ltmp_baseline": pre_ltmp_baseline,
        "perturbation_scoring": perturbation_scoring,
        "first_ltmp_marker": first_ltmp["name"] if first_ltmp else None,
        "control_window_summary": control_window_summary,
        "tempo_only_window": tempo_window,
        "lrst": lrst,
        "final_bpm_restored": restored,
        "limitations": args.limitations,
    }


def write_control_report(path, control_window, result):
    lines = []
    add = lines.append
    add("CONTROL WINDOW REPORT")
    add("STATUS: " + control_window["status"])
    add("GENERATED_UTC: " + result["generated_utc"])
    add("TELEMETRY: " + control_window["telemetry_path"])
    add("ESP STATUS: " + control_window["esp_path"])
    add("CONTROL START: " + str(control_window["control_start_utc"]))
    add("CONTROL END: " + str(control_window["control_window_summary"]["end_utc"]))
    add("PERTURBATIONS SENT: " + str(control_window["control_window_summary"]["perturbations_sent"]))
    add("MARKER ORDER: " + ", ".join(control_window["marker_order"]))
    add("")
    add("CONTROL WINDOW SUMMARY")
    add(json.dumps(control_window["control_window_summary"], indent=2, sort_keys=True))
    add("")
    add("ESP SAMPLER")
    add(json.dumps(control_window["esp_sampler"], indent=2, sort_keys=True))
    add("")
    add("ESP STATUS SUMMARY")
    add(json.dumps(control_window["esp_summary"], indent=2, sort_keys=True))
    add("")
    add("TELEMETRY SUMMARY")
    add(json.dumps(control_window["telemetry_summary"], indent=2, sort_keys=True))
    add("")
    add("SPP THRESHOLD AND BEAT AT MARKERS")
    add(json.dumps(control_window["spp_context"], indent=2, sort_keys=True))
    add("")
    add("PRE-LTMP BASELINE (UNSCORED)")
    add(json.dumps(control_window["pre_ltmp_baseline"], indent=2, sort_keys=True))
    add("")
    add("PERTURBATION SCORING")
    add(json.dumps({
        "status": control_window["perturbation_scoring"],
        "first_ltmp_marker": control_window["first_ltmp_marker"],
        "phase_count": len(control_window["phases"]),
        "rule": "Everything before the first LTMP marker is baseline and is not scored as a perturbation. No event is assumed.",
    }, indent=2, sort_keys=True))
    add("")
    add("PHASE SCORES")
    add(json.dumps(control_window["phases"], indent=2, sort_keys=True))
    add("")
    add("TEMPO-ONLY LTMP 60 S METRO WINDOW")
    add(json.dumps(control_window["tempo_only_window"], indent=2, sort_keys=True))
    add("")
    add("LRST PEERS")
    add(json.dumps(control_window["lrst"], indent=2, sort_keys=True))
    add("")
    add("FINAL BPM RESTORED")
    add(json.dumps(control_window["final_bpm_restored"], indent=2, sort_keys=True))
    add("")
    add("LIMITATIONS")
    add(control_window["limitations"])
    add("")
    add("END OF CONTROL WINDOW REPORT")
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text("\n".join(lines) + "\n", encoding="utf-8")


def close_witness(result):
    payload = {
        "status": "CLOSED",
        "closed_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
        "status_label": result["status"],
        "completion_signal": result["completion"]["completion_signal"],
        "completion_criterion": COMPLETION_CRITERION,
        "input": result["input"],
        "report": result["report"],
        "sample_count": result["analysis"]["metrics"]["total_samples"],
        "acceptance": result["analysis"]["acceptance"],
        "boundary_stats": result["analysis"]["boundaries"]["spacing_stats"],
        "esp_sampling": result["esp_sampling"],
        "control_window_status": result["control_window"]["status"],
        "control_report": CONTROL_REPORT,
        "limitations": result["limitations"],
        "self_test_passed": result["self_test"]["passed"],
    }
    Path(FINAL_WITNESS).parent.mkdir(parents=True, exist_ok=True)
    Path(FINAL_WITNESS).write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n", encoding="utf-8")


def build_parser():
    parser = argparse.ArgumentParser(description="Analyze 420 detached 1 Hz aloop telemetry for phrase predictability")
    parser.add_argument("--input", default=DEFAULT_INPUT)
    parser.add_argument("--report", default=DEFAULT_REPORT)
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--status", choices=("INTERIM", "FINAL"), default="INTERIM")
    parser.add_argument("--completion-signal", default="")
    parser.add_argument("--phase-jump-threshold", type=float, default=1.0)
    parser.add_argument("--boundary-alignment-tolerance", type=float, default=1.0)
    parser.add_argument("--spacing-tolerance-seconds", type=float, default=1.0)
    parser.add_argument("--gap-burst-threshold", type=int, default=3)
    parser.add_argument("--control-telemetry", default=CONTROL_TELEMETRY_DEFAULT)
    parser.add_argument("--control-esp", default=CONTROL_ESP_FILE)
    parser.add_argument("--control-markers", default=CONTROL_MARKERS_DEFAULT)
    parser.add_argument("--control-report", default=CONTROL_REPORT)
    parser.add_argument("--metro-spike-threshold-us", type=float, default=1000.0)
    parser.add_argument("--limitations", default=DEFAULT_LIMITATIONS)
    parser.add_argument("--close-witness", action="store_true")
    return parser


def main(argv=None):
    args = build_parser().parse_args(argv)
    constants = load_constants()
    if args.self_test:
        try:
            self_test = run_self_test(args, constants)
        except Exception as exc:
            print(json.dumps({"ok": False, "status": "SELF_TEST", "self_test": {"passed": False, "error": repr(exc), "path": str(getattr(exc, "filename", "") or "")}}, separators=(",", ":")))
            return 1
        print(json.dumps({"ok": True, "status": "SELF_TEST", "self_test": {key: self_test[key] for key in ("passed", "generated_rows", "required_flags", "assertions", "flag_counts", "report")}}, separators=(",", ":")))
        return 0
    if args.status == "FINAL" and not args.completion_signal.strip():
        print(json.dumps({"ok": False, "error": "FINAL requires --completion-signal from 420"}, separators=(",", ":")))
        return 2
    try:
        rows, parse_errors, start_epoch = load_rows(args.input)
        analysis = analyze(rows, args, constants, parse_errors)
        self_test = run_self_test(args, constants)
        control_window = analyze_control_window(args, constants)
        result = {
            "ok": True,
            "status": args.status,
            "generated_utc": dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds").replace("+00:00", "Z"),
            "input": args.input,
            "report": args.report,
            "control_report": args.control_report,
            "constants": constants,
            "analysis": analysis,
            "input_context": input_context(args.input, rows),
            "completion": completion_context(args.input, rows, args),
            "esp_sampling": esp_context(args),
            "control_window": control_window,
            "limitations": args.limitations,
            "witness_closure": load_json_if_exists(FINAL_WITNESS),
            "self_test": self_test,
        }
        result["compact"] = compact_summary(result)
        if args.close_witness and args.status == "FINAL":
            close_witness(result)
            result["witness_closure"] = load_json_if_exists(FINAL_WITNESS)
            result["completion"] = completion_context(args.input, rows, args)
            result["compact"] = compact_summary(result)
        write_full_report(args.report, result)
        write_control_report(args.control_report, control_window, result)
        print(json.dumps(result["compact"], separators=(",", ":")))
        return 0
    except Exception as exc:
        print(json.dumps({"ok": False, "error": repr(exc)}, separators=(",", ":")))
        return 1


if __name__ == "__main__":
    sys.exit(main())
