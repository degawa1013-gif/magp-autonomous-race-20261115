"""Deterministic summaries of sampled drive diagnostics; no vehicle/file I/O.

Perception mode is metadata.mode; the row's top-level mode and
metadata.vehicle_control.mode describe requested/applied driving mode instead.
No annotations/ground truth exist, so these summaries do not measure accuracy.
"""
from collections import Counter
import math
from pathlib import PurePosixPath


_PERCEPTION_MODES = ("TRACK", "DEGRADED", "LOST", "STOP", "INIT", "OTHER", "UNKNOWN")
_DRIVING_MODES = frozenset(("AUTO", "STEER_AUTO", "MANUAL", "STOP"))
_BAD_MODES = frozenset(("DEGRADED", "LOST", "STOP", "INIT"))
_TIMINGS = ("camera_to_output_ms", "processing_to_output_ms", "frame_interval_ms")


def _number(value):
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        try:
            value = float(value)
            return value if math.isfinite(value) else None
        except (OverflowError, ValueError):
            pass
    return None


def _distribution(values, total=None):
    values = sorted(values)
    result = {"count": len(values)}
    if total is not None:
        result["missing_or_invalid"] = total - len(values)
    if not values:
        return result
    def percentile(p):
        at = (len(values) - 1) * p
        low, high = math.floor(at), math.ceil(at)
        fraction = at - low
        return round(values[low] * (1. - fraction) + values[high] * fraction, 3)
    result.update(min=round(values[0], 3), p50=percentile(.5), p95=percentile(.95),
                  p99=percentile(.99), max=round(values[-1], 3))
    return result


def _dict(value):
    return value if isinstance(value, dict) else {}


def _corner_pace_summary(rows):
    """Summarize recorded policy decisions without inventing actuator motion."""
    present = active = limited = comparable = violations = 0
    phases, reasons = Counter(), Counter()
    names = ("requested_percent", "cap_percent", "applied_percent")
    outputs = {name: [] for name in names}
    active_outputs = {name: [] for name in names}
    gain_names = ("center_error_gain_active", "center_error_d_gain_active",
                  "speed_pd_blend", "curve_pd_blend")
    gains = {name: [] for name in gain_names}
    speed_pd = Counter()
    for row in rows:
        metadata = _dict(row.get("metadata"))
        flag = metadata.get("speed_pd_active")
        speed_pd["true" if flag is True else "false" if flag is False else "unknown"] += 1
        for name in gain_names:
            value = _number(metadata.get(name))
            if value is not None and value >= 0 and ("blend" not in name or value <= 1.):
                gains[name].append(value)
        policy = _dict(metadata.get("corner_pace"))
        if not policy:
            continue
        present += 1
        is_active = policy.get("active") is True
        active += is_active
        limited += is_active and policy.get("limited") is True
        for key, counts in (("phase", phases), ("reason", reasons)):
            value = policy.get(key)
            counts[value if isinstance(value, str) and 0 < len(value) <= 128 else "UNKNOWN"] += 1
        observed = {}
        for name in names:
            value = _number(policy.get(name))
            if value is not None and 0. <= value <= 100.:
                outputs[name].append(value)
                observed[name] = value
                if is_active:
                    active_outputs[name].append(value)
        if is_active and "cap_percent" in observed and "applied_percent" in observed:
            comparable += 1
            violations += observed["applied_percent"] > observed["cap_percent"] + .02
    return {
        "policy_present_samples": present,
        "policy_missing_samples": len(rows) - present,
        "active_samples": active,
        "phase_counts": dict(sorted(phases.items())),
        "reason_counts": dict(sorted(reasons.items())),
        "limited_samples": limited,
        "output_percent": {name: _distribution(outputs[name], present) for name in names},
        "active_output_percent": {name: _distribution(active_outputs[name], active) for name in names},
        "cap_comparable_active_samples": comparable,
        "cap_violation_samples": violations,
        "cap_violation_tolerance_output_points": .02,
        "gain_distributions": {name: _distribution(gains[name], len(rows)) for name in gain_names},
        "speed_pd_active_counts": {name: speed_pd[name] for name in ("true", "false", "unknown")},
        "limitations": {
            "software_commands_not_measured_steering_or_motor_response": True,
            "wheel_speed_is_uncalibrated": True,
            "lap_time_gain_not_established": True,
            "note": "上限違反はactive=Trueでcap/applied両値がある保存標本のソフト指令比較。欠損は不明。実舵角・減速距離・未保存tick・校正済み路面速度・ラップ短縮を検証しない。",
        },
    }


def _perception(row):
    metadata = _dict(row.get("metadata"))
    mode = metadata.get("mode")
    if mode is None and row.get("mode") in _PERCEPTION_MODES:
        # Compatibility with early, perception-only logs. Never treat AUTO as
        # a perception state when metadata is absent.
        mode = row.get("mode")
    if not isinstance(mode, str):
        return "UNKNOWN"
    mode = mode.strip().upper()
    if mode in _PERCEPTION_MODES:
        return mode
    return "OTHER"


def _driving(row):
    metadata = _dict(row.get("metadata"))
    mode = _dict(metadata.get("vehicle_control")).get("mode", row.get("mode"))
    if not isinstance(mode, str):
        return "UNKNOWN"
    mode = mode.strip().upper()
    return mode if mode in _DRIVING_MODES else "UNKNOWN"


def _path(value):
    if not isinstance(value, str) or not 1 <= len(value) <= 256:
        return None
    path = PurePosixPath(value)
    if path.is_absolute() or ".." in path.parts or "\\" in value or "\x00" in value:
        return None
    return value


def _reference(row, index):
    extra = _dict(row.get("extra_masks"))
    result = {"row_index": index, "frame": row.get("frame"),
              "source_frame_index": _dict(row.get("metadata")).get("frame_index"),
              "image": _path(row.get("image")), "mask": _path(row.get("mask")),
              "extra_masks": {str(key): path for key, value in extra.items()
                              if (path := _path(value)) is not None},
              "perception_mode": _perception(row), "driving_mode": _driving(row)}
    # Scalar identifiers are sufficient and guarantee strict JSON output even
    # when the input came from a damaged/custom diagnostic log.
    for key in ("frame", "source_frame_index"):
        value = result[key]
        if not isinstance(value, (str, int)) or isinstance(value, bool):
            result[key] = None
    return result


def analyze_rows(rows, events=None):
    """Return machine-readable observations, at most 12 representative frames.

    Input rows and events are not changed. Missing/nonfinite values remain
    unknown. Non-TRACK spans refer only to sequential saved observations;
    unrecorded control ticks and truth between saved samples are unknown.
    ``events`` is optional and normally comes from events.jsonl.
    """
    invalid_input_type = not isinstance(rows, (list, tuple))
    supplied = rows if not invalid_input_type else []
    valid = [row for row in supplied if isinstance(row, dict)]
    count = len(valid)
    modes = [_perception(row) for row in valid]
    perception_counts = Counter(modes)
    driving_counts = Counter(_driving(row) for row in valid)
    relative = [_number(row.get("relative_time_ms")) for row in valid]
    absolute = [_number(row.get("capture_time_unix_ms")) for row in valid]
    if any(value is not None for value in relative):
        clock, times = "relative_time_ms", relative
    elif any(value is not None for value in absolute):
        clock, times = "capture_time_unix_ms", absolute
    else:
        clock, times = None, [None] * count
    spacing, nonmonotonic = [], 0
    for previous, current in zip(times, times[1:]):
        if previous is None or current is None:
            continue
        if current < previous:
            nonmonotonic += 1
        else:
            difference = _number(current - previous)
            if difference is not None:
                spacing.append(difference)

    timing_values = {name: [] for name in _TIMINGS}
    signed_gaps, gap_by_index, age_by_index = [], {}, {}
    candidates = {"frames_with_candidates": 0, "candidates_total": 0,
                  "score_ge_1_count": 0, "score_lt_1_count": 0,
                  "score_missing_or_invalid_count": 0,
                  "explicit_accepted_true_count": 0, "explicit_accepted_false_count": 0,
                  "explicit_accepted_missing_count": 0,
                  "note": "score>=1は記録された採用スコア区分。最終点列への採用・正誤・拒否原因を推測しない。"}
    scores, score_counts = [], Counter()
    for index, row in enumerate(valid):
        metadata = _dict(row.get("metadata"))
        timing = _dict(metadata.get("loop_timing"))
        if not timing:
            timing = _dict(metadata.get("timings"))
        for name in _TIMINGS:
            value = _number(timing.get(name))
            if value is not None and value >= 0:
                timing_values[name].append(value)
                if name == "camera_to_output_ms":
                    age_by_index[index] = value
        raw, used = _number(metadata.get("raw_center_error")), _number(metadata.get("used_center_error"))
        if raw is not None and used is not None:
            difference = _number(raw - used)
            if difference is not None:
                signed_gaps.append(difference)
                gap_by_index[index] = abs(difference)
        row_candidates = metadata.get("candidates")
        if not isinstance(row_candidates, (list, tuple)):
            continue
        entries = [entry for entry in row_candidates if isinstance(entry, dict)]
        candidates["frames_with_candidates"] += bool(entries)
        for candidate in entries:
            candidates["candidates_total"] += 1
            accepted = candidate.get("accepted")
            if accepted is True:
                candidates["explicit_accepted_true_count"] += 1
            elif accepted is False:
                candidates["explicit_accepted_false_count"] += 1
            else:
                candidates["explicit_accepted_missing_count"] += 1
            score = _number(candidate.get("score"))
            if score is None:
                candidates["score_missing_or_invalid_count"] += 1
            else:
                scores.append(score)
                score_counts[format(score, ".8g")] += 1
                candidates["score_ge_1_count" if score >= 1. else "score_lt_1_count"] += 1
    visible_scores = score_counts.most_common(64)
    candidates["score_counts"] = dict(sorted(visible_scores))
    candidates["score_counts_other_occurrences"] = sum(score_counts.values()) - sum(value for _, value in visible_scores)
    candidates["score_distribution"] = _distribution(scores)

    # A missing/unknown state interrupts a known non-TRACK span. At the clip
    # boundary the duration ends at the last observed sample, never extrapolated.
    spans, active_start = [], None
    for index in range(count + 1):
        is_bad = index < count and modes[index] in _BAD_MODES
        if is_bad and active_start is None:
            active_start = index
        elif not is_bad and active_start is not None:
            end = index - 1
            recovered = index if index < count and modes[index] == "TRACK" else None
            end_time = times[recovered] if recovered is not None else times[end]
            start_time = times[active_start]
            duration = None
            if start_time is not None and end_time is not None and end_time >= start_time:
                difference = _number(end_time - start_time)
                if difference is not None:
                    duration = round(difference, 3)
            spans.append({"start_row_index": active_start, "end_row_index": end,
                          "start_frame": _reference(valid[active_start], active_start)["frame"],
                          "end_frame": _reference(valid[end], end)["frame"],
                          "recovery_row_index": recovered,
                          "sample_count": index - active_start,
                          "observed_duration_ms": duration,
                          "starts_at_clip_boundary": active_start == 0,
                          "ends_at_clip_boundary": end == count - 1,
                          "ended_on_unknown_state": index < count and modes[index] in ("UNKNOWN", "OTHER"),
                          "states": sorted(set(modes[active_start:index]))})
            active_start = None
    measured_spans = [span for span in spans if span["observed_duration_ms"] is not None]
    longest = max(measured_spans, key=lambda span: span["observed_duration_ms"], default=None)

    # Equal per-category allowance prevents one long loss or latency burst from
    # hiding marker context and filter-lag examples in the 12-frame review set.
    loss_indices = []
    for span in sorted(spans, key=lambda value: value["observed_duration_ms"] or 0, reverse=True):
        if not set(span["states"]) & {"LOST", "STOP"}:
            continue
        start, end = span["start_row_index"], span["end_row_index"]
        loss_indices.extend((start, end, span["recovery_row_index"]))
    gaps = sorted(gap_by_index, key=lambda index: gap_by_index[index], reverse=True)
    ages = sorted(age_by_index, key=lambda index: age_by_index[index], reverse=True)
    mark_indices = []
    invalid_events = 0
    event_list = events if isinstance(events, (list, tuple)) else []
    for event in event_list:
        if not isinstance(event, dict):
            invalid_events += 1
            continue
        event_frame = event.get("frame")
        matches = [index for index, row in enumerate(valid) if event_frame is not None and row.get("frame") == event_frame]
        if not matches:
            event_time = _number(event.get("time_unix_ms"))
            timed = [(abs(value - event_time), index) for index, value in enumerate(absolute)
                     if value is not None and event_time is not None]
            if timed:
                matches = [min(timed)[1]]
        if matches:
            index = matches[0]
            mark_indices.extend((index, index - 1 if index else None,
                                 index + 1 if index + 1 < count else None))
    selections = {}
    for category, indices in (("perception_loss", loss_indices),
                              ("largest_raw_used_gap", gaps),
                              ("largest_camera_age", ages),
                              ("marker_nearby", mark_indices)):
        taken = set()
        for index in indices:
            if index is None or index in taken:
                continue
            taken.add(index)
            chosen = selections.setdefault(index, {"reasons": []})
            chosen["reasons"].append(category)
            if len(taken) >= 3:
                break
    representatives = []
    for index in sorted(selections):
        reference = _reference(valid[index], index)
        reference.update(selections[index])
        reference["time_ms"] = times[index]
        reference["abs_raw_used_gap_px"] = gap_by_index.get(index)
        reference["camera_to_output_ms"] = age_by_index.get(index)
        representatives.append(reference)
    return {
        "schema_version": 1,
        "limitations": {
            "ground_truth_available": False,
            "not_a_detection_accuracy_measurement": True,
            "actuator_values_are_software_commands_not_measured_angles": True,
            "sampled_recording_default_target_fps": 10,
            "all_control_ticks_recorded": False,
            "note": "通常10fpsで保存した観測の集計。正解ラベルなし。知覚状態の比率は認識精度ではなく、指令値は実舵角ではない。フレーム間の状態と区間端の継続時間は未観測。",
        },
        "input": {"rows": len(supplied), "valid_rows": count,
                  "invalid_rows": len(supplied) - count,
                  "invalid_rows_container": invalid_input_type,
                  "invalid_events": invalid_events,
                  "time_basis": clock, "missing_timestamps": sum(value is None for value in times),
                  "backwards_timestamp_steps": nonmonotonic,
                  "sample_spacing_ms": _distribution(spacing)},
        "perception_state": {
            "counts": {mode: perception_counts[mode] for mode in _PERCEPTION_MODES},
            "ratios": {mode: round(perception_counts[mode] / count, 6) if count else None for mode in _PERCEPTION_MODES},
            "ratio_denominator": "all_valid_saved_rows_including_unknown",
            "non_track_span_count": len(spans),
            "non_track_spans": sorted(spans, key=lambda value: value["observed_duration_ms"] or 0, reverse=True)[:20],
            "non_track_spans_detail_limit": 20,
            "longest_non_track_ms": longest["observed_duration_ms"] if longest is not None else (0 if not spans else None),
            "longest_non_track_span": longest,
        },
        "driving_mode_counts": dict(sorted(driving_counts.items())),
        "raw_used_gap_px": {
            "signed_raw_minus_used": _distribution(signed_gaps, count),
            "absolute": _distribution(list(gap_by_index.values()), count),
            "frames_abs_gap_ge_10_px": sum(value >= 10 for value in gap_by_index.values()),
        },
        "timings_ms": {name: _distribution(timing_values[name], count) for name in _TIMINGS},
        "candidate_observations": candidates,
        "corner_pace": _corner_pace_summary(valid),
        "representative_frames": representatives,
    }
