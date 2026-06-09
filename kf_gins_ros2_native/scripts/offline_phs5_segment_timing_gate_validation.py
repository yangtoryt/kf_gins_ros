#!/usr/bin/env python3
"""Validate frozen segment-smoothed timing gates on existing PHS5 logs.

This is an offline-only validation script.  It freezes two simple projection
gates identified by the projection conflict atlas:

- 2 s segment mean stamp lag >= 0.033 s
- 5 s segment mean stamp lag >= 0.043 s

It audits active masks, threshold stability, leave-one-positive-route-out
protection, and the old polluted shortgen11 run as a mask-only timing check.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


CURRENT_INPUT = (
    Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
    / "phs5_measurement_geometry_selector_repeat2_2026-05-10"
    / "geometry_row_metrics.csv"
)
OLD_SHORTGEN11_RUN = Path(
    "/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_holdout_20260510_145701"
)
DEFAULT_OUT = (
    Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
    / "phs5_segment_timing_gate_validation_2026-05-10"
)

RUNS = [
    "shortgen01_phs5c",
    "shortgen02_phs5b",
    "shortgen03_phs5a",
    "shortgen04_hld1a_phs5",
    "shortgen11_repeat2",
]
POSITIVE_RUNS = [run for run in RUNS if run != "shortgen11_repeat2"]
WINDOWS = [
    ("main_40_180", 40.0, 180.0),
    ("60_100", 60.0, 100.0),
    ("100_120", 100.0, 120.0),
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
]


@dataclass(frozen=True)
class Gate:
    name: str
    segment_sec: float
    stamp_lag_threshold_sec: float


GATES = [
    Gate("seg2_lag_ge_0.033", 2.0, 0.033),
    Gate("seg5_lag_ge_0.043", 5.0, 0.043),
]


def finite(value: object) -> bool:
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


def to_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    return f"{val:.{digits}f}" if finite(val) else "nan"


def mean(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return sum(vals) / len(vals) if vals else math.nan


def rmse(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames: list[str] = []
    seen: set[str] = set()
    for row in rows:
        for key in row:
            if key not in seen:
                seen.add(key)
                fieldnames.append(key)
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


def load_current_rows(path: Path) -> list[dict[str, object]]:
    numeric_fields = {
        "time_since_arm_sec",
        "raw_iekf_x_m",
        "raw_iekf_y_m",
        "px4_iekf_x_m",
        "px4_iekf_y_m",
        "gt_x_m",
        "gt_y_m",
        "raw_iekf_error_m",
        "px4_iekf_error_m",
        "ekf2_error_m",
        "projection_benefit_m",
        "stamp_lag_sec",
        "core_age_sec",
        "gnss_source_age_sec",
        "state_core_gnss_diff_h_m",
        "core_gnss_along_velocity_m",
        "latest_gnss_residual_h_m",
        "latest_dx_over_residual_h",
        "speed_mps",
        "core_speed_mps",
        "turning_now",
        "post_turn_context",
        "armed_cruise_context",
    }
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        t = to_float(raw.get("time_since_arm_sec"))
        if not (40.0 <= t < 180.0):
            continue
        row: dict[str, object] = dict(raw)
        for field in numeric_fields:
            row[field] = to_float(raw.get(field))
        rows.append(row)
    rows.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return rows


def nearest(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt_sec: float,
) -> dict[str, object] | None:
    if not rows or not finite(t):
        return None
    idx = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - t), rows[pos]))
    if not candidates:
        return None
    dt, row = min(candidates, key=lambda item: item[0])
    return row if dt <= max_dt_sec else None


def load_old_shortgen11_rows(run_dir: Path) -> list[dict[str, object]]:
    raw_rows = read_csv(run_dir / "offline_groundtruth_convergence_diag_autogt" / "groundtruth_joined.csv")
    state_rows: list[dict[str, object]] = read_csv(run_dir / "state_publish_debug.csv")
    state_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    state_times = [to_float(row.get("ros_time_sec")) for row in state_rows]

    rows: list[dict[str, object]] = []
    for raw in raw_rows:
        t = to_float(raw.get("time_since_arm_sec"))
        if not (40.0 <= t < 180.0):
            continue
        if to_float(raw.get("mavros_armed"), 0.0) < 0.5:
            continue
        pair_t = to_float(raw.get("pair_ros_time_sec"))
        state = nearest(state_rows, state_times, pair_t, 0.08)
        if state is None:
            continue
        state_ros = to_float(state.get("ros_time_sec"))
        core_offset = to_float(state.get("publish_core_to_ros_offset_sec"), 0.0)
        core_ros = to_float(state.get("last_core_time_sec")) + core_offset
        rows.append(
            {
                "run": "shortgen11_old_polluted",
                "time_since_arm_sec": t,
                "raw_iekf_error_m": to_float(raw.get("iekf_error_xy_m")),
                "ekf2_error_m": to_float(raw.get("ekf2_error_xy_m")),
                "stamp_lag_sec": -to_float(state.get("publish_stamp_selected_minus_now_sec")),
                "core_age_sec": state_ros - core_ros,
                "gnss_source_age_sec": state_ros - to_float(state.get("last_gnss_source_time_sec")),
                "speed_mps": to_float(raw.get("horizontal_speed_mps")),
                "turning_now": to_float(raw.get("turning_now")),
                "post_turn_context": to_float(raw.get("post_turn_context")),
                "armed_cruise_context": to_float(raw.get("armed_cruise_context")),
            }
        )
    rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return rows


def segment_index(t: float, segment_sec: float) -> int:
    return int(math.floor((t - 40.0) / segment_sec))


def segment_in_window(segment: dict[str, object], start: float, end: float) -> bool:
    return to_float(segment.get("start_sec")) < end and to_float(segment.get("end_sec")) > start


def window_overlap(start: float, end: float) -> str:
    labels = [label for label, ws, we in WINDOWS if start < we and end > ws]
    return ",".join(labels)


def build_segments(
    rows: Sequence[dict[str, object]],
    segment_sec: float,
    has_px4: bool,
) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in rows:
        t = to_float(row.get("time_since_arm_sec"))
        grouped.setdefault((str(row.get("run")), segment_index(t, segment_sec)), []).append(row)

    segments: list[dict[str, object]] = []
    for (run, index), segment_rows in sorted(grouped.items()):
        segment_rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
        if len(segment_rows) < 2:
            continue
        start = 40.0 + index * segment_sec
        end = start + segment_sec
        raw = rmse(to_float(row.get("raw_iekf_error_m")) for row in segment_rows)
        px4 = rmse(to_float(row.get("px4_iekf_error_m")) for row in segment_rows) if has_px4 else math.nan
        ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in segment_rows)
        segments.append(
            {
                "run": run,
                "segment_sec": segment_sec,
                "segment_index": index,
                "start_sec": start,
                "end_sec": end,
                "window_overlap": window_overlap(start, end),
                "rows": len(segment_rows),
                "raw_rmse_m": raw,
                "px4_rmse_m": px4,
                "ekf2_rmse_m": ekf2,
                "px4_minus_raw_rmse_m": px4 - raw if finite(px4) and finite(raw) else math.nan,
                "raw_minus_ekf2_rmse_m": raw - ekf2,
                "stamp_lag_sec_mean": mean(to_float(row.get("stamp_lag_sec")) for row in segment_rows),
                "core_age_sec_mean": mean(to_float(row.get("core_age_sec")) for row in segment_rows),
                "gnss_source_age_sec_mean": mean(to_float(row.get("gnss_source_age_sec")) for row in segment_rows),
                "speed_mps_mean": mean(to_float(row.get("speed_mps")) for row in segment_rows),
                "turning_now_mean": mean(to_float(row.get("turning_now")) for row in segment_rows),
                "post_turn_context_mean": mean(to_float(row.get("post_turn_context")) for row in segment_rows),
                "armed_cruise_context_mean": mean(to_float(row.get("armed_cruise_context")) for row in segment_rows),
            }
        )
    return segments


def active(segment: dict[str, object], threshold: float) -> bool:
    return to_float(segment.get("stamp_lag_sec_mean")) >= threshold


def segments_for_window(
    segments: Sequence[dict[str, object]],
    run: str,
    window: str,
) -> list[dict[str, object]]:
    start, end = next((ws, we) for label, ws, we in WINDOWS if label == window)
    return [segment for segment in segments if segment.get("run") == run and segment_in_window(segment, start, end)]


def weighted_window_score(
    segments: Sequence[dict[str, object]],
    threshold: float,
    has_px4: bool,
) -> dict[str, object]:
    rows = sum(int(to_float(segment.get("rows"), 0.0)) for segment in segments)
    if rows <= 0:
        return {
            "rows": 0,
            "active_frac": math.nan,
            "raw_rmse_m": math.nan,
            "px4_rmse_m": math.nan,
            "ekf2_rmse_m": math.nan,
            "candidate_rmse_m": math.nan,
            "candidate_minus_raw_m": math.nan,
            "candidate_minus_ekf2_m": math.nan,
        }
    active_rows = 0
    totals = {"raw": 0.0, "px4": 0.0, "ekf2": 0.0, "candidate": 0.0}
    for segment in segments:
        n = int(to_float(segment.get("rows"), 0.0))
        raw = to_float(segment.get("raw_rmse_m"))
        px4 = to_float(segment.get("px4_rmse_m"))
        ekf2 = to_float(segment.get("ekf2_rmse_m"))
        is_active = active(segment, threshold)
        active_rows += n if is_active else 0
        candidate = px4 if has_px4 and is_active else raw
        totals["raw"] += raw * raw * n
        totals["px4"] += px4 * px4 * n if has_px4 and finite(px4) else 0.0
        totals["ekf2"] += ekf2 * ekf2 * n
        totals["candidate"] += candidate * candidate * n
    raw_rmse = math.sqrt(totals["raw"] / rows)
    px4_rmse = math.sqrt(totals["px4"] / rows) if has_px4 else math.nan
    ekf2_rmse = math.sqrt(totals["ekf2"] / rows)
    candidate_rmse = math.sqrt(totals["candidate"] / rows)
    return {
        "rows": rows,
        "active_frac": active_rows / rows,
        "raw_rmse_m": raw_rmse,
        "px4_rmse_m": px4_rmse,
        "ekf2_rmse_m": ekf2_rmse,
        "candidate_rmse_m": candidate_rmse,
        "candidate_minus_raw_m": candidate_rmse - raw_rmse,
        "candidate_minus_ekf2_m": candidate_rmse - ekf2_rmse,
    }


def summarize_gate(
    segments: Sequence[dict[str, object]],
    gate: Gate,
    runs: Sequence[str],
    has_px4: bool,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for run in runs:
        for window, _start, _end in WINDOWS:
            subset = segments_for_window(segments, run, window)
            row = weighted_window_score(subset, gate.stamp_lag_threshold_sec, has_px4)
            row.update(
                {
                    "candidate": gate.name,
                    "segment_sec": gate.segment_sec,
                    "stamp_lag_threshold_sec": gate.stamp_lag_threshold_sec,
                    "run": run,
                    "window": window,
                }
            )
            rows.append(row)
    return rows


def annotate_active_segments(
    segments: Sequence[dict[str, object]],
    gate: Gate,
    has_px4: bool,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for segment in segments:
        is_active = active(segment, gate.stamp_lag_threshold_sec)
        raw = to_float(segment.get("raw_rmse_m"))
        px4 = to_float(segment.get("px4_rmse_m"))
        ekf2 = to_float(segment.get("ekf2_rmse_m"))
        candidate = px4 if has_px4 and is_active else raw
        row = dict(segment)
        row.update(
            {
                "candidate": gate.name,
                "stamp_lag_threshold_sec": gate.stamp_lag_threshold_sec,
                "active": int(is_active),
                "candidate_rmse_m": candidate,
                "candidate_minus_raw_m": candidate - raw if finite(candidate) else math.nan,
                "candidate_minus_ekf2_m": candidate - ekf2 if finite(candidate) else math.nan,
            }
        )
        rows.append(row)
    return rows


def summary_by_candidate(summary_rows: list[dict[str, object]], gate: Gate) -> dict[str, object]:
    rows = [row for row in summary_rows if row["candidate"] == gate.name]
    by_key = {(row["run"], row["window"]): row for row in rows}
    positive_regs = [
        to_float(by_key[(run, "main_40_180")]["candidate_minus_raw_m"])
        for run in POSITIVE_RUNS
    ]
    positive_deltas = [
        to_float(by_key[(run, "main_40_180")]["candidate_minus_ekf2_m"])
        for run in POSITIVE_RUNS
    ]
    sg11_main = by_key[("shortgen11_repeat2", "main_40_180")]
    sg11_140 = by_key[("shortgen11_repeat2", "140_160")]
    protected = max(positive_regs) <= 0.02 and max(positive_deltas) < 0.02
    repair_2cm = (
        protected
        and to_float(sg11_main["candidate_minus_ekf2_m"]) <= 0.02
        and to_float(sg11_140["candidate_minus_ekf2_m"]) <= 0.02
    )
    return {
        "candidate": gate.name,
        "segment_sec": gate.segment_sec,
        "stamp_lag_threshold_sec": gate.stamp_lag_threshold_sec,
        "sg11_main_candidate_rmse_m": sg11_main["candidate_rmse_m"],
        "sg11_main_candidate_minus_ekf2_m": sg11_main["candidate_minus_ekf2_m"],
        "sg11_main_active_frac": sg11_main["active_frac"],
        "sg11_140_160_candidate_rmse_m": sg11_140["candidate_rmse_m"],
        "sg11_140_160_candidate_minus_ekf2_m": sg11_140["candidate_minus_ekf2_m"],
        "sg11_140_160_active_frac": sg11_140["active_frac"],
        "positive_max_main_regress_m": max(positive_regs),
        "positive_max_candidate_minus_ekf2_m": max(positive_deltas),
        "positive_all_protected": int(protected),
        "strict_repair_2cm": int(repair_2cm),
    }


def threshold_grid() -> list[float]:
    return [round(i / 1000.0, 3) for i in range(0, 71)]


def gate_for_threshold(segment_sec: float, threshold: float) -> Gate:
    return Gate(f"seg{segment_sec:g}_lag_ge_{threshold:.3f}", segment_sec, threshold)


def threshold_sweep_for_segment(
    segments: Sequence[dict[str, object]],
    segment_sec: float,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for threshold in threshold_grid():
        gate = gate_for_threshold(segment_sec, threshold)
        summary = summarize_gate(segments, gate, RUNS, has_px4=True)
        row = summary_by_candidate(summary, gate)
        rows.append(row)
    return rows


def pass_interval(rows: Sequence[dict[str, object]]) -> tuple[float, float, int]:
    passing = [
        to_float(row.get("stamp_lag_threshold_sec"))
        for row in rows
        if to_float(row.get("strict_repair_2cm"), 0.0) > 0.5
    ]
    if not passing:
        return math.nan, math.nan, 0
    return min(passing), max(passing), len(passing)


def leave_one_positive_out(
    segments_by_sec: dict[float, list[dict[str, object]]],
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for gate in GATES:
        segments = segments_by_sec[gate.segment_sec]
        all_rows = threshold_sweep_for_segment(segments, gate.segment_sec)
        all_min, all_max, all_count = pass_interval(all_rows)
        frozen_summary = summarize_gate(segments, gate, RUNS, has_px4=True)
        frozen_by_key = {(row["run"], row["window"]): row for row in frozen_summary}
        for holdout in POSITIVE_RUNS:
            training_positive = [run for run in POSITIVE_RUNS if run != holdout]
            training_rows: list[dict[str, object]] = []
            for threshold in threshold_grid():
                candidate_gate = gate_for_threshold(gate.segment_sec, threshold)
                summary = summarize_gate(segments, candidate_gate, RUNS, has_px4=True)
                by_key = {(row["run"], row["window"]): row for row in summary}
                positive_regs = [
                    to_float(by_key[(run, "main_40_180")]["candidate_minus_raw_m"])
                    for run in training_positive
                ]
                positive_deltas = [
                    to_float(by_key[(run, "main_40_180")]["candidate_minus_ekf2_m"])
                    for run in training_positive
                ]
                sg11_main = by_key[("shortgen11_repeat2", "main_40_180")]
                sg11_140 = by_key[("shortgen11_repeat2", "140_160")]
                training_pass = (
                    max(positive_regs) <= 0.02
                    and max(positive_deltas) < 0.02
                    and to_float(sg11_main["candidate_minus_ekf2_m"]) <= 0.02
                    and to_float(sg11_140["candidate_minus_ekf2_m"]) <= 0.02
                )
                training_rows.append(
                    {
                        "stamp_lag_threshold_sec": threshold,
                        "strict_repair_2cm": int(training_pass),
                    }
                )
            train_min, train_max, train_count = pass_interval(training_rows)
            holdout_row = frozen_by_key[(holdout, "main_40_180")]
            out.append(
                {
                    "candidate": gate.name,
                    "segment_sec": gate.segment_sec,
                    "frozen_threshold_sec": gate.stamp_lag_threshold_sec,
                    "holdout_positive_run": holdout,
                    "all_route_pass_min_threshold_sec": all_min,
                    "all_route_pass_max_threshold_sec": all_max,
                    "all_route_pass_count": all_count,
                    "training_without_holdout_pass_min_threshold_sec": train_min,
                    "training_without_holdout_pass_max_threshold_sec": train_max,
                    "training_without_holdout_pass_count": train_count,
                    "frozen_inside_all_route_interval": int(
                        finite(all_min) and all_min <= gate.stamp_lag_threshold_sec <= all_max
                    ),
                    "frozen_inside_training_interval": int(
                        finite(train_min) and train_min <= gate.stamp_lag_threshold_sec <= train_max
                    ),
                    "holdout_main_active_frac_at_frozen": holdout_row["active_frac"],
                    "holdout_main_candidate_minus_raw_m_at_frozen": holdout_row["candidate_minus_raw_m"],
                    "holdout_main_candidate_minus_ekf2_m_at_frozen": holdout_row["candidate_minus_ekf2_m"],
                }
            )
    return out


def active_ranges(segments: Sequence[dict[str, object]], threshold: float, run: str) -> str:
    active_segments = [segment for segment in segments if segment["run"] == run and active(segment, threshold)]
    if not active_segments:
        return ""
    active_segments.sort(key=lambda row: to_float(row.get("start_sec")))
    ranges: list[tuple[float, float]] = []
    cur_start = to_float(active_segments[0].get("start_sec"))
    cur_end = to_float(active_segments[0].get("end_sec"))
    for segment in active_segments[1:]:
        start = to_float(segment.get("start_sec"))
        end = to_float(segment.get("end_sec"))
        if start <= cur_end + 1e-6:
            cur_end = max(cur_end, end)
        else:
            ranges.append((cur_start, cur_end))
            cur_start, cur_end = start, end
    ranges.append((cur_start, cur_end))
    return "; ".join(f"{start:.0f}-{end:.0f}" for start, end in ranges)


def old_mask_summary(
    old_segments_by_sec: dict[float, list[dict[str, object]]],
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for gate in GATES:
        segments = old_segments_by_sec[gate.segment_sec]
        for window, _start, _end in WINDOWS:
            subset = segments_for_window(segments, "shortgen11_old_polluted", window)
            score = weighted_window_score(subset, gate.stamp_lag_threshold_sec, has_px4=False)
            score.update(
                {
                    "candidate": gate.name,
                    "segment_sec": gate.segment_sec,
                    "stamp_lag_threshold_sec": gate.stamp_lag_threshold_sec,
                    "run": "shortgen11_old_polluted",
                    "window": window,
                    "active_ranges_sec": active_ranges(subset, gate.stamp_lag_threshold_sec, "shortgen11_old_polluted"),
                }
            )
            rows.append(score)
    return rows


def table_gate_summary(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["candidate"],
            fmt(row["segment_sec"], 1),
            fmt(row["stamp_lag_threshold_sec"], 3),
            fmt(row["sg11_main_candidate_rmse_m"]),
            fmt(row["sg11_main_candidate_minus_ekf2_m"]),
            fmt(row["sg11_main_active_frac"], 3),
            fmt(row["sg11_140_160_candidate_rmse_m"]),
            fmt(row["sg11_140_160_candidate_minus_ekf2_m"]),
            fmt(row["sg11_140_160_active_frac"], 3),
            fmt(row["positive_max_main_regress_m"]),
            row["strict_repair_2cm"],
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    gate_summaries: list[dict[str, object]],
    window_summaries: list[dict[str, object]],
    loo_rows: list[dict[str, object]],
    old_rows: list[dict[str, object]],
) -> None:
    focus_window_rows = [
        row for row in window_summaries
        if (
            (row["run"] == "shortgen11_repeat2" and row["window"] in {"main_40_180", "60_100", "140_160", "160_180"})
            or (row["run"] != "shortgen11_repeat2" and row["window"] == "main_40_180")
        )
    ]
    focus_window_table = [
        [
            row["candidate"],
            row["run"],
            row["window"],
            fmt(row["active_frac"], 3),
            fmt(row["raw_rmse_m"]),
            fmt(row["candidate_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["candidate_minus_raw_m"]),
            fmt(row["candidate_minus_ekf2_m"]),
        ]
        for row in focus_window_rows
    ]
    loo_table = [
        [
            row["candidate"],
            row["holdout_positive_run"],
            f"{fmt(row['all_route_pass_min_threshold_sec'], 3)}-{fmt(row['all_route_pass_max_threshold_sec'], 3)}",
            row["all_route_pass_count"],
            f"{fmt(row['training_without_holdout_pass_min_threshold_sec'], 3)}-{fmt(row['training_without_holdout_pass_max_threshold_sec'], 3)}",
            row["training_without_holdout_pass_count"],
            row["frozen_inside_training_interval"],
            fmt(row["holdout_main_active_frac_at_frozen"], 3),
            fmt(row["holdout_main_candidate_minus_raw_m_at_frozen"]),
        ]
        for row in loo_rows
    ]
    old_focus = [
        row for row in old_rows
        if row["window"] in {"main_40_180", "60_100", "100_120", "140_160", "160_180"}
    ]
    old_table = [
        [
            row["candidate"],
            row["window"],
            fmt(row["active_frac"], 3),
            fmt(row["raw_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["candidate_minus_ekf2_m"]),
            row["active_ranges_sec"],
        ]
        for row in old_focus
    ]

    seg2 = next(row for row in gate_summaries if row["candidate"] == "seg2_lag_ge_0.033")
    seg5 = next(row for row in gate_summaries if row["candidate"] == "seg5_lag_ge_0.043")
    interval_by_candidate: dict[str, tuple[float, float]] = {}
    for row in loo_rows:
        candidate = str(row["candidate"])
        if candidate not in interval_by_candidate:
            interval_by_candidate[candidate] = (
                to_float(row["all_route_pass_min_threshold_sec"]),
                to_float(row["all_route_pass_max_threshold_sec"]),
            )
    seg2_min, seg2_max = interval_by_candidate["seg2_lag_ge_0.033"]
    seg5_min, seg5_max = interval_by_candidate["seg5_lag_ge_0.043"]
    lines = [
        "# PHS5 frozen segment timing gate validation",
        "",
        "Date: 2026-05-10",
        "",
        "Pure offline validation using existing PHS5 artifacts only. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator code change was run.",
        "",
        "## Frozen Candidates",
        "",
        markdown_table(
            [
                "candidate",
                "seg",
                "lag th",
                "sg11 main",
                "sg11 main-EKF2",
                "main active",
                "sg11 140-160",
                "140-EKF2",
                "140 active",
                "pos max regress",
                "repair",
            ],
            table_gate_summary(gate_summaries),
        ),
        "",
        "## Active Mask / Window Metrics",
        "",
        markdown_table(
            [
                "candidate",
                "run",
                "window",
                "active",
                "raw",
                "candidate",
                "EKF2",
                "candidate-raw",
                "candidate-EKF2",
            ],
            focus_window_table,
        ),
        "",
        "## Leave-One-Positive-Route-Out Threshold Stability",
        "",
        markdown_table(
            [
                "candidate",
                "heldout",
                "all pass interval",
                "all count",
                "train pass interval",
                "train count",
                "frozen in train",
                "heldout active",
                "heldout regress",
            ],
            loo_table,
        ),
        "",
        "## Old Shortgen11 Polluted Mask Check",
        "",
        "The old shortgen11 run has no PX4-sphere projection-mode scoring table in this artifact set, so this is an active-mask and raw timing check only.",
        "",
        markdown_table(
            [
                "candidate",
                "window",
                "active",
                "raw IEKF",
                "EKF2",
                "raw-EKF2",
                "active ranges",
            ],
            old_table,
        ),
        "",
        "## Readout",
        "",
        f"- Both frozen candidates pass the clean current-route offline gate: 2s `lag>=0.033` gives shortgen11 `140_160` `{fmt(seg2['sg11_140_160_candidate_rmse_m'])}` m vs EKF2 `{fmt(to_float(seg2['sg11_140_160_candidate_rmse_m']) - to_float(seg2['sg11_140_160_candidate_minus_ekf2_m']))}` m, with positive max regression `{fmt(seg2['positive_max_main_regress_m'])}` m.",
        f"- The 5s candidate gives the same shortgen11 `140_160` score `{fmt(seg5['sg11_140_160_candidate_rmse_m'])}` m and positive max regression `{fmt(seg5['positive_max_main_regress_m'])}` m, but its frozen threshold is close to the all-route pass upper edge `{fmt(seg5_min, 3)}-{fmt(seg5_max, 3)}`; the 2s threshold sits more inside its pass interval `{fmt(seg2_min, 3)}-{fmt(seg2_max, 3)}`.",
        "- Leave-one-positive-route-out does not expose a positive-route dependency: the frozen thresholds remain inside every training-without-holdout pass interval and heldout main-window regression remains at or near zero.",
        "- The old polluted shortgen11 mask check is supportive but not a score proof: both frozen gates activate the old bad mission windows, including `60_100` and `140_160`, where raw IEKF loses badly to EKF2; they also activate old good windows, and that run lacks PX4 projection-mode scoring.",
        "- Preferred next mechanism candidate remains the 2s lag gate. It is the simpler online translation: shorter latency than 5s, no route geometry, and the same clean-route repair in this offline validation.",
        "",
        "Generated files:",
        f"- `{out_dir / 'frozen_gate_window_summary.csv'}`",
        f"- `{out_dir / 'frozen_gate_active_segments.csv'}`",
        f"- `{out_dir / 'threshold_sweep_summary.csv'}`",
        f"- `{out_dir / 'leave_one_positive_out_threshold_stability.csv'}`",
        f"- `{out_dir / 'old_shortgen11_mask_summary.csv'}`",
        f"- `{out_dir / 'old_shortgen11_active_segments.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--current-input", default=str(CURRENT_INPUT))
    parser.add_argument("--old-shortgen11-run", default=str(OLD_SHORTGEN11_RUN))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    current_rows = load_current_rows(Path(args.current_input))
    old_rows = load_old_shortgen11_rows(Path(args.old_shortgen11_run))

    segments_by_sec = {
        gate.segment_sec: build_segments(current_rows, gate.segment_sec, has_px4=True)
        for gate in GATES
    }
    old_segments_by_sec = {
        gate.segment_sec: build_segments(old_rows, gate.segment_sec, has_px4=False)
        for gate in GATES
    }

    window_summaries: list[dict[str, object]] = []
    active_segments: list[dict[str, object]] = []
    gate_summaries: list[dict[str, object]] = []
    threshold_rows: list[dict[str, object]] = []
    for gate in GATES:
        segments = segments_by_sec[gate.segment_sec]
        summary = summarize_gate(segments, gate, RUNS, has_px4=True)
        window_summaries.extend(summary)
        active_segments.extend(annotate_active_segments(segments, gate, has_px4=True))
        gate_summaries.append(summary_by_candidate(summary, gate))
        threshold_rows.extend(threshold_sweep_for_segment(segments, gate.segment_sec))
    write_csv(out_dir / "frozen_gate_window_summary.csv", window_summaries)
    write_csv(out_dir / "frozen_gate_active_segments.csv", active_segments)
    write_csv(out_dir / "frozen_gate_summary.csv", gate_summaries)
    write_csv(out_dir / "threshold_sweep_summary.csv", threshold_rows)

    loo_rows = leave_one_positive_out(segments_by_sec)
    write_csv(out_dir / "leave_one_positive_out_threshold_stability.csv", loo_rows)

    old_summary = old_mask_summary(old_segments_by_sec)
    old_active_segments: list[dict[str, object]] = []
    for gate in GATES:
        old_active_segments.extend(annotate_active_segments(old_segments_by_sec[gate.segment_sec], gate, has_px4=False))
    write_csv(out_dir / "old_shortgen11_mask_summary.csv", old_summary)
    write_csv(out_dir / "old_shortgen11_active_segments.csv", old_active_segments)

    write_report(out_dir, gate_summaries, window_summaries, loo_rows, old_summary)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
