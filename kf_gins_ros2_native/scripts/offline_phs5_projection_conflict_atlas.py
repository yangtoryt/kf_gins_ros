#!/usr/bin/env python3
"""Build a segment-level atlas for PHS5 projection conflicts.

The input is the row-level measurement-geometry table produced from existing
PHS5 logs.  This script does not run PX4, Gazebo, MAVROS, QGC, RViz2, or any
estimator code; it only groups already joined offline rows into short segments.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


DEFAULT_INPUT = (
    Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
    / "phs5_measurement_geometry_selector_repeat2_2026-05-10"
    / "geometry_row_metrics.csv"
)
DEFAULT_OUT = (
    Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
    / "phs5_projection_conflict_atlas_2026-05-10"
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
ALPHAS = (0.0, 0.25, 0.5, 0.75, 1.0)


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


def stddev(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    if len(vals) < 2:
        return math.nan
    avg = sum(vals) / len(vals)
    return math.sqrt(sum((value - avg) ** 2 for value in vals) / (len(vals) - 1))


def rmse(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


def angle_deg(east: float, north: float) -> float:
    if not finite(east) or not finite(north) or (abs(east) < 1e-9 and abs(north) < 1e-9):
        return math.nan
    return math.degrees(math.atan2(east, north))


def angle_diff_deg(a: float, b: float) -> float:
    if not finite(a) or not finite(b):
        return math.nan
    diff = (a - b + 180.0) % 360.0 - 180.0
    return diff


def abs_angle_diff_deg(a: float, b: float) -> float:
    diff = angle_diff_deg(a, b)
    return abs(diff) if finite(diff) else math.nan


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


def row_error(row: dict[str, object], alpha: float) -> float:
    raw_x = to_float(row.get("raw_iekf_x_m"))
    raw_y = to_float(row.get("raw_iekf_y_m"))
    px4_x = to_float(row.get("px4_iekf_x_m"))
    px4_y = to_float(row.get("px4_iekf_y_m"))
    gt_x = to_float(row.get("gt_x_m"))
    gt_y = to_float(row.get("gt_y_m"))
    x = raw_x + alpha * (px4_x - raw_x)
    y = raw_y + alpha * (px4_y - raw_y)
    return math.hypot(x - gt_x, y - gt_y)


def load_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        t = to_float(raw.get("time_since_arm_sec"))
        if not (40.0 <= t < 180.0):
            continue
        row: dict[str, object] = dict(raw)
        row["time_since_arm_sec"] = t
        for key in (
            "speed_mps",
            "core_speed_mps",
            "gyro_deg_s",
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
            "core_gnss_cross_velocity_m",
            "latest_gnss_residual_h_m",
            "latest_gnss_residual_along_velocity_m",
            "latest_gnss_residual_cross_velocity_m",
            "latest_dx_pos_h_m",
            "latest_dx_pos_along_velocity_m",
            "latest_dx_pos_cross_velocity_m",
            "latest_dx_over_residual_h",
            "raw_error_along_velocity_m",
            "raw_error_cross_velocity_m",
            "projection_delta_along_velocity_m",
            "projection_delta_cross_velocity_m",
            "turning_now",
            "post_turn_context",
            "armed_cruise_context",
        ):
            row[key] = to_float(raw.get(key))
        rows.append(row)
    rows.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return rows


def segment_key(row: dict[str, object], segment_sec: float) -> tuple[str, int]:
    t = to_float(row.get("time_since_arm_sec"))
    return str(row.get("run")), int(math.floor((t - 40.0) / segment_sec))


def window_label_for_segment(start: float, end: float) -> str:
    labels = [label for label, ws, we in WINDOWS if start < we and end > ws]
    return ",".join(labels)


def make_segment(run: str, index: int, rows: list[dict[str, object]], segment_sec: float) -> dict[str, object]:
    rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    start = 40.0 + index * segment_sec
    end = start + segment_sec
    first = rows[0]
    last = rows[-1]

    raw_start_x = to_float(first.get("raw_iekf_x_m"))
    raw_start_y = to_float(first.get("raw_iekf_y_m"))
    raw_end_x = to_float(last.get("raw_iekf_x_m"))
    raw_end_y = to_float(last.get("raw_iekf_y_m"))
    gt_start_x = to_float(first.get("gt_x_m"))
    gt_start_y = to_float(first.get("gt_y_m"))
    gt_end_x = to_float(last.get("gt_x_m"))
    gt_end_y = to_float(last.get("gt_y_m"))
    raw_mid_x = mean(to_float(row.get("raw_iekf_x_m")) for row in rows)
    raw_mid_y = mean(to_float(row.get("raw_iekf_y_m")) for row in rows)
    gt_mid_x = mean(to_float(row.get("gt_x_m")) for row in rows)
    gt_mid_y = mean(to_float(row.get("gt_y_m")) for row in rows)
    proj_delta_e = mean(to_float(row.get("px4_iekf_x_m")) - to_float(row.get("raw_iekf_x_m")) for row in rows)
    proj_delta_n = mean(to_float(row.get("px4_iekf_y_m")) - to_float(row.get("raw_iekf_y_m")) for row in rows)

    alpha_errors = {alpha: rmse(row_error(row, alpha) for row in rows) for alpha in ALPHAS}
    best_alpha = min(ALPHAS, key=lambda alpha: alpha_errors[alpha])
    raw_rmse = alpha_errors[0.0]
    px4_rmse = alpha_errors[1.0]
    ekf2_rmse = rmse(to_float(row.get("ekf2_error_m")) for row in rows)

    raw_leg_heading = angle_deg(raw_end_x - raw_start_x, raw_end_y - raw_start_y)
    gt_leg_heading = angle_deg(gt_end_x - gt_start_x, gt_end_y - gt_start_y)
    projection_heading = angle_deg(proj_delta_e, proj_delta_n)

    segment: dict[str, object] = {
        "run": run,
        "segment_index": index,
        "start_sec": start,
        "end_sec": end,
        "window_overlap": window_label_for_segment(start, end),
        "rows": len(rows),
        "raw_rmse_m": raw_rmse,
        "px4_rmse_m": px4_rmse,
        "ekf2_rmse_m": ekf2_rmse,
        "px4_minus_raw_rmse_m": px4_rmse - raw_rmse,
        "raw_minus_ekf2_rmse_m": raw_rmse - ekf2_rmse,
        "px4_minus_ekf2_rmse_m": px4_rmse - ekf2_rmse,
        "oracle_alpha": best_alpha,
        "oracle_rmse_m": alpha_errors[best_alpha],
        "oracle_minus_raw_rmse_m": alpha_errors[best_alpha] - raw_rmse,
        "oracle_minus_ekf2_rmse_m": alpha_errors[best_alpha] - ekf2_rmse,
        "oracle_mode": "raw" if best_alpha == 0.0 else "px4" if best_alpha == 1.0 else f"blend_{best_alpha:.2f}",
        "projection_class": "help" if raw_rmse - px4_rmse > 0.02 else "harm" if px4_rmse - raw_rmse > 0.02 else "neutral",
        "projection_benefit_rmse_m": raw_rmse - px4_rmse,
        "raw_mid_x_m": raw_mid_x,
        "raw_mid_y_m": raw_mid_y,
        "raw_dist_home_m": math.hypot(raw_mid_x, raw_mid_y),
        "raw_bearing_home_deg": angle_deg(raw_mid_x, raw_mid_y),
        "raw_leg_heading_deg": raw_leg_heading,
        "raw_leg_length_m": math.hypot(raw_end_x - raw_start_x, raw_end_y - raw_start_y),
        "gt_mid_x_m": gt_mid_x,
        "gt_mid_y_m": gt_mid_y,
        "gt_dist_home_m": math.hypot(gt_mid_x, gt_mid_y),
        "gt_bearing_home_deg": angle_deg(gt_mid_x, gt_mid_y),
        "gt_leg_heading_deg": gt_leg_heading,
        "gt_leg_length_m": math.hypot(gt_end_x - gt_start_x, gt_end_y - gt_start_y),
        "projection_delta_e_m": proj_delta_e,
        "projection_delta_n_m": proj_delta_n,
        "projection_delta_h_m": math.hypot(proj_delta_e, proj_delta_n),
        "projection_delta_heading_deg": projection_heading,
        "raw_leg_vs_projection_abs_deg": abs_angle_diff_deg(raw_leg_heading, projection_heading),
        "gt_leg_vs_projection_abs_deg": abs_angle_diff_deg(gt_leg_heading, projection_heading),
    }

    for field in FEATURE_FIELDS:
        segment[f"{field}_mean"] = mean(to_float(row.get(field)) for row in rows)
        segment[f"{field}_std"] = stddev(to_float(row.get(field)) for row in rows)
    return segment


FEATURE_FIELDS = [
    "stamp_lag_sec",
    "core_age_sec",
    "gnss_source_age_sec",
    "state_core_gnss_diff_h_m",
    "core_gnss_along_velocity_m",
    "core_gnss_cross_velocity_m",
    "latest_gnss_residual_h_m",
    "latest_gnss_residual_along_velocity_m",
    "latest_gnss_residual_cross_velocity_m",
    "latest_dx_pos_h_m",
    "latest_dx_pos_along_velocity_m",
    "latest_dx_pos_cross_velocity_m",
    "latest_dx_over_residual_h",
    "raw_error_along_velocity_m",
    "raw_error_cross_velocity_m",
    "projection_delta_along_velocity_m",
    "projection_delta_cross_velocity_m",
    "speed_mps",
    "core_speed_mps",
    "gyro_deg_s",
    "turning_now",
    "post_turn_context",
    "armed_cruise_context",
]


SELECTOR_FEATURES = [
    "stamp_lag_sec_mean",
    "core_age_sec_mean",
    "gnss_source_age_sec_mean",
    "state_core_gnss_diff_h_m_mean",
    "core_gnss_along_velocity_m_mean",
    "core_gnss_cross_velocity_m_mean",
    "latest_gnss_residual_h_m_mean",
    "latest_gnss_residual_along_velocity_m_mean",
    "latest_dx_over_residual_h_mean",
    "projection_delta_along_velocity_m_mean",
    "projection_delta_cross_velocity_m_mean",
    "projection_delta_h_m",
    "projection_delta_heading_deg",
    "raw_leg_vs_projection_abs_deg",
    "raw_dist_home_m",
    "raw_bearing_home_deg",
    "raw_leg_heading_deg",
    "raw_leg_length_m",
    "speed_mps_mean",
    "core_speed_mps_mean",
    "gyro_deg_s_mean",
    "turning_now_mean",
    "post_turn_context_mean",
]


def build_segments(rows: list[dict[str, object]], segment_sec: float) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in rows:
        grouped.setdefault(segment_key(row, segment_sec), []).append(row)
    segments = [
        make_segment(run, index, segment_rows, segment_sec)
        for (run, index), segment_rows in sorted(grouped.items())
        if len(segment_rows) >= 3
    ]
    return segments


def segment_in_window(segment: dict[str, object], start: float, end: float) -> bool:
    return to_float(segment.get("start_sec")) < end and to_float(segment.get("end_sec")) > start


def segment_rows_for_window(
    segments: Sequence[dict[str, object]],
    run: str,
    window: str,
) -> list[dict[str, object]]:
    start, end = next((ws, we) for label, ws, we in WINDOWS if label == window)
    return [segment for segment in segments if segment.get("run") == run and segment_in_window(segment, start, end)]


def aggregate_segment_scores(
    segments: Sequence[dict[str, object]],
    mode: str,
) -> dict[str, object]:
    row_count = sum(int(to_float(segment.get("rows"), 0.0)) for segment in segments)
    if row_count <= 0:
        return {
            "rows": 0,
            "raw_rmse_m": math.nan,
            "px4_rmse_m": math.nan,
            "ekf2_rmse_m": math.nan,
            "candidate_rmse_m": math.nan,
            "candidate_minus_raw_m": math.nan,
            "candidate_minus_ekf2_m": math.nan,
        }

    def weighted_rmse(field: str) -> float:
        total = 0.0
        count = 0
        for segment in segments:
            value = to_float(segment.get(field))
            rows = int(to_float(segment.get("rows"), 0.0))
            if finite(value) and rows > 0:
                total += value * value * rows
                count += rows
        return math.sqrt(total / count) if count else math.nan

    raw = weighted_rmse("raw_rmse_m")
    px4 = weighted_rmse("px4_rmse_m")
    ekf2 = weighted_rmse("ekf2_rmse_m")
    candidate_field = {
        "raw": "raw_rmse_m",
        "px4": "px4_rmse_m",
        "oracle": "oracle_rmse_m",
    }[mode]
    candidate = weighted_rmse(candidate_field)
    return {
        "rows": row_count,
        "raw_rmse_m": raw,
        "px4_rmse_m": px4,
        "ekf2_rmse_m": ekf2,
        "candidate_rmse_m": candidate,
        "candidate_minus_raw_m": candidate - raw,
        "candidate_minus_ekf2_m": candidate - ekf2,
    }


def oracle_window_summary(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for run in RUNS:
        for window, _start, _end in WINDOWS:
            subset = segment_rows_for_window(segments, run, window)
            for mode in ("raw", "px4", "oracle"):
                row = aggregate_segment_scores(subset, mode)
                row["run"] = run
                row["window"] = window
                row["mode"] = mode
                out.append(row)
    return out


def feature_separation(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    help_segments = [segment for segment in segments if segment.get("projection_class") == "help"]
    harm_segments = [segment for segment in segments if segment.get("projection_class") == "harm"]
    out: list[dict[str, object]] = []
    for feature in SELECTOR_FEATURES:
        help_values = [to_float(segment.get(feature)) for segment in help_segments]
        harm_values = [to_float(segment.get(feature)) for segment in harm_segments]
        help_mean = mean(help_values)
        harm_mean = mean(harm_values)
        pooled = math.hypot(stddev(help_values), stddev(harm_values))
        score = abs(help_mean - harm_mean) / pooled if finite(pooled) and pooled > 1e-9 else math.nan
        out.append(
            {
                "feature": feature,
                "help_count": len([value for value in help_values if finite(value)]),
                "harm_count": len([value for value in harm_values if finite(value)]),
                "help_mean": help_mean,
                "harm_mean": harm_mean,
                "mean_diff_help_minus_harm": help_mean - harm_mean,
                "separation_score": score,
            }
        )
    out.sort(key=lambda row: to_float(row.get("separation_score")), reverse=True)
    return out


@dataclass(frozen=True)
class Condition:
    feature: str
    op: str
    threshold: float

    def matches(self, segment: dict[str, object]) -> bool:
        value = to_float(segment.get(self.feature))
        if not finite(value):
            return False
        if self.op == "ge":
            return value >= self.threshold
        if self.op == "le":
            return value <= self.threshold
        raise ValueError(self.op)

    def label(self) -> str:
        safe = self.feature.replace("_mean", "").replace("_deg", "deg").replace("_m", "m")
        return f"{safe}_{self.op}_{self.threshold:.3f}"


@dataclass(frozen=True)
class SegmentCandidate:
    name: str
    conditions: tuple[Condition, ...]

    def active(self, segment: dict[str, object]) -> bool:
        return all(condition.matches(segment) for condition in self.conditions)


def candidate_label(conditions: Sequence[Condition]) -> str:
    if not conditions:
        return "always"
    return "__".join(condition.label() for condition in conditions)


def quantile_thresholds(values: list[float]) -> list[float]:
    vals = sorted(value for value in values if finite(value))
    if len(vals) < 8:
        return []
    thresholds: list[float] = []
    for q in (0.2, 0.35, 0.5, 0.65, 0.8):
        idx = min(len(vals) - 1, max(0, int(round(q * (len(vals) - 1)))))
        thresholds.append(vals[idx])
    unique: list[float] = []
    for value in thresholds:
        if not any(abs(value - seen) < 1e-9 for seen in unique):
            unique.append(value)
    return unique


def build_candidates(segments: Sequence[dict[str, object]]) -> list[SegmentCandidate]:
    base_conditions: list[Condition] = []
    for feature in SELECTOR_FEATURES:
        thresholds = quantile_thresholds([to_float(segment.get(feature)) for segment in segments])
        for threshold in thresholds:
            base_conditions.append(Condition(feature, "ge", threshold))
            base_conditions.append(Condition(feature, "le", threshold))

    candidates = [SegmentCandidate("always_px4", tuple())]
    candidates.extend(
        SegmentCandidate(candidate_label((condition,)), (condition,))
        for condition in base_conditions
    )

    # Pair conditions are diagnostic.  Keep the count bounded by using the most
    # separated single features; otherwise a tiny segment set can be overfit.
    top_features = {
        row["feature"]
        for row in feature_separation(segments)[:8]
        if finite(row.get("separation_score"))
    }
    top_conditions = [condition for condition in base_conditions if condition.feature in top_features]
    for i, first in enumerate(top_conditions):
        for second in top_conditions[i + 1 :]:
            if first.feature == second.feature:
                continue
            conditions = (first, second)
            candidates.append(SegmentCandidate(candidate_label(conditions), conditions))
    return candidates


def candidate_window_scores(
    segments: Sequence[dict[str, object]],
    candidate: SegmentCandidate,
) -> dict[tuple[str, str], dict[str, object]]:
    out: dict[tuple[str, str], dict[str, object]] = {}
    for run in RUNS:
        for window, _start, _end in WINDOWS:
            subset = segment_rows_for_window(segments, run, window)
            out[(run, window)] = aggregate_candidate_scores(subset, candidate)
    return out


def aggregate_candidate_scores(
    segments: Sequence[dict[str, object]],
    candidate: SegmentCandidate,
) -> dict[str, object]:
    row_count = sum(int(to_float(segment.get("rows"), 0.0)) for segment in segments)
    if row_count <= 0:
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
        rows = int(to_float(segment.get("rows"), 0.0))
        if rows <= 0:
            continue
        active = candidate.active(segment)
        active_rows += rows if active else 0
        raw = to_float(segment.get("raw_rmse_m"))
        px4 = to_float(segment.get("px4_rmse_m"))
        ekf2 = to_float(segment.get("ekf2_rmse_m"))
        chosen = px4 if active else raw
        totals["raw"] += raw * raw * rows
        totals["px4"] += px4 * px4 * rows
        totals["ekf2"] += ekf2 * ekf2 * rows
        totals["candidate"] += chosen * chosen * rows
    raw_rmse = math.sqrt(totals["raw"] / row_count)
    px4_rmse = math.sqrt(totals["px4"] / row_count)
    ekf2_rmse = math.sqrt(totals["ekf2"] / row_count)
    candidate_rmse = math.sqrt(totals["candidate"] / row_count)
    return {
        "rows": row_count,
        "active_frac": active_rows / row_count,
        "raw_rmse_m": raw_rmse,
        "px4_rmse_m": px4_rmse,
        "ekf2_rmse_m": ekf2_rmse,
        "candidate_rmse_m": candidate_rmse,
        "candidate_minus_raw_m": candidate_rmse - raw_rmse,
        "candidate_minus_ekf2_m": candidate_rmse - ekf2_rmse,
    }


def summarize_candidate(
    segments: Sequence[dict[str, object]],
    candidate: SegmentCandidate,
) -> dict[str, object]:
    scores = candidate_window_scores(segments, candidate)
    positive_regs = [
        to_float(scores[(run, "main_40_180")]["candidate_minus_raw_m"])
        for run in POSITIVE_RUNS
    ]
    positive_deltas = [
        to_float(scores[(run, "main_40_180")]["candidate_minus_ekf2_m"])
        for run in POSITIVE_RUNS
    ]
    sg11_main = scores[("shortgen11_repeat2", "main_40_180")]
    sg11_140 = scores[("shortgen11_repeat2", "140_160")]
    protected = max(positive_regs) <= 0.02 and max(positive_deltas) < 0.02
    strict_repair_2cm = (
        protected
        and to_float(sg11_main["candidate_minus_ekf2_m"]) <= 0.02
        and to_float(sg11_140["candidate_minus_ekf2_m"]) <= 0.02
    )
    return {
        "candidate": candidate.name,
        "condition_count": len(candidate.conditions),
        "sg11_main_raw_rmse_m": sg11_main["raw_rmse_m"],
        "sg11_main_candidate_rmse_m": sg11_main["candidate_rmse_m"],
        "sg11_main_ekf2_rmse_m": sg11_main["ekf2_rmse_m"],
        "sg11_main_candidate_minus_ekf2_m": sg11_main["candidate_minus_ekf2_m"],
        "sg11_main_active_frac": sg11_main["active_frac"],
        "sg11_140_160_raw_rmse_m": sg11_140["raw_rmse_m"],
        "sg11_140_160_candidate_rmse_m": sg11_140["candidate_rmse_m"],
        "sg11_140_160_ekf2_rmse_m": sg11_140["ekf2_rmse_m"],
        "sg11_140_160_candidate_minus_ekf2_m": sg11_140["candidate_minus_ekf2_m"],
        "sg11_140_160_active_frac": sg11_140["active_frac"],
        "positive_max_main_regress_m": max(positive_regs),
        "positive_mean_main_regress_m": mean(positive_regs),
        "positive_max_candidate_minus_ekf2_m": max(positive_deltas),
        "positive_all_protected": int(protected),
        "strict_repair_2cm": int(strict_repair_2cm),
    }


def candidate_window_metrics(
    segments: Sequence[dict[str, object]],
    candidate: SegmentCandidate,
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    scores = candidate_window_scores(segments, candidate)
    for (run, window), score in scores.items():
        row = dict(score)
        row["candidate"] = candidate.name
        row["run"] = run
        row["window"] = window
        out.append(row)
    return out


def choose_top(summary_rows: list[dict[str, object]], protected: bool) -> list[dict[str, object]]:
    rows = [
        row
        for row in summary_rows
        if not protected or to_float(row.get("positive_all_protected"), 0.0) > 0.5
    ]
    rows.sort(
        key=lambda row: (
            to_float(row.get("sg11_140_160_candidate_minus_ekf2_m")),
            to_float(row.get("positive_max_main_regress_m")),
            to_float(row.get("sg11_main_candidate_minus_ekf2_m")),
        )
    )
    return rows[:10]


def write_conflict_pairs(segments: list[dict[str, object]], out_dir: Path) -> list[dict[str, object]]:
    focus = [
        segment
        for segment in segments
        if (
            segment["run"] == "shortgen11_repeat2"
            and 140.0 <= to_float(segment.get("start_sec")) < 160.0
        )
        or (
            segment["run"] == "shortgen04_hld1a_phs5"
            and to_float(segment.get("projection_benefit_rmse_m")) < -0.10
        )
    ]
    focus.sort(key=lambda row: (str(row.get("run")), to_float(row.get("start_sec"))))
    write_csv(out_dir / "projection_conflict_focus_segments.csv", focus)
    return focus


def row_by_mode(rows: list[dict[str, object]], run: str, window: str, mode: str) -> dict[str, object]:
    for row in rows:
        if row["run"] == run and row["window"] == window and row["mode"] == mode:
            return row
    return {}


def summary_table(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["candidate"],
            row["condition_count"],
            fmt(row["sg11_main_candidate_rmse_m"]),
            fmt(row["sg11_main_candidate_minus_ekf2_m"]),
            fmt(row["sg11_140_160_candidate_rmse_m"]),
            fmt(row["sg11_140_160_candidate_minus_ekf2_m"]),
            fmt(row["positive_max_main_regress_m"]),
            row["positive_all_protected"],
            row["strict_repair_2cm"],
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    input_path: Path,
    segment_sec: float,
    segments: list[dict[str, object]],
    oracle_rows: list[dict[str, object]],
    separation_rows: list[dict[str, object]],
    candidate_summary_rows: list[dict[str, object]],
    selected_metrics: list[dict[str, object]],
    focus_segments: list[dict[str, object]],
) -> None:
    strict_repairs = [
        row for row in candidate_summary_rows if to_float(row.get("strict_repair_2cm"), 0.0) > 0.5
    ]
    strict_repairs.sort(
        key=lambda row: (
            to_float(row.get("sg11_140_160_candidate_minus_ekf2_m")),
            to_float(row.get("positive_max_main_regress_m")),
        )
    )
    protected_top = choose_top(candidate_summary_rows, protected=True)
    relaxed_top = choose_top(candidate_summary_rows, protected=False)

    sg11_140_raw = row_by_mode(oracle_rows, "shortgen11_repeat2", "140_160", "raw")
    sg11_140_px4 = row_by_mode(oracle_rows, "shortgen11_repeat2", "140_160", "px4")
    sg11_140_oracle = row_by_mode(oracle_rows, "shortgen11_repeat2", "140_160", "oracle")
    sg04_main_raw = row_by_mode(oracle_rows, "shortgen04_hld1a_phs5", "main_40_180", "raw")
    sg04_main_px4 = row_by_mode(oracle_rows, "shortgen04_hld1a_phs5", "main_40_180", "px4")
    sg04_main_oracle = row_by_mode(oracle_rows, "shortgen04_hld1a_phs5", "main_40_180", "oracle")

    class_counts: dict[str, int] = {}
    for segment in segments:
        key = str(segment.get("projection_class"))
        class_counts[key] = class_counts.get(key, 0) + 1

    top_features = [
        [
            row["feature"],
            fmt(row["help_mean"]),
            fmt(row["harm_mean"]),
            fmt(row["mean_diff_help_minus_harm"]),
            fmt(row["separation_score"]),
        ]
        for row in separation_rows[:10]
    ]
    focus_table = [
        [
            row["run"],
            fmt(row["start_sec"], 1),
            row["projection_class"],
            fmt(row["projection_benefit_rmse_m"]),
            fmt(row["raw_rmse_m"]),
            fmt(row["px4_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            row["oracle_mode"],
            fmt(row["stamp_lag_sec_mean"]),
            fmt(row["raw_dist_home_m"]),
            fmt(row["raw_leg_heading_deg"]),
            fmt(row["raw_leg_vs_projection_abs_deg"]),
        ]
        for row in focus_segments[:24]
    ]
    metric_table = [
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
        for row in selected_metrics
        if (
            (row["run"] == "shortgen11_repeat2" and row["window"] in {"main_40_180", "140_160"})
            or (row["run"] != "shortgen11_repeat2" and row["window"] == "main_40_180")
        )
    ]

    lines = [
        "# PHS5 projection conflict atlas",
        "",
        "Date: 2026-05-10",
        "",
        "Pure offline analysis using existing PHS5-derived row metrics only. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator code change was run.",
        "",
        "## Scope",
        "",
        f"- Input: `{input_path}`",
        f"- Segment length: `{segment_sec:.1f}` s; segments: `{len(segments)}`.",
        "- The oracle columns use groundtruth for diagnosis only; they are an upper bound, not a deployable selector.",
        "- Candidate selectors use segment-level timing, update geometry, frame projection geometry, and route-geometry proxies from the existing row table.",
        "",
        "## Oracle Boundary",
        "",
        markdown_table(
            ["case", "raw", "px4", "oracle", "EKF2"],
            [
                [
                    "shortgen11 140-160",
                    fmt(sg11_140_raw.get("candidate_rmse_m")),
                    fmt(sg11_140_px4.get("candidate_rmse_m")),
                    fmt(sg11_140_oracle.get("candidate_rmse_m")),
                    fmt(sg11_140_raw.get("ekf2_rmse_m")),
                ],
                [
                    "shortgen04 main",
                    fmt(sg04_main_raw.get("candidate_rmse_m")),
                    fmt(sg04_main_px4.get("candidate_rmse_m")),
                    fmt(sg04_main_oracle.get("candidate_rmse_m")),
                    fmt(sg04_main_raw.get("ekf2_rmse_m")),
                ],
            ],
        ),
        "",
        f"Projection segment classes: help `{class_counts.get('help', 0)}`, neutral `{class_counts.get('neutral', 0)}`, harm `{class_counts.get('harm', 0)}`.",
        "",
        "## Feature Separation",
        "",
        markdown_table(
            ["feature", "help mean", "harm mean", "diff", "score"],
            top_features,
        ),
        "",
        "## Focus Conflict Segments",
        "",
        markdown_table(
            [
                "run",
                "start",
                "class",
                "benefit",
                "raw",
                "px4",
                "EKF2",
                "oracle",
                "lag",
                "raw dist",
                "raw heading",
                "leg/proj abs",
            ],
            focus_table,
        ),
        "",
        "## Top Protected Segment Selectors",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "sg11 main",
                "sg11 main-EKF2",
                "sg11 140-160",
                "sg11 140-EKF2",
                "pos max regress",
                "protected",
                "repair 2cm",
            ],
            summary_table(protected_top[:8]),
        ),
        "",
        "## Top Relaxed Segment Selectors",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "sg11 main",
                "sg11 main-EKF2",
                "sg11 140-160",
                "sg11 140-EKF2",
                "pos max regress",
                "protected",
                "repair 2cm",
            ],
            summary_table(relaxed_top[:8]),
        ),
        "",
        "## Selected Window Metrics",
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
            metric_table,
        ),
        "",
        "## Readout",
        "",
    ]
    if strict_repairs:
        best = strict_repairs[0]
        lines.append(
            f"- A protected segment selector exists under this diagnostic grid: `{best['candidate']}` repairs shortgen11 `140_160` to `{fmt(best['sg11_140_160_candidate_rmse_m'])}` m with positive max regression `{fmt(best['positive_max_main_regress_m'])}` m. This is a stronger offline candidate than the row-level lag gate because segment smoothing can apply projection to low-lag rows inside a high-lag segment, but it is not a mechanism success until segment-length sensitivity and new-route holdouts confirm it."
        )
    else:
        lines.append(
            "- No protected segment selector in this grid repairs shortgen11 `140_160` to the 0.02 m EKF2 tolerance. The oracle still shows the conflict is theoretically switchable if a better classifier signal is found."
        )
    lines.extend(
        [
            f"- Segment oracle confirms the upper bound is large: shortgen11 `140_160` can move from raw `{fmt(sg11_140_raw.get('candidate_rmse_m'))}` to oracle `{fmt(sg11_140_oracle.get('candidate_rmse_m'))}` m, while shortgen04 main can stay near oracle `{fmt(sg04_main_oracle.get('candidate_rmse_m'))}` m instead of global PX4 `{fmt(sg04_main_px4.get('candidate_rmse_m'))}` m.",
            "- The feature-separation table is the next evidence source: if the top separators are mostly absolute route geometry, the selector is likely route-overfit; if frame/update geometry dominates, it may justify a default-off online diagnostic mechanism.",
            "",
            "Generated files:",
            f"- `{out_dir / 'segment_projection_atlas.csv'}`",
            f"- `{out_dir / 'oracle_window_summary.csv'}`",
            f"- `{out_dir / 'projection_feature_separation.csv'}`",
            f"- `{out_dir / 'segment_selector_candidate_summary.csv'}`",
            f"- `{out_dir / 'selected_segment_selector_window_metrics.csv'}`",
            f"- `{out_dir / 'projection_conflict_focus_segments.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    parser.add_argument("--segment-sec", type=float, default=5.0)
    args = parser.parse_args()

    input_path = Path(args.input)
    out_dir = Path(args.out_dir)
    rows = load_rows(input_path)
    segments = build_segments(rows, args.segment_sec)
    write_csv(out_dir / "segment_projection_atlas.csv", segments)

    oracle_rows = oracle_window_summary(segments)
    write_csv(out_dir / "oracle_window_summary.csv", oracle_rows)

    separation_rows = feature_separation(segments)
    write_csv(out_dir / "projection_feature_separation.csv", separation_rows)

    candidates = build_candidates(segments)
    summary_rows = [summarize_candidate(segments, candidate) for candidate in candidates]
    write_csv(out_dir / "segment_selector_candidate_summary.csv", summary_rows)

    selected_names = {
        *(str(row["candidate"]) for row in choose_top(summary_rows, protected=True)[:3]),
        *(str(row["candidate"]) for row in choose_top(summary_rows, protected=False)[:3]),
        "always_px4",
    }
    selected_candidates = [candidate for candidate in candidates if candidate.name in selected_names]
    selected_metrics: list[dict[str, object]] = []
    for candidate in selected_candidates:
        selected_metrics.extend(candidate_window_metrics(segments, candidate))
    write_csv(out_dir / "selected_segment_selector_window_metrics.csv", selected_metrics)

    focus_segments = write_conflict_pairs(segments, out_dir)
    write_report(
        out_dir,
        input_path,
        args.segment_sec,
        segments,
        oracle_rows,
        separation_rows,
        summary_rows,
        selected_metrics,
        focus_segments,
    )
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
