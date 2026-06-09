#!/usr/bin/env python3
"""Diagnose residual shortgen02 errors after PX4-compatible projection."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path

from offline_cross_route_failure_atlas import (
    finite,
    markdown_table,
    mean,
    percentile,
    to_float,
    write_csv,
)


SUMMARY_WINDOWS = [
    ("120-124p5", 120.0, 124.5),
    ("151-154p5", 151.0, 154.5),
    ("151-160", 151.0, 160.0),
    ("171-178p5", 171.0, 178.5),
    ("120-180", 120.0, 180.0),
]

ALIGN_WINDOWS = [
    ("early_0_20", 0.0, 20.0),
    ("target_120_180", 120.0, 180.0),
    ("cluster_151_160", 151.0, 160.0),
    ("late_171_178p5", 171.0, 178.5),
]

UPDATE_CONTEXT_WINDOWS = [
    (119.5, 124.5),
    (149.5, 154.5),
    (171.0, 178.5),
]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def bool01(value: object) -> bool:
    text = str(value).strip().lower()
    return text in {"1", "true", "yes"} or to_float(value, 0.0) > 0.5


def norm2(x: float, y: float) -> float:
    return math.hypot(x, y)


def context_label(row: dict[str, object]) -> str:
    if bool01(row.get("turning_now")):
        return "turning"
    if bool01(row.get("post_turn_context")):
        return "post_turn"
    if bool01(row.get("armed_cruise_context")):
        return "armed_cruise"
    if bool01(row.get("terminal_descent_context")):
        return "terminal"
    return "none"


def is_armed(row: dict[str, object]) -> bool:
    return to_float(row.get("mavros_armed"), 0.0) > 0.5


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return finite(t) and start <= t < end


def load_sorted(path: Path, time_key: str) -> tuple[list[dict[str, object]], list[float]]:
    rows: list[dict[str, object]] = read_csv(path)
    rows.sort(key=lambda row: to_float(row.get(time_key)))
    return rows, [to_float(row.get(time_key)) for row in rows]


def nearest(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt_sec: float,
) -> tuple[dict[str, object] | None, float]:
    if not rows or not finite(t):
        return None, math.nan
    idx = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - t), rows[pos]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    return (row, dt) if dt <= max_dt_sec else (None, dt)


def prev_next(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
) -> tuple[dict[str, object] | None, dict[str, object] | None]:
    idx = bisect.bisect_right(times, t)
    prev_row = rows[idx - 1] if 0 <= idx - 1 < len(rows) else None
    next_row = rows[idx] if 0 <= idx < len(rows) else None
    return prev_row, next_row


def load_projection_rows(run_dir: Path, mode: str) -> tuple[list[dict[str, object]], list[float]]:
    return load_sorted(
        run_dir / "offline_groundtruth_projection_modes" / mode / "groundtruth_joined.csv",
        "pair_ros_time_sec",
    )


def add_update_interval_context(
    row: dict[str, object],
    update_rows: list[dict[str, object]],
    update_times: list[float],
) -> None:
    ros_t = to_float(row.get("ros_time_sec"))
    nearest_update, nearest_dt = nearest(update_rows, update_times, ros_t, 0.25)
    prev_update, next_update = prev_next(update_rows, update_times, ros_t)

    row["nearest_position_update_dt_sec"] = nearest_dt
    if nearest_update is not None:
        row["nearest_update_time_since_arm_sec"] = nearest_update.get("time_since_arm_sec")
        row["nearest_update_context"] = context_label(nearest_update)
        row["nearest_update_residual_h_m"] = nearest_update.get("gnss_residual_h_m")
        row["nearest_update_dx_over_residual_h"] = nearest_update.get("dx_over_residual_h")
        row["nearest_update_pressure_active"] = nearest_update.get("phase_error_memory_pressure_active")
        row["nearest_update_candidate_active"] = nearest_update.get("phase_error_memory_candidate_active")
        row["nearest_update_reason"] = nearest_update.get("phase_error_memory_reason")
    else:
        row["nearest_update_time_since_arm_sec"] = math.nan
        row["nearest_update_context"] = ""
        row["nearest_update_residual_h_m"] = math.nan
        row["nearest_update_dx_over_residual_h"] = math.nan
        row["nearest_update_pressure_active"] = ""
        row["nearest_update_candidate_active"] = ""
        row["nearest_update_reason"] = ""

    if prev_update is not None:
        row["prev_update_time_since_arm_sec"] = prev_update.get("time_since_arm_sec")
    else:
        row["prev_update_time_since_arm_sec"] = math.nan
    if next_update is not None:
        row["next_update_time_since_arm_sec"] = next_update.get("time_since_arm_sec")
    else:
        row["next_update_time_since_arm_sec"] = math.nan

    if prev_update is None or next_update is None:
        row["inter_update_dt_sec"] = math.nan
        row["inter_update_phase_0_1"] = math.nan
        return
    prev_t = to_float(prev_update.get("ros_time_sec"))
    next_t = to_float(next_update.get("ros_time_sec"))
    dt = next_t - prev_t
    row["inter_update_dt_sec"] = dt
    row["inter_update_phase_0_1"] = (ros_t - prev_t) / dt if dt > 1e-12 else math.nan


def build_pair_rows(run_dir: Path) -> list[dict[str, object]]:
    current_rows, current_times = load_projection_rows(run_dir, "gps_fit_all")
    px4_rows, _ = load_projection_rows(run_dir, "px4_sphere_anisotropic")
    update_rows, update_times = load_sorted(
        run_dir / "phase_error_memory_diag" / "position_update_rows_with_pmem.csv",
        "ros_time_sec",
    )

    out: list[dict[str, object]] = []
    for px4 in px4_rows:
        ros_t = to_float(px4.get("pair_ros_time_sec"))
        current, current_dt = nearest(current_rows, current_times, ros_t, 0.025)
        if current is None:
            continue
        row: dict[str, object] = {
            "time_since_arm_sec": to_float(px4.get("time_since_arm_sec")),
            "ros_time_sec": ros_t,
            "current_join_dt_sec": current_dt,
            "context": context_label(px4),
            "turning_now": px4.get("turning_now"),
            "post_turn_context": px4.get("post_turn_context"),
            "armed_cruise_context": px4.get("armed_cruise_context"),
            "mavros_armed": px4.get("mavros_armed"),
            "mavros_mode": px4.get("mavros_mode"),
            "horizontal_speed_mps": px4.get("horizontal_speed_mps"),
            "vertical_speed_mps": px4.get("vertical_speed_mps"),
            "gyro_deg_s": px4.get("gyro_deg_s"),
            "source_yaw_rate_deg_s": px4.get("source_yaw_rate_deg_s"),
            "core_gnss_diff_h_m": px4.get("core_gnss_diff_h_m"),
            "last_position_residual_h_m": px4.get("last_position_residual_h_m"),
            "gt_x_m": px4.get("gt_x_m"),
            "gt_y_m": px4.get("gt_y_m"),
            "ekf2_raw_x_m": px4.get("ekf2_x_m"),
            "ekf2_raw_y_m": px4.get("ekf2_y_m"),
            "iekf_px4_raw_x_m": px4.get("iekf_px4_x_m"),
            "iekf_px4_raw_y_m": px4.get("iekf_px4_y_m"),
            "ekf2_error_h_m": px4.get("ekf2_error_xy_m"),
            "ekf2_error_x_m": px4.get("ekf2_error_x_m"),
            "ekf2_error_y_m": px4.get("ekf2_error_y_m"),
            "current_iekf_error_h_m": current.get("iekf_error_xy_m"),
            "current_iekf_error_x_m": current.get("iekf_error_x_m"),
            "current_iekf_error_y_m": current.get("iekf_error_y_m"),
            "px4_iekf_error_h_m": px4.get("iekf_error_xy_m"),
            "px4_iekf_error_x_m": px4.get("iekf_error_x_m"),
            "px4_iekf_error_y_m": px4.get("iekf_error_y_m"),
        }
        row["current_to_px4_improvement_m"] = (
            to_float(row.get("current_iekf_error_h_m")) - to_float(row.get("px4_iekf_error_h_m"))
        )
        row["px4_minus_ekf2_m"] = to_float(row.get("px4_iekf_error_h_m")) - to_float(row.get("ekf2_error_h_m"))
        add_update_interval_context(row, update_rows, update_times)
        out.append(row)
    return out


def load_timing_rows(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    path = run_dir / "shortgen02_measurement_timing_projection_diag" / "target_projection_timing_rows.csv"
    if not path.exists():
        return [], []
    return load_sorted(path, "ros_time_sec")


def build_target_rows(run_dir: Path, pair_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    pair_rows = sorted(pair_rows, key=lambda row: to_float(row.get("ros_time_sec")))
    pair_times = [to_float(row.get("ros_time_sec")) for row in pair_rows]
    timing_rows, timing_times = load_timing_rows(run_dir)
    target_rows = read_csv(run_dir / "shortgen02_measurement_geometry_diag" / "target_update_geometry_rows.csv")

    out: list[dict[str, object]] = []
    for target in target_rows:
        ros_t = to_float(target.get("ros_time_sec"))
        pair, pair_dt = nearest(pair_rows, pair_times, ros_t, 0.15)
        if pair is None:
            continue
        timing, timing_dt = nearest(timing_rows, timing_times, ros_t, 0.15)
        row: dict[str, object] = {
            "time_since_arm_sec": pair.get("time_since_arm_sec"),
            "ros_time_sec": ros_t,
            "pair_join_dt_sec": pair_dt,
            "timing_join_dt_sec": timing_dt,
            "context": pair.get("context"),
            "ekf2_error_h_m": pair.get("ekf2_error_h_m"),
            "current_iekf_error_h_m": pair.get("current_iekf_error_h_m"),
            "px4_iekf_error_h_m": pair.get("px4_iekf_error_h_m"),
            "current_to_px4_improvement_m": pair.get("current_to_px4_improvement_m"),
            "px4_minus_ekf2_m": pair.get("px4_minus_ekf2_m"),
            "px4_iekf_error_x_m": pair.get("px4_iekf_error_x_m"),
            "px4_iekf_error_y_m": pair.get("px4_iekf_error_y_m"),
            "gt_x_m": pair.get("gt_x_m"),
            "gt_y_m": pair.get("gt_y_m"),
            "ekf2_raw_x_m": pair.get("ekf2_raw_x_m"),
            "ekf2_raw_y_m": pair.get("ekf2_raw_y_m"),
            "iekf_px4_raw_x_m": pair.get("iekf_px4_raw_x_m"),
            "iekf_px4_raw_y_m": pair.get("iekf_px4_raw_y_m"),
            "horizontal_speed_mps": pair.get("horizontal_speed_mps"),
            "vertical_speed_mps": pair.get("vertical_speed_mps"),
            "gyro_deg_s": pair.get("gyro_deg_s"),
            "core_gnss_diff_h_m": pair.get("core_gnss_diff_h_m"),
            "gnss_measurement_error_h_m": target.get("gnss_measurement_error_h_m"),
            "gnss_delta_over_need": target.get("gnss_delta_over_need"),
            "update_over_gnss_delta": target.get("update_over_gnss_delta"),
            "phase_error_memory_pressure_active": target.get("phase_error_memory_pressure_active"),
            "phase_error_memory_candidate_active": target.get("phase_error_memory_candidate_active"),
            "phase_error_memory_reason": target.get("phase_error_memory_reason"),
            "nearest_update_residual_h_m": pair.get("nearest_update_residual_h_m"),
            "nearest_update_dx_over_residual_h": pair.get("nearest_update_dx_over_residual_h"),
            "nearest_update_reason": pair.get("nearest_update_reason"),
        }
        if timing is not None:
            row["core_native_velocity_mismatch_h_mps"] = timing.get("core_native_velocity_mismatch_h_mps")
            row["gps_fit_all_error_h_m"] = timing.get("gps_fit_all_error_h_m")
            row["gps_fit_moving_error_h_m"] = timing.get("gps_fit_moving_error_h_m")
        else:
            row["core_native_velocity_mismatch_h_mps"] = math.nan
            row["gps_fit_all_error_h_m"] = math.nan
            row["gps_fit_moving_error_h_m"] = math.nan
        out.append(row)
    out.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return out


def summarize_subset(label: str, subset: list[dict[str, object]]) -> dict[str, object]:
    current_vals = [to_float(row.get("current_iekf_error_h_m")) for row in subset]
    px4_vals = [to_float(row.get("px4_iekf_error_h_m")) for row in subset]
    ekf2_vals = [to_float(row.get("ekf2_error_h_m")) for row in subset]
    return {
        "window": label,
        "rows": len(subset),
        "ekf2_mean_m": mean(ekf2_vals),
        "current_iekf_mean_m": mean(current_vals),
        "px4_iekf_mean_m": mean(px4_vals),
        "current_minus_ekf2_mean_m": mean(current_vals) - mean(ekf2_vals),
        "px4_minus_ekf2_mean_m": mean(px4_vals) - mean(ekf2_vals),
        "px4_iekf_p95_m": percentile(px4_vals, 95.0),
        "px4_iekf_max_m": max((item for item in px4_vals if finite(item)), default=math.nan),
        "px4_frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in px4_vals),
        "px4_gt_ekf2_frac": mean(
            1.0 if to_float(row.get("px4_iekf_error_h_m")) > to_float(row.get("ekf2_error_h_m")) else 0.0
            for row in subset
        ),
        "current_to_px4_improvement_mean_m": mean(to_float(row.get("current_to_px4_improvement_m")) for row in subset),
        "core_gnss_diff_mean_m": mean(to_float(row.get("core_gnss_diff_h_m")) for row in subset),
        "vertical_speed_abs_mean_mps": mean(abs(to_float(row.get("vertical_speed_mps"))) for row in subset),
    }


def build_window_summary(
    pair_rows: list[dict[str, object]],
    target_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    summary: list[dict[str, object]] = []
    armed_rows = [row for row in pair_rows if is_armed(row)]
    for label, start, end in SUMMARY_WINDOWS:
        summary.append(summarize_subset(label, [row for row in armed_rows if in_window(row, start, end)]))
    summary.append(summarize_subset("original_target_high_rows", target_rows))
    return summary


def build_context_summary(
    target_rows: list[dict[str, object]],
    top_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for sample, rows in (("original_target_high_rows", target_rows), ("top_px4_errors_120_180", top_rows)):
        contexts = sorted({str(row.get("context")) for row in rows})
        for ctx in contexts:
            subset = [row for row in rows if str(row.get("context")) == ctx]
            out.append({
                "sample": sample,
                "context": ctx,
                "rows": len(subset),
                "ekf2_mean_m": mean(to_float(row.get("ekf2_error_h_m")) for row in subset),
                "current_iekf_mean_m": mean(to_float(row.get("current_iekf_error_h_m")) for row in subset),
                "px4_iekf_mean_m": mean(to_float(row.get("px4_iekf_error_h_m")) for row in subset),
                "px4_gt_ekf2_frac": mean(
                    1.0 if to_float(row.get("px4_iekf_error_h_m")) > to_float(row.get("ekf2_error_h_m")) else 0.0
                    for row in subset
                ),
            })
    return out


def alignment_offsets(rows: list[dict[str, object]], start: float, end: float) -> dict[str, object]:
    subset = [row for row in rows if is_armed(row) and in_window(row, start, end)]
    ekf2_dx = [to_float(row.get("gt_x_m")) - to_float(row.get("ekf2_raw_x_m")) for row in subset]
    ekf2_dy = [to_float(row.get("gt_y_m")) - to_float(row.get("ekf2_raw_y_m")) for row in subset]
    iekf_dx = [to_float(row.get("gt_x_m")) - to_float(row.get("iekf_px4_raw_x_m")) for row in subset]
    iekf_dy = [to_float(row.get("gt_y_m")) - to_float(row.get("iekf_px4_raw_y_m")) for row in subset]
    return {
        "rows": len(subset),
        "ekf2_offset_x_m": mean(ekf2_dx),
        "ekf2_offset_y_m": mean(ekf2_dy),
        "iekf_offset_x_m": mean(iekf_dx),
        "iekf_offset_y_m": mean(iekf_dy),
    }


def aligned_error_h(row: dict[str, object], offsets: dict[str, object], estimator: str) -> float:
    gt_x = to_float(row.get("gt_x_m"))
    gt_y = to_float(row.get("gt_y_m"))
    if estimator == "ekf2":
        x = to_float(row.get("ekf2_raw_x_m")) + to_float(offsets.get("ekf2_offset_x_m"))
        y = to_float(row.get("ekf2_raw_y_m")) + to_float(offsets.get("ekf2_offset_y_m"))
    else:
        x = to_float(row.get("iekf_px4_raw_x_m")) + to_float(offsets.get("iekf_offset_x_m"))
        y = to_float(row.get("iekf_px4_raw_y_m")) + to_float(offsets.get("iekf_offset_y_m"))
    return norm2(x - gt_x, y - gt_y)


def build_alignment_rows(
    pair_rows: list[dict[str, object]],
    target_rows: list[dict[str, object]],
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    offset_rows: list[dict[str, object]] = []
    summary_rows: list[dict[str, object]] = []
    early_offsets: dict[str, object] | None = None
    for label, start, end in ALIGN_WINDOWS:
        offsets = alignment_offsets(pair_rows, start, end)
        row = {"align_window": label, "align_start_sec": start, "align_end_sec": end, **offsets}
        row["ekf2_offset_h_m"] = norm2(to_float(row.get("ekf2_offset_x_m")), to_float(row.get("ekf2_offset_y_m")))
        row["iekf_offset_h_m"] = norm2(to_float(row.get("iekf_offset_x_m")), to_float(row.get("iekf_offset_y_m")))
        if early_offsets is None:
            early_offsets = row
        row["ekf2_offset_delta_from_early_h_m"] = norm2(
            to_float(row.get("ekf2_offset_x_m")) - to_float(early_offsets.get("ekf2_offset_x_m")),
            to_float(row.get("ekf2_offset_y_m")) - to_float(early_offsets.get("ekf2_offset_y_m")),
        )
        row["iekf_offset_delta_from_early_h_m"] = norm2(
            to_float(row.get("iekf_offset_x_m")) - to_float(early_offsets.get("iekf_offset_x_m")),
            to_float(row.get("iekf_offset_y_m")) - to_float(early_offsets.get("iekf_offset_y_m")),
        )
        offset_rows.append(row)

        for eval_label, eval_start, eval_end in SUMMARY_WINDOWS:
            subset = [item for item in pair_rows if is_armed(item) and in_window(item, eval_start, eval_end)]
            summary_rows.append(alignment_summary_row(label, eval_label, subset, row))
        summary_rows.append(alignment_summary_row(label, "original_target_high_rows", target_rows, row))
    return offset_rows, summary_rows


def alignment_summary_row(
    align_label: str,
    eval_label: str,
    subset: list[dict[str, object]],
    offsets: dict[str, object],
) -> dict[str, object]:
    ekf2_vals = [aligned_error_h(row, offsets, "ekf2") for row in subset]
    iekf_vals = [aligned_error_h(row, offsets, "iekf") for row in subset]
    return {
        "align_window": align_label,
        "eval_window": eval_label,
        "rows": len(subset),
        "ekf2_mean_m": mean(ekf2_vals),
        "iekf_mean_m": mean(iekf_vals),
        "iekf_minus_ekf2_mean_m": mean(iekf_vals) - mean(ekf2_vals),
        "ekf2_p95_m": percentile(ekf2_vals, 95.0),
        "iekf_p95_m": percentile(iekf_vals, 95.0),
        "iekf_frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in iekf_vals),
        "iekf_max_m": max((item for item in iekf_vals if finite(item)), default=math.nan),
    }


def build_update_context_rows(
    run_dir: Path,
    pair_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    updates = read_csv(run_dir / "phase_error_memory_diag" / "position_update_rows_with_pmem.csv")
    pair_rows = sorted(pair_rows, key=lambda row: to_float(row.get("ros_time_sec")))
    pair_times = [to_float(row.get("ros_time_sec")) for row in pair_rows]
    out: list[dict[str, object]] = []
    for update in updates:
        t_arm = to_float(update.get("time_since_arm_sec"))
        if not any(start <= t_arm < end for start, end in UPDATE_CONTEXT_WINDOWS):
            continue
        pair, pair_dt = nearest(pair_rows, pair_times, to_float(update.get("ros_time_sec")), 0.15)
        if pair is None:
            continue
        out.append({
            "sequence": update.get("sequence"),
            "time_since_arm_sec": t_arm,
            "ros_time_sec": update.get("ros_time_sec"),
            "pair_join_dt_sec": pair_dt,
            "context": context_label(update),
            "ekf2_error_h_m": pair.get("ekf2_error_h_m"),
            "current_iekf_error_h_m": pair.get("current_iekf_error_h_m"),
            "px4_iekf_error_h_m": pair.get("px4_iekf_error_h_m"),
            "px4_iekf_error_x_m": pair.get("px4_iekf_error_x_m"),
            "px4_iekf_error_y_m": pair.get("px4_iekf_error_y_m"),
            "horizontal_speed_mps": pair.get("horizontal_speed_mps"),
            "vertical_speed_mps": pair.get("vertical_speed_mps"),
            "gyro_deg_s": pair.get("gyro_deg_s"),
            "core_gnss_diff_h_m": pair.get("core_gnss_diff_h_m"),
            "gnss_residual_h_m": update.get("gnss_residual_h_m"),
            "dx_pos_h_norm_m": update.get("dx_pos_h_norm_m"),
            "dx_over_residual_h": update.get("dx_over_residual_h"),
            "gnss_hnis_2d": update.get("gnss_hnis_2d"),
            "phase_error_memory_pressure_active": update.get("phase_error_memory_pressure_active"),
            "phase_error_memory_candidate_active": update.get("phase_error_memory_candidate_active"),
            "phase_error_memory_reason": update.get("phase_error_memory_reason"),
        })
    out.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return out


def row_lookup(rows: list[dict[str, object]], key: str, value: str) -> dict[str, object]:
    return next(row for row in rows if str(row.get(key)) == value)


def write_report(
    out_dir: Path,
    window_summary: list[dict[str, object]],
    context_summary: list[dict[str, object]],
    target_rows: list[dict[str, object]],
    top_rows: list[dict[str, object]],
    offset_rows: list[dict[str, object]],
    alignment_rows: list[dict[str, object]],
    update_context_rows: list[dict[str, object]],
) -> None:
    window_table = [
        [
            row["window"],
            row["rows"],
            fmt(row["ekf2_mean_m"]),
            fmt(row["current_iekf_mean_m"]),
            fmt(row["px4_iekf_mean_m"]),
            fmt(row["px4_minus_ekf2_mean_m"]),
            fmt(row["px4_iekf_p95_m"]),
            fmt(row["px4_frac_over_0p3"], 3),
            fmt(row["current_to_px4_improvement_mean_m"]),
        ]
        for row in window_summary
    ]
    context_table = [
        [
            row["sample"],
            row["context"],
            row["rows"],
            fmt(row["ekf2_mean_m"]),
            fmt(row["current_iekf_mean_m"]),
            fmt(row["px4_iekf_mean_m"]),
            fmt(row["px4_gt_ekf2_frac"], 3),
        ]
        for row in context_summary
    ]
    target_table = [
        [
            fmt(row["time_since_arm_sec"], 1),
            row["context"],
            fmt(row["ekf2_error_h_m"]),
            fmt(row["current_iekf_error_h_m"]),
            fmt(row["px4_iekf_error_h_m"]),
            fmt(row["current_to_px4_improvement_m"]),
            fmt(row["core_native_velocity_mismatch_h_mps"]),
            row["phase_error_memory_reason"],
        ]
        for row in target_rows
    ]
    top_table = [
        [
            fmt(row["time_since_arm_sec"], 1),
            row["context"],
            fmt(row["ekf2_error_h_m"]),
            fmt(row["current_iekf_error_h_m"]),
            fmt(row["px4_iekf_error_h_m"]),
            fmt(row["px4_iekf_error_x_m"]),
            fmt(row["px4_iekf_error_y_m"]),
            fmt(row["horizontal_speed_mps"]),
            fmt(row["vertical_speed_mps"], 2),
            fmt(row["gyro_deg_s"], 1),
            fmt(row["core_gnss_diff_h_m"]),
        ]
        for row in top_rows[:14]
    ]
    offset_table = [
        [
            row["align_window"],
            row["rows"],
            fmt(row["ekf2_offset_delta_from_early_h_m"]),
            fmt(row["iekf_offset_delta_from_early_h_m"]),
            fmt(row["iekf_offset_x_m"]),
            fmt(row["iekf_offset_y_m"]),
        ]
        for row in offset_rows
    ]
    align_selected = [
        row for row in alignment_rows
        if row["align_window"] in {"early_0_20", "target_120_180", "cluster_151_160", "late_171_178p5"}
        and row["eval_window"] in {"120-180", "151-154p5", "171-178p5", "original_target_high_rows"}
    ]
    align_table = [
        [
            row["align_window"],
            row["eval_window"],
            row["rows"],
            fmt(row["ekf2_mean_m"]),
            fmt(row["iekf_mean_m"]),
            fmt(row["iekf_minus_ekf2_mean_m"]),
            fmt(row["iekf_p95_m"]),
            fmt(row["iekf_max_m"]),
        ]
        for row in align_selected
    ]
    update_cluster = [
        row for row in update_context_rows
        if 149.5 <= to_float(row.get("time_since_arm_sec")) < 154.5
    ]
    update_table = [
        [
            fmt(row["time_since_arm_sec"], 1),
            row["context"],
            fmt(row["ekf2_error_h_m"]),
            fmt(row["current_iekf_error_h_m"]),
            fmt(row["px4_iekf_error_h_m"]),
            fmt(row["gnss_residual_h_m"]),
            fmt(row["dx_over_residual_h"], 3),
            row["phase_error_memory_candidate_active"],
            row["phase_error_memory_reason"],
        ]
        for row in update_cluster
    ]

    target_summary = row_lookup(window_summary, "window", "original_target_high_rows")
    full_summary = row_lookup(window_summary, "window", "120-180")
    cluster_summary = row_lookup(window_summary, "window", "151-154p5")

    lines = [
        "# Shortgen02 Post-Projection Residual Diagnostic",
        "",
        "This report keeps estimator outputs fixed and diagnoses what remains after applying the PX4-sphere-compatible offline output projection.",
        "",
        "## Residual Window Summary",
        "",
        markdown_table(
            [
                "window",
                "rows",
                "ekf2_mean",
                "current_iekf_mean",
                "px4_iekf_mean",
                "px4_minus_ekf2",
                "px4_p95",
                "px4_frac_gt_0p3",
                "projection_gain",
            ],
            window_table,
        ),
        "",
        "## Context Coverage",
        "",
        markdown_table(
            ["sample", "ctx", "rows", "ekf2_mean", "current_mean", "px4_mean", "px4_gt_ekf2_frac"],
            context_table,
        ),
        "",
        "## Original Target Rows After PX4 Projection",
        "",
        markdown_table(
            ["t_arm", "ctx", "ekf2", "current", "px4", "gain", "native/core_vdiff", "pmem_reason"],
            target_table,
        ),
        "",
        "## Largest PX4-Projected IEKF Errors In 120-180 s",
        "",
        markdown_table(
            ["t_arm", "ctx", "ekf2", "current", "px4", "px4_ex", "px4_ey", "hspeed", "vspeed", "gyro", "core_gnss"],
            top_table,
        ),
        "",
        "## Local Alignment Cross-Check",
        "",
        markdown_table(
            ["align_window", "rows", "ekf2_shift", "iekf_shift", "iekf_dx", "iekf_dy"],
            offset_table,
        ),
        "",
        markdown_table(
            ["align", "eval", "rows", "ekf2_mean", "iekf_mean", "delta", "iekf_p95", "iekf_max"],
            align_table,
        ),
        "",
        "## Position Update Context Around 151-154.5 s",
        "",
        markdown_table(
            ["t_arm", "ctx", "ekf2", "current", "px4", "residual", "dx/res", "candidate", "reason"],
            update_table,
        ),
        "",
        "## Interpretation",
        "",
        f"- PX4 projection changes the original target high rows from current IEKF mean `{fmt(target_summary['current_iekf_mean_m'])}` m to `{fmt(target_summary['px4_iekf_mean_m'])}` m, but they remain above EKF2 by `{fmt(target_summary['px4_minus_ekf2_mean_m'])}` m on average.",
        f"- Over the full `120-180 s` window the PX4-projected IEKF mean is `{fmt(full_summary['px4_iekf_mean_m'])}` m versus EKF2 `{fmt(full_summary['ekf2_mean_m'])}` m, so the route mean is already better than EKF2 after projection normalization.",
        f"- The remaining worst cluster is no longer only the original late target set: `151-154.5 s` has PX4-projected IEKF mean `{fmt(cluster_summary['px4_iekf_mean_m'])}` m and p95 `{fmt(cluster_summary['px4_iekf_p95_m'])}` m.",
        "- The original target rows span `armed_cruise`, `turning`, `post_turn`, and `none`, so an online turn/post-turn gate cannot cover all late target rows by construction.",
        "- Local alignment can move one phase cluster down, but the required IEKF offset shifts by several decimeters between alignment windows. That is a local residual/phase problem, not a single global translation that should be hidden by another keeper threshold.",
        "- The 151-154.5 s update rows show candidate-active GNSS updates with small `dx/res` values while PX4-projected pair errors spike between and around updates; the next diagnosis should stay offline and explain these local residual spikes before adding a new online mechanism.",
        "",
        "Generated files:",
        f"- `{out_dir / 'post_projection_window_summary.csv'}`",
        f"- `{out_dir / 'post_projection_context_summary.csv'}`",
        f"- `{out_dir / 'original_target_high_rows_after_projection.csv'}`",
        f"- `{out_dir / 'top_px4_projection_errors_120_180.csv'}`",
        f"- `{out_dir / 'projection_local_alignment_offsets.csv'}`",
        f"- `{out_dir / 'projection_local_alignment_summary.csv'}`",
        f"- `{out_dir / 'post_projection_update_context_rows.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out-dir")
    parser.add_argument("--top-n", type=int, default=20)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir) if args.out_dir else run_dir / "shortgen02_projection_residual_diag"
    out_dir.mkdir(parents=True, exist_ok=True)

    pair_rows = build_pair_rows(run_dir)
    target_rows = build_target_rows(run_dir, pair_rows)
    top_rows = sorted(
        [row for row in pair_rows if is_armed(row) and in_window(row, 120.0, 180.0)],
        key=lambda row: to_float(row.get("px4_iekf_error_h_m")),
        reverse=True,
    )[: args.top_n]
    window_summary = build_window_summary(pair_rows, target_rows)
    context_summary = build_context_summary(target_rows, top_rows)
    offset_rows, alignment_rows = build_alignment_rows(pair_rows, target_rows)
    update_context_rows = build_update_context_rows(run_dir, pair_rows)

    write_csv(out_dir / "post_projection_window_summary.csv", window_summary)
    write_csv(out_dir / "post_projection_context_summary.csv", context_summary)
    write_csv(out_dir / "original_target_high_rows_after_projection.csv", target_rows)
    write_csv(out_dir / "top_px4_projection_errors_120_180.csv", top_rows)
    write_csv(out_dir / "projection_local_alignment_offsets.csv", offset_rows)
    write_csv(out_dir / "projection_local_alignment_summary.csv", alignment_rows)
    write_csv(out_dir / "post_projection_update_context_rows.csv", update_context_rows)
    write_report(out_dir, window_summary, context_summary, target_rows, top_rows, offset_rows, alignment_rows, update_context_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
