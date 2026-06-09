#!/usr/bin/env python3
"""Diagnose PHS2 already-projected online shortgen02 tail rows.

This script intentionally uses offline_groundtruth_convergence_diag, where the
online /kf_gins/odom output is scored directly. It does not apply the PX4 sphere
projection again.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Iterable


WINDOWS = [
    ("120-124p5", 120.0, 124.5),
    ("151-154p5", 151.0, 154.5),
    ("171-178p5", 171.0, 178.5),
    ("120-140", 120.0, 140.0),
    ("140-160", 140.0, 160.0),
    ("160-180", 160.0, 180.0),
    ("120-180", 120.0, 180.0),
]


def finite(value: float) -> bool:
    return math.isfinite(value)


def to_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return result if finite(result) else default


def truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"} or to_float(value, 0.0) > 0.5


def mean(values: Iterable[float]) -> float:
    vals = [item for item in values if finite(item)]
    return sum(vals) / len(vals) if vals else math.nan


def percentile(values: Iterable[float], pct: float) -> float:
    vals = sorted(item for item in values if finite(item))
    if not vals:
        return math.nan
    if len(vals) == 1:
        return vals[0]
    pos = (len(vals) - 1) * pct / 100.0
    lo = math.floor(pos)
    hi = math.ceil(pos)
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - pos) + vals[hi] * (pos - lo)


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def read_csv(path: Path) -> list[dict[str, str]]:
    if not path.exists() or path.stat().st_size == 0:
        return []
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


def context_label(row: dict[str, object]) -> str:
    text = str(row.get("context", "")).strip()
    if text:
        return text
    if truthy(row.get("turning_now")):
        return "turning"
    if truthy(row.get("post_turn_context")):
        return "post_turn"
    if truthy(row.get("armed_cruise_context")):
        return "armed_cruise"
    if truthy(row.get("terminal_descent_context")):
        return "terminal"
    return "none"


def in_window(row: dict[str, object], start: float, end: float, key: str = "time_since_arm_sec") -> bool:
    t = to_float(row.get(key))
    return finite(t) and start <= t < end


def norm2(x: float, y: float) -> float:
    return math.hypot(x, y) if finite(x) and finite(y) else math.nan


def velocity_mismatch(row: dict[str, object]) -> float:
    nv_n = to_float(row.get("native_velocity_vN_mps"))
    nv_e = to_float(row.get("native_velocity_vE_mps"))
    cv_n = to_float(row.get("core_velocity_vN_mps"))
    cv_e = to_float(row.get("core_velocity_vE_mps"))
    return norm2(nv_n - cv_n, nv_e - cv_e)


def gnss_position_residual_h(row: dict[str, object]) -> float:
    return norm2(
        to_float(row.get("gnss_position_residual_n_m")),
        to_float(row.get("gnss_position_residual_e_m")),
    )


def interval_for_time(
    rows: list[dict[str, object]],
    t: float,
) -> dict[str, object] | None:
    if not finite(t):
        return None
    for row in rows:
        start = to_float(row.get("start_time_since_arm_sec"))
        end = to_float(row.get("end_time_since_arm_sec"))
        if finite(start) and finite(end) and start <= t < end:
            return row
    return None


def summarize_pair_rows(label: str, rows: list[dict[str, object]], high_threshold_m: float) -> dict[str, object]:
    iekf = [to_float(row.get("iekf_error_xy_m")) for row in rows]
    ekf2 = [to_float(row.get("ekf2_error_xy_m")) for row in rows]
    high = [row for row in rows if to_float(row.get("iekf_error_xy_m")) > high_threshold_m]
    err_x = [to_float(row.get("iekf_error_x_m")) for row in rows]
    err_y = [to_float(row.get("iekf_error_y_m")) for row in rows]
    shift_x = mean(err_x)
    shift_y = mean(err_y)
    local = [
        norm2(to_float(row.get("iekf_error_x_m")) - shift_x,
              to_float(row.get("iekf_error_y_m")) - shift_y)
        for row in rows
    ]
    return {
        "window": label,
        "rows": len(rows),
        "high_threshold_m": high_threshold_m,
        "high_rows": len(high),
        "high_frac": len(high) / len(rows) if rows else math.nan,
        "ekf2_mean_m": mean(ekf2),
        "iekf_mean_m": mean(iekf),
        "iekf_minus_ekf2_mean_m": mean(iekf) - mean(ekf2),
        "ekf2_p95_m": percentile(ekf2, 95.0),
        "iekf_p95_m": percentile(iekf, 95.0),
        "iekf_error_x_mean_m": shift_x,
        "iekf_error_y_mean_m": shift_y,
        "iekf_error_mean_vector_norm_m": norm2(shift_x, shift_y),
        "iekf_local_shifted_mean_m": mean(local),
        "iekf_local_shifted_p95_m": percentile(local, 95.0),
    }


def summarize_enriched(label: str, rows: list[dict[str, object]]) -> dict[str, object]:
    return {
        "sample": label,
        "rows": len(rows),
        "ekf2_mean_m": mean(to_float(row.get("ekf2_error_h_m")) for row in rows),
        "iekf_mean_m": mean(to_float(row.get("iekf_error_h_m")) for row in rows),
        "gnss_measurement_error_mean_m": mean(
            to_float(row.get("geometry_inferred_gnss_measurement_error_h_m")) for row in rows),
        "gps_fit_all_error_mean_m": mean(to_float(row.get("gps_fit_all_error_h_m")) for row in rows),
        "gps_fit_moving_error_mean_m": mean(to_float(row.get("gps_fit_moving_error_h_m")) for row in rows),
        "moving_fit_gain_mean_m": mean(
            to_float(row.get("gps_fit_all_error_h_m")) - to_float(row.get("gps_fit_moving_error_h_m"))
            for row in rows),
        "core_native_vdiff_mean_mps": mean(to_float(row.get("core_native_velocity_mismatch_h_mps")) for row in rows),
        "core_gnss_diff_mean_m": mean(to_float(row.get("core_gnss_diff_h_m")) for row in rows),
        "gnss_residual_mean_m": mean(to_float(row.get("gnss_residual_h_m")) for row in rows),
        "update_dx_mean_m": mean(to_float(row.get("dx_pos_h_norm_m")) for row in rows),
        "update_dx_over_residual_mean": mean(to_float(row.get("dx_over_residual_h")) for row in rows),
        "update_improvement_mean_m": mean(to_float(row.get("error_improvement_h_m")) for row in rows),
        "interval_iekf_vec_mean_m": mean(to_float(row.get("interval_iekf_vector_growth_h_m")) for row in rows),
        "interval_ekf2_vec_mean_m": mean(to_float(row.get("interval_ekf2_vector_change_h_m")) for row in rows),
        "interval_delta_mean_m": mean(to_float(row.get("interval_iekf_minus_ekf2_vector_change_m")) for row in rows),
    }


def build_enriched_rows(
    pair_rows: list[dict[str, object]],
    update_rows: list[dict[str, object]],
    update_times: list[float],
    gnss_rows: list[dict[str, object]],
    gnss_times: list[float],
    timing_rows: list[dict[str, object]],
    timing_times: list[float],
    interval_rows: list[dict[str, object]],
    high_threshold_m: float,
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for pair in pair_rows:
        t_arm = to_float(pair.get("time_since_arm_sec"))
        if not (120.0 <= t_arm < 180.0):
            continue
        iekf_err = to_float(pair.get("iekf_error_xy_m"))
        if not (iekf_err > high_threshold_m):
            continue
        update, update_dt = nearest(update_rows, update_times, t_arm, 0.35)
        gnss = None
        gnss_dt = math.nan
        if update is not None:
            gnss, gnss_dt = nearest(
                gnss_rows,
                gnss_times,
                to_float(update.get("update_time_sec")),
                0.20,
            )
        timing, timing_dt = nearest(timing_rows, timing_times, t_arm, 0.20)
        interval = interval_for_time(interval_rows, t_arm)

        row: dict[str, object] = {
            "time_since_arm_sec": t_arm,
            "pair_ros_time_sec": pair.get("pair_ros_time_sec"),
            "context": context_label(pair),
            "mavros_mode": pair.get("mavros_mode"),
            "ekf2_error_h_m": pair.get("ekf2_error_xy_m"),
            "iekf_error_h_m": pair.get("iekf_error_xy_m"),
            "iekf_minus_ekf2_h_m": iekf_err - to_float(pair.get("ekf2_error_xy_m")),
            "iekf_error_x_m": pair.get("iekf_error_x_m"),
            "iekf_error_y_m": pair.get("iekf_error_y_m"),
            "horizontal_speed_mps": pair.get("horizontal_speed_mps"),
            "vertical_speed_mps": pair.get("vertical_speed_mps"),
            "gyro_deg_s": pair.get("gyro_deg_s"),
            "core_gnss_diff_h_m": pair.get("core_gnss_diff_h_m"),
            "nearest_update_dt_sec": update_dt,
            "timing_row_dt_sec": timing_dt,
        }

        if update is not None:
            for key in (
                "update_time_sec",
                "error_before_h_m",
                "error_after_h_m",
                "error_improvement_h_m",
                "dx_pos_h_norm_m",
                "dx_over_residual_h",
                "gnss_residual_h_m",
                "gnss_hnis_2d",
                "gnss_position_std_h_m",
            ):
                row[key] = update.get(key)
        if gnss is not None:
            row["gnss_update_join_dt_sec"] = gnss_dt
            gnss_resid_h = gnss_position_residual_h(gnss)
            if not finite(to_float(row.get("gnss_residual_h_m"))) and finite(gnss_resid_h):
                row["gnss_residual_h_m"] = gnss_resid_h
            dx_h = to_float(row.get("dx_pos_h_norm_m"))
            if (
                not finite(to_float(row.get("dx_over_residual_h")))
                and finite(dx_h)
                and gnss_resid_h > 1e-12
            ):
                row["dx_over_residual_h"] = dx_h / gnss_resid_h
            row["native_velocity_vN_mps"] = gnss.get("native_velocity_vN_mps")
            row["native_velocity_vE_mps"] = gnss.get("native_velocity_vE_mps")
            row["core_velocity_vN_mps"] = gnss.get("core_velocity_vN_mps")
            row["core_velocity_vE_mps"] = gnss.get("core_velocity_vE_mps")
            row["core_native_velocity_mismatch_h_mps"] = velocity_mismatch(gnss)
            for key in (
                "turn_postturn_native_velocity_deweight_core_native_residual_h_mps",
                "gnss_velocity_outward_damping_core_native_residual_h_mps",
                "gnss_position_gain_response_effective_std_h_m",
                "gnss_position_gain_response_reason",
                "mission_cov_hygiene_reason",
                "phase_error_memory_debug_enabled",
                "phase_error_memory_candidate_active",
                "phase_error_memory_reason",
            ):
                row[key] = gnss.get(key)
        if timing is not None:
            for key in (
                "geometry_inferred_gnss_measurement_error_h_m",
                "geometry_gnss_delta_over_need",
                "gps_raw_error_h_m",
                "gps_fit_all_error_h_m",
                "gps_fit_moving_error_h_m",
                "best_lag_sec",
                "best_lag_error_h_m",
                "same_minus_best_lag_error_h_m",
                "same_minus_geometry_inferred_error_h_m",
            ):
                row[key] = timing.get(key)
        if interval is not None:
            row["interval_start_time_since_arm_sec"] = interval.get("start_time_since_arm_sec")
            row["interval_end_time_since_arm_sec"] = interval.get("end_time_since_arm_sec")
            row["interval_dt_sec"] = interval.get("dt_sec")
            row["interval_iekf_scalar_growth_m"] = interval.get("iekf_scalar_growth_m")
            row["interval_iekf_vector_growth_h_m"] = interval.get("iekf_vector_growth_h_m")
            row["interval_ekf2_scalar_growth_m"] = interval.get("ekf2_scalar_growth_m")
            row["interval_ekf2_vector_change_h_m"] = interval.get("ekf2_vector_change_h_m")
            row["interval_iekf_minus_ekf2_vector_change_m"] = interval.get("iekf_minus_ekf2_vector_change_m")
        out.append(row)
    return out


def group_by_context(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    groups: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        groups.setdefault(context_label(row), []).append(row)
    out = []
    for ctx, ctx_rows in sorted(groups.items()):
        summary = summarize_enriched(ctx, ctx_rows)
        summary["context"] = ctx
        out.append(summary)
    return out


def summarize_intervals(interval_rows: list[dict[str, object]], target_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    windows = []
    for label, start, end in WINDOWS:
        rows = [
            row for row in interval_rows
            if in_window(row, start, end, "start_time_since_arm_sec")
        ]
        windows.append({
            "window": label,
            "intervals": len(rows),
            "iekf_vec_mean_m": mean(to_float(row.get("iekf_vector_growth_h_m")) for row in rows),
            "iekf_vec_p95_m": percentile((to_float(row.get("iekf_vector_growth_h_m")) for row in rows), 95.0),
            "ekf2_vec_mean_m": mean(to_float(row.get("ekf2_vector_change_h_m")) for row in rows),
            "ekf2_vec_p95_m": percentile((to_float(row.get("ekf2_vector_change_h_m")) for row in rows), 95.0),
            "iekf_minus_ekf2_vec_mean_m": mean(
                to_float(row.get("iekf_minus_ekf2_vector_change_m")) for row in rows),
            "iekf_vec_gt_ekf2_frac": mean(
                1.0 if to_float(row.get("iekf_vector_growth_h_m")) >
                to_float(row.get("ekf2_vector_change_h_m")) else 0.0
                for row in rows),
        })

    used: dict[tuple[float, float], dict[str, object]] = {}
    for row in target_rows:
        start = to_float(row.get("interval_start_time_since_arm_sec"))
        end = to_float(row.get("interval_end_time_since_arm_sec"))
        if finite(start) and finite(end):
            used[(start, end)] = row
    rows = list(used.values())
    windows.append({
        "window": "target_high_unique_intervals",
        "intervals": len(rows),
        "iekf_vec_mean_m": mean(to_float(row.get("interval_iekf_vector_growth_h_m")) for row in rows),
        "iekf_vec_p95_m": percentile((to_float(row.get("interval_iekf_vector_growth_h_m")) for row in rows), 95.0),
        "ekf2_vec_mean_m": mean(to_float(row.get("interval_ekf2_vector_change_h_m")) for row in rows),
        "ekf2_vec_p95_m": percentile((to_float(row.get("interval_ekf2_vector_change_h_m")) for row in rows), 95.0),
        "iekf_minus_ekf2_vec_mean_m": mean(
            to_float(row.get("interval_iekf_minus_ekf2_vector_change_m")) for row in rows),
        "iekf_vec_gt_ekf2_frac": mean(
            1.0 if to_float(row.get("interval_iekf_vector_growth_h_m")) >
            to_float(row.get("interval_ekf2_vector_change_h_m")) else 0.0
            for row in rows),
    })
    return windows


def write_report(
    out_dir: Path,
    pair_summary: list[dict[str, object]],
    target_summary: list[dict[str, object]],
    context_summary: list[dict[str, object]],
    interval_summary: list[dict[str, object]],
    target_rows: list[dict[str, object]],
    high_threshold_m: float,
) -> None:
    by_window = {row["window"]: row for row in pair_summary}
    row_120_180 = by_window.get("120-180", {})
    lines = [
        "# Shortgen02 PHS2 Tail Diagnostic",
        "",
        "This report scores the online already-projected `/kf_gins/odom` directly from `offline_groundtruth_convergence_diag/groundtruth_joined.csv`.",
        "No offline PX4 projection is applied here, so this is PHS2-compatible and avoids double projection.",
        "",
        f"High-tail threshold: `{high_threshold_m:.3f} m` IEKF horizontal groundtruth error.",
        "",
        "## Online Window Summary",
        "",
        markdown_table(
            [
                "window", "rows", "high", "ekf2_mean", "iekf_mean", "delta",
                "ekf2_p95", "iekf_p95", "mean_vec", "local_mean", "local_p95",
            ],
            [
                [
                    row["window"],
                    int(to_float(row.get("rows"), 0.0)),
                    int(to_float(row.get("high_rows"), 0.0)),
                    fmt(row.get("ekf2_mean_m")),
                    fmt(row.get("iekf_mean_m")),
                    fmt(row.get("iekf_minus_ekf2_mean_m")),
                    fmt(row.get("ekf2_p95_m")),
                    fmt(row.get("iekf_p95_m")),
                    fmt(row.get("iekf_error_mean_vector_norm_m")),
                    fmt(row.get("iekf_local_shifted_mean_m")),
                    fmt(row.get("iekf_local_shifted_p95_m")),
                ]
                for row in pair_summary
            ],
        ),
        "",
        "## Target High Rows",
        "",
        markdown_table(
            [
                "sample", "rows", "ekf2", "iekf", "gnss_meas", "gps_all",
                "gps_moving", "move_gain", "v_mismatch", "gnss_resid",
                "dx", "dx/res", "growth_delta",
            ],
            [
                [
                    row["sample"],
                    int(to_float(row.get("rows"), 0.0)),
                    fmt(row.get("ekf2_mean_m")),
                    fmt(row.get("iekf_mean_m")),
                    fmt(row.get("gnss_measurement_error_mean_m")),
                    fmt(row.get("gps_fit_all_error_mean_m")),
                    fmt(row.get("gps_fit_moving_error_mean_m")),
                    fmt(row.get("moving_fit_gain_mean_m")),
                    fmt(row.get("core_native_vdiff_mean_mps")),
                    fmt(row.get("gnss_residual_mean_m")),
                    fmt(row.get("update_dx_mean_m")),
                    fmt(row.get("update_dx_over_residual_mean")),
                    fmt(row.get("interval_delta_mean_m")),
                ]
                for row in target_summary
            ],
        ),
        "",
        "## Context Split For High Rows",
        "",
        markdown_table(
            [
                "context", "rows", "ekf2", "iekf", "gnss_meas",
                "v_mismatch", "dx/res", "growth_delta",
            ],
            [
                [
                    row.get("context", row.get("sample", "")),
                    int(to_float(row.get("rows"), 0.0)),
                    fmt(row.get("ekf2_mean_m")),
                    fmt(row.get("iekf_mean_m")),
                    fmt(row.get("gnss_measurement_error_mean_m")),
                    fmt(row.get("core_native_vdiff_mean_mps")),
                    fmt(row.get("update_dx_over_residual_mean")),
                    fmt(row.get("interval_delta_mean_m")),
                ]
                for row in context_summary
            ],
        ),
        "",
        "## Between-Update Growth",
        "",
        markdown_table(
            [
                "window", "intervals", "iekf_vec", "iekf_p95",
                "ekf2_vec", "ekf2_p95", "delta", "frac",
            ],
            [
                [
                    row["window"],
                    int(to_float(row.get("intervals"), 0.0)),
                    fmt(row.get("iekf_vec_mean_m")),
                    fmt(row.get("iekf_vec_p95_m")),
                    fmt(row.get("ekf2_vec_mean_m")),
                    fmt(row.get("ekf2_vec_p95_m")),
                    fmt(row.get("iekf_minus_ekf2_vec_mean_m")),
                    fmt(row.get("iekf_vec_gt_ekf2_frac"), 3),
                ]
                for row in interval_summary
            ],
        ),
        "",
        "## Largest High Rows",
        "",
        markdown_table(
            [
                "t_arm", "ctx", "ekf2", "iekf", "gnss_meas", "gps_all",
                "gps_moving", "v_mismatch", "dx/res", "growth_delta",
            ],
            [
                [
                    fmt(row.get("time_since_arm_sec"), 1),
                    row.get("context", ""),
                    fmt(row.get("ekf2_error_h_m")),
                    fmt(row.get("iekf_error_h_m")),
                    fmt(row.get("geometry_inferred_gnss_measurement_error_h_m")),
                    fmt(row.get("gps_fit_all_error_h_m")),
                    fmt(row.get("gps_fit_moving_error_h_m")),
                    fmt(row.get("core_native_velocity_mismatch_h_mps")),
                    fmt(row.get("dx_over_residual_h")),
                    fmt(row.get("interval_iekf_minus_ekf2_vector_change_m")),
                ]
                for row in sorted(
                    target_rows,
                    key=lambda item: to_float(item.get("iekf_error_h_m")),
                    reverse=True,
                )[:20]
            ],
        ),
        "",
        "## Interpretation",
        "",
        (
            f"- In `120-180 s`, online IEKF mean is `{fmt(row_120_180.get('iekf_mean_m'))}` m "
            f"versus EKF2 `{fmt(row_120_180.get('ekf2_mean_m'))}` m, with "
            f"`{int(to_float(row_120_180.get('high_rows'), 0.0))}` high-tail rows above "
            f"`{high_threshold_m:.3f}` m."
        ),
        "- The `mean_vec` and `local_mean` columns estimate how much of each window is a local residual-vector bias versus scatter after a local recentering. This is diagnostic only and is not a keeper scoring mode.",
        "- The target-high table joins the same high rows to GNSS measurement geometry, native/core velocity mismatch, accepted update response, and the containing propagation interval.",
        "- If `gps_moving` is much lower than `gps_all`, the local measurement/projection fit is route-phase dependent; if `growth_delta` is high, between-update IEKF motion is a separate tail source.",
        "",
        "Generated files:",
        f"- `{out_dir / 'phs2_tail_window_summary.csv'}`",
        f"- `{out_dir / 'phs2_tail_high_rows.csv'}`",
        f"- `{out_dir / 'phs2_tail_high_context_summary.csv'}`",
        f"- `{out_dir / 'phs2_tail_interval_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--high-error-threshold-m", type=float, default=0.3)
    args = parser.parse_args()

    run_dir = args.run_dir
    out_dir = args.out_dir if args.out_dir is not None else run_dir / "shortgen02_phs2_tail_diag"
    out_dir.mkdir(parents=True, exist_ok=True)

    pair_rows, _ = load_sorted(
        run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv",
        "time_since_arm_sec",
    )
    pair_rows = [
        row for row in pair_rows
        if to_float(row.get("mavros_armed"), 0.0) > 0.5
    ]
    update_rows, update_times = load_sorted(
        run_dir / "phase_error_memory_diag" / "position_update_rows_with_pmem.csv",
        "time_since_arm_sec",
    )
    gnss_rows, gnss_times = load_sorted(run_dir / "gnss_update_debug.csv", "update_time_sec")
    timing_rows, timing_times = load_sorted(
        run_dir / "shortgen02_measurement_timing_projection_diag" / "target_projection_timing_rows.csv",
        "time_since_arm_sec",
    )
    interval_rows, _ = load_sorted(
        run_dir / "shortgen02_measurement_timing_projection_diag" / "propagation_compare_rows.csv",
        "start_time_since_arm_sec",
    )

    pair_summary = [
        summarize_pair_rows(
            label,
            [row for row in pair_rows if in_window(row, start, end)],
            args.high_error_threshold_m,
        )
        for label, start, end in WINDOWS
    ]
    target_rows = build_enriched_rows(
        pair_rows,
        update_rows,
        update_times,
        gnss_rows,
        gnss_times,
        timing_rows,
        timing_times,
        interval_rows,
        args.high_error_threshold_m,
    )
    target_summary = [
        summarize_enriched("high_rows_120_180", target_rows),
    ]
    for label, start, end in WINDOWS:
        rows = [
            row for row in target_rows
            if in_window(row, start, end)
        ]
        if rows:
            target_summary.append(summarize_enriched(f"high_{label}", rows))

    context_summary = group_by_context(target_rows)
    interval_summary = summarize_intervals(interval_rows, target_rows)

    write_csv(out_dir / "phs2_tail_window_summary.csv", pair_summary)
    write_csv(out_dir / "phs2_tail_high_rows.csv", target_rows)
    write_csv(out_dir / "phs2_tail_high_summary.csv", target_summary)
    write_csv(out_dir / "phs2_tail_high_context_summary.csv", context_summary)
    write_csv(out_dir / "phs2_tail_interval_summary.csv", interval_summary)
    write_report(
        out_dir,
        pair_summary,
        target_summary,
        context_summary,
        interval_summary,
        target_rows,
        args.high_error_threshold_m,
    )

    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'phs2_tail_window_summary.csv'}")
    print(f"wrote: {out_dir / 'phs2_tail_high_rows.csv'}")
    print(f"wrote: {out_dir / 'phs2_tail_high_summary.csv'}")
    print(f"wrote: {out_dir / 'phs2_tail_high_context_summary.csv'}")
    print(f"wrote: {out_dir / 'phs2_tail_interval_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
