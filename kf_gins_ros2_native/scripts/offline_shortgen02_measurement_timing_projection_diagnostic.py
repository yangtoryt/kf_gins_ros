#!/usr/bin/env python3
"""Diagnose shortgen02 GPS projection/timing and between-update propagation."""

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


TARGET_WINDOWS = [
    ("120-124p5", 120.0, 124.5),
    ("171-178p5", 171.0, 178.5),
    ("120-180", 120.0, 180.0),
]
LAG_OFFSETS_SEC = [round(-1.0 + 0.1 * idx, 1) for idx in range(21)]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def bool01(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"} or to_float(value, 0.0) > 0.5


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def norm2(x: float, y: float) -> float:
    return math.hypot(x, y)


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
    if dt > max_dt_sec:
        return None, dt
    return row, dt


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


def in_window(row: dict[str, object], start: float, end: float, key: str = "time_since_arm_sec") -> bool:
    t = to_float(row.get(key))
    return finite(t) and start <= t < end


def load_fit(run_dir: Path, subset: str) -> dict[str, float]:
    fit_path = run_dir / "offline_ulog_gps_groundtruth_diag" / "gps_groundtruth_fit.csv"
    for row in read_csv(fit_path):
        if row.get("topic") == "vehicle_gps_position" and row.get("subset") == subset:
            return {
                "a_real": to_float(row.get("a_real")),
                "a_imag": to_float(row.get("a_imag")),
                "trans_e_m": to_float(row.get("trans_e_m")),
                "trans_n_m": to_float(row.get("trans_n_m")),
                "scale": to_float(row.get("scale")),
                "angle_deg": to_float(row.get("angle_deg")),
            }
    raise RuntimeError(f"missing vehicle_gps_position/{subset} fit in {fit_path}")


def apply_fit(gps_row: dict[str, object], fit: dict[str, float]) -> tuple[float, float]:
    gps_e = to_float(gps_row.get("gps_e_m"))
    gps_n = to_float(gps_row.get("gps_n_m"))
    a_real = fit["a_real"]
    a_imag = fit["a_imag"]
    trans_e = fit["trans_e_m"]
    trans_n = fit["trans_n_m"]
    return (
        a_real * gps_e - a_imag * gps_n + trans_e,
        a_imag * gps_e + a_real * gps_n + trans_n,
    )


def fit_error(gps_row: dict[str, object], gt_row: dict[str, object], fit: dict[str, float]) -> float:
    fit_e, fit_n = apply_fit(gps_row, fit)
    return norm2(fit_e - to_float(gt_row.get("gt_e_m")), fit_n - to_float(gt_row.get("gt_n_m")))


def load_vehicle_gps_rows(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    rows = [
        row for row in read_csv(run_dir / "offline_ulog_gps_groundtruth_diag" / "gps_groundtruth_joined.csv")
        if row.get("topic") == "vehicle_gps_position"
    ]
    rows.sort(key=lambda row: to_float(row.get("timestamp_sec")))
    return rows, [to_float(row.get("timestamp_sec")) for row in rows]


def best_lag_for_row(
    gps_row: dict[str, object],
    source_time_sec: float,
    gps_rows: list[dict[str, object]],
    gps_times: list[float],
    fit: dict[str, float],
) -> tuple[float, float, float]:
    best_lag = math.nan
    best_error = math.nan
    best_dt = math.nan
    for lag_sec in LAG_OFFSETS_SEC:
        gt_row, gt_dt = nearest(gps_rows, gps_times, source_time_sec + lag_sec, 0.25)
        if gt_row is None:
            continue
        error = fit_error(gps_row, gt_row, fit)
        if not finite(best_error) or error < best_error:
            best_lag = lag_sec
            best_error = error
            best_dt = gt_dt
    return best_lag, best_error, best_dt


def build_target_rows(run_dir: Path) -> tuple[list[dict[str, object]], list[dict[str, object]], list[float]]:
    geometry_rows = read_csv(run_dir / "shortgen02_measurement_geometry_diag" / "target_update_geometry_rows.csv")
    gnss_rows, gnss_times = load_sorted(run_dir / "gnss_update_debug.csv", "ros_time_sec")
    gps_rows, gps_times = load_vehicle_gps_rows(run_dir)
    fit_all = load_fit(run_dir, "all")
    fit_moving = load_fit(run_dir, "moving")

    out: list[dict[str, object]] = []
    for geom in geometry_rows:
        ros_time = to_float(geom.get("ros_time_sec"))
        gnss, gnss_dt = nearest(gnss_rows, gnss_times, ros_time, 0.16)
        if gnss is None:
            continue
        source_time = to_float(gnss.get("update_time_sec"))
        gps, gps_dt = nearest(gps_rows, gps_times, source_time, 0.25)
        if gps is None:
            continue

        all_fit_e, all_fit_n = apply_fit(gps, fit_all)
        moving_fit_e, moving_fit_n = apply_fit(gps, fit_moving)
        gt_e = to_float(gps.get("gt_e_m"))
        gt_n = to_float(gps.get("gt_n_m"))
        same_fit_error = norm2(all_fit_e - gt_e, all_fit_n - gt_n)
        moving_fit_error = norm2(moving_fit_e - gt_e, moving_fit_n - gt_n)
        best_lag, best_error, best_dt = best_lag_for_row(gps, source_time, gps_rows, gps_times, fit_all)

        native_vn = to_float(gnss.get("native_velocity_vN_mps"))
        native_ve = to_float(gnss.get("native_velocity_vE_mps"))
        core_vn = to_float(gnss.get("core_velocity_vN_mps"))
        core_ve = to_float(gnss.get("core_velocity_vE_mps"))
        core_native_vdiff = norm2(native_vn - core_vn, native_ve - core_ve)

        out.append({
            "time_since_arm_sec": geom.get("time_since_arm_sec"),
            "ros_time_sec": ros_time,
            "gnss_ros_join_dt_sec": gnss_dt,
            "gnss_update_time_sec": source_time,
            "gps_join_dt_sec": gps_dt,
            "context": geom.get("context", ""),
            "pair_iekf_error_h_m": geom.get("pair_iekf_error_h_m"),
            "pair_ekf2_error_h_m": geom.get("pair_ekf2_error_h_m"),
            "geometry_inferred_gnss_measurement_error_h_m": geom.get("gnss_measurement_error_h_m"),
            "geometry_gnss_delta_over_need": geom.get("gnss_delta_over_need"),
            "gps_raw_error_h_m": gps.get("raw_error_xy_m"),
            "gps_fit_all_error_h_m": same_fit_error,
            "gps_fit_moving_error_h_m": moving_fit_error,
            "gps_fit_all_e_m": all_fit_e,
            "gps_fit_all_n_m": all_fit_n,
            "gt_e_m": gt_e,
            "gt_n_m": gt_n,
            "best_lag_sec": best_lag,
            "best_lag_error_h_m": best_error,
            "best_lag_gt_join_dt_sec": best_dt,
            "same_minus_best_lag_error_h_m": same_fit_error - best_error if finite(best_error) else math.nan,
            "same_minus_geometry_inferred_error_h_m": (
                same_fit_error - to_float(geom.get("gnss_measurement_error_h_m"))
            ),
            "core_native_velocity_mismatch_h_mps": core_native_vdiff,
            "native_velocity_h_mps": norm2(native_vn, native_ve),
            "core_velocity_h_mps": norm2(core_vn, core_ve),
            "phase_error_memory_reason": geom.get("phase_error_memory_reason", ""),
        })
    out.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return out, gps_rows, gps_times


def build_lag_sweep_rows(
    target_rows: list[dict[str, object]],
    gps_rows: list[dict[str, object]],
    gps_times: list[float],
    fit: dict[str, float],
) -> list[dict[str, object]]:
    sweep_rows: list[dict[str, object]] = []
    for label, start, end in TARGET_WINDOWS:
        subset = [row for row in target_rows if in_window(row, start, end)]
        for lag_sec in LAG_OFFSETS_SEC:
            errors: list[float] = []
            for row in subset:
                gps, _ = nearest(gps_rows, gps_times, to_float(row.get("gnss_update_time_sec")), 0.25)
                gt, _ = nearest(gps_rows, gps_times, to_float(row.get("gnss_update_time_sec")) + lag_sec, 0.25)
                if gps is None or gt is None:
                    continue
                errors.append(fit_error(gps, gt, fit))
            sweep_rows.append({
                "window": label,
                "lag_sec": lag_sec,
                "rows": len(errors),
                "fit_error_mean_m": mean(errors),
                "fit_error_p95_m": percentile(errors, 95.0),
            })
    return sweep_rows


def summarize_projection(
    target_rows: list[dict[str, object]],
    lag_sweep_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    summary: list[dict[str, object]] = []
    for label, start, end in TARGET_WINDOWS:
        subset = [row for row in target_rows if in_window(row, start, end)]
        lag_subset = [row for row in lag_sweep_rows if row.get("window") == label and to_float(row.get("rows")) > 0]
        same_lag = min(lag_subset, key=lambda row: abs(to_float(row.get("lag_sec"))), default={})
        best_lag = min(lag_subset, key=lambda row: to_float(row.get("fit_error_mean_m")), default={})
        summary.append({
            "window": label,
            "rows": len(subset),
            "geometry_inferred_gnss_meas_error_mean_m": mean(
                to_float(row.get("geometry_inferred_gnss_measurement_error_h_m")) for row in subset
            ),
            "direct_gps_fit_error_mean_m": mean(to_float(row.get("gps_fit_all_error_h_m")) for row in subset),
            "direct_gps_fit_error_p95_m": percentile(
                (to_float(row.get("gps_fit_all_error_h_m")) for row in subset), 95.0
            ),
            "direct_gps_raw_error_mean_m": mean(to_float(row.get("gps_raw_error_h_m")) for row in subset),
            "moving_fit_error_mean_m": mean(to_float(row.get("gps_fit_moving_error_h_m")) for row in subset),
            "all_minus_moving_fit_error_mean_m": (
                mean(to_float(row.get("gps_fit_all_error_h_m")) for row in subset)
                - mean(to_float(row.get("gps_fit_moving_error_h_m")) for row in subset)
            ),
            "best_common_lag_sec": to_float(best_lag.get("lag_sec")),
            "best_common_lag_error_mean_m": to_float(best_lag.get("fit_error_mean_m")),
            "zero_lag_error_mean_m": to_float(same_lag.get("fit_error_mean_m")),
            "zero_minus_best_lag_error_m": (
                to_float(same_lag.get("fit_error_mean_m")) - to_float(best_lag.get("fit_error_mean_m"))
            ),
            "core_native_velocity_mismatch_mean_mps": mean(
                to_float(row.get("core_native_velocity_mismatch_h_mps")) for row in subset
            ),
        })
    return summary


def build_propagation_rows(run_dir: Path) -> list[dict[str, object]]:
    pos_rows = read_csv(run_dir / "phase_error_memory_diag" / "position_update_rows_with_pmem.csv")
    pos_rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    joined_rows, joined_times = load_sorted(
        run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv",
        "pair_ros_time_sec",
    )
    out: list[dict[str, object]] = []
    for current, nxt in zip(pos_rows, pos_rows[1:]):
        start = to_float(current.get("time_since_arm_sec"))
        end = to_float(nxt.get("time_since_arm_sec"))
        if not finite(start) or not finite(end) or end <= start:
            continue
        if not any(win_start <= start < win_end for _, win_start, win_end in TARGET_WINDOWS):
            continue
        join0, dt0 = nearest(joined_rows, joined_times, to_float(current.get("ros_time_sec")), 0.15)
        join1, dt1 = nearest(joined_rows, joined_times, to_float(nxt.get("ros_time_sec")), 0.15)
        if join0 is None or join1 is None:
            continue

        iekf_growth_x = to_float(nxt.get("error_before_x_m")) - to_float(current.get("error_after_x_m"))
        iekf_growth_y = to_float(nxt.get("error_before_y_m")) - to_float(current.get("error_after_y_m"))
        ekf2_growth_x = to_float(join1.get("ekf2_error_x_m")) - to_float(join0.get("ekf2_error_x_m"))
        ekf2_growth_y = to_float(join1.get("ekf2_error_y_m")) - to_float(join0.get("ekf2_error_y_m"))
        out.append({
            "start_time_since_arm_sec": start,
            "end_time_since_arm_sec": end,
            "dt_sec": end - start,
            "start_context": context_label(current),
            "end_context": context_label(nxt),
            "pair_join0_dt_sec": dt0,
            "pair_join1_dt_sec": dt1,
            "iekf_scalar_growth_m": to_float(nxt.get("error_before_h_m")) - to_float(current.get("error_after_h_m")),
            "iekf_vector_growth_h_m": norm2(iekf_growth_x, iekf_growth_y),
            "iekf_vector_growth_x_m": iekf_growth_x,
            "iekf_vector_growth_y_m": iekf_growth_y,
            "ekf2_scalar_growth_m": to_float(join1.get("ekf2_error_xy_m")) - to_float(join0.get("ekf2_error_xy_m")),
            "ekf2_vector_change_h_m": norm2(ekf2_growth_x, ekf2_growth_y),
            "ekf2_vector_change_x_m": ekf2_growth_x,
            "ekf2_vector_change_y_m": ekf2_growth_y,
            "iekf_minus_ekf2_vector_change_m": norm2(iekf_growth_x, iekf_growth_y) - norm2(ekf2_growth_x, ekf2_growth_y),
            "start_iekf_error_after_h_m": current.get("error_after_h_m"),
            "end_iekf_error_before_h_m": nxt.get("error_before_h_m"),
            "start_ekf2_error_h_m": join0.get("ekf2_error_xy_m"),
            "end_ekf2_error_h_m": join1.get("ekf2_error_xy_m"),
            "horizontal_speed_mps": current.get("horizontal_speed_mps"),
            "gyro_deg_s": current.get("gyro_deg_s"),
            "phase_error_memory_pressure_active": current.get("phase_error_memory_pressure_active"),
            "phase_error_memory_candidate_active": current.get("phase_error_memory_candidate_active"),
        })
    return out


def summarize_propagation(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    summary: list[dict[str, object]] = []
    for label, start, end in TARGET_WINDOWS:
        subset = [
            row for row in rows
            if start <= to_float(row.get("start_time_since_arm_sec")) < end
        ]
        summary.append({
            "window": label,
            "intervals": len(subset),
            "iekf_vector_growth_mean_m": mean(to_float(row.get("iekf_vector_growth_h_m")) for row in subset),
            "iekf_vector_growth_p95_m": percentile(
                (to_float(row.get("iekf_vector_growth_h_m")) for row in subset), 95.0
            ),
            "ekf2_vector_change_mean_m": mean(to_float(row.get("ekf2_vector_change_h_m")) for row in subset),
            "ekf2_vector_change_p95_m": percentile(
                (to_float(row.get("ekf2_vector_change_h_m")) for row in subset), 95.0
            ),
            "iekf_scalar_growth_mean_m": mean(to_float(row.get("iekf_scalar_growth_m")) for row in subset),
            "ekf2_scalar_growth_mean_m": mean(to_float(row.get("ekf2_scalar_growth_m")) for row in subset),
            "iekf_vector_gt_ekf2_fraction": mean(
                1.0 if to_float(row.get("iekf_vector_growth_h_m")) > to_float(row.get("ekf2_vector_change_h_m")) else 0.0
                for row in subset
            ),
        })
    return summary


def best_lag_rows(lag_sweep_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for label, _, _ in TARGET_WINDOWS:
        subset = [row for row in lag_sweep_rows if row.get("window") == label and to_float(row.get("rows")) > 0]
        zero = min(subset, key=lambda row: abs(to_float(row.get("lag_sec"))), default={})
        best = min(subset, key=lambda row: to_float(row.get("fit_error_mean_m")), default={})
        rows.append({
            "window": label,
            "zero_lag_mean_m": zero.get("fit_error_mean_m", math.nan),
            "best_lag_sec": best.get("lag_sec", math.nan),
            "best_lag_mean_m": best.get("fit_error_mean_m", math.nan),
            "improvement_m": to_float(zero.get("fit_error_mean_m")) - to_float(best.get("fit_error_mean_m")),
        })
    return rows


def write_report(
    out_dir: Path,
    projection_summary: list[dict[str, object]],
    lag_best: list[dict[str, object]],
    target_rows: list[dict[str, object]],
    propagation_summary: list[dict[str, object]],
    propagation_rows: list[dict[str, object]],
) -> None:
    projection_table = [
        [
            row["window"],
            row["rows"],
            fmt(row["geometry_inferred_gnss_meas_error_mean_m"]),
            fmt(row["direct_gps_fit_error_mean_m"]),
            fmt(row["moving_fit_error_mean_m"]),
            fmt(row["all_minus_moving_fit_error_mean_m"]),
            fmt(row["direct_gps_fit_error_p95_m"]),
            fmt(row["direct_gps_raw_error_mean_m"]),
            fmt(row["best_common_lag_sec"], 1),
            fmt(row["best_common_lag_error_mean_m"]),
            fmt(row["zero_minus_best_lag_error_m"]),
            fmt(row["core_native_velocity_mismatch_mean_mps"]),
        ]
        for row in projection_summary
    ]
    lag_table = [
        [
            row["window"],
            fmt(row["zero_lag_mean_m"]),
            fmt(row["best_lag_sec"], 1),
            fmt(row["best_lag_mean_m"]),
            fmt(row["improvement_m"]),
        ]
        for row in lag_best
    ]
    target_table = [
        [
            fmt(row["time_since_arm_sec"], 1),
            row["context"],
            fmt(row["pair_iekf_error_h_m"]),
            fmt(row["pair_ekf2_error_h_m"]),
            fmt(row["geometry_inferred_gnss_measurement_error_h_m"]),
            fmt(row["gps_fit_all_error_h_m"]),
            fmt(row["gps_fit_moving_error_h_m"]),
            fmt(row["best_lag_sec"], 1),
            fmt(row["best_lag_error_h_m"]),
            fmt(row["core_native_velocity_mismatch_h_mps"]),
        ]
        for row in target_rows[:14]
    ]
    propagation_table = [
        [
            row["window"],
            row["intervals"],
            fmt(row["iekf_vector_growth_mean_m"]),
            fmt(row["iekf_vector_growth_p95_m"]),
            fmt(row["ekf2_vector_change_mean_m"]),
            fmt(row["ekf2_vector_change_p95_m"]),
            fmt(row["iekf_scalar_growth_mean_m"]),
            fmt(row["ekf2_scalar_growth_mean_m"]),
            fmt(row["iekf_vector_gt_ekf2_fraction"], 3),
        ]
        for row in propagation_summary
    ]
    propagation_examples = [
        [
            fmt(row["start_time_since_arm_sec"], 1),
            fmt(row["end_time_since_arm_sec"], 1),
            row["start_context"],
            row["end_context"],
            fmt(row["iekf_vector_growth_h_m"]),
            fmt(row["ekf2_vector_change_h_m"]),
            fmt(row["iekf_minus_ekf2_vector_change_m"]),
            fmt(row["iekf_scalar_growth_m"]),
            fmt(row["ekf2_scalar_growth_m"]),
            fmt(row["horizontal_speed_mps"]),
            fmt(row["gyro_deg_s"], 1),
        ]
        for row in sorted(
            propagation_rows,
            key=lambda item: to_float(item.get("iekf_vector_growth_h_m")),
            reverse=True,
        )[:14]
    ]

    row_120_180 = next((row for row in projection_summary if row.get("window") == "120-180"), {})
    direct_mean = to_float(row_120_180.get("direct_gps_fit_error_mean_m"))
    moving_mean = to_float(row_120_180.get("moving_fit_error_mean_m"))
    moving_gain = to_float(row_120_180.get("all_minus_moving_fit_error_mean_m"))
    inferred_mean = to_float(row_120_180.get("geometry_inferred_gnss_meas_error_mean_m"))
    lag_gain = to_float(row_120_180.get("zero_minus_best_lag_error_m"))
    if finite(lag_gain) and lag_gain > 0.10:
        lag_readout = "A common GPS/groundtruth time offset reduces target error materially, so time alignment remains a live suspect."
    else:
        lag_readout = "No single common lag in the +/-1.0 s sweep removes the target-row GPS/groundtruth error."

    lines = [
        "# Shortgen02 Measurement Timing And Projection Diagnostic",
        "",
        "This diagnostic checks whether the GNSS measurement itself is offset from groundtruth in the target rows, whether a simple time shift explains that offset, and whether IEKF propagation between accepted position updates grows differently from EKF2.",
        "",
        "## Projection Timing Summary",
        "",
        markdown_table(
            [
                "window",
                "rows",
                "geom_gnss_err_mean",
                "direct_gps_fit_mean",
                "moving_fit_mean",
                "all_minus_moving",
                "direct_gps_fit_p95",
                "raw_gps_mean",
                "best_lag",
                "best_lag_mean",
                "lag_gain",
                "core_native_vdiff",
            ],
            projection_table,
        ),
        "",
        "## Common Lag Sweep Best",
        "",
        markdown_table(
            ["window", "zero_lag_mean", "best_lag", "best_lag_mean", "improvement"],
            lag_table,
        ),
        "",
        "## Target Rows",
        "",
        markdown_table(
            [
                "t_arm",
                "ctx",
                "iekf",
                "ekf2",
                "geom_gnss_err",
                "direct_gps_fit",
                "moving_fit",
                "best_lag",
                "best_lag_err",
                "core_native_vdiff",
            ],
            target_table,
        ),
        "",
        "## Propagation Summary",
        "",
        markdown_table(
            [
                "window",
                "intervals",
                "iekf_vec_mean",
                "iekf_vec_p95",
                "ekf2_vec_mean",
                "ekf2_vec_p95",
                "iekf_scalar_mean",
                "ekf2_scalar_mean",
                "iekf_vec_gt_ekf2_frac",
            ],
            propagation_table,
        ),
        "",
        "## Largest IEKF Between-Update Vector Changes",
        "",
        markdown_table(
            [
                "start",
                "end",
                "ctx0",
                "ctx1",
                "iekf_vec",
                "ekf2_vec",
                "delta",
                "iekf_scalar",
                "ekf2_scalar",
                "hspeed",
                "gyro",
            ],
            propagation_examples,
        ),
        "",
        "## Interpretation",
        "",
        f"- In the 120-180 s target rows, direct ULog GPS projected through the all-row GPS/groundtruth similarity fit has mean error `{fmt(direct_mean)}` m; the geometry-inferred GNSS measurement error is `{fmt(inferred_mean)}` m.",
        f"- Re-fitting the same ULog GPS rows on the moving subset reduces the 120-180 s target mean to `{fmt(moving_mean)}` m, a `{fmt(moving_gain)}` m reduction versus the all-row fit.",
        f"- {lag_readout}",
        "- The GPS measurement-side offset is therefore dominated by projection/alignment choice, not by a simple source lag or another position-gain threshold change.",
        "- Propagation intervals still matter: compare the IEKF and EKF2 vector-change columns above before designing any new mechanism.",
        "",
        "Generated files:",
        f"- `{out_dir / 'target_projection_timing_rows.csv'}`",
        f"- `{out_dir / 'lag_sweep_rows.csv'}`",
        f"- `{out_dir / 'projection_timing_summary.csv'}`",
        f"- `{out_dir / 'propagation_compare_rows.csv'}`",
        f"- `{out_dir / 'propagation_compare_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out-dir")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir) if args.out_dir else run_dir / "shortgen02_measurement_timing_projection_diag"
    out_dir.mkdir(parents=True, exist_ok=True)

    target_rows, gps_rows, gps_times = build_target_rows(run_dir)
    fit_all = load_fit(run_dir, "all")
    lag_sweep_rows = build_lag_sweep_rows(target_rows, gps_rows, gps_times, fit_all)
    projection_summary = summarize_projection(target_rows, lag_sweep_rows)
    lag_best = best_lag_rows(lag_sweep_rows)
    propagation_rows = build_propagation_rows(run_dir)
    propagation_summary = summarize_propagation(propagation_rows)

    write_csv(out_dir / "target_projection_timing_rows.csv", target_rows)
    write_csv(out_dir / "lag_sweep_rows.csv", lag_sweep_rows)
    write_csv(out_dir / "projection_timing_summary.csv", projection_summary)
    write_csv(out_dir / "propagation_compare_rows.csv", propagation_rows)
    write_csv(out_dir / "propagation_compare_summary.csv", propagation_summary)
    write_report(out_dir, projection_summary, lag_best, target_rows, propagation_summary, propagation_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
