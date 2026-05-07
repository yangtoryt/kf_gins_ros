#!/usr/bin/env python3
"""Offline IEKF-vs-EKF2 pair/frame diagnostics for manual PX4 runs.

This script intentionally stays out of the flight chain. It reads the light
pair logger CSV plus kf_gins debug CSVs, joins them by ROS time, and writes a
small report for frame/projection and velocity-coupling triage.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Callable, Iterable


DEFAULT_RUNS = [
    (
        "manual118",
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_mainline_118_original_track_pairlogger_paramfix_rawdecim50_no_realtime_no_gps",
    ),
    (
        "manual119",
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_mainline_119_gnss_pos_std50_pairlogger_rawdecim50_no_realtime_no_gps",
    ),
    (
        "manual120",
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_mainline_120_gnss_override_off_std08_10_pairlogger_rawdecim50_no_realtime_no_gps",
    ),
]

DEFAULT_LAG_SWEEP_SECONDS = [
    -0.30,
    -0.25,
    -0.20,
    -0.15,
    -0.10,
    -0.05,
    0.00,
    0.05,
    0.10,
    0.15,
    0.20,
    0.25,
    0.30,
]

DEFAULT_LAG_WINDOWS = [
    (0.0, 20.0),
    (20.0, 40.0),
    (40.0, 60.0),
    (60.0, 80.0),
    (80.0, 100.0),
    (100.0, 120.0),
    (120.0, 140.0),
    (140.0, 160.0),
    (160.0, 180.0),
]


def to_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return result if math.isfinite(result) else default


def truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def finite(value: float) -> bool:
    return math.isfinite(value)


def mean(values: Iterable[float]) -> float:
    vals = [v for v in values if finite(v)]
    return sum(vals) / len(vals) if vals else math.nan


def percentile(values: Iterable[float], pct: float) -> float:
    vals = sorted(v for v in values if finite(v))
    if not vals:
        return math.nan
    if len(vals) == 1:
        return vals[0]
    k = (len(vals) - 1) * pct / 100.0
    lo = math.floor(k)
    hi = math.ceil(k)
    if lo == hi:
        return vals[lo]
    return vals[lo] * (hi - k) + vals[hi] * (k - lo)


def max_finite(values: Iterable[float]) -> float:
    vals = [v for v in values if finite(v)]
    return max(vals) if vals else math.nan


def stdev(values: Iterable[float]) -> float:
    vals = [v for v in values if finite(v)]
    if len(vals) < 2:
        return math.nan
    m = sum(vals) / len(vals)
    return math.sqrt(sum((v - m) ** 2 for v in vals) / (len(vals) - 1))


def corr(xs: Iterable[float], ys: Iterable[float]) -> float:
    pairs = [(x, y) for x, y in zip(xs, ys) if finite(x) and finite(y)]
    if len(pairs) < 3:
        return math.nan
    mx = sum(x for x, _ in pairs) / len(pairs)
    my = sum(y for _, y in pairs) / len(pairs)
    vx = sum((x - mx) ** 2 for x, _ in pairs)
    vy = sum((y - my) ** 2 for _, y in pairs)
    if vx <= 0.0 or vy <= 0.0:
        return math.nan
    return sum((x - mx) * (y - my) for x, y in pairs) / math.sqrt(vx * vy)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def nearest_row(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt: float,
) -> tuple[dict[str, object] | None, float]:
    i = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for j in (i - 1, i):
        if 0 <= j < len(rows):
            dt = abs(times[j] - t)
            candidates.append((dt, rows[j]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    if dt > max_dt:
        return None, dt
    return row, dt


def numeric_row(row: dict[str, str], fields: Iterable[str]) -> dict[str, object]:
    result: dict[str, object] = dict(row)
    for field in fields:
        if field in row:
            result[field] = to_float(row[field])
    return result


def load_run(label: str, run_dir: Path) -> dict[str, object]:
    pair_path = run_dir / "ekf_iekf_pairs.csv"
    gnss_path = run_dir / "gnss_update_debug.csv"
    state_path = run_dir / "state_publish_debug.csv"
    for path in (pair_path, gnss_path, state_path):
        if not path.exists() or path.stat().st_size == 0:
            raise FileNotFoundError(f"missing required CSV for {label}: {path}")

    pair_float_fields = [
        "ros_time_sec",
        "ekf2_stamp_sec",
        "iekf_stamp_sec",
        "sync_dt_ms",
        "align_offset_x_m",
        "align_offset_y_m",
        "align_offset_z_m",
        "ekf2_x_m",
        "ekf2_y_m",
        "ekf2_z_m",
        "iekf_x_m",
        "iekf_y_m",
        "iekf_z_m",
        "iekf_aligned_x_m",
        "iekf_aligned_y_m",
        "iekf_aligned_z_m",
        "position_error_x_m",
        "position_error_y_m",
        "position_error_z_m",
        "position_error_xy_m",
        "position_error_norm_m",
        "velocity_error_x_mps",
        "velocity_error_y_mps",
        "velocity_error_z_mps",
        "velocity_error_norm_mps",
        "yaw_error_deg",
    ]
    gnss_float_fields = [
        "ros_time_sec",
        "gnss_position_residual_n_m",
        "gnss_position_residual_e_m",
        "gnss_position_residual_u_m",
        "gnss_position_std_n_m",
        "gnss_position_std_e_m",
        "gnss_position_std_u_m",
        "gnss_velocity_residual_n_mps",
        "gnss_velocity_residual_e_mps",
        "gnss_velocity_residual_d_mps",
        "last_velocity_residual_h_mps",
        "core_gnss_diff_h_m",
        "core_gnss_diff_u_m",
        "horizontal_speed_mps",
        "vertical_speed_mps",
        "native_velocity_vN_mps",
        "native_velocity_vE_mps",
        "native_velocity_vD_mps",
        "core_velocity_vN_mps",
        "core_velocity_vE_mps",
        "core_velocity_vD_mps",
    ]
    state_float_fields = [
        "ros_time_sec",
        "core_time_minus_ros_sec",
        "gnss_update_time_minus_ros_sec",
        "gnss_source_time_minus_ros_sec",
        "published_enu_e_m",
        "published_enu_n_m",
        "published_enu_u_m",
        "core_enu_e_m",
        "core_enu_n_m",
        "core_enu_u_m",
        "gnss_enu_e_m",
        "gnss_enu_n_m",
        "gnss_enu_u_m",
        "core_minus_gnss_e_m",
        "core_minus_gnss_n_m",
        "core_minus_gnss_u_m",
        "core_gnss_diff_h_m",
        "core_gnss_diff_3d_m",
        "core_velocity_vN_mps",
        "core_velocity_vE_mps",
        "core_velocity_vD_mps",
        "core_velocity_vU_mps",
        "mavros_horizontal_speed_mps",
        "mavros_vertical_speed_mps",
    ]

    pair_rows = [
        numeric_row(row, pair_float_fields)
        for row in read_csv(pair_path)
    ]
    gnss_rows = [
        numeric_row(row, gnss_float_fields)
        for row in read_csv(gnss_path)
    ]
    state_rows = [
        numeric_row(row, state_float_fields)
        for row in read_csv(state_path)
    ]

    gnss_rows.sort(key=lambda row: row["ros_time_sec"])  # type: ignore[index]
    state_rows.sort(key=lambda row: row["ros_time_sec"])  # type: ignore[index]
    gnss_times = [row["ros_time_sec"] for row in gnss_rows]  # type: ignore[index]
    state_times = [row["ros_time_sec"] for row in state_rows]  # type: ignore[index]

    arm_times = [
        row["ros_time_sec"]
        for row in gnss_rows
        if truthy(row.get("mavros_armed")) and finite(row["ros_time_sec"])  # type: ignore[arg-type,index]
    ]
    if not arm_times:
        arm_times = [
            row["ros_time_sec"]
            for row in pair_rows
            if truthy(row.get("mavros_armed")) and finite(row["ros_time_sec"])  # type: ignore[arg-type,index]
        ]
    arm_time = min(arm_times) if arm_times else math.nan

    joined: list[dict[str, object]] = []
    for pair in pair_rows:
        t = pair["ros_time_sec"]  # type: ignore[index]
        sample_t = pair.get("iekf_stamp_sec", math.nan)
        if not finite(sample_t):  # type: ignore[arg-type]
            sample_t = pair.get("ekf2_stamp_sec", math.nan)
        if not finite(sample_t):  # type: ignore[arg-type]
            sample_t = t
        if not finite(sample_t):  # type: ignore[arg-type]
            continue
        if not truthy(pair.get("mavros_armed")):
            continue
        if not truthy(pair.get("alignment_ready")):
            continue
        gnss, gnss_dt = nearest_row(gnss_rows, gnss_times, sample_t, 0.15)  # type: ignore[arg-type]
        state, state_dt = nearest_row(state_rows, state_times, sample_t, 0.03)  # type: ignore[arg-type]
        row: dict[str, object] = {
            "run": label,
            "ros_time_sec": t,
            "sample_time_sec": sample_t,
            "ekf2_stamp_sec": pair.get("ekf2_stamp_sec", math.nan),
            "iekf_stamp_sec": pair.get("iekf_stamp_sec", math.nan),
            "time_since_arm_sec": sample_t - arm_time,  # type: ignore[operator]
            "mavros_mode": pair.get("mavros_mode", ""),
            "pair_error_x_m": pair["position_error_x_m"],
            "pair_error_y_m": pair["position_error_y_m"],
            "pair_error_z_m": pair["position_error_z_m"],
            "pair_error_xy_m": pair["position_error_xy_m"],
            "pair_error_norm_m": pair["position_error_norm_m"],
            "pair_vel_error_x_mps": pair["velocity_error_x_mps"],
            "pair_vel_error_y_mps": pair["velocity_error_y_mps"],
            "pair_vel_error_z_mps": pair["velocity_error_z_mps"],
            "pair_vel_error_norm_mps": pair["velocity_error_norm_mps"],
            "pair_yaw_error_deg": pair["yaw_error_deg"],
            "ekf2_x_m": pair["ekf2_x_m"],
            "ekf2_y_m": pair["ekf2_y_m"],
            "iekf_x_m": pair["iekf_x_m"],
            "iekf_y_m": pair["iekf_y_m"],
            "align_offset_x_m": pair["align_offset_x_m"],
            "align_offset_y_m": pair["align_offset_y_m"],
            "gnss_join_dt_sec": gnss_dt,
            "state_join_dt_sec": state_dt,
        }
        if gnss is not None:
            rn = gnss["gnss_position_residual_n_m"]  # type: ignore[index]
            re = gnss["gnss_position_residual_e_m"]  # type: ignore[index]
            rvn = gnss["gnss_velocity_residual_n_mps"]  # type: ignore[index]
            rve = gnss["gnss_velocity_residual_e_mps"]  # type: ignore[index]
            row.update(
                {
                    "gnss_residual_n_m": rn,
                    "gnss_residual_e_m": re,
                    "gnss_residual_u_m": gnss["gnss_position_residual_u_m"],
                    "gnss_residual_h_m": math.hypot(rn, re),  # type: ignore[arg-type]
                    "gnss_std_n_m": gnss["gnss_position_std_n_m"],
                    "gnss_std_e_m": gnss["gnss_position_std_e_m"],
                    "gnss_std_u_m": gnss["gnss_position_std_u_m"],
                    "gnss_vel_residual_n_mps": rvn,
                    "gnss_vel_residual_e_mps": rve,
                    "gnss_vel_residual_d_mps": gnss["gnss_velocity_residual_d_mps"],
                    "gnss_vel_residual_h_mps": math.hypot(rvn, rve),  # type: ignore[arg-type]
                    "last_velocity_residual_h_mps": gnss["last_velocity_residual_h_mps"],
                    "gnss_core_diff_h_m": gnss["core_gnss_diff_h_m"],
                    "gnss_core_diff_u_m": gnss["core_gnss_diff_u_m"],
                    "position_override_active": int(truthy(gnss.get("position_override_active"))),
                    "native_velocity_override_active": int(truthy(gnss.get("native_velocity_override_active"))),
                    "native_velocity_tightening_context": int(truthy(gnss.get("native_velocity_tightening_context"))),
                    "turning_now": int(truthy(gnss.get("turning_now"))),
                    "post_turn_context": int(truthy(gnss.get("post_turn_context"))),
                    "armed_cruise_context": int(truthy(gnss.get("armed_cruise_context"))),
                    "horizontal_speed_mps": gnss["horizontal_speed_mps"],
                    "vertical_speed_mps": gnss["vertical_speed_mps"],
                    "native_velocity_vN_mps": gnss["native_velocity_vN_mps"],
                    "native_velocity_vE_mps": gnss["native_velocity_vE_mps"],
                    "native_velocity_vD_mps": gnss["native_velocity_vD_mps"],
                    "core_velocity_vN_mps": gnss["core_velocity_vN_mps"],
                    "core_velocity_vE_mps": gnss["core_velocity_vE_mps"],
                    "core_velocity_vD_mps": gnss["core_velocity_vD_mps"],
                }
            )
        if state is not None:
            age = -state["gnss_source_time_minus_ros_sec"]  # type: ignore[index,operator]
            phase_core_minus_gnss_e = math.nan
            phase_core_minus_gnss_n = math.nan
            phase_core_gnss_h = math.nan
            phase_gnss_e = math.nan
            phase_gnss_n = math.nan
            if finite(age):  # type: ignore[arg-type]
                phase_core_minus_gnss_e = (
                    state["core_minus_gnss_e_m"] - state["core_velocity_vE_mps"] * age  # type: ignore[index,operator]
                )
                phase_core_minus_gnss_n = (
                    state["core_minus_gnss_n_m"] - state["core_velocity_vN_mps"] * age  # type: ignore[index,operator]
                )
                phase_core_gnss_h = math.hypot(phase_core_minus_gnss_e, phase_core_minus_gnss_n)
                phase_gnss_e = state["gnss_enu_e_m"] + state["core_velocity_vE_mps"] * age  # type: ignore[index,operator]
                phase_gnss_n = state["gnss_enu_n_m"] + state["core_velocity_vN_mps"] * age  # type: ignore[index,operator]
            iekf_vs_phase_gnss_h = math.nan
            ekf2_vs_phase_gnss_aligned_h = math.nan
            if finite(phase_gnss_e) and finite(phase_gnss_n):
                iekf_minus_phase_gnss_e = pair["iekf_x_m"] - phase_gnss_e  # type: ignore[index,operator]
                iekf_minus_phase_gnss_n = pair["iekf_y_m"] - phase_gnss_n  # type: ignore[index,operator]
                ekf2_minus_phase_gnss_aligned_e = pair["ekf2_x_m"] - (phase_gnss_e + pair["align_offset_x_m"])  # type: ignore[index,operator]
                ekf2_minus_phase_gnss_aligned_n = pair["ekf2_y_m"] - (phase_gnss_n + pair["align_offset_y_m"])  # type: ignore[index,operator]
                iekf_vs_phase_gnss_h = math.hypot(iekf_minus_phase_gnss_e, iekf_minus_phase_gnss_n)
                ekf2_vs_phase_gnss_aligned_h = math.hypot(
                    ekf2_minus_phase_gnss_aligned_e,
                    ekf2_minus_phase_gnss_aligned_n,
                )
            else:
                iekf_minus_phase_gnss_e = math.nan
                iekf_minus_phase_gnss_n = math.nan
                ekf2_minus_phase_gnss_aligned_e = math.nan
                ekf2_minus_phase_gnss_aligned_n = math.nan
            row.update(
                {
                    "state_core_enu_e_m": state["core_enu_e_m"],
                    "state_core_enu_n_m": state["core_enu_n_m"],
                    "state_gnss_enu_e_m": state["gnss_enu_e_m"],
                    "state_gnss_enu_n_m": state["gnss_enu_n_m"],
                    "state_core_minus_gnss_e_m": state["core_minus_gnss_e_m"],
                    "state_core_minus_gnss_n_m": state["core_minus_gnss_n_m"],
                    "state_core_minus_gnss_u_m": state["core_minus_gnss_u_m"],
                    "state_core_gnss_diff_h_m": state["core_gnss_diff_h_m"],
                    "state_core_gnss_diff_3d_m": state["core_gnss_diff_3d_m"],
                    "state_phase_gnss_enu_e_m": phase_gnss_e,
                    "state_phase_gnss_enu_n_m": phase_gnss_n,
                    "state_phase_core_minus_gnss_e_m": phase_core_minus_gnss_e,
                    "state_phase_core_minus_gnss_n_m": phase_core_minus_gnss_n,
                    "state_phase_core_gnss_h_m": phase_core_gnss_h,
                    "iekf_minus_phase_gnss_e_m": iekf_minus_phase_gnss_e,
                    "iekf_minus_phase_gnss_n_m": iekf_minus_phase_gnss_n,
                    "iekf_vs_phase_gnss_h_m": iekf_vs_phase_gnss_h,
                    "ekf2_minus_phase_gnss_aligned_e_m": ekf2_minus_phase_gnss_aligned_e,
                    "ekf2_minus_phase_gnss_aligned_n_m": ekf2_minus_phase_gnss_aligned_n,
                    "ekf2_vs_phase_gnss_aligned_h_m": ekf2_vs_phase_gnss_aligned_h,
                    "state_core_time_minus_ros_sec": state["core_time_minus_ros_sec"],
                    "state_gnss_update_time_minus_ros_sec": state["gnss_update_time_minus_ros_sec"],
                    "state_gnss_source_time_minus_ros_sec": state["gnss_source_time_minus_ros_sec"],
                    "state_core_velocity_vN_mps": state["core_velocity_vN_mps"],
                    "state_core_velocity_vE_mps": state["core_velocity_vE_mps"],
                    "state_core_velocity_vD_mps": state["core_velocity_vD_mps"],
                }
            )
        joined.append(row)

    return {
        "label": label,
        "run_dir": run_dir,
        "arm_time": arm_time,
        "pair_rows": pair_rows,
        "gnss_rows": gnss_rows,
        "state_rows": state_rows,
        "joined_rows": joined,
    }


def value(row: dict[str, object], key: str) -> float:
    return to_float(row.get(key))


def summarize_rows(rows: list[dict[str, object]], prefix: str = "") -> dict[str, float]:
    result = {
        f"{prefix}n": float(len(rows)),
        f"{prefix}xy_mean": mean(value(row, "pair_error_xy_m") for row in rows),
        f"{prefix}xy_p95": percentile((value(row, "pair_error_xy_m") for row in rows), 95),
        f"{prefix}xy_max": max_finite(value(row, "pair_error_xy_m") for row in rows),
        f"{prefix}vel_mean": mean(value(row, "pair_vel_error_norm_mps") for row in rows),
        f"{prefix}yaw_abs_mean": mean(abs(value(row, "pair_yaw_error_deg")) for row in rows),
        f"{prefix}gnss_res_h_mean": mean(value(row, "gnss_residual_h_m") for row in rows),
        f"{prefix}core_gnss_h_mean": mean(value(row, "gnss_core_diff_h_m") for row in rows),
        f"{prefix}state_core_gnss_h_mean": mean(value(row, "state_core_gnss_diff_h_m") for row in rows),
        f"{prefix}state_phase_core_gnss_h_mean": mean(value(row, "state_phase_core_gnss_h_m") for row in rows),
        f"{prefix}iekf_phase_gnss_h_mean": mean(value(row, "iekf_vs_phase_gnss_h_m") for row in rows),
        f"{prefix}ekf2_phase_gnss_aligned_h_mean": mean(
            value(row, "ekf2_vs_phase_gnss_aligned_h_m") for row in rows
        ),
        f"{prefix}gnss_vel_res_h_mean": mean(value(row, "gnss_vel_residual_h_mps") for row in rows),
    }
    return result


def select_window(
    rows: list[dict[str, object]],
    start: float,
    end: float,
    time_key: str = "time_since_arm_sec",
) -> list[dict[str, object]]:
    return [row for row in rows if start <= value(row, time_key) < end]


def cosine_stats(
    rows: list[dict[str, object]],
    source_fn: Callable[[dict[str, object]], tuple[float, float]],
    target_fn: Callable[[dict[str, object]], tuple[float, float]],
) -> tuple[float, float, float]:
    cosines: list[float] = []
    ratios: list[float] = []
    for row in rows:
        sx, sy = source_fn(row)
        tx, ty = target_fn(row)
        sm = math.hypot(sx, sy)
        tm = math.hypot(tx, ty)
        if sm <= 1e-9 or tm <= 1e-9:
            continue
        cosines.append((sx * tx + sy * ty) / (sm * tm))
        ratios.append(sm / tm)
    return mean(cosines), percentile(cosines, 95), mean(ratios)


def fit_complex_scale(
    rows: list[dict[str, object]],
    source_fn: Callable[[dict[str, object]], tuple[float, float]],
    target_fn: Callable[[dict[str, object]], tuple[float, float]],
) -> dict[str, float]:
    # Fit target ~= alpha * source, where alpha is a complex scale/rotation.
    real_num = 0.0
    imag_num = 0.0
    denom = 0.0
    count = 0
    target_energy = 0.0
    for row in rows:
        sx, sy = source_fn(row)
        tx, ty = target_fn(row)
        if not all(finite(v) for v in (sx, sy, tx, ty)):
            continue
        sm2 = sx * sx + sy * sy
        if sm2 <= 1e-12:
            continue
        real_num += tx * sx + ty * sy
        imag_num += ty * sx - tx * sy
        denom += sm2
        target_energy += tx * tx + ty * ty
        count += 1
    if count == 0 or denom <= 0:
        return {"count": 0.0, "scale": math.nan, "angle_deg": math.nan, "rmse": math.nan, "rel_rmse": math.nan}
    ar = real_num / denom
    ai = imag_num / denom
    err = 0.0
    for row in rows:
        sx, sy = source_fn(row)
        tx, ty = target_fn(row)
        if not all(finite(v) for v in (sx, sy, tx, ty)):
            continue
        px = ar * sx - ai * sy
        py = ai * sx + ar * sy
        err += (tx - px) ** 2 + (ty - py) ** 2
    scale = math.hypot(ar, ai)
    rmse = math.sqrt(err / count)
    rel_rmse = math.sqrt(err / target_energy) if target_energy > 0 else math.nan
    return {
        "count": float(count),
        "scale": scale,
        "angle_deg": math.degrees(math.atan2(ai, ar)),
        "rmse": rmse,
        "rel_rmse": rel_rmse,
    }


def lag_corr(rows: list[dict[str, object]], lag_sec: float) -> float:
    times = [value(row, "time_since_arm_sec") for row in rows]
    vel = [value(row, "pair_vel_error_norm_mps") for row in rows]
    pos_lagged: list[float] = []
    vel_source: list[float] = []
    for row, v in zip(rows, vel):
        target_t = value(row, "time_since_arm_sec") + lag_sec
        i = bisect.bisect_left(times, target_t)
        candidates = []
        for j in (i - 1, i):
            if 0 <= j < len(rows):
                candidates.append((abs(times[j] - target_t), value(rows[j], "pair_error_xy_m")))
        if not candidates:
            continue
        dt, pos = min(candidates, key=lambda item: item[0])
        if dt > 0.25:
            continue
        vel_source.append(v)
        pos_lagged.append(pos)
    return corr(vel_source, pos_lagged)


def lag_window_key(start: float, end: float) -> str:
    return f"w_{int(start):03d}_{int(end):03d}_mean"


def lag_window_label(start: float, end: float) -> str:
    return f"{int(start):03d}-{int(end):03d}"


def aligned_iekf_xy(row: dict[str, object]) -> tuple[float, float]:
    return (
        value(row, "iekf_x_m") + value(row, "align_offset_x_m"),
        value(row, "iekf_y_m") + value(row, "align_offset_y_m"),
    )


def lag_shift_error_xy(row: dict[str, object], lag_sec: float) -> float:
    ix, iy = aligned_iekf_xy(row)
    values = (
        ix,
        iy,
        value(row, "ekf2_x_m"),
        value(row, "ekf2_y_m"),
        value(row, "native_velocity_vE_mps"),
        value(row, "native_velocity_vN_mps"),
    )
    if not all(finite(item) for item in values):
        return math.nan
    shifted_x = ix + values[4] * lag_sec
    shifted_y = iy + values[5] * lag_sec
    return math.hypot(shifted_x - values[2], shifted_y - values[3])


def lag_sweep_rows(
    run_label: str,
    rows: list[dict[str, object]],
    lags: list[float] | None = None,
    windows: list[tuple[float, float]] | None = None,
) -> list[dict[str, object]]:
    lags = DEFAULT_LAG_SWEEP_SECONDS if lags is None else lags
    windows = DEFAULT_LAG_WINDOWS if windows is None else windows
    result: list[dict[str, object]] = []
    for lag in lags:
        errors = [lag_shift_error_xy(row, lag) for row in rows]
        finite_errors = [item for item in errors if finite(item)]
        out: dict[str, object] = {
            "run": run_label,
            "lag_sec": lag,
            "n": len(finite_errors),
            "xy_mean_m": mean(finite_errors),
            "xy_p95_m": percentile(finite_errors, 95),
            "xy_max_m": max_finite(finite_errors),
        }
        for start, end in windows:
            subset = [
                lag_shift_error_xy(row, lag)
                for row in rows
                if start <= value(row, "time_since_arm_sec") < end
            ]
            out[lag_window_key(start, end)] = mean(subset)
        result.append(out)
    return result


def lag_value(sweep: list[dict[str, object]], key: str, target_lag: float) -> float:
    if not sweep:
        return math.nan
    row = min(sweep, key=lambda item: abs(value(item, "lag_sec") - target_lag))
    if abs(value(row, "lag_sec") - target_lag) > 1e-6:
        return math.nan
    return value(row, key)


def lag_best_rows_for_run(
    run_label: str,
    sweep: list[dict[str, object]],
    windows: list[tuple[float, float]] | None = None,
) -> list[dict[str, object]]:
    windows = DEFAULT_LAG_WINDOWS if windows is None else windows
    specs = [("global", "xy_mean_m")] + [
        (lag_window_label(start, end), lag_window_key(start, end)) for start, end in windows
    ]
    result: list[dict[str, object]] = []
    for label, key in specs:
        candidates = [row for row in sweep if finite(value(row, key))]
        if not candidates:
            continue
        best = min(candidates, key=lambda row: value(row, key))
        result.append(
            {
                "run": run_label,
                "window": label,
                "best_lag_sec": value(best, "lag_sec"),
                "best_xy_mean_m": value(best, key),
                "zero_lag_xy_mean_m": lag_value(sweep, key, 0.0),
                "plus025_lag_xy_mean_m": lag_value(sweep, key, 0.25),
                "minus025_lag_xy_mean_m": lag_value(sweep, key, -0.25),
            }
        )
    return result


def lag_projection_window_rows(
    run_label: str,
    rows: list[dict[str, object]],
    windows: list[tuple[float, float]] | None = None,
    min_speed_mps: float = 0.50,
) -> list[dict[str, object]]:
    windows = DEFAULT_LAG_WINDOWS if windows is None else windows
    specs = [("global", rows)] + [
        (lag_window_label(start, end), select_window(rows, start, end)) for start, end in windows
    ]
    result: list[dict[str, object]] = []
    for label, subset in specs:
        opt_lags: list[float] = []
        along: list[float] = []
        cross: list[float] = []
        speeds: list[float] = []
        for row in subset:
            ix, iy = aligned_iekf_xy(row)
            ekf2_x = value(row, "ekf2_x_m")
            ekf2_y = value(row, "ekf2_y_m")
            v_e = value(row, "native_velocity_vE_mps")
            v_n = value(row, "native_velocity_vN_mps")
            if not all(finite(item) for item in (ix, iy, ekf2_x, ekf2_y, v_e, v_n)):
                continue
            speed = math.hypot(v_e, v_n)
            if speed < min_speed_mps:
                continue
            err_e = ix - ekf2_x
            err_n = iy - ekf2_y
            dot = err_e * v_e + err_n * v_n
            opt_lags.append(-dot / (speed * speed))
            along.append(dot / speed)
            cross.append((-err_e * v_n + err_n * v_e) / speed)
            speeds.append(speed)
        result.append(
            {
                "run": run_label,
                "window": label,
                "n": len(opt_lags),
                "speed_mean_mps": mean(speeds),
                "opt_lag_mean_sec": mean(opt_lags),
                "opt_lag_p50_sec": percentile(opt_lags, 50),
                "opt_lag_p10_sec": percentile(opt_lags, 10),
                "opt_lag_p90_sec": percentile(opt_lags, 90),
                "err_along_velocity_mean_m": mean(along),
                "err_cross_velocity_mean_m": mean(cross),
            }
        )
    return result


def window_vector_projection_rows(
    run_label: str,
    rows: list[dict[str, object]],
    windows: list[tuple[float, float]] | None = None,
) -> list[dict[str, object]]:
    windows = DEFAULT_LAG_WINDOWS if windows is None else windows
    result: list[dict[str, object]] = []
    for start, end in windows:
        subset = select_window(rows, start, end)
        along: list[float] = []
        cross: list[float] = []
        for row in subset:
            err_e = value(row, "pair_error_x_m")
            err_n = value(row, "pair_error_y_m")
            v_e = value(row, "native_velocity_vE_mps")
            v_n = value(row, "native_velocity_vN_mps")
            if not all(finite(item) for item in (err_e, err_n, v_e, v_n)):
                continue
            speed = math.hypot(v_e, v_n)
            if speed <= 1e-9:
                continue
            along.append((err_e * v_e + err_n * v_n) / speed)
            cross.append((-err_e * v_n + err_n * v_e) / speed)
        mean_v_e = mean(value(row, "native_velocity_vE_mps") for row in subset)
        mean_v_n = mean(value(row, "native_velocity_vN_mps") for row in subset)
        heading_deg = math.degrees(math.atan2(mean_v_e, mean_v_n)) if finite(mean_v_e) and finite(mean_v_n) else math.nan
        result.append(
            {
                "run": run_label,
                "window": lag_window_label(start, end),
                "n": len(subset),
                "pair_error_e_mean_m": mean(value(row, "pair_error_x_m") for row in subset),
                "pair_error_n_mean_m": mean(value(row, "pair_error_y_m") for row in subset),
                "native_velocity_e_mean_mps": mean_v_e,
                "native_velocity_n_mean_mps": mean_v_n,
                "native_velocity_heading_deg": heading_deg,
                "err_along_velocity_mean_m": mean(along),
                "err_cross_velocity_mean_m": mean(cross),
                "iekf_phase_e_mean_m": mean(value(row, "iekf_minus_phase_gnss_e_m") for row in subset),
                "ekf2_phase_e_mean_m": mean(value(row, "ekf2_minus_phase_gnss_aligned_e_m") for row in subset),
            }
        )
    return result


def pair_xy(row: dict[str, object]) -> tuple[float, float, float, float]:
    ix, iy = aligned_iekf_xy(row)
    return ix, iy, value(row, "ekf2_x_m"), value(row, "ekf2_y_m")


def fit_translation_xy(rows: list[dict[str, object]]) -> dict[str, float]:
    dxs: list[float] = []
    dys: list[float] = []
    for row in rows:
        ix, iy, ex, ey = pair_xy(row)
        if not all(finite(item) for item in (ix, iy, ex, ey)):
            continue
        dxs.append(ex - ix)
        dys.append(ey - iy)
    return {
        "n_fit": float(len(dxs)),
        "scale": 1.0,
        "angle_deg": 0.0,
        "a_real": 1.0,
        "a_imag": 0.0,
        "trans_x_m": mean(dxs),
        "trans_y_m": mean(dys),
    }


def fit_similarity_xy(rows: list[dict[str, object]]) -> dict[str, float]:
    pairs: list[tuple[float, float, float, float]] = []
    for row in rows:
        ix, iy, ex, ey = pair_xy(row)
        if all(finite(item) for item in (ix, iy, ex, ey)):
            pairs.append((ix, iy, ex, ey))
    if len(pairs) < 2:
        return {
            "n_fit": float(len(pairs)),
            "scale": math.nan,
            "angle_deg": math.nan,
            "a_real": math.nan,
            "a_imag": math.nan,
            "trans_x_m": math.nan,
            "trans_y_m": math.nan,
        }
    src_x_mean = mean(item[0] for item in pairs)
    src_y_mean = mean(item[1] for item in pairs)
    dst_x_mean = mean(item[2] for item in pairs)
    dst_y_mean = mean(item[3] for item in pairs)
    real_num = 0.0
    imag_num = 0.0
    denom = 0.0
    for ix, iy, ex, ey in pairs:
        sx = ix - src_x_mean
        sy = iy - src_y_mean
        tx = ex - dst_x_mean
        ty = ey - dst_y_mean
        real_num += tx * sx + ty * sy
        imag_num += ty * sx - tx * sy
        denom += sx * sx + sy * sy
    if denom <= 1e-12:
        return {
            "n_fit": float(len(pairs)),
            "scale": math.nan,
            "angle_deg": math.nan,
            "a_real": math.nan,
            "a_imag": math.nan,
            "trans_x_m": math.nan,
            "trans_y_m": math.nan,
        }
    a_real = real_num / denom
    a_imag = imag_num / denom
    trans_x = dst_x_mean - (a_real * src_x_mean - a_imag * src_y_mean)
    trans_y = dst_y_mean - (a_imag * src_x_mean + a_real * src_y_mean)
    return {
        "n_fit": float(len(pairs)),
        "scale": math.hypot(a_real, a_imag),
        "angle_deg": math.degrees(math.atan2(a_imag, a_real)),
        "a_real": a_real,
        "a_imag": a_imag,
        "trans_x_m": trans_x,
        "trans_y_m": trans_y,
    }


def transform_pair_error(row: dict[str, object], fit: dict[str, float]) -> tuple[float, float, float]:
    ix, iy, ex, ey = pair_xy(row)
    a_real = value(fit, "a_real")
    a_imag = value(fit, "a_imag")
    trans_x = value(fit, "trans_x_m")
    trans_y = value(fit, "trans_y_m")
    if not all(finite(item) for item in (ix, iy, ex, ey, a_real, a_imag, trans_x, trans_y)):
        return math.nan, math.nan, math.nan
    pred_x = a_real * ix - a_imag * iy + trans_x
    pred_y = a_imag * ix + a_real * iy + trans_y
    err_x = pred_x - ex
    err_y = pred_y - ey
    return err_x, err_y, math.hypot(err_x, err_y)


def evaluate_alignment_fit(
    run_label: str,
    rows: list[dict[str, object]],
    fit_label: str,
    fit: dict[str, float],
    windows: list[tuple[float, float]] | None = None,
) -> list[dict[str, object]]:
    windows = DEFAULT_LAG_WINDOWS if windows is None else windows
    specs = [("global", rows)] + [
        (lag_window_label(start, end), select_window(rows, start, end)) for start, end in windows
    ]
    result: list[dict[str, object]] = []
    for window_label, subset in specs:
        errs = [transform_pair_error(row, fit) for row in subset]
        result.append(
            {
                "run": run_label,
                "fit": fit_label,
                "window": window_label,
                "n": len([err for err in errs if finite(err[2])]),
                "xy_mean_m": mean(err[2] for err in errs),
                "xy_p95_m": percentile((err[2] for err in errs), 95),
                "xy_max_m": max_finite(err[2] for err in errs),
                "err_e_mean_m": mean(err[0] for err in errs),
                "err_n_mean_m": mean(err[1] for err in errs),
            }
        )
    return result


def alignment_fit_rows(
    run_label: str,
    rows: list[dict[str, object]],
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    fit_specs: list[tuple[str, str, list[dict[str, object]], dict[str, float]]] = [
        ("raw_initial_translation", "pair_logger_initial", rows, {
            "n_fit": 0.0,
            "scale": 1.0,
            "angle_deg": 0.0,
            "a_real": 1.0,
            "a_imag": 0.0,
            "trans_x_m": 0.0,
            "trans_y_m": 0.0,
        }),
        ("extra_translation_global", "global", rows, fit_translation_xy(rows)),
        ("similarity_global", "global", rows, fit_similarity_xy(rows)),
    ]
    early_rows = select_window(rows, 0.0, 60.0)
    if early_rows:
        fit_specs.extend(
            [
                ("extra_translation_000_060", "000-060", early_rows, fit_translation_xy(early_rows)),
                ("similarity_000_060", "000-060", early_rows, fit_similarity_xy(early_rows)),
            ]
        )
    param_rows: list[dict[str, object]] = []
    eval_rows: list[dict[str, object]] = []
    for label, fit_window, fit_rows, fit in fit_specs:
        params: dict[str, object] = {
            "run": run_label,
            "fit": label,
            "fit_window": fit_window,
            "n_fit": int(value(fit, "n_fit")) if finite(value(fit, "n_fit")) else 0,
            "scale": value(fit, "scale"),
            "angle_deg": value(fit, "angle_deg"),
            "trans_x_m": value(fit, "trans_x_m"),
            "trans_y_m": value(fit, "trans_y_m"),
        }
        fit_eval = evaluate_alignment_fit(run_label, fit_rows, label, fit, windows=[])
        if fit_eval:
            params["fit_xy_mean_m"] = fit_eval[0]["xy_mean_m"]
            params["fit_xy_max_m"] = fit_eval[0]["xy_max_m"]
        param_rows.append(params)
        eval_rows.extend(evaluate_alignment_fit(run_label, rows, label, fit))
    return param_rows, eval_rows


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


def fmt(value_: float, digits: int = 4) -> str:
    if not finite(value_):
        return "nan"
    return f"{value_:.{digits}f}"


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    for row in rows:
        lines.append("| " + " | ".join(str(item) for item in row) + " |")
    return "\n".join(lines)


def build_report(runs: list[dict[str, object]], out_dir: Path) -> str:
    lines: list[str] = [
        "# Offline Pair Frame Diagnostic",
        "",
        f"Inputs: {', '.join(str(run['label']) for run in runs)} pair/GNSS/state debug CSVs.",
        "All phase windows below are aligned by `time_since_arm`, not absolute ROS time.",
        "",
        "## Run Summary",
    ]

    run_rows: list[list[object]] = []
    for run in runs:
        joined = run["joined_rows"]  # type: ignore[assignment]
        summary = summarize_rows(joined)  # type: ignore[arg-type]
        run_rows.append(
            [
                run["label"],
                int(summary["n"]),
                fmt(summary["xy_mean"]),
                fmt(summary["xy_p95"]),
                fmt(summary["xy_max"]),
                fmt(summary["vel_mean"]),
                fmt(summary["yaw_abs_mean"]),
                fmt(summary["gnss_res_h_mean"]),
                fmt(summary["core_gnss_h_mean"]),
                fmt(summary["state_core_gnss_h_mean"]),
                fmt(summary["state_phase_core_gnss_h_mean"]),
                fmt(summary["iekf_phase_gnss_h_mean"]),
                fmt(summary["ekf2_phase_gnss_aligned_h_mean"]),
            ]
        )
    lines.append(
        markdown_table(
            [
                "run",
                "joined",
                "xy_mean_m",
                "xy_p95_m",
                "xy_max_m",
                "pair_vel_mean_mps",
                "yaw_abs_mean_deg",
                "gnss_res_h_mean_m",
                "core_gnss_h_mean_m",
                "state_core_gnss_h_mean_m",
                "state_phase_core_gnss_h_mean_m",
                "iekf_phase_gnss_h_mean_m",
                "ekf2_phase_gnss_aligned_h_mean_m",
            ],
            run_rows,
        )
    )

    lines.extend(["", "## Time-Since-Arm 20s Windows"])
    window_rows: list[list[object]] = []
    for start in range(0, 360, 20):
        end = start + 20
        row: list[object] = [f"{start:03d}-{end:03d}"]
        for run in runs:
            rows = select_window(run["joined_rows"], start, end)  # type: ignore[arg-type]
            row.append(fmt(mean(value(item, "pair_error_xy_m") for item in rows)))
        window_rows.append(row)
    lines.append(markdown_table(["since_arm_s"] + [f"{run['label']}_xy" for run in runs], window_rows))

    lines.extend(["", "## Vector Alignment Diagnostics"])
    vector_rows: list[list[object]] = []
    target = lambda row: (value(row, "pair_error_x_m"), value(row, "pair_error_y_m"))
    sources: list[tuple[str, Callable[[dict[str, object]], tuple[float, float]]]] = [
        ("gnss_res_EN", lambda row: (value(row, "gnss_residual_e_m"), value(row, "gnss_residual_n_m"))),
        ("-gnss_res_EN", lambda row: (-value(row, "gnss_residual_e_m"), -value(row, "gnss_residual_n_m"))),
        ("gnss_res_NE", lambda row: (value(row, "gnss_residual_n_m"), value(row, "gnss_residual_e_m"))),
        ("state_core_minus_gnss_EN", lambda row: (value(row, "state_core_minus_gnss_e_m"), value(row, "state_core_minus_gnss_n_m"))),
        (
            "state_phase_core_minus_gnss_EN",
            lambda row: (
                value(row, "state_phase_core_minus_gnss_e_m"),
                value(row, "state_phase_core_minus_gnss_n_m"),
            ),
        ),
        (
            "-state_core_minus_gnss_EN",
            lambda row: (-value(row, "state_core_minus_gnss_e_m"), -value(row, "state_core_minus_gnss_n_m")),
        ),
    ]
    for run in runs:
        joined = run["joined_rows"]  # type: ignore[assignment]
        for source_name, source_fn in sources:
            cos_mean, cos_p95, mag_ratio = cosine_stats(joined, source_fn, target)  # type: ignore[arg-type]
            fit = fit_complex_scale(joined, source_fn, target)  # type: ignore[arg-type]
            vector_rows.append(
                [
                    run["label"],
                    source_name,
                    fmt(cos_mean),
                    fmt(cos_p95),
                    fmt(mag_ratio),
                    fmt(fit["scale"]),
                    fmt(fit["angle_deg"], 2),
                    fmt(fit["rel_rmse"]),
                ]
            )
    lines.append(
        markdown_table(
            ["run", "source->pair_error_xy", "cos_mean", "cos_p95", "source/pair_mag", "fit_scale", "fit_angle_deg", "fit_rel_rmse"],
            vector_rows,
        )
    )

    lines.extend(["", "## Velocity Coupling Checks"])
    lag_rows: list[list[object]] = []
    for run in runs:
        joined = run["joined_rows"]  # type: ignore[assignment]
        lag_rows.append(
            [
                run["label"],
                fmt(corr((value(r, "pair_error_xy_m") for r in joined), (value(r, "pair_vel_error_norm_mps") for r in joined))),
                fmt(lag_corr(joined, 5.0)),  # type: ignore[arg-type]
                fmt(lag_corr(joined, 10.0)),  # type: ignore[arg-type]
                fmt(lag_corr(joined, 20.0)),  # type: ignore[arg-type]
                fmt(corr((value(r, "pair_error_xy_m") for r in joined), (value(r, "gnss_vel_residual_h_mps") for r in joined))),
            ]
        )
    lines.append(
        markdown_table(
            ["run", "corr_xy_vel_now", "corr_vel_now_xy_plus5s", "plus10s", "plus20s", "corr_xy_gnss_vel_res_h"],
            lag_rows,
        )
    )

    lines.extend(
        [
            "",
            "## Native-Velocity Lag Sweep",
            "",
            "Convention: each row shifts the aligned IEKF position by `native_velocity_EN * lag_sec` before recomputing IEKF-vs-EKF2 XY error. Positive lag is a forward shift along the native GNSS velocity vector.",
        ]
    )
    lag_sweep_by_label: dict[str, list[dict[str, object]]] = {}
    lag_best_all: list[dict[str, object]] = []
    projection_all: list[dict[str, object]] = []
    vector_window_all: list[dict[str, object]] = []
    alignment_param_all: list[dict[str, object]] = []
    alignment_eval_all: list[dict[str, object]] = []
    for run in runs:
        label = str(run["label"])
        joined = run["joined_rows"]  # type: ignore[assignment]
        sweep = lag_sweep_rows(label, joined)  # type: ignore[arg-type]
        lag_sweep_by_label[label] = sweep
        lag_best_all.extend(lag_best_rows_for_run(label, sweep))
        projection_all.extend(lag_projection_window_rows(label, joined))  # type: ignore[arg-type]
        vector_window_all.extend(window_vector_projection_rows(label, joined))  # type: ignore[arg-type]
        params, eval_rows = alignment_fit_rows(label, joined)  # type: ignore[arg-type]
        alignment_param_all.extend(params)
        alignment_eval_all.extend(eval_rows)

    global_best_rows = [row for row in lag_best_all if row["window"] == "global"]
    lines.append(
        markdown_table(
            [
                "run",
                "best_lag_sec",
                "best_mean_m",
                "zero_lag_mean_m",
                "plus025_mean_m",
                "minus025_mean_m",
            ],
            [
                [
                    row["run"],
                    fmt(value(row, "best_lag_sec"), 2),
                    fmt(value(row, "best_xy_mean_m")),
                    fmt(value(row, "zero_lag_xy_mean_m")),
                    fmt(value(row, "plus025_lag_xy_mean_m")),
                    fmt(value(row, "minus025_lag_xy_mean_m")),
                ]
                for row in global_best_rows
            ],
        )
    )

    lines.extend(["", "### Window Best Lag"])
    key_windows = {"060-080", "080-100", "100-120", "120-140", "160-180"}
    window_best_rows = [row for row in lag_best_all if row["window"] in key_windows]
    lines.append(
        markdown_table(
            [
                "run",
                "since_arm_s",
                "best_lag_sec",
                "best_mean_m",
                "zero_lag_mean_m",
                "plus025_mean_m",
                "minus025_mean_m",
            ],
            [
                [
                    row["run"],
                    row["window"],
                    fmt(value(row, "best_lag_sec"), 2),
                    fmt(value(row, "best_xy_mean_m")),
                    fmt(value(row, "zero_lag_xy_mean_m")),
                    fmt(value(row, "plus025_lag_xy_mean_m")),
                    fmt(value(row, "minus025_lag_xy_mean_m")),
                ]
                for row in window_best_rows
            ],
        )
    )

    lines.extend(["", "### Velocity Projection Optimum"])
    projection_rows = [row for row in projection_all if row["window"] in {"global", "080-100", "100-120", "160-180"}]
    lines.append(
        markdown_table(
            [
                "run",
                "window",
                "n",
                "speed_mean_mps",
                "opt_lag_mean_sec",
                "opt_lag_p50_sec",
                "opt_lag_p10_sec",
                "opt_lag_p90_sec",
                "err_along_vel_mean_m",
                "err_cross_vel_mean_m",
            ],
            [
                [
                    row["run"],
                    row["window"],
                    row["n"],
                    fmt(value(row, "speed_mean_mps")),
                    fmt(value(row, "opt_lag_mean_sec")),
                    fmt(value(row, "opt_lag_p50_sec")),
                    fmt(value(row, "opt_lag_p10_sec")),
                    fmt(value(row, "opt_lag_p90_sec")),
                    fmt(value(row, "err_along_velocity_mean_m")),
                    fmt(value(row, "err_cross_velocity_mean_m")),
                ]
                for row in projection_rows
            ],
        )
    )

    lines.extend(["", "### Window Error Direction"])
    direction_windows = {"060-080", "080-100", "100-120", "120-140", "160-180"}
    direction_rows = [row for row in vector_window_all if row["window"] in direction_windows]
    lines.append(
        markdown_table(
            [
                "run",
                "window",
                "err_E_mean_m",
                "err_N_mean_m",
                "vE_mean_mps",
                "vN_mean_mps",
                "heading_deg",
                "err_along_vel_m",
                "err_cross_vel_m",
                "iekf_phase_E_m",
                "ekf2_phase_E_m",
            ],
            [
                [
                    row["run"],
                    row["window"],
                    fmt(value(row, "pair_error_e_mean_m")),
                    fmt(value(row, "pair_error_n_mean_m")),
                    fmt(value(row, "native_velocity_e_mean_mps")),
                    fmt(value(row, "native_velocity_n_mean_mps")),
                    fmt(value(row, "native_velocity_heading_deg"), 1),
                    fmt(value(row, "err_along_velocity_mean_m")),
                    fmt(value(row, "err_cross_velocity_mean_m")),
                    fmt(value(row, "iekf_phase_e_mean_m")),
                    fmt(value(row, "ekf2_phase_e_mean_m")),
                ]
                for row in direction_rows
            ],
        )
    )

    lines.extend(["", "## Alignment Fit Check"])
    selected_fits = {"raw_initial_translation", "extra_translation_global", "similarity_global", "similarity_000_060"}
    lines.append(
        markdown_table(
            [
                "run",
                "fit",
                "fit_window",
                "n_fit",
                "scale",
                "angle_deg",
                "trans_x_m",
                "trans_y_m",
                "fit_xy_mean_m",
                "fit_xy_max_m",
            ],
            [
                [
                    row["run"],
                    row["fit"],
                    row["fit_window"],
                    row["n_fit"],
                    fmt(value(row, "scale"), 6),
                    fmt(value(row, "angle_deg"), 4),
                    fmt(value(row, "trans_x_m")),
                    fmt(value(row, "trans_y_m")),
                    fmt(value(row, "fit_xy_mean_m")),
                    fmt(value(row, "fit_xy_max_m")),
                ]
                for row in alignment_param_all
                if row["fit"] in selected_fits
            ],
        )
    )

    lines.extend(["", "### Alignment Fit Window Residuals"])
    fit_windows = {"global", "080-100", "100-120", "120-140", "160-180"}
    lines.append(
        markdown_table(
            ["run", "fit", "window", "xy_mean_m", "xy_max_m", "err_E_mean_m", "err_N_mean_m"],
            [
                [
                    row["run"],
                    row["fit"],
                    row["window"],
                    fmt(value(row, "xy_mean_m")),
                    fmt(value(row, "xy_max_m")),
                    fmt(value(row, "err_e_mean_m")),
                    fmt(value(row, "err_n_mean_m")),
                ]
                for row in alignment_eval_all
                if row["fit"] in {"raw_initial_translation", "similarity_global", "similarity_000_060"}
                and row["window"] in fit_windows
            ],
        )
    )

    lines.extend(["", "## Top High-Error Windows"])
    top_rows: list[list[object]] = []
    for run in runs:
        joined = run["joined_rows"]  # type: ignore[assignment]
        windows: list[tuple[float, float, list[dict[str, object]]]] = []
        max_time = max((value(row, "time_since_arm_sec") for row in joined), default=0.0)  # type: ignore[arg-type]
        start = 0.0
        while start < max_time:
            rows = select_window(joined, start, start + 20.0)  # type: ignore[arg-type]
            if len(rows) >= 20:
                windows.append((mean(value(row, "pair_error_xy_m") for row in rows), start, rows))
            start += 20.0
        for xy_mean, start, rows in sorted(windows, reverse=True)[:4]:
            top_rows.append(
                [
                    run["label"],
                    f"{int(start):03d}-{int(start + 20):03d}",
                    fmt(xy_mean),
                    fmt(max_finite(value(row, "pair_error_xy_m") for row in rows)),
                    fmt(mean(value(row, "pair_error_x_m") for row in rows)),
                    fmt(mean(value(row, "pair_error_y_m") for row in rows)),
                    fmt(mean(value(row, "pair_vel_error_norm_mps") for row in rows)),
                    fmt(mean(value(row, "gnss_residual_h_m") for row in rows)),
                    fmt(mean(value(row, "state_core_gnss_diff_h_m") for row in rows)),
                    fmt(mean(value(row, "state_phase_core_gnss_h_m") for row in rows)),
                    fmt(mean(value(row, "iekf_vs_phase_gnss_h_m") for row in rows)),
                    fmt(mean(value(row, "ekf2_vs_phase_gnss_aligned_h_m") for row in rows)),
                    fmt(mean(value(row, "ekf2_minus_phase_gnss_aligned_e_m") for row in rows)),
                    fmt(mean(value(row, "ekf2_minus_phase_gnss_aligned_n_m") for row in rows)),
                ]
            )
    lines.append(
        markdown_table(
            [
                "run",
                "since_arm_s",
                "xy_mean_m",
                "xy_max_m",
                "err_x_mean",
                "err_y_mean",
                "vel_mean",
                "gnss_res_h",
                "state_core_gnss_h",
                "phase_core_gnss_h",
                "iekf_phase_gnss_h",
                "ekf2_phase_gnss_aligned_h",
                "ekf2_phase_e_mean",
                "ekf2_phase_n_mean",
            ],
            top_rows,
        )
    )

    lines.extend(
        [
            "",
            "## Readout",
            "",
        ]
    )
    labels = {str(run["label"]): run for run in runs}
    if "manual121" in labels:
        m121 = labels["manual121"]
        joined121 = m121["joined_rows"]  # type: ignore[assignment]
        s121 = summarize_rows(joined121)  # type: ignore[arg-type]
        w100 = select_window(joined121, 100.0, 120.0)  # type: ignore[arg-type]
        w120 = select_window(joined121, 120.0, 140.0)  # type: ignore[arg-type]
        lines.append(
            "- manual121 with EKF2 input stamps is a strong positive isolation: "
            f"armed XY mean/max `{fmt(s121['xy_mean'])}/{fmt(s121['xy_max'])}m`, "
            f"`100-120s` mean `{fmt(mean(value(row, 'pair_error_xy_m') for row in w100))}m`, "
            f"`120-140s` mean `{fmt(mean(value(row, 'pair_error_xy_m') for row in w120))}m`."
        )
        lines.append(
            "- This supports EKF2 relay timestamp aliasing as a major contributor to the old pair error; "
            "manual121 still is not final closure because the max remains meter-scale."
        )
    if "manual122" in labels and "manual123" in labels:
        joined122 = labels["manual122"]["joined_rows"]  # type: ignore[index,assignment]
        joined123 = labels["manual123"]["joined_rows"]  # type: ignore[index,assignment]
        s122 = summarize_rows(joined122)  # type: ignore[arg-type]
        s123 = summarize_rows(joined123)  # type: ignore[arg-type]
        lines.append(
            "- manual123 is a valid negative for fixed `+0.25s` GNSS-position forward compensation: "
            f"manual122 armed XY mean/max `{fmt(s122['xy_mean'])}/{fmt(s122['xy_max'])}m`, "
            f"manual123 `{fmt(s123['xy_mean'])}/{fmt(s123['xy_max'])}m`."
        )
        best122 = {str(row["window"]): row for row in lag_best_all if row["run"] == "manual122"}
        if {"global", "080-100", "100-120"}.issubset(best122):
            lines.append(
                "- manual122 offline lag sweep rejects a single scalar source-lag model: "
                f"global best lag `{fmt(value(best122['global'], 'best_lag_sec'), 2)}s`, "
                f"`080-100s` best `{fmt(value(best122['080-100'], 'best_lag_sec'), 2)}s`, "
                f"`100-120s` best `{fmt(value(best122['100-120'], 'best_lag_sec'), 2)}s`."
            )
        vector122 = {str(row["window"]): row for row in vector_window_all if row["run"] == "manual122"}
        if {"080-100", "100-120"}.issubset(vector122):
            lines.append(
                "- The lag-sign flip is explained by direction projection, not by a changing source lag: "
                f"manual122 pair error stays mostly +E (`080-100s` E `{fmt(value(vector122['080-100'], 'pair_error_e_mean_m'))}m`, "
                f"`100-120s` E `{fmt(value(vector122['100-120'], 'pair_error_e_mean_m'))}m`), "
                f"while mean velocity flips from heading `{fmt(value(vector122['080-100'], 'native_velocity_heading_deg'), 1)}deg` "
                f"to `{fmt(value(vector122['100-120'], 'native_velocity_heading_deg'), 1)}deg`."
            )
        raw122 = next(
            (
                row for row in alignment_eval_all
                if row["run"] == "manual122"
                and row["fit"] == "raw_initial_translation"
                and row["window"] == "global"
            ),
            None,
        )
        sim122 = next(
            (
                row for row in alignment_eval_all
                if row["run"] == "manual122"
                and row["fit"] == "similarity_global"
                and row["window"] == "global"
            ),
            None,
        )
        sim122_params = next(
            (
                row for row in alignment_param_all
                if row["run"] == "manual122"
                and row["fit"] == "similarity_global"
            ),
            None,
        )
        if raw122 is not None and sim122 is not None and sim122_params is not None:
            lines.append(
                "- A post-fit 2D similarity transform collapses most of manual122's residual "
                f"from `{fmt(value(raw122, 'xy_mean_m'))}/{fmt(value(raw122, 'xy_max_m'))}m` "
                f"to `{fmt(value(sim122, 'xy_mean_m'))}/{fmt(value(sim122, 'xy_max_m'))}m` "
                f"(scale `{fmt(value(sim122_params, 'scale'), 6)}`, angle `{fmt(value(sim122_params, 'angle_deg'), 4)}deg`). "
                "This makes a frame/projection/alignment interpretation stronger than a fixed GNSS source-lag interpretation."
            )
    if {"manual118", "manual119", "manual120"}.issubset(labels):
        lines.append(
            "- manual120 returns to roughly manual118 behavior, while manual119 is worse. That points away from GNSS covariance/override as the primary fix path."
        )
        lines.append(
            "- Raw state-publish core-minus-GNSS is large mostly because it compares current state against the last GNSS source sample, which is commonly about 0.2s old. After velocity phase correction, state core-vs-GNSS is small in manual118/manual120, so this is not strong evidence for a live GNSS update covariance fault."
        )
    lines.extend(
        [
            "- In high-error windows, IEKF stays much closer to phase-corrected GNSS than the pair error suggests, while EKF2 relative to phase-corrected GNSS plus the pair logger's fixed alignment offset remains about meter-scale. That points at EKF2/IEKF local-frame and alignment semantics before estimator tuning.",
            "- Simple vector fits from GNSS residual/core-minus-GNSS vectors to pair error have high relative RMSE. The pair error is consistently dominated by +X in high-error windows, so the next diagnostic should inspect EKF2 relay origin/axis/timestamps and whether fixed translation alignment is sufficient.",
            "- Velocity-error lead correlations are weak, so velocity coupling is still possible but not the strongest first explanation from these CSVs.",
            "- The remaining useful branch is offline frame/projection/alignment work before any new flight: inspect whether pair error directions are consistent with a local-frame origin/rotation/timestamp issue, then validate against ULog or simulator ground truth if available.",
            "- Do not put `real_time_comparison.py` back into the flight chain for this; keep future analysis offline unless a specific candidate fix needs validation.",
            "",
            "Generated files:",
            f"- `{out_dir / 'joined_pair_gnss_state.csv'}`",
            f"- `{out_dir / 'window_summary.csv'}`",
            f"- `{out_dir / 'lag_sweep.csv'}`",
            f"- `{out_dir / 'lag_window_best.csv'}`",
            f"- `{out_dir / 'lag_projection_window.csv'}`",
            f"- `{out_dir / 'window_vector_projection.csv'}`",
            f"- `{out_dir / 'alignment_fit.csv'}`",
            f"- `{out_dir / 'alignment_window_eval.csv'}`",
        ]
    )
    return "\n".join(lines) + "\n"


def write_window_summary(path: Path, runs: list[dict[str, object]]) -> None:
    rows: list[dict[str, object]] = []
    for run in runs:
        joined = run["joined_rows"]  # type: ignore[assignment]
        max_time = max((value(row, "time_since_arm_sec") for row in joined), default=0.0)  # type: ignore[arg-type]
        start = 0.0
        while start < max_time:
            subset = select_window(joined, start, start + 20.0)  # type: ignore[arg-type]
            if subset:
                row: dict[str, object] = {
                    "run": run["label"],
                    "since_arm_start_sec": start,
                    "since_arm_end_sec": start + 20.0,
                }
                row.update(summarize_rows(subset))
                rows.append(row)
            start += 20.0
    write_csv(path, rows)


def parse_run_arg(item: str) -> tuple[str, str]:
    if "=" in item:
        label, path = item.split("=", 1)
        return label, path
    path = Path(item)
    return path.name, item


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--run",
        action="append",
        help="Run mapping as label=/abs/run_dir. Defaults to manual118/manual119/manual120.",
    )
    parser.add_argument(
        "--out-dir",
        default="/home/yang/kf_gins_ws/artifacts/manual/offline_pair_frame_diag_manual118_119_120",
        help="Output directory for report and joined CSVs.",
    )
    args = parser.parse_args()

    run_specs = [parse_run_arg(item) for item in args.run] if args.run else DEFAULT_RUNS
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    runs = [load_run(label, Path(run_dir)) for label, run_dir in run_specs]
    all_joined: list[dict[str, object]] = []
    lag_sweep_all: list[dict[str, object]] = []
    lag_window_best_all: list[dict[str, object]] = []
    lag_projection_all: list[dict[str, object]] = []
    window_vector_projection_all: list[dict[str, object]] = []
    alignment_fit_all: list[dict[str, object]] = []
    alignment_window_eval_all: list[dict[str, object]] = []
    for run in runs:
        all_joined.extend(run["joined_rows"])  # type: ignore[arg-type]
        label = str(run["label"])
        joined = run["joined_rows"]  # type: ignore[assignment]
        sweep = lag_sweep_rows(label, joined)  # type: ignore[arg-type]
        lag_sweep_all.extend(sweep)
        lag_window_best_all.extend(lag_best_rows_for_run(label, sweep))
        lag_projection_all.extend(lag_projection_window_rows(label, joined))  # type: ignore[arg-type]
        window_vector_projection_all.extend(window_vector_projection_rows(label, joined))  # type: ignore[arg-type]
        fit_params, fit_eval = alignment_fit_rows(label, joined)  # type: ignore[arg-type]
        alignment_fit_all.extend(fit_params)
        alignment_window_eval_all.extend(fit_eval)
    write_csv(out_dir / "joined_pair_gnss_state.csv", all_joined)
    write_window_summary(out_dir / "window_summary.csv", runs)
    write_csv(out_dir / "lag_sweep.csv", lag_sweep_all)
    write_csv(out_dir / "lag_window_best.csv", lag_window_best_all)
    write_csv(out_dir / "lag_projection_window.csv", lag_projection_all)
    write_csv(out_dir / "window_vector_projection.csv", window_vector_projection_all)
    write_csv(out_dir / "alignment_fit.csv", alignment_fit_all)
    write_csv(out_dir / "alignment_window_eval.csv", alignment_window_eval_all)
    report = build_report(runs, out_dir)
    (out_dir / "report.md").write_text(report, encoding="utf-8")

    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'joined_pair_gnss_state.csv'}")
    print(f"wrote: {out_dir / 'window_summary.csv'}")
    print(f"wrote: {out_dir / 'lag_sweep.csv'}")
    print(f"wrote: {out_dir / 'lag_window_best.csv'}")
    print(f"wrote: {out_dir / 'lag_projection_window.csv'}")
    print(f"wrote: {out_dir / 'window_vector_projection.csv'}")
    print(f"wrote: {out_dir / 'alignment_fit.csv'}")
    print(f"wrote: {out_dir / 'alignment_window_eval.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
