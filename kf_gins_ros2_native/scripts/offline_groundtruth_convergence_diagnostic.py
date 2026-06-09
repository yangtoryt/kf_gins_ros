#!/usr/bin/env python3
"""Compare EKF2 and projection-normalized IEKF against ULog groundtruth."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Iterable


DEFAULT_WINDOWS = [
    (0.0, 20.0),
    (20.0, 40.0),
    (40.0, 60.0),
    (60.0, 80.0),
    (80.0, 100.0),
    (100.0, 120.0),
    (120.0, 140.0),
    (140.0, 160.0),
    (160.0, 180.0),
    (300.0, 360.0),
]

WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
PX4_SPHERE_RADIUS_M = 6371000.0


def finite(value: float) -> bool:
    return math.isfinite(value)


def to_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return result if finite(result) else default


def truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


def mean(values: Iterable[float]) -> float:
    vals = [item for item in values if finite(item)]
    return sum(vals) / len(vals) if vals else math.nan


def rmse(values: Iterable[float]) -> float:
    vals = [item for item in values if finite(item)]
    return math.sqrt(sum(item * item for item in vals) / len(vals)) if vals else math.nan


def percentile(values: Iterable[float], pct: float) -> float:
    vals = sorted(item for item in values if finite(item))
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
    vals = [item for item in values if finite(item)]
    return max(vals) if vals else math.nan


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


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


def find_topic_csv(ulog_dir: Path, topic: str) -> Path:
    matches = sorted(ulog_dir.glob(f"*_{topic}_0.csv"))
    if not matches:
        raise FileNotFoundError(f"missing ulog2csv export for topic {topic!r} in {ulog_dir}")
    if len(matches) > 1:
        names = ", ".join(path.name for path in matches)
        raise RuntimeError(f"multiple ulog2csv exports for topic {topic!r} in {ulog_dir}: {names}")
    return matches[0]


def llh_to_ecef(lat_rad: float, lon_rad: float, h_m: float) -> tuple[float, float, float]:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (n + h_m) * cos_lat * math.cos(lon_rad),
        (n + h_m) * cos_lat * math.sin(lon_rad),
        (n * (1.0 - WGS84_E2) + h_m) * sin_lat,
    )


def ecef_to_enu(
    ecef: tuple[float, float, float],
    origin_ecef: tuple[float, float, float],
    origin_lat_rad: float,
    origin_lon_rad: float,
) -> tuple[float, float, float]:
    dx = ecef[0] - origin_ecef[0]
    dy = ecef[1] - origin_ecef[1]
    dz = ecef[2] - origin_ecef[2]
    sin_lat = math.sin(origin_lat_rad)
    cos_lat = math.cos(origin_lat_rad)
    sin_lon = math.sin(origin_lon_rad)
    cos_lon = math.cos(origin_lon_rad)
    east = -sin_lon * dx + cos_lon * dy
    north = -sin_lat * cos_lon * dx - sin_lat * sin_lon * dy + cos_lat * dz
    up = cos_lat * cos_lon * dx + cos_lat * sin_lon * dy + sin_lat * dz
    return east, north, up


def local_groundtruth_ref_valid(path: Path) -> bool:
    for row in read_csv(path)[:20]:
        ref_lat = to_float(row.get("ref_lat"))
        ref_lon = to_float(row.get("ref_lon"))
        if finite(ref_lat) and finite(ref_lon) and (abs(ref_lat) > 1.0e-6 or abs(ref_lon) > 1.0e-6):
            return True
    return False


def load_local_groundtruth(path: Path) -> tuple[list[dict[str, float]], list[float]]:
    rows: list[dict[str, float]] = []
    for row in read_csv(path):
        t = to_float(row.get("timestamp")) * 1e-6
        n = to_float(row.get("x"))
        e = to_float(row.get("y"))
        d = to_float(row.get("z"))
        if all(finite(item) for item in (t, n, e, d)):
            rows.append(
                {
                    "timestamp_sec": t,
                    "x_m": e,       # ROS ENU x = East
                    "y_m": n,       # ROS ENU y = North
                    "z_m": -d,      # ROS ENU z = Up
                }
            )
    rows.sort(key=lambda item: item["timestamp_sec"])
    return rows, [row["timestamp_sec"] for row in rows]


def load_global_groundtruth(path: Path) -> tuple[list[dict[str, float]], list[float]]:
    raw_rows = read_csv(path)
    origin: tuple[float, float, float] | None = None
    origin_lat_rad = math.nan
    origin_lon_rad = math.nan
    rows: list[dict[str, float]] = []
    for row in raw_rows:
        t = to_float(row.get("timestamp")) * 1e-6
        lat_deg = to_float(row.get("lat"))
        lon_deg = to_float(row.get("lon"))
        alt_m = to_float(row.get("alt"))
        if not all(finite(item) for item in (t, lat_deg, lon_deg, alt_m)):
            continue
        lat_rad = math.radians(lat_deg)
        lon_rad = math.radians(lon_deg)
        ecef = llh_to_ecef(lat_rad, lon_rad, alt_m)
        if origin is None:
            origin = ecef
            origin_lat_rad = lat_rad
            origin_lon_rad = lon_rad
        east, north, up = ecef_to_enu(ecef, origin, origin_lat_rad, origin_lon_rad)
        rows.append({"timestamp_sec": t, "x_m": east, "y_m": north, "z_m": up})
    rows.sort(key=lambda item: item["timestamp_sec"])
    return rows, [row["timestamp_sec"] for row in rows]


def load_groundtruth(
    ulog_dir: Path,
    source: str = "auto",
) -> tuple[list[dict[str, float]], list[float], str]:
    path = find_topic_csv(ulog_dir, "vehicle_local_position_groundtruth")
    if source not in {"auto", "local", "global"}:
        raise ValueError(f"unsupported groundtruth source: {source}")
    if source == "local" or (source == "auto" and local_groundtruth_ref_valid(path)):
        rows, times = load_local_groundtruth(path)
        return rows, times, "local"
    if source == "local":
        rows, times = load_local_groundtruth(path)
        return rows, times, "local"
    global_path = find_topic_csv(ulog_dir, "vehicle_global_position_groundtruth")
    rows, times = load_global_groundtruth(global_path)
    return rows, times, "global"


def load_groundtruth_ref_lat_rad(ulog_dir: Path) -> float:
    path = find_topic_csv(ulog_dir, "vehicle_local_position_groundtruth")
    for row in read_csv(path):
        ref_lat = to_float(row.get("ref_lat"))
        if finite(ref_lat):
            return math.radians(ref_lat)
    raise RuntimeError(f"missing ref_lat in {path}")


def wgs84_radii(lat_rad: float) -> tuple[float, float]:
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    rn = WGS84_A_M / denom
    rm = WGS84_A_M * (1.0 - WGS84_E2) / (denom * denom * denom)
    return rm, rn


def nearest_groundtruth(
    rows: list[dict[str, float]],
    times: list[float],
    t: float,
    max_dt_sec: float,
) -> tuple[dict[str, float] | None, float]:
    i = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, float]]] = []
    for idx in (i - 1, i):
        if 0 <= idx < len(rows):
            candidates.append((abs(times[idx] - t), rows[idx]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    if dt > max_dt_sec:
        return None, dt
    return row, dt


def load_gnss_context(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    path = run_dir / "gnss_update_debug.csv"
    if not path.exists() or path.stat().st_size == 0:
        return [], []
    rows: list[dict[str, object]] = []
    for row in read_csv(path):
        t = to_float(row.get("ros_time_sec"))
        if not finite(t):
            continue
        rows.append(
            {
                "ros_time_sec": t,
                "turning_now": int(truthy(row.get("turning_now"))),
                "post_turn_context": int(truthy(row.get("post_turn_context"))),
                "armed_cruise_context": int(truthy(row.get("armed_cruise_context"))),
                "horizontal_speed_mps": to_float(row.get("horizontal_speed_mps")),
                "vertical_speed_mps": to_float(row.get("vertical_speed_mps")),
                "gyro_deg_s": to_float(row.get("gyro_deg_s")),
                "source_yaw_rate_deg_s": to_float(row.get("source_yaw_rate_deg_s")),
                "core_gnss_diff_h_m": to_float(row.get("core_gnss_diff_h_m")),
                "last_position_residual_h_m": to_float(row.get("last_position_residual_h_m")),
                "medium_gap_active": int(truthy(row.get("medium_gap_active"))),
                "medium_gap_segmented": int(truthy(row.get("medium_gap_segmented"))),
            }
        )
    rows.sort(key=lambda item: item["ros_time_sec"])  # type: ignore[index]
    return rows, [to_float(row["ros_time_sec"]) for row in rows]


def nearest_context(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt_sec: float = 0.25,
) -> tuple[dict[str, object] | None, float]:
    if not rows:
        return None, math.nan
    i = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for idx in (i - 1, i):
        if 0 <= idx < len(rows):
            candidates.append((abs(times[idx] - t), rows[idx]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    if dt > max_dt_sec:
        return None, dt
    return row, dt


def load_fit(path: Path, topic: str, subset: str) -> dict[str, float]:
    for row in read_csv(path):
        if row.get("topic") == topic and row.get("subset") == subset:
            scale = to_float(row.get("scale"))
            angle_deg = to_float(row.get("angle_deg"))
            if not all(finite(item) for item in (scale, angle_deg)):
                break
            angle = math.radians(angle_deg)
            return {
                "scale": scale,
                "angle_deg": angle_deg,
                "a_real": scale * math.cos(angle),
                "a_imag": scale * math.sin(angle),
            }
    raise RuntimeError(f"fit row not found: {path} topic={topic} subset={subset}")


def transform_xy(x: float, y: float, fit: dict[str, float]) -> tuple[float, float]:
    mode = fit.get("projection_mode", "gps_fit")
    if mode == "raw_wgs84_enu":
        return x, y
    if mode == "px4_sphere_anisotropic":
        return x * fit["px4_east_scale"], y * fit["px4_north_scale"]
    a_real = fit["a_real"]
    a_imag = fit["a_imag"]
    return a_real * x - a_imag * y, a_imag * x + a_real * y


def build_projection(
    fit_csv: Path,
    fit_topic: str,
    fit_subset: str,
    projection_mode: str,
    ulog_dir: Path,
) -> dict[str, float]:
    if projection_mode == "gps_fit":
        fit = load_fit(fit_csv, fit_topic, fit_subset)
        fit["projection_mode"] = projection_mode
        return fit
    if projection_mode == "raw_wgs84_enu":
        return {
            "projection_mode": projection_mode,
            "scale": 1.0,
            "angle_deg": 0.0,
            "a_real": 1.0,
            "a_imag": 0.0,
        }
    if projection_mode == "px4_sphere_anisotropic":
        ref_lat = load_groundtruth_ref_lat_rad(ulog_dir)
        rm, rn = wgs84_radii(ref_lat)
        east_scale = PX4_SPHERE_RADIUS_M / rn
        north_scale = PX4_SPHERE_RADIUS_M / rm
        return {
            "projection_mode": projection_mode,
            "scale": math.nan,
            "angle_deg": 0.0,
            "a_real": math.nan,
            "a_imag": math.nan,
            "px4_east_scale": east_scale,
            "px4_north_scale": north_scale,
            "projection_ref_lat_deg": math.degrees(ref_lat),
        }
    raise ValueError(f"unsupported projection mode: {projection_mode}")


def joined_rows(
    run_label: str,
    run_dir: Path,
    ulog_dir: Path,
    fit: dict[str, float],
    max_join_dt_sec: float,
    groundtruth_source: str,
    pair_csv: Path | None = None,
) -> list[dict[str, object]]:
    gt_rows, gt_times, resolved_groundtruth_source = load_groundtruth(ulog_dir, groundtruth_source)
    fit["groundtruth_source"] = resolved_groundtruth_source
    ctx_rows, ctx_times = load_gnss_context(run_dir)
    pair_path = pair_csv or (run_dir / "ekf_iekf_pairs.csv")
    if not pair_path.exists():
        raise FileNotFoundError(pair_path)

    raw_rows: list[dict[str, object]] = []
    arm_time = math.nan
    for row in read_csv(pair_path):
        t = to_float(row.get("ekf2_stamp_sec"))
        if truthy(row.get("mavros_armed")) and finite(t):
            arm_time = t
            break

    for row in read_csv(pair_path):
        t = to_float(row.get("ekf2_stamp_sec"))
        ros_t = to_float(row.get("ros_time_sec"), t)
        if not finite(t):
            continue
        gt, gt_dt = nearest_groundtruth(gt_rows, gt_times, t, max_join_dt_sec)
        if gt is None:
            continue
        ctx, ctx_dt = nearest_context(ctx_rows, ctx_times, ros_t)
        ekf2_x = to_float(row.get("ekf2_x_m"))
        ekf2_y = to_float(row.get("ekf2_y_m"))
        iekf_x = to_float(row.get("iekf_x_m"))
        iekf_y = to_float(row.get("iekf_y_m"))
        if not all(finite(item) for item in (ekf2_x, ekf2_y, iekf_x, iekf_y)):
            continue
        iekf_px4_x, iekf_px4_y = transform_xy(iekf_x, iekf_y, fit)
        time_since_arm = t - arm_time if finite(arm_time) else math.nan
        raw_rows.append(
            {
                "run": run_label,
                "stamp_sec": t,
                "pair_ros_time_sec": ros_t,
                "time_since_arm_sec": time_since_arm,
                "mavros_armed": int(truthy(row.get("mavros_armed"))),
                "mavros_mode": row.get("mavros_mode", ""),
                "alignment_ready": int(truthy(row.get("alignment_ready"))),
                "gt_join_dt_sec": gt_dt,
                "context_join_dt_sec": ctx_dt,
                "turning_now": int(ctx.get("turning_now", 0)) if ctx else 0,
                "post_turn_context": int(ctx.get("post_turn_context", 0)) if ctx else 0,
                "armed_cruise_context": int(ctx.get("armed_cruise_context", 0)) if ctx else 0,
                "horizontal_speed_mps": to_float(ctx.get("horizontal_speed_mps")) if ctx else math.nan,
                "vertical_speed_mps": to_float(ctx.get("vertical_speed_mps")) if ctx else math.nan,
                "gyro_deg_s": to_float(ctx.get("gyro_deg_s")) if ctx else math.nan,
                "source_yaw_rate_deg_s": to_float(ctx.get("source_yaw_rate_deg_s")) if ctx else math.nan,
                "core_gnss_diff_h_m": to_float(ctx.get("core_gnss_diff_h_m")) if ctx else math.nan,
                "last_position_residual_h_m": to_float(ctx.get("last_position_residual_h_m")) if ctx else math.nan,
                "medium_gap_active": int(ctx.get("medium_gap_active", 0)) if ctx else 0,
                "medium_gap_segmented": int(ctx.get("medium_gap_segmented", 0)) if ctx else 0,
                "gt_x_m": gt["x_m"],
                "gt_y_m": gt["y_m"],
                "gt_z_m": gt["z_m"],
                "ekf2_x_m": ekf2_x,
                "ekf2_y_m": ekf2_y,
                "iekf_px4_x_m": iekf_px4_x,
                "iekf_px4_y_m": iekf_px4_y,
            }
        )
    return raw_rows


def compute_offsets(rows: list[dict[str, object]], align_start: float, align_end: float) -> dict[str, float]:
    ekf2_dx: list[float] = []
    ekf2_dy: list[float] = []
    iekf_dx: list[float] = []
    iekf_dy: list[float] = []
    for row in rows:
        if int(row.get("mavros_armed", 0)) != 1:
            continue
        t = to_float(row.get("time_since_arm_sec"))
        if not (align_start <= t <= align_end):
            continue
        gt_x = to_float(row.get("gt_x_m"))
        gt_y = to_float(row.get("gt_y_m"))
        ekf2_x = to_float(row.get("ekf2_x_m"))
        ekf2_y = to_float(row.get("ekf2_y_m"))
        iekf_x = to_float(row.get("iekf_px4_x_m"))
        iekf_y = to_float(row.get("iekf_px4_y_m"))
        if all(finite(item) for item in (gt_x, gt_y, ekf2_x, ekf2_y, iekf_x, iekf_y)):
            ekf2_dx.append(gt_x - ekf2_x)
            ekf2_dy.append(gt_y - ekf2_y)
            iekf_dx.append(gt_x - iekf_x)
            iekf_dy.append(gt_y - iekf_y)
    return {
        "align_rows": float(len(ekf2_dx)),
        "ekf2_offset_x_m": mean(ekf2_dx),
        "ekf2_offset_y_m": mean(ekf2_dy),
        "iekf_offset_x_m": mean(iekf_dx),
        "iekf_offset_y_m": mean(iekf_dy),
    }


def apply_offsets(rows: list[dict[str, object]], offsets: dict[str, float]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for row in rows:
        out = dict(row)
        gt_x = to_float(row.get("gt_x_m"))
        gt_y = to_float(row.get("gt_y_m"))
        ekf2_x = to_float(row.get("ekf2_x_m")) + offsets["ekf2_offset_x_m"]
        ekf2_y = to_float(row.get("ekf2_y_m")) + offsets["ekf2_offset_y_m"]
        iekf_x = to_float(row.get("iekf_px4_x_m")) + offsets["iekf_offset_x_m"]
        iekf_y = to_float(row.get("iekf_px4_y_m")) + offsets["iekf_offset_y_m"]
        ekf2_err_x = ekf2_x - gt_x
        ekf2_err_y = ekf2_y - gt_y
        iekf_err_x = iekf_x - gt_x
        iekf_err_y = iekf_y - gt_y
        out.update(
            {
                "ekf2_aligned_x_m": ekf2_x,
                "ekf2_aligned_y_m": ekf2_y,
                "iekf_normalized_aligned_x_m": iekf_x,
                "iekf_normalized_aligned_y_m": iekf_y,
                "ekf2_error_x_m": ekf2_err_x,
                "ekf2_error_y_m": ekf2_err_y,
                "ekf2_error_xy_m": math.hypot(ekf2_err_x, ekf2_err_y),
                "iekf_error_x_m": iekf_err_x,
                "iekf_error_y_m": iekf_err_y,
                "iekf_error_xy_m": math.hypot(iekf_err_x, iekf_err_y),
            }
        )
        result.append(out)
    return result


def summarize(rows: list[dict[str, object]], label: str, start: float | None, end: float | None) -> list[dict[str, object]]:
    subset: list[dict[str, object]] = []
    for row in rows:
        if int(row.get("mavros_armed", 0)) != 1:
            continue
        t = to_float(row.get("time_since_arm_sec"))
        if start is not None and (not finite(t) or t < start):
            continue
        if end is not None and (not finite(t) or t >= end):
            continue
        subset.append(row)
    result: list[dict[str, object]] = []
    for estimator, key in (("ekf2", "ekf2_error_xy_m"), ("iekf_normalized", "iekf_error_xy_m")):
        errors = [to_float(row.get(key)) for row in subset]
        result.append(
            {
                "window": label,
                "estimator": estimator,
                "rows": len(errors),
                "xy_rmse_m": rmse(errors),
                "xy_mean_m": mean(errors),
                "xy_p95_m": percentile(errors, 95.0),
                "xy_max_m": max_finite(errors),
                "frac_over_0p3": mean(1.0 if finite(item) and item > 0.3 else 0.0 for item in errors),
            }
        )
    return result


def summarize_subset(rows: list[dict[str, object]], label: str, subset: list[dict[str, object]]) -> list[dict[str, object]]:
    result: list[dict[str, object]] = []
    for estimator, key in (("ekf2", "ekf2_error_xy_m"), ("iekf_normalized", "iekf_error_xy_m")):
        errors = [to_float(row.get(key)) for row in subset]
        result.append(
            {
                "window": label,
                "estimator": estimator,
                "rows": len(errors),
                "xy_rmse_m": rmse(errors),
                "xy_mean_m": mean(errors),
                "xy_p95_m": percentile(errors, 95.0),
                "xy_max_m": max_finite(errors),
                "frac_over_0p3": mean(1.0 if finite(item) and item > 0.3 else 0.0 for item in errors),
            }
        )
    return result


def segment_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    armed = [row for row in rows if int(row.get("mavros_armed", 0)) == 1]

    def in_window(row: dict[str, object], start: float, end: float) -> bool:
        t = to_float(row.get("time_since_arm_sec"))
        return finite(t) and start <= t < end

    segments: list[tuple[str, list[dict[str, object]]]] = [
        ("segment_mission_all", [row for row in armed if row.get("mavros_mode") == "AUTO.MISSION"]),
        ("segment_rtl_all", [row for row in armed if row.get("mavros_mode") == "AUTO.RTL"]),
        ("segment_main_maneuver_40_120", [row for row in armed if in_window(row, 40.0, 120.0)]),
        ("segment_main_maneuver_40_180", [row for row in armed if in_window(row, 40.0, 180.0)]),
        (
            "segment_turn_or_post_context",
            [
                row
                for row in armed
                if int(row.get("turning_now", 0)) == 1 or int(row.get("post_turn_context", 0)) == 1
            ],
        ),
        (
            "segment_armed_cruise_context",
            [row for row in armed if int(row.get("armed_cruise_context", 0)) == 1],
        ),
        ("segment_pre_rtl_250_300", [row for row in armed if in_window(row, 250.0, 300.0)]),
        ("segment_landing_tail_300_360", [row for row in armed if in_window(row, 300.0, 360.0)]),
    ]
    result: list[dict[str, object]] = []
    for label, subset in segments:
        result.extend(summarize_subset(rows, label, subset))
    return result


def build_report(
    summary_rows: list[dict[str, object]],
    offsets: dict[str, float],
    fit: dict[str, float],
    joined_rows: list[dict[str, object]],
    out_dir: Path,
) -> str:
    projection_mode = str(fit.get("projection_mode", "gps_fit"))
    armed = [row for row in summary_rows if row["window"] == "armed_all"]
    segment_labels = {
        "segment_mission_all",
        "segment_rtl_all",
        "segment_main_maneuver_40_120",
        "segment_main_maneuver_40_180",
        "segment_turn_or_post_context",
        "segment_armed_cruise_context",
        "segment_pre_rtl_250_300",
        "segment_landing_tail_300_360",
    }
    segment_summary = [row for row in summary_rows if row["window"] in segment_labels]
    key_windows = [row for row in summary_rows if row["window"] != "armed_all" and row["window"] not in segment_labels]
    worst_iekf = sorted(
        [row for row in joined_rows if int(row.get("mavros_armed", 0)) == 1],
        key=lambda row: to_float(row.get("iekf_error_xy_m")),
        reverse=True,
    )[:8]
    lines = [
        "# Groundtruth Convergence Diagnostic",
        "",
        "This report compares EKF2 and projection-normalized IEKF against ULog `vehicle_local_position_groundtruth` in a common ROS ENU frame.",
        f"Groundtruth source: `{fit.get('groundtruth_source', 'local')}`.",
        "",
        "Both estimators are aligned to groundtruth using the same early armed window. IEKF XY projection is selected by `projection_mode`, then translated only for the early-window origin offset.",
        "",
        "## Alignment",
        markdown_table(
            [
                "projection_mode",
                "align_rows",
                "fit_scale",
                "fit_angle_deg",
                "px4_east_scale",
                "px4_north_scale",
                "ekf2_offset_x",
                "ekf2_offset_y",
                "iekf_offset_x",
                "iekf_offset_y",
            ],
            [
                [
                    projection_mode,
                    int(offsets["align_rows"]),
                    fmt(fit["scale"], 6),
                    fmt(fit["angle_deg"], 4),
                    fmt(fit.get("px4_east_scale"), 7),
                    fmt(fit.get("px4_north_scale"), 7),
                    fmt(offsets["ekf2_offset_x_m"]),
                    fmt(offsets["ekf2_offset_y_m"]),
                    fmt(offsets["iekf_offset_x_m"]),
                    fmt(offsets["iekf_offset_y_m"]),
                ]
            ],
        ),
        "",
        "## Armed Summary",
        markdown_table(
            ["estimator", "rows", "xy_rmse_m", "xy_mean_m", "xy_p95_m", "xy_max_m", "frac_over_0p3"],
            [
                [
                    row["estimator"],
                    row["rows"],
                    fmt(row["xy_rmse_m"]),
                    fmt(row["xy_mean_m"]),
                    fmt(row["xy_p95_m"]),
                    fmt(row["xy_max_m"]),
                    fmt(row["frac_over_0p3"], 3),
                ]
                for row in armed
            ],
        ),
        "",
        "## Performance Segments",
        markdown_table(
            ["segment", "estimator", "rows", "xy_rmse_m", "xy_mean_m", "xy_p95_m", "xy_max_m", "frac_over_0p3"],
            [
                [
                    row["window"],
                    row["estimator"],
                    row["rows"],
                    fmt(row["xy_rmse_m"]),
                    fmt(row["xy_mean_m"]),
                    fmt(row["xy_p95_m"]),
                    fmt(row["xy_max_m"]),
                    fmt(row["frac_over_0p3"], 3),
                ]
                for row in segment_summary
            ],
        ),
        "",
        "## Key Windows",
        markdown_table(
            ["window", "estimator", "rows", "xy_rmse_m", "xy_mean_m", "xy_p95_m", "xy_max_m", "frac_over_0p3"],
            [
                [
                    row["window"],
                    row["estimator"],
                    row["rows"],
                    fmt(row["xy_rmse_m"]),
                    fmt(row["xy_mean_m"]),
                    fmt(row["xy_p95_m"]),
                    fmt(row["xy_max_m"]),
                    fmt(row["frac_over_0p3"], 3),
                ]
                for row in key_windows
            ],
        ),
        "",
        "## Largest IEKF Errors",
        markdown_table(
            [
                "t_since_arm",
                "mode",
                "iekf_xy",
                "ekf2_xy",
                "h_speed",
                "v_speed",
                "turning",
                "post_turn",
                "armed_cruise",
                "core_gnss_h",
            ],
            [
                [
                    fmt(row.get("time_since_arm_sec"), 1),
                    row.get("mavros_mode", ""),
                    fmt(row.get("iekf_error_xy_m")),
                    fmt(row.get("ekf2_error_xy_m")),
                    fmt(row.get("horizontal_speed_mps")),
                    fmt(row.get("vertical_speed_mps")),
                    row.get("turning_now", 0),
                    row.get("post_turn_context", 0),
                    row.get("armed_cruise_context", 0),
                    fmt(row.get("core_gnss_diff_h_m")),
                ]
                for row in worst_iekf
            ],
        ),
        "",
        "Generated files:",
        f"- `{out_dir / 'groundtruth_joined.csv'}`",
        f"- `{out_dir / 'groundtruth_summary.csv'}`",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-label", required=True)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--ulog-dir", required=True)
    parser.add_argument("--fit-csv", required=True)
    parser.add_argument("--out-dir", required=True)
    parser.add_argument("--fit-topic", default="vehicle_gps_position")
    parser.add_argument("--fit-subset", default="all")
    parser.add_argument(
        "--projection-mode",
        choices=["gps_fit", "raw_wgs84_enu", "px4_sphere_anisotropic"],
        default="gps_fit",
        help="IEKF XY output-frame normalization before early translation alignment.",
    )
    parser.add_argument("--max-join-dt-sec", type=float, default=0.03)
    parser.add_argument(
        "--pair-csv",
        default="",
        help="Optional EKF2/IEKF pair CSV path. Defaults to <run-dir>/ekf_iekf_pairs.csv.",
    )
    parser.add_argument(
        "--groundtruth-source",
        choices=["auto", "local", "global"],
        default="auto",
        help="Use local groundtruth, global groundtruth converted to ENU, or auto fallback when local ref_lat/ref_lon are invalid.",
    )
    parser.add_argument("--align-start-sec", type=float, default=0.0)
    parser.add_argument("--align-end-sec", type=float, default=20.0)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    fit = build_projection(
        Path(args.fit_csv),
        args.fit_topic,
        args.fit_subset,
        args.projection_mode,
        Path(args.ulog_dir),
    )
    raw = joined_rows(
        args.run_label,
        Path(args.run_dir),
        Path(args.ulog_dir),
        fit,
        args.max_join_dt_sec,
        args.groundtruth_source,
        Path(args.pair_csv) if args.pair_csv else None,
    )
    offsets = compute_offsets(raw, args.align_start_sec, args.align_end_sec)
    if int(offsets["align_rows"]) <= 0:
        raise RuntimeError("no rows available for groundtruth alignment window")
    joined = apply_offsets(raw, offsets)
    summary_rows: list[dict[str, object]] = []
    summary_rows.extend(summarize(joined, "armed_all", None, None))
    summary_rows.extend(segment_rows(joined))
    for start, end in DEFAULT_WINDOWS:
        summary_rows.extend(summarize(joined, f"{start:.0f}-{end:.0f}", start, end))

    write_csv(out_dir / "groundtruth_joined.csv", joined)
    write_csv(out_dir / "groundtruth_summary.csv", summary_rows)
    (out_dir / "report.md").write_text(build_report(summary_rows, offsets, fit, joined, out_dir), encoding="utf-8")
    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'groundtruth_joined.csv'}")
    print(f"wrote: {out_dir / 'groundtruth_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
