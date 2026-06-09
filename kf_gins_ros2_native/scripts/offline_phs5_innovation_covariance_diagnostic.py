#!/usr/bin/env python3
"""Offline PHS5 GNSS position innovation/covariance diagnostic.

This derives a formal-ish 2D innovation covariance for accepted GNSS position
updates from existing logs:

    S = P_pos_before + R_gnss

where P_pos_before comes from `state_update_debug.csv` and R_gnss comes from
the applied GNSS position stds in `gnss_update_debug.csv`.  It then joins the
updates to groundtruth pair rows to compare innovation pressure, covariance
shrinkage, correction direction, and IEKF-vs-EKF2 error windows across the
PHS5 positive routes and shortgen11 target runs.

This is offline-only and does not start PX4/Gazebo/MAVROS/QGC or change
estimator behavior.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_measurement_geometry_selector as mg
import offline_phs5_selector_replay_after_lag_failure as replay


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "phs5_innovation_covariance_diagnostic_2026-05-10"
WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

POSITIVE_RUNS = replay.POSITIVE_RUNS
TARGET_RUNS = replay.TARGET_RUNS
WINDOWS = [
    ("main_40_180", 40.0, 180.0),
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
    ("140_180", 140.0, 180.0),
]


def finite(value: object) -> bool:
    return mg.finite(value)


def to_float(value: object, default: float = math.nan) -> float:
    return mg.to_float(value, default)


def fmt(value: object, digits: int = 4) -> str:
    return mg.fmt(value, digits)


def pct_fmt(value: object) -> str:
    val = to_float(value)
    return f"{100.0 * val:.1f}%" if finite(val) else "nan"


def mean(values: Iterable[float]) -> float:
    return mg.mean(values)


def percentile(values: Iterable[float], p: float) -> float:
    vals = sorted(value for value in values if finite(value))
    if not vals:
        return math.nan
    if len(vals) == 1:
        return vals[0]
    rank = (len(vals) - 1) * p / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return vals[lo]
    frac = rank - lo
    return vals[lo] * (1.0 - frac) + vals[hi] * frac


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
    return mg.markdown_table(headers, rows)


def norm(east: float, north: float) -> float:
    return math.hypot(east, north) if finite(east) and finite(north) else math.nan


def dot(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    if not all(finite(value) for value in (a_e, a_n, b_e, b_n)):
        return math.nan
    return a_e * b_e + a_n * b_n


def cosine(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    a_h = norm(a_e, a_n)
    b_h = norm(b_e, b_n)
    if not finite(a_h) or not finite(b_h) or a_h < 1e-9 or b_h < 1e-9:
        return math.nan
    return dot(a_e, a_n, b_e, b_n) / (a_h * b_h)


def inv2(a: float, b: float, c: float) -> tuple[float, float, float, float]:
    det = a * c - b * b
    if not finite(det) or abs(det) < 1e-12:
        return math.nan, math.nan, math.nan, math.nan
    return c / det, -b / det, -b / det, a / det


def quad2(north: float, east: float, a: float, b: float, c: float) -> float:
    i00, i01, i10, i11 = inv2(a, b, c)
    if not all(finite(value) for value in (north, east, i00, i01, i10, i11)):
        return math.nan
    return north * (i00 * north + i01 * east) + east * (i10 * north + i11 * east)


def matvec2(a: float, b: float, c: float, north: float, east: float) -> tuple[float, float]:
    if not all(finite(value) for value in (a, b, c, north, east)):
        return math.nan, math.nan
    return a * north + b * east, b * north + c * east


def trace_std(pn: float, pe: float) -> float:
    if not finite(pn) or not finite(pe):
        return math.nan
    return math.sqrt(max(0.0, 0.5 * (pn + pe)))


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


def llh_to_local_enu(
    lat_deg: float,
    lon_deg: float,
    h_m: float,
    origin_ecef: tuple[float, float, float],
    origin_lat_rad: float,
    origin_lon_rad: float,
) -> tuple[float, float, float]:
    if not all(finite(value) for value in (lat_deg, lon_deg, h_m)):
        return math.nan, math.nan, math.nan
    lat_rad = math.radians(lat_deg)
    lon_rad = math.radians(lon_deg)
    return ecef_to_enu(llh_to_ecef(lat_rad, lon_rad, h_m), origin_ecef, origin_lat_rad, origin_lon_rad)


def local_origin_from_state_rows(rows: Sequence[dict[str, str]]) -> tuple[tuple[float, float, float], float, float] | None:
    for row in rows:
        lat_deg = to_float(row.get("lat_after_deg"), to_float(row.get("lat_before_deg")))
        lon_deg = to_float(row.get("lon_after_deg"), to_float(row.get("lon_before_deg")))
        h_m = to_float(row.get("h_after_m"), to_float(row.get("h_before_m")))
        if all(finite(value) for value in (lat_deg, lon_deg, h_m)):
            lat_rad = math.radians(lat_deg)
            lon_rad = math.radians(lon_deg)
            return llh_to_ecef(lat_rad, lon_rad, h_m), lat_rad, lon_rad
    return None


def nearest(
    rows: Sequence[dict[str, object]],
    times: Sequence[float],
    t: float,
    max_dt: float,
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
    return (row, dt) if dt <= max_dt else (None, dt)


def load_joined(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    path = mg.mode_dir(run_dir) / "raw_wgs84_enu" / "groundtruth_joined.csv"
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key, value in raw.items():
            if key not in {"run", "mavros_mode"}:
                row[key] = to_float(value)
        rows.append(row)
    rows.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
    return rows, [to_float(row.get("pair_ros_time_sec")) for row in rows]


def load_update_rows(spec: replay.RunSpec) -> list[dict[str, object]]:
    state_rows = [
        row
        for row in read_csv(spec.path / "state_update_debug.csv")
        if row.get("event_type") == "gnss_position" and to_float(row.get("applied"), 0.0) > 0.5
    ]
    origin = local_origin_from_state_rows(state_rows)
    if origin is None:
        return []
    origin_ecef, origin_lat_rad, origin_lon_rad = origin
    gnss_rows = [
        row
        for row in read_csv(spec.path / "gnss_update_debug.csv")
        if to_float(row.get("gnss_position_applied"), 0.0) > 0.5
    ]
    gnss_rows.sort(key=lambda row: to_float(row.get("update_time_sec")))
    gnss_update_times = [to_float(row.get("update_time_sec")) for row in gnss_rows]
    gnss_ros_times = [to_float(row.get("ros_time_sec")) for row in gnss_rows]
    joined_rows, joined_times = load_joined(spec.path)

    out: list[dict[str, object]] = []
    for state in state_rows:
        seq = int(to_float(state.get("sequence"), -1.0))
        state_update_t = to_float(state.get("update_time_sec"))
        gnss, gnss_join_dt = nearest(gnss_rows, gnss_update_times, state_update_t, 0.03)
        if gnss is None:
            gnss, gnss_join_dt = nearest(gnss_rows, gnss_ros_times, to_float(state.get("ros_time_sec")), 0.15)
            if gnss is None:
                continue
        ros_t = to_float(state.get("ros_time_sec"))
        joined, join_dt = nearest(joined_rows, joined_times, ros_t, 0.15)
        if joined is None:
            continue

        p_nn = to_float(state.get("cov_pos_n_before_m2"))
        p_ee = to_float(state.get("cov_pos_e_before_m2"))
        p_ne = to_float(state.get("cov_pos_ne_before_m2"), 0.0)
        p_nn_after = to_float(state.get("cov_pos_n_after_m2"))
        p_ee_after = to_float(state.get("cov_pos_e_after_m2"))
        p_ne_after = to_float(state.get("cov_pos_ne_after_m2"), 0.0)
        r_n_std = to_float(gnss.get("gnss_position_std_n_m"))
        r_e_std = to_float(gnss.get("gnss_position_std_e_m"))
        residual_n = to_float(gnss.get("gnss_position_residual_n_m"))
        residual_e = to_float(gnss.get("gnss_position_residual_e_m"))
        dx_n = to_float(state.get("dx_pos_n_m"))
        dx_e = to_float(state.get("dx_pos_e_m"))

        s_nn = p_nn + r_n_std * r_n_std
        s_ee = p_ee + r_e_std * r_e_std
        s_ne = p_ne
        nis = quad2(residual_n, residual_e, s_nn, s_ne, s_ee)
        nis_r_only = quad2(residual_n, residual_e, r_n_std * r_n_std, 0.0, r_e_std * r_e_std)

        s_i00, s_i01, s_i10, s_i11 = inv2(s_nn, s_ne, s_ee)
        kr_n_n, kr_n_e = matvec2(p_nn, p_ne, p_ee, s_i00 * residual_n + s_i01 * residual_e, s_i10 * residual_n + s_i11 * residual_e)
        pred_dx_n = kr_n_n
        pred_dx_e = kr_n_e

        err_e = to_float(joined.get("iekf_normalized_aligned_x_m")) - to_float(joined.get("gt_x_m"))
        err_n = to_float(joined.get("iekf_normalized_aligned_y_m")) - to_float(joined.get("gt_y_m"))
        err_h = norm(err_e, err_n)
        need_e = -err_e
        need_n = -err_n
        need_h = err_h

        before_e, before_n, _before_u = llh_to_local_enu(
            to_float(state.get("lat_before_deg")),
            to_float(state.get("lon_before_deg")),
            to_float(state.get("h_before_m")),
            origin_ecef,
            origin_lat_rad,
            origin_lon_rad,
        )
        after_e, after_n, _after_u = llh_to_local_enu(
            to_float(state.get("lat_after_deg")),
            to_float(state.get("lon_after_deg")),
            to_float(state.get("h_after_m")),
            origin_ecef,
            origin_lat_rad,
            origin_lon_rad,
        )
        offset_e = to_float(joined.get("iekf_normalized_aligned_x_m")) - after_e
        offset_n = to_float(joined.get("iekf_normalized_aligned_y_m")) - after_n
        gt_e = to_float(joined.get("gt_x_m"))
        gt_n = to_float(joined.get("gt_y_m"))
        before_err_e = before_e + offset_e - gt_e
        before_err_n = before_n + offset_n - gt_n
        after_err_e = after_e + offset_e - gt_e
        after_err_n = after_n + offset_n - gt_n
        before_err_h = norm(before_err_e, before_err_n)
        after_err_h = norm(after_err_e, after_err_n)
        update_error_delta = after_err_h - before_err_h if finite(after_err_h) and finite(before_err_h) else math.nan

        dx_h = norm(dx_e, dx_n)
        residual_h = norm(residual_e, residual_n)
        pred_dx_h = norm(pred_dx_e, pred_dx_n)
        cov_before_std = trace_std(p_nn, p_ee)
        cov_after_std = trace_std(p_nn_after, p_ee_after)
        cov_shrink = cov_before_std - cov_after_std if finite(cov_before_std) and finite(cov_after_std) else math.nan

        out.append(
            {
                "run": spec.label,
                "role": spec.role,
                "group": spec.group,
                "sequence": seq,
                "gnss_sequence": int(to_float(gnss.get("sequence"), -1.0)),
                "ros_time_sec": ros_t,
                "state_update_time_sec": state_update_t,
                "gnss_join_dt_sec": gnss_join_dt,
                "time_since_arm_sec": to_float(joined.get("time_since_arm_sec")),
                "join_dt_sec": join_dt,
                "mavros_mode": state.get("mavros_mode", ""),
                "turning_now": to_float(state.get("turning_now"), 0.0),
                "post_turn_context": to_float(state.get("post_turn_context"), 0.0),
                "armed_cruise_context": to_float(state.get("armed_cruise_context"), 0.0),
                "speed_mps": to_float(state.get("horizontal_speed_mps")),
                "gyro_deg_s": to_float(state.get("gyro_deg_s")),
                "iekf_error_m": err_h,
                "ekf2_error_m": to_float(joined.get("ekf2_error_xy_m")),
                "iekf_minus_ekf2_m": err_h - to_float(joined.get("ekf2_error_xy_m")),
                "update_before_error_h_m": before_err_h,
                "update_after_error_h_m": after_err_h,
                "update_error_delta_h_m": update_error_delta,
                "update_error_improvement_h_m": -update_error_delta if finite(update_error_delta) else math.nan,
                "residual_n_m": residual_n,
                "residual_e_m": residual_e,
                "residual_h_m": residual_h,
                "residual_std_n_m": r_n_std,
                "residual_std_e_m": r_e_std,
                "residual_std_h_rms_m": trace_std(r_n_std * r_n_std, r_e_std * r_e_std),
                "p_pos_n_before_m2": p_nn,
                "p_pos_e_before_m2": p_ee,
                "p_pos_ne_before_m2": p_ne,
                "p_pos_n_after_m2": p_nn_after,
                "p_pos_e_after_m2": p_ee_after,
                "p_pos_ne_after_m2": p_ne_after,
                "p_pos_std_before_h_m": cov_before_std,
                "p_pos_std_after_h_m": cov_after_std,
                "p_pos_std_shrink_h_m": cov_shrink,
                "p_pos_std_shrink_frac": cov_shrink / cov_before_std if finite(cov_shrink) and finite(cov_before_std) and cov_before_std > 1e-9 else math.nan,
                "s_pos_n_m2": s_nn,
                "s_pos_e_m2": s_ee,
                "s_pos_ne_m2": s_ne,
                "s_pos_std_h_m": trace_std(s_nn, s_ee),
                "formal_hnis_2d": nis,
                "formal_whitened_norm": math.sqrt(nis) if finite(nis) and nis >= 0.0 else math.nan,
                "r_only_hnis_2d": nis_r_only,
                "dx_pos_n_m": dx_n,
                "dx_pos_e_m": dx_e,
                "dx_pos_h_m": dx_h,
                "dx_over_residual_h": dx_h / residual_h if finite(dx_h) and finite(residual_h) and residual_h > 1e-9 else math.nan,
                "pred_dx_n_m": pred_dx_n,
                "pred_dx_e_m": pred_dx_e,
                "pred_dx_h_m": pred_dx_h,
                "actual_over_pred_dx_h": dx_h / pred_dx_h if finite(dx_h) and finite(pred_dx_h) and pred_dx_h > 1e-9 else math.nan,
                "dx_vs_residual_cos": cosine(dx_e, dx_n, residual_e, residual_n),
                "pred_dx_vs_residual_cos": cosine(pred_dx_e, pred_dx_n, residual_e, residual_n),
                "residual_vs_needed_cos": cosine(residual_e, residual_n, need_e, need_n),
                "dx_vs_needed_cos": cosine(dx_e, dx_n, need_e, need_n),
                "pred_dx_vs_needed_cos": cosine(pred_dx_e, pred_dx_n, need_e, need_n),
                "residual_along_needed_m": dot(residual_e, residual_n, need_e, need_n) / need_h if finite(need_h) and need_h > 1e-9 else math.nan,
                "dx_along_needed_m": dot(dx_e, dx_n, need_e, need_n) / need_h if finite(need_h) and need_h > 1e-9 else math.nan,
                "pred_dx_along_needed_m": dot(pred_dx_e, pred_dx_n, need_e, need_n) / need_h if finite(need_h) and need_h > 1e-9 else math.nan,
            }
        )
    out.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return out


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def summarize(rows: Sequence[dict[str, object]], run: str, label: str, start: float, end: float) -> dict[str, object]:
    subset = [row for row in rows if row.get("run") == run and in_window(row, start, end)]
    return {
        "run": run,
        "group": "positive" if run in POSITIVE_RUNS else "target",
        "window": label,
        "rows": len(subset),
        "iekf_rmse_m": rmse(to_float(row.get("iekf_error_m")) for row in subset),
        "ekf2_rmse_m": rmse(to_float(row.get("ekf2_error_m")) for row in subset),
        "iekf_minus_ekf2_rmse_m": rmse(to_float(row.get("iekf_error_m")) for row in subset)
        - rmse(to_float(row.get("ekf2_error_m")) for row in subset),
        "update_before_error_mean_m": mean(to_float(row.get("update_before_error_h_m")) for row in subset),
        "update_after_error_mean_m": mean(to_float(row.get("update_after_error_h_m")) for row in subset),
        "update_error_delta_mean_m": mean(to_float(row.get("update_error_delta_h_m")) for row in subset),
        "update_error_delta_p95_m": percentile([to_float(row.get("update_error_delta_h_m")) for row in subset], 95.0),
        "update_worsen_frac": mean(1.0 if to_float(row.get("update_error_delta_h_m")) > 0.0 else 0.0 for row in subset),
        "formal_hnis_mean": mean(to_float(row.get("formal_hnis_2d")) for row in subset),
        "formal_hnis_p95": percentile([to_float(row.get("formal_hnis_2d")) for row in subset], 95.0),
        "formal_whitened_mean": mean(to_float(row.get("formal_whitened_norm")) for row in subset),
        "formal_hnis_gt_5p99_frac": mean(1.0 if to_float(row.get("formal_hnis_2d")) > 5.991 else 0.0 for row in subset),
        "residual_h_mean_m": mean(to_float(row.get("residual_h_m")) for row in subset),
        "residual_h_p95_m": percentile([to_float(row.get("residual_h_m")) for row in subset], 95.0),
        "residual_std_h_mean_m": mean(to_float(row.get("residual_std_h_rms_m")) for row in subset),
        "s_pos_std_h_mean_m": mean(to_float(row.get("s_pos_std_h_m")) for row in subset),
        "p_pos_std_before_mean_m": mean(to_float(row.get("p_pos_std_before_h_m")) for row in subset),
        "p_pos_std_shrink_mean_m": mean(to_float(row.get("p_pos_std_shrink_h_m")) for row in subset),
        "p_pos_std_shrink_frac_mean": mean(to_float(row.get("p_pos_std_shrink_frac")) for row in subset),
        "dx_h_mean_m": mean(to_float(row.get("dx_pos_h_m")) for row in subset),
        "dx_over_residual_mean": mean(to_float(row.get("dx_over_residual_h")) for row in subset),
        "actual_over_pred_dx_mean": mean(to_float(row.get("actual_over_pred_dx_h")) for row in subset),
        "residual_vs_needed_cos_mean": mean(to_float(row.get("residual_vs_needed_cos")) for row in subset),
        "dx_vs_needed_cos_mean": mean(to_float(row.get("dx_vs_needed_cos")) for row in subset),
        "pred_dx_vs_needed_cos_mean": mean(to_float(row.get("pred_dx_vs_needed_cos")) for row in subset),
        "residual_along_needed_mean_m": mean(to_float(row.get("residual_along_needed_m")) for row in subset),
        "dx_along_needed_mean_m": mean(to_float(row.get("dx_along_needed_m")) for row in subset),
        "pred_dx_along_needed_mean_m": mean(to_float(row.get("pred_dx_along_needed_m")) for row in subset),
        "turning_frac": mean(to_float(row.get("turning_now")) for row in subset),
        "post_turn_frac": mean(to_float(row.get("post_turn_context")) for row in subset),
        "armed_cruise_frac": mean(to_float(row.get("armed_cruise_context")) for row in subset),
    }


def build_window_summaries(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    runs = [spec.label for spec in replay.RUNS]
    for run in runs:
        for label, start, end in WINDOWS:
            out.append(summarize(rows, run, label, start, end))
    return out


def group_summary(window_rows: Sequence[dict[str, object]], group: str, window: str) -> dict[str, object]:
    subset = [row for row in window_rows if row.get("group") == group and row.get("window") == window and to_float(row.get("rows")) > 0.0]
    weights = [to_float(row.get("rows")) for row in subset]

    def wmean(field: str) -> float:
        vals = [(to_float(row.get(field)), to_float(row.get("rows"))) for row in subset]
        vals = [(value, weight) for value, weight in vals if finite(value) and finite(weight) and weight > 0.0]
        total = sum(weight for _value, weight in vals)
        return sum(value * weight for value, weight in vals) / total if total > 0.0 else math.nan

    return {
        "group": group,
        "window": window,
        "runs": len(subset),
        "rows": sum(weights),
        "iekf_minus_ekf2_rmse_m": wmean("iekf_minus_ekf2_rmse_m"),
        "formal_hnis_mean": wmean("formal_hnis_mean"),
        "formal_hnis_p95_mean": wmean("formal_hnis_p95"),
        "formal_hnis_gt_5p99_frac": wmean("formal_hnis_gt_5p99_frac"),
        "update_error_delta_mean_m": wmean("update_error_delta_mean_m"),
        "update_worsen_frac": wmean("update_worsen_frac"),
        "residual_h_mean_m": wmean("residual_h_mean_m"),
        "residual_std_h_mean_m": wmean("residual_std_h_mean_m"),
        "p_pos_std_before_mean_m": wmean("p_pos_std_before_mean_m"),
        "p_pos_std_shrink_frac_mean": wmean("p_pos_std_shrink_frac_mean"),
        "dx_over_residual_mean": wmean("dx_over_residual_mean"),
        "residual_vs_needed_cos_mean": wmean("residual_vs_needed_cos_mean"),
        "dx_vs_needed_cos_mean": wmean("dx_vs_needed_cos_mean"),
        "dx_along_needed_mean_m": wmean("dx_along_needed_mean_m"),
    }


def build_group_summaries(window_rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for group in ("positive", "target"):
        for window, _start, _end in WINDOWS:
            out.append(group_summary(window_rows, group, window))
    return out


def feature_separation(window_rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    target = [
        row for row in window_rows
        if row.get("group") == "target" and row.get("window") in {"140_160", "160_180"} and to_float(row.get("rows")) > 0.0
    ]
    positive = [
        row for row in window_rows
        if row.get("group") == "positive" and row.get("window") == "main_40_180" and to_float(row.get("rows")) > 0.0
    ]
    fields = [
        "formal_hnis_mean",
        "formal_hnis_p95",
        "formal_hnis_gt_5p99_frac",
        "update_error_delta_mean_m",
        "update_worsen_frac",
        "residual_h_mean_m",
        "residual_std_h_mean_m",
        "s_pos_std_h_mean_m",
        "p_pos_std_before_mean_m",
        "p_pos_std_shrink_frac_mean",
        "dx_h_mean_m",
        "dx_over_residual_mean",
        "actual_over_pred_dx_mean",
        "residual_vs_needed_cos_mean",
        "dx_vs_needed_cos_mean",
        "pred_dx_vs_needed_cos_mean",
        "residual_along_needed_mean_m",
        "dx_along_needed_mean_m",
        "pred_dx_along_needed_mean_m",
    ]
    out: list[dict[str, object]] = []
    for field in fields:
        target_vals = [to_float(row.get(field)) for row in target]
        positive_vals = [to_float(row.get(field)) for row in positive]
        target_mean = mean(target_vals)
        positive_mean = mean(positive_vals)
        target_var = mean((value - target_mean) ** 2 for value in target_vals if finite(value) and finite(target_mean))
        positive_var = mean((value - positive_mean) ** 2 for value in positive_vals if finite(value) and finite(positive_mean))
        pooled = math.sqrt(target_var + positive_var) if finite(target_var) and finite(positive_var) else math.nan
        out.append(
            {
                "feature": field,
                "target_bad_count": len([value for value in target_vals if finite(value)]),
                "positive_main_count": len([value for value in positive_vals if finite(value)]),
                "target_bad_mean": target_mean,
                "positive_main_mean": positive_mean,
                "diff_target_minus_positive": target_mean - positive_mean,
                "separation_score": abs(target_mean - positive_mean) / pooled if finite(pooled) and pooled > 1e-9 else math.nan,
            }
        )
    out.sort(key=lambda row: to_float(row.get("separation_score")), reverse=True)
    return out


def table_group(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["group"],
            row["window"],
            int(to_float(row["rows"], 0.0)),
            fmt(row["iekf_minus_ekf2_rmse_m"]),
            fmt(row["formal_hnis_mean"]),
            fmt(row["formal_hnis_gt_5p99_frac"]),
            fmt(row["update_error_delta_mean_m"]),
            fmt(row["update_worsen_frac"]),
            fmt(row["residual_h_mean_m"]),
            fmt(row["p_pos_std_before_mean_m"]),
            fmt(row["p_pos_std_shrink_frac_mean"]),
            fmt(row["dx_over_residual_mean"]),
            fmt(row["dx_vs_needed_cos_mean"]),
            fmt(row["dx_along_needed_mean_m"]),
        ]
        for row in rows
    ]


def table_runs(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["window"],
            int(to_float(row["rows"], 0.0)),
            fmt(row["iekf_minus_ekf2_rmse_m"]),
            fmt(row["formal_hnis_mean"]),
            fmt(row["formal_hnis_gt_5p99_frac"]),
            fmt(row["update_error_delta_mean_m"]),
            fmt(row["update_worsen_frac"]),
            fmt(row["residual_h_mean_m"]),
            fmt(row["p_pos_std_before_mean_m"]),
            fmt(row["p_pos_std_shrink_frac_mean"]),
            fmt(row["dx_over_residual_mean"]),
            fmt(row["dx_vs_needed_cos_mean"]),
            fmt(row["dx_along_needed_mean_m"]),
        ]
        for row in rows
    ]


def table_sep(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["feature"],
            row["target_bad_count"],
            row["positive_main_count"],
            fmt(row["target_bad_mean"]),
            fmt(row["positive_main_mean"]),
            fmt(row["diff_target_minus_positive"]),
            fmt(row["separation_score"]),
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    update_rows: list[dict[str, object]],
    window_rows: list[dict[str, object]],
    group_rows: list[dict[str, object]],
    sep_rows: list[dict[str, object]],
) -> None:
    focus_group = [
        row for row in group_rows
        if row.get("window") in {"main_40_180", "140_160", "160_180"}
    ]
    focus_runs = [
        row for row in window_rows
        if row.get("window") in {"140_160", "160_180"}
        and (row.get("group") == "target" or row.get("run") == "shortgen04_hld1a_phs5")
    ]
    lines = [
        "# PHS5 innovation/covariance diagnostic",
        "",
        "Date: 2026-05-10",
        "",
        "Offline-only derivation from existing PHS5 logs. No flight stack, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "Derived 2D position innovation covariance: `S = P_pos_before + R_gnss`.",
        "",
        f"accepted position updates joined: `{len(update_rows)}`",
        "",
        "## Group Window Summary",
        "",
        markdown_table(
            [
                "group",
                "window",
                "rows",
                "IEKF-EKF2",
                "HNIS mean",
                "HNIS>5.99",
                "upd delta",
                "worsen",
                "residual h",
                "P std before",
                "P shrink frac",
                "dx/resid",
                "dx need cos",
                "dx along need",
            ],
            table_group(focus_group),
        ),
        "",
        "## Target And Holdout Focus",
        "",
        markdown_table(
            [
                "run",
                "window",
                "rows",
                "IEKF-EKF2",
                "HNIS mean",
                "HNIS>5.99",
                "upd delta",
                "worsen",
                "residual h",
                "P std before",
                "P shrink frac",
                "dx/resid",
                "dx need cos",
                "dx along need",
            ],
            table_runs(focus_runs),
        ),
        "",
        "## Feature Separation",
        "",
        markdown_table(
            ["feature", "target bad n", "positive main n", "target mean", "positive mean", "diff", "score"],
            table_sep(sep_rows[:16]),
        ),
        "",
        "## Readout",
        "",
        "- `HNIS` here is derived offline from logged covariance and GNSS std, not from a newly run estimator.",
        "- `dx need cos` and `dx along need` use groundtruth, so they are diagnostic-only.",
        "- Use this report to decide whether the next mechanism should be covariance/innovation handling or whether more logging is required.",
        "",
        "Generated files:",
        f"- `{out_dir / 'innovation_update_rows.csv'}`",
        f"- `{out_dir / 'innovation_window_summary.csv'}`",
        f"- `{out_dir / 'innovation_group_summary.csv'}`",
        f"- `{out_dir / 'innovation_feature_separation.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    all_updates: list[dict[str, object]] = []
    for spec in replay.RUNS:
        all_updates.extend(load_update_rows(spec))

    window_rows = build_window_summaries(all_updates)
    group_rows = build_group_summaries(window_rows)
    sep_rows = feature_separation(window_rows)

    write_csv(out_dir / "innovation_update_rows.csv", all_updates)
    write_csv(out_dir / "innovation_window_summary.csv", window_rows)
    write_csv(out_dir / "innovation_group_summary.csv", group_rows)
    write_csv(out_dir / "innovation_feature_separation.csv", sep_rows)
    write_report(out_dir, all_updates, window_rows, group_rows, sep_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
