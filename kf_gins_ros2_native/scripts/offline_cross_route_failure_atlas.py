#!/usr/bin/env python3
"""Build a cross-route IEKF/EKF2 failure atlas from existing offline artifacts."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import pandas as pd


WGS84_A_M = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)

KEY_WINDOWS = [
    "segment_mission_all",
    "segment_main_maneuver_40_120",
    "segment_main_maneuver_40_180",
    "segment_rtl_all",
    "segment_landing_tail_300_360",
    "40-60",
    "60-80",
    "80-100",
    "100-120",
    "120-140",
    "140-160",
    "160-180",
]


@dataclass(frozen=True)
class RunSpec:
    label: str
    route: str
    bucket: str
    path: Path
    note: str


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


def mean(values: Iterable[float]) -> float:
    vals = [item for item in values if finite(item)]
    return sum(vals) / len(vals) if vals else math.nan


def percentile(values: Iterable[float], pct: float) -> float:
    vals = sorted(float(item) for item in values if finite(item))
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


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def pct_fmt(value: object) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val * 100.0:.1f}%"


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


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


def default_runs(ws_root: Path) -> list[RunSpec]:
    manual = ws_root / "artifacts" / "manual"
    return [
        RunSpec(
            "manual124_main_proof",
            "mainline_reference",
            "primary_positive",
            manual / "offline_groundtruth_convergence_manual124",
            "paper main proof sample",
        ),
        RunSpec(
            "manual125_main_repeat",
            "mainline_reference",
            "primary_positive",
            manual / "offline_groundtruth_convergence_manual125",
            "paper main proof repeat",
        ),
        RunSpec(
            "zheng02_repeat",
            "zheng02_short_complex",
            "supplement_positive",
            manual / "manual_segment_zheng02_sitl_home_headless_compare_core8_pairlogger_mainmission_performance_repeat",
            "supplementary short-complex route",
        ),
        RunSpec(
            "shortgen01_repeat",
            "shortgen01_s_bend",
            "narrow_positive",
            manual / "manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_performance_repeat_after_cooldown",
            "clean cooldown repeat; original performance run was runtime-unclean",
        ),
        RunSpec(
            "shortgen02_baseline",
            "shortgen02_box_uturn",
            "negative_route_sensitivity",
            manual / "manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_stateupdate_precision_baseline_20260508_230331",
            "baseline/default diagnostic sample with state-update logger",
        ),
        RunSpec(
            "shortgen03_performance",
            "shortgen03_figure8",
            "negative_route_sensitivity",
            manual / "manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_performance",
            "clean shortgen03 performance sample",
        ),
    ]


def summary_path(run_dir: Path) -> Path:
    return run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_summary.csv"


def joined_path(run_dir: Path) -> Path:
    return run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv"


def resolve_summary_path(run_dir: Path) -> Path | None:
    direct = summary_path(run_dir)
    if direct.exists():
        return direct
    fallback = run_dir / "groundtruth_summary.csv"
    if fallback.exists():
        return fallback
    return None


def resolve_joined_path(run_dir: Path) -> Path | None:
    direct = joined_path(run_dir)
    if direct.exists():
        return direct
    fallback = run_dir / "groundtruth_joined.csv"
    if fallback.exists():
        return fallback
    return None


def load_performance_windows(spec: RunSpec) -> list[dict[str, object]]:
    path = resolve_summary_path(spec.path)
    if path is None:
        return []
    df = pd.read_csv(path)
    rows: list[dict[str, object]] = []
    for window in df["window"].drop_duplicates().tolist():
        ekf2 = df[(df["window"] == window) & (df["estimator"] == "ekf2")]
        iekf = df[(df["window"] == window) & (df["estimator"] == "iekf_normalized")]
        if ekf2.empty or iekf.empty:
            continue
        e = ekf2.iloc[0]
        i = iekf.iloc[0]
        ekf2_mean = to_float(e.get("xy_mean_m"))
        iekf_mean = to_float(i.get("xy_mean_m"))
        ekf2_p95 = to_float(e.get("xy_p95_m"))
        iekf_p95 = to_float(i.get("xy_p95_m"))
        ekf2_rmse = to_float(e.get("xy_rmse_m"))
        iekf_rmse = to_float(i.get("xy_rmse_m"))
        mean_delta = iekf_mean - ekf2_mean
        p95_delta = iekf_p95 - ekf2_p95
        rmse_delta = iekf_rmse - ekf2_rmse
        rows.append(
            {
                "label": spec.label,
                "route": spec.route,
                "bucket": spec.bucket,
                "window": window,
                "ekf2_rows": int(to_float(e.get("rows"), 0)),
                "iekf_rows": int(to_float(i.get("rows"), 0)),
                "ekf2_mean_m": ekf2_mean,
                "iekf_mean_m": iekf_mean,
                "mean_delta_iekf_minus_ekf2_m": mean_delta,
                "mean_reduction_frac": (ekf2_mean - iekf_mean) / ekf2_mean if ekf2_mean > 1.0e-9 else math.nan,
                "ekf2_rmse_m": ekf2_rmse,
                "iekf_rmse_m": iekf_rmse,
                "rmse_delta_iekf_minus_ekf2_m": rmse_delta,
                "rmse_reduction_frac": (ekf2_rmse - iekf_rmse) / ekf2_rmse if ekf2_rmse > 1.0e-9 else math.nan,
                "ekf2_p95_m": ekf2_p95,
                "iekf_p95_m": iekf_p95,
                "p95_delta_iekf_minus_ekf2_m": p95_delta,
                "outcome": "iekf_better" if mean_delta < -0.01 else ("ekf2_better" if mean_delta > 0.01 else "tie"),
            }
        )
    return rows


def route_summary(spec: RunSpec, perf_rows: list[dict[str, object]]) -> dict[str, object]:
    by_window = {str(row["window"]): row for row in perf_rows}

    def delta(window: str) -> float:
        row = by_window.get(window)
        return to_float(row.get("mean_delta_iekf_minus_ekf2_m")) if row else math.nan

    def reduction(window: str) -> float:
        row = by_window.get(window)
        return to_float(row.get("mean_reduction_frac")) if row else math.nan

    numeric_windows = []
    for row in perf_rows:
        window = str(row["window"])
        if "-" not in window or window.startswith("segment_"):
            continue
        start = to_float(window.split("-", 1)[0])
        if finite(start):
            numeric_windows.append((start, window, row))
    numeric_windows.sort()
    bad_numeric = [
        (start, window, row)
        for start, window, row in numeric_windows
        if start >= 40.0 and to_float(row.get("mean_delta_iekf_minus_ekf2_m")) > 0.03
    ]
    worst = max(
        perf_rows,
        key=lambda row: to_float(row.get("mean_delta_iekf_minus_ekf2_m"), -1.0e9),
        default={},
    )
    mission_delta = delta("segment_mission_all")
    main_delta = delta("segment_main_maneuver_40_120")
    main_ext_delta = delta("segment_main_maneuver_40_180")
    rtl_delta = delta("segment_rtl_all")
    landing_delta = delta("segment_landing_tail_300_360")

    tail_ok = (not finite(rtl_delta) or rtl_delta <= 0.03) and (
        not finite(landing_delta) or landing_delta <= 0.03
    )
    if mission_delta < -0.03 and main_delta < -0.03 and tail_ok:
        verdict = "broad_positive"
    elif mission_delta < -0.03 and main_delta < -0.03:
        verdict = "main_positive_tail_limited"
    elif mission_delta <= 0.03 and main_delta < -0.03:
        verdict = "narrow_positive"
    elif mission_delta > 0.03 or main_ext_delta > 0.03:
        verdict = "route_sensitive_negative"
    else:
        verdict = "mixed_or_tie"

    return {
        "label": spec.label,
        "route": spec.route,
        "bucket": spec.bucket,
        "note": spec.note,
        "verdict": verdict,
        "mission_mean_delta_m": mission_delta,
        "main_40_120_mean_delta_m": main_delta,
        "main_40_180_mean_delta_m": main_ext_delta,
        "rtl_mean_delta_m": rtl_delta,
        "landing_tail_mean_delta_m": landing_delta,
        "mission_mean_reduction_frac": reduction("segment_mission_all"),
        "main_40_120_reduction_frac": reduction("segment_main_maneuver_40_120"),
        "worst_window": worst.get("window", ""),
        "worst_window_mean_delta_m": worst.get("mean_delta_iekf_minus_ekf2_m", math.nan),
        "first_bad_window": bad_numeric[0][1] if bad_numeric else "",
        "first_bad_window_start_s": bad_numeric[0][0] if bad_numeric else math.nan,
    }


def load_fit(run_dir: Path) -> dict[str, float] | None:
    for rel in (
        "offline_ulog_gps_groundtruth_diag/gps_groundtruth_fit.csv",
        "offline_gps_groundtruth_projection_diag/gps_groundtruth_fit.csv",
    ):
        path = run_dir / rel
        if not path.exists():
            continue
        with path.open(newline="") as handle:
            for row in csv.DictReader(handle):
                if row.get("topic") == "vehicle_gps_position" and row.get("subset") == "all":
                    scale = to_float(row.get("scale"))
                    angle_deg = to_float(row.get("angle_deg"))
                    angle = math.radians(angle_deg)
                    return {
                        "scale": scale,
                        "angle_deg": angle_deg,
                        "a_real": scale * math.cos(angle),
                        "a_imag": scale * math.sin(angle),
                    }
    return None


def find_groundtruth_csv(run_dir: Path) -> Path | None:
    matches = sorted(run_dir.rglob("*_vehicle_local_position_groundtruth_0.csv"))
    return matches[0] if matches else None


def nearest(rows: list[dict[str, object]], times: list[float], t: float, max_dt: float) -> tuple[dict[str, object] | None, float]:
    idx = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for j in (idx - 1, idx):
        if 0 <= j < len(rows):
            candidates.append((abs(times[j] - t), rows[j]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    if dt > max_dt:
        return None, dt
    return row, dt


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


def load_groundtruth_ref_and_velocity(run_dir: Path) -> tuple[float, float, tuple[float, float, float], list[dict[str, object]], list[float]] | None:
    path = find_groundtruth_csv(run_dir)
    if path is None:
        return None
    rows = list(csv.DictReader(path.open(newline="")))
    if not rows:
        return None
    first = rows[0]
    ref_lat = math.radians(to_float(first.get("ref_lat")))
    ref_lon = math.radians(to_float(first.get("ref_lon")))
    ref_alt = to_float(first.get("ref_alt"))
    if not all(finite(item) for item in (ref_lat, ref_lon, ref_alt)):
        return None
    origin = llh_to_ecef(ref_lat, ref_lon, ref_alt)
    velocity_rows: list[dict[str, object]] = []
    for row in rows:
        t = to_float(row.get("timestamp")) * 1.0e-6
        v_n = to_float(row.get("vx"))
        v_e = to_float(row.get("vy"))
        if all(finite(item) for item in (t, v_n, v_e)):
            velocity_rows.append({"timestamp_sec": t, "vN_mps": v_n, "vE_mps": v_e})
    velocity_rows.sort(key=lambda row: to_float(row["timestamp_sec"]))
    return ref_lat, ref_lon, origin, velocity_rows, [to_float(row["timestamp_sec"]) for row in velocity_rows]


def state_update_metrics(spec: RunSpec) -> list[dict[str, object]]:
    state_path = spec.path / "state_update_debug.csv"
    joined = resolve_joined_path(spec.path)
    fit = load_fit(spec.path)
    gt_pack = load_groundtruth_ref_and_velocity(spec.path)
    if not state_path.exists() or joined is None or fit is None or gt_pack is None:
        return []

    ref_lat, ref_lon, origin, gt_vel_rows, gt_vel_times = gt_pack
    joined_df = pd.read_csv(joined).sort_values("pair_ros_time_sec")
    joined_rows = joined_df.to_dict("records")
    joined_times = [to_float(row["pair_ros_time_sec"]) for row in joined_rows]
    state_df = pd.read_csv(state_path)

    def transform_state(lat_deg: object, lon_deg: object, h_m: object) -> tuple[float, float]:
        e, n, _ = ecef_to_enu(
            llh_to_ecef(math.radians(to_float(lat_deg)), math.radians(to_float(lon_deg)), to_float(h_m)),
            origin,
            ref_lat,
            ref_lon,
        )
        return (
            fit["a_real"] * e - fit["a_imag"] * n,
            fit["a_imag"] * e + fit["a_real"] * n,
        )

    pos_rows: list[dict[str, object]] = []
    pos_src = state_df[(state_df["event_type"] == "gnss_position") & (state_df["applied"].astype(int) == 1)]
    for row in pos_src.to_dict("records"):
        ros_t = to_float(row.get("ros_time_sec"))
        g, _ = nearest(joined_rows, joined_times, ros_t, max_dt=0.11)
        if g is None:
            continue
        before_x0, before_y0 = transform_state(row.get("lat_before_deg"), row.get("lon_before_deg"), row.get("h_before_m"))
        after_x0, after_y0 = transform_state(row.get("lat_after_deg"), row.get("lon_after_deg"), row.get("h_after_m"))
        pos_rows.append(
            {
                "ros_time_sec": ros_t,
                "time_since_arm_sec": to_float(g.get("time_since_arm_sec")),
                "gt_x_m": to_float(g.get("gt_x_m")),
                "gt_y_m": to_float(g.get("gt_y_m")),
                "stamp_sec": to_float(g.get("stamp_sec")),
                "before_x0_m": before_x0,
                "before_y0_m": before_y0,
                "after_x0_m": after_x0,
                "after_y0_m": after_y0,
            }
        )
    align = [row for row in pos_rows if 0.0 <= to_float(row["time_since_arm_sec"]) <= 20.0]
    off_x = mean(to_float(row["gt_x_m"]) - to_float(row["after_x0_m"]) for row in align)
    off_y = mean(to_float(row["gt_y_m"]) - to_float(row["after_y0_m"]) for row in align)
    for row in pos_rows:
        before_x = to_float(row["before_x0_m"]) + off_x
        before_y = to_float(row["before_y0_m"]) + off_y
        after_x = to_float(row["after_x0_m"]) + off_x
        after_y = to_float(row["after_y0_m"]) + off_y
        gt_x = to_float(row["gt_x_m"])
        gt_y = to_float(row["gt_y_m"])
        before_error = math.hypot(before_x - gt_x, before_y - gt_y)
        after_error = math.hypot(after_x - gt_x, after_y - gt_y)
        row.update(
            {
                "before_error_m": before_error,
                "after_error_m": after_error,
                "delta_error_m": after_error - before_error,
            }
        )

    inter_rows = []
    for current, nxt in zip(pos_rows, pos_rows[1:]):
        start = to_float(current["time_since_arm_sec"])
        end = to_float(nxt["time_since_arm_sec"])
        if not finite(start) or not finite(end):
            continue
        inter_rows.append(
            {
                "time_since_arm_start_sec": start,
                "time_since_arm_end_sec": end,
                "growth_m": to_float(nxt["before_error_m"]) - to_float(current["after_error_m"]),
                "dt_sec": to_float(nxt["ros_time_sec"]) - to_float(current["ros_time_sec"]),
            }
        )

    vel_rows: list[dict[str, object]] = []
    vel_src = state_df[(state_df["event_type"] == "gnss_velocity") & (state_df["applied"].astype(int) == 1)]
    for row in vel_src.to_dict("records"):
        ros_t = to_float(row.get("ros_time_sec"))
        g, _ = nearest(joined_rows, joined_times, ros_t, max_dt=0.11)
        if g is None:
            continue
        gv, _ = nearest(gt_vel_rows, gt_vel_times, to_float(g.get("stamp_sec")), max_dt=0.03)
        if gv is None:
            continue
        after_x0, after_y0 = transform_state(row.get("lat_after_deg"), row.get("lon_after_deg"), row.get("h_after_m"))
        pos_error_e = after_x0 + off_x - to_float(g.get("gt_x_m"))
        pos_error_n = after_y0 + off_y - to_float(g.get("gt_y_m"))
        err_after_n = to_float(row.get("vN_after_mps")) - to_float(gv.get("vN_mps"))
        err_after_e = to_float(row.get("vE_after_mps")) - to_float(gv.get("vE_mps"))
        pos_norm = math.hypot(pos_error_e, pos_error_n)
        radial = (pos_error_n * err_after_n + pos_error_e * err_after_e) / pos_norm if pos_norm > 1.0e-12 else math.nan
        vel_rows.append(
            {
                "time_since_arm_sec": to_float(g.get("time_since_arm_sec")),
                "after_vel_error_h_mps": math.hypot(err_after_n, err_after_e),
                "after_vel_radial_mps": radial,
            }
        )

    windows = [(40.0, 120.0, "40-120"), (120.0, 140.0, "120-140"), (140.0, 160.0, "140-160"), (160.0, 180.0, "160-180")]
    rows: list[dict[str, object]] = []
    for start, end, name in windows:
        p_subset = [row for row in pos_rows if start <= to_float(row["time_since_arm_sec"]) < end]
        i_subset = [row for row in inter_rows if start <= to_float(row["time_since_arm_start_sec"]) < end and start <= to_float(row["time_since_arm_end_sec"]) < end]
        v_subset = [row for row in vel_rows if start <= to_float(row["time_since_arm_sec"]) < end]
        rows.append(
            {
                "label": spec.label,
                "route": spec.route,
                "window": name,
                "pos_updates": len(p_subset),
                "pos_update_delta_mean_m": mean(to_float(row["delta_error_m"]) for row in p_subset),
                "pos_update_improve_frac": mean(1.0 if to_float(row["delta_error_m"]) < 0.0 else 0.0 for row in p_subset),
                "inter_update_count": len(i_subset),
                "inter_update_growth_mean_m": mean(to_float(row["growth_m"]) for row in i_subset),
                "inter_update_growth_p95_m": percentile([to_float(row["growth_m"]) for row in i_subset], 95.0),
                "inter_update_positive_frac": mean(1.0 if to_float(row["growth_m"]) > 0.0 else 0.0 for row in i_subset),
                "vel_updates": len(v_subset),
                "vel_after_error_mean_mps": mean(to_float(row["after_vel_error_h_mps"]) for row in v_subset),
                "vel_after_radial_mean_mps": mean(to_float(row["after_vel_radial_mps"]) for row in v_subset),
                "vel_after_radial_positive_frac": mean(1.0 if to_float(row["after_vel_radial_mps"]) > 0.0 else 0.0 for row in v_subset),
                "state_update_data": "present",
            }
        )
    return rows


def classify_failure(perf: dict[str, object], state_rows: list[dict[str, object]]) -> dict[str, object]:
    label = str(perf["label"])
    route = str(perf["route"])
    verdict = str(perf["verdict"])
    primary_cause = "not_diagnosed"
    evidence = ""
    next_probe = ""
    if verdict in {"broad_positive", "main_positive_tail_limited"}:
        primary_cause = "positive_or_tail_limit"
        evidence = "IEKF is better in mission/main windows; limitations are route tail or RTL/landing."
        next_probe = "Do not tune this route for short-route route-sensitivity."
    elif verdict == "narrow_positive":
        primary_cause = "late_window_regression"
        evidence = "IEKF wins early/main maneuver but loses later windows or tail."
        next_probe = "Use as positive boundary, not as keeper proof for all geometries."
    elif verdict == "route_sensitive_negative":
        relevant_state = [row for row in state_rows if row["label"] == label]
        late = next((row for row in relevant_state if row["window"] == "160-180"), None)
        mid = next((row for row in relevant_state if row["window"] == "140-160"), None)
        if late:
            pos_improve = to_float(late.get("pos_update_improve_frac"))
            pos_delta = to_float(late.get("pos_update_delta_mean_m"))
            inter_growth = to_float(late.get("inter_update_growth_mean_m"))
            radial = to_float(late.get("vel_after_radial_mean_mps"))
            if pos_improve >= 0.6 and pos_delta < 0.0 and inter_growth > 0.0:
                primary_cause = "post_update_propagation_growth"
                evidence = (
                    f"160-180s GNSS position updates usually improve error "
                    f"(delta {pos_delta:.4f} m, improve {pos_improve:.2f}), "
                    f"but inter-update growth stays positive ({inter_growth:.4f} m)."
                )
                next_probe = "Prefer propagation/motion/yaw-bias process diagnostics over more GNSS update gain tuning."
            elif radial > 0.03:
                primary_cause = "velocity_outward_residual"
                evidence = f"160-180s velocity radial residual remains outward ({radial:.4f} m/s)."
                next_probe = "Velocity residual handling is relevant, but GVR1a/b/c did not clear the mean criterion."
            else:
                primary_cause = "state_update_mixed"
                evidence = "State-update diagnostics are present but do not isolate a single update-side cause."
                next_probe = "Inspect yaw/bias/covariance traces around the worst windows."
        elif mid:
            primary_cause = "mid_late_route_sensitivity_no_late_state"
            evidence = "State-update diagnostics exist but not for the route's worst terminal window."
            next_probe = "Repeat only if this route becomes the next diagnostic target."
        else:
            primary_cause = "performance_only_negative"
            evidence = "Route-level groundtruth metrics are negative, but no state-update logger is available."
            next_probe = "If needed, run one diagnostic-only baseline sample with state-update logger, not a mechanism branch."
    return {
        "label": label,
        "route": route,
        "verdict": verdict,
        "primary_cause": primary_cause,
        "evidence": evidence,
        "next_probe": next_probe,
    }


def build_report(
    route_rows: list[dict[str, object]],
    perf_rows: list[dict[str, object]],
    state_rows: list[dict[str, object]],
    failure_rows: list[dict[str, object]],
    out_dir: Path,
) -> str:
    route_table = []
    for row in route_rows:
        route_table.append(
            [
                row["label"],
                row["bucket"],
                row["verdict"],
                fmt(row["mission_mean_delta_m"]),
                pct_fmt(row["mission_mean_reduction_frac"]),
                fmt(row["main_40_120_mean_delta_m"]),
                pct_fmt(row["main_40_120_reduction_frac"]),
                row["first_bad_window"],
                row["worst_window"],
                fmt(row["worst_window_mean_delta_m"]),
            ]
        )

    window_focus = []
    focus_names = {"segment_mission_all", "segment_main_maneuver_40_120", "segment_main_maneuver_40_180", "120-140", "140-160", "160-180", "segment_rtl_all", "segment_landing_tail_300_360"}
    for row in perf_rows:
        if row["window"] not in focus_names:
            continue
        window_focus.append(
            [
                row["label"],
                row["window"],
                fmt(row["ekf2_mean_m"]),
                fmt(row["iekf_mean_m"]),
                fmt(row["mean_delta_iekf_minus_ekf2_m"]),
                row["outcome"],
            ]
        )

    state_table = []
    for row in state_rows:
        state_table.append(
            [
                row["label"],
                row["window"],
                row["pos_updates"],
                fmt(row["pos_update_delta_mean_m"]),
                fmt(row["pos_update_improve_frac"], 3),
                row["inter_update_count"],
                fmt(row["inter_update_growth_mean_m"]),
                fmt(row["inter_update_growth_p95_m"]),
                fmt(row["vel_after_radial_mean_mps"]),
                fmt(row["vel_after_radial_positive_frac"], 3),
            ]
        )

    failure_table = [
        [row["label"], row["verdict"], row["primary_cause"], row["evidence"], row["next_probe"]]
        for row in failure_rows
    ]

    positives = [row for row in route_rows if row["verdict"] in {"broad_positive", "main_positive_tail_limited", "narrow_positive"}]
    negatives = [row for row in route_rows if row["verdict"] == "route_sensitive_negative"]
    lines = [
        "# Cross-Route IEKF Failure Atlas",
        "",
        "Scope: offline-only synthesis of existing groundtruth and state-update artifacts. No new flight, no rebuild, and no new mechanism branch is implied by this report.",
        "",
        "## Route Verdicts",
        markdown_table(
            [
                "run",
                "bucket",
                "verdict",
                "mission delta m",
                "mission reduction",
                "40-120 delta m",
                "40-120 reduction",
                "first bad window",
                "worst window",
                "worst delta m",
            ],
            route_table,
        ),
        "",
        "Delta means `IEKF mean - EKF2 mean`; negative is better for IEKF.",
        "",
        "## Key Windows",
        markdown_table(["run", "window", "ekf2 mean", "iekf mean", "delta", "outcome"], window_focus),
        "",
        "## State-Update Deep Slice",
        (
            markdown_table(
                [
                    "run",
                    "window",
                    "pos n",
                    "pos delta mean",
                    "pos improve frac",
                    "inter n",
                    "inter growth mean",
                    "inter growth p95",
                    "vel radial mean",
                    "vel radial pos frac",
                ],
                state_table,
            )
            if state_table
            else "No selected atlas run has state-update diagnostics."
        ),
        "",
        "## Failure Classification",
        markdown_table(["run", "verdict", "primary cause", "evidence", "next probe"], failure_table),
        "",
        "## Readout",
        f"- Positive/main-proof samples: {len(positives)} of {len(route_rows)} selected runs.",
        f"- Route-sensitive negatives: {len(negatives)} of {len(route_rows)} selected runs.",
        "- The strongest confirmed positive evidence remains manual124/manual125; zheng02 is a supplementary positive; shortgen01 is only a narrow positive because late windows/tail regress.",
        "- shortgen02 and shortgen03 are clean negative route-sensitivity samples. The common shape is not an immediate startup failure; the loss grows in mid/late mission windows.",
        "- In the only selected run with state-update diagnostics, shortgen02 baseline, GNSS position updates usually reduce error in the 160-180 s window, while inter-update growth remains positive. That points away from simply increasing GNSS position correction strength.",
        "- GVR1a/b/c already tested the velocity-outward branch on shortgen02 and did not clear the mean-growth criterion, so the atlas should not reopen hold/hysteresis tuning.",
        "",
        "## Default Next Step",
        "Do not start another mechanism flight from this report alone. The next code direction should be a narrow propagation/motion-context/yaw-bias diagnostic or model change, and the first validation should be one diagnostic-only baseline repeat on a negative holdout route only after the hypothesis is explicit.",
        "",
        "Generated files:",
        f"- `{out_dir / 'route_verdicts.csv'}`",
        f"- `{out_dir / 'performance_windows.csv'}`",
        f"- `{out_dir / 'state_update_atlas.csv'}`",
        f"- `{out_dir / 'failure_classification.csv'}`",
    ]
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ws-root",
        type=Path,
        default=Path("/home/yang/kf_gins_ws"),
        help="Workspace root containing artifacts/manual.",
    )
    parser.add_argument(
        "--out-dir",
        type=Path,
        default=None,
        help="Output directory. Default: artifacts/manual/cross_route_failure_atlas_2026-05-09",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    ws_root = args.ws_root
    out_dir = args.out_dir or (ws_root / "artifacts" / "manual" / "cross_route_failure_atlas_2026-05-09")
    out_dir.mkdir(parents=True, exist_ok=True)

    specs = [spec for spec in default_runs(ws_root) if spec.path.exists()]
    all_perf: list[dict[str, object]] = []
    route_rows: list[dict[str, object]] = []
    state_rows: list[dict[str, object]] = []
    for spec in specs:
        perf = load_performance_windows(spec)
        all_perf.extend(perf)
        route_rows.append(route_summary(spec, perf))
        state_rows.extend(state_update_metrics(spec))

    failure_rows = [classify_failure(row, state_rows) for row in route_rows]
    write_csv(out_dir / "performance_windows.csv", all_perf)
    write_csv(out_dir / "route_verdicts.csv", route_rows)
    write_csv(out_dir / "state_update_atlas.csv", state_rows)
    write_csv(out_dir / "failure_classification.csv", failure_rows)
    report = build_report(route_rows, all_perf, state_rows, failure_rows, out_dir)
    (out_dir / "cross_route_failure_atlas.md").write_text(report)
    print(f"wrote: {out_dir / 'cross_route_failure_atlas.md'}")
    print(f"wrote: {out_dir / 'route_verdicts.csv'}")
    print(f"wrote: {out_dir / 'performance_windows.csv'}")
    print(f"wrote: {out_dir / 'state_update_atlas.csv'}")
    print(f"wrote: {out_dir / 'failure_classification.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
