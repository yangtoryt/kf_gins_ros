#!/usr/bin/env python3
"""Slice shortgen02 inter-update IEKF error growth by motion and propagation context."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Callable, Iterable

import pandas as pd

from offline_cross_route_failure_atlas import (
    finite,
    load_groundtruth_ref_and_velocity,
    markdown_table,
    mean,
    nearest,
    percentile,
    resolve_joined_path,
    to_float,
    write_csv,
)


DEFAULT_RUN_DIR = Path(
    "/home/yang/kf_gins_ws/artifacts/manual/"
    "manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_"
    "stateupdate_precision_baseline_20260508_230331"
)


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def pct_fmt(value: object) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{100.0 * val:.1f}%"


def bool01(value: object) -> bool:
    return to_float(value, 0.0) > 0.5


def angle_delta_deg(a: object, b: object) -> float:
    da = to_float(a)
    db = to_float(b)
    if not finite(da) or not finite(db):
        return math.nan
    return (da - db + 180.0) % 360.0 - 180.0


def rms_std(*variances: object) -> float:
    vals = [max(to_float(v), 0.0) for v in variances if finite(v)]
    if not vals:
        return math.nan
    return math.sqrt(sum(vals) / len(vals))


def nearest_by_time(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt: float,
) -> dict[str, object] | None:
    row, _ = nearest(rows, times, t, max_dt=max_dt)
    return row


def load_csv_rows(path: Path) -> list[dict[str, object]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def context_label(row: dict[str, object]) -> str:
    if bool01(row.get("turning_now")):
        return "turning"
    if bool01(row.get("post_turn_context")):
        return "post_turn"
    if bool01(row.get("armed_cruise_context")):
        return "armed_cruise"
    return "none"


def speed_bin(value: object) -> str:
    v = to_float(value)
    if not finite(v):
        return "unknown"
    if v < 0.5:
        return "speed_lt_0p5"
    if v < 3.0:
        return "speed_0p5_3"
    return "speed_ge_3"


def gyro_bin(value: object) -> str:
    v = abs(to_float(value))
    if not finite(v):
        return "unknown"
    if v < 2.0:
        return "gyro_lt_2"
    if v < 10.0:
        return "gyro_2_10"
    return "gyro_ge_10"


def yaw_rate_bin(value: object) -> str:
    v = abs(to_float(value))
    if not finite(v):
        return "unknown"
    if v < 0.05:
        return "source_yaw_lt_0p05"
    if v < 0.5:
        return "source_yaw_0p05_0p5"
    return "source_yaw_ge_0p5"


def radial_bin(value: object) -> str:
    v = to_float(value)
    if not finite(v):
        return "unknown"
    if v > 0.02:
        return "vel_outward_gt_0p02"
    if v < -0.02:
        return "vel_inward_lt_-0p02"
    return "vel_radial_near_zero"


def hnis_bin(value: object) -> str:
    v = to_float(value)
    if not finite(v):
        return "unknown"
    if v < 9.0:
        return "hnis_lt_9"
    if v < 25.0:
        return "hnis_9_25"
    return "hnis_ge_25"


def tpn_active_bin(value: object) -> str:
    v = to_float(value)
    if not finite(v):
        return "tpn_unknown"
    return "tpn_active" if v > 0.5 else "tpn_inactive"


def pgr_active_bin(value: object) -> str:
    v = to_float(value)
    if not finite(v):
        return "pgr_unknown"
    return "pgr_active" if v > 0.5 else "pgr_inactive"


def tvd_active_bin(value: object) -> str:
    v = to_float(value)
    if not finite(v):
        return "tvd_unknown"
    return "tvd_active" if v > 0.5 else "tvd_inactive"


def window_label(t: object) -> str:
    v = to_float(t)
    if not finite(v):
        return "unknown"
    if 40.0 <= v < 120.0:
        return "40-120"
    if 120.0 <= v < 140.0:
        return "120-140"
    if 140.0 <= v < 160.0:
        return "140-160"
    if 160.0 <= v < 180.0:
        return "160-180"
    if 120.0 <= v < 180.0:
        return "120-180"
    return "other"


def pearson(xs: Iterable[float], ys: Iterable[float]) -> tuple[float, int]:
    pairs = [(float(x), float(y)) for x, y in zip(xs, ys) if finite(x) and finite(y)]
    n = len(pairs)
    if n < 3:
        return math.nan, n
    mx = mean(x for x, _ in pairs)
    my = mean(y for _, y in pairs)
    vx = sum((x - mx) ** 2 for x, _ in pairs)
    vy = sum((y - my) ** 2 for _, y in pairs)
    if vx <= 0.0 or vy <= 0.0:
        return math.nan, n
    cov = sum((x - mx) * (y - my) for x, y in pairs)
    return cov / math.sqrt(vx * vy), n


def build_position_update_rows(run_dir: Path) -> list[dict[str, object]]:
    gain_path = run_dir / "position_update_gain_diag" / "gnss_position_update_gain_rows.csv"
    state_path = run_dir / "state_update_debug.csv"
    if not gain_path.exists():
        raise FileNotFoundError(f"missing position gain rows: {gain_path}")
    if not state_path.exists():
        raise FileNotFoundError(f"missing state-update debug CSV: {state_path}")

    gain_rows = load_csv_rows(gain_path)
    state_df = pd.read_csv(state_path)
    state_rows = state_df.to_dict("records")
    pos_state_by_seq = {
        int(to_float(row.get("sequence"))): row
        for row in state_rows
        if row.get("event_type") == "gnss_position" and finite(row.get("sequence"))
    }
    vel_rows = [row for row in state_rows if row.get("event_type") == "gnss_velocity"]
    vel_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    vel_times = [to_float(row.get("ros_time_sec")) for row in vel_rows]
    heading_path = run_dir / "heading_update_debug.csv"
    heading_rows = load_csv_rows(heading_path) if heading_path.exists() else []
    heading_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    heading_times = [to_float(row.get("ros_time_sec")) for row in heading_rows]

    gnss_path = run_dir / "gnss_update_debug.csv"
    if not gnss_path.exists():
        gnss_path = run_dir / "gnss_nis_debug.csv"
    gnss_rows = load_csv_rows(gnss_path) if gnss_path.exists() else []
    gnss_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    gnss_times = [to_float(row.get("ros_time_sec")) for row in gnss_rows]

    joined_path = resolve_joined_path(run_dir)
    gt_pack = load_groundtruth_ref_and_velocity(run_dir)
    if joined_path is None or gt_pack is None:
        raise FileNotFoundError("missing groundtruth joined or ULog groundtruth velocity source")
    joined_rows = pd.read_csv(joined_path).sort_values("pair_ros_time_sec").to_dict("records")
    joined_times = [to_float(row.get("pair_ros_time_sec")) for row in joined_rows]
    _, _, _, gt_vel_rows, gt_vel_times = gt_pack

    enriched: list[dict[str, object]] = []
    for row in gain_rows:
        seq = int(to_float(row.get("sequence"), -1.0))
        ros_t = to_float(row.get("ros_time_sec"))
        state = pos_state_by_seq.get(seq)
        vel = nearest_by_time(vel_rows, vel_times, ros_t, 0.03)
        heading = nearest_by_time(heading_rows, heading_times, ros_t, 0.15)
        gnss = nearest_by_time(gnss_rows, gnss_times, ros_t, 0.11)
        joined = nearest_by_time(joined_rows, joined_times, ros_t, 0.11)
        gt_vel = None
        if joined is not None:
            gt_vel = nearest_by_time(gt_vel_rows, gt_vel_times, to_float(joined.get("stamp_sec")), 0.03)

        pos_ex = to_float(row.get("error_after_x_m"))
        pos_ey = to_float(row.get("error_after_y_m"))
        pos_norm = math.hypot(pos_ex, pos_ey)
        v_err_n = math.nan
        v_err_e = math.nan
        v_err_h = math.nan
        v_radial = math.nan
        if vel is not None and gt_vel is not None:
            v_err_n = to_float(vel.get("vN_after_mps")) - to_float(gt_vel.get("vN_mps"))
            v_err_e = to_float(vel.get("vE_after_mps")) - to_float(gt_vel.get("vE_mps"))
            v_err_h = math.hypot(v_err_n, v_err_e)
            if pos_norm > 1.0e-9:
                v_radial = (pos_ex * v_err_n + pos_ey * v_err_e) / pos_norm

        out = dict(row)
        out.update(
            {
                "context": context_label(row),
                "speed_bin": speed_bin(row.get("horizontal_speed_mps")),
                "gyro_bin": gyro_bin(joined.get("gyro_deg_s") if joined else row.get("horizontal_speed_mps")),
                "source_yaw_rate_bin": yaw_rate_bin(joined.get("source_yaw_rate_deg_s") if joined else math.nan),
                "vel_after_error_h_mps": v_err_h,
                "vel_after_radial_mps": v_radial,
                "vel_after_radial_bin": radial_bin(v_radial),
                "heading_residual_before_abs_deg": abs(to_float(heading.get("residual_before_deg"))) if heading else math.nan,
                "heading_residual_after_abs_deg": abs(to_float(heading.get("residual_after_deg"))) if heading else math.nan,
                "heading_yaw_correction_abs_deg": abs(to_float(heading.get("yaw_correction_abs_deg"))) if heading else math.nan,
                "joined_gyro_deg_s": to_float(joined.get("gyro_deg_s")) if joined else math.nan,
                "joined_source_yaw_rate_deg_s": to_float(joined.get("source_yaw_rate_deg_s")) if joined else math.nan,
                "joined_horizontal_speed_mps": to_float(joined.get("horizontal_speed_mps")) if joined else math.nan,
                "pair_join_dt_sec": abs(to_float(joined.get("pair_ros_time_sec")) - ros_t) if joined else math.nan,
                "gnss_latest_imu_effective_dt_sec": to_float(gnss.get("latest_imu_effective_dt_sec")) if gnss else math.nan,
                "gnss_medium_gap_active": to_float(gnss.get("medium_gap_active")) if gnss else math.nan,
                "gnss_medium_gap_segmented": to_float(gnss.get("medium_gap_segmented")) if gnss else math.nan,
                "gnss_native_velocity_used": to_float(gnss.get("pending_native_velocity_used")) if gnss else math.nan,
                "tpn_enabled": to_float(gnss.get("turn_rate_propagation_noise_probe_enabled")) if gnss else math.nan,
                "tpn_phase_allowed": to_float(gnss.get("turn_rate_propagation_noise_probe_phase_allowed")) if gnss else math.nan,
                "tpn_motion_ok": to_float(gnss.get("turn_rate_propagation_noise_probe_motion_ok")) if gnss else math.nan,
                "tpn_triggered": to_float(gnss.get("turn_rate_propagation_noise_probe_triggered")) if gnss else math.nan,
                "tpn_active": to_float(gnss.get("turn_rate_propagation_noise_probe_active")) if gnss else math.nan,
                "tpn_applied": to_float(gnss.get("turn_rate_propagation_noise_probe_applied")) if gnss else math.nan,
                "tpn_reason": gnss.get("turn_rate_propagation_noise_probe_reason", "unknown") if gnss else "unknown",
                "tpn_gyro_score": to_float(gnss.get("turn_rate_propagation_noise_probe_gyro_score")) if gnss else math.nan,
                "tpn_arw_q_scale": to_float(gnss.get("turn_rate_propagation_noise_probe_arw_q_scale")) if gnss else math.nan,
                "tpn_vrw_q_scale": to_float(gnss.get("turn_rate_propagation_noise_probe_vrw_q_scale")) if gnss else math.nan,
                "tpn_gyrbias_q_scale": to_float(gnss.get("turn_rate_propagation_noise_probe_gyrbias_q_scale")) if gnss else math.nan,
                "tpn_accbias_q_scale": to_float(gnss.get("turn_rate_propagation_noise_probe_accbias_q_scale")) if gnss else math.nan,
                "pgr_enabled": to_float(gnss.get("gnss_position_gain_response_enabled")) if gnss else math.nan,
                "pgr_phase_allowed": to_float(gnss.get("gnss_position_gain_response_phase_allowed")) if gnss else math.nan,
                "pgr_context_ok": to_float(gnss.get("gnss_position_gain_response_context_ok")) if gnss else math.nan,
                "pgr_motion_ok": to_float(gnss.get("gnss_position_gain_response_motion_ok")) if gnss else math.nan,
                "pgr_gain_ok": to_float(gnss.get("gnss_position_gain_response_gain_ok")) if gnss else math.nan,
                "pgr_triggered": to_float(gnss.get("gnss_position_gain_response_triggered")) if gnss else math.nan,
                "pgr_active": to_float(gnss.get("gnss_position_gain_response_active")) if gnss else math.nan,
                "pgr_applied": to_float(gnss.get("gnss_position_gain_response_applied")) if gnss else math.nan,
                "pgr_reason": gnss.get("gnss_position_gain_response_reason", "unknown") if gnss else "unknown",
                "pgr_score": to_float(gnss.get("gnss_position_gain_response_score")) if gnss else math.nan,
                "pgr_hnis_h": to_float(gnss.get("gnss_position_gain_response_hnis_h")) if gnss else math.nan,
                "pgr_prev_dx_over_residual_h": to_float(gnss.get("gnss_position_gain_response_prev_dx_over_residual_h")) if gnss else math.nan,
                "pgr_effective_std_h_m": to_float(gnss.get("gnss_position_gain_response_effective_std_h_m")) if gnss else math.nan,
                "pgr_multiplier_h": to_float(gnss.get("gnss_position_gain_response_multiplier_h")) if gnss else math.nan,
                "tvd_enabled": to_float(gnss.get("turn_postturn_native_velocity_deweight_enabled")) if gnss else math.nan,
                "tvd_phase_allowed": to_float(gnss.get("turn_postturn_native_velocity_deweight_phase_allowed")) if gnss else math.nan,
                "tvd_context_ok": to_float(gnss.get("turn_postturn_native_velocity_deweight_context_ok")) if gnss else math.nan,
                "tvd_motion_ok": to_float(gnss.get("turn_postturn_native_velocity_deweight_motion_ok")) if gnss else math.nan,
                "tvd_triggered": to_float(gnss.get("turn_postturn_native_velocity_deweight_triggered")) if gnss else math.nan,
                "tvd_active": to_float(gnss.get("turn_postturn_native_velocity_deweight_active")) if gnss else math.nan,
                "tvd_applied": to_float(gnss.get("turn_postturn_native_velocity_deweight_applied")) if gnss else math.nan,
                "tvd_reason": gnss.get("turn_postturn_native_velocity_deweight_reason", "unknown") if gnss else "unknown",
                "tvd_score": to_float(gnss.get("turn_postturn_native_velocity_deweight_score")) if gnss else math.nan,
                "tvd_radial_mps": to_float(gnss.get("turn_postturn_native_velocity_deweight_radial_mps")) if gnss else math.nan,
                "tvd_core_native_residual_h_mps": to_float(gnss.get("turn_postturn_native_velocity_deweight_core_native_residual_h_mps")) if gnss else math.nan,
                "tvd_effective_std_h_mps": to_float(gnss.get("turn_postturn_native_velocity_deweight_effective_std_h_mps")) if gnss else math.nan,
                "pos_cov_std_before_m": rms_std(state.get("cov_pos_n_before_m2"), state.get("cov_pos_e_before_m2")) if state else math.nan,
                "vel_cov_std_before_mps": rms_std(state.get("cov_vel_n_before_m2ps2"), state.get("cov_vel_e_before_m2ps2")) if state else math.nan,
                "yaw_cov_std_before_deg": math.degrees(math.sqrt(max(to_float(state.get("cov_yaw_before_rad2")), 0.0))) if state else math.nan,
                "bg_z_std_before_degps": math.degrees(math.sqrt(max(to_float(state.get("cov_bg_z_before_rad2ps2")), 0.0))) if state else math.nan,
                "ba_h_std_before_mps2": rms_std(state.get("cov_ba_x_before_m2ps4"), state.get("cov_ba_y_before_m2ps4")) if state else math.nan,
                "gyrbias_z_before_degps": math.degrees(to_float(state.get("gyrbias_z_before_radps"))) if state else math.nan,
                "accbias_h_before_mps2": math.hypot(to_float(state.get("accbias_x_before_mps2")), to_float(state.get("accbias_y_before_mps2"))) if state else math.nan,
                "state_dx_phi_yaw_deg": to_float(state.get("dx_phi_yaw_deg")) if state else math.nan,
                "state_dx_bg_z_degps": math.degrees(to_float(state.get("dx_bg_z_radps"))) if state else math.nan,
                "state_dx_ba_h_mps2": math.hypot(to_float(state.get("dx_ba_x_mps2")), to_float(state.get("dx_ba_y_mps2"))) if state else math.nan,
            }
        )
        enriched.append(out)

    enriched.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return enriched


def build_intervals(pos_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for current, nxt in zip(pos_rows, pos_rows[1:]):
        start = to_float(current.get("time_since_arm_sec"))
        end = to_float(nxt.get("time_since_arm_sec"))
        if not finite(start) or not finite(end) or end <= start:
            continue
        dt = to_float(nxt.get("ros_time_sec")) - to_float(current.get("ros_time_sec"))
        if not finite(dt) or dt <= 0.0:
            continue
        growth = to_float(nxt.get("error_before_h_m")) - to_float(current.get("error_after_h_m"))
        yaw_change = abs(angle_delta_deg(nxt.get("yaw_before_deg"), current.get("yaw_after_deg")))
        rows.append(
            {
                "start_time_since_arm_sec": start,
                "end_time_since_arm_sec": end,
                "window": window_label(start),
                "dt_sec": dt,
                "growth_m": growth,
                "growth_rate_mps": growth / dt,
                "positive_growth": 1 if growth > 0.0 else 0,
                "start_error_after_h_m": to_float(current.get("error_after_h_m")),
                "end_error_before_h_m": to_float(nxt.get("error_before_h_m")),
                "position_update_delta_h_m": to_float(current.get("error_delta_h_m")),
                "position_update_improvement_h_m": to_float(current.get("error_improvement_h_m")),
                "pair_iekf_error_h_m": to_float(current.get("pair_iekf_error_h_m")),
                "pair_ekf2_error_h_m": to_float(current.get("pair_ekf2_error_h_m")),
                "iekf_minus_ekf2_h_m": to_float(current.get("pair_iekf_error_h_m")) - to_float(current.get("pair_ekf2_error_h_m")),
                "context": current.get("context"),
                "speed_bin": current.get("speed_bin"),
                "gyro_bin": current.get("gyro_bin"),
                "source_yaw_rate_bin": current.get("source_yaw_rate_bin"),
                "vel_after_radial_bin": current.get("vel_after_radial_bin"),
                "horizontal_speed_mps": to_float(current.get("joined_horizontal_speed_mps"), to_float(current.get("horizontal_speed_mps"))),
                "gyro_abs_deg_s": abs(to_float(current.get("joined_gyro_deg_s"))),
                "source_yaw_rate_abs_deg_s": abs(to_float(current.get("joined_source_yaw_rate_deg_s"))),
                "yaw_change_abs_deg": yaw_change,
                "vel_after_error_h_mps": to_float(current.get("vel_after_error_h_mps")),
                "vel_after_radial_mps": to_float(current.get("vel_after_radial_mps")),
                "heading_residual_before_abs_deg": to_float(current.get("heading_residual_before_abs_deg")),
                "heading_yaw_correction_abs_deg": to_float(current.get("heading_yaw_correction_abs_deg")),
                "gnss_residual_h_m": to_float(current.get("gnss_residual_h_m")),
                "gnss_hnis_2d": to_float(current.get("gnss_hnis_2d")),
                "dx_over_residual_h": to_float(current.get("dx_over_residual_h")),
                "dx_pos_h_norm_m": to_float(current.get("dx_pos_h_norm_m")),
                "pos_cov_std_before_m": to_float(current.get("pos_cov_std_before_m")),
                "vel_cov_std_before_mps": to_float(current.get("vel_cov_std_before_mps")),
                "yaw_cov_std_before_deg": to_float(current.get("yaw_cov_std_before_deg")),
                "bg_z_std_before_degps": to_float(current.get("bg_z_std_before_degps")),
                "ba_h_std_before_mps2": to_float(current.get("ba_h_std_before_mps2")),
                "gyrbias_z_before_degps": to_float(current.get("gyrbias_z_before_degps")),
                "accbias_h_before_mps2": to_float(current.get("accbias_h_before_mps2")),
                "state_dx_phi_yaw_deg": to_float(current.get("state_dx_phi_yaw_deg")),
                "state_dx_bg_z_degps": to_float(current.get("state_dx_bg_z_degps")),
                "state_dx_ba_h_mps2": to_float(current.get("state_dx_ba_h_mps2")),
                "latest_imu_effective_dt_sec": to_float(current.get("gnss_latest_imu_effective_dt_sec")),
                "medium_gap_active": to_float(current.get("gnss_medium_gap_active")),
                "medium_gap_segmented": to_float(current.get("gnss_medium_gap_segmented")),
                "native_velocity_used": to_float(current.get("gnss_native_velocity_used")),
                "tpn_enabled": to_float(current.get("tpn_enabled")),
                "tpn_phase_allowed": to_float(current.get("tpn_phase_allowed")),
                "tpn_motion_ok": to_float(current.get("tpn_motion_ok")),
                "tpn_triggered": to_float(current.get("tpn_triggered")),
                "tpn_active": to_float(current.get("tpn_active")),
                "tpn_applied": to_float(current.get("tpn_applied")),
                "tpn_active_bin": tpn_active_bin(current.get("tpn_active")),
                "tpn_reason": current.get("tpn_reason", "unknown"),
                "tpn_gyro_score": to_float(current.get("tpn_gyro_score")),
                "tpn_arw_q_scale": to_float(current.get("tpn_arw_q_scale")),
                "tpn_vrw_q_scale": to_float(current.get("tpn_vrw_q_scale")),
                "tpn_gyrbias_q_scale": to_float(current.get("tpn_gyrbias_q_scale")),
                "tpn_accbias_q_scale": to_float(current.get("tpn_accbias_q_scale")),
                "pgr_enabled": to_float(current.get("pgr_enabled")),
                "pgr_phase_allowed": to_float(current.get("pgr_phase_allowed")),
                "pgr_context_ok": to_float(current.get("pgr_context_ok")),
                "pgr_motion_ok": to_float(current.get("pgr_motion_ok")),
                "pgr_gain_ok": to_float(current.get("pgr_gain_ok")),
                "pgr_triggered": to_float(current.get("pgr_triggered")),
                "pgr_active": to_float(current.get("pgr_active")),
                "pgr_applied": to_float(current.get("pgr_applied")),
                "pgr_active_bin": pgr_active_bin(current.get("pgr_active")),
                "pgr_reason": current.get("pgr_reason", "unknown"),
                "pgr_score": to_float(current.get("pgr_score")),
                "pgr_hnis_h": to_float(current.get("pgr_hnis_h")),
                "pgr_prev_dx_over_residual_h": to_float(current.get("pgr_prev_dx_over_residual_h")),
                "pgr_effective_std_h_m": to_float(current.get("pgr_effective_std_h_m")),
                "pgr_multiplier_h": to_float(current.get("pgr_multiplier_h")),
                "tvd_enabled": to_float(current.get("tvd_enabled")),
                "tvd_phase_allowed": to_float(current.get("tvd_phase_allowed")),
                "tvd_context_ok": to_float(current.get("tvd_context_ok")),
                "tvd_motion_ok": to_float(current.get("tvd_motion_ok")),
                "tvd_triggered": to_float(current.get("tvd_triggered")),
                "tvd_active": to_float(current.get("tvd_active")),
                "tvd_applied": to_float(current.get("tvd_applied")),
                "tvd_active_bin": tvd_active_bin(current.get("tvd_active")),
                "tvd_reason": current.get("tvd_reason", "unknown"),
                "tvd_score": to_float(current.get("tvd_score")),
                "tvd_radial_mps": to_float(current.get("tvd_radial_mps")),
                "tvd_core_native_residual_h_mps": to_float(current.get("tvd_core_native_residual_h_mps")),
                "tvd_effective_std_h_mps": to_float(current.get("tvd_effective_std_h_mps")),
            }
        )
    return rows


def subset_120_180(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    return [row for row in rows if 120.0 <= to_float(row.get("start_time_since_arm_sec")) < 180.0]


def summarize_group(rows: list[dict[str, object]], group_name: str, key_func: Callable[[dict[str, object]], str]) -> list[dict[str, object]]:
    buckets: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        buckets.setdefault(key_func(row), []).append(row)
    out: list[dict[str, object]] = []
    for key, group in sorted(buckets.items()):
        out.append(
            {
                "group_name": group_name,
                "group_value": key,
                "count": len(group),
                "growth_mean_m": mean(to_float(row.get("growth_m")) for row in group),
                "growth_median_m": percentile([to_float(row.get("growth_m")) for row in group], 50.0),
                "growth_p95_m": percentile([to_float(row.get("growth_m")) for row in group], 95.0),
                "positive_growth_frac": mean(to_float(row.get("positive_growth")) for row in group),
                "position_update_delta_mean_m": mean(to_float(row.get("position_update_delta_h_m")) for row in group),
                "position_update_improve_mean_m": mean(to_float(row.get("position_update_improvement_h_m")) for row in group),
                "start_error_mean_m": mean(to_float(row.get("start_error_after_h_m")) for row in group),
                "iekf_minus_ekf2_mean_m": mean(to_float(row.get("iekf_minus_ekf2_h_m")) for row in group),
                "vel_radial_mean_mps": mean(to_float(row.get("vel_after_radial_mps")) for row in group),
                "vel_error_mean_mps": mean(to_float(row.get("vel_after_error_h_mps")) for row in group),
                "speed_mean_mps": mean(to_float(row.get("horizontal_speed_mps")) for row in group),
                "gyro_abs_mean_deg_s": mean(to_float(row.get("gyro_abs_deg_s")) for row in group),
                "source_yaw_abs_mean_deg_s": mean(to_float(row.get("source_yaw_rate_abs_deg_s")) for row in group),
                "heading_resid_abs_mean_deg": mean(to_float(row.get("heading_residual_before_abs_deg")) for row in group),
                "hnis_mean": mean(to_float(row.get("gnss_hnis_2d")) for row in group),
                "pos_cov_std_mean_m": mean(to_float(row.get("pos_cov_std_before_m")) for row in group),
                "yaw_cov_std_mean_deg": mean(to_float(row.get("yaw_cov_std_before_deg")) for row in group),
                "bg_z_std_mean_degps": mean(to_float(row.get("bg_z_std_before_degps")) for row in group),
                "imu_effective_dt_p95_sec": percentile([to_float(row.get("latest_imu_effective_dt_sec")) for row in group], 95.0),
                "medium_gap_frac": mean(to_float(row.get("medium_gap_active")) for row in group),
                "tpn_active_frac": mean(to_float(row.get("tpn_active")) for row in group),
                "pgr_active_frac": mean(to_float(row.get("pgr_active")) for row in group),
                "tvd_active_frac": mean(to_float(row.get("tvd_active")) for row in group),
                "tpn_gyro_score_mean": mean(to_float(row.get("tpn_gyro_score")) for row in group),
                "tpn_arw_q_scale_mean": mean(to_float(row.get("tpn_arw_q_scale")) for row in group),
                "tpn_vrw_q_scale_mean": mean(to_float(row.get("tpn_vrw_q_scale")) for row in group),
                "tvd_score_mean": mean(to_float(row.get("tvd_score")) for row in group),
                "tvd_radial_mean_mps": mean(to_float(row.get("tvd_radial_mps")) for row in group),
                "tvd_core_native_residual_mean_mps": mean(to_float(row.get("tvd_core_native_residual_h_mps")) for row in group),
                "tvd_effective_std_mean_mps": mean(to_float(row.get("tvd_effective_std_h_mps")) for row in group),
            }
        )
    return out


def build_group_summaries(intervals: list[dict[str, object]]) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    rows.extend(summarize_group(intervals, "window", lambda row: str(row.get("window"))))
    late = subset_120_180(intervals)
    rows.extend(summarize_group(late, "late_context_120_180", lambda row: str(row.get("context"))))
    rows.extend(summarize_group(late, "late_speed_bin_120_180", lambda row: str(row.get("speed_bin"))))
    rows.extend(summarize_group(late, "late_gyro_bin_120_180", lambda row: str(row.get("gyro_bin"))))
    rows.extend(summarize_group(late, "late_source_yaw_bin_120_180", lambda row: str(row.get("source_yaw_rate_bin"))))
    rows.extend(summarize_group(late, "late_velocity_radial_bin_120_180", lambda row: str(row.get("vel_after_radial_bin"))))
    rows.extend(summarize_group(late, "late_hnis_bin_120_180", lambda row: hnis_bin(row.get("gnss_hnis_2d"))))
    rows.extend(summarize_group(late, "late_tpn_active_bin_120_180", lambda row: str(row.get("tpn_active_bin"))))
    rows.extend(summarize_group(late, "late_pgr_active_bin_120_180", lambda row: str(row.get("pgr_active_bin"))))
    rows.extend(summarize_group(late, "late_tvd_active_bin_120_180", lambda row: str(row.get("tvd_active_bin"))))
    return rows


def build_correlations(intervals: list[dict[str, object]]) -> list[dict[str, object]]:
    late = subset_120_180(intervals)
    features = [
        "dt_sec",
        "horizontal_speed_mps",
        "gyro_abs_deg_s",
        "source_yaw_rate_abs_deg_s",
        "yaw_change_abs_deg",
        "vel_after_error_h_mps",
        "vel_after_radial_mps",
        "heading_residual_before_abs_deg",
        "heading_yaw_correction_abs_deg",
        "gnss_residual_h_m",
        "gnss_hnis_2d",
        "dx_over_residual_h",
        "dx_pos_h_norm_m",
        "pos_cov_std_before_m",
        "vel_cov_std_before_mps",
        "yaw_cov_std_before_deg",
        "bg_z_std_before_degps",
        "ba_h_std_before_mps2",
        "gyrbias_z_before_degps",
        "accbias_h_before_mps2",
        "latest_imu_effective_dt_sec",
        "medium_gap_active",
        "native_velocity_used",
        "tpn_active",
        "tpn_gyro_score",
        "tpn_arw_q_scale",
        "tpn_vrw_q_scale",
        "tpn_gyrbias_q_scale",
        "tpn_accbias_q_scale",
        "pgr_active",
        "pgr_applied",
        "pgr_score",
        "pgr_hnis_h",
        "pgr_prev_dx_over_residual_h",
        "pgr_effective_std_h_m",
        "pgr_multiplier_h",
        "tvd_active",
        "tvd_applied",
        "tvd_score",
        "tvd_radial_mps",
        "tvd_core_native_residual_h_mps",
        "tvd_effective_std_h_mps",
    ]
    rows: list[dict[str, object]] = []
    ys = [to_float(row.get("growth_m")) for row in late]
    for feature in features:
        xs = [to_float(row.get(feature)) for row in late]
        corr, n = pearson(xs, ys)
        rows.append(
            {
                "feature": feature,
                "pearson_corr_with_growth_120_180": corr,
                "abs_corr": abs(corr) if finite(corr) else math.nan,
                "n": n,
                "feature_mean": mean(xs),
                "growth_mean_m": mean(ys),
            }
        )
    rows.sort(key=lambda row: to_float(row.get("abs_corr"), -1.0), reverse=True)
    return rows


def top_growth_rows(intervals: list[dict[str, object]], limit: int = 20) -> list[dict[str, object]]:
    late = subset_120_180(intervals)
    ordered = sorted(late, key=lambda row: to_float(row.get("growth_m"), -1.0), reverse=True)
    keep_cols = [
        "start_time_since_arm_sec",
        "end_time_since_arm_sec",
        "window",
        "growth_m",
        "start_error_after_h_m",
        "end_error_before_h_m",
        "position_update_delta_h_m",
        "context",
        "horizontal_speed_mps",
        "gyro_abs_deg_s",
        "source_yaw_rate_abs_deg_s",
        "vel_after_radial_mps",
        "vel_after_error_h_mps",
        "heading_residual_before_abs_deg",
        "gnss_residual_h_m",
        "gnss_hnis_2d",
        "dx_over_residual_h",
        "pos_cov_std_before_m",
        "yaw_cov_std_before_deg",
        "latest_imu_effective_dt_sec",
        "medium_gap_active",
        "tpn_active",
        "pgr_active",
        "pgr_score",
        "tpn_gyro_score",
        "tpn_arw_q_scale",
        "tpn_vrw_q_scale",
        "tvd_active",
        "tvd_score",
        "tvd_effective_std_h_mps",
    ]
    return [{col: row.get(col, math.nan) for col in keep_cols} for row in ordered[:limit]]


def build_report(
    run_dir: Path,
    out_dir: Path,
    intervals: list[dict[str, object]],
    summaries: list[dict[str, object]],
    correlations: list[dict[str, object]],
    top_rows: list[dict[str, object]],
) -> str:
    window_rows = [row for row in summaries if row["group_name"] == "window" and row["group_value"] in {"40-120", "120-140", "140-160", "160-180"}]
    context_rows = [row for row in summaries if row["group_name"] == "late_context_120_180"]
    radial_rows = [row for row in summaries if row["group_name"] == "late_velocity_radial_bin_120_180"]
    gyro_rows = [row for row in summaries if row["group_name"] == "late_gyro_bin_120_180"]
    tpn_rows = [row for row in summaries if row["group_name"] == "late_tpn_active_bin_120_180" and row["group_value"] != "tpn_unknown"]
    pgr_rows = [row for row in summaries if row["group_name"] == "late_pgr_active_bin_120_180" and row["group_value"] != "pgr_unknown"]
    tvd_rows = [row for row in summaries if row["group_name"] == "late_tvd_active_bin_120_180" and row["group_value"] != "tvd_unknown"]
    corr_rows = [row for row in correlations if finite(row.get("pearson_corr_with_growth_120_180"))][:8]

    lines = [
        "# Shortgen02 Propagation Context Diagnostic",
        "",
        f"Run directory: `{run_dir}`",
        "",
        "Scope: offline-only slice of existing `state_update_debug.csv`, position-update gain rows, pairlogger groundtruth, and ULog groundtruth velocity. No new flight, rebuild, or estimator behavior change is included.",
        "",
        "## Inter-Update Growth By Window",
        markdown_table(
            ["window", "n", "growth mean", "growth p95", "positive frac", "pos update delta", "vel radial", "gyro", "hNIS", "pos cov std"],
            [
                [
                    row["group_value"],
                    row["count"],
                    fmt(row["growth_mean_m"]),
                    fmt(row["growth_p95_m"]),
                    pct_fmt(row["positive_growth_frac"]),
                    fmt(row["position_update_delta_mean_m"]),
                    fmt(row["vel_radial_mean_mps"]),
                    fmt(row["gyro_abs_mean_deg_s"]),
                    fmt(row["hnis_mean"]),
                    fmt(row["pos_cov_std_mean_m"]),
                ]
                for row in window_rows
            ],
        ),
        "",
        "## 120-180s Context Split",
        markdown_table(
            ["context", "n", "growth mean", "growth p95", "positive frac", "start err", "vel radial", "heading resid", "hNIS", "imu dt p95"],
            [
                [
                    row["group_value"],
                    row["count"],
                    fmt(row["growth_mean_m"]),
                    fmt(row["growth_p95_m"]),
                    pct_fmt(row["positive_growth_frac"]),
                    fmt(row["start_error_mean_m"]),
                    fmt(row["vel_radial_mean_mps"]),
                    fmt(row["heading_resid_abs_mean_deg"]),
                    fmt(row["hnis_mean"]),
                    fmt(row["imu_effective_dt_p95_sec"], 5),
                ]
                for row in context_rows
            ],
        ),
        "",
        "## Velocity Radial Split",
        markdown_table(
            ["radial bin", "n", "growth mean", "growth p95", "positive frac", "vel radial", "vel error", "speed", "gyro"],
            [
                [
                    row["group_value"],
                    row["count"],
                    fmt(row["growth_mean_m"]),
                    fmt(row["growth_p95_m"]),
                    pct_fmt(row["positive_growth_frac"]),
                    fmt(row["vel_radial_mean_mps"]),
                    fmt(row["vel_error_mean_mps"]),
                    fmt(row["speed_mean_mps"]),
                    fmt(row["gyro_abs_mean_deg_s"]),
                ]
                for row in radial_rows
            ],
        ),
        "",
        "## Gyro Split",
        markdown_table(
            ["gyro bin", "n", "growth mean", "growth p95", "positive frac", "vel radial", "heading resid", "hNIS"],
            [
                [
                    row["group_value"],
                    row["count"],
                    fmt(row["growth_mean_m"]),
                    fmt(row["growth_p95_m"]),
                    pct_fmt(row["positive_growth_frac"]),
                    fmt(row["vel_radial_mean_mps"]),
                    fmt(row["heading_resid_abs_mean_deg"]),
                    fmt(row["hnis_mean"]),
                ]
                for row in gyro_rows
            ],
        ),
        "",
        "## TPN1a Split",
        markdown_table(
            ["TPN bin", "n", "growth mean", "growth p95", "positive frac", "TPN active", "gyro score", "ARW Q scale", "VRW Q scale"],
            [
                [
                    row["group_value"],
                    row["count"],
                    fmt(row["growth_mean_m"]),
                    fmt(row["growth_p95_m"]),
                    pct_fmt(row["positive_growth_frac"]),
                    pct_fmt(row["tpn_active_frac"]),
                    fmt(row["tpn_gyro_score_mean"]),
                    fmt(row["tpn_arw_q_scale_mean"]),
                    fmt(row["tpn_vrw_q_scale_mean"]),
                ]
                for row in tpn_rows
            ],
        ) if tpn_rows else "_No TPN1a debug columns found in this run._",
        "",
        "## PGR1a Split",
        markdown_table(
            ["PGR bin", "n", "growth mean", "growth p95", "positive frac", "PGR active", "pos update delta", "hNIS"],
            [
                [
                    row["group_value"],
                    row["count"],
                    fmt(row["growth_mean_m"]),
                    fmt(row["growth_p95_m"]),
                    pct_fmt(row["positive_growth_frac"]),
                    pct_fmt(row["pgr_active_frac"]),
                    fmt(row["position_update_delta_mean_m"]),
                    fmt(row["hnis_mean"]),
                ]
                for row in pgr_rows
            ],
        ) if pgr_rows else "_No PGR1a debug columns found in this run._",
        "",
        "## TVD1a Split",
        markdown_table(
            ["TVD bin", "n", "growth mean", "growth p95", "positive frac", "TVD active", "score", "native residual", "effective std"],
            [
                [
                    row["group_value"],
                    row["count"],
                    fmt(row["growth_mean_m"]),
                    fmt(row["growth_p95_m"]),
                    pct_fmt(row["positive_growth_frac"]),
                    pct_fmt(row["tvd_active_frac"]),
                    fmt(row["tvd_score_mean"]),
                    fmt(row["tvd_core_native_residual_mean_mps"]),
                    fmt(row["tvd_effective_std_mean_mps"]),
                ]
                for row in tvd_rows
            ],
        ) if tvd_rows else "_No TVD1a debug columns found in this run._",
        "",
        "## Strongest 120-180s Correlations",
        markdown_table(
            ["feature", "corr", "n", "feature mean"],
            [
                [
                    row["feature"],
                    fmt(row["pearson_corr_with_growth_120_180"]),
                    row["n"],
                    fmt(row["feature_mean"]),
                ]
                for row in corr_rows
            ],
        ),
        "",
        "## Largest Growth Intervals",
        markdown_table(
            ["t0", "t1", "growth", "ctx", "vel radial", "gyro", "heading resid", "hNIS", "TVD", "TVD std", "pos gain", "imu dt"],
            [
                [
                    fmt(row["start_time_since_arm_sec"], 1),
                    fmt(row["end_time_since_arm_sec"], 1),
                    fmt(row["growth_m"]),
                    row["context"],
                    fmt(row["vel_after_radial_mps"]),
                    fmt(row["gyro_abs_deg_s"]),
                    fmt(row["heading_residual_before_abs_deg"]),
                    fmt(row["gnss_hnis_2d"]),
                    fmt(row["tvd_active"]),
                    fmt(row["tvd_effective_std_h_mps"]),
                    fmt(row["dx_over_residual_h"]),
                    fmt(row["latest_imu_effective_dt_sec"], 5),
                ]
                for row in top_rows[:12]
            ],
        ),
        "",
        "## Readout",
        "- This report measures propagation-side growth as `next GNSS-position before-error - current GNSS-position after-error`, so positive values are error gained between accepted position updates.",
        "- Use this as a hypothesis narrowing artifact. It does not validate a mechanism by itself.",
        "- If the largest-growth rows cluster by velocity radial sign, test a position-response mechanism with a radial velocity guard. If they cluster by gyro/yaw context, test a yaw/motion-context mechanism. If they do not cluster, avoid another narrow heuristic and add a richer propagation residual logger first.",
        "",
        "Generated files:",
        f"- `{out_dir / 'inter_update_intervals.csv'}`",
        f"- `{out_dir / 'context_summary.csv'}`",
        f"- `{out_dir / 'feature_correlations.csv'}`",
        f"- `{out_dir / 'top_growth_intervals.csv'}`",
    ]
    return "\n".join(lines) + "\n"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, default=DEFAULT_RUN_DIR)
    parser.add_argument("--out-dir", type=Path, default=None)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    run_dir = args.run_dir
    out_dir = args.out_dir or (run_dir / "propagation_context_diag")
    out_dir.mkdir(parents=True, exist_ok=True)

    pos_rows = build_position_update_rows(run_dir)
    intervals = build_intervals(pos_rows)
    summaries = build_group_summaries(intervals)
    correlations = build_correlations(intervals)
    top_rows = top_growth_rows(intervals)

    write_csv(out_dir / "position_update_context_rows.csv", pos_rows)
    write_csv(out_dir / "inter_update_intervals.csv", intervals)
    write_csv(out_dir / "context_summary.csv", summaries)
    write_csv(out_dir / "feature_correlations.csv", correlations)
    write_csv(out_dir / "top_growth_intervals.csv", top_rows)
    report = build_report(run_dir, out_dir, intervals, summaries, correlations, top_rows)
    (out_dir / "report.md").write_text(report)
    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'inter_update_intervals.csv'}")
    print(f"wrote: {out_dir / 'context_summary.csv'}")
    print(f"wrote: {out_dir / 'feature_correlations.csv'}")
    print(f"wrote: {out_dir / 'top_growth_intervals.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
