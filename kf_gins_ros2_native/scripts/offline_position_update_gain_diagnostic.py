#!/usr/bin/env python3
"""Build GNSS position update gain/error rows from state-update debug logs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path

import pandas as pd

from offline_cross_route_failure_atlas import (
    ecef_to_enu,
    finite,
    load_fit,
    load_groundtruth_ref_and_velocity,
    llh_to_ecef,
    mean,
    nearest,
    percentile,
    resolve_joined_path,
    to_float,
    write_csv,
)


def transform_state(
    lat_deg: object,
    lon_deg: object,
    h_m: object,
    ref_lat: float,
    ref_lon: float,
    origin: tuple[float, float, float],
    fit: dict[str, float],
) -> tuple[float, float]:
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


def load_csv_rows(path: Path) -> list[dict[str, object]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def build_rows(run_dir: Path) -> list[dict[str, object]]:
    state_path = run_dir / "state_update_debug.csv"
    joined_path = resolve_joined_path(run_dir)
    fit = load_fit(run_dir)
    gt_pack = load_groundtruth_ref_and_velocity(run_dir)
    if not state_path.exists():
        raise FileNotFoundError(f"missing state update debug CSV: {state_path}")
    if joined_path is None:
        raise FileNotFoundError("missing groundtruth_joined.csv")
    if fit is None:
        raise FileNotFoundError("missing GPS-groundtruth fit CSV")
    if gt_pack is None:
        raise FileNotFoundError("missing ULog groundtruth CSV")

    ref_lat, ref_lon, origin, _, _ = gt_pack
    joined_rows = pd.read_csv(joined_path).sort_values("pair_ros_time_sec").to_dict("records")
    joined_times = [to_float(row.get("pair_ros_time_sec")) for row in joined_rows]

    gnss_path = run_dir / "gnss_nis_debug.csv"
    gnss_rows = load_csv_rows(gnss_path) if gnss_path.exists() else []
    gnss_by_seq = {
        int(to_float(row.get("sequence"))): row
        for row in gnss_rows
        if finite(row.get("sequence"))
    }
    gnss_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    gnss_times = [to_float(row.get("ros_time_sec")) for row in gnss_rows]

    state_df = pd.read_csv(state_path)
    pos_src = state_df[
        (state_df["event_type"] == "gnss_position") &
        (state_df["applied"].astype(int) == 1)
    ]

    raw_rows: list[dict[str, object]] = []
    for row in pos_src.to_dict("records"):
        ros_t = to_float(row.get("ros_time_sec"))
        joined, pair_dt = nearest(joined_rows, joined_times, ros_t, max_dt=0.11)
        if joined is None:
            continue
        before_x0, before_y0 = transform_state(
            row.get("lat_before_deg"), row.get("lon_before_deg"), row.get("h_before_m"),
            ref_lat, ref_lon, origin, fit)
        after_x0, after_y0 = transform_state(
            row.get("lat_after_deg"), row.get("lon_after_deg"), row.get("h_after_m"),
            ref_lat, ref_lon, origin, fit)
        raw_rows.append(
            {
                "sequence": int(to_float(row.get("sequence"))),
                "ros_time_sec": ros_t,
                "update_time_sec": to_float(row.get("update_time_sec")),
                "time_since_arm_sec": to_float(joined.get("time_since_arm_sec")),
                "pair_join_dt_sec": pair_dt,
                "gt_x_m": to_float(joined.get("gt_x_m")),
                "gt_y_m": to_float(joined.get("gt_y_m")),
                "before_x0_m": before_x0,
                "before_y0_m": before_y0,
                "after_x0_m": after_x0,
                "after_y0_m": after_y0,
                "state": row,
                "joined": joined,
            }
        )

    align = [
        row for row in raw_rows
        if 0.0 <= to_float(row.get("time_since_arm_sec")) <= 20.0
    ]
    off_x = mean(to_float(row.get("gt_x_m")) - to_float(row.get("after_x0_m")) for row in align)
    off_y = mean(to_float(row.get("gt_y_m")) - to_float(row.get("after_y0_m")) for row in align)
    if not finite(off_x):
        off_x = 0.0
    if not finite(off_y):
        off_y = 0.0

    rows: list[dict[str, object]] = []
    for item in raw_rows:
        state = item["state"]  # type: ignore[assignment]
        joined = item["joined"]  # type: ignore[assignment]
        seq = int(to_float(item.get("sequence")))
        ros_t = to_float(item.get("ros_time_sec"))
        gnss = gnss_by_seq.get(seq)
        if gnss is None:
            gnss, _ = nearest(gnss_rows, gnss_times, ros_t, max_dt=0.11)

        before_x = to_float(item.get("before_x0_m")) + off_x
        before_y = to_float(item.get("before_y0_m")) + off_y
        after_x = to_float(item.get("after_x0_m")) + off_x
        after_y = to_float(item.get("after_y0_m")) + off_y
        gt_x = to_float(item.get("gt_x_m"))
        gt_y = to_float(item.get("gt_y_m"))
        error_before_x = before_x - gt_x
        error_before_y = before_y - gt_y
        error_after_x = after_x - gt_x
        error_after_y = after_y - gt_y
        error_before_h = math.hypot(error_before_x, error_before_y)
        error_after_h = math.hypot(error_after_x, error_after_y)
        dx_pos_h = to_float(state.get("dx_pos_h_norm_m"))
        gnss_resid_h = to_float(gnss.get("gnss_position_residual_h_m")) if gnss else math.nan
        dx_over_resid = dx_pos_h / gnss_resid_h if finite(dx_pos_h) and finite(gnss_resid_h) and gnss_resid_h > 1.0e-9 else math.nan

        out = {
            "sequence": seq,
            "ros_time_sec": ros_t,
            "update_time_sec": to_float(item.get("update_time_sec")),
            "time_since_arm_sec": to_float(item.get("time_since_arm_sec")),
            "pair_join_dt_sec": to_float(item.get("pair_join_dt_sec")),
            "mavros_mode": state.get("mavros_mode", ""),
            "mavros_armed": to_float(state.get("mavros_armed")),
            "turning_now": to_float(state.get("turning_now")),
            "post_turn_context": to_float(state.get("post_turn_context")),
            "armed_cruise_context": to_float(state.get("armed_cruise_context")),
            "terminal_descent_context": to_float(state.get("terminal_descent_context")),
            "horizontal_speed_mps": to_float(state.get("horizontal_speed_mps")),
            "vertical_speed_mps": to_float(state.get("vertical_speed_mps")),
            "gyro_deg_s": to_float(state.get("gyro_deg_s")),
            "source_yaw_rate_deg_s": to_float(state.get("source_yaw_rate_deg_s")),
            "yaw_before_deg": to_float(state.get("yaw_before_deg")),
            "yaw_after_deg": to_float(state.get("yaw_after_deg")),
            "gt_x_m": gt_x,
            "gt_y_m": gt_y,
            "before_x_m": before_x,
            "before_y_m": before_y,
            "after_x_m": after_x,
            "after_y_m": after_y,
            "error_before_x_m": error_before_x,
            "error_before_y_m": error_before_y,
            "error_before_h_m": error_before_h,
            "error_after_x_m": error_after_x,
            "error_after_y_m": error_after_y,
            "error_after_h_m": error_after_h,
            "error_delta_h_m": error_after_h - error_before_h,
            "error_improvement_h_m": error_before_h - error_after_h,
            "pair_iekf_error_h_m": to_float(joined.get("iekf_error_xy_m")),
            "pair_ekf2_error_h_m": to_float(joined.get("ekf2_error_xy_m")),
            "dx_pos_h_norm_m": dx_pos_h,
            "dx_pos_n_m": to_float(state.get("dx_pos_n_m")),
            "dx_pos_e_m": to_float(state.get("dx_pos_e_m")),
            "dx_over_residual_h": dx_over_resid,
            "gnss_residual_h_m": gnss_resid_h,
            "gnss_hnis_2d": to_float(gnss.get("gnss_position_nis_h_2d")) if gnss else math.nan,
            "gnss_position_std_h_m": mean([
                to_float(gnss.get("gnss_position_std_n_m")) if gnss else math.nan,
                to_float(gnss.get("gnss_position_std_e_m")) if gnss else math.nan,
            ]),
            "gnss_position_std_source_label": gnss.get("gnss_position_std_source_label", "") if gnss else "",
        }
        if gnss:
            for key in (
                "gnss_position_gain_response_enabled",
                "gnss_position_gain_response_phase_allowed",
                "gnss_position_gain_response_context_ok",
                "gnss_position_gain_response_motion_ok",
                "gnss_position_gain_response_gain_ok",
                "gnss_position_gain_response_triggered",
                "gnss_position_gain_response_active",
                "gnss_position_gain_response_applied",
                "gnss_position_gain_response_reason",
                "gnss_position_gain_response_residual_h_m",
                "gnss_position_gain_response_hnis_h",
                "gnss_position_gain_response_prev_dx_over_residual_h",
                "gnss_position_gain_response_score",
                "gnss_position_gain_response_effective_std_h_m",
                "gnss_position_gain_response_multiplier_h",
                "phase_error_memory_debug_enabled",
                "phase_error_memory_state_update_matched",
                "phase_error_memory_recent_turnpost_age_sec",
                "phase_error_memory_recent_turnpost_active",
                "phase_error_memory_residual_h_m",
                "phase_error_memory_dx_pos_h_m",
                "phase_error_memory_dx_over_residual_h",
                "phase_error_memory_pos_std_h_before_m",
                "phase_error_memory_pressure_active",
                "phase_error_memory_candidate_active",
                "phase_error_memory_reason",
                "phase_error_memory_residual_threshold_h_m",
                "phase_error_memory_dx_over_residual_threshold",
                "phase_error_memory_recent_turnpost_hold_sec",
            ):
                out[key] = gnss.get(key, math.nan)
        rows.append(out)

    rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return rows


def write_report(out_dir: Path, rows: list[dict[str, object]]) -> None:
    intervals = []
    for current, nxt in zip(rows, rows[1:]):
        start = to_float(current.get("time_since_arm_sec"))
        end = to_float(nxt.get("time_since_arm_sec"))
        if not finite(start) or not finite(end) or end <= start:
            continue
        intervals.append(
            {
                "start_time_since_arm_sec": start,
                "end_time_since_arm_sec": end,
                "growth_m": to_float(nxt.get("error_before_h_m")) - to_float(current.get("error_after_h_m")),
                "pgr_active": to_float(current.get("gnss_position_gain_response_active")),
                "pgr_applied": to_float(current.get("gnss_position_gain_response_applied")),
            }
        )
    late = [row for row in intervals if 120.0 <= to_float(row.get("start_time_since_arm_sec")) < 180.0]
    active = [row for row in late if to_float(row.get("pgr_active")) > 0.5]
    inactive = [row for row in late if not (to_float(row.get("pgr_active")) > 0.5)]
    lines = [
        "# Position Update Gain Diagnostic",
        "",
        f"position update rows: {len(rows)}",
        f"late 120-180 intervals: {len(late)}",
        f"PGR active late intervals: {len(active)}",
        f"PGR inactive late intervals: {len(inactive)}",
        "",
        "| subset | n | growth_mean_m | growth_p95_m | positive_frac |",
        "| --- | ---: | ---: | ---: | ---: |",
    ]
    for name, subset in (("late_all", late), ("late_pgr_active", active), ("late_pgr_inactive", inactive)):
        vals = [to_float(row.get("growth_m")) for row in subset]
        pos = [1.0 if to_float(row.get("growth_m")) > 0.0 else 0.0 for row in subset]
        lines.append(
            f"| {name} | {len(subset)} | {mean(vals):.6g} | {percentile(vals, 95.0):.6g} | {mean(pos):.6g} |"
        )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", type=Path, required=True)
    parser.add_argument("--out-dir", type=Path, default=None)
    args = parser.parse_args()

    out_dir = args.out_dir if args.out_dir is not None else args.run_dir / "position_update_gain_diag"
    out_dir.mkdir(parents=True, exist_ok=True)
    rows = build_rows(args.run_dir)
    write_csv(out_dir / "gnss_position_update_gain_rows.csv", rows)
    write_report(out_dir, rows)
    print(f"wrote: {out_dir / 'gnss_position_update_gain_rows.csv'}")
    print(f"wrote: {out_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
