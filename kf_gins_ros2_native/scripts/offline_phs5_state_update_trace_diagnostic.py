#!/usr/bin/env python3
"""Summarize enriched PHS5 state-update traces against ULog groundtruth."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Iterable

import pandas as pd


WINDOWS = [
    ("0-20", 0.0, 20.0),
    ("20-40", 20.0, 40.0),
    ("40-60", 40.0, 60.0),
    ("60-80", 60.0, 80.0),
    ("80-100", 80.0, 100.0),
    ("100-120", 100.0, 120.0),
    ("120-140", 120.0, 140.0),
    ("140-160", 140.0, 160.0),
    ("160-180", 160.0, 180.0),
    ("120-180", 120.0, 180.0),
    ("40-180", 40.0, 180.0),
]


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


def read_csv(path: Path) -> list[dict[str, object]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def resolve_joined_path(run_dir: Path) -> Path:
    for rel in (
        "offline_groundtruth_convergence_diag/groundtruth_joined.csv",
        "offline_groundtruth_convergence_diag_autogt/groundtruth_joined.csv",
        "groundtruth_joined.csv",
    ):
        path = run_dir / rel
        if path.exists():
            return path
    raise FileNotFoundError(f"missing groundtruth_joined.csv under {run_dir}")


def load_fit(run_dir: Path) -> tuple[float, float]:
    path = run_dir / "offline_ulog_gps_groundtruth_diag" / "gps_groundtruth_fit.csv"
    if not path.exists():
        return 1.0, 0.0
    for row in read_csv(path):
        if row.get("topic") == "vehicle_gps_position" and row.get("subset") == "all":
            scale = to_float(row.get("scale"), 1.0)
            angle = math.radians(to_float(row.get("angle_deg"), 0.0))
            return scale * math.cos(angle), scale * math.sin(angle)
    return 1.0, 0.0


def nearest(rows: list[dict[str, object]], times: list[float], t: float, max_dt: float) -> tuple[dict[str, object] | None, float]:
    if not finite(t):
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


def norm2(x: float, y: float) -> float:
    return math.hypot(x, y)


def dot_unit(ax: float, ay: float, bx: float, by: float) -> float:
    an = norm2(ax, ay)
    bn = norm2(bx, by)
    if an < 1.0e-9 or bn < 1.0e-9:
        return math.nan
    return (ax * bx + ay * by) / (an * bn)


def transform_fit(e: float, n: float, a_real: float, a_imag: float) -> tuple[float, float]:
    return a_real * e - a_imag * n, a_imag * e + a_real * n


def build_rows(run_dir: Path, max_join_dt_sec: float) -> tuple[list[dict[str, object]], list[dict[str, object]], list[dict[str, object]]]:
    state_path = run_dir / "state_update_debug.csv"
    if not state_path.exists():
        raise FileNotFoundError(f"missing state update debug CSV: {state_path}")
    state_df = pd.read_csv(state_path)
    required = {
        "core_enu_e_before_m",
        "core_enu_n_before_m",
        "px4_sphere_enu_e_before_m",
        "px4_sphere_enu_n_before_m",
        "gnss_position_residual_e_m",
        "gnss_position_residual_n_m",
        "kalman_gain_horizontal_trace",
    }
    missing = sorted(required - set(state_df.columns))
    if missing:
        raise RuntimeError("state_update_debug.csv lacks enriched columns: " + ", ".join(missing))

    joined_rows = read_csv(resolve_joined_path(run_dir))
    joined_rows.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
    joined_times = [to_float(row.get("pair_ros_time_sec")) for row in joined_rows]
    a_real, a_imag = load_fit(run_dir)

    pos_df = state_df[
        (state_df["event_type"] == "gnss_position")
        & (state_df["applied"].astype(int) == 1)
        & (state_df["state_update_have_enu"].astype(int) == 1)
    ]

    raw_rows: list[dict[str, object]] = []
    for row in pos_df.to_dict("records"):
        ros_t = to_float(row.get("ros_time_sec"))
        joined, join_dt = nearest(joined_rows, joined_times, ros_t, max_join_dt_sec)
        if joined is None:
            continue
        gt_x = to_float(joined.get("gt_x_m"))
        gt_y = to_float(joined.get("gt_y_m"))
        raw_before = (
            to_float(row.get("core_enu_e_before_m")),
            to_float(row.get("core_enu_n_before_m")),
        )
        raw_after = (
            to_float(row.get("core_enu_e_after_m")),
            to_float(row.get("core_enu_n_after_m")),
        )
        sphere_before = (
            to_float(row.get("px4_sphere_enu_e_before_m")),
            to_float(row.get("px4_sphere_enu_n_before_m")),
        )
        sphere_after = (
            to_float(row.get("px4_sphere_enu_e_after_m")),
            to_float(row.get("px4_sphere_enu_n_after_m")),
        )
        gpsfit_before = transform_fit(*raw_before, a_real, a_imag)
        gpsfit_after = transform_fit(*raw_after, a_real, a_imag)

        # KF-GINS logs position residual as state-minus-measurement in N/E/U.
        residual_e = to_float(row.get("gnss_position_residual_e_m"))
        residual_n = to_float(row.get("gnss_position_residual_n_m"))
        raw_meas = (raw_before[0] - residual_e, raw_before[1] - residual_n)
        gpsfit_meas = transform_fit(*raw_meas, a_real, a_imag)

        base = {
            "sequence": int(to_float(row.get("sequence"), -1)),
            "ros_time_sec": ros_t,
            "update_time_sec": to_float(row.get("update_time_sec")),
            "join_dt_sec": join_dt,
            "time_since_arm_sec": to_float(joined.get("time_since_arm_sec")),
            "mavros_mode": row.get("mavros_mode", ""),
            "turning_now": int(to_float(row.get("turning_now"), 0.0)),
            "post_turn_context": int(to_float(row.get("post_turn_context"), 0.0)),
            "armed_cruise_context": int(to_float(row.get("armed_cruise_context"), 0.0)),
            "horizontal_speed_mps": to_float(row.get("horizontal_speed_mps")),
            "gyro_deg_s": to_float(row.get("gyro_deg_s")),
            "gt_x_m": gt_x,
            "gt_y_m": gt_y,
            "ekf2_error_xy_m": to_float(joined.get("ekf2_error_xy_m")),
            "iekf_output_error_xy_m": to_float(joined.get("iekf_error_xy_m")),
            "residual_h_m": to_float(row.get("gnss_position_residual_h_m")),
            "dx_pos_h_norm_m": to_float(row.get("dx_pos_h_norm_m")),
            "dx_pos_h_over_residual_h": to_float(row.get("dx_pos_h_over_residual_h")),
            "nis_h_2d": to_float(row.get("gnss_position_nis_h_2d")),
            "nis_3d": to_float(row.get("gnss_position_nis_3d")),
            "std_n_m": to_float(row.get("gnss_position_std_n_m")),
            "std_e_m": to_float(row.get("gnss_position_std_e_m")),
            "s_nn_m2": to_float(row.get("gnss_position_s_nn_m2")),
            "s_ee_m2": to_float(row.get("gnss_position_s_ee_m2")),
            "kalman_gain_horizontal_trace": to_float(row.get("kalman_gain_horizontal_trace")),
            "kalman_gain_horizontal_fro": to_float(row.get("kalman_gain_horizontal_fro")),
            "kalman_gain_pn_from_meas_n": to_float(row.get("kalman_gain_pn_from_meas_n")),
            "kalman_gain_pe_from_meas_e": to_float(row.get("kalman_gain_pe_from_meas_e")),
        }
        frames = {
            "raw_wgs84_enu": (raw_before, raw_after, raw_meas),
            "gps_fit_all": (gpsfit_before, gpsfit_after, gpsfit_meas),
            "px4_sphere_anisotropic": (sphere_before, sphere_after, (math.nan, math.nan)),
        }
        for frame, (before, after, meas) in frames.items():
            raw_rows.append({
                **base,
                "frame": frame,
                "before_x0_m": before[0],
                "before_y0_m": before[1],
                "after_x0_m": after[0],
                "after_y0_m": after[1],
                "measurement_x0_m": meas[0],
                "measurement_y0_m": meas[1],
            })

    offsets: dict[str, tuple[float, float]] = {}
    for frame in sorted({str(row["frame"]) for row in raw_rows}):
        align = [
            row for row in raw_rows
            if row.get("frame") == frame and 0.0 <= to_float(row.get("time_since_arm_sec")) <= 20.0
        ]
        offsets[frame] = (
            mean(to_float(row.get("gt_x_m")) - to_float(row.get("after_x0_m")) for row in align),
            mean(to_float(row.get("gt_y_m")) - to_float(row.get("after_y0_m")) for row in align),
        )

    enriched: list[dict[str, object]] = []
    for row in raw_rows:
        frame = str(row["frame"])
        ox, oy = offsets.get(frame, (0.0, 0.0))
        before_x = to_float(row.get("before_x0_m")) + ox
        before_y = to_float(row.get("before_y0_m")) + oy
        after_x = to_float(row.get("after_x0_m")) + ox
        after_y = to_float(row.get("after_y0_m")) + oy
        meas_x = to_float(row.get("measurement_x0_m")) + ox
        meas_y = to_float(row.get("measurement_y0_m")) + oy
        gt_x = to_float(row.get("gt_x_m"))
        gt_y = to_float(row.get("gt_y_m"))
        need_x = gt_x - before_x
        need_y = gt_y - before_y
        update_x = after_x - before_x
        update_y = after_y - before_y
        meas_delta_x = meas_x - before_x
        meas_delta_y = meas_y - before_y
        before_err = norm2(before_x - gt_x, before_y - gt_y)
        after_err = norm2(after_x - gt_x, after_y - gt_y)
        meas_err = norm2(meas_x - gt_x, meas_y - gt_y)
        enriched.append({
            **row,
            "align_offset_x_m": ox,
            "align_offset_y_m": oy,
            "before_x_m": before_x,
            "before_y_m": before_y,
            "after_x_m": after_x,
            "after_y_m": after_y,
            "measurement_x_m": meas_x,
            "measurement_y_m": meas_y,
            "before_error_xy_m": before_err,
            "after_error_xy_m": after_err,
            "measurement_error_xy_m": meas_err,
            "update_improvement_m": before_err - after_err,
            "need_h_m": norm2(need_x, need_y),
            "update_h_m": norm2(update_x, update_y),
            "measurement_delta_h_m": norm2(meas_delta_x, meas_delta_y),
            "update_over_need": norm2(update_x, update_y) / norm2(need_x, need_y) if norm2(need_x, need_y) > 1.0e-9 else math.nan,
            "measurement_delta_over_need": norm2(meas_delta_x, meas_delta_y) / norm2(need_x, need_y) if norm2(need_x, need_y) > 1.0e-9 else math.nan,
            "update_need_cos": dot_unit(update_x, update_y, need_x, need_y),
            "measurement_need_cos": dot_unit(meas_delta_x, meas_delta_y, need_x, need_y),
        })

    projection_summary: list[dict[str, object]] = []
    gain_summary: list[dict[str, object]] = []
    for window, start, end in WINDOWS:
        for frame in sorted({str(row["frame"]) for row in enriched}):
            rows = [
                row for row in enriched
                if row.get("frame") == frame and start <= to_float(row.get("time_since_arm_sec")) < end
            ]
            projection_summary.append({
                "window": window,
                "frame": frame,
                "rows": len(rows),
                "before_mean_m": mean(to_float(row.get("before_error_xy_m")) for row in rows),
                "after_mean_m": mean(to_float(row.get("after_error_xy_m")) for row in rows),
                "after_p95_m": percentile((to_float(row.get("after_error_xy_m")) for row in rows), 95.0),
                "measurement_mean_m": mean(to_float(row.get("measurement_error_xy_m")) for row in rows),
                "update_improvement_mean_m": mean(to_float(row.get("update_improvement_m")) for row in rows),
                "update_over_need_mean": mean(to_float(row.get("update_over_need")) for row in rows),
                "measurement_delta_over_need_mean": mean(to_float(row.get("measurement_delta_over_need")) for row in rows),
                "update_need_cos_mean": mean(to_float(row.get("update_need_cos")) for row in rows),
                "measurement_need_cos_mean": mean(to_float(row.get("measurement_need_cos")) for row in rows),
            })
        base_rows = [
            row for row in enriched
            if row.get("frame") == "raw_wgs84_enu" and start <= to_float(row.get("time_since_arm_sec")) < end
        ]
        gain_summary.append({
            "window": window,
            "rows": len(base_rows),
            "residual_h_mean_m": mean(to_float(row.get("residual_h_m")) for row in base_rows),
            "residual_h_p95_m": percentile((to_float(row.get("residual_h_m")) for row in base_rows), 95.0),
            "dx_pos_h_mean_m": mean(to_float(row.get("dx_pos_h_norm_m")) for row in base_rows),
            "dx_pos_h_over_residual_h_mean": mean(to_float(row.get("dx_pos_h_over_residual_h")) for row in base_rows),
            "nis_h_2d_mean": mean(to_float(row.get("nis_h_2d")) for row in base_rows),
            "nis_3d_mean": mean(to_float(row.get("nis_3d")) for row in base_rows),
            "std_h_mean_m": mean(
                math.hypot(to_float(row.get("std_n_m")), to_float(row.get("std_e_m")))
                for row in base_rows
            ),
            "s_h_trace_mean_m2": mean(
                to_float(row.get("s_nn_m2")) + to_float(row.get("s_ee_m2"))
                for row in base_rows
            ),
            "kalman_gain_trace_mean": mean(to_float(row.get("kalman_gain_horizontal_trace")) for row in base_rows),
            "kalman_gain_fro_mean": mean(to_float(row.get("kalman_gain_horizontal_fro")) for row in base_rows),
            "ekf2_error_mean_m": mean(to_float(row.get("ekf2_error_xy_m")) for row in base_rows),
            "iekf_output_error_mean_m": mean(to_float(row.get("iekf_output_error_xy_m")) for row in base_rows),
        })
    return enriched, projection_summary, gain_summary


def build_report(projection_summary: list[dict[str, object]], gain_summary: list[dict[str, object]], out_dir: Path) -> str:
    selected_windows = ["100-120", "120-140", "140-160", "160-180", "120-180"]
    window_rows: list[list[object]] = []
    selector_rows: list[list[object]] = []
    for window in selected_windows:
        rows = [row for row in projection_summary if row.get("window") == window]
        best = min(rows, key=lambda row: to_float(row.get("after_mean_m"), math.inf)) if rows else None
        if best:
            selector_rows.append([
                window,
                best.get("frame", ""),
                fmt(best.get("after_mean_m")),
                fmt(best.get("after_p95_m")),
                fmt(best.get("update_improvement_mean_m")),
                fmt(best.get("measurement_mean_m")),
            ])
        for frame in ("raw_wgs84_enu", "gps_fit_all", "px4_sphere_anisotropic"):
            row = next((item for item in rows if item.get("frame") == frame), None)
            if row is None:
                continue
            window_rows.append([
                window,
                frame,
                int(to_float(row.get("rows"), 0.0)),
                fmt(row.get("before_mean_m")),
                fmt(row.get("after_mean_m")),
                fmt(row.get("after_p95_m")),
                fmt(row.get("measurement_mean_m")),
                fmt(row.get("update_over_need_mean")),
                fmt(row.get("update_need_cos_mean")),
            ])

    gain_rows: list[list[object]] = []
    for window in selected_windows:
        row = next((item for item in gain_summary if item.get("window") == window), None)
        if row is None:
            continue
        gain_rows.append([
            window,
            int(to_float(row.get("rows"), 0.0)),
            fmt(row.get("residual_h_mean_m")),
            fmt(row.get("dx_pos_h_mean_m")),
            fmt(row.get("dx_pos_h_over_residual_h_mean")),
            fmt(row.get("kalman_gain_trace_mean")),
            fmt(row.get("kalman_gain_fro_mean")),
            fmt(row.get("nis_h_2d_mean")),
            fmt(row.get("ekf2_error_mean_m")),
            fmt(row.get("iekf_output_error_mean_m")),
        ])

    lines = [
        "# PHS5 Enriched State Update Trace Diagnostic",
        "",
        "This is an offline-only diagnostic over `state_update_debug.csv` joined to ULog groundtruth. It does not change estimator behavior.",
        "",
        "## Segment Projection Selector",
        "",
        markdown_table(
            ["window", "best_frame", "after_mean", "after_p95", "update_improvement", "measurement_mean"],
            selector_rows,
        ),
        "",
        "## Projection And Measurement Geometry",
        "",
        markdown_table(
            [
                "window",
                "frame",
                "rows",
                "before_mean",
                "after_mean",
                "after_p95",
                "measurement_mean",
                "update/need",
                "update_need_cos",
            ],
            window_rows,
        ),
        "",
        "## Gain And Residual Summary",
        "",
        markdown_table(
            [
                "window",
                "rows",
                "resid_h",
                "dx_h",
                "dx/resid",
                "K_trace",
                "K_fro",
                "hNIS",
                "ekf2_out",
                "iekf_out",
            ],
            gain_rows,
        ),
        "",
        "## Readout",
        "",
        "- `after_mean` is the aligned post-position-update state error in the named output frame.",
        "- `measurement_mean` is inferred from the logged GNSS position residual and is only available for raw/GPS-fit frames.",
        "- Use the selector table as a diagnostic for segment-dependent frame choice; it is not yet an online mechanism.",
        "",
        "Generated files:",
        f"- `{out_dir / 'state_update_trace_rows.csv'}`",
        f"- `{out_dir / 'window_projection_summary.csv'}`",
        f"- `{out_dir / 'window_gain_summary.csv'}`",
    ]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True, type=Path)
    parser.add_argument("--out-dir", type=Path)
    parser.add_argument("--max-join-dt-sec", type=float, default=0.15)
    args = parser.parse_args()

    out_dir = args.out_dir or (args.run_dir / "offline_phs5_state_update_trace_diag")
    rows, projection_summary, gain_summary = build_rows(args.run_dir, args.max_join_dt_sec)
    write_csv(out_dir / "state_update_trace_rows.csv", rows)
    write_csv(out_dir / "window_projection_summary.csv", projection_summary)
    write_csv(out_dir / "window_gain_summary.csv", gain_summary)
    (out_dir / "report.md").write_text(build_report(projection_summary, gain_summary, out_dir), encoding="utf-8")
    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'state_update_trace_rows.csv'}")
    print(f"wrote: {out_dir / 'window_projection_summary.csv'}")
    print(f"wrote: {out_dir / 'window_gain_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
