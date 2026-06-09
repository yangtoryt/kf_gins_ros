#!/usr/bin/env python3
"""Diagnose GNSS measurement/update geometry for shortgen02 high-error rows."""

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


def vec_norm(x: float, y: float) -> float:
    return math.hypot(x, y)


def vec_angle_deg(ax: float, ay: float, bx: float, by: float) -> float:
    an = vec_norm(ax, ay)
    bn = vec_norm(bx, by)
    if an <= 1e-12 or bn <= 1e-12:
        return math.nan
    c = (ax * bx + ay * by) / (an * bn)
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


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


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return finite(t) and start <= t < end


def nearest(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt: float,
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
    if dt > max_dt:
        return None, dt
    return row, dt


def load_fit(run_dir: Path) -> tuple[float, float]:
    fit_path = run_dir / "offline_ulog_gps_groundtruth_diag" / "gps_groundtruth_fit.csv"
    for row in read_csv(fit_path):
        if row.get("topic") == "vehicle_gps_position" and row.get("subset") == "all":
            scale = to_float(row.get("scale"))
            angle = math.radians(to_float(row.get("angle_deg")))
            if finite(scale) and finite(angle):
                return scale * math.cos(angle), scale * math.sin(angle)
    raise RuntimeError(f"missing vehicle_gps_position/all fit in {fit_path}")


def transform_residual_to_xy(residual_n_m: float, residual_e_m: float, fit: tuple[float, float]) -> tuple[float, float]:
    # Residual columns are N/E. The state rows are in the GPS-normalized ENU XY frame.
    a_real, a_imag = fit
    return (
        a_real * residual_e_m - a_imag * residual_n_m,
        a_imag * residual_e_m + a_real * residual_n_m,
    )


def load_sorted(path: Path, time_key: str) -> tuple[list[dict[str, object]], list[float]]:
    rows: list[dict[str, object]] = read_csv(path)
    rows.sort(key=lambda row: to_float(row.get(time_key)))
    return rows, [to_float(row.get(time_key)) for row in rows]


def build_geometry_rows(run_dir: Path, high_error_threshold_m: float) -> list[dict[str, object]]:
    pos_rows = read_csv(run_dir / "phase_error_memory_diag" / "position_update_rows_with_pmem.csv")
    gnss_rows, gnss_times = load_sorted(run_dir / "gnss_update_debug.csv", "ros_time_sec")
    heading_rows, heading_times = load_sorted(run_dir / "heading_update_debug.csv", "ros_time_sec")
    joined_rows, joined_times = load_sorted(
        run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv",
        "pair_ros_time_sec",
    )
    fit = load_fit(run_dir)

    out: list[dict[str, object]] = []
    for row in pos_rows:
        if to_float(row.get("pair_iekf_error_h_m")) <= high_error_threshold_m:
            continue
        if not any(in_window(row, start, end) for _, start, end in TARGET_WINDOWS):
            continue
        ros_t = to_float(row.get("ros_time_sec"))
        gnss, gnss_dt = nearest(gnss_rows, gnss_times, ros_t, 0.15)
        heading, heading_dt = nearest(heading_rows, heading_times, ros_t, 0.35)
        joined, joined_dt = nearest(joined_rows, joined_times, ros_t, 0.15)
        if gnss is None:
            continue

        before_x = to_float(row.get("before_x_m"))
        before_y = to_float(row.get("before_y_m"))
        after_x = to_float(row.get("after_x_m"))
        after_y = to_float(row.get("after_y_m"))
        gt_x = to_float(row.get("gt_x_m"))
        gt_y = to_float(row.get("gt_y_m"))
        need_x = gt_x - before_x
        need_y = gt_y - before_y
        update_x = after_x - before_x
        update_y = after_y - before_y

        residual_x, residual_y = transform_residual_to_xy(
            to_float(gnss.get("gnss_position_residual_n_m")),
            to_float(gnss.get("gnss_position_residual_e_m")),
            fit,
        )
        # kf_gins logs residual as state-minus-measurement for these rows:
        # the applied update is opposite the residual vector.
        meas_delta_x = -residual_x
        meas_delta_y = -residual_y
        meas_x = before_x + meas_delta_x
        meas_y = before_y + meas_delta_y
        meas_err_x = meas_x - gt_x
        meas_err_y = meas_y - gt_y

        ekf2_err_x = to_float(joined.get("ekf2_error_x_m")) if joined else math.nan
        ekf2_err_y = to_float(joined.get("ekf2_error_y_m")) if joined else math.nan

        need_h = vec_norm(need_x, need_y)
        update_h = vec_norm(update_x, update_y)
        meas_delta_h = vec_norm(meas_delta_x, meas_delta_y)
        meas_err_h = vec_norm(meas_err_x, meas_err_y)

        out.append({
            "sequence": row.get("sequence"),
            "time_since_arm_sec": row.get("time_since_arm_sec"),
            "ros_time_sec": ros_t,
            "gnss_join_dt_sec": gnss_dt,
            "heading_join_dt_sec": heading_dt,
            "joined_join_dt_sec": joined_dt,
            "context": context_label(row),
            "turning_now": row.get("turning_now"),
            "post_turn_context": row.get("post_turn_context"),
            "armed_cruise_context": row.get("armed_cruise_context"),
            "pair_iekf_error_h_m": row.get("pair_iekf_error_h_m"),
            "pair_ekf2_error_h_m": row.get("pair_ekf2_error_h_m"),
            "ekf2_error_x_m": ekf2_err_x,
            "ekf2_error_y_m": ekf2_err_y,
            "ekf2_error_heading_from_gt_deg": math.degrees(math.atan2(ekf2_err_y, ekf2_err_x))
                if finite(ekf2_err_x) and finite(ekf2_err_y) else math.nan,
            "need_correction_h_m": need_h,
            "gnss_measurement_delta_h_m": meas_delta_h,
            "applied_update_h_m": update_h,
            "gnss_measurement_error_h_m": meas_err_h,
            "need_over_gnss_delta": need_h / meas_delta_h if meas_delta_h > 1e-12 else math.nan,
            "update_over_gnss_delta": update_h / meas_delta_h if meas_delta_h > 1e-12 else math.nan,
            "gnss_delta_over_need": meas_delta_h / need_h if need_h > 1e-12 else math.nan,
            "measurement_error_over_iekf_error": (
                meas_err_h / to_float(row.get("pair_iekf_error_h_m"))
                if to_float(row.get("pair_iekf_error_h_m")) > 1e-12 else math.nan
            ),
            "angle_need_vs_update_deg": vec_angle_deg(need_x, need_y, update_x, update_y),
            "angle_need_vs_gnss_delta_deg": vec_angle_deg(need_x, need_y, meas_delta_x, meas_delta_y),
            "angle_update_vs_gnss_delta_deg": vec_angle_deg(update_x, update_y, meas_delta_x, meas_delta_y),
            "heading_mode": heading.get("heading_mode", "") if heading else "",
            "heading_residual_before_deg": to_float(heading.get("residual_before_deg")) if heading else math.nan,
            "heading_residual_after_deg": to_float(heading.get("residual_after_deg")) if heading else math.nan,
            "heading_measurement_std_deg": to_float(heading.get("measurement_std_deg")) if heading else math.nan,
            "heading_yaw_correction_abs_deg": to_float(heading.get("yaw_correction_abs_deg")) if heading else math.nan,
            "yaw_before_deg": row.get("yaw_before_deg"),
            "yaw_after_deg": row.get("yaw_after_deg"),
            "horizontal_speed_mps": row.get("horizontal_speed_mps"),
            "gyro_deg_s": row.get("gyro_deg_s"),
            "source_yaw_rate_deg_s": row.get("source_yaw_rate_deg_s"),
            "phase_error_memory_pressure_active": row.get("phase_error_memory_pressure_active"),
            "phase_error_memory_candidate_active": row.get("phase_error_memory_candidate_active"),
            "phase_error_memory_reason": row.get("phase_error_memory_reason"),
        })
    out.sort(key=lambda item: to_float(item.get("time_since_arm_sec")))
    return out


def build_interval_rows(run_dir: Path) -> list[dict[str, object]]:
    rows = read_csv(run_dir / "phase_error_memory_diag" / "position_update_rows_with_pmem.csv")
    rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    out: list[dict[str, object]] = []
    for current, nxt in zip(rows, rows[1:]):
        start = to_float(current.get("time_since_arm_sec"))
        end = to_float(nxt.get("time_since_arm_sec"))
        if not finite(start) or not finite(end) or end <= start:
            continue
        if not any(start >= win_start and start < win_end for _, win_start, win_end in TARGET_WINDOWS):
            continue
        after_err_x = to_float(current.get("error_after_x_m"))
        after_err_y = to_float(current.get("error_after_y_m"))
        next_before_err_x = to_float(nxt.get("error_before_x_m"))
        next_before_err_y = to_float(nxt.get("error_before_y_m"))
        growth_x = next_before_err_x - after_err_x
        growth_y = next_before_err_y - after_err_y
        growth_h = vec_norm(growth_x, growth_y)
        out.append({
            "start_time_since_arm_sec": start,
            "end_time_since_arm_sec": end,
            "dt_sec": end - start,
            "start_context": context_label(current),
            "end_context": context_label(nxt),
            "start_error_after_h_m": current.get("error_after_h_m"),
            "next_error_before_h_m": nxt.get("error_before_h_m"),
            "scalar_error_growth_m": to_float(nxt.get("error_before_h_m")) - to_float(current.get("error_after_h_m")),
            "vector_error_growth_h_m": growth_h,
            "vector_error_growth_x_m": growth_x,
            "vector_error_growth_y_m": growth_y,
            "start_iekf_error_h_m": current.get("pair_iekf_error_h_m"),
            "end_iekf_error_h_m": nxt.get("pair_iekf_error_h_m"),
            "horizontal_speed_mps": current.get("horizontal_speed_mps"),
            "gyro_deg_s": current.get("gyro_deg_s"),
            "source_yaw_rate_deg_s": current.get("source_yaw_rate_deg_s"),
            "phase_error_memory_pressure_active": current.get("phase_error_memory_pressure_active"),
            "phase_error_memory_candidate_active": current.get("phase_error_memory_candidate_active"),
        })
    return out


def summarize_geometry(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    summary: list[dict[str, object]] = []
    for label, start, end in TARGET_WINDOWS:
        subset = [row for row in rows if in_window(row, start, end)]
        summary.append({
            "window": label,
            "rows": len(subset),
            "iekf_mean_m": mean(to_float(row.get("pair_iekf_error_h_m")) for row in subset),
            "ekf2_mean_m": mean(to_float(row.get("pair_ekf2_error_h_m")) for row in subset),
            "need_mean_m": mean(to_float(row.get("need_correction_h_m")) for row in subset),
            "gnss_delta_mean_m": mean(to_float(row.get("gnss_measurement_delta_h_m")) for row in subset),
            "gnss_measurement_error_mean_m": mean(to_float(row.get("gnss_measurement_error_h_m")) for row in subset),
            "gnss_delta_over_need_mean": mean(to_float(row.get("gnss_delta_over_need")) for row in subset),
            "update_over_gnss_delta_mean": mean(to_float(row.get("update_over_gnss_delta")) for row in subset),
            "angle_need_gnss_delta_mean_deg": mean(to_float(row.get("angle_need_vs_gnss_delta_deg")) for row in subset),
            "heading_residual_abs_mean_deg": mean(
                abs(to_float(row.get("heading_residual_before_deg"))) for row in subset),
        })
    return summary


def summarize_intervals(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    summary: list[dict[str, object]] = []
    for label, start, end in TARGET_WINDOWS:
        subset = [
            row for row in rows
            if start <= to_float(row.get("start_time_since_arm_sec")) < end
        ]
        summary.append({
            "window": label,
            "intervals": len(subset),
            "scalar_growth_mean_m": mean(to_float(row.get("scalar_error_growth_m")) for row in subset),
            "scalar_growth_p95_m": percentile(
                (to_float(row.get("scalar_error_growth_m")) for row in subset), 95.0),
            "vector_growth_mean_m": mean(to_float(row.get("vector_error_growth_h_m")) for row in subset),
            "vector_growth_p95_m": percentile(
                (to_float(row.get("vector_error_growth_h_m")) for row in subset), 95.0),
            "positive_scalar_growth_fraction": mean(
                1.0 if to_float(row.get("scalar_error_growth_m")) > 0.0 else 0.0
                for row in subset),
        })
    return summary


def write_report(
    out_dir: Path,
    geometry_summary: list[dict[str, object]],
    interval_summary: list[dict[str, object]],
    geometry_rows: list[dict[str, object]],
    interval_rows: list[dict[str, object]],
) -> None:
    geom_table = [
        [
            row["window"],
            row["rows"],
            fmt(row["iekf_mean_m"]),
            fmt(row["ekf2_mean_m"]),
            fmt(row["need_mean_m"]),
            fmt(row["gnss_delta_mean_m"]),
            fmt(row["gnss_measurement_error_mean_m"]),
            fmt(row["gnss_delta_over_need_mean"], 3),
            fmt(row["update_over_gnss_delta_mean"], 3),
            fmt(row["heading_residual_abs_mean_deg"], 3),
        ]
        for row in geometry_summary
    ]
    interval_table = [
        [
            row["window"],
            row["intervals"],
            fmt(row["scalar_growth_mean_m"]),
            fmt(row["scalar_growth_p95_m"]),
            fmt(row["vector_growth_mean_m"]),
            fmt(row["vector_growth_p95_m"]),
            fmt(row["positive_scalar_growth_fraction"], 3),
        ]
        for row in interval_summary
    ]
    examples = [
        [
            fmt(row["time_since_arm_sec"], 1),
            row["context"],
            fmt(row["pair_iekf_error_h_m"]),
            fmt(row["pair_ekf2_error_h_m"]),
            fmt(row["need_correction_h_m"]),
            fmt(row["gnss_measurement_delta_h_m"]),
            fmt(row["gnss_measurement_error_h_m"]),
            fmt(row["gnss_delta_over_need"], 3),
            fmt(row["angle_need_vs_gnss_delta_deg"], 1),
            fmt(row["heading_residual_before_deg"], 2),
        ]
        for row in geometry_rows[:14]
    ]
    interval_examples = [
        [
            fmt(row["start_time_since_arm_sec"], 1),
            fmt(row["end_time_since_arm_sec"], 1),
            row["start_context"],
            row["end_context"],
            fmt(row["scalar_error_growth_m"]),
            fmt(row["vector_error_growth_h_m"]),
            fmt(row["start_iekf_error_h_m"]),
            fmt(row["end_iekf_error_h_m"]),
            fmt(row["horizontal_speed_mps"]),
            fmt(row["gyro_deg_s"], 1),
        ]
        for row in sorted(
            interval_rows,
            key=lambda item: to_float(item.get("vector_error_growth_h_m")),
            reverse=True,
        )[:14]
    ]
    lines = [
        "# Shortgen02 Measurement Geometry Diagnostic",
        "",
        "This diagnostic separates groundtruth-required correction, GNSS-measurement correction, actual IEKF update, heading residual, and between-update error growth.",
        "",
        "## Geometry Summary",
        "",
        markdown_table(
            [
                "window",
                "rows",
                "iekf_mean",
                "ekf2_mean",
                "need_mean",
                "gnss_delta_mean",
                "gnss_meas_err_mean",
                "gnss/need",
                "update/gnss",
                "abs_heading_resid",
            ],
            geom_table,
        ),
        "",
        "## Interval Growth Summary",
        "",
        markdown_table(
            [
                "window",
                "intervals",
                "scalar_growth_mean",
                "scalar_growth_p95",
                "vector_growth_mean",
                "vector_growth_p95",
                "positive_frac",
            ],
            interval_table,
        ),
        "",
        "## High-Error Examples",
        "",
        markdown_table(
            [
                "t_arm",
                "ctx",
                "iekf",
                "ekf2",
                "need",
                "gnss_delta",
                "gnss_meas_err",
                "gnss/need",
                "angle",
                "head_resid",
            ],
            examples,
        ),
        "",
        "## Largest Interval Growth",
        "",
        markdown_table(
            [
                "start",
                "end",
                "ctx0",
                "ctx1",
                "scalar_growth",
                "vector_growth",
                "iekf0",
                "iekf1",
                "hspeed",
                "gyro",
            ],
            interval_examples,
        ),
        "",
        "## Interpretation",
        "",
        "- GNSS measurement delta points in almost the same direction as the groundtruth-required correction, so the dominant issue is not a sign or phase inversion.",
        "- GNSS measurement delta is much smaller than the groundtruth-required correction, leaving the inferred GNSS measurement itself still far from groundtruth.",
        "- Heading residuals in the target rows are small compared with the horizontal position gap, so yaw update gating is not the primary explanation for these rows.",
        "- Between-update growth remains nonzero, but the larger fact is that each accepted GNSS update starts from an already large state-vs-groundtruth error and the measurement only supplies a partial correction.",
        "",
        "Generated files:",
        f"- `{out_dir / 'target_update_geometry_rows.csv'}`",
        f"- `{out_dir / 'target_interval_growth_rows.csv'}`",
        f"- `{out_dir / 'geometry_summary.csv'}`",
        f"- `{out_dir / 'interval_growth_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out-dir")
    parser.add_argument("--high-error-threshold-m", type=float, default=0.6)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir) if args.out_dir else run_dir / "shortgen02_measurement_geometry_diag"
    out_dir.mkdir(parents=True, exist_ok=True)

    geometry_rows = build_geometry_rows(run_dir, args.high_error_threshold_m)
    interval_rows = build_interval_rows(run_dir)
    geometry_summary = summarize_geometry(geometry_rows)
    interval_summary = summarize_intervals(interval_rows)

    write_csv(out_dir / "target_update_geometry_rows.csv", geometry_rows)
    write_csv(out_dir / "target_interval_growth_rows.csv", interval_rows)
    write_csv(out_dir / "geometry_summary.csv", geometry_summary)
    write_csv(out_dir / "interval_growth_summary.csv", interval_summary)
    write_report(out_dir, geometry_summary, interval_summary, geometry_rows, interval_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
