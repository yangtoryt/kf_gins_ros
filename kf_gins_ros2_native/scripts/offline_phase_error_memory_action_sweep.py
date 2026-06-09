#!/usr/bin/env python3
"""Offline selector/action sweep for phase-error memory diagnostics."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

from offline_cross_route_failure_atlas import (
    finite,
    markdown_table,
    mean,
    percentile,
    to_float,
    write_csv,
)


WINDOWS = [
    ("120-180", 120.0, 180.0),
    ("140-180", 140.0, 180.0),
    ("160-180", 160.0, 180.0),
]

TARGET_STDS_M = [0.06, 0.05, 0.04, 0.03]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def bool01(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"} or to_float(value, 0.0) > 0.5


def pct(value: float) -> str:
    if not finite(value):
        return "nan"
    return f"{100.0 * value:.1f}%"


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return finite(t) and start <= t < end


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


def residual_h(row: dict[str, object]) -> float:
    val = to_float(row.get("phase_error_memory_residual_h_m"))
    if finite(val):
        return val
    return to_float(row.get("gnss_residual_h_m"))


def dx_over_residual(row: dict[str, object]) -> float:
    val = to_float(row.get("phase_error_memory_dx_over_residual_h"))
    if finite(val):
        return val
    return to_float(row.get("dx_over_residual_h"))


def recent_age(row: dict[str, object]) -> float:
    return to_float(row.get("phase_error_memory_recent_turnpost_age_sec"))


def pressure(row: dict[str, object], residual_thr: float, dx_ratio_thr: float) -> bool:
    res = residual_h(row)
    ratio = dx_over_residual(row)
    return finite(res) and finite(ratio) and res >= residual_thr and ratio <= dx_ratio_thr


def selector_active(row: dict[str, object], name: str) -> bool:
    if name == "pressure_r0p16_dx0p22":
        return pressure(row, 0.16, 0.22)
    if name == "pressure_r0p14_dx0p22":
        return pressure(row, 0.14, 0.22)
    if name == "pressure_r0p12_dx0p22":
        return pressure(row, 0.12, 0.22)
    if name == "pressure_r0p16_dx0p25":
        return pressure(row, 0.16, 0.25)
    if name == "pressure_r0p14_dx0p25":
        return pressure(row, 0.14, 0.25)
    if name.startswith("pressure_r0p16_dx0p22_recent"):
        hold = float(name.rsplit("recent", 1)[1].replace("s", ""))
        age = recent_age(row)
        return pressure(row, 0.16, 0.22) and finite(age) and age <= hold
    if name == "pressure_r0p16_dx0p22_turnpost_now":
        return pressure(row, 0.16, 0.22) and (
            bool01(row.get("turning_now")) or bool01(row.get("post_turn_context"))
        )
    if name == "pressure_r0p16_dx0p22_mission_context":
        return pressure(row, 0.16, 0.22) and (
            bool01(row.get("turning_now"))
            or bool01(row.get("post_turn_context"))
            or bool01(row.get("armed_cruise_context"))
        )
    raise KeyError(name)


def selector_names() -> list[str]:
    return [
        "pressure_r0p16_dx0p22",
        "pressure_r0p14_dx0p22",
        "pressure_r0p12_dx0p22",
        "pressure_r0p16_dx0p25",
        "pressure_r0p14_dx0p25",
        "pressure_r0p16_dx0p22_recent10s",
        "pressure_r0p16_dx0p22_recent15s",
        "pressure_r0p16_dx0p22_recent20s",
        "pressure_r0p16_dx0p22_recent25s",
        "pressure_r0p16_dx0p22_turnpost_now",
        "pressure_r0p16_dx0p22_mission_context",
    ]


def summarize_selector(
    label: str,
    rows: list[dict[str, object]],
    selector: str,
    high_error_threshold_m: float,
) -> dict[str, object]:
    high = [row for row in rows if to_float(row.get("pair_iekf_error_h_m")) > high_error_threshold_m]
    low = [row for row in rows if to_float(row.get("pair_iekf_error_h_m")) <= high_error_threshold_m]
    active = [row for row in rows if selector_active(row, selector)]
    high_active = [row for row in high if selector_active(row, selector)]
    low_active = [row for row in low if selector_active(row, selector)]
    ekf2_when_active = [to_float(row.get("pair_ekf2_error_h_m")) for row in active]
    iekf_when_active = [to_float(row.get("pair_iekf_error_h_m")) for row in active]
    return {
        "window": label,
        "selector": selector,
        "rows": len(rows),
        "high_rows": len(high),
        "active_rows": len(active),
        "active_fraction": len(active) / len(rows) if rows else math.nan,
        "high_active_rows": len(high_active),
        "high_coverage": len(high_active) / len(high) if high else math.nan,
        "low_active_rows": len(low_active),
        "low_active_fraction": len(low_active) / len(low) if low else math.nan,
        "active_iekf_mean_m": mean(iekf_when_active),
        "active_ekf2_mean_m": mean(ekf2_when_active),
    }


def inferred_new_gain(k_old: float, old_std_m: float, target_std_m: float) -> float:
    if not (finite(k_old) and finite(old_std_m) and finite(target_std_m)):
        return math.nan
    if k_old <= 0.0 or k_old >= 0.95 or old_std_m <= 0.0 or target_std_m <= 0.0:
        return math.nan
    p_est = k_old * old_std_m * old_std_m / (1.0 - k_old)
    return p_est / (p_est + target_std_m * target_std_m)


def projected_after_error(row: dict[str, object], target_std_m: float) -> float:
    before_x = to_float(row.get("before_x_m"))
    before_y = to_float(row.get("before_y_m"))
    after_x = to_float(row.get("after_x_m"))
    after_y = to_float(row.get("after_y_m"))
    gt_x = to_float(row.get("gt_x_m"))
    gt_y = to_float(row.get("gt_y_m"))
    k_old = dx_over_residual(row)
    old_std = to_float(row.get("gnss_position_std_h_m"), 0.08)
    if not all(finite(item) for item in (before_x, before_y, after_x, after_y, gt_x, gt_y, k_old, old_std)):
        return math.nan
    if old_std <= target_std_m:
        return to_float(row.get("error_after_h_m"))
    k_new = inferred_new_gain(k_old, old_std, target_std_m)
    if not finite(k_new) or k_old <= 0.0:
        return math.nan
    ratio = min(max(k_new / k_old, 0.0), 4.0)
    proj_x = before_x + (after_x - before_x) * ratio
    proj_y = before_y + (after_y - before_y) * ratio
    return math.hypot(proj_x - gt_x, proj_y - gt_y)


def required_scale_for_error(
    row: dict[str, object],
    target_error_m: float,
) -> tuple[float, float, float]:
    before_x = to_float(row.get("before_x_m"))
    before_y = to_float(row.get("before_y_m"))
    after_x = to_float(row.get("after_x_m"))
    after_y = to_float(row.get("after_y_m"))
    gt_x = to_float(row.get("gt_x_m"))
    gt_y = to_float(row.get("gt_y_m"))
    if not all(finite(item) for item in (before_x, before_y, after_x, after_y, gt_x, gt_y, target_error_m)):
        return math.nan, math.nan, math.nan
    dx = after_x - before_x
    dy = after_y - before_y
    vx = gt_x - before_x
    vy = gt_y - before_y
    a = dx * dx + dy * dy
    if a <= 1e-12:
        return math.nan, math.nan, math.nan

    s_opt = (vx * dx + vy * dy) / a
    min_error = math.hypot(before_x + s_opt * dx - gt_x, before_y + s_opt * dy - gt_y)

    # Solve |before + s * dx - gt|^2 <= target_error_m^2 and return the
    # smallest scale at or above the observed update scale s=1.
    b_quad = -2.0 * (vx * dx + vy * dy)
    c_quad = vx * vx + vy * vy - target_error_m * target_error_m
    disc = b_quad * b_quad - 4.0 * a * c_quad
    if disc < 0.0:
        return s_opt, min_error, math.nan
    lo = (-b_quad - math.sqrt(disc)) / (2.0 * a)
    hi = (-b_quad + math.sqrt(disc)) / (2.0 * a)
    if hi < 1.0:
        return s_opt, min_error, math.nan
    need = max(1.0, lo)
    if need > hi:
        return s_opt, min_error, math.nan
    return s_opt, min_error, need


def build_required_scale_rows(
    rows: list[dict[str, object]],
    high_error_threshold_m: float,
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for label, start, end in WINDOWS:
        high = [
            row for row in rows
            if in_window(row, start, end)
            and to_float(row.get("pair_iekf_error_h_m")) > high_error_threshold_m
        ]
        high.sort(key=lambda row: to_float(row.get("pair_iekf_error_h_m")), reverse=True)
        for row in high:
            ekf2_error = to_float(row.get("pair_ekf2_error_h_m"))
            s_opt, min_error, need_ekf2 = required_scale_for_error(row, ekf2_error)
            _, _, need_0p3 = required_scale_for_error(row, 0.3)
            k_old = dx_over_residual(row)
            max_physical_scale = 1.0 / k_old if finite(k_old) and k_old > 0.0 else math.nan
            out.append({
                "window": label,
                "sequence": row.get("sequence"),
                "time_since_arm_sec": row.get("time_since_arm_sec"),
                "context": context_label(row),
                "pair_iekf_error_h_m": row.get("pair_iekf_error_h_m"),
                "pair_ekf2_error_h_m": row.get("pair_ekf2_error_h_m"),
                "residual_h_m": residual_h(row),
                "dx_over_residual_h": k_old,
                "max_physical_scale_from_gain1": max_physical_scale,
                "optimal_scale_along_current_dx": s_opt,
                "min_error_along_current_dx_m": min_error,
                "need_scale_to_match_ekf2": need_ekf2,
                "need_scale_to_0p3m": need_0p3,
                "ekf2_reachable_by_full_gnss_correction": (
                    int(finite(need_ekf2) and finite(max_physical_scale) and need_ekf2 <= max_physical_scale)
                ),
                "threshold_0p3_reachable_by_full_gnss_correction": (
                    int(finite(need_0p3) and finite(max_physical_scale) and need_0p3 <= max_physical_scale)
                ),
                "phase_error_memory_pressure_active": int(
                    selector_active(row, "pressure_r0p16_dx0p22")),
                "phase_error_memory_candidate_active": int(
                    selector_active(row, "pressure_r0p16_dx0p22_recent15s")),
            })
    return out


def summarize_required_scale(
    required_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    summary: list[dict[str, object]] = []
    for label, _, _ in WINDOWS:
        rows = [row for row in required_rows if row.get("window") == label]
        need_ekf2 = [to_float(row.get("need_scale_to_match_ekf2")) for row in rows]
        need_0p3 = [to_float(row.get("need_scale_to_0p3m")) for row in rows]
        summary.append({
            "window": label,
            "high_rows": len(rows),
            "need_ekf2_finite_rows": sum(1 for item in need_ekf2 if finite(item)),
            "need_ekf2_mean_scale": mean(need_ekf2),
            "need_ekf2_min_scale": min((item for item in need_ekf2 if finite(item)), default=math.nan),
            "need_ekf2_max_scale": max((item for item in need_ekf2 if finite(item)), default=math.nan),
            "need_0p3_finite_rows": sum(1 for item in need_0p3 if finite(item)),
            "need_0p3_mean_scale": mean(need_0p3),
            "need_0p3_min_scale": min((item for item in need_0p3 if finite(item)), default=math.nan),
            "need_0p3_max_scale": max((item for item in need_0p3 if finite(item)), default=math.nan),
            "ekf2_reachable_rows": sum(
                int(to_float(row.get("ekf2_reachable_by_full_gnss_correction"), 0.0) > 0.5)
                for row in rows),
            "threshold_0p3_reachable_rows": sum(
                int(to_float(row.get("threshold_0p3_reachable_by_full_gnss_correction"), 0.0) > 0.5)
                for row in rows),
        })
    return summary


def summarize_projection(
    label: str,
    rows: list[dict[str, object]],
    selector: str,
    target_std_m: float,
    high_error_threshold_m: float,
) -> dict[str, object]:
    active = [row for row in rows if selector_active(row, selector)]
    high_active = [
        row for row in active
        if to_float(row.get("pair_iekf_error_h_m")) > high_error_threshold_m
    ]
    actual = [to_float(row.get("error_after_h_m")) for row in active]
    projected = [projected_after_error(row, target_std_m) for row in active]
    high_actual = [to_float(row.get("error_after_h_m")) for row in high_active]
    high_projected = [projected_after_error(row, target_std_m) for row in high_active]
    return {
        "window": label,
        "selector": selector,
        "target_std_h_m": target_std_m,
        "active_rows": len(active),
        "high_active_rows": len(high_active),
        "actual_after_mean_m": mean(actual),
        "projected_after_mean_m": mean(projected),
        "projected_delta_mean_m": mean(projected) - mean(actual),
        "actual_after_p95_m": percentile(actual, 95.0),
        "projected_after_p95_m": percentile(projected, 95.0),
        "high_actual_after_mean_m": mean(high_actual),
        "high_projected_after_mean_m": mean(high_projected),
        "high_projected_delta_mean_m": mean(high_projected) - mean(high_actual),
    }


def write_report(
    out_dir: Path,
    selector_rows: list[dict[str, object]],
    projection_rows: list[dict[str, object]],
    required_summary_rows: list[dict[str, object]],
    high_error_threshold_m: float,
) -> None:
    main_selectors = [
        "pressure_r0p16_dx0p22",
        "pressure_r0p16_dx0p22_recent15s",
        "pressure_r0p16_dx0p22_recent20s",
        "pressure_r0p16_dx0p22_mission_context",
    ]
    selector_table: list[list[object]] = []
    for row in selector_rows:
        if row["window"] == "120-180" and row["selector"] in main_selectors:
            selector_table.append([
                row["selector"],
                row["high_rows"],
                row["high_active_rows"],
                pct(to_float(row["high_coverage"])),
                pct(to_float(row["active_fraction"])),
                pct(to_float(row["low_active_fraction"])),
            ])

    projection_table: list[list[object]] = []
    for row in projection_rows:
        if (
            row["window"] in {"120-180", "160-180"}
            and row["selector"] in {"pressure_r0p16_dx0p22", "pressure_r0p16_dx0p22_recent20s"}
            and abs(to_float(row["target_std_h_m"]) - 0.04) < 1e-9
        ):
            projection_table.append([
                row["window"],
                row["selector"],
                row["active_rows"],
                fmt(row["actual_after_mean_m"]),
                fmt(row["projected_after_mean_m"]),
                fmt(row["projected_delta_mean_m"]),
                fmt(row["high_projected_delta_mean_m"]),
            ])

    lines = [
        "# Phase Error Memory Action Sweep",
        "",
        f"high-error threshold: {high_error_threshold_m:.3f} m",
        "",
        "This is an offline selector and one-step action approximation. It does not prove closed-loop performance.",
        "The projection infers a scalar gain from the observed update and estimates the immediate after-update error if horizontal GNSS position std were lowered.",
        "",
        "## Selector Snapshot",
        "",
        markdown_table(
            ["selector", "high", "high_hit", "high_cov", "active_all", "active_low_error"],
            selector_table,
        ),
        "",
        "## One-Step Projection Snapshot",
        "",
        markdown_table(
            [
                "window",
                "selector",
                "active",
                "actual_mean",
                "projected_mean",
                "delta_mean",
                "high_delta_mean",
            ],
            projection_table,
        ),
        "",
        "## Required Correction Scale",
        "",
        markdown_table(
            [
                "window",
                "high",
                "need_ekf2_mean",
                "need_ekf2_min",
                "need_ekf2_max",
                "ekf2_reachable",
                "need_0.3_mean",
                "0.3_reachable",
            ],
            [
                [
                    row["window"],
                    row["high_rows"],
                    fmt(row["need_ekf2_mean_scale"], 2),
                    fmt(row["need_ekf2_min_scale"], 2),
                    fmt(row["need_ekf2_max_scale"], 2),
                    row["ekf2_reachable_rows"],
                    fmt(row["need_0p3_mean_scale"], 2),
                    row["threshold_0p3_reachable_rows"],
                ]
                for row in required_summary_rows
            ],
        ),
        "",
        "## Readout",
        "",
        "- `pressure_r0p16_dx0p22` is the pure weak-position-update label.",
        "- Fixed recent-turn/post holds trade coverage for lower active fraction; this can become route-tuned if used as the action gate.",
        "- If required correction scale is larger than `1 / dx_over_residual`, even a full gain-to-one GNSS correction along the observed update direction cannot reach the target.",
        "- Use this report only to choose the next shortgen02 dev-run mechanism, not as holdout evidence.",
        "",
        "Generated files:",
        f"- `{out_dir / 'pmem_action_selector_sweep.csv'}`",
        f"- `{out_dir / 'pmem_action_projection_sweep.csv'}`",
        f"- `{out_dir / 'pmem_required_correction_scale.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out-dir")
    parser.add_argument("--high-error-threshold-m", type=float, default=0.6)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir) if args.out_dir else run_dir / "phase_error_memory_action_sweep"
    rows_csv = run_dir / "phase_error_memory_diag" / "position_update_rows_with_pmem.csv"
    if not rows_csv.exists():
        raise FileNotFoundError(f"missing PMEM diagnostic rows: {rows_csv}")
    rows: list[dict[str, object]] = read_csv(rows_csv)

    selector_rows: list[dict[str, object]] = []
    projection_rows: list[dict[str, object]] = []
    for label, start, end in WINDOWS:
        subset = [row for row in rows if in_window(row, start, end)]
        for selector in selector_names():
            selector_rows.append(
                summarize_selector(label, subset, selector, args.high_error_threshold_m)
            )
        for selector in [
            "pressure_r0p16_dx0p22",
            "pressure_r0p16_dx0p22_recent15s",
            "pressure_r0p16_dx0p22_recent20s",
            "pressure_r0p16_dx0p22_mission_context",
        ]:
            for target_std_m in TARGET_STDS_M:
                projection_rows.append(
                    summarize_projection(
                        label,
                        subset,
                        selector,
                        target_std_m,
                        args.high_error_threshold_m,
                    )
                )

    out_dir.mkdir(parents=True, exist_ok=True)
    required_rows = build_required_scale_rows(rows, args.high_error_threshold_m)
    required_summary_rows = summarize_required_scale(required_rows)
    write_csv(out_dir / "pmem_action_selector_sweep.csv", selector_rows)
    write_csv(out_dir / "pmem_action_projection_sweep.csv", projection_rows)
    write_csv(out_dir / "pmem_required_correction_scale.csv", required_rows)
    write_csv(out_dir / "pmem_required_correction_scale_summary.csv", required_summary_rows)
    write_report(
        out_dir,
        selector_rows,
        projection_rows,
        required_summary_rows,
        args.high_error_threshold_m,
    )
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
