#!/usr/bin/env python3
"""Summarize offline groundtruth projection-mode A/B outputs."""

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


MODE_DIRS = [
    ("gps_fit_all", "current gps_fit all"),
    ("raw_wgs84_enu", "raw WGS84 ENU"),
    ("px4_sphere_anisotropic", "PX4 sphere anisotropic"),
]
SELECTED_WINDOWS = [
    "armed_all",
    "segment_mission_all",
    "segment_main_maneuver_40_180",
    "120-140",
    "140-160",
    "160-180",
]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def load_mode_summary(modes_dir: Path) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for mode_dir, label in MODE_DIRS:
        path = modes_dir / mode_dir / "groundtruth_summary.csv"
        for row in read_csv(path):
            if row.get("window") not in SELECTED_WINDOWS:
                continue
            out.append({
                "projection_mode": mode_dir,
                "projection_label": label,
                "window": row.get("window", ""),
                "estimator": row.get("estimator", ""),
                "rows": to_float(row.get("rows"), 0.0),
                "xy_rmse_m": to_float(row.get("xy_rmse_m")),
                "xy_mean_m": to_float(row.get("xy_mean_m")),
                "xy_p95_m": to_float(row.get("xy_p95_m")),
                "xy_max_m": to_float(row.get("xy_max_m")),
                "frac_over_0p3": to_float(row.get("frac_over_0p3")),
            })
    return out


def nearest(rows: list[dict[str, object]], times: list[float], t: float, max_dt_sec: float) -> dict[str, object] | None:
    if not finite(t):
        return None
    idx = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - t), rows[pos]))
    if not candidates:
        return None
    dt, row = min(candidates, key=lambda item: item[0])
    return row if dt <= max_dt_sec else None


def load_target_high_summary(run_dir: Path, modes_dir: Path) -> list[dict[str, object]]:
    target_path = run_dir / "shortgen02_measurement_geometry_diag" / "target_update_geometry_rows.csv"
    if not target_path.exists():
        return []
    target_rows = read_csv(target_path)
    target_times = [to_float(row.get("ros_time_sec")) for row in target_rows]
    out: list[dict[str, object]] = []
    for mode_dir, label in MODE_DIRS:
        joined: list[dict[str, object]] = read_csv(modes_dir / mode_dir / "groundtruth_joined.csv")
        joined.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
        joined_times = [to_float(row.get("pair_ros_time_sec")) for row in joined]
        matched: list[dict[str, object]] = []
        for t in target_times:
            row = nearest(joined, joined_times, t, 0.15)
            if row is not None:
                matched.append(row)
        for estimator, key in (("ekf2", "ekf2_error_xy_m"), ("iekf_normalized", "iekf_error_xy_m")):
            errors = [to_float(row.get(key)) for row in matched]
            out.append({
                "projection_mode": mode_dir,
                "projection_label": label,
                "window": "target_high_rows",
                "estimator": estimator,
                "rows": len(errors),
                "xy_mean_m": mean(errors),
                "xy_p95_m": percentile(errors, 95.0),
                "xy_max_m": max((item for item in errors if finite(item)), default=math.nan),
                "frac_over_0p3": mean(1.0 if finite(item) and item > 0.3 else 0.0 for item in errors),
            })
    return out


def lookup(rows: list[dict[str, object]], mode: str, window: str, estimator: str) -> dict[str, object]:
    return next(
        row for row in rows
        if row.get("projection_mode") == mode and row.get("window") == window and row.get("estimator") == estimator
    )


def maybe_lookup(rows: list[dict[str, object]], mode: str, window: str, estimator: str) -> dict[str, object] | None:
    return next(
        (
            row for row in rows
            if row.get("projection_mode") == mode and row.get("window") == window and row.get("estimator") == estimator
        ),
        None,
    )


def write_report(
    out_dir: Path,
    selected_rows: list[dict[str, object]],
    target_rows: list[dict[str, object]],
) -> None:
    table: list[list[object]] = []
    for mode, label in MODE_DIRS:
        for window in ("armed_all", "120-140", "140-160", "160-180"):
            ekf2 = lookup(selected_rows, mode, window, "ekf2")
            iekf = lookup(selected_rows, mode, window, "iekf_normalized")
            table.append([
                label,
                window,
                int(to_float(ekf2.get("rows"), 0.0)),
                fmt(ekf2.get("xy_mean_m")),
                fmt(iekf.get("xy_mean_m")),
                fmt(to_float(iekf.get("xy_mean_m")) - to_float(ekf2.get("xy_mean_m"))),
                fmt(ekf2.get("xy_p95_m")),
                fmt(iekf.get("xy_p95_m")),
            ])

    high_table: list[list[object]] = []
    for mode, label in MODE_DIRS:
        ekf2 = maybe_lookup(target_rows, mode, "target_high_rows", "ekf2")
        iekf = maybe_lookup(target_rows, mode, "target_high_rows", "iekf_normalized")
        if ekf2 is None or iekf is None:
            continue
        high_table.append([
            label,
            int(to_float(ekf2.get("rows"), 0.0)),
            fmt(ekf2.get("xy_mean_m")),
            fmt(iekf.get("xy_mean_m")),
            fmt(to_float(iekf.get("xy_mean_m")) - to_float(ekf2.get("xy_mean_m"))),
            fmt(ekf2.get("xy_p95_m")),
            fmt(iekf.get("xy_p95_m")),
            fmt(iekf.get("frac_over_0p3"), 3),
        ])

    current_160 = lookup(selected_rows, "gps_fit_all", "160-180", "iekf_normalized")
    px4_160 = lookup(selected_rows, "px4_sphere_anisotropic", "160-180", "iekf_normalized")
    current_high = maybe_lookup(target_rows, "gps_fit_all", "target_high_rows", "iekf_normalized")
    px4_high = maybe_lookup(target_rows, "px4_sphere_anisotropic", "target_high_rows", "iekf_normalized")
    current_high_ekf2 = maybe_lookup(target_rows, "gps_fit_all", "target_high_rows", "ekf2")
    px4_high_ekf2 = maybe_lookup(target_rows, "px4_sphere_anisotropic", "target_high_rows", "ekf2")

    lines = [
        "# Groundtruth Projection Mode A/B Summary",
        "",
        "This report compares projection modes in `offline_groundtruth_convergence_diagnostic.py` using the same PMEM0 shortgen02 run, same ULog groundtruth, same early `0-20 s` translation alignment, and unchanged estimator outputs.",
        "",
        "## Window Summary",
        "",
        markdown_table(
            ["projection", "window", "rows", "ekf2_mean", "iekf_mean", "iekf_minus_ekf2", "ekf2_p95", "iekf_p95"],
            table,
        ),
        "",
        "## Target High Rows",
        "",
        (
            markdown_table(
                ["projection", "rows", "ekf2_mean", "iekf_mean", "iekf_minus_ekf2", "ekf2_p95", "iekf_p95", "iekf_frac_over_0p3"],
                high_table,
            )
            if high_table
            else "No target-high geometry rows were available for this run."
        ),
        "",
        "## Readout",
        "",
        f"- Current `gps_fit_all` keeps `160-180 s` IEKF mean at `{fmt(current_160.get('xy_mean_m'))}` m; PX4-sphere anisotropic projection gives `{fmt(px4_160.get('xy_mean_m'))}` m.",
        (
            f"- Current target high rows are EKF2 `{fmt(current_high_ekf2.get('xy_mean_m'))}` m vs IEKF `{fmt(current_high.get('xy_mean_m'))}` m."
            if current_high is not None and current_high_ekf2 is not None
            else "- Target-high rows were not available for this run."
        ),
        (
            f"- PX4-sphere anisotropic target high rows are EKF2 `{fmt(px4_high_ekf2.get('xy_mean_m'))}` m vs IEKF `{fmt(px4_high.get('xy_mean_m'))}` m."
            if px4_high is not None and px4_high_ekf2 is not None
            else "- PX4-sphere target-high rows were not available for this run."
        ),
        "- This is an offline output-frame/evaluation correction only. It should not be presented as an estimator mechanism until an opt-in online output topic or equivalent validation is added.",
        "",
        "Generated files:",
        f"- `{out_dir / 'projection_mode_window_summary.csv'}`",
        f"- `{out_dir / 'projection_mode_target_high_summary.csv'}`",
    ]
    (out_dir / "projection_mode_ab_summary.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--modes-dir")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    modes_dir = Path(args.modes_dir) if args.modes_dir else run_dir / "offline_groundtruth_projection_modes"
    selected_rows = load_mode_summary(modes_dir)
    target_rows = load_target_high_summary(run_dir, modes_dir)
    write_csv(modes_dir / "projection_mode_window_summary.csv", selected_rows)
    write_csv(modes_dir / "projection_mode_target_high_summary.csv", target_rows)
    write_report(modes_dir, selected_rows, target_rows)
    print(f"wrote: {modes_dir / 'projection_mode_ab_summary.md'}")


if __name__ == "__main__":
    main()
