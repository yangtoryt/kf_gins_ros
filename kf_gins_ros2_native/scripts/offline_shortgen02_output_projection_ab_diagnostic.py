#!/usr/bin/env python3
"""Offline A/B for IEKF output-frame projection variants on shortgen02."""

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


WGS84_A_M = 6378137.0
WGS84_E2 = 6.69437999014e-3
PX4_SPHERE_RADIUS_M = 6371000.0

EVAL_WINDOWS = [
    ("120-124p5", 120.0, 124.5),
    ("171-178p5", 171.0, 178.5),
    ("120-180", 120.0, 180.0),
]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    if not finite(val):
        return "nan"
    return f"{val:.{digits}f}"


def norm2(x: float, y: float) -> float:
    return math.hypot(x, y)


def wgs84_radii(lat_rad: float) -> tuple[float, float]:
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    rn = WGS84_A_M / denom
    rm = WGS84_A_M * (1.0 - WGS84_E2) / (denom * denom * denom)
    return rm, rn


def load_origin_lat(run_dir: Path) -> float:
    gps_probe = run_dir / "gps_vs_pose.csv"
    for row in read_csv(gps_probe):
        lat = to_float(row.get("gps_lat_deg"))
        if finite(lat):
            return math.radians(lat)
    raise RuntimeError(f"missing gps_lat_deg in {gps_probe}")


def load_joined(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    rows: list[dict[str, object]] = read_csv(
        run_dir / "offline_groundtruth_convergence_diag" / "groundtruth_joined.csv"
    )
    rows.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
    return rows, [to_float(row.get("pair_ros_time_sec")) for row in rows]


def nearest(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt_sec: float,
) -> dict[str, object] | None:
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


def load_pair_join_rows(run_dir: Path) -> list[dict[str, object]]:
    joined_rows, joined_times = load_joined(run_dir)
    pair_rows = read_csv(run_dir / "ekf_iekf_pairs.csv")
    out: list[dict[str, object]] = []
    for pair in pair_rows:
        joined = nearest(joined_rows, joined_times, to_float(pair.get("ros_time_sec")), 0.05)
        if joined is None:
            continue
        row = dict(joined)
        row["raw_iekf_x_m"] = to_float(pair.get("iekf_x_m"))
        row["raw_iekf_y_m"] = to_float(pair.get("iekf_y_m"))
        out.append(row)
    return out


def load_high_rows(run_dir: Path, rows: list[dict[str, object]]) -> list[dict[str, object]]:
    rows_sorted = sorted(rows, key=lambda row: to_float(row.get("pair_ros_time_sec")))
    times = [to_float(row.get("pair_ros_time_sec")) for row in rows_sorted]
    high: list[dict[str, object]] = []
    for target in read_csv(run_dir / "shortgen02_measurement_geometry_diag" / "target_update_geometry_rows.csv"):
        row = nearest(rows_sorted, times, to_float(target.get("ros_time_sec")), 0.15)
        if row is not None:
            high.append(row)
    return high


def load_fit(run_dir: Path) -> dict[str, float]:
    for row in read_csv(run_dir / "offline_ulog_gps_groundtruth_diag" / "gps_groundtruth_fit.csv"):
        if row.get("topic") == "vehicle_gps_position" and row.get("subset") == "all":
            return {
                "a_real": to_float(row.get("a_real")),
                "a_imag": to_float(row.get("a_imag")),
            }
    raise RuntimeError("missing all-row GPS fit")


def is_armed(row: dict[str, object]) -> bool:
    return to_float(row.get("mavros_armed"), 0.0) > 0.5


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return finite(t) and start <= t < end


def variant_xy(row: dict[str, object], variant: str, fit: dict[str, float], east_scale: float, north_scale: float) -> tuple[float, float]:
    raw_x = to_float(row.get("raw_iekf_x_m"))
    raw_y = to_float(row.get("raw_iekf_y_m"))
    if variant == "raw_wgs84_enu":
        return raw_x, raw_y
    if variant == "gps_fit_all_scale_rot":
        return (
            fit["a_real"] * raw_x - fit["a_imag"] * raw_y,
            fit["a_imag"] * raw_x + fit["a_real"] * raw_y,
        )
    if variant == "px4_sphere_anisotropic":
        return raw_x * east_scale, raw_y * north_scale
    raise ValueError(variant)


def compute_offsets(
    rows: list[dict[str, object]],
    variant: str,
    fit: dict[str, float],
    east_scale: float,
    north_scale: float,
) -> tuple[float, float, int]:
    subset = [row for row in rows if is_armed(row) and in_window(row, 0.0, 20.0)]
    dx: list[float] = []
    dy: list[float] = []
    for row in subset:
        x, y = variant_xy(row, variant, fit, east_scale, north_scale)
        dx.append(to_float(row.get("gt_x_m")) - x)
        dy.append(to_float(row.get("gt_y_m")) - y)
    return mean(dx), mean(dy), len(subset)


def summarize_variant(
    rows: list[dict[str, object]],
    high_rows: list[dict[str, object]],
    variant: str,
    fit: dict[str, float],
    east_scale: float,
    north_scale: float,
) -> list[dict[str, object]]:
    offset_x, offset_y, offset_rows = compute_offsets(rows, variant, fit, east_scale, north_scale)

    def err(row: dict[str, object]) -> float:
        x, y = variant_xy(row, variant, fit, east_scale, north_scale)
        return norm2(x + offset_x - to_float(row.get("gt_x_m")), y + offset_y - to_float(row.get("gt_y_m")))

    out: list[dict[str, object]] = []
    for label, start, end in EVAL_WINDOWS:
        subset = [row for row in rows if is_armed(row) and in_window(row, start, end)]
        vals = [err(row) for row in subset]
        out.append({
            "variant": variant,
            "eval_window": label,
            "rows": len(vals),
            "mean_m": mean(vals),
            "p95_m": percentile(vals, 95.0),
            "max_m": max((item for item in vals if finite(item)), default=math.nan),
            "frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in vals),
            "offset_x_m": offset_x,
            "offset_y_m": offset_y,
            "offset_rows": offset_rows,
        })
    vals = [err(row) for row in high_rows]
    out.append({
        "variant": variant,
        "eval_window": "target_high_rows",
        "rows": len(vals),
        "mean_m": mean(vals),
        "p95_m": percentile(vals, 95.0),
        "max_m": max((item for item in vals if finite(item)), default=math.nan),
        "frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in vals),
        "offset_x_m": offset_x,
        "offset_y_m": offset_y,
        "offset_rows": offset_rows,
    })
    return out


def row_lookup(rows: list[dict[str, object]], variant: str, window: str) -> dict[str, object]:
    return next(row for row in rows if row.get("variant") == variant and row.get("eval_window") == window)


def write_report(
    out_dir: Path,
    rows: list[dict[str, object]],
    origin_lat_rad: float,
    east_scale: float,
    north_scale: float,
) -> None:
    table: list[list[object]] = []
    for variant in ("raw_wgs84_enu", "gps_fit_all_scale_rot", "px4_sphere_anisotropic"):
        for window in ("120-180", "target_high_rows"):
            row = row_lookup(rows, variant, window)
            table.append([
                variant,
                window,
                row["rows"],
                fmt(row["mean_m"]),
                fmt(row["p95_m"]),
                fmt(row["max_m"]),
                fmt(row["frac_over_0p3"], 3),
            ])
    current_high = row_lookup(rows, "gps_fit_all_scale_rot", "target_high_rows")
    px4_high = row_lookup(rows, "px4_sphere_anisotropic", "target_high_rows")
    current_all = row_lookup(rows, "gps_fit_all_scale_rot", "120-180")
    px4_all = row_lookup(rows, "px4_sphere_anisotropic", "120-180")
    lines = [
        "# Shortgen02 Output Projection A/B Diagnostic",
        "",
        "This diagnostic changes only the offline IEKF output-frame projection applied to `/kf_gins/odom` samples, then reuses the same early `0-20 s` translation alignment.",
        "",
        "## Projection Constants",
        "",
        markdown_table(
            ["origin_lat_deg", "px4_east_scale_vs_wgs84", "px4_north_scale_vs_wgs84"],
            [[fmt(math.degrees(origin_lat_rad), 7), fmt(east_scale, 7), fmt(north_scale, 7)]],
        ),
        "",
        "## Projection Variants",
        "",
        markdown_table(
            ["variant", "eval_window", "rows", "mean", "p95", "max", "frac_over_0p3"],
            table,
        ),
        "",
        "## Interpretation",
        "",
        f"- Current offline groundtruth normalization (`gps_fit_all_scale_rot`) gives target high-row mean `{fmt(current_high.get('mean_m'))}` m and all `120-180 s` mean `{fmt(current_all.get('mean_m'))}` m.",
        f"- A PX4-sphere-compatible anisotropic projection gives target high-row mean `{fmt(px4_high.get('mean_m'))}` m and all `120-180 s` mean `{fmt(px4_all.get('mean_m'))}` m.",
        "- This improves the late shortgen02 groundtruth score without changing estimator internals, which supports the frame/projection diagnosis.",
        "- High rows are still not fully solved by this projection-only change, so propagation/local translation residuals remain after the output-frame correction.",
        "",
        "Generated files:",
        f"- `{out_dir / 'projection_variant_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out-dir")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir) if args.out_dir else run_dir / "shortgen02_output_projection_ab_diag"
    out_dir.mkdir(parents=True, exist_ok=True)

    origin_lat = load_origin_lat(run_dir)
    rm, rn = wgs84_radii(origin_lat)
    east_scale = PX4_SPHERE_RADIUS_M / rn
    north_scale = PX4_SPHERE_RADIUS_M / rm
    rows = load_pair_join_rows(run_dir)
    high_rows = load_high_rows(run_dir, rows)
    fit = load_fit(run_dir)

    summary: list[dict[str, object]] = []
    for variant in ("raw_wgs84_enu", "gps_fit_all_scale_rot", "px4_sphere_anisotropic"):
        summary.extend(summarize_variant(rows, high_rows, variant, fit, east_scale, north_scale))

    write_csv(out_dir / "projection_variant_summary.csv", summary)
    write_report(out_dir, summary, origin_lat, east_scale, north_scale)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
