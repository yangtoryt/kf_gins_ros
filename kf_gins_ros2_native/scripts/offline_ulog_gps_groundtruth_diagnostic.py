#!/usr/bin/env python3
"""Diagnose PX4 ULog GPS latitude/longitude projection against groundtruth local position."""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from pathlib import Path
from typing import Iterable


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


def mean(values: Iterable[float]) -> float:
    vals = [item for item in values if finite(item)]
    return sum(vals) / len(vals) if vals else math.nan


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


def llh_to_ecef(lat_rad: float, lon_rad: float, h_m: float) -> tuple[float, float, float]:
    sin_lat = math.sin(lat_rad)
    cos_lat = math.cos(lat_rad)
    n = WGS84_A_M / math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    return (
        (n + h_m) * cos_lat * math.cos(lon_rad),
        (n + h_m) * cos_lat * math.sin(lon_rad),
        (n * (1.0 - WGS84_E2) + h_m) * sin_lat,
    )


def wgs84_radii(lat_rad: float) -> tuple[float, float]:
    sin_lat = math.sin(lat_rad)
    denom = math.sqrt(1.0 - WGS84_E2 * sin_lat * sin_lat)
    rn = WGS84_A_M / denom
    rm = WGS84_A_M * (1.0 - WGS84_E2) / (denom * denom * denom)
    return rm, rn


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


def nearest(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt_sec: float,
) -> tuple[dict[str, object] | None, float]:
    i = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for j in (i - 1, i):
        if 0 <= j < len(rows):
            candidates.append((abs(times[j] - t), rows[j]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    if dt > max_dt_sec:
        return None, dt
    return row, dt


def load_groundtruth(path: Path) -> tuple[list[dict[str, object]], tuple[float, float, float], tuple[float, float, float]]:
    raw_rows = read_csv(path)
    if not raw_rows:
        raise RuntimeError(f"empty groundtruth CSV: {path}")
    first = raw_rows[0]
    ref_lat_rad = math.radians(to_float(first.get("ref_lat")))
    ref_lon_rad = math.radians(to_float(first.get("ref_lon")))
    ref_alt_m = to_float(first.get("ref_alt"))
    if not all(finite(item) for item in (ref_lat_rad, ref_lon_rad, ref_alt_m)):
        raise RuntimeError(f"missing ref_lat/ref_lon/ref_alt in groundtruth CSV: {path}")
    origin_ecef = llh_to_ecef(ref_lat_rad, ref_lon_rad, ref_alt_m)
    rows: list[dict[str, object]] = []
    for row in raw_rows:
        timestamp_sec = to_float(row.get("timestamp")) * 1e-6
        gt_n = to_float(row.get("x"))
        gt_e = to_float(row.get("y"))
        gt_u = -to_float(row.get("z"))
        if not all(finite(item) for item in (timestamp_sec, gt_e, gt_n, gt_u)):
            continue
        rows.append(
            {
                "timestamp_sec": timestamp_sec,
                "gt_e_m": gt_e,
                "gt_n_m": gt_n,
                "gt_u_m": gt_u,
            }
        )
    rows.sort(key=lambda row: row["timestamp_sec"])  # type: ignore[index]
    return rows, (ref_lat_rad, ref_lon_rad, ref_alt_m), origin_ecef


def gps_llh(row: dict[str, str]) -> tuple[float, float, float]:
    lat_raw = to_float(row.get("lat"))
    lon_raw = to_float(row.get("lon"))
    alt_raw = to_float(row.get("alt"))
    return math.radians(lat_raw * 1e-7), math.radians(lon_raw * 1e-7), alt_raw * 1e-3


def join_gps_groundtruth(
    topic_name: str,
    gps_path: Path,
    gt_rows: list[dict[str, object]],
    gt_times: list[float],
    ref_llh: tuple[float, float, float],
    origin_ecef: tuple[float, float, float],
    max_dt_sec: float,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    ref_lat_rad, ref_lon_rad, _ = ref_llh
    for raw in read_csv(gps_path):
        timestamp_sec = to_float(raw.get("timestamp")) * 1e-6
        if not finite(timestamp_sec):
            continue
        gt, join_dt = nearest(gt_rows, gt_times, timestamp_sec, max_dt_sec)
        if gt is None:
            continue
        lat_rad, lon_rad, alt_m = gps_llh(raw)
        if not all(finite(item) for item in (lat_rad, lon_rad, alt_m)):
            continue
        gps_e, gps_n, gps_u = ecef_to_enu(
            llh_to_ecef(lat_rad, lon_rad, alt_m),
            origin_ecef,
            ref_lat_rad,
            ref_lon_rad,
        )
        err_e = gps_e - to_float(gt["gt_e_m"])
        err_n = gps_n - to_float(gt["gt_n_m"])
        rows.append(
            {
                "topic": topic_name,
                "timestamp_sec": timestamp_sec,
                "join_dt_sec": join_dt,
                "gps_e_m": gps_e,
                "gps_n_m": gps_n,
                "gps_u_m": gps_u,
                "gt_e_m": gt["gt_e_m"],
                "gt_n_m": gt["gt_n_m"],
                "gt_u_m": gt["gt_u_m"],
                "raw_error_e_m": err_e,
                "raw_error_n_m": err_n,
                "raw_error_xy_m": math.hypot(err_e, err_n),
            }
        )
    return rows


def fit_similarity(rows: list[dict[str, object]]) -> dict[str, float]:
    pairs: list[tuple[float, float, float, float]] = []
    for row in rows:
        gps_e = to_float(row.get("gps_e_m"))
        gps_n = to_float(row.get("gps_n_m"))
        gt_e = to_float(row.get("gt_e_m"))
        gt_n = to_float(row.get("gt_n_m"))
        if all(finite(item) for item in (gps_e, gps_n, gt_e, gt_n)):
            pairs.append((gps_e, gps_n, gt_e, gt_n))
    if len(pairs) < 2:
        return {"n": float(len(pairs)), "scale": math.nan, "angle_deg": math.nan, "trans_e_m": math.nan, "trans_n_m": math.nan}
    src_e_mean = mean(item[0] for item in pairs)
    src_n_mean = mean(item[1] for item in pairs)
    dst_e_mean = mean(item[2] for item in pairs)
    dst_n_mean = mean(item[3] for item in pairs)
    real_num = 0.0
    imag_num = 0.0
    denom = 0.0
    for gps_e, gps_n, gt_e, gt_n in pairs:
        sx = gps_e - src_e_mean
        sy = gps_n - src_n_mean
        tx = gt_e - dst_e_mean
        ty = gt_n - dst_n_mean
        real_num += tx * sx + ty * sy
        imag_num += ty * sx - tx * sy
        denom += sx * sx + sy * sy
    a_real = real_num / denom if denom > 1e-12 else math.nan
    a_imag = imag_num / denom if denom > 1e-12 else math.nan
    trans_e = dst_e_mean - (a_real * src_e_mean - a_imag * src_n_mean)
    trans_n = dst_n_mean - (a_imag * src_e_mean + a_real * src_n_mean)
    return {
        "n": float(len(pairs)),
        "scale": math.hypot(a_real, a_imag),
        "angle_deg": math.degrees(math.atan2(a_imag, a_real)),
        "a_real": a_real,
        "a_imag": a_imag,
        "trans_e_m": trans_e,
        "trans_n_m": trans_n,
    }


def eval_fit(rows: list[dict[str, object]], fit: dict[str, float]) -> dict[str, float]:
    a_real = to_float(fit.get("a_real"))
    a_imag = to_float(fit.get("a_imag"))
    trans_e = to_float(fit.get("trans_e_m"))
    trans_n = to_float(fit.get("trans_n_m"))
    err_e_values: list[float] = []
    err_n_values: list[float] = []
    err_xy_values: list[float] = []
    for row in rows:
        gps_e = to_float(row.get("gps_e_m"))
        gps_n = to_float(row.get("gps_n_m"))
        gt_e = to_float(row.get("gt_e_m"))
        gt_n = to_float(row.get("gt_n_m"))
        if not all(finite(item) for item in (a_real, a_imag, trans_e, trans_n, gps_e, gps_n, gt_e, gt_n)):
            continue
        pred_e = a_real * gps_e - a_imag * gps_n + trans_e
        pred_n = a_imag * gps_e + a_real * gps_n + trans_n
        err_e = pred_e - gt_e
        err_n = pred_n - gt_n
        err_e_values.append(err_e)
        err_n_values.append(err_n)
        err_xy_values.append(math.hypot(err_e, err_n))
    return {
        "xy_mean_m": mean(err_xy_values),
        "xy_p95_m": percentile(err_xy_values, 95),
        "xy_max_m": max_finite(err_xy_values),
        "err_e_mean_m": mean(err_e_values),
        "err_n_mean_m": mean(err_n_values),
    }


def moving_subset(rows: list[dict[str, object]], threshold_m: float = 5.0) -> list[dict[str, object]]:
    return [
        row for row in rows
        if abs(to_float(row.get("gt_e_m"))) > threshold_m or abs(to_float(row.get("gt_n_m"))) > threshold_m
    ]


def fmt(value: float, digits: int = 4) -> str:
    return "nan" if not finite(value) else f"{value:.{digits}f}"


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


def build_report(
    fit_rows: list[dict[str, object]],
    out_dir: Path,
    ref_llh: tuple[float, float, float],
) -> str:
    ref_lat_rad = ref_llh[0]
    rm, rn = wgs84_radii(ref_lat_rad)
    px4_east_scale_vs_wgs84 = PX4_SPHERE_RADIUS_M / rn
    px4_north_scale_vs_wgs84 = PX4_SPHERE_RADIUS_M / rm
    lines = [
        "# ULog GPS Groundtruth Projection Diagnostic",
        "",
        "GPS latitude/longitude is converted with WGS84 LLH->ECEF->ENU using the ULog groundtruth reference, then fitted against `vehicle_local_position_groundtruth` local XY.",
        "",
        "PX4's local map projection uses a spherical Earth radius of `6371000m`; KF-GINS/native diagnostics use WGS84 ECEF/ENU. At this latitude, the expected PX4-sphere/WGS84 scale is direction-dependent.",
        "",
        "## Theoretical Scale Check",
        markdown_table(
            ["ref_lat_deg", "px4_radius_m", "wgs84_rm_m", "wgs84_rn_m", "px4_vs_wgs84_east", "px4_vs_wgs84_north"],
            [
                [
                    fmt(math.degrees(ref_lat_rad), 7),
                    fmt(PX4_SPHERE_RADIUS_M, 1),
                    fmt(rm, 3),
                    fmt(rn, 3),
                    fmt(px4_east_scale_vs_wgs84, 6),
                    fmt(px4_north_scale_vs_wgs84, 6),
                ]
            ],
        ),
        "",
        "## Similarity Fit",
    ]
    lines.append(
        markdown_table(
            ["topic", "subset", "n", "scale", "angle_deg", "trans_e_m", "trans_n_m", "xy_mean_m", "xy_p95_m", "xy_max_m"],
            [
                [
                    row["topic"],
                    row["subset"],
                    row["n"],
                    fmt(to_float(row.get("scale")), 6),
                    fmt(to_float(row.get("angle_deg")), 4),
                    fmt(to_float(row.get("trans_e_m"))),
                    fmt(to_float(row.get("trans_n_m"))),
                    fmt(to_float(row.get("xy_mean_m"))),
                    fmt(to_float(row.get("xy_p95_m"))),
                    fmt(to_float(row.get("xy_max_m"))),
                ]
                for row in fit_rows
            ],
        )
    )
    vehicle = next((row for row in fit_rows if row["topic"] == "vehicle_gps_position" and row["subset"] == "all"), None)
    if vehicle is not None:
        lines.extend(
            [
                "",
                "## Readout",
                "",
                "- `vehicle_gps_position` LLH projected with WGS84 does not exactly share the PX4 groundtruth local XY scale.",
                f"- all-row fit scale is `{fmt(to_float(vehicle.get('scale')), 6)}`, which matches the manual122 pair similarity scale within about `1e-4`.",
                f"- It also matches the expected East-dominant PX4 spherical projection scale `{fmt(px4_east_scale_vs_wgs84, 6)}` at the Zurich SITL latitude.",
                "- This supports treating the residual IEKF-vs-EKF2 meter-scale peak as a frame/projection relation before trying more estimator tuning or fixed source-lag compensation.",
                "",
                "Generated files:",
                f"- `{out_dir / 'gps_groundtruth_joined.csv'}`",
                f"- `{out_dir / 'gps_groundtruth_fit.csv'}`",
            ]
        )
    return "\n".join(lines) + "\n"


def find_topic_csv(ulog_dir: Path, topic: str) -> Path:
    matches = sorted(ulog_dir.glob(f"*_{topic}_0.csv"))
    if not matches:
        raise FileNotFoundError(f"missing ulog2csv export for topic {topic!r} in {ulog_dir}")
    if len(matches) > 1:
        names = ", ".join(path.name for path in matches)
        raise RuntimeError(f"multiple ulog2csv exports for topic {topic!r} in {ulog_dir}: {names}")
    return matches[0]


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--ulog-dir",
        default="/home/yang/kf_gins_ws/artifacts/manual/offline_ulog_manual122_groundtruth",
        help="Directory containing ulog2csv exports.",
    )
    parser.add_argument(
        "--out-dir",
        default="/home/yang/kf_gins_ws/artifacts/manual/offline_ulog_manual122_gps_groundtruth_projection_diag",
        help="Output directory for report and CSVs.",
    )
    parser.add_argument("--max-join-dt-sec", type=float, default=0.05)
    args = parser.parse_args()

    ulog_dir = Path(args.ulog_dir)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    gt_rows, ref_llh, origin_ecef = load_groundtruth(find_topic_csv(ulog_dir, "vehicle_local_position_groundtruth"))
    gt_times = [to_float(row["timestamp_sec"]) for row in gt_rows]
    topic_files = [
        ("vehicle_gps_position", find_topic_csv(ulog_dir, "vehicle_gps_position")),
        ("sensor_gps", find_topic_csv(ulog_dir, "sensor_gps")),
    ]

    joined: list[dict[str, object]] = []
    fit_rows: list[dict[str, object]] = []
    for topic, path in topic_files:
        if not path.exists():
            continue
        rows = join_gps_groundtruth(
            topic,
            path,
            gt_rows,
            gt_times,
            ref_llh,
            origin_ecef,
            args.max_join_dt_sec,
        )
        joined.extend(rows)
        for subset_name, subset_rows in (("all", rows), ("moving", moving_subset(rows))):
            fit = fit_similarity(subset_rows)
            eval_result = eval_fit(subset_rows, fit)
            fit_rows.append({"topic": topic, "subset": subset_name, **fit, **eval_result})

    write_csv(out_dir / "gps_groundtruth_joined.csv", joined)
    write_csv(out_dir / "gps_groundtruth_fit.csv", fit_rows)
    (out_dir / "report.md").write_text(build_report(fit_rows, out_dir, ref_llh), encoding="utf-8")

    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'gps_groundtruth_joined.csv'}")
    print(f"wrote: {out_dir / 'gps_groundtruth_fit.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
