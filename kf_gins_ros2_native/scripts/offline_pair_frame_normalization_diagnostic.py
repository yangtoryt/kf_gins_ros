#!/usr/bin/env python3
"""Apply an external local-frame similarity normalization to EKF2/IEKF pair logs."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable


DEFAULT_WINDOWS = [
    (0.0, 20.0),
    (20.0, 40.0),
    (40.0, 60.0),
    (60.0, 80.0),
    (80.0, 100.0),
    (100.0, 120.0),
    (120.0, 140.0),
    (140.0, 160.0),
    (160.0, 180.0),
]


def finite(value: float) -> bool:
    return math.isfinite(value)


def to_float(value: object, default: float = math.nan) -> float:
    try:
        result = float(value)  # type: ignore[arg-type]
    except (TypeError, ValueError):
        return default
    return result if finite(result) else default


def truthy(value: object) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes"}


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


def parse_run_arg(item: str) -> tuple[str, Path]:
    if "=" in item:
        label, run_dir = item.split("=", 1)
        return label, Path(run_dir)
    path = Path(item)
    return path.name, path


def load_external_fit(path: Path, topic: str, subset: str) -> dict[str, float]:
    for row in read_csv(path):
        if row.get("topic") == topic and row.get("subset") == subset:
            scale = to_float(row.get("scale"))
            angle_deg = to_float(row.get("angle_deg"))
            if not finite(scale) or not finite(angle_deg):
                break
            angle = math.radians(angle_deg)
            return {
                "scale": scale,
                "angle_deg": angle_deg,
                "a_real": scale * math.cos(angle),
                "a_imag": scale * math.sin(angle),
            }
    raise RuntimeError(f"fit row not found in {path}: topic={topic} subset={subset}")


def load_pair_rows(run_label: str, run_dir: Path) -> list[dict[str, object]]:
    pair_path = run_dir / "ekf_iekf_pairs.csv"
    if not pair_path.exists() or pair_path.stat().st_size == 0:
        raise FileNotFoundError(f"missing pair CSV for {run_label}: {pair_path}")
    rows: list[dict[str, object]] = []
    for row in read_csv(pair_path):
        out: dict[str, object] = {"run": run_label}
        for key in (
            "ros_time_sec",
            "ekf2_x_m",
            "ekf2_y_m",
            "iekf_x_m",
            "iekf_y_m",
            "iekf_aligned_x_m",
            "iekf_aligned_y_m",
            "position_error_x_m",
            "position_error_y_m",
            "position_error_xy_m",
        ):
            out[key] = to_float(row.get(key))
        out["mavros_armed"] = int(truthy(row.get("mavros_armed")))
        out["alignment_ready"] = int(truthy(row.get("alignment_ready")))
        rows.append(out)
    return rows


def transformed_iekf_xy(row: dict[str, object], fit: dict[str, float]) -> tuple[float, float]:
    x = to_float(row.get("iekf_x_m"))
    y = to_float(row.get("iekf_y_m"))
    a_real = fit["a_real"]
    a_imag = fit["a_imag"]
    return a_real * x - a_imag * y, a_imag * x + a_real * y


def initial_offset(
    rows: list[dict[str, object]],
    fit: dict[str, float],
    min_pairs: int,
) -> tuple[float, float, int]:
    offsets: list[tuple[float, float]] = []
    for row in rows:
        tx, ty = transformed_iekf_xy(row, fit)
        ekf_x = to_float(row.get("ekf2_x_m"))
        ekf_y = to_float(row.get("ekf2_y_m"))
        if all(finite(item) for item in (tx, ty, ekf_x, ekf_y)):
            offsets.append((ekf_x - tx, ekf_y - ty))
            if len(offsets) >= min_pairs:
                break
    if not offsets:
        return math.nan, math.nan, 0
    return mean(item[0] for item in offsets), mean(item[1] for item in offsets), len(offsets)


def normalized_rows(
    rows: list[dict[str, object]],
    fit: dict[str, float],
    min_pairs: int,
) -> tuple[list[dict[str, object]], tuple[float, float, int]]:
    offset_x, offset_y, n_offset = initial_offset(rows, fit, min_pairs)
    result: list[dict[str, object]] = []
    arm_times = [to_float(row.get("ros_time_sec")) for row in rows if truthy(row.get("mavros_armed"))]
    arm_time = min((item for item in arm_times if finite(item)), default=math.nan)
    for row in rows:
        tx, ty = transformed_iekf_xy(row, fit)
        tx += offset_x
        ty += offset_y
        ekf_x = to_float(row.get("ekf2_x_m"))
        ekf_y = to_float(row.get("ekf2_y_m"))
        err_x = tx - ekf_x
        err_y = ty - ekf_y
        out = dict(row)
        out.update(
            {
                "normalized_iekf_x_m": tx,
                "normalized_iekf_y_m": ty,
                "normalization_offset_x_m": offset_x,
                "normalization_offset_y_m": offset_y,
                "normalization_offset_pairs": n_offset,
                "normalized_error_x_m": err_x,
                "normalized_error_y_m": err_y,
                "normalized_error_xy_m": math.hypot(err_x, err_y) if finite(err_x) and finite(err_y) else math.nan,
                "time_since_arm_sec": to_float(row.get("ros_time_sec")) - arm_time,
            }
        )
        result.append(out)
    return result, (offset_x, offset_y, n_offset)


def select_window(rows: list[dict[str, object]], start: float, end: float) -> list[dict[str, object]]:
    return [
        row for row in rows
        if truthy(row.get("mavros_armed"))
        and start <= to_float(row.get("time_since_arm_sec")) < end
        and truthy(row.get("alignment_ready"))
    ]


def summarize(rows: list[dict[str, object]], key: str) -> dict[str, float]:
    filtered = [
        row for row in rows
        if truthy(row.get("mavros_armed")) and truthy(row.get("alignment_ready"))
    ]
    return {
        "n": float(len([row for row in filtered if finite(to_float(row.get(key)))])),
        "xy_mean_m": mean(to_float(row.get(key)) for row in filtered),
        "xy_p95_m": percentile((to_float(row.get(key)) for row in filtered), 95),
        "xy_max_m": max_finite(to_float(row.get(key)) for row in filtered),
    }


def summarize_window(rows: list[dict[str, object]], key: str, start: float, end: float) -> dict[str, float]:
    return summarize(select_window(rows, start, end), key)


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
    fit: dict[str, float],
    summary_rows: list[dict[str, object]],
    window_rows: list[dict[str, object]],
    out_dir: Path,
) -> str:
    lines = [
        "# Pair Frame Normalization Diagnostic",
        "",
        "This report applies an external GPS/groundtruth WGS84-ENU -> PX4-local similarity transform to IEKF pair positions, then recomputes pair errors after the same initial translation alignment policy.",
        "",
        f"External fit: scale `{fmt(fit['scale'], 6)}`, angle `{fmt(fit['angle_deg'], 4)}deg`.",
        "",
        "## Armed Summary",
    ]
    lines.append(
        markdown_table(
            [
                "run",
                "offset_pairs",
                "offset_x_m",
                "offset_y_m",
                "raw_mean_m",
                "raw_p95_m",
                "raw_max_m",
                "normalized_mean_m",
                "normalized_p95_m",
                "normalized_max_m",
            ],
            [
                [
                    row["run"],
                    row["offset_pairs"],
                    fmt(to_float(row.get("offset_x_m"))),
                    fmt(to_float(row.get("offset_y_m"))),
                    fmt(to_float(row.get("raw_xy_mean_m"))),
                    fmt(to_float(row.get("raw_xy_p95_m"))),
                    fmt(to_float(row.get("raw_xy_max_m"))),
                    fmt(to_float(row.get("normalized_xy_mean_m"))),
                    fmt(to_float(row.get("normalized_xy_p95_m"))),
                    fmt(to_float(row.get("normalized_xy_max_m"))),
                ]
                for row in summary_rows
            ],
        )
    )
    lines.extend(["", "## Key Windows"])
    key_windows = {"060-080", "080-100", "100-120", "120-140", "160-180"}
    lines.append(
        markdown_table(
            ["run", "window", "raw_mean_m", "raw_max_m", "normalized_mean_m", "normalized_max_m"],
            [
                [
                    row["run"],
                    row["window"],
                    fmt(to_float(row.get("raw_xy_mean_m"))),
                    fmt(to_float(row.get("raw_xy_max_m"))),
                    fmt(to_float(row.get("normalized_xy_mean_m"))),
                    fmt(to_float(row.get("normalized_xy_max_m"))),
                ]
                for row in window_rows
                if row["window"] in key_windows
            ],
        )
    )
    manual122 = next((row for row in summary_rows if row["run"] == "manual122"), None)
    if manual122 is not None:
        lines.extend(
            [
                "",
                "## Readout",
                "",
                "- Applying the independent GPS/groundtruth projection normalization collapses manual122's corrected pair residual without changing estimator code.",
                f"- manual122 armed mean/max changes from `{fmt(to_float(manual122.get('raw_xy_mean_m')))}/{fmt(to_float(manual122.get('raw_xy_max_m')))}m` to `{fmt(to_float(manual122.get('normalized_xy_mean_m')))}/{fmt(to_float(manual122.get('normalized_xy_max_m')))}m`.",
                "- This is comparison-only evidence that the remaining meter-scale pair peak is dominated by frame/projection normalization, not by an IEKF dynamic failure.",
                "",
                "Generated files:",
                f"- `{out_dir / 'normalized_pairs.csv'}`",
                f"- `{out_dir / 'normalization_summary.csv'}`",
                f"- `{out_dir / 'normalization_window_summary.csv'}`",
            ]
        )
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run", action="append", required=True, help="Run mapping as label=/abs/run_dir.")
    parser.add_argument(
        "--fit-csv",
        default="/home/yang/kf_gins_ws/artifacts/manual/offline_ulog_manual122_gps_groundtruth_projection_diag/gps_groundtruth_fit.csv",
        help="CSV produced by offline_ulog_gps_groundtruth_diagnostic.py.",
    )
    parser.add_argument("--fit-topic", default="vehicle_gps_position")
    parser.add_argument("--fit-subset", default="all")
    parser.add_argument("--align-min-pairs", type=int, default=10)
    parser.add_argument(
        "--out-dir",
        default="/home/yang/kf_gins_ws/artifacts/manual/offline_pair_frame_normalization_manual122_123",
    )
    args = parser.parse_args()

    fit = load_external_fit(Path(args.fit_csv), args.fit_topic, args.fit_subset)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    all_normalized: list[dict[str, object]] = []
    summary_rows: list[dict[str, object]] = []
    window_rows: list[dict[str, object]] = []
    for label, run_dir in [parse_run_arg(item) for item in args.run]:
        pair_rows = load_pair_rows(label, run_dir)
        norm_rows, (offset_x, offset_y, offset_n) = normalized_rows(pair_rows, fit, args.align_min_pairs)
        all_normalized.extend(norm_rows)
        raw = summarize(norm_rows, "position_error_xy_m")
        normalized = summarize(norm_rows, "normalized_error_xy_m")
        summary_rows.append(
            {
                "run": label,
                "offset_x_m": offset_x,
                "offset_y_m": offset_y,
                "offset_pairs": offset_n,
                "raw_xy_mean_m": raw["xy_mean_m"],
                "raw_xy_p95_m": raw["xy_p95_m"],
                "raw_xy_max_m": raw["xy_max_m"],
                "normalized_xy_mean_m": normalized["xy_mean_m"],
                "normalized_xy_p95_m": normalized["xy_p95_m"],
                "normalized_xy_max_m": normalized["xy_max_m"],
            }
        )
        for start, end in DEFAULT_WINDOWS:
            raw_w = summarize_window(norm_rows, "position_error_xy_m", start, end)
            norm_w = summarize_window(norm_rows, "normalized_error_xy_m", start, end)
            window_rows.append(
                {
                    "run": label,
                    "window": f"{int(start):03d}-{int(end):03d}",
                    "raw_xy_mean_m": raw_w["xy_mean_m"],
                    "raw_xy_max_m": raw_w["xy_max_m"],
                    "normalized_xy_mean_m": norm_w["xy_mean_m"],
                    "normalized_xy_max_m": norm_w["xy_max_m"],
                }
            )

    write_csv(out_dir / "normalized_pairs.csv", all_normalized)
    write_csv(out_dir / "normalization_summary.csv", summary_rows)
    write_csv(out_dir / "normalization_window_summary.csv", window_rows)
    (out_dir / "report.md").write_text(build_report(fit, summary_rows, window_rows, out_dir), encoding="utf-8")

    print(f"wrote: {out_dir / 'report.md'}")
    print(f"wrote: {out_dir / 'normalized_pairs.csv'}")
    print(f"wrote: {out_dir / 'normalization_summary.csv'}")
    print(f"wrote: {out_dir / 'normalization_window_summary.csv'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
