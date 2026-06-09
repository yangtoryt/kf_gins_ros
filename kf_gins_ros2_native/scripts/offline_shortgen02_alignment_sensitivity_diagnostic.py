#!/usr/bin/env python3
"""Measure how shortgen02 groundtruth scores depend on alignment window."""

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


ALIGN_WINDOWS = [
    ("early_0_20", 0.0, 20.0),
    ("main_40_180", 40.0, 180.0),
    ("target_120_180", 120.0, 180.0),
    ("late_171_178p5", 171.0, 178.5),
]
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


def is_armed(row: dict[str, object]) -> bool:
    return to_float(row.get("mavros_armed"), 0.0) > 0.5


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return finite(t) and start <= t < end


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
    if not rows or not finite(t):
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


def alignment_offsets(rows: list[dict[str, object]], start: float, end: float) -> dict[str, object]:
    subset = [row for row in rows if is_armed(row) and in_window(row, start, end)]
    ekf2_dx = [to_float(row.get("gt_x_m")) - to_float(row.get("ekf2_x_m")) for row in subset]
    ekf2_dy = [to_float(row.get("gt_y_m")) - to_float(row.get("ekf2_y_m")) for row in subset]
    iekf_dx = [to_float(row.get("gt_x_m")) - to_float(row.get("iekf_px4_x_m")) for row in subset]
    iekf_dy = [to_float(row.get("gt_y_m")) - to_float(row.get("iekf_px4_y_m")) for row in subset]
    ekf2_x = mean(ekf2_dx)
    ekf2_y = mean(ekf2_dy)
    iekf_x = mean(iekf_dx)
    iekf_y = mean(iekf_dy)
    return {
        "rows": len(subset),
        "ekf2_offset_x_m": ekf2_x,
        "ekf2_offset_y_m": ekf2_y,
        "ekf2_offset_h_m": norm2(ekf2_x, ekf2_y),
        "iekf_offset_x_m": iekf_x,
        "iekf_offset_y_m": iekf_y,
        "iekf_offset_h_m": norm2(iekf_x, iekf_y),
    }


def error_h(row: dict[str, object], offsets: dict[str, object], estimator: str) -> float:
    gt_x = to_float(row.get("gt_x_m"))
    gt_y = to_float(row.get("gt_y_m"))
    if estimator == "ekf2":
        x = to_float(row.get("ekf2_x_m")) + to_float(offsets.get("ekf2_offset_x_m"))
        y = to_float(row.get("ekf2_y_m")) + to_float(offsets.get("ekf2_offset_y_m"))
    else:
        x = to_float(row.get("iekf_px4_x_m")) + to_float(offsets.get("iekf_offset_x_m"))
        y = to_float(row.get("iekf_px4_y_m")) + to_float(offsets.get("iekf_offset_y_m"))
    return norm2(x - gt_x, y - gt_y)


def load_high_rows(run_dir: Path, joined_rows: list[dict[str, object]], joined_times: list[float]) -> list[dict[str, object]]:
    target_rows = read_csv(run_dir / "shortgen02_measurement_geometry_diag" / "target_update_geometry_rows.csv")
    out: list[dict[str, object]] = []
    for row in target_rows:
        joined = nearest(joined_rows, joined_times, to_float(row.get("ros_time_sec")), 0.15)
        if joined is None:
            continue
        out.append(joined)
    return out


def build_offset_rows(rows: list[dict[str, object]]) -> list[dict[str, object]]:
    raw: list[dict[str, object]] = []
    for label, start, end in ALIGN_WINDOWS:
        offsets = alignment_offsets(rows, start, end)
        raw.append({"align_window": label, "align_start_sec": start, "align_end_sec": end, **offsets})
    early = next(row for row in raw if row["align_window"] == "early_0_20")
    for row in raw:
        row["ekf2_offset_delta_from_early_h_m"] = norm2(
            to_float(row.get("ekf2_offset_x_m")) - to_float(early.get("ekf2_offset_x_m")),
            to_float(row.get("ekf2_offset_y_m")) - to_float(early.get("ekf2_offset_y_m")),
        )
        row["iekf_offset_delta_from_early_h_m"] = norm2(
            to_float(row.get("iekf_offset_x_m")) - to_float(early.get("iekf_offset_x_m")),
            to_float(row.get("iekf_offset_y_m")) - to_float(early.get("iekf_offset_y_m")),
        )
    return raw


def summarize_subset(
    subset: list[dict[str, object]],
    offsets: dict[str, object],
    align_window: str,
    eval_window: str,
) -> list[dict[str, object]]:
    ekf2_errors = [error_h(row, offsets, "ekf2") for row in subset]
    iekf_errors = [error_h(row, offsets, "iekf") for row in subset]
    return [
        {
            "align_window": align_window,
            "eval_window": eval_window,
            "estimator": "ekf2",
            "rows": len(ekf2_errors),
            "mean_m": mean(ekf2_errors),
            "p95_m": percentile(ekf2_errors, 95.0),
            "max_m": max((item for item in ekf2_errors if finite(item)), default=math.nan),
            "frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in ekf2_errors),
        },
        {
            "align_window": align_window,
            "eval_window": eval_window,
            "estimator": "iekf",
            "rows": len(iekf_errors),
            "mean_m": mean(iekf_errors),
            "p95_m": percentile(iekf_errors, 95.0),
            "max_m": max((item for item in iekf_errors if finite(item)), default=math.nan),
            "frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in iekf_errors),
        },
        {
            "align_window": align_window,
            "eval_window": eval_window,
            "estimator": "iekf_minus_ekf2",
            "rows": len(iekf_errors),
            "mean_m": mean(iekf_errors) - mean(ekf2_errors),
            "p95_m": math.nan,
            "max_m": math.nan,
            "frac_over_0p3": math.nan,
        },
    ]


def build_error_rows(
    rows: list[dict[str, object]],
    high_rows: list[dict[str, object]],
    offset_rows: list[dict[str, object]],
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for offsets in offset_rows:
        label = str(offsets["align_window"])
        for eval_label, start, end in EVAL_WINDOWS:
            subset = [row for row in rows if is_armed(row) and in_window(row, start, end)]
            out.extend(summarize_subset(subset, offsets, label, eval_label))
        out.extend(summarize_subset(high_rows, offsets, label, "target_high_rows"))
    return out


def row_lookup(error_rows: list[dict[str, object]], align: str, window: str, estimator: str) -> dict[str, object]:
    return next(
        row for row in error_rows
        if row.get("align_window") == align and row.get("eval_window") == window and row.get("estimator") == estimator
    )


def write_report(out_dir: Path, offset_rows: list[dict[str, object]], error_rows: list[dict[str, object]]) -> None:
    offset_table = [
        [
            row["align_window"],
            row["rows"],
            fmt(row["ekf2_offset_x_m"]),
            fmt(row["ekf2_offset_y_m"]),
            fmt(row["ekf2_offset_delta_from_early_h_m"]),
            fmt(row["iekf_offset_x_m"]),
            fmt(row["iekf_offset_y_m"]),
            fmt(row["iekf_offset_delta_from_early_h_m"]),
        ]
        for row in offset_rows
    ]
    selected: list[list[object]] = []
    for align in ("early_0_20", "main_40_180", "target_120_180", "late_171_178p5"):
        for window in ("120-180", "target_high_rows"):
            ekf2 = row_lookup(error_rows, align, window, "ekf2")
            iekf = row_lookup(error_rows, align, window, "iekf")
            delta = row_lookup(error_rows, align, window, "iekf_minus_ekf2")
            selected.append([
                align,
                window,
                ekf2["rows"],
                fmt(ekf2["mean_m"]),
                fmt(iekf["mean_m"]),
                fmt(delta["mean_m"]),
                fmt(ekf2["p95_m"]),
                fmt(iekf["p95_m"]),
            ])

    early_high_iekf = row_lookup(error_rows, "early_0_20", "target_high_rows", "iekf")
    early_high_ekf2 = row_lookup(error_rows, "early_0_20", "target_high_rows", "ekf2")
    target_high_iekf = row_lookup(error_rows, "target_120_180", "target_high_rows", "iekf")
    target_high_ekf2 = row_lookup(error_rows, "target_120_180", "target_high_rows", "ekf2")
    early_offsets = next(row for row in offset_rows if row["align_window"] == "early_0_20")
    target_offsets = next(row for row in offset_rows if row["align_window"] == "target_120_180")
    iekf_offset_shift = to_float(target_offsets.get("iekf_offset_delta_from_early_h_m"))
    ekf2_offset_shift = to_float(target_offsets.get("ekf2_offset_delta_from_early_h_m"))

    lines = [
        "# Shortgen02 Alignment Sensitivity Diagnostic",
        "",
        "This diagnostic recomputes EKF2 and projection-normalized IEKF groundtruth errors using different translation-alignment windows while keeping the same run and same raw joined rows.",
        "",
        "## Alignment Offsets",
        "",
        markdown_table(
            [
                "align_window",
                "rows",
                "ekf2_dx",
                "ekf2_dy",
                "ekf2_shift_from_early",
                "iekf_dx",
                "iekf_dy",
                "iekf_shift_from_early",
            ],
            offset_table,
        ),
        "",
        "## Selected Error Sensitivity",
        "",
        markdown_table(
            ["align_window", "eval_window", "rows", "ekf2_mean", "iekf_mean", "iekf_minus_ekf2", "ekf2_p95", "iekf_p95"],
            selected,
        ),
        "",
        "## Interpretation",
        "",
        f"- With the original early `0-20 s` alignment, target high rows are EKF2 `{fmt(early_high_ekf2.get('mean_m'))}` m vs IEKF `{fmt(early_high_iekf.get('mean_m'))}` m.",
        f"- With `120-180 s` local alignment, the same high rows become EKF2 `{fmt(target_high_ekf2.get('mean_m'))}` m vs IEKF `{fmt(target_high_iekf.get('mean_m'))}` m.",
        f"- The IEKF alignment offset shifts `{fmt(iekf_offset_shift)}` m from early to `120-180 s`; EKF2 shifts only `{fmt(ekf2_offset_shift)}` m.",
        "- This points to a frame-origin/projection-normalization offset in the IEKF/GNSS path as the dominant target-row effect. It does not by itself prove a keeper mechanism, but it does explain why gain/threshold tuning could not close the gap.",
        "",
        "Generated files:",
        f"- `{out_dir / 'alignment_offset_rows.csv'}`",
        f"- `{out_dir / 'alignment_error_summary.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out-dir")
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir) if args.out_dir else run_dir / "shortgen02_alignment_sensitivity_diag"
    out_dir.mkdir(parents=True, exist_ok=True)

    joined_rows, joined_times = load_joined(run_dir)
    high_rows = load_high_rows(run_dir, joined_rows, joined_times)
    offset_rows = build_offset_rows(joined_rows)
    error_rows = build_error_rows(joined_rows, high_rows, offset_rows)

    write_csv(out_dir / "alignment_offset_rows.csv", offset_rows)
    write_csv(out_dir / "alignment_error_summary.csv", error_rows)
    write_report(out_dir, offset_rows, error_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
