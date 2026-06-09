#!/usr/bin/env python3
"""Audit GNSS-source-age projection gates with a stamp-lag guard.

This is offline-only.  Gate features are online-visible; groundtruth and
projection-mode coordinates are used only to score counterfactual output
projection choices.
"""

from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from typing import Iterable


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_INPUT = (
    BASE
    / "phs5_cross_route_accbias_selector_seg10_plus_online_alpha12_plus_diagvariants_2026-05-11"
    / "cross_route_row_metrics.csv"
)
DEFAULT_OUT = BASE / "phs5_gnss_source_age_lag_guard_selector_diagvariants_2026-05-11"

SEGMENT_SECS = [5.0, 10.0, 15.0, 20.0]
AGE_THRESHOLDS = [0.195, 0.200, 0.205, 0.210, 0.215, 0.220, 0.225, 0.230]
LAG_MINS: list[float | None] = [None, -0.020, 0.000, 0.001, 0.005]
ALPHAS = [1.10, 1.15, 1.20, 1.25, 1.30]
WINDOWS = {
    "main_40_180": (40.0, 180.0),
    "140_160": (140.0, 160.0),
    "160_180": (160.0, 180.0),
}


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


def fmt(value: object, digits: int = 4) -> str:
    val = to_float(value)
    return f"{val:.{digits}f}" if math.isfinite(val) else "nan"


def mean(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return sum(vals) / len(vals) if vals else math.nan


def rmse(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


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


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


def run_sets(rows: list[dict[str, str]]) -> tuple[list[str], list[str], list[str]]:
    by_group: dict[str, set[str]] = defaultdict(set)
    for row in rows:
        run = str(row.get("run", "")).strip()
        group = str(row.get("group", "")).strip()
        if run and group:
            by_group[group].add(run)
    return (
        sorted(by_group.get("positive", set())),
        sorted(by_group.get("target", set())),
        sorted(by_group.get("diagnostic", set())),
    )


def in_window(row: dict[str, str], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def segment_index(row: dict[str, str], segment_sec: float) -> int:
    return int(math.floor((to_float(row.get("time_since_arm_sec")) - 40.0) / segment_sec))


def active_map(
    rows: list[dict[str, str]],
    segment_sec: float,
    age_threshold_sec: float,
    lag_min_sec: float | None,
) -> dict[tuple[str, int], bool]:
    grouped: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if in_window(row, 40.0, 180.0):
            grouped[(str(row.get("run")), segment_index(row, segment_sec))].append(row)
    out: dict[tuple[str, int], bool] = {}
    for key, segment_rows in grouped.items():
        age_mean = mean(to_float(row.get("gnss_source_age_sec")) for row in segment_rows)
        lag_mean = mean(to_float(row.get("stamp_lag_sec")) for row in segment_rows)
        active = finite(age_mean) and age_mean <= age_threshold_sec
        if lag_min_sec is not None:
            active = active and finite(lag_mean) and lag_mean >= lag_min_sec
        out[key] = active
    return out


def candidate_error(
    row: dict[str, str],
    alpha: float,
    segment_sec: float,
    active_by_segment: dict[tuple[str, int], bool],
) -> tuple[float, bool]:
    key = (str(row.get("run")), segment_index(row, segment_sec))
    active = active_by_segment.get(key, False)
    raw_x = to_float(row.get("raw_iekf_x_m"))
    raw_y = to_float(row.get("raw_iekf_y_m"))
    px4_x = to_float(row.get("px4_iekf_x_m"))
    px4_y = to_float(row.get("px4_iekf_y_m"))
    gt_x = to_float(row.get("gt_x_m"))
    gt_y = to_float(row.get("gt_y_m"))
    cand_x = raw_x + alpha * (px4_x - raw_x) if active else raw_x
    cand_y = raw_y + alpha * (px4_y - raw_y) if active else raw_y
    return math.hypot(cand_x - gt_x, cand_y - gt_y), active


def eval_window(
    rows: list[dict[str, str]],
    run: str,
    window: str,
    segment_sec: float,
    age_threshold_sec: float,
    lag_min_sec: float | None,
    alpha: float,
    active_by_segment: dict[tuple[str, int], bool] | None = None,
) -> dict[str, object]:
    start, end = WINDOWS[window]
    active_by_segment = active_by_segment or active_map(rows, segment_sec, age_threshold_sec, lag_min_sec)
    window_rows = [
        row for row in rows
        if row.get("run") == run and in_window(row, start, end)
    ]
    cand_errors: list[float] = []
    active_flags: list[float] = []
    for row in window_rows:
        err, active = candidate_error(row, alpha, segment_sec, active_by_segment)
        cand_errors.append(err)
        active_flags.append(1.0 if active else 0.0)
    raw = rmse(to_float(row.get("raw_iekf_error_m")) for row in window_rows)
    px4 = rmse(to_float(row.get("px4_iekf_error_m")) for row in window_rows)
    ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in window_rows)
    candidate = rmse(cand_errors)
    return {
        "run": run,
        "window": window,
        "rows": len(window_rows),
        "segment_sec": segment_sec,
        "age_threshold_sec": age_threshold_sec,
        "lag_min_sec": "" if lag_min_sec is None else lag_min_sec,
        "alpha": alpha,
        "active_frac": mean(active_flags),
        "age_mean_sec": mean(to_float(row.get("gnss_source_age_sec")) for row in window_rows),
        "lag_mean_sec": mean(to_float(row.get("stamp_lag_sec")) for row in window_rows),
        "raw_rmse_m": raw,
        "px4_rmse_m": px4,
        "candidate_rmse_m": candidate,
        "ekf2_rmse_m": ekf2,
        "candidate_minus_raw_m": candidate - raw,
        "candidate_minus_ekf2_m": candidate - ekf2,
    }


def score_candidate(
    rows: list[dict[str, str]],
    positive_runs: list[str],
    target_runs: list[str],
    diagnostic_runs: list[str],
    segment_sec: float,
    age_threshold_sec: float,
    lag_min_sec: float | None,
    alpha: float,
) -> dict[str, object]:
    active_by_segment = active_map(rows, segment_sec, age_threshold_sec, lag_min_sec)
    pos = [
        eval_window(rows, run, "main_40_180", segment_sec, age_threshold_sec, lag_min_sec, alpha, active_by_segment)
        for run in positive_runs
    ]
    targets = [
        eval_window(rows, run, window, segment_sec, age_threshold_sec, lag_min_sec, alpha, active_by_segment)
        for run in target_runs
        for window in ("140_160", "160_180")
    ]
    diagnostics = [
        eval_window(rows, run, "main_40_180", segment_sec, age_threshold_sec, lag_min_sec, alpha, active_by_segment)
        for run in diagnostic_runs
    ]
    shortgen11_diag = [
        eval_window(rows, run, window, segment_sec, age_threshold_sec, lag_min_sec, alpha, active_by_segment)
        for run in diagnostic_runs
        if run.startswith("shortgen11")
        for window in ("140_160", "160_180")
    ]
    protected = (
        max(to_float(row["candidate_minus_raw_m"]) for row in pos) <= 0.02
        and max(to_float(row["candidate_minus_ekf2_m"]) for row in pos) < 0.02
    )
    strict = protected and max(to_float(row["candidate_minus_ekf2_m"]) for row in targets) <= 0.02
    return {
        "segment_sec": segment_sec,
        "age_threshold_sec": age_threshold_sec,
        "lag_min_sec": "" if lag_min_sec is None else lag_min_sec,
        "alpha": alpha,
        "positive_max_main_regress_m": max(to_float(row["candidate_minus_raw_m"]) for row in pos),
        "positive_max_candidate_minus_ekf2_m": max(to_float(row["candidate_minus_ekf2_m"]) for row in pos),
        "positive_all_protected": int(protected),
        "target_worst_candidate_minus_ekf2_m": max(to_float(row["candidate_minus_ekf2_m"]) for row in targets),
        "strict_all_target_pass": int(strict),
        "diagnostic_max_main_regress_m": max(to_float(row["candidate_minus_raw_m"]) for row in diagnostics) if diagnostics else math.nan,
        "diagnostic_shortgen11_worst_candidate_minus_ekf2_m": max(
            to_float(row["candidate_minus_ekf2_m"]) for row in shortgen11_diag
        ) if shortgen11_diag else math.nan,
    }


def table_summary(rows: list[dict[str, object]], limit: int = 16) -> list[list[object]]:
    rows = sorted(
        rows,
        key=lambda row: (
            -to_float(row.get("strict_all_target_pass")),
            to_float(row.get("target_worst_candidate_minus_ekf2_m")),
            to_float(row.get("diagnostic_shortgen11_worst_candidate_minus_ekf2_m")),
            to_float(row.get("diagnostic_max_main_regress_m")),
            to_float(row.get("segment_sec")),
        ),
    )
    return [
        [
            fmt(row["segment_sec"], 1),
            fmt(row["age_threshold_sec"], 3),
            "none" if row["lag_min_sec"] == "" else fmt(row["lag_min_sec"], 3),
            fmt(row["alpha"], 2),
            fmt(row["positive_max_main_regress_m"]),
            fmt(row["target_worst_candidate_minus_ekf2_m"]),
            row["strict_all_target_pass"],
            fmt(row["diagnostic_max_main_regress_m"]),
            fmt(row["diagnostic_shortgen11_worst_candidate_minus_ekf2_m"]),
        ]
        for row in rows[:limit]
    ]


def table_windows(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["window"],
            fmt(row["age_mean_sec"]),
            fmt(row["lag_mean_sec"]),
            fmt(row["active_frac"], 3),
            fmt(row["raw_rmse_m"]),
            fmt(row["px4_rmse_m"]),
            fmt(row["candidate_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["candidate_minus_raw_m"]),
            fmt(row["candidate_minus_ekf2_m"]),
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    summary_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
) -> None:
    strict = [row for row in summary_rows if to_float(row.get("strict_all_target_pass")) > 0.5]
    selected = [
        row for row in summary_rows
        if fmt(row.get("segment_sec"), 1) == "10.0"
        and fmt(row.get("age_threshold_sec"), 3) == "0.205"
        and fmt(row.get("lag_min_sec"), 3) == "0.000"
        and fmt(row.get("alpha"), 2) == "1.20"
    ][0]
    lines = [
        "# PHS5 GNSS-source-age plus stamp-lag guard audit",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only. Candidate uses segment-smoothed `gnss_source_age_sec` with an optional segment-smoothed `stamp_lag_sec` guard; groundtruth is used only to score output projection counterfactuals.",
        "",
        "## Candidate Summary",
        "",
        markdown_table(
            [
                "seg",
                "age thr",
                "lag min",
                "alpha",
                "pos regress",
                "target worst-EKF2",
                "strict",
                "diag regress",
                "diag s11 worst-EKF2",
            ],
            table_summary(summary_rows),
        ),
        "",
        "## Selected Candidate Windows",
        "",
        "Selected stress candidate: 10 s segment mean `gnss_source_age_sec <= 0.205` AND `stamp_lag_sec >= 0.000`, alpha `1.20`.",
        "",
        markdown_table(
            [
                "run",
                "window",
                "age",
                "lag",
                "active",
                "raw",
                "PX4",
                "candidate",
                "EKF2",
                "cand-raw",
                "cand-EKF2",
            ],
            table_windows(selected_rows),
        ),
        "",
        "## Readout",
        "",
        f"- Strict formal pass candidates found: `{len(strict)}`.",
        f"- Selected stress candidate formal target worst candidate-EKF2: `{fmt(selected['target_worst_candidate_minus_ekf2_m'])}` m.",
        f"- Selected stress candidate diagnostic main-window max candidate-raw regression: `{fmt(selected['diagnostic_max_main_regress_m'])}` m.",
        f"- Selected stress candidate diagnostic shortgen11 worst candidate-EKF2: `{fmt(selected['diagnostic_shortgen11_worst_candidate_minus_ekf2_m'])}` m.",
        "- The stamp-lag guard suppresses the shortgen11 coretrace negative-lag 140-160 trigger, but that diagnostic run remains a miss if treated as a target because raw itself stays worse than EKF2 there.",
        "- Global-source projection diagnostics still regress and should remain outside the default projection mode; they are a source-frame warning, not evidence for enabling global projection.",
        "",
        "Generated files:",
        f"- `{out_dir / 'candidate_summary.csv'}`",
        f"- `{out_dir / 'selected_window_metrics.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows = read_csv(args.input_csv)
    positive_runs, target_runs, diagnostic_runs = run_sets(rows)

    summary_rows = [
        score_candidate(rows, positive_runs, target_runs, diagnostic_runs, segment_sec, age_thr, lag_min, alpha)
        for segment_sec in SEGMENT_SECS
        for age_thr in AGE_THRESHOLDS
        for lag_min in LAG_MINS
        for alpha in ALPHAS
    ]
    write_csv(args.out_dir / "candidate_summary.csv", summary_rows)

    selected_rows: list[dict[str, object]] = []
    for run in positive_runs:
        selected_rows.append(eval_window(rows, run, "main_40_180", 10.0, 0.205, 0.0, 1.20))
    for run in target_runs:
        selected_rows.append(eval_window(rows, run, "140_160", 10.0, 0.205, 0.0, 1.20))
        selected_rows.append(eval_window(rows, run, "160_180", 10.0, 0.205, 0.0, 1.20))
    for run in diagnostic_runs:
        selected_rows.append(eval_window(rows, run, "main_40_180", 10.0, 0.205, 0.0, 1.20))
        if run.startswith("shortgen11"):
            selected_rows.append(eval_window(rows, run, "140_160", 10.0, 0.205, 0.0, 1.20))
            selected_rows.append(eval_window(rows, run, "160_180", 10.0, 0.205, 0.0, 1.20))
    write_csv(args.out_dir / "selected_window_metrics.csv", selected_rows)
    write_report(args.out_dir, summary_rows, selected_rows)
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
