#!/usr/bin/env python3
"""Sweep super-PX4 projection alpha under the accbias-history selector."""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable

import pandas as pd


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_INPUT = BASE / "phs5_cross_route_accbias_selector_2026-05-11" / "cross_route_row_metrics.csv"
DEFAULT_OUT = BASE / "phs5_accbias_projection_alpha_sweep_2026-05-11"

POSITIVE_RUNS = [
    "shortgen01_phs5c",
    "shortgen02_phs5b",
    "shortgen03_phs5a",
    "shortgen04_hld1a_phs5",
]
TARGET_RUNS = [
    "shortgen11_repeat2",
    "shortgen11_seg2lagdiag_174924",
    "shortgen11_seg2lagproj025_182101",
    "shortgen11_seg2lagproj025_183529",
    "shortgen11_accbiasz_diag",
    "shortgen11_accbiasz_apply",
]
ALPHAS = [0.0, 0.5, 0.75, 1.0, 1.1, 1.2, 1.25, 1.3, 1.35, 1.4, 1.5, 1.75, 2.0]


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
    value_f = to_float(value)
    return f"{value_f:.{digits}f}" if math.isfinite(value_f) else "nan"


def rmse(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


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


def subset(df: pd.DataFrame, run: str, start: float, end: float) -> pd.DataFrame:
    return df[(df["run"] == run) & (df["time_since_arm_sec"] >= start) & (df["time_since_arm_sec"] < end)]


def selector_active(frame: pd.DataFrame, selector: str) -> pd.Series:
    if selector == "accbias_history_ge_0p4":
        return pd.to_numeric(frame["accbias_z_history_deep_frac"], errors="coerce") >= 0.4
    if selector == "stamp_lag_ge_0p020":
        return pd.to_numeric(frame["stamp_lag_sec"], errors="coerce") >= 0.020
    if selector == "always":
        return pd.Series(True, index=frame.index)
    raise ValueError(f"unsupported selector: {selector}")


def candidate_error(frame: pd.DataFrame, alpha: float, selector: str) -> tuple[pd.Series, pd.Series]:
    active = selector_active(frame, selector)
    raw_x = pd.to_numeric(frame["raw_iekf_x_m"], errors="coerce")
    raw_y = pd.to_numeric(frame["raw_iekf_y_m"], errors="coerce")
    px4_x = pd.to_numeric(frame["px4_iekf_x_m"], errors="coerce")
    px4_y = pd.to_numeric(frame["px4_iekf_y_m"], errors="coerce")
    gt_x = pd.to_numeric(frame["gt_x_m"], errors="coerce")
    gt_y = pd.to_numeric(frame["gt_y_m"], errors="coerce")
    cand_x = raw_x.where(~active, raw_x + alpha * (px4_x - raw_x))
    cand_y = raw_y.where(~active, raw_y + alpha * (px4_y - raw_y))
    err = ((cand_x - gt_x) ** 2 + (cand_y - gt_y) ** 2) ** 0.5
    return err, active


def eval_window(
    frame: pd.DataFrame,
    alpha: float,
    selector: str,
    run: str,
    window: str,
    start: float,
    end: float,
) -> dict[str, object]:
    sub = subset(frame, run, start, end)
    err, active = candidate_error(sub, alpha, selector)
    raw = pd.to_numeric(sub["raw_iekf_error_m"], errors="coerce")
    px4 = pd.to_numeric(sub["px4_iekf_error_m"], errors="coerce")
    ekf2 = pd.to_numeric(sub["ekf2_error_m"], errors="coerce")
    cand_rmse = rmse(err)
    raw_rmse = rmse(raw)
    ekf2_rmse = rmse(ekf2)
    return {
        "alpha": alpha,
        "run": run,
        "window": window,
        "rows": len(sub),
        "active_frac": float(active.mean()) if len(active) else math.nan,
        "raw_rmse_m": raw_rmse,
        "px4_rmse_m": rmse(px4),
        "candidate_rmse_m": cand_rmse,
        "ekf2_rmse_m": ekf2_rmse,
        "candidate_minus_raw_m": cand_rmse - raw_rmse,
        "candidate_minus_ekf2_m": cand_rmse - ekf2_rmse,
    }


def alpha_summary(frame: pd.DataFrame, alpha: float, selector: str, target_runs: list[str]) -> dict[str, object]:
    pos_regs: list[float] = []
    pos_gaps: list[float] = []
    target_140_gaps: list[float] = []
    target_160_gaps: list[float] = []
    repeat2_140: dict[str, object] | None = None
    repeat2_160: dict[str, object] | None = None

    for run in POSITIVE_RUNS:
        row = eval_window(frame, alpha, selector, run, "main_40_180", 40.0, 180.0)
        pos_regs.append(to_float(row["candidate_minus_raw_m"]))
        pos_gaps.append(to_float(row["candidate_minus_ekf2_m"]))

    for run in target_runs:
        row140 = eval_window(frame, alpha, selector, run, "140_160", 140.0, 160.0)
        row160 = eval_window(frame, alpha, selector, run, "160_180", 160.0, 180.0)
        target_140_gaps.append(to_float(row140["candidate_minus_ekf2_m"]))
        target_160_gaps.append(to_float(row160["candidate_minus_ekf2_m"]))
        if run == "shortgen11_repeat2":
            repeat2_140 = row140
            repeat2_160 = row160

    assert repeat2_140 is not None and repeat2_160 is not None
    protected = max(pos_regs) <= 0.02 and max(pos_gaps) < 0.02
    strict_all = protected and max(target_140_gaps + target_160_gaps) <= 0.02
    strict_repeat2 = (
        protected
        and to_float(repeat2_140["candidate_minus_ekf2_m"]) <= 0.02
        and to_float(repeat2_160["candidate_minus_raw_m"]) <= 0.02
    )
    return {
        "alpha": alpha,
        "positive_max_main_regress_m": max(pos_regs),
        "positive_max_candidate_minus_ekf2_m": max(pos_gaps),
        "positive_all_protected": int(protected),
        "repeat2_140_candidate_minus_ekf2_m": repeat2_140["candidate_minus_ekf2_m"],
        "repeat2_160_candidate_minus_raw_m": repeat2_160["candidate_minus_raw_m"],
        "target_worst_140_candidate_minus_ekf2_m": max(target_140_gaps),
        "target_worst_160_candidate_minus_ekf2_m": max(target_160_gaps),
        "strict_repeat2_pass": int(strict_repeat2),
        "strict_all_target_pass": int(strict_all),
    }


def table_summary(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["alpha"], 2),
            fmt(row["positive_max_main_regress_m"]),
            row["positive_all_protected"],
            fmt(row["repeat2_140_candidate_minus_ekf2_m"]),
            fmt(row["repeat2_160_candidate_minus_raw_m"]),
            fmt(row["target_worst_140_candidate_minus_ekf2_m"]),
            fmt(row["target_worst_160_candidate_minus_ekf2_m"]),
            row["strict_all_target_pass"],
        ]
        for row in rows
    ]


def table_windows(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["alpha"], 2),
            row["run"],
            row["window"],
            fmt(row["active_frac"], 3),
            fmt(row["raw_rmse_m"]),
            fmt(row["px4_rmse_m"]),
            fmt(row["candidate_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["candidate_minus_ekf2_m"]),
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    selector: str,
    summary_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
) -> None:
    passing = [row for row in summary_rows if to_float(row["strict_all_target_pass"], 0.0) > 0.5]
    lines = [
        "# PHS5 projection alpha sweep",
        "",
        "Date: 2026-05-11",
        "",
        f"Offline-only row-level counterfactual. Selector is `{selector}`; alpha blends from raw IEKF output toward and beyond PX4-sphere projection. Groundtruth is used only for scoring.",
        "",
        "## Alpha Summary",
        "",
        markdown_table(
            [
                "alpha",
                "pos max regress",
                "protected",
                "repeat2 140-EKF2",
                "repeat2 160 raw delta",
                "worst target 140-EKF2",
                "worst target 160-EKF2",
                "strict all",
            ],
            table_summary(summary_rows),
        ),
        "",
        "## Selected Window Metrics",
        "",
        markdown_table(
            [
                "alpha",
                "run",
                "window",
                "active",
                "raw",
                "PX4 alpha1",
                "candidate",
                "EKF2",
                "candidate-EKF2",
            ],
            table_windows(selected_rows),
        ),
        "",
        "## Readout",
        "",
    ]
    if passing:
        alpha_labels = ", ".join(fmt(row["alpha"], 2) for row in passing)
        lines.append(
            f"- Strict all-target pass appears for alpha values: `{alpha_labels}`."
        )
    else:
        lines.append("- No alpha in the tested grid passes the strict all-target gate.")
    lines.extend(
        [
            "- Alpha 1.0 is full PX4-sphere projection. It still leaves `shortgen11_accbiasz_diag` 140-160 above EKF2 by about 0.031 m.",
            "- Alpha 1.1-1.3 repairs that blocker in this offline table while keeping the positive routes inactive under the accbias-history selector.",
            "- This is not an online keeper yet. A super-PX4 output projection is an empirical correction and must be stress-tested on additional holdouts before implementation.",
            "",
            "Generated files:",
            f"- `{out_dir / 'alpha_summary.csv'}`",
            f"- `{out_dir / 'selected_window_metrics.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-csv", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument(
        "--selector",
        choices=("accbias_history_ge_0p4", "stamp_lag_ge_0p020", "always"),
        default="accbias_history_ge_0p4",
    )
    parser.add_argument("--extra-target-run", action="append", default=[])
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)
    frame = pd.read_csv(args.input_csv)
    target_runs = [*TARGET_RUNS, *args.extra_target_run]
    summary_rows = [alpha_summary(frame, alpha, args.selector, target_runs) for alpha in ALPHAS]
    write_csv(args.out_dir / "alpha_summary.csv", summary_rows)

    selected_alphas = {1.0, 1.1, 1.2, 1.3}
    selected_rows: list[dict[str, object]] = []
    for alpha in ALPHAS:
        if alpha not in selected_alphas:
            continue
        for run in target_runs:
            selected_rows.append(eval_window(frame, alpha, args.selector, run, "140_160", 140.0, 160.0))
            selected_rows.append(eval_window(frame, alpha, args.selector, run, "160_180", 160.0, 180.0))
    write_csv(args.out_dir / "selected_window_metrics.csv", selected_rows)
    write_report(args.out_dir, args.selector, summary_rows, selected_rows)
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
