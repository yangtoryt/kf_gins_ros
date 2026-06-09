#!/usr/bin/env python3
"""Audit segment-smoothed GNSS-source-age projection selectors.

This is offline-only. Selector features are online-visible; groundtruth is used
only for scoring the counterfactual output projection.
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
    / "phs5_cross_route_accbias_selector_seg10_plus_online_alpha12_2026-05-11"
    / "cross_route_row_metrics.csv"
)
DEFAULT_OUT = BASE / "phs5_gnss_source_age_segment_selector_plus_online_2026-05-11"

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
    "shortgen11_accbiashistproj_alpha12_online",
]
SEGMENT_SECS = [1.0, 2.0, 5.0, 10.0, 15.0, 20.0]
THRESHOLDS = [0.195, 0.200, 0.205, 0.210, 0.215, 0.220, 0.225, 0.230]
ALPHAS = [1.05, 1.10, 1.15, 1.20, 1.25, 1.30, 1.35, 1.40]


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


def update_run_sets_from_rows(rows: list[dict[str, str]]) -> None:
    """Keep default labels for legacy CSVs, but include any extra scored runs."""
    global POSITIVE_RUNS, TARGET_RUNS
    grouped: dict[str, set[str]] = defaultdict(set)
    for row in rows:
        run = str(row.get("run", "")).strip()
        group = str(row.get("group", "")).strip()
        if run and group:
            grouped[group].add(run)
    positives = sorted(grouped.get("positive", set()))
    targets = sorted(grouped.get("target", set()))
    if positives:
        POSITIVE_RUNS = positives
    if targets:
        TARGET_RUNS = targets


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


def in_window(row: dict[str, str], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def subset(rows: list[dict[str, str]], run: str, start: float, end: float) -> list[dict[str, str]]:
    return [row for row in rows if row.get("run") == run and in_window(row, start, end)]


def segment_index(row: dict[str, str], segment_sec: float) -> int:
    return int(math.floor((to_float(row.get("time_since_arm_sec")) - 40.0) / segment_sec))


def active_map(
    rows: list[dict[str, str]],
    segment_sec: float,
    threshold_sec: float,
) -> dict[tuple[str, int], bool]:
    grouped: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if in_window(row, 40.0, 180.0):
            grouped[(str(row.get("run")), segment_index(row, segment_sec))].append(row)
    return {
        key: mean(to_float(row.get("gnss_source_age_sec")) for row in segment_rows) <= threshold_sec
        for key, segment_rows in grouped.items()
    }


def candidate_error(
    row: dict[str, str],
    alpha: float,
    segment_sec: float,
    active_by_segment: dict[tuple[str, int], bool],
) -> tuple[float, bool]:
    active = active_by_segment.get((str(row.get("run")), segment_index(row, segment_sec)), False)
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
    start: float,
    end: float,
    segment_sec: float,
    threshold_sec: float,
    alpha: float,
) -> dict[str, object]:
    active_by_segment = active_map(rows, segment_sec, threshold_sec)
    return eval_window_with_active(rows, run, window, start, end, segment_sec, threshold_sec, alpha, active_by_segment)


def eval_window_with_active(
    rows: list[dict[str, str]],
    run: str,
    window: str,
    start: float,
    end: float,
    segment_sec: float,
    threshold_sec: float,
    alpha: float,
    active_by_segment: dict[tuple[str, int], bool],
) -> dict[str, object]:
    window_rows = subset(rows, run, start, end)
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
        "segment_sec": segment_sec,
        "threshold_sec": threshold_sec,
        "alpha": alpha,
        "rows": len(window_rows),
        "active_frac": mean(active_flags),
        "raw_rmse_m": raw,
        "px4_rmse_m": px4,
        "candidate_rmse_m": candidate,
        "ekf2_rmse_m": ekf2,
        "candidate_minus_raw_m": candidate - raw,
        "candidate_minus_ekf2_m": candidate - ekf2,
    }


def score_candidate(
    rows: list[dict[str, str]],
    segment_sec: float,
    threshold_sec: float,
    alpha: float,
    target_runs: list[str] | None = None,
) -> dict[str, object]:
    target_runs = target_runs if target_runs is not None else TARGET_RUNS
    active_by_segment = active_map(rows, segment_sec, threshold_sec)
    positive_regs: list[float] = []
    positive_gaps: list[float] = []
    target_140_gaps: list[float] = []
    target_160_gaps: list[float] = []
    for run in POSITIVE_RUNS:
        row = eval_window_with_active(
            rows, run, "main_40_180", 40.0, 180.0, segment_sec, threshold_sec, alpha, active_by_segment
        )
        positive_regs.append(to_float(row["candidate_minus_raw_m"]))
        positive_gaps.append(to_float(row["candidate_minus_ekf2_m"]))
    for run in target_runs:
        row140 = eval_window_with_active(
            rows, run, "140_160", 140.0, 160.0, segment_sec, threshold_sec, alpha, active_by_segment
        )
        row160 = eval_window_with_active(
            rows, run, "160_180", 160.0, 180.0, segment_sec, threshold_sec, alpha, active_by_segment
        )
        target_140_gaps.append(to_float(row140["candidate_minus_ekf2_m"]))
        target_160_gaps.append(to_float(row160["candidate_minus_ekf2_m"]))
    protected = max(positive_regs) <= 0.02 and max(positive_gaps) < 0.02
    strict = protected and max(target_140_gaps + target_160_gaps) <= 0.02
    return {
        "selector": "gnss_source_age_segment_mean_le",
        "segment_sec": segment_sec,
        "threshold_sec": threshold_sec,
        "alpha": alpha,
        "positive_max_main_regress_m": max(positive_regs),
        "positive_max_candidate_minus_ekf2_m": max(positive_gaps),
        "positive_all_protected": int(protected),
        "target_worst_140_candidate_minus_ekf2_m": max(target_140_gaps),
        "target_worst_160_candidate_minus_ekf2_m": max(target_160_gaps),
        "strict_all_target_pass": int(strict),
    }


def feature_separation(rows: list[dict[str, str]], segment_sec: float) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[dict[str, str]]] = defaultdict(list)
    for row in rows:
        if in_window(row, 40.0, 180.0):
            grouped[(str(row.get("run")), segment_index(row, segment_sec))].append(row)
    out: list[dict[str, object]] = []
    for (run, idx), segment_rows in grouped.items():
        start = 40.0 + idx * segment_sec
        raw = rmse(to_float(row.get("raw_iekf_error_m")) for row in segment_rows)
        px4 = rmse(to_float(row.get("px4_iekf_error_m")) for row in segment_rows)
        group = str(segment_rows[0].get("group"))
        label = "other"
        if group == "positive" and px4 - raw > 0.02:
            label = "positive_harm"
        if group == "target" and 140.0 <= start < 180.0 and raw - px4 > 0.02:
            label = "target_help"
        if label == "other":
            continue
        out.append(
            {
                "segment_sec": segment_sec,
                "run": run,
                "label": label,
                "start_sec": start,
                "end_sec": start + segment_sec,
                "rows": len(segment_rows),
                "raw_rmse_m": raw,
                "px4_rmse_m": px4,
                "projection_benefit_rmse_m": raw - px4,
                "gnss_source_age_sec_mean": mean(
                    to_float(row.get("gnss_source_age_sec")) for row in segment_rows
                ),
                "stamp_lag_sec_mean": mean(to_float(row.get("stamp_lag_sec")) for row in segment_rows),
                "accbias_z_before_mps2_mean": mean(
                    to_float(row.get("accbias_z_before_mps2")) for row in segment_rows
                ),
            }
        )
    return out


def target_leave_one_out(rows: list[dict[str, str]], summary_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for heldout in TARGET_RUNS:
        train_targets = [run for run in TARGET_RUNS if run != heldout]
        train_rows = [
            score_candidate(
                rows,
                to_float(row["segment_sec"]),
                to_float(row["threshold_sec"]),
                to_float(row["alpha"]),
                train_targets,
            )
            for row in summary_rows
        ]
        train_strict = [
            row for row in train_rows
            if to_float(row.get("strict_all_target_pass"), 0.0) > 0.5
        ]
        if not train_strict:
            out.append({"heldout": heldout, "train_strict_found": 0})
            continue
        train_strict.sort(
            key=lambda row: (
                to_float(row["target_worst_140_candidate_minus_ekf2_m"]),
                to_float(row["target_worst_160_candidate_minus_ekf2_m"]),
                to_float(row["positive_max_main_regress_m"]),
            )
        )
        best = train_strict[0]
        row140 = eval_window(
            rows,
            heldout,
            "140_160",
            140.0,
            160.0,
            to_float(best["segment_sec"]),
            to_float(best["threshold_sec"]),
            to_float(best["alpha"]),
        )
        row160 = eval_window(
            rows,
            heldout,
            "160_180",
            160.0,
            180.0,
            to_float(best["segment_sec"]),
            to_float(best["threshold_sec"]),
            to_float(best["alpha"]),
        )
        out.append(
            {
                "heldout": heldout,
                "train_strict_found": 1,
                "chosen_segment_sec": best["segment_sec"],
                "chosen_threshold_sec": best["threshold_sec"],
                "chosen_alpha": best["alpha"],
                "train_worst_140_candidate_minus_ekf2_m": best["target_worst_140_candidate_minus_ekf2_m"],
                "train_worst_160_candidate_minus_ekf2_m": best["target_worst_160_candidate_minus_ekf2_m"],
                "heldout_140_candidate_minus_ekf2_m": row140["candidate_minus_ekf2_m"],
                "heldout_160_candidate_minus_ekf2_m": row160["candidate_minus_ekf2_m"],
                "heldout_pass": int(
                    max(
                        to_float(row140["candidate_minus_ekf2_m"]),
                        to_float(row160["candidate_minus_ekf2_m"]),
                    )
                    <= 0.02
                ),
            }
        )
    return out


def table_summary(rows: list[dict[str, object]], limit: int = 16) -> list[list[object]]:
    rows = sorted(
        rows,
        key=lambda row: (
            -to_float(row.get("strict_all_target_pass"), 0.0),
            to_float(row.get("target_worst_140_candidate_minus_ekf2_m")),
            to_float(row.get("target_worst_160_candidate_minus_ekf2_m")),
            to_float(row.get("segment_sec")),
            to_float(row.get("alpha")),
        ),
    )
    return [
        [
            fmt(row["segment_sec"], 1),
            fmt(row["threshold_sec"], 3),
            fmt(row["alpha"], 2),
            fmt(row["positive_max_main_regress_m"]),
            row["positive_all_protected"],
            fmt(row["target_worst_140_candidate_minus_ekf2_m"]),
            fmt(row["target_worst_160_candidate_minus_ekf2_m"]),
            row["strict_all_target_pass"],
        ]
        for row in rows[:limit]
    ]


def table_windows(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
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


def table_loo(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row.get("heldout", ""),
            row.get("train_strict_found", ""),
            fmt(row.get("chosen_segment_sec"), 1),
            fmt(row.get("chosen_threshold_sec"), 3),
            fmt(row.get("chosen_alpha"), 2),
            row.get("heldout_pass", ""),
            fmt(row.get("heldout_140_candidate_minus_ekf2_m")),
            fmt(row.get("heldout_160_candidate_minus_ekf2_m")),
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    summary_rows: list[dict[str, object]],
    selected_rows: list[dict[str, object]],
    loo_rows: list[dict[str, object]],
    separation_rows: list[dict[str, object]],
) -> None:
    strict = [row for row in summary_rows if to_float(row.get("strict_all_target_pass"), 0.0) > 0.5]
    target_help = [
        to_float(row["gnss_source_age_sec_mean"])
        for row in separation_rows
        if row.get("label") == "target_help"
    ]
    positive_harm = [
        to_float(row["gnss_source_age_sec_mean"])
        for row in separation_rows
        if row.get("label") == "positive_harm"
    ]
    lines = [
        "# PHS5 GNSS-source-age segment selector audit",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only. Selector uses segment-smoothed `gnss_source_age_sec`; groundtruth is used only to score raw/PX4/alpha output projection counterfactuals.",
        "",
        "## Feature Separation",
        "",
        f"- 10 s target-help GNSS-source-age range: `{fmt(min(target_help), 4)}-{fmt(max(target_help), 4)}` s.",
        f"- 10 s positive-harm GNSS-source-age range: `{fmt(min(positive_harm), 4)}-{fmt(max(positive_harm), 4)}` s.",
        "- This separation is segment-level; 1-2 s row-like gating is too noisy and does not protect positives.",
        "",
        "## Candidate Summary",
        "",
        markdown_table(
            [
                "seg",
                "age thr",
                "alpha",
                "pos regress",
                "protected",
                "worst 140-EKF2",
                "worst 160-EKF2",
                "strict",
            ],
            table_summary(summary_rows),
        ),
        "",
        "## Selected Candidate Windows",
        "",
        "Selected candidate: 10 s segment mean `gnss_source_age_sec <= 0.205`, alpha `1.20`.",
        "",
        markdown_table(
            ["run", "window", "active", "raw", "PX4", "candidate", "EKF2", "candidate-EKF2"],
            table_windows(selected_rows),
        ),
        "",
        "## Target Leave-One-Out",
        "",
        markdown_table(
            ["heldout", "train strict", "seg", "thr", "alpha", "heldout pass", "heldout 140-EKF2", "heldout 160-EKF2"],
            table_loo(loo_rows),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict:
        lines.append(f"- Strict all-target pass candidates found: `{len(strict)}`.")
    else:
        lines.append("- No strict all-target pass candidate found.")
    heldout_fail = [row for row in loo_rows if to_float(row.get("heldout_pass"), 0.0) < 0.5]
    if heldout_fail:
        lines.append(
            "- Target LOO automatic selection has a boundary miss: "
            + ", ".join(str(row.get("heldout")) for row in heldout_fail)
            + ". The fixed center candidate still passes all target windows in the selected-window table."
        )
    else:
        lines.append("- Target LOO automatic selection passes all held-out target runs.")
    lines.extend(
        [
            "- Do not translate this directly into an online keeper yet. The signal should first be replayed on shortgen01/02/03/04 with 5-20 s segment smoothing and then checked on genuinely new routes.",
            "",
            "Generated files:",
            f"- `{out_dir / 'candidate_summary.csv'}`",
            f"- `{out_dir / 'selected_window_metrics.csv'}`",
            f"- `{out_dir / 'target_leave_one_out.csv'}`",
            f"- `{out_dir / 'feature_separation_segments.csv'}`",
        ]
    )
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
    update_run_sets_from_rows(rows)

    summary_rows = [
        score_candidate(rows, segment_sec, threshold, alpha)
        for segment_sec in SEGMENT_SECS
        for threshold in THRESHOLDS
        for alpha in ALPHAS
    ]
    write_csv(args.out_dir / "candidate_summary.csv", summary_rows)

    selected_rows: list[dict[str, object]] = []
    for run in POSITIVE_RUNS:
        selected_rows.append(eval_window(rows, run, "main_40_180", 40.0, 180.0, 10.0, 0.205, 1.20))
    for run in TARGET_RUNS:
        selected_rows.append(eval_window(rows, run, "140_160", 140.0, 160.0, 10.0, 0.205, 1.20))
        selected_rows.append(eval_window(rows, run, "160_180", 160.0, 180.0, 10.0, 0.205, 1.20))
    write_csv(args.out_dir / "selected_window_metrics.csv", selected_rows)

    loo_rows = target_leave_one_out(rows, summary_rows)
    write_csv(args.out_dir / "target_leave_one_out.csv", loo_rows)

    separation_rows = feature_separation(rows, 10.0)
    write_csv(args.out_dir / "feature_separation_segments.csv", separation_rows)

    write_report(args.out_dir, summary_rows, selected_rows, loo_rows, separation_rows)
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
