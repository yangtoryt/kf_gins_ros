#!/usr/bin/env python3
"""Counterfactual scores for shortgen02 IEKF publish-phase hold rows."""

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
from offline_shortgen02_projection_residual_diagnostic import fmt


COUNTERFACTUALS = [
    "baseline",
    "drop_hold_like",
    "retime_hold_nearest_nonhold",
    "retime_hold_prev_nonhold",
    "retime_hold_next_nonhold",
]


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def is_hold(row: dict[str, object]) -> bool:
    return to_float(row.get("publish_hold_like"), 0.0) > 0.5


def add_sample_kind(rows: list[dict[str, object]], kind: str) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for row in rows:
        item = dict(row)
        item["sample_kind"] = kind
        out.append(item)
    return out


def load_publish_rows(run_dir: Path) -> list[dict[str, object]]:
    path = run_dir / "shortgen02_publish_phase_diag" / "publish_phase_rows_120_180.csv"
    rows: list[dict[str, object]] = read_csv(path)
    rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return rows


def nearest(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt_sec: float,
) -> tuple[dict[str, object] | None, float]:
    if not rows or not finite(t):
        return None, math.nan
    idx = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - t), rows[pos]))
    if not candidates:
        return None, math.nan
    dt, row = min(candidates, key=lambda item: item[0])
    return (row, dt) if dt <= max_dt_sec else (None, dt)


def load_target_publish_rows(run_dir: Path, publish_rows: list[dict[str, object]]) -> list[dict[str, object]]:
    target_path = run_dir / "shortgen02_publish_phase_diag" / "publish_phase_target_high_rows.csv"
    if not target_path.exists():
        return []
    times = [to_float(row.get("time_since_arm_sec")) for row in publish_rows]
    out: list[dict[str, object]] = []
    for target in read_csv(target_path):
        row, dt = nearest(publish_rows, times, to_float(target.get("time_since_arm_sec")), 0.16)
        if row is None:
            continue
        item = dict(row)
        item["target_join_dt_sec"] = dt
        item["target_context"] = target.get("context", row.get("context", ""))
        out.append(item)
    return out


def nonhold_candidate_from_pool(
    candidate_rows: list[dict[str, object]],
    source_t: float,
    mode: str,
    max_retime_dt_sec: float,
) -> tuple[dict[str, object] | None, float]:
    times = [to_float(row.get("time_since_arm_sec")) for row in candidate_rows]
    idx = bisect.bisect_left(times, source_t)
    prev_item: tuple[float, dict[str, object]] | None = None
    next_item: tuple[float, dict[str, object]] | None = None
    for pos in range(idx - 1, -1, -1):
        if is_hold(candidate_rows[pos]):
            continue
        dt = abs(source_t - to_float(candidate_rows[pos].get("time_since_arm_sec")))
        prev_item = (dt, candidate_rows[pos])
        break
    for pos in range(idx, len(candidate_rows)):
        if is_hold(candidate_rows[pos]):
            continue
        dt = abs(to_float(candidate_rows[pos].get("time_since_arm_sec")) - source_t)
        next_item = (dt, candidate_rows[pos])
        break

    chosen: tuple[float, dict[str, object]] | None
    if mode == "retime_hold_prev_nonhold":
        chosen = prev_item
    elif mode == "retime_hold_next_nonhold":
        chosen = next_item
    elif mode == "retime_hold_nearest_nonhold":
        choices = [item for item in (prev_item, next_item) if item is not None]
        chosen = min(choices, key=lambda item: item[0]) if choices else None
    else:
        raise ValueError(mode)

    if chosen is None:
        return None, math.nan
    dt, row = chosen
    if dt > max_retime_dt_sec:
        return None, dt
    return row, dt


def apply_counterfactual(
    rows: list[dict[str, object]],
    mode: str,
    max_retime_dt_sec: float,
    candidate_rows: list[dict[str, object]] | None = None,
) -> list[dict[str, object]]:
    candidates = sorted(candidate_rows if candidate_rows is not None else rows, key=lambda row: to_float(row.get("time_since_arm_sec")))
    out: list[dict[str, object]] = []
    for row in rows:
        if mode == "baseline":
            item = dict(row)
            item["counterfactual"] = mode
            item["action"] = "keep"
            item["source_time_since_arm_sec"] = row.get("time_since_arm_sec")
            item["replacement_time_since_arm_sec"] = row.get("time_since_arm_sec")
            item["replacement_dt_sec"] = 0.0
            out.append(item)
            continue

        if mode == "drop_hold_like":
            if is_hold(row):
                continue
            item = dict(row)
            item["counterfactual"] = mode
            item["action"] = "keep_non_hold"
            item["source_time_since_arm_sec"] = row.get("time_since_arm_sec")
            item["replacement_time_since_arm_sec"] = row.get("time_since_arm_sec")
            item["replacement_dt_sec"] = 0.0
            out.append(item)
            continue

        if not is_hold(row):
            item = dict(row)
            item["counterfactual"] = mode
            item["action"] = "keep_non_hold"
            item["source_time_since_arm_sec"] = row.get("time_since_arm_sec")
            item["replacement_time_since_arm_sec"] = row.get("time_since_arm_sec")
            item["replacement_dt_sec"] = 0.0
            out.append(item)
            continue

        replacement, replacement_dt = nonhold_candidate_from_pool(
            candidates,
            to_float(row.get("time_since_arm_sec")),
            mode,
            max_retime_dt_sec,
        )
        if replacement is None:
            item = dict(row)
            item["counterfactual"] = mode
            item["action"] = "hold_unreplaced"
            item["source_time_since_arm_sec"] = row.get("time_since_arm_sec")
            item["replacement_time_since_arm_sec"] = row.get("time_since_arm_sec")
            item["replacement_dt_sec"] = replacement_dt
            out.append(item)
            continue

        item = dict(replacement)
        item["counterfactual"] = mode
        item["action"] = "retime_hold_to_nonhold"
        item["source_time_since_arm_sec"] = row.get("time_since_arm_sec")
        item["source_context"] = row.get("context")
        item["source_ekf2_error_h_m"] = row.get("ekf2_error_h_m")
        item["source_current_iekf_error_h_m"] = row.get("current_iekf_error_h_m")
        item["source_px4_iekf_error_h_m"] = row.get("px4_iekf_error_h_m")
        item["replacement_time_since_arm_sec"] = replacement.get("time_since_arm_sec")
        item["replacement_dt_sec"] = replacement_dt
        item["source_publish_hold_like"] = row.get("publish_hold_like")
        out.append(item)
    return out


def summarize_rows(sample_kind: str, mode: str, rows: list[dict[str, object]]) -> dict[str, object]:
    px4_vals = [to_float(row.get("px4_iekf_error_h_m")) for row in rows]
    ekf2_vals = [to_float(row.get("ekf2_error_h_m")) for row in rows]
    current_vals = [to_float(row.get("current_iekf_error_h_m")) for row in rows]
    actions = [str(row.get("action", "")) for row in rows]
    return {
        "sample_kind": sample_kind,
        "counterfactual": mode,
        "rows": len(rows),
        "hold_rows_remaining": sum(1 for row in rows if is_hold(row)),
        "retimed_rows": sum(1 for action in actions if action == "retime_hold_to_nonhold"),
        "unreplaced_hold_rows": sum(1 for action in actions if action == "hold_unreplaced"),
        "ekf2_mean_m": mean(ekf2_vals),
        "current_iekf_mean_m": mean(current_vals),
        "px4_iekf_mean_m": mean(px4_vals),
        "px4_minus_ekf2_mean_m": mean(px4_vals) - mean(ekf2_vals),
        "px4_iekf_p95_m": percentile(px4_vals, 95.0),
        "px4_iekf_max_m": max((item for item in px4_vals if finite(item)), default=math.nan),
        "px4_frac_over_0p3": mean(1.0 if item > 0.3 else 0.0 for item in px4_vals),
        "retime_abs_dt_mean_sec": mean(
            to_float(row.get("replacement_dt_sec")) for row in rows
            if str(row.get("action", "")) == "retime_hold_to_nonhold"
        ),
        "retime_abs_dt_max_sec": max(
            (
                to_float(row.get("replacement_dt_sec")) for row in rows
                if str(row.get("action", "")) == "retime_hold_to_nonhold"
            ),
            default=math.nan,
        ),
    }


def build_counterfactual_rows(
    sample_kind: str,
    source_rows: list[dict[str, object]],
    max_retime_dt_sec: float,
    candidate_rows: list[dict[str, object]] | None = None,
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    all_rows: list[dict[str, object]] = []
    summary: list[dict[str, object]] = []
    rows = add_sample_kind(source_rows, sample_kind)
    candidates = add_sample_kind(candidate_rows if candidate_rows is not None else source_rows, sample_kind)
    for mode in COUNTERFACTUALS:
        counterfactual_rows = apply_counterfactual(rows, mode, max_retime_dt_sec, candidates)
        all_rows.extend(counterfactual_rows)
        summary.append(summarize_rows(sample_kind, mode, counterfactual_rows))
    return all_rows, summary


def row_lookup(rows: list[dict[str, object]], sample: str, mode: str) -> dict[str, object]:
    return next(row for row in rows if row.get("sample_kind") == sample and row.get("counterfactual") == mode)


def write_report(
    out_dir: Path,
    summary_rows: list[dict[str, object]],
    target_counterfactual_rows: list[dict[str, object]],
    max_retime_dt_sec: float,
) -> None:
    summary_table = [
        [
            row["sample_kind"],
            row["counterfactual"],
            row["rows"],
            row["retimed_rows"],
            row["hold_rows_remaining"],
            fmt(row["ekf2_mean_m"]),
            fmt(row["current_iekf_mean_m"]),
            fmt(row["px4_iekf_mean_m"]),
            fmt(row["px4_minus_ekf2_mean_m"]),
            fmt(row["px4_iekf_p95_m"]),
            fmt(row["px4_frac_over_0p3"], 3),
            fmt(row["retime_abs_dt_max_sec"], 3),
        ]
        for row in summary_rows
    ]
    target_examples = [
        row for row in target_counterfactual_rows
        if row.get("counterfactual") == "retime_hold_nearest_nonhold"
    ]
    target_table = [
        [
            fmt(row["source_time_since_arm_sec"], 1),
            row.get("source_context", row.get("context", "")),
            row["action"],
            fmt(row.get("source_px4_iekf_error_h_m", row.get("px4_iekf_error_h_m"))),
            fmt(row["replacement_time_since_arm_sec"], 1),
            row.get("context", ""),
            fmt(row["replacement_dt_sec"], 3),
            fmt(row["ekf2_error_h_m"]),
            fmt(row["px4_iekf_error_h_m"]),
        ]
        for row in target_examples
    ]

    baseline_all = row_lookup(summary_rows, "120-180", "baseline")
    drop_all = row_lookup(summary_rows, "120-180", "drop_hold_like")
    retime_all = row_lookup(summary_rows, "120-180", "retime_hold_nearest_nonhold")
    baseline_target = row_lookup(summary_rows, "original_target_high_rows", "baseline")
    drop_target = row_lookup(summary_rows, "original_target_high_rows", "drop_hold_like")
    retime_target = row_lookup(summary_rows, "original_target_high_rows", "retime_hold_nearest_nonhold")

    lines = [
        "# Shortgen02 Publish Phase Counterfactual",
        "",
        "This report changes only the offline scoring rows. It does not modify estimator output, projection constants, or groundtruth.",
        "",
        f"Retiming variants replace a hold-like row with a neighboring non-hold row when the replacement is within `{max_retime_dt_sec:.2f} s`.",
        "",
        "## Counterfactual Summary",
        "",
        markdown_table(
            [
                "sample",
                "variant",
                "rows",
                "retimed",
                "hold_left",
                "ekf2_mean",
                "current_mean",
                "px4_mean",
                "px4-ekf2",
                "px4_p95",
                "px4_frac_gt_0p3",
                "max_retime_dt",
            ],
            summary_table,
        ),
        "",
        "## Original Target High Retiming Detail",
        "",
        markdown_table(
            [
                "source_t",
                "source_ctx",
                "action",
                "source_px4",
                "replacement_t",
                "replacement_ctx",
                "dt",
                "replacement_ekf2",
                "replacement_px4",
            ],
            target_table,
        ),
        "",
        "## Interpretation",
        "",
        f"- Baseline PX4-projected `120-180 s` is IEKF `{fmt(baseline_all['px4_iekf_mean_m'])}` m vs EKF2 `{fmt(baseline_all['ekf2_mean_m'])}` m.",
        f"- Dropping hold-like rows changes `120-180 s` to IEKF `{fmt(drop_all['px4_iekf_mean_m'])}` m vs EKF2 `{fmt(drop_all['ekf2_mean_m'])}` m; nearest non-hold retiming gives IEKF `{fmt(retime_all['px4_iekf_mean_m'])}` m.",
        f"- Baseline original target high rows are IEKF `{fmt(baseline_target['px4_iekf_mean_m'])}` m vs EKF2 `{fmt(baseline_target['ekf2_mean_m'])}` m.",
        f"- Dropping target hold-like rows leaves `{int(drop_target['rows'])}` rows at IEKF `{fmt(drop_target['px4_iekf_mean_m'])}` m; nearest retiming keeps `{int(retime_target['rows'])}` rows and gives IEKF `{fmt(retime_target['px4_iekf_mean_m'])}` m vs EKF2 `{fmt(retime_target['ekf2_mean_m'])}` m.",
        "- Treat this as a scoring/output-phase counterfactual, not a final performance claim. A real keeper still needs a code-level reason for the new-stamp/no-motion publish rows and a controlled online validation.",
        "",
        "Generated files:",
        f"- `{out_dir / 'publish_phase_counterfactual_summary.csv'}`",
        f"- `{out_dir / 'publish_phase_counterfactual_rows_120_180.csv'}`",
        f"- `{out_dir / 'publish_phase_counterfactual_target_rows.csv'}`",
    ]
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument("--out-dir")
    parser.add_argument("--max-retime-dt-sec", type=float, default=0.25)
    args = parser.parse_args()

    run_dir = Path(args.run_dir)
    out_dir = Path(args.out_dir) if args.out_dir else run_dir / "shortgen02_publish_phase_counterfactual"
    out_dir.mkdir(parents=True, exist_ok=True)

    publish_rows = load_publish_rows(run_dir)
    target_rows = load_target_publish_rows(run_dir, publish_rows)

    all_counterfactual_rows, all_summary = build_counterfactual_rows(
        "120-180",
        publish_rows,
        args.max_retime_dt_sec,
    )
    target_counterfactual_rows, target_summary = build_counterfactual_rows(
        "original_target_high_rows",
        target_rows,
        args.max_retime_dt_sec,
        publish_rows,
    )
    summary_rows = all_summary + target_summary

    write_csv(out_dir / "publish_phase_counterfactual_summary.csv", summary_rows)
    write_csv(out_dir / "publish_phase_counterfactual_rows_120_180.csv", all_counterfactual_rows)
    write_csv(out_dir / "publish_phase_counterfactual_target_rows.csv", target_counterfactual_rows)
    write_report(out_dir, summary_rows, target_counterfactual_rows, args.max_retime_dt_sec)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
