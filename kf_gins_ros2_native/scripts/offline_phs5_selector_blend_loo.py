#!/usr/bin/env python3
"""Offline PHS5 soft projection-blend selector audit.

Hard PX4-sphere projection repairs shortgen11 but harms positive routes. This
script tests whether selector-controlled partial blends can keep the positive
routes protected while repairing shortgen11 target windows. It uses row-level
raw/PX4/groundtruth coordinates, so blend RMSE is computed exactly rather than
approximated from endpoint RMSEs.
"""

from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_selector_family_loo as hard


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_INPUT = BASE / "phs5_selector_replay_after_lag_failure_2026-05-10" / "geometry_row_metrics.csv"
DEFAULT_OUT = BASE / "phs5_selector_blend_loo_2026-05-10"
SEGMENT_LENGTHS = (1.0, 2.0, 5.0, 10.0)
ALPHAS = (0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.75, 1.00)
FOCUS_FAMILIES = {
    "global",
    "lag_only",
    "core_along",
    "core_along_x_proj_cross",
    "core_along_x_phase",
    "core_proj_phase",
    "core_state_proj",
    "state_diff_x_proj_cross",
    "state_proj_phase",
}


def finite(value: object) -> bool:
    return hard.finite(value)


def to_float(value: object, default: float = math.nan) -> float:
    return hard.to_float(value, default)


def fmt(value: object, digits: int = 4) -> str:
    return hard.fmt(value, digits)


def mean(values: Iterable[float]) -> float:
    return hard.mean(values)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: list[dict[str, object]]) -> None:
    hard.write_csv(path, rows)


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    return hard.markdown_table(headers, rows)


def load_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key in raw:
            if key not in {"run", "role", "group", "mavros_mode", "publish_projection_action"}:
                row[key] = to_float(raw.get(key))
        rows.append(row)
    return rows


def segment_index(t: float, segment_sec: float) -> int:
    return int(math.floor((t - 40.0) / segment_sec))


SEGMENT_FEATURES = [
    "stamp_lag_sec",
    "core_age_sec",
    "gnss_source_age_sec",
    "state_core_gnss_diff_h_m",
    "core_gnss_along_velocity_m",
    "core_gnss_cross_velocity_m",
    "latest_gnss_residual_h_m",
    "latest_gnss_residual_along_velocity_m",
    "latest_gnss_residual_cross_velocity_m",
    "latest_dx_pos_h_m",
    "latest_dx_pos_along_velocity_m",
    "latest_dx_pos_cross_velocity_m",
    "latest_dx_over_residual_h",
    "projection_delta_along_velocity_m",
    "projection_delta_cross_velocity_m",
    "core_speed_mps",
    "gyro_deg_s",
    "turning_now",
    "post_turn_context",
    "armed_cruise_context",
]


def build_segments(rows: Sequence[dict[str, object]], segment_sec: float) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in rows:
        t = to_float(row.get("time_since_arm_sec"))
        if 40.0 <= t < 180.0:
            grouped.setdefault((str(row.get("run")), segment_index(t, segment_sec)), []).append(row)

    segments: list[dict[str, object]] = []
    for (run, index), segment_rows in sorted(grouped.items()):
        if len(segment_rows) < 2:
            continue
        start = 40.0 + index * segment_sec
        raw_sse = 0.0
        blend_cross = 0.0
        blend_delta_sse = 0.0
        ekf2_sse = 0.0
        for row in segment_rows:
            raw_x = to_float(row.get("raw_iekf_x_m"))
            raw_y = to_float(row.get("raw_iekf_y_m"))
            px4_x = to_float(row.get("px4_iekf_x_m"))
            px4_y = to_float(row.get("px4_iekf_y_m"))
            gt_x = to_float(row.get("gt_x_m"))
            gt_y = to_float(row.get("gt_y_m"))
            ekf2_err = to_float(row.get("ekf2_error_m"))
            raw_ex = raw_x - gt_x
            raw_ey = raw_y - gt_y
            delta_x = px4_x - raw_x
            delta_y = px4_y - raw_y
            raw_sse += raw_ex * raw_ex + raw_ey * raw_ey
            blend_cross += raw_ex * delta_x + raw_ey * delta_y
            blend_delta_sse += delta_x * delta_x + delta_y * delta_y
            ekf2_sse += ekf2_err * ekf2_err
        segment: dict[str, object] = {
            "run": run,
            "group": "positive" if run in hard.POSITIVE_RUNS else "target",
            "segment_sec": segment_sec,
            "segment_index": index,
            "start_sec": start,
            "end_sec": start + segment_sec,
            "rows": len(segment_rows),
            "raw_sse": raw_sse,
            "blend_cross": blend_cross,
            "blend_delta_sse": blend_delta_sse,
            "px4_sse": raw_sse + 2.0 * blend_cross + blend_delta_sse,
            "ekf2_sse": ekf2_sse,
        }
        for feature in SEGMENT_FEATURES:
            segment[f"{feature}_mean"] = mean(to_float(row.get(feature)) for row in segment_rows)
        segments.append(segment)
    return segments


def segment_in_window(segment: dict[str, object], window: str) -> bool:
    start, end = hard.WINDOWS[window]
    return to_float(segment.get("start_sec")) < end and to_float(segment.get("end_sec")) > start


def select_segments(
    segments: Sequence[dict[str, object]],
    runs: Sequence[str],
    window: str,
) -> list[dict[str, object]]:
    run_set = set(runs)
    return [row for row in segments if row.get("run") in run_set and segment_in_window(row, window)]


def blend_sse(segment: dict[str, object], alpha: float) -> float:
    return (
        to_float(segment.get("raw_sse"))
        + 2.0 * alpha * to_float(segment.get("blend_cross"))
        + alpha * alpha * to_float(segment.get("blend_delta_sse"))
    )


def aggregate(
    segments: Sequence[dict[str, object]],
    candidate: hard.Candidate,
    alpha: float,
) -> dict[str, object]:
    rows = sum(int(to_float(segment.get("rows"), 0.0)) for segment in segments)
    if rows <= 0:
        return {
            "rows": 0,
            "active_frac": math.nan,
            "raw_rmse_m": math.nan,
            "px4_rmse_m": math.nan,
            "ekf2_rmse_m": math.nan,
            "candidate_rmse_m": math.nan,
            "candidate_minus_raw_m": math.nan,
            "candidate_minus_ekf2_m": math.nan,
        }
    raw_sse = 0.0
    px4_sse = 0.0
    ekf2_sse = 0.0
    candidate_sse = 0.0
    active_rows = 0
    for segment in segments:
        n = int(to_float(segment.get("rows"), 0.0))
        raw_sse += to_float(segment.get("raw_sse"))
        px4_sse += to_float(segment.get("px4_sse"))
        ekf2_sse += to_float(segment.get("ekf2_sse"))
        if candidate.active(segment):
            active_rows += n
            candidate_sse += blend_sse(segment, alpha)
        else:
            candidate_sse += to_float(segment.get("raw_sse"))
    raw_rmse = math.sqrt(raw_sse / rows)
    px4_rmse = math.sqrt(px4_sse / rows)
    ekf2_rmse = math.sqrt(ekf2_sse / rows)
    candidate_rmse = math.sqrt(max(0.0, candidate_sse) / rows)
    return {
        "rows": rows,
        "active_frac": active_rows / rows,
        "raw_rmse_m": raw_rmse,
        "px4_rmse_m": px4_rmse,
        "ekf2_rmse_m": ekf2_rmse,
        "candidate_rmse_m": candidate_rmse,
        "candidate_minus_raw_m": candidate_rmse - raw_rmse,
        "candidate_minus_ekf2_m": candidate_rmse - ekf2_rmse,
    }


def score_candidate(
    segments: Sequence[dict[str, object]],
    candidate: hard.Candidate,
    alpha: float,
    positive_runs: Sequence[str],
    target_runs: Sequence[str],
) -> dict[str, object]:
    pos_main = [
        aggregate(select_segments(segments, [run], "main_40_180"), candidate, alpha)
        for run in positive_runs
    ]
    target_main = [
        aggregate(select_segments(segments, [run], "main_40_180"), candidate, alpha)
        for run in target_runs
    ]
    target_140 = [
        aggregate(select_segments(segments, [run], "140_160"), candidate, alpha)
        for run in target_runs
    ]
    target_160 = [
        aggregate(select_segments(segments, [run], "160_180"), candidate, alpha)
        for run in target_runs
    ]
    pos_reg = [to_float(row["candidate_minus_raw_m"]) for row in pos_main]
    pos_gap = [to_float(row["candidate_minus_ekf2_m"]) for row in pos_main]
    target_140_gap = [to_float(row["candidate_minus_ekf2_m"]) for row in target_140]
    target_160_gap = [to_float(row["candidate_minus_ekf2_m"]) for row in target_160]
    target_main_gap = [to_float(row["candidate_minus_ekf2_m"]) for row in target_main]
    target_140_improve = [
        to_float(row["raw_rmse_m"]) - to_float(row["candidate_rmse_m"])
        for row in target_140
    ]
    pos_max_regress = max(pos_reg)
    pos_max_gap = max(pos_gap)
    worst_140 = max(target_140_gap)
    worst_160 = max(target_160_gap)
    protected = pos_max_regress <= 0.02 and pos_max_gap < 0.02
    return {
        "candidate": candidate.name,
        "family": candidate.family,
        "condition_count": len(candidate.conditions),
        "alpha": alpha,
        "positive_max_main_regress_m": pos_max_regress,
        "positive_max_candidate_minus_ekf2_m": pos_max_gap,
        "positive_all_protected": int(protected),
        "target_worst_main_candidate_minus_ekf2_m": max(target_main_gap),
        "target_worst_140_160_candidate_minus_ekf2_m": worst_140,
        "target_worst_160_180_candidate_minus_ekf2_m": worst_160,
        "target_mean_140_160_improve_m": mean(target_140_improve),
        "target_140_160_pass": int(worst_140 <= 0.02),
        "target_160_180_pass": int(worst_160 <= 0.02),
        "strict_pass": int(protected and worst_140 <= 0.02 and worst_160 <= 0.02),
    }


def sort_key(row: dict[str, object]) -> tuple[float, float, float, float, float]:
    return (
        0.0 if to_float(row.get("positive_all_protected"), 0.0) > 0.5 else 1.0,
        to_float(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
        to_float(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
        to_float(row.get("positive_max_main_regress_m")),
        -to_float(row.get("target_mean_140_160_improve_m")),
    )


def choose_best(rows: Sequence[dict[str, object]]) -> dict[str, object] | None:
    return sorted(rows, key=sort_key)[0] if rows else None


def top_protected_candidates(
    summary_rows: Sequence[dict[str, object]],
    candidates: Sequence[hard.Candidate],
    limit: int = 40,
) -> list[hard.Candidate]:
    protected = [
        row for row in summary_rows
        if to_float(row.get("positive_all_protected"), 0.0) > 0.5
    ]
    protected.sort(key=sort_key)
    selected_keys = {
        (str(row["family"]), str(row["candidate"]))
        for row in protected[:limit]
    }
    selected_keys.add(("global", "always_px4"))
    out = [
        candidate for candidate in candidates
        if (candidate.family, candidate.name) in selected_keys
    ]
    return out


def family_best_rows(summary_rows: Sequence[dict[str, object]], segment_sec: float) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for family in sorted({str(row["family"]) for row in summary_rows}):
        family_rows = [row for row in summary_rows if row["family"] == family]
        best = choose_best(family_rows)
        if best is not None:
            row = dict(best)
            row["segment_sec"] = segment_sec
            out.append(row)
    out.sort(key=lambda row: (to_float(row["segment_sec"]), sort_key(row)))
    return out


def target_leave_one_out(
    segments: Sequence[dict[str, object]],
    candidates: Sequence[hard.Candidate],
    segment_sec: float,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for holdout in hard.TARGET_RUNS:
        train_targets = [run for run in hard.TARGET_RUNS if run != holdout]
        scored = [
            score_candidate(segments, candidate, alpha, hard.POSITIVE_RUNS, train_targets)
            for candidate in candidates
            for alpha in ALPHAS
        ]
        protected = [row for row in scored if to_float(row.get("positive_all_protected"), 0.0) > 0.5]
        best = choose_best(protected)
        if best is None:
            continue
        candidate = next(
            item for item in candidates
            if item.name == best["candidate"] and item.family == best["family"]
        )
        alpha = to_float(best["alpha"])
        h140 = aggregate(select_segments(segments, [holdout], "140_160"), candidate, alpha)
        h160 = aggregate(select_segments(segments, [holdout], "160_180"), candidate, alpha)
        row = dict(best)
        row.update(
            {
                "segment_sec": segment_sec,
                "holdout": holdout,
                "holdout_140_candidate_minus_ekf2_m": h140["candidate_minus_ekf2_m"],
                "holdout_160_candidate_minus_ekf2_m": h160["candidate_minus_ekf2_m"],
                "holdout_140_180_pass": int(
                    to_float(h140["candidate_minus_ekf2_m"]) <= 0.02
                    and to_float(h160["candidate_minus_ekf2_m"]) <= 0.02
                ),
            }
        )
        rows.append(row)
    return rows


def table_rows(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["segment_sec"], 1),
            row["family"],
            row["candidate"],
            fmt(row["alpha"], 2),
            row["positive_all_protected"],
            fmt(row["positive_max_main_regress_m"]),
            fmt(row["target_worst_140_160_candidate_minus_ekf2_m"]),
            fmt(row["target_worst_160_180_candidate_minus_ekf2_m"]),
            fmt(row["target_mean_140_160_improve_m"]),
            row["strict_pass"],
        ]
        for row in rows
    ]


def loo_table(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["segment_sec"], 1),
            row["holdout"],
            row["family"],
            row["candidate"],
            fmt(row["alpha"], 2),
            row["holdout_140_180_pass"],
            fmt(row["holdout_140_candidate_minus_ekf2_m"]),
            fmt(row["holdout_160_candidate_minus_ekf2_m"]),
            fmt(row["positive_max_main_regress_m"]),
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    summary_rows: list[dict[str, object]],
    family_rows: list[dict[str, object]],
    target_loo: list[dict[str, object]],
) -> None:
    strict = [row for row in summary_rows if to_float(row.get("strict_pass"), 0.0) > 0.5]
    strict.sort(key=sort_key)
    best_by_segment: list[dict[str, object]] = []
    for segment_sec in sorted({to_float(row["segment_sec"]) for row in summary_rows}):
        best = choose_best([row for row in summary_rows if to_float(row["segment_sec"]) == segment_sec])
        if best is not None:
            best_by_segment.append(best)
    family_focus = [
        row for row in family_rows
        if row["family"] in {
            "global",
            "lag_only",
            "core_along_x_proj_cross",
            "core_along_x_phase",
            "core_proj_phase",
            "core_state_proj",
            "state_diff_x_proj_cross",
        }
    ]
    family_focus.sort(key=lambda row: (to_float(row["segment_sec"]), sort_key(row), str(row["family"])))

    lines = [
        "# PHS5 soft projection-blend selector audit",
        "",
        "Date: 2026-05-10",
        "",
        "Offline-only exact blend counterfactual using row-level raw/PX4/groundtruth coordinates. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "## Acceptance",
        "",
        "- Positive protection: shortgen01/02/03/04 `main_40_180` candidate regression <= 0.02 m and candidate-EKF2 < 0.02 m.",
        "- Target repair: every shortgen11 target run `140_160` and `160_180` candidate-EKF2 <= 0.02 m.",
        f"- Blend alphas tested: `{', '.join(f'{alpha:.2f}' for alpha in ALPHAS)}`.",
        "",
        "## Best Overall By Segment",
        "",
        markdown_table(
            [
                "seg",
                "family",
                "candidate",
                "alpha",
                "protected",
                "pos regress",
                "worst 140-EKF2",
                "worst 160-EKF2",
                "mean 140 improve",
                "strict",
            ],
            table_rows(best_by_segment),
        ),
        "",
        "## Strict-Pass Candidates",
        "",
    ]
    if strict:
        lines.append(
            markdown_table(
                [
                    "seg",
                    "family",
                    "candidate",
                    "alpha",
                    "protected",
                    "pos regress",
                    "worst 140-EKF2",
                    "worst 160-EKF2",
                    "mean 140 improve",
                    "strict",
                ],
                table_rows(strict[:12]),
            )
        )
    else:
        lines.append("No strict-pass candidates found.")
    lines.extend(
        [
            "",
            "## Best Family Rows",
            "",
            markdown_table(
                [
                    "seg",
                    "family",
                    "candidate",
                    "alpha",
                    "protected",
                    "pos regress",
                    "worst 140-EKF2",
                    "worst 160-EKF2",
                    "mean 140 improve",
                    "strict",
                ],
                table_rows(family_focus[:40]),
            ),
            "",
            "## Target Run Leave-One-Out",
            "",
            markdown_table(
                [
                    "seg",
                    "heldout",
                    "family",
                    "candidate",
                    "alpha",
                    "heldout pass",
                    "heldout 140-EKF2",
                    "heldout 160-EKF2",
                    "pos regress",
                ],
                loo_table(target_loo),
            ),
            "",
            "## Readout",
            "",
        ]
    )
    if strict:
        best = strict[0]
        lines.append(
            f"- Soft blending changes the decision boundary: `{best['family']}` / `{best['candidate']}` with alpha `{fmt(best['alpha'], 2)}` at `{fmt(best['segment_sec'], 1)}` s satisfies the current offline protection and target-repair criteria."
        )
    else:
        lines.append(
            "- Soft blending did not produce a strict-pass selector in this grid."
        )
    lines.extend(
        [
            "- Treat any strict-pass row as a candidate for further offline stress only, not as an online keeper: it was tuned on the current shortgen set and still needs route leave-one-out with genuinely new holdouts.",
            "- If a soft-blend row survives, the next mechanism should be default-off and diagnostic first, with alpha and active mask logged separately from hard projection.",
            "",
            "Generated files:",
            f"- `{out_dir / 'blend_candidate_summary_by_segment.csv'}`",
            f"- `{out_dir / 'blend_best_family_summary.csv'}`",
            f"- `{out_dir / 'blend_target_leave_one_out.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    rows = load_rows(Path(args.input))
    candidates = [
        candidate for candidate in hard.build_candidates()
        if candidate.family in FOCUS_FAMILIES
    ]
    out_dir = Path(args.out_dir)

    summary_rows: list[dict[str, object]] = []
    family_rows: list[dict[str, object]] = []
    segments_by_sec: dict[float, list[dict[str, object]]] = {}
    for segment_sec in SEGMENT_LENGTHS:
        segments = build_segments(rows, segment_sec)
        segments_by_sec[segment_sec] = segments
        scored = [
            score_candidate(segments, candidate, alpha, hard.POSITIVE_RUNS, hard.TARGET_RUNS)
            for candidate in candidates
            for alpha in ALPHAS
        ]
        for row in scored:
            row["segment_sec"] = segment_sec
        summary_rows.extend(scored)
        family_rows.extend(family_best_rows(scored, segment_sec))

    loo_candidates = top_protected_candidates(summary_rows, candidates)
    target_loo_rows: list[dict[str, object]] = []
    for segment_sec, segments in segments_by_sec.items():
        target_loo_rows.extend(target_leave_one_out(segments, loo_candidates, segment_sec))

    write_csv(out_dir / "blend_candidate_summary_by_segment.csv", summary_rows)
    write_csv(out_dir / "blend_best_family_summary.csv", family_rows)
    write_csv(out_dir / "blend_target_leave_one_out.csv", target_loo_rows)
    write_report(out_dir, summary_rows, family_rows, target_loo_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
