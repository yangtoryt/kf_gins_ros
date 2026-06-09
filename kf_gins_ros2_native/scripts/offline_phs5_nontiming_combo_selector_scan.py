#!/usr/bin/env python3
"""Offline non-timing combo selector scan for PHS5 alpha projection.

This follows the oracle-gap diagnostic but refuses timing/phase-like fields as
candidate activators. It tests one- and two-condition segment gates over
online-visible projection/update geometry features, then scores the resulting
alpha-selection counterfactual against existing groundtruth rows.
"""

from __future__ import annotations

import argparse
import csv
import itertools
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_measurement_geometry_selector as mg


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_ROW_METRICS = BASE / "phs5_delayed_update_consistency_selector_2026-05-11" / "row_metrics.csv"
DEFAULT_SEGMENTS = BASE / "phs5_projection_oracle_gap_diagnostic_2026-05-11" / "segment_oracle_metrics.csv"
DEFAULT_OUT = BASE / "phs5_nontiming_combo_selector_scan_2026-05-11"

WINDOWS = [
    ("main_40_180", 40.0, 180.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
]
POSITIVE_RUNS = {
    "shortgen01_phs5c",
    "shortgen02_phs5b",
    "shortgen03_phs5a",
    "shortgen04_hld1a_phs5",
    "shortgen04_current_agelag_lag0",
    "shortgen04_current_lag0255",
}
TARGET_RUNS = {"shortgen11_repeat2", "shortgen11_current_agelag"}
TARGET_WINDOWS = {"140_160", "160_180"}

EXCLUDE_FEATURE_MARKERS = (
    "gnss_source_age",
    "stamp_lag",
    "core_age_sec",
    "t_mean",
    "segment_index",
    "update_count",
    "turning",
    "post_turn",
    "armed_cruise",
    "mavros",
    "vel_e",
    "vel_n",
    "core_speed",
    "raw_",
    "alpha_",
    "ekf2_",
    "error",
    "gt",
    "oracle",
    "harmful",
    "active",
    "target_",
    "positive_",
    "main_window",
    "projection_delta_e_m",
    "projection_delta_n_m",
    "core_minus_gnss_e",
    "core_minus_gnss_n",
)


@dataclass(frozen=True)
class Condition:
    feature: str
    op: str
    threshold: float

    def name(self) -> str:
        value = f"{abs(self.threshold):.4f}".replace(".", "p")
        if self.threshold < 0.0:
            value = "m" + value
        return f"{self.feature}_{self.op}_{value}"

    def active(self, row: dict[str, object]) -> bool:
        value = to_float(row.get(self.feature))
        if not finite(value):
            return False
        if self.op == "ge":
            return value >= self.threshold
        if self.op == "le":
            return value <= self.threshold
        raise ValueError(self.op)


@dataclass(frozen=True)
class Candidate:
    conditions: tuple[Condition, ...]

    def name(self) -> str:
        return "__".join(condition.name() for condition in self.conditions)

    def active(self, row: dict[str, object]) -> bool:
        return all(condition.active(row) for condition in self.conditions)


def finite(value: object) -> bool:
    return mg.finite(value)


def to_float(value: object, default: float = math.nan) -> float:
    return mg.to_float(value, default)


def fmt(value: object, digits: int = 4) -> str:
    return mg.fmt(value, digits)


def rmse(values: Iterable[float]) -> float:
    return mg.rmse(values)


def mean(values: Iterable[float]) -> float:
    return mg.mean(values)


def read_csv(path: Path) -> list[dict[str, str]]:
    with path.open(newline="") as handle:
        return list(csv.DictReader(handle))


def write_csv(path: Path, rows: Sequence[dict[str, object]]) -> None:
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


def markdown_table(headers: list[str], rows: Sequence[Sequence[object]]) -> str:
    return mg.markdown_table(list(headers), [list(row) for row in rows])


def load_numeric_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key, value in raw.items():
            if key not in {"run", "group", "source", "window"}:
                row[key] = to_float(value)
        rows.append(row)
    return rows


def feature_allowed(feature: str) -> bool:
    return feature.endswith("_mean") and not any(marker in feature for marker in EXCLUDE_FEATURE_MARKERS)


def split_segments(
    segments: Sequence[dict[str, object]],
) -> tuple[list[dict[str, object]], list[dict[str, object]]]:
    target_repair = [
        seg
        for seg in segments
        if to_float(seg.get("target_window"), 0.0) > 0.5
        and to_float(seg.get("oracle_repair"), 0.0) > 0.5
    ]
    positive_harm = [
        seg
        for seg in segments
        if to_float(seg.get("positive_run"), 0.0) > 0.5
        and to_float(seg.get("projection_harmful"), 0.0) > 0.5
    ]
    return target_repair, positive_harm


def candidate_conditions(segments: Sequence[dict[str, object]]) -> list[Condition]:
    target_repair, positive_harm = split_segments(segments)
    features = [feature for feature in segments[0].keys() if feature_allowed(feature)]
    conditions: list[Condition] = []
    seen: set[str] = set()
    for feature in features:
        target_values = [to_float(seg.get(feature)) for seg in target_repair if finite(seg.get(feature))]
        bad_values = [to_float(seg.get(feature)) for seg in positive_harm if finite(seg.get(feature))]
        values = sorted({*target_values, *bad_values})
        if len(target_values) < 2 or len(bad_values) < 2 or len(values) < 4:
            continue
        thresholds = values[1:-1]
        if len(thresholds) > 12:
            step = max(1, len(thresholds) // 12)
            thresholds = thresholds[::step]
        for op in ("ge", "le"):
            scored: list[tuple[float, float, float, float]] = []
            for threshold in thresholds:
                if op == "ge":
                    recall = sum(value >= threshold for value in target_values) / len(target_values)
                    bad_active = sum(value >= threshold for value in bad_values) / len(bad_values)
                else:
                    recall = sum(value <= threshold for value in target_values) / len(target_values)
                    bad_active = sum(value <= threshold for value in bad_values) / len(bad_values)
                score = recall - bad_active
                if recall <= 0.0:
                    continue
                scored.append((score, recall, -bad_active, threshold))
            scored.sort(reverse=True)
            for _score, _recall, _neg_bad, threshold in scored[:2]:
                condition = Condition(feature, op, threshold)
                name = condition.name()
                if name not in seen:
                    seen.add(name)
                    conditions.append(condition)
    return conditions


def condition_score(
    condition: Condition,
    target_repair: Sequence[dict[str, object]],
    positive_harm: Sequence[dict[str, object]],
) -> float:
    if not target_repair or not positive_harm:
        return 0.0
    recall = sum(condition.active(seg) for seg in target_repair) / len(target_repair)
    bad = sum(condition.active(seg) for seg in positive_harm) / len(positive_harm)
    return recall - bad


def make_candidates(
    conditions: Sequence[Condition],
    segments: Sequence[dict[str, object]],
    max_conditions: int = 48,
    max_pairs: int = 1200,
) -> list[Candidate]:
    target_repair, positive_harm = split_segments(segments)
    scored_conditions = sorted(
        ((condition_score(condition, target_repair, positive_harm), condition) for condition in conditions),
        key=lambda item: item[0],
        reverse=True,
    )
    selected_conditions = [condition for score, condition in scored_conditions[:max_conditions] if score > 0.0]
    candidates = [Candidate((condition,)) for condition in selected_conditions]
    pair_rows: list[tuple[float, Candidate]] = []
    for left, right in itertools.combinations(selected_conditions, 2):
        if left.feature == right.feature:
            continue
        if "cov_trace" in left.feature and "cov_trace" in right.feature:
            continue
        candidate = Candidate((left, right))
        if not target_repair or not positive_harm:
            score = 0.0
        else:
            recall = sum(candidate.active(seg) for seg in target_repair) / len(target_repair)
            bad = sum(candidate.active(seg) for seg in positive_harm) / len(positive_harm)
            score = recall - bad
        if score > 0.0:
            pair_rows.append((score, candidate))
    pair_rows.sort(key=lambda item: item[0], reverse=True)
    candidates.extend(candidate for _score, candidate in pair_rows[:max_pairs])
    return candidates


def build_segment_index(segments: Sequence[dict[str, object]]) -> dict[tuple[str, int], dict[str, object]]:
    return {
        (str(seg["run"]), int(to_float(seg.get("segment_index")))): seg
        for seg in segments
    }


WindowIndex = dict[tuple[str, str], list[dict[str, object]]]


def build_window_index(rows: Sequence[dict[str, object]]) -> WindowIndex:
    index: WindowIndex = {}
    for run in POSITIVE_RUNS | TARGET_RUNS:
        run_rows = [row for row in rows if row.get("run") == run]
        for window, start, end in WINDOWS:
            if run in POSITIVE_RUNS and window != "main_40_180":
                continue
            if run in TARGET_RUNS and window not in {"main_40_180", "140_160", "160_180"}:
                continue
            index[(run, window)] = [row for row in run_rows if start <= to_float(row.get("t")) < end]
    return index


def window_metric(
    window_index: WindowIndex,
    segment_index: dict[tuple[str, int], dict[str, object]],
    candidate: Candidate,
    run: str,
    window: str,
) -> dict[str, object]:
    subset = window_index[(run, window)]
    selected_errors: list[float] = []
    active_count = 0
    for row in subset:
        seg = segment_index.get((run, int(to_float(row.get("segment_index"), -1))))
        active = bool(seg is not None and candidate.active(seg))
        active_count += 1 if active else 0
        selected_errors.append(to_float(row.get("alpha_error_m")) if active else to_float(row.get("raw_error_m")))
    raw = rmse(to_float(row.get("raw_error_m")) for row in subset)
    selected = rmse(selected_errors)
    ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in subset)
    return {
        "candidate": candidate.name(),
        "run": run,
        "window": window,
        "rows": len(subset),
        "active_frac": active_count / len(subset) if subset else math.nan,
        "raw_rmse_m": raw,
        "selected_rmse_m": selected,
        "ekf2_rmse_m": ekf2,
        "selected_minus_raw_m": selected - raw if finite(selected) and finite(raw) else math.nan,
        "selected_minus_ekf2_m": selected - ekf2 if finite(selected) and finite(ekf2) else math.nan,
    }


def summarize_candidate(
    window_index: WindowIndex,
    segments: Sequence[dict[str, object]],
    segment_index: dict[tuple[str, int], dict[str, object]],
    candidate: Candidate,
) -> dict[str, object]:
    active_segments = [seg for seg in segments if candidate.active(seg)]
    target_repair, positive_harm = split_segments(segments)
    target_hit = [seg for seg in target_repair if candidate.active(seg)]
    bad_hit = [seg for seg in positive_harm if candidate.active(seg)]
    metrics: dict[tuple[str, str], dict[str, object]] = {}
    for run in sorted(POSITIVE_RUNS | TARGET_RUNS):
        for window, start, end in WINDOWS:
            if run in POSITIVE_RUNS and window != "main_40_180":
                continue
            if run in TARGET_RUNS and window not in {"main_40_180", "140_160", "160_180"}:
                continue
            metrics[(run, window)] = window_metric(window_index, segment_index, candidate, run, window)
    pos_reg = max(
        (
            to_float(metrics[(run, "main_40_180")].get("selected_minus_raw_m"))
            for run in POSITIVE_RUNS
            if finite(metrics[(run, "main_40_180")].get("selected_minus_raw_m"))
        ),
        default=math.nan,
    )
    target_improve = sum(
        max(0.0, -to_float(metrics[(run, window)].get("selected_minus_raw_m"), 0.0))
        for run in TARGET_RUNS
        for window in TARGET_WINDOWS
    )
    target_worst_ekf2 = max(
        (
            to_float(metrics[(run, window)].get("selected_minus_ekf2_m"))
            for run in TARGET_RUNS
            for window in TARGET_WINDOWS
            if finite(metrics[(run, window)].get("selected_minus_ekf2_m"))
        ),
        default=math.nan,
    )
    target_min_active = min(
        (
            to_float(metrics[(run, window)].get("active_frac"))
            for run in TARGET_RUNS
            for window in TARGET_WINDOWS
            if finite(metrics[(run, window)].get("active_frac"))
        ),
        default=math.nan,
    )
    strict = (
        finite(pos_reg)
        and pos_reg <= 0.01
        and target_improve >= 0.08
        and finite(target_worst_ekf2)
        and target_worst_ekf2 <= 0.03
        and finite(target_min_active)
        and target_min_active >= 0.20
    )
    return {
        "candidate": candidate.name(),
        "conditions": len(candidate.conditions),
        "active_segments": len(active_segments),
        "target_oracle_hit": len(target_hit),
        "target_oracle_total": len(target_repair),
        "positive_harmful_hit": len(bad_hit),
        "positive_harmful_total": len(positive_harm),
        "target_oracle_recall": len(target_hit) / len(target_repair) if target_repair else math.nan,
        "positive_harmful_active": len(bad_hit) / len(positive_harm) if positive_harm else math.nan,
        "positive_max_regress_m": pos_reg,
        "target_improve_sum_m": target_improve,
        "target_worst_candidate_minus_ekf2_m": target_worst_ekf2,
        "target_min_active_frac": target_min_active,
        "strict_pass": 1 if strict else 0,
    }


def sort_summary(row: dict[str, object]) -> tuple[float, float, float, float, float, float]:
    return (
        -to_float(row.get("strict_pass"), 0.0),
        to_float(row.get("positive_max_regress_m"), 1e9),
        to_float(row.get("target_worst_candidate_minus_ekf2_m"), 1e9),
        -to_float(row.get("target_improve_sum_m"), 0.0),
        to_float(row.get("positive_harmful_active"), 1e9),
        -to_float(row.get("target_oracle_recall"), 0.0),
    )


def selected_window_metrics(
    window_index: WindowIndex,
    segment_index: dict[tuple[str, int], dict[str, object]],
    candidates: Sequence[Candidate],
) -> list[dict[str, object]]:
    output: list[dict[str, object]] = []
    for candidate in candidates:
        for run in [
            "shortgen01_phs5c",
            "shortgen02_phs5b",
            "shortgen03_phs5a",
            "shortgen04_hld1a_phs5",
            "shortgen04_current_agelag_lag0",
            "shortgen04_current_lag0255",
            "shortgen11_repeat2",
            "shortgen11_current_agelag",
        ]:
            for window, start, end in WINDOWS:
                if run in POSITIVE_RUNS and window != "main_40_180":
                    continue
                if run in TARGET_RUNS and window not in {"main_40_180", "140_160", "160_180"}:
                    continue
                output.append(window_metric(window_index, segment_index, candidate, run, window))
    return output


def write_report(
    out_dir: Path,
    condition_count: int,
    candidate_rows: Sequence[dict[str, object]],
    selected_metrics: Sequence[dict[str, object]],
) -> None:
    strict_count = sum(to_float(row.get("strict_pass"), 0.0) > 0.5 for row in candidate_rows)
    protected_count = sum(
        finite(row.get("positive_max_regress_m")) and to_float(row.get("positive_max_regress_m")) <= 0.01
        for row in candidate_rows
    )
    target_repairing = sum(
        to_float(row.get("target_oracle_recall"), 0.0) >= 0.5
        and to_float(row.get("positive_harmful_active"), 1.0) <= 0.25
        for row in candidate_rows
    )
    top_table = [
        [
            row["candidate"],
            row["conditions"],
            fmt(row["positive_max_regress_m"]),
            fmt(row["target_improve_sum_m"]),
            fmt(row["target_worst_candidate_minus_ekf2_m"]),
            fmt(row["target_min_active_frac"], 3),
            fmt(row["target_oracle_recall"], 3),
            fmt(row["positive_harmful_active"], 3),
            row["strict_pass"],
        ]
        for row in candidate_rows[:15]
    ]
    tradeoff_rows = sorted(
        candidate_rows,
        key=lambda row: (
            -to_float(row.get("target_oracle_recall"), 0.0),
            to_float(row.get("positive_harmful_active"), 1.0),
            -to_float(row.get("target_improve_sum_m"), 0.0),
        ),
    )[:15]
    tradeoff_table = [
        [
            row["candidate"],
            row["conditions"],
            fmt(row["positive_max_regress_m"]),
            fmt(row["target_improve_sum_m"]),
            fmt(row["target_worst_candidate_minus_ekf2_m"]),
            fmt(row["target_oracle_recall"], 3),
            fmt(row["positive_harmful_active"], 3),
        ]
        for row in tradeoff_rows
    ]
    metric_table = [
        [
            row["candidate"],
            row["run"],
            row["window"],
            fmt(row["active_frac"], 3),
            fmt(row["raw_rmse_m"]),
            fmt(row["selected_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["selected_minus_raw_m"]),
            fmt(row["selected_minus_ekf2_m"]),
        ]
        for row in selected_metrics[:60]
    ]
    report = [
        "# PHS5 non-timing combo selector scan",
        "",
        "Date: 2026-05-11",
        "",
        "## Scope",
        "",
        "Offline-only scan over segment-level online-visible features from the oracle-gap diagnostic. Timing, phase, segment index/time, update-count, pure route-motion fields, and absolute ENU-axis projection/core-minus-GNSS components are excluded from candidate activation.",
        "",
        f"- conditions generated: `{condition_count}`",
        f"- candidates tested: `{len(candidate_rows)}`",
        f"- strict pass count: `{strict_count}`",
        f"- protected candidate count (`positive_max_regress_m <= 0.01`): `{protected_count}`",
        f"- partial target-repair candidates (`target_recall >= 0.5`, harmful_active <= 0.25): `{target_repairing}`",
        "",
        "## Protected-First Ranking",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "pos_reg",
                "target improve",
                "target worst-EKF2",
                "target min active",
                "target recall",
                "harm active",
                "strict",
            ],
            top_table,
        ),
        "",
        "## Target-Recall Tradeoff",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "pos_reg",
                "target improve",
                "target worst-EKF2",
                "target recall",
                "harm active",
            ],
            tradeoff_table,
        ),
        "",
        "## Selected Window Metrics",
        "",
        markdown_table(
            ["candidate", "run", "window", "active", "raw", "selected", "EKF2", "sel-raw", "sel-EKF2"],
            metric_table,
        ),
        "",
        "## Readout",
        "",
    ]
    if strict_count:
        report.append("At least one non-timing combo candidate passed the offline strict bar. This still requires an existing-route opt-in diagnostic before any new holdout.")
    else:
        report.append("No non-timing/axis-safe combo candidate passed the strict bar. The protected candidates avoid positive harmful segments but still leave the shortgen11 140-160 s target windows above EKF2; target-recall candidates can repair all target oracle segments only by accepting positive/control regressions.")
    report.extend(
        [
            "",
            "Generated files:",
            "",
            f"- `{out_dir / 'candidate_summary.csv'}`",
            f"- `{out_dir / 'selected_window_metrics.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(report) + "\n")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--rows", type=Path, default=DEFAULT_ROW_METRICS)
    parser.add_argument("--segments", type=Path, default=DEFAULT_SEGMENTS)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    rows = load_numeric_rows(args.rows)
    segments = load_numeric_rows(args.segments)
    conditions = candidate_conditions(segments)
    candidates = make_candidates(conditions, segments)
    segment_index = build_segment_index(segments)
    window_index = build_window_index(rows)
    summary_rows = [
        summarize_candidate(window_index, segments, segment_index, candidate) for candidate in candidates
    ]
    summary_rows.sort(key=sort_summary)

    chosen_names = [str(row["candidate"]) for row in summary_rows[:20]]
    tradeoff = sorted(
        summary_rows,
        key=lambda row: (
            -to_float(row.get("target_oracle_recall"), 0.0),
            to_float(row.get("positive_harmful_active"), 1.0),
            -to_float(row.get("target_improve_sum_m"), 0.0),
        ),
    )[:20]
    for row in tradeoff:
        name = str(row["candidate"])
        if name not in chosen_names:
            chosen_names.append(name)
    chosen = [candidate for candidate in candidates if candidate.name() in set(chosen_names)]
    chosen.sort(key=lambda candidate: chosen_names.index(candidate.name()))
    metric_rows = selected_window_metrics(window_index, segment_index, chosen)

    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.out_dir / "candidate_summary.csv", summary_rows)
    write_csv(args.out_dir / "selected_window_metrics.csv", metric_rows)
    write_report(args.out_dir, len(conditions), summary_rows, metric_rows)
    print(f"conditions={len(conditions)} candidates={len(candidates)} strict={sum(to_float(r.get('strict_pass'), 0.0) > 0.5 for r in summary_rows)}")
    print(args.out_dir / "report.md")


if __name__ == "__main__":
    main()
