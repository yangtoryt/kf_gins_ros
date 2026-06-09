#!/usr/bin/env python3
"""Offline PHS5 selector-family leave-one-out audit.

This consumes the segment tables produced by
offline_phs5_selector_replay_after_lag_failure.py. It keeps the candidate space
small enough to audit by hand and reports whether any timing/frame/phase/update
geometry selector family can both protect shortgen01/02/03/04 and repair the
shortgen11 target windows across segment lengths.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "phs5_selector_family_loo_2026-05-10"
SEGMENT_INPUTS = {
    1.0: BASE / "phs5_selector_replay_after_lag_failure_seg1_2026-05-10" / "segment_projection_summary.csv",
    2.0: BASE / "phs5_selector_replay_after_lag_failure_2026-05-10" / "segment_projection_summary.csv",
    5.0: BASE / "phs5_selector_replay_after_lag_failure_seg5_2026-05-10" / "segment_projection_summary.csv",
    10.0: BASE / "phs5_selector_replay_after_lag_failure_seg10_2026-05-10" / "segment_projection_summary.csv",
}

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
]
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
    return f"{val:.{digits}f}" if finite(val) else "nan"


def mean(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return sum(vals) / len(vals) if vals else math.nan


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


@dataclass(frozen=True)
class Condition:
    field: str
    op: str
    threshold: float

    def matches(self, row: dict[str, object]) -> bool:
        value = to_float(row.get(self.field))
        if not finite(value):
            return False
        if self.op == "ge":
            return value >= self.threshold
        if self.op == "le":
            return value <= self.threshold
        if self.op == "abs_le":
            return abs(value) <= self.threshold
        if self.op == "abs_ge":
            return abs(value) >= self.threshold
        raise ValueError(self.op)

    def label(self) -> str:
        field = self.field.replace("_mean", "")
        if self.op.startswith("abs_"):
            return f"{self.op}_{field}_{self.threshold:.3f}"
        return f"{field}_{self.op}_{self.threshold:.3f}"


@dataclass(frozen=True)
class Candidate:
    name: str
    family: str
    conditions: tuple[Condition, ...]

    def active(self, row: dict[str, object]) -> bool:
        return all(condition.matches(row) for condition in self.conditions)


def make_candidate(family: str, conditions: Sequence[Condition]) -> Candidate:
    if not conditions:
        return Candidate("always_px4", family, tuple())
    return Candidate("__".join(condition.label() for condition in conditions), family, tuple(conditions))


def build_candidates() -> list[Candidate]:
    candidates: list[Candidate] = [make_candidate("global", tuple())]

    lag = [Condition("stamp_lag_sec_mean", "ge", value) for value in (0.020, 0.025, 0.030, 0.033, 0.040, 0.045)]
    core_along = [
        Condition("core_gnss_along_velocity_m_mean", "le", value)
        for value in (0.75, 0.80, 0.85, 0.90, 0.95, 1.00, 1.05)
    ]
    state_diff = [
        Condition("state_core_gnss_diff_h_m_mean", "le", value)
        for value in (0.75, 0.85, 0.95, 1.05, 1.15, 1.30, 1.40)
    ]
    proj_cross_le = [
        Condition("projection_delta_cross_velocity_m_mean", "le", value)
        for value in (-0.12, -0.10, -0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04)
    ]
    proj_cross_abs = [
        Condition("projection_delta_cross_velocity_m_mean", "abs_ge", value)
        for value in (0.02, 0.04, 0.06, 0.08, 0.10)
    ]
    residual_h = [
        Condition("latest_gnss_residual_h_m_mean", "le", value)
        for value in (0.15, 0.20, 0.30, 0.50, 1.00)
    ]
    dx_over = [
        Condition("latest_dx_over_residual_h_mean", "le", value)
        for value in (0.12, 0.16, 0.20, 0.25)
    ]
    phases = [
        Condition("armed_cruise_context_mean", "ge", 0.50),
        Condition("turning_now_mean", "le", 0.25),
        Condition("post_turn_context_mean", "le", 0.25),
    ]

    groups = {
        "lag_only": lag,
        "core_along": core_along,
        "state_diff": state_diff,
        "projection_cross": proj_cross_le + proj_cross_abs,
        "residual_update": residual_h + dx_over,
        "phase": phases,
    }
    for family, conditions in groups.items():
        candidates.extend(make_candidate(family, (condition,)) for condition in conditions)

    pair_specs = [
        ("lag_x_core_along", lag, core_along),
        ("lag_x_state_diff", lag, state_diff),
        ("core_along_x_state_diff", core_along, state_diff),
        ("core_along_x_proj_cross", core_along, proj_cross_le),
        ("state_diff_x_proj_cross", state_diff, proj_cross_le),
        ("core_along_x_phase", core_along, phases),
        ("state_diff_x_phase", state_diff, phases),
        ("proj_cross_x_phase", proj_cross_le, phases),
        ("core_along_x_residual", core_along, residual_h + dx_over),
        ("state_diff_x_residual", state_diff, residual_h + dx_over),
    ]
    for family, first_group, second_group in pair_specs:
        for first in first_group:
            for second in second_group:
                candidates.append(make_candidate(family, (first, second)))

    triple_specs = [
        ("core_state_proj", core_along, state_diff, proj_cross_le),
        ("core_proj_phase", core_along, proj_cross_le, phases),
        ("state_proj_phase", state_diff, proj_cross_le, phases),
        ("core_state_phase", core_along, state_diff, phases),
    ]
    for family, first_group, second_group, third_group in triple_specs:
        for first in first_group:
            for second in second_group:
                for third in third_group:
                    candidates.append(make_candidate(family, (first, second, third)))

    unique: dict[str, Candidate] = {}
    for candidate in candidates:
        unique[f"{candidate.family}:{candidate.name}"] = candidate
    return list(unique.values())


def load_segments(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key in raw:
            if key not in {"run", "group", "role", "projection_class"}:
                row[key] = to_float(raw.get(key))
        rows.append(row)
    return rows


def segment_in_window(segment: dict[str, object], window: str) -> bool:
    start, end = WINDOWS[window]
    return to_float(segment.get("start_sec")) < end and to_float(segment.get("end_sec")) > start


def select_segments(
    segments: Sequence[dict[str, object]],
    runs: Sequence[str],
    window: str,
) -> list[dict[str, object]]:
    run_set = set(runs)
    return [row for row in segments if row.get("run") in run_set and segment_in_window(row, window)]


def aggregate(
    segments: Sequence[dict[str, object]],
    candidate: Candidate,
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
    active_rows = 0
    totals = {"raw": 0.0, "px4": 0.0, "ekf2": 0.0, "candidate": 0.0}
    for segment in segments:
        n = int(to_float(segment.get("rows"), 0.0))
        raw = to_float(segment.get("raw_rmse_m"))
        px4 = to_float(segment.get("px4_rmse_m"))
        ekf2 = to_float(segment.get("ekf2_rmse_m"))
        is_active = candidate.active(segment)
        chosen = px4 if is_active else raw
        active_rows += n if is_active else 0
        totals["raw"] += raw * raw * n
        totals["px4"] += px4 * px4 * n
        totals["ekf2"] += ekf2 * ekf2 * n
        totals["candidate"] += chosen * chosen * n
    raw_rmse = math.sqrt(totals["raw"] / rows)
    px4_rmse = math.sqrt(totals["px4"] / rows)
    ekf2_rmse = math.sqrt(totals["ekf2"] / rows)
    candidate_rmse = math.sqrt(totals["candidate"] / rows)
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
    candidate: Candidate,
    positive_runs: Sequence[str],
    target_runs: Sequence[str],
) -> dict[str, object]:
    pos_main: dict[str, dict[str, object]] = {}
    target_140: dict[str, dict[str, object]] = {}
    target_160: dict[str, dict[str, object]] = {}
    target_main: dict[str, dict[str, object]] = {}
    for run in positive_runs:
        pos_main[run] = aggregate(select_segments(segments, [run], "main_40_180"), candidate)
    for run in target_runs:
        target_main[run] = aggregate(select_segments(segments, [run], "main_40_180"), candidate)
        target_140[run] = aggregate(select_segments(segments, [run], "140_160"), candidate)
        target_160[run] = aggregate(select_segments(segments, [run], "160_180"), candidate)

    positive_regs = [to_float(row["candidate_minus_raw_m"]) for row in pos_main.values()]
    positive_gaps = [to_float(row["candidate_minus_ekf2_m"]) for row in pos_main.values()]
    target_140_gaps = [to_float(row["candidate_minus_ekf2_m"]) for row in target_140.values()]
    target_160_gaps = [to_float(row["candidate_minus_ekf2_m"]) for row in target_160.values()]
    target_main_gaps = [to_float(row["candidate_minus_ekf2_m"]) for row in target_main.values()]
    target_140_improve = [
        to_float(row["raw_rmse_m"]) - to_float(row["candidate_rmse_m"])
        for row in target_140.values()
    ]

    pos_max_regress = max(positive_regs) if positive_regs else math.nan
    pos_max_gap = max(positive_gaps) if positive_gaps else math.nan
    target_worst_140 = max(target_140_gaps) if target_140_gaps else math.nan
    target_worst_160 = max(target_160_gaps) if target_160_gaps else math.nan
    positive_protected = finite(pos_max_regress) and pos_max_regress <= 0.02 and pos_max_gap < 0.02
    target_140_pass = finite(target_worst_140) and target_worst_140 <= 0.02
    target_160_pass = finite(target_worst_160) and target_worst_160 <= 0.02
    return {
        "candidate": candidate.name,
        "family": candidate.family,
        "condition_count": len(candidate.conditions),
        "positive_max_main_regress_m": pos_max_regress,
        "positive_max_candidate_minus_ekf2_m": pos_max_gap,
        "positive_all_protected": int(positive_protected),
        "target_worst_main_candidate_minus_ekf2_m": max(target_main_gaps) if target_main_gaps else math.nan,
        "target_worst_140_160_candidate_minus_ekf2_m": target_worst_140,
        "target_worst_160_180_candidate_minus_ekf2_m": target_worst_160,
        "target_mean_140_160_improve_m": mean(target_140_improve),
        "target_140_160_pass": int(target_140_pass),
        "target_160_180_pass": int(target_160_pass),
        "strict_pass": int(positive_protected and target_140_pass and target_160_pass),
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
    if not rows:
        return None
    return sorted(rows, key=sort_key)[0]


def family_best_rows(summary_rows: Sequence[dict[str, object]], segment_sec: float) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    families = sorted({str(row["family"]) for row in summary_rows})
    for family in families:
        family_rows = [row for row in summary_rows if row["family"] == family]
        best = choose_best(family_rows)
        if best is not None:
            row = dict(best)
            row["segment_sec"] = segment_sec
            out.append(row)
    out.sort(key=lambda row: (to_float(row.get("segment_sec")), sort_key(row)))
    return out


def positive_leave_one_out(
    segments: Sequence[dict[str, object]],
    candidates: Sequence[Candidate],
    segment_sec: float,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for holdout in POSITIVE_RUNS:
        train_pos = [run for run in POSITIVE_RUNS if run != holdout]
        scored = [score_candidate(segments, candidate, train_pos, TARGET_RUNS) for candidate in candidates]
        scored = [row for row in scored if to_float(row.get("positive_all_protected"), 0.0) > 0.5]
        best = choose_best(scored)
        if best is None:
            rows.append({"segment_sec": segment_sec, "holdout": holdout, "selected": ""})
            continue
        candidate = next(candidate for candidate in candidates if candidate.name == best["candidate"] and candidate.family == best["family"])
        holdout_score = aggregate(select_segments(segments, [holdout], "main_40_180"), candidate)
        out = dict(best)
        out.update(
            {
                "segment_sec": segment_sec,
                "holdout": holdout,
                "selected": best["candidate"],
                "holdout_main_active_frac": holdout_score["active_frac"],
                "holdout_main_candidate_minus_raw_m": holdout_score["candidate_minus_raw_m"],
                "holdout_main_candidate_minus_ekf2_m": holdout_score["candidate_minus_ekf2_m"],
                "holdout_protected": int(
                    to_float(holdout_score["candidate_minus_raw_m"]) <= 0.02
                    and to_float(holdout_score["candidate_minus_ekf2_m"]) < 0.02
                ),
            }
        )
        rows.append(out)
    return rows


def target_leave_one_out(
    segments: Sequence[dict[str, object]],
    candidates: Sequence[Candidate],
    segment_sec: float,
) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for holdout in TARGET_RUNS:
        train_target = [run for run in TARGET_RUNS if run != holdout]
        scored = [score_candidate(segments, candidate, POSITIVE_RUNS, train_target) for candidate in candidates]
        protected = [row for row in scored if to_float(row.get("positive_all_protected"), 0.0) > 0.5]
        best = choose_best(protected)
        if best is None:
            rows.append({"segment_sec": segment_sec, "holdout": holdout, "selected": ""})
            continue
        candidate = next(candidate for candidate in candidates if candidate.name == best["candidate"] and candidate.family == best["family"])
        holdout_140 = aggregate(select_segments(segments, [holdout], "140_160"), candidate)
        holdout_160 = aggregate(select_segments(segments, [holdout], "160_180"), candidate)
        out = dict(best)
        out.update(
            {
                "segment_sec": segment_sec,
                "holdout": holdout,
                "selected": best["candidate"],
                "holdout_140_active_frac": holdout_140["active_frac"],
                "holdout_140_candidate_minus_ekf2_m": holdout_140["candidate_minus_ekf2_m"],
                "holdout_160_active_frac": holdout_160["active_frac"],
                "holdout_160_candidate_minus_ekf2_m": holdout_160["candidate_minus_ekf2_m"],
                "holdout_140_180_pass": int(
                    to_float(holdout_140["candidate_minus_ekf2_m"]) <= 0.02
                    and to_float(holdout_160["candidate_minus_ekf2_m"]) <= 0.02
                ),
            }
        )
        rows.append(out)
    return rows


def row_table(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["segment_sec"], 1),
            row["family"],
            row["candidate"],
            row["positive_all_protected"],
            fmt(row["positive_max_main_regress_m"]),
            fmt(row["target_worst_140_160_candidate_minus_ekf2_m"]),
            fmt(row["target_worst_160_180_candidate_minus_ekf2_m"]),
            fmt(row["target_mean_140_160_improve_m"]),
            row["strict_pass"],
        ]
        for row in rows
    ]


def loo_positive_table(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["segment_sec"], 1),
            row["holdout"],
            row.get("family", ""),
            row.get("selected", ""),
            row.get("holdout_protected", ""),
            fmt(row.get("holdout_main_candidate_minus_raw_m")),
            fmt(row.get("holdout_main_candidate_minus_ekf2_m")),
            fmt(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
            fmt(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
        ]
        for row in rows
    ]


def loo_target_table(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            fmt(row["segment_sec"], 1),
            row["holdout"],
            row.get("family", ""),
            row.get("selected", ""),
            row.get("holdout_140_180_pass", ""),
            fmt(row.get("holdout_140_candidate_minus_ekf2_m")),
            fmt(row.get("holdout_160_candidate_minus_ekf2_m")),
            fmt(row.get("positive_max_main_regress_m")),
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    family_rows: list[dict[str, object]],
    all_best: list[dict[str, object]],
    pos_loo: list[dict[str, object]],
    target_loo: list[dict[str, object]],
) -> None:
    strict = [row for row in all_best if to_float(row.get("strict_pass"), 0.0) > 0.5]
    best_by_segment = []
    for segment_sec in sorted({to_float(row.get("segment_sec")) for row in all_best}):
        best = choose_best([row for row in all_best if to_float(row.get("segment_sec")) == segment_sec])
        if best is not None:
            best_by_segment.append(best)

    family_focus = [
        row for row in family_rows
        if row["family"] in {
            "lag_only",
            "core_along",
            "core_along_x_proj_cross",
            "core_along_x_phase",
            "core_state_proj",
            "core_proj_phase",
            "global",
            "state_diff_x_proj_cross",
        }
    ]
    family_focus.sort(key=lambda row: (to_float(row["segment_sec"]), sort_key(row), str(row["family"])))

    lines = [
        "# PHS5 selector-family leave-one-out audit",
        "",
        "Date: 2026-05-10",
        "",
        "Offline-only audit over replay segment tables. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "## Acceptance",
        "",
        "- Positive protection: shortgen01/02/03/04 `main_40_180` candidate regression <= 0.02 m and candidate-EKF2 < 0.02 m.",
        "- Target repair: every shortgen11 target run `140_160` and `160_180` candidate-EKF2 <= 0.02 m.",
        "",
        "## Best Overall By Segment",
        "",
        markdown_table(
            [
                "seg",
                "family",
                "candidate",
                "protected",
                "pos regress",
                "worst 140-EKF2",
                "worst 160-EKF2",
                "mean 140 improve",
                "strict",
            ],
            row_table(best_by_segment),
        ),
        "",
        "## Best Family Rows",
        "",
        markdown_table(
            [
                "seg",
                "family",
                "candidate",
                "protected",
                "pos regress",
                "worst 140-EKF2",
                "worst 160-EKF2",
                "mean 140 improve",
                "strict",
            ],
            row_table(family_focus[:36]),
        ),
        "",
        "## Positive Route Leave-One-Out",
        "",
        markdown_table(
            [
                "seg",
                "heldout",
                "family",
                "selected",
                "heldout protected",
                "heldout cand-raw",
                "heldout cand-EKF2",
                "train worst 140",
                "train worst 160",
            ],
            loo_positive_table(pos_loo),
        ),
        "",
        "## Target Run Leave-One-Out",
        "",
        markdown_table(
            [
                "seg",
                "heldout",
                "family",
                "selected",
                "heldout pass",
                "heldout 140-EKF2",
                "heldout 160-EKF2",
                "pos regress",
            ],
            loo_target_table(target_loo),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict:
        best = choose_best(strict)
        lines.append(
            f"- A strict-pass candidate exists in this family audit: `{best['candidate']}` at `{fmt(best['segment_sec'], 1)}` s. Treat it as offline-only until independent route holdouts confirm it."
        )
    else:
        lines.append(
            "- No candidate in the compact hand-auditable family passes both positive protection and all shortgen11 target 140-180 repair."
        )
    lines.extend(
        [
            "- The best protected candidates still come from lag-only or phase-guarded geometry families, and they miss at least one complete shortgen11 rerun.",
            "- The best target-repair geometry candidates remain unprotected because they activate harmful shortgen01-04 segments.",
            "- Default next step should remain offline: either add a new observable that separates positive-harm segments, or change the action from hard PX4-sphere projection to a softer/local blend before another online run.",
            "",
            "Generated files:",
            f"- `{out_dir / 'candidate_summary_by_segment.csv'}`",
            f"- `{out_dir / 'best_family_summary.csv'}`",
            f"- `{out_dir / 'positive_leave_one_out.csv'}`",
            f"- `{out_dir / 'target_leave_one_out.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    candidates = build_candidates()

    summary_rows: list[dict[str, object]] = []
    family_rows: list[dict[str, object]] = []
    pos_loo: list[dict[str, object]] = []
    target_loo: list[dict[str, object]] = []
    for segment_sec, input_path in SEGMENT_INPUTS.items():
        segments = load_segments(input_path)
        scored = [score_candidate(segments, candidate, POSITIVE_RUNS, TARGET_RUNS) for candidate in candidates]
        for row in scored:
            row["segment_sec"] = segment_sec
        summary_rows.extend(scored)
        family_rows.extend(family_best_rows(scored, segment_sec))
        pos_loo.extend(positive_leave_one_out(segments, candidates, segment_sec))
        target_loo.extend(target_leave_one_out(segments, candidates, segment_sec))

    write_csv(out_dir / "candidate_summary_by_segment.csv", summary_rows)
    write_csv(out_dir / "best_family_summary.csv", family_rows)
    write_csv(out_dir / "positive_leave_one_out.csv", pos_loo)
    write_csv(out_dir / "target_leave_one_out.csv", target_loo)
    write_report(out_dir, family_rows, summary_rows, pos_loo, target_loo)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
