#!/usr/bin/env python3
"""Offline atlas for PHS5 between-update propagation carry-in intervals."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_UPDATES = BASE / "phs5_innovation_covariance_diagnostic_2026-05-10" / "innovation_update_rows.csv"
DEFAULT_WINDOWS = BASE / "phs5_update_propagation_timeline_2026-05-10" / "timeline_window_summary.csv"
DEFAULT_OUT = BASE / "phs5_propagation_interval_atlas_2026-05-10"

TARGET_RUNS = {
    "shortgen11_repeat2",
    "shortgen11_seg2lagdiag_174924",
    "shortgen11_seg2lagproj025_182101",
    "shortgen11_seg2lagproj025_183529",
}
WINDOWS = [
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
    ("main_40_180", 40.0, 180.0),
]


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


def percentile(values: Iterable[float], p: float) -> float:
    vals = sorted(value for value in values if finite(value))
    if not vals:
        return math.nan
    rank = (len(vals) - 1) * p / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return vals[lo]
    frac = rank - lo
    return vals[lo] * (1.0 - frac) + vals[hi] * frac


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


def window_label(t: float) -> str:
    for label, start, end in WINDOWS:
        if start <= t < end:
            return label
    return "other"


def load_updates(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key, value in raw.items():
            if key not in {"run", "role", "group", "mavros_mode", "publish_projection_action"}:
                row[key] = to_float(value)
        rows.append(row)
    rows.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return rows


def load_bad_windows(path: Path) -> set[tuple[str, str]]:
    bad: set[tuple[str, str]] = set()
    for raw in read_csv(path):
        run = raw.get("run", "")
        window = raw.get("window", "")
        if run in TARGET_RUNS and window in {"140_160", "160_180"} and to_float(raw.get("iekf_minus_ekf2_rmse_m")) > 0.05:
            bad.add((run, window))
    bad.add(("shortgen11_repeat2", "120_140"))
    return bad


def run_groups(rows: Sequence[dict[str, object]]) -> dict[str, list[dict[str, object]]]:
    out: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        out.setdefault(str(row.get("run")), []).append(row)
    for group in out.values():
        group.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    return out


def build_intervals(updates: Sequence[dict[str, object]], bad_windows: set[tuple[str, str]]) -> list[dict[str, object]]:
    intervals: list[dict[str, object]] = []
    for run, rows in run_groups(updates).items():
        for current, nxt in zip(rows, rows[1:]):
            t0 = to_float(current.get("time_since_arm_sec"))
            t1 = to_float(nxt.get("time_since_arm_sec"))
            if not finite(t0) or not finite(t1) or t1 <= t0:
                continue
            label = window_label(t0)
            if label == "other":
                continue
            after = to_float(current.get("update_after_error_h_m"))
            next_before = to_float(nxt.get("update_before_error_h_m"))
            delta = next_before - after if finite(after) and finite(next_before) else math.nan
            group = str(current.get("group"))
            target_focus = (run, label) in bad_windows
            positive_main = group == "positive" and 40.0 <= t0 < 180.0
            intervals.append(
                {
                    "run": run,
                    "group": group,
                    "window": label,
                    "start_sec": t0,
                    "end_sec": t1,
                    "dt_sec": t1 - t0,
                    "propagation_delta_m": delta,
                    "propagation_rate_mps": delta / (t1 - t0) if finite(delta) and t1 > t0 else math.nan,
                    "propagation_positive": int(finite(delta) and delta > 0.0),
                    "target_focus": int(target_focus),
                    "target_focus_growth": int(target_focus and finite(delta) and delta > 0.0),
                    "positive_main": int(positive_main),
                    "positive_main_growth": int(positive_main and finite(delta) and delta > 0.0),
                    "speed_mps": to_float(current.get("speed_mps")),
                    "gyro_abs_deg_s": abs(to_float(current.get("gyro_deg_s"))),
                    "turning_now": to_float(current.get("turning_now")),
                    "post_turn_context": to_float(current.get("post_turn_context")),
                    "armed_cruise_context": to_float(current.get("armed_cruise_context")),
                    "formal_hnis_2d": to_float(current.get("formal_hnis_2d")),
                    "formal_whitened_norm": to_float(current.get("formal_whitened_norm")),
                    "residual_h_m": to_float(current.get("residual_h_m")),
                    "dx_pos_h_m": to_float(current.get("dx_pos_h_m")),
                    "dx_over_residual_h": to_float(current.get("dx_over_residual_h")),
                    "p_pos_std_before_h_m": to_float(current.get("p_pos_std_before_h_m")),
                    "p_pos_std_shrink_frac": to_float(current.get("p_pos_std_shrink_frac")),
                    "update_error_delta_h_m": to_float(current.get("update_error_delta_h_m")),
                    "update_worsened": int(to_float(current.get("update_error_delta_h_m")) > 0.0),
                    "start_after_error_m": after,
                    "end_before_error_m": next_before,
                }
            )
    return intervals


FEATURES = [
    "dt_sec",
    "speed_mps",
    "gyro_abs_deg_s",
    "turning_now",
    "post_turn_context",
    "armed_cruise_context",
    "formal_hnis_2d",
    "formal_whitened_norm",
    "residual_h_m",
    "dx_pos_h_m",
    "dx_over_residual_h",
    "p_pos_std_before_h_m",
    "p_pos_std_shrink_frac",
]

DIAG_FEATURES = [
    "update_error_delta_h_m",
    "update_worsened",
    "start_after_error_m",
    "end_before_error_m",
]


def feature_separation(intervals: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    target = [row for row in intervals if to_float(row.get("target_focus_growth")) > 0.5]
    positive = [row for row in intervals if to_float(row.get("positive_main_growth")) > 0.5]
    out: list[dict[str, object]] = []
    for field in FEATURES + DIAG_FEATURES:
        target_vals = [to_float(row.get(field)) for row in target]
        positive_vals = [to_float(row.get(field)) for row in positive]
        target_mean = mean(target_vals)
        positive_mean = mean(positive_vals)
        target_var = mean((value - target_mean) ** 2 for value in target_vals if finite(value) and finite(target_mean))
        positive_var = mean((value - positive_mean) ** 2 for value in positive_vals if finite(value) and finite(positive_mean))
        pooled = math.sqrt(target_var + positive_var) if finite(target_var) and finite(positive_var) else math.nan
        out.append(
            {
                "feature": field,
                "online_safe": int(field in FEATURES),
                "target_growth_count": len([value for value in target_vals if finite(value)]),
                "positive_growth_count": len([value for value in positive_vals if finite(value)]),
                "target_growth_mean": target_mean,
                "positive_growth_mean": positive_mean,
                "diff_target_minus_positive": target_mean - positive_mean,
                "separation_score": abs(target_mean - positive_mean) / pooled if finite(pooled) and pooled > 1e-9 else math.nan,
            }
        )
    out.sort(key=lambda row: to_float(row.get("separation_score"), -1.0), reverse=True)
    return out


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
        raise ValueError(self.op)

    def label(self) -> str:
        return f"{self.field}_{self.op}_{self.threshold:.3f}"


@dataclass(frozen=True)
class Candidate:
    family: str
    conditions: tuple[Condition, ...]

    @property
    def name(self) -> str:
        return "__".join(condition.label() for condition in self.conditions)

    def active(self, row: dict[str, object]) -> bool:
        return all(condition.matches(row) for condition in self.conditions)


def base_conditions() -> list[Condition]:
    specs = {
        "dt_sec": (("ge", 0.38), ("ge", 0.42), ("le", 0.38)),
        "speed_mps": (("ge", 2.0), ("ge", 3.5), ("le", 2.0)),
        "gyro_abs_deg_s": (("ge", 2.0), ("ge", 8.0), ("le", 2.0)),
        "turning_now": (("ge", 0.5),),
        "post_turn_context": (("ge", 0.5), ("le", 0.5)),
        "armed_cruise_context": (("ge", 0.5),),
        "formal_hnis_2d": (("ge", 6.0), ("ge", 10.0), ("le", 6.0)),
        "residual_h_m": (("ge", 0.16), ("ge", 0.22), ("le", 0.12)),
        "dx_pos_h_m": (("ge", 0.03), ("ge", 0.05), ("le", 0.02)),
        "dx_over_residual_h": (("le", 0.17), ("ge", 0.19), ("ge", 0.22)),
        "p_pos_std_before_h_m": (("ge", 0.028), ("ge", 0.031), ("le", 0.027)),
        "p_pos_std_shrink_frac": (("le", 0.09), ("ge", 0.10), ("ge", 0.12)),
    }
    out: list[Condition] = []
    for field, thresholds in specs.items():
        for op, threshold in thresholds:
            out.append(Condition(field, op, threshold))
    return out


def build_candidates() -> list[Candidate]:
    conditions = base_conditions()
    candidates = [Candidate(condition.field, (condition,)) for condition in conditions]
    guard_fields = {"armed_cruise_context", "turning_now", "post_turn_context", "gyro_abs_deg_s", "speed_mps"}
    guards = [condition for condition in conditions if condition.field in guard_fields]
    metrics = [condition for condition in conditions if condition.field not in guard_fields]
    for metric in metrics:
        for guard in guards:
            if metric.field != guard.field:
                candidates.append(Candidate(f"{metric.field}_x_{guard.field}", (metric, guard)))
    unique: dict[str, Candidate] = {}
    for candidate in candidates:
        unique[f"{candidate.family}:{candidate.name}"] = candidate
    return list(unique.values())


def active_frac(rows: Sequence[dict[str, object]], candidate: Candidate) -> float:
    if not rows:
        return math.nan
    return sum(1 for row in rows if candidate.active(row)) / len(rows)


def positive_delta_capture(rows: Sequence[dict[str, object]], candidate: Candidate) -> float:
    positives = [max(0.0, to_float(row.get("propagation_delta_m"))) for row in rows]
    total = sum(positives)
    if total <= 1e-9:
        return math.nan
    active = sum(value for row, value in zip(rows, positives) if candidate.active(row))
    return active / total


def score_candidate(intervals: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
    target_focus = [row for row in intervals if to_float(row.get("target_focus")) > 0.5]
    target_growth = [row for row in intervals if to_float(row.get("target_focus_growth")) > 0.5]
    positive_main = [row for row in intervals if to_float(row.get("positive_main")) > 0.5]
    positive_growth = [row for row in intervals if to_float(row.get("positive_main_growth")) > 0.5]
    target_cov = active_frac(target_focus, candidate)
    target_growth_cov = active_frac(target_growth, candidate)
    positive_active = active_frac(positive_main, candidate)
    positive_growth_active = active_frac(positive_growth, candidate)
    target_prop_capture = positive_delta_capture(target_focus, candidate)
    positive_prop_capture = positive_delta_capture(positive_main, candidate)
    strict = (
        finite(target_growth_cov)
        and target_growth_cov >= 0.60
        and finite(positive_active)
        and positive_active <= 0.25
        and finite(positive_prop_capture)
        and positive_prop_capture <= 0.30
    )
    return {
        "family": candidate.family,
        "candidate": candidate.name,
        "target_focus_active_frac": target_cov,
        "target_growth_active_frac": target_growth_cov,
        "target_positive_delta_capture_frac": target_prop_capture,
        "positive_main_active_frac": positive_active,
        "positive_growth_active_frac": positive_growth_active,
        "positive_positive_delta_capture_frac": positive_prop_capture,
        "strict_signature": int(strict),
    }


def score_candidates(intervals: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    rows = [score_candidate(intervals, candidate) for candidate in build_candidates()]
    rows.sort(
        key=lambda row: (
            0.0 if to_float(row.get("strict_signature")) > 0.5 else 1.0,
            -to_float(row.get("target_growth_active_frac"), -1.0),
            to_float(row.get("positive_main_active_frac"), 1.0),
            to_float(row.get("positive_positive_delta_capture_frac"), 1.0),
        )
    )
    return rows


def interval_summary(intervals: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    groups = [
        ("target_focus", lambda row: to_float(row.get("target_focus")) > 0.5),
        ("target_focus_growth", lambda row: to_float(row.get("target_focus_growth")) > 0.5),
        ("positive_main", lambda row: to_float(row.get("positive_main")) > 0.5),
        ("positive_main_growth", lambda row: to_float(row.get("positive_main_growth")) > 0.5),
    ]
    out: list[dict[str, object]] = []
    for name, pred in groups:
        rows = [row for row in intervals if pred(row)]
        out.append(
            {
                "group": name,
                "rows": len(rows),
                "prop_delta_mean_m": mean(to_float(row.get("propagation_delta_m")) for row in rows),
                "prop_delta_p90_m": percentile([to_float(row.get("propagation_delta_m")) for row in rows], 90.0),
                "positive_frac": mean(to_float(row.get("propagation_positive")) for row in rows),
                "speed_mean_mps": mean(to_float(row.get("speed_mps")) for row in rows),
                "gyro_mean_deg_s": mean(to_float(row.get("gyro_abs_deg_s")) for row in rows),
                "hnis_mean": mean(to_float(row.get("formal_hnis_2d")) for row in rows),
                "residual_h_mean_m": mean(to_float(row.get("residual_h_m")) for row in rows),
                "dx_over_residual_mean": mean(to_float(row.get("dx_over_residual_h")) for row in rows),
            }
        )
    return out


def table_summary(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["group"],
            row["rows"],
            fmt(row["prop_delta_mean_m"]),
            fmt(row["prop_delta_p90_m"]),
            fmt(row["positive_frac"]),
            fmt(row["speed_mean_mps"]),
            fmt(row["gyro_mean_deg_s"]),
            fmt(row["hnis_mean"]),
            fmt(row["residual_h_mean_m"]),
            fmt(row["dx_over_residual_mean"]),
        ]
        for row in rows
    ]


def table_sep(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["feature"],
            row["online_safe"],
            row["target_growth_count"],
            row["positive_growth_count"],
            fmt(row["target_growth_mean"]),
            fmt(row["positive_growth_mean"]),
            fmt(row["diff_target_minus_positive"]),
            fmt(row["separation_score"]),
        ]
        for row in rows
    ]


def table_candidates(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["family"],
            row["candidate"],
            fmt(row["target_growth_active_frac"]),
            fmt(row["target_positive_delta_capture_frac"]),
            fmt(row["positive_main_active_frac"]),
            fmt(row["positive_positive_delta_capture_frac"]),
            row["strict_signature"],
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    summary_rows: list[dict[str, object]],
    separation_rows: list[dict[str, object]],
    candidate_rows: list[dict[str, object]],
) -> None:
    strict = [row for row in candidate_rows if to_float(row.get("strict_signature")) > 0.5]
    protected = [
        row for row in candidate_rows
        if to_float(row.get("positive_main_active_frac")) <= 0.25
        and to_float(row.get("positive_positive_delta_capture_frac")) <= 0.30
    ]
    protected.sort(
        key=lambda row: (
            -to_float(row.get("target_growth_active_frac"), -1.0),
            to_float(row.get("positive_main_active_frac"), 1.0),
        )
    )
    lines = [
        "# PHS5 propagation interval atlas",
        "",
        "Date: 2026-05-10",
        "",
        "Offline-only atlas from existing update/propagation logs. No flight stack, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "Target focus windows are target bad windows plus repeat2 120-140s as the carry-in lead-in.",
        "",
        "## Interval Summary",
        "",
        markdown_table(
            ["group", "rows", "prop mean", "prop p90", "positive frac", "speed", "gyro", "HNIS", "residual h", "dx/resid"],
            table_summary(summary_rows),
        ),
        "",
        "## Feature Separation",
        "",
        markdown_table(
            ["feature", "online", "target n", "positive n", "target mean", "positive mean", "diff", "score"],
            table_sep(separation_rows[:18]),
        ),
        "",
        "## Top Protected Online-Safe Signatures",
        "",
        markdown_table(
            ["family", "candidate", "target growth active", "target prop capture", "positive active", "positive prop capture", "strict"],
            table_candidates(protected[:16]),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict:
        lines.append(f"- Strict online-safe propagation signatures found: `{len(strict)}`.")
    else:
        lines.append("- No tested online-safe propagation signature covers target growth while keeping positive-route activation low.")
    lines.extend(
        [
            "- Diagnostic-only features may separate better, but they depend on groundtruth/update error and cannot be promoted directly.",
            "",
            "Generated files:",
            f"- `{out_dir / 'propagation_interval_atlas_rows.csv'}`",
            f"- `{out_dir / 'propagation_interval_summary.csv'}`",
            f"- `{out_dir / 'propagation_feature_separation.csv'}`",
            f"- `{out_dir / 'propagation_signature_candidates.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--updates", default=str(DEFAULT_UPDATES))
    parser.add_argument("--windows", default=str(DEFAULT_WINDOWS))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    intervals = build_intervals(load_updates(Path(args.updates)), load_bad_windows(Path(args.windows)))
    summary_rows = interval_summary(intervals)
    separation_rows = feature_separation(intervals)
    candidate_rows = score_candidates(intervals)
    out_dir = Path(args.out_dir)
    write_csv(out_dir / "propagation_interval_atlas_rows.csv", intervals)
    write_csv(out_dir / "propagation_interval_summary.csv", summary_rows)
    write_csv(out_dir / "propagation_feature_separation.csv", separation_rows)
    write_csv(out_dir / "propagation_signature_candidates.csv", candidate_rows)
    write_report(out_dir, summary_rows, separation_rows, candidate_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
