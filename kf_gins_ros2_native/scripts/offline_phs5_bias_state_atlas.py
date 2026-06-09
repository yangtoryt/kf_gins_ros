#!/usr/bin/env python3
"""Offline atlas for PHS5 estimator bias/covariance state signatures."""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import pandas as pd


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_UPDATES = BASE / "phs5_innovation_covariance_diagnostic_2026-05-10" / "innovation_update_rows.csv"
DEFAULT_OUT = BASE / "phs5_bias_state_atlas_2026-05-11"

RUN_DIRS = {
    "shortgen01_phs5c": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_phs5c_publish_core_stamp_bias_m003_20260510_003839"
    ),
    "shortgen02_phs5b": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_phs5b_publish_core_stamp_bias_m003_20260510_003144"
    ),
    "shortgen03_phs5a": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_phs5a_publish_core_stamp_bias_m003_20260510_002551"
    ),
    "shortgen04_hld1a_phs5": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_hld1a_phs5_publish_core_stamp_bias_m003_20260510_005528"
    ),
    "shortgen11_repeat2": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"
    ),
}

DIAGNOSTIC_RUN_DIRS = {
    "shortgen04_coretrace_old": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_20260510_224922"
    ),
    "shortgen04_coretrace_retry1": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_retry1_20260510_225734"
    ),
    "shortgen04_futureclamp": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_futureclamp_20260510_231317"
    ),
    "shortgen11_coretrace": Path(
        "/home/yang/kf_gins_ws/artifacts/manual/"
        "manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_20260510_223106"
    ),
}

STATE_COLUMNS = [
    "sequence",
    "armed_time_sec",
    "turning_now",
    "post_turn_context",
    "armed_cruise_context",
    "horizontal_speed_mps",
    "gyro_deg_s",
    "accbias_x_before_mps2",
    "accbias_y_before_mps2",
    "accbias_z_before_mps2",
    "gyrbias_x_before_radps",
    "gyrbias_y_before_radps",
    "gyrbias_z_before_radps",
    "dx_pos_h_norm_m",
    "dx_vel_h_norm_mps",
    "dx_phi_yaw_deg",
    "cov_pos_n_before_m2",
    "cov_pos_e_before_m2",
    "cov_pos_ne_before_m2",
    "cov_vel_n_before_m2ps2",
    "cov_vel_e_before_m2ps2",
    "cov_vel_ne_before_m2ps2",
    "cov_yaw_before_rad2",
    "cov_ba_x_before_m2ps4",
    "cov_ba_y_before_m2ps4",
    "cov_ba_z_before_m2ps4",
    "cov_bg_x_before_rad2ps2",
    "cov_bg_y_before_rad2ps2",
    "cov_bg_z_before_rad2ps2",
]

WINDOWS = [
    ("40-180", 40.0, 180.0),
    ("120-140", 120.0, 140.0),
    ("140-160", 140.0, 160.0),
    ("160-180", 160.0, 180.0),
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


def percentile(values: Iterable[float], pct: float) -> float:
    vals = sorted(value for value in values if finite(value))
    if not vals:
        return math.nan
    if len(vals) == 1:
        return vals[0]
    rank = (len(vals) - 1) * pct / 100.0
    lo = int(math.floor(rank))
    hi = int(math.ceil(rank))
    if lo == hi:
        return vals[lo]
    frac = rank - lo
    return vals[lo] * (1.0 - frac) + vals[hi] * frac


def markdown_table(headers: list[str], rows: list[list[object]]) -> str:
    lines = [
        "| " + " | ".join(headers) + " |",
        "| " + " | ".join("---" for _ in headers) + " |",
    ]
    lines.extend("| " + " | ".join(str(item) for item in row) + " |" for row in rows)
    return "\n".join(lines)


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


def numeric_series(frame: pd.DataFrame, column: str) -> pd.Series:
    if column not in frame:
        return pd.Series([math.nan] * len(frame), index=frame.index, dtype="float64")
    return pd.to_numeric(frame[column], errors="coerce")


def load_state_rows(run_dirs: dict[str, Path]) -> pd.DataFrame:
    frames: list[pd.DataFrame] = []
    for run, run_dir in run_dirs.items():
        path = run_dir / "state_update_debug.csv"
        if not path.exists():
            continue
        frame = pd.read_csv(path)
        if "event_type" in frame and "applied" in frame:
            frame = frame[
                (frame["event_type"] == "gnss_position")
                & (pd.to_numeric(frame["applied"], errors="coerce") == 1)
            ].copy()
        for column in STATE_COLUMNS:
            if column not in frame:
                frame[column] = math.nan
        frame = frame[STATE_COLUMNS].copy()
        frame["run"] = run
        frames.append(frame)
    if not frames:
        return pd.DataFrame()
    out = pd.concat(frames, ignore_index=True)
    out["sequence"] = pd.to_numeric(out["sequence"], errors="coerce").astype("Int64")
    return out


def add_derived(frame: pd.DataFrame) -> pd.DataFrame:
    out = frame.copy()
    ax = numeric_series(out, "accbias_x_before_mps2")
    ay = numeric_series(out, "accbias_y_before_mps2")
    az = numeric_series(out, "accbias_z_before_mps2")
    gx = numeric_series(out, "gyrbias_x_before_radps")
    gy = numeric_series(out, "gyrbias_y_before_radps")
    gz = numeric_series(out, "gyrbias_z_before_radps")
    out["accbias_h_norm_mps2"] = (ax * ax + ay * ay) ** 0.5
    out["accbias_norm_mps2"] = (ax * ax + ay * ay + az * az) ** 0.5
    out["accbias_z_abs_mps2"] = az.abs()
    out["gyrbias_norm_radps"] = (gx * gx + gy * gy + gz * gz) ** 0.5
    out["gyro_abs_deg_s"] = numeric_series(out, "gyro_deg_s").abs()
    out["cov_pos_trace_h_m2"] = numeric_series(out, "cov_pos_n_before_m2") + numeric_series(out, "cov_pos_e_before_m2")
    out["cov_vel_trace_h_m2ps2"] = numeric_series(out, "cov_vel_n_before_m2ps2") + numeric_series(out, "cov_vel_e_before_m2ps2")
    out["cov_ba_trace_m2ps4"] = (
        numeric_series(out, "cov_ba_x_before_m2ps4")
        + numeric_series(out, "cov_ba_y_before_m2ps4")
        + numeric_series(out, "cov_ba_z_before_m2ps4")
    )
    out["cov_bg_trace_rad2ps2"] = (
        numeric_series(out, "cov_bg_x_before_rad2ps2")
        + numeric_series(out, "cov_bg_y_before_rad2ps2")
        + numeric_series(out, "cov_bg_z_before_rad2ps2")
    )
    return out


def window_label(t: float) -> str:
    for label, start, end in WINDOWS:
        if label != "40-180" and start <= t < end:
            return label
    return "other"


def build_rows(updates_path: Path) -> pd.DataFrame:
    updates = pd.read_csv(updates_path)
    updates["sequence"] = pd.to_numeric(updates["sequence"], errors="coerce").astype("Int64")
    state = add_derived(load_state_rows(RUN_DIRS))
    merged = updates.merge(state, on=["run", "sequence"], how="inner", suffixes=("", "_state"))
    merged = add_derived(merged)
    rows: list[dict[str, object]] = []
    for run, group in merged.groupby("run", sort=False):
        group = group.sort_values("time_since_arm_sec").reset_index(drop=True)
        for index in range(len(group) - 1):
            cur = group.iloc[index]
            nxt = group.iloc[index + 1]
            t = to_float(cur.get("time_since_arm_sec"))
            if not 40.0 <= t < 180.0:
                continue
            prop_delta = to_float(nxt.get("update_before_error_h_m")) - to_float(cur.get("update_after_error_h_m"))
            row = cur.to_dict()
            row["window"] = window_label(t)
            row["propagation_delta_m"] = prop_delta
            row["propagation_positive"] = int(finite(prop_delta) and prop_delta > 0.0)
            row["target_focus"] = int(run == "shortgen11_repeat2" and row["window"] in {"120-140", "140-160"})
            row["target_focus_growth"] = int(row["target_focus"] and finite(prop_delta) and prop_delta > 0.0)
            row["positive_main"] = int(row.get("group") == "positive")
            row["positive_main_growth"] = int(row["positive_main"] and finite(prop_delta) and prop_delta > 0.0)
            rows.append(row)
    return pd.DataFrame(rows)


FEATURES = [
    "accbias_z_before_mps2",
    "accbias_z_abs_mps2",
    "accbias_norm_mps2",
    "accbias_h_norm_mps2",
    "gyrbias_norm_radps",
    "dx_pos_h_norm_m",
    "dx_vel_h_norm_mps",
    "dx_phi_yaw_deg",
    "cov_pos_trace_h_m2",
    "cov_vel_trace_h_m2ps2",
    "cov_yaw_before_rad2",
    "cov_ba_trace_m2ps4",
    "cov_bg_trace_rad2ps2",
    "speed_mps",
    "gyro_abs_deg_s",
    "turning_now",
    "post_turn_context",
    "armed_cruise_context",
]


def feature_separation(rows: pd.DataFrame) -> list[dict[str, object]]:
    target = rows[rows["target_focus_growth"] > 0.5]
    positive = rows[rows["positive_main_growth"] > 0.5]
    out: list[dict[str, object]] = []
    for feature in FEATURES:
        target_vals = pd.to_numeric(target.get(feature), errors="coerce")
        positive_vals = pd.to_numeric(positive.get(feature), errors="coerce")
        target_mean = float(target_vals.mean())
        positive_mean = float(positive_vals.mean())
        pooled = math.sqrt(float(target_vals.var(ddof=0)) + float(positive_vals.var(ddof=0)))
        out.append(
            {
                "feature": feature,
                "target_growth_count": int(target_vals.notna().sum()),
                "positive_growth_count": int(positive_vals.notna().sum()),
                "target_growth_mean": target_mean,
                "positive_growth_mean": positive_mean,
                "diff_target_minus_positive": target_mean - positive_mean,
                "separation_score": abs(target_mean - positive_mean) / pooled if finite(pooled) and pooled > 1e-12 else math.nan,
            }
        )
    out.sort(key=lambda row: to_float(row.get("separation_score"), -1.0), reverse=True)
    return out


@dataclass(frozen=True)
class Condition:
    field: str
    op: str
    threshold: float

    def matches(self, rows: pd.DataFrame) -> pd.Series:
        values = pd.to_numeric(rows.get(self.field), errors="coerce")
        if self.op == "ge":
            return values >= self.threshold
        if self.op == "le":
            return values <= self.threshold
        raise ValueError(self.op)

    def label(self) -> str:
        sign = "ge" if self.op == "ge" else "le"
        return f"{self.field}_{sign}_{self.threshold:.3f}"


@dataclass(frozen=True)
class Candidate:
    family: str
    conditions: tuple[Condition, ...]

    @property
    def name(self) -> str:
        return "__".join(condition.label() for condition in self.conditions)

    def mask(self, rows: pd.DataFrame) -> pd.Series:
        if rows.empty:
            return pd.Series([], dtype=bool)
        result = pd.Series([True] * len(rows), index=rows.index)
        for condition in self.conditions:
            result &= condition.matches(rows)
        return result


def base_conditions() -> list[Condition]:
    specs = {
        "accbias_z_before_mps2": (("le", -0.12), ("le", -0.15), ("le", -0.18), ("le", -0.20)),
        "accbias_z_abs_mps2": (("ge", 0.15), ("ge", 0.20), ("ge", 0.25)),
        "accbias_norm_mps2": (("ge", 0.18), ("ge", 0.20), ("ge", 0.25)),
        "accbias_h_norm_mps2": (("ge", 0.08), ("ge", 0.12), ("ge", 0.16)),
        "dx_pos_h_norm_m": (("ge", 0.03), ("ge", 0.04), ("le", 0.03)),
        "dx_vel_h_norm_mps": (("ge", 0.010), ("ge", 0.014), ("le", 0.010)),
        "speed_mps": (("ge", 3.0), ("ge", 3.5), ("le", 2.0)),
        "gyro_abs_deg_s": (("le", 8.0), ("ge", 8.0), ("le", 5.0)),
        "armed_cruise_context": (("ge", 0.5),),
        "turning_now": (("ge", 0.5), ("le", 0.5)),
    }
    out: list[Condition] = []
    for field, entries in specs.items():
        for op, threshold in entries:
            out.append(Condition(field, op, threshold))
    return out


def build_candidates() -> list[Candidate]:
    conditions = base_conditions()
    candidates = [Candidate(condition.field, (condition,)) for condition in conditions]
    bias_conditions = [condition for condition in conditions if condition.field.startswith("accbias")]
    guards = [
        condition for condition in conditions
        if condition.field in {"speed_mps", "gyro_abs_deg_s", "armed_cruise_context", "turning_now", "dx_pos_h_norm_m"}
    ]
    for bias in bias_conditions:
        for guard in guards:
            candidates.append(Candidate(f"{bias.field}_x_{guard.field}", (bias, guard)))
    unique: dict[str, Candidate] = {}
    for candidate in candidates:
        unique[f"{candidate.family}:{candidate.name}"] = candidate
    return list(unique.values())


def active_fraction(rows: pd.DataFrame, candidate: Candidate) -> float:
    if rows.empty:
        return math.nan
    return float(candidate.mask(rows).mean())


def positive_delta_capture(rows: pd.DataFrame, candidate: Candidate) -> float:
    if rows.empty:
        return math.nan
    deltas = pd.to_numeric(rows["propagation_delta_m"], errors="coerce").clip(lower=0.0)
    total = float(deltas.sum())
    if total <= 1e-12:
        return math.nan
    return float(deltas[candidate.mask(rows)].sum() / total)


def score_candidates(rows: pd.DataFrame) -> list[dict[str, object]]:
    target_focus = rows[rows["target_focus"] > 0.5]
    target_growth = rows[rows["target_focus_growth"] > 0.5]
    positive_main = rows[rows["positive_main"] > 0.5]
    positive_growth = rows[rows["positive_main_growth"] > 0.5]
    out: list[dict[str, object]] = []
    for candidate in build_candidates():
        target_growth_active = active_fraction(target_growth, candidate)
        positive_main_active = active_fraction(positive_main, candidate)
        positive_capture = positive_delta_capture(positive_main, candidate)
        target_capture = positive_delta_capture(target_focus, candidate)
        strict = (
            finite(target_growth_active)
            and target_growth_active >= 0.60
            and finite(target_capture)
            and target_capture >= 0.60
            and finite(positive_main_active)
            and positive_main_active <= 0.25
            and finite(positive_capture)
            and positive_capture <= 0.30
        )
        out.append(
            {
                "family": candidate.family,
                "candidate": candidate.name,
                "target_focus_active_frac": active_fraction(target_focus, candidate),
                "target_growth_active_frac": target_growth_active,
                "target_positive_delta_capture_frac": target_capture,
                "positive_main_active_frac": positive_main_active,
                "positive_growth_active_frac": active_fraction(positive_growth, candidate),
                "positive_positive_delta_capture_frac": positive_capture,
                "strict_signature": int(strict),
            }
        )
    out.sort(
        key=lambda row: (
            0.0 if to_float(row.get("strict_signature")) > 0.5 else 1.0,
            -to_float(row.get("target_growth_active_frac"), -1.0),
            to_float(row.get("positive_main_active_frac"), 1.0),
            to_float(row.get("positive_positive_delta_capture_frac"), 1.0),
        )
    )
    return out


def window_summary(rows: pd.DataFrame) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for run, group in rows.groupby("run", sort=False):
        for label, start, end in WINDOWS:
            subset = group[
                (pd.to_numeric(group["time_since_arm_sec"], errors="coerce") >= start)
                & (pd.to_numeric(group["time_since_arm_sec"], errors="coerce") < end)
            ]
            out.append(
                {
                    "run": run,
                    "window": label,
                    "rows": len(subset),
                    "accbias_z_mean_mps2": mean(pd.to_numeric(subset.get("accbias_z_before_mps2"), errors="coerce")),
                    "accbias_norm_mean_mps2": mean(pd.to_numeric(subset.get("accbias_norm_mps2"), errors="coerce")),
                    "accbias_z_le_m0_15_frac": mean(
                        (pd.to_numeric(subset.get("accbias_z_before_mps2"), errors="coerce") <= -0.15).astype(float)
                    ),
                    "prop_delta_mean_m": mean(pd.to_numeric(subset.get("propagation_delta_m"), errors="coerce")),
                    "prop_positive_frac": mean(pd.to_numeric(subset.get("propagation_positive"), errors="coerce")),
                    "speed_mean_mps": mean(pd.to_numeric(subset.get("speed_mps"), errors="coerce")),
                    "gyro_abs_mean_deg_s": mean(pd.to_numeric(subset.get("gyro_abs_deg_s"), errors="coerce")),
                    "dx_pos_h_mean_m": mean(pd.to_numeric(subset.get("dx_pos_h_norm_m"), errors="coerce")),
                }
            )
    return out


def diagnostic_activation() -> list[dict[str, object]]:
    state = add_derived(load_state_rows(DIAGNOSTIC_RUN_DIRS))
    out: list[dict[str, object]] = []
    if state.empty:
        return out
    for run, group in state.groupby("run", sort=False):
        t = pd.to_numeric(group["armed_time_sec"], errors="coerce")
        for label, start, end in WINDOWS:
            subset = group[(t >= start) & (t < end)]
            out.append(
                {
                    "run": run,
                    "window": label,
                    "rows": len(subset),
                    "accbias_z_mean_mps2": mean(pd.to_numeric(subset.get("accbias_z_before_mps2"), errors="coerce")),
                    "accbias_norm_mean_mps2": mean(pd.to_numeric(subset.get("accbias_norm_mps2"), errors="coerce")),
                    "accbias_z_le_m0_15_frac": mean(
                        (pd.to_numeric(subset.get("accbias_z_before_mps2"), errors="coerce") <= -0.15).astype(float)
                    ),
                    "speed_mean_mps": mean(pd.to_numeric(subset.get("horizontal_speed_mps"), errors="coerce")),
                    "gyro_abs_mean_deg_s": mean(pd.to_numeric(subset.get("gyro_abs_deg_s"), errors="coerce")),
                }
            )
    return out


def table_window(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    keep = {"shortgen01_phs5c", "shortgen02_phs5b", "shortgen03_phs5a", "shortgen04_hld1a_phs5", "shortgen11_repeat2"}
    selected = [
        row for row in rows
        if row.get("run") in keep and row.get("window") in {"120-140", "140-160", "160-180", "40-180"}
    ]
    return [
        [
            row["run"],
            row["window"],
            row["rows"],
            fmt(row["accbias_z_mean_mps2"]),
            fmt(row["accbias_norm_mean_mps2"]),
            fmt(row["accbias_z_le_m0_15_frac"]),
            fmt(row["prop_delta_mean_m"]),
            fmt(row["prop_positive_frac"]),
            fmt(row["speed_mean_mps"]),
            fmt(row["gyro_abs_mean_deg_s"]),
        ]
        for row in selected
    ]


def table_features(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["feature"],
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
            fmt(row["target_focus_active_frac"]),
            fmt(row["target_growth_active_frac"]),
            fmt(row["target_positive_delta_capture_frac"]),
            fmt(row["positive_main_active_frac"]),
            fmt(row["positive_positive_delta_capture_frac"]),
            row["strict_signature"],
        ]
        for row in rows
    ]


def table_diag(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["run"],
            row["window"],
            row["rows"],
            fmt(row["accbias_z_mean_mps2"]),
            fmt(row["accbias_norm_mean_mps2"]),
            fmt(row["accbias_z_le_m0_15_frac"]),
            fmt(row["speed_mean_mps"]),
            fmt(row["gyro_abs_mean_deg_s"]),
        ]
        for row in rows
        if row.get("window") in {"120-140", "140-160", "160-180", "40-180"}
    ]


def write_report(
    out_dir: Path,
    rows: pd.DataFrame,
    summary_rows: list[dict[str, object]],
    separation_rows: list[dict[str, object]],
    candidate_rows: list[dict[str, object]],
    diagnostic_rows: list[dict[str, object]],
) -> None:
    strict = [row for row in candidate_rows if to_float(row.get("strict_signature")) > 0.5]
    top_candidates = candidate_rows[:18]
    target_focus = rows[rows["target_focus"] > 0.5]
    positive_main = rows[rows["positive_main"] > 0.5]
    lines = [
        "# PHS5 bias-state atlas",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only atlas from existing PHS5 state-update and update/propagation CSVs. No flight stack, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "Formal target focus is accepted negative `shortgen11_repeat2` 120-160s, including the 120-140s carry-in and 140-160s failure window. Formal positive protection is shortgen01/02/03/04 HLD1a 40-180s.",
        "",
        f"Rows: target focus `{len(target_focus)}`, positive main `{len(positive_main)}`.",
        "",
        "## Window Summary",
        "",
        markdown_table(
            ["run", "window", "rows", "ba_z", "ba_norm", "ba_z<=-0.15", "prop mean", "prop pos", "speed", "gyro"],
            table_window(summary_rows),
        ),
        "",
        "## Feature Separation",
        "",
        markdown_table(
            ["feature", "target n", "positive n", "target mean", "positive mean", "diff", "score"],
            table_features(separation_rows[:16]),
        ),
        "",
        "## Candidate Signatures",
        "",
        markdown_table(
            [
                "family",
                "candidate",
                "target active",
                "target growth active",
                "target prop capture",
                "positive active",
                "positive prop capture",
                "strict",
            ],
            table_candidates(top_candidates),
        ),
        "",
        "## Diagnostic Runs",
        "",
        markdown_table(
            ["run", "window", "rows", "ba_z", "ba_norm", "ba_z<=-0.15", "speed", "gyro"],
            table_diag(diagnostic_rows),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict:
        lines.append(f"- Strict online-visible bias-state signatures found: `{len(strict)}`.")
    else:
        lines.append("- No strict online-visible bias-state signature was found.")
    lines.extend(
        [
            "- The strongest accepted-negative separator is persistent negative vertical accelerometer bias state, not accelerometer-bias magnitude alone.",
            "- `accbias_z_before_mps2 <= -0.15` covers most repeat2 120-160s target propagation growth while barely activating on formal positive main windows.",
            "- This is a diagnostic/root-cause signature, not proof that a bias-triggered correction will improve IEKF. It should not be promoted directly to an online mechanism without a bounded replay/mechanism test.",
            "",
            "Generated files:",
            f"- `{out_dir / 'bias_state_rows.csv'}`",
            f"- `{out_dir / 'bias_window_summary.csv'}`",
            f"- `{out_dir / 'bias_feature_separation.csv'}`",
            f"- `{out_dir / 'bias_candidate_signatures.csv'}`",
            f"- `{out_dir / 'bias_diagnostic_activation.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--updates", type=Path, default=DEFAULT_UPDATES)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    args = parser.parse_args()

    rows = build_rows(args.updates)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    summary_rows = window_summary(rows)
    separation_rows = feature_separation(rows)
    candidate_rows = score_candidates(rows)
    diagnostic_rows = diagnostic_activation()

    write_csv(args.out_dir / "bias_state_rows.csv", rows.to_dict("records"))
    write_csv(args.out_dir / "bias_window_summary.csv", summary_rows)
    write_csv(args.out_dir / "bias_feature_separation.csv", separation_rows)
    write_csv(args.out_dir / "bias_candidate_signatures.csv", candidate_rows)
    write_csv(args.out_dir / "bias_diagnostic_activation.csv", diagnostic_rows)
    write_report(args.out_dir, rows, summary_rows, separation_rows, candidate_rows, diagnostic_rows)
    print(f"wrote: {args.out_dir / 'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
