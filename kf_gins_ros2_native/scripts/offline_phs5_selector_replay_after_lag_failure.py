#!/usr/bin/env python3
"""Replay PHS5 projection selectors after the lag-only online failure.

This script is offline-only. It extends the existing PHS5 row join to include
the clean shortgen11 repeat2 run, the 17:49 lag diagnostic run, and the two
complete 18:21/18:35 opt-in lag-projection reruns. The goal is to explain which
projection/measurement-geometry features remain stable after the online
lag-only action failed to reproduce.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_measurement_geometry_selector as mg


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "phs5_selector_replay_after_lag_failure_2026-05-10"


@dataclass(frozen=True)
class RunSpec:
    label: str
    role: str
    group: str
    path: Path


RUNS = [
    RunSpec(
        "shortgen01_phs5c",
        "development/control",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_phs5c_publish_core_stamp_bias_m003_20260510_003839"),
    ),
    RunSpec(
        "shortgen02_phs5b",
        "development/control",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_phs5b_publish_core_stamp_bias_m003_20260510_003144"),
    ),
    RunSpec(
        "shortgen03_phs5a",
        "development/control",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_phs5a_publish_core_stamp_bias_m003_20260510_002551"),
    ),
    RunSpec(
        "shortgen04_hld1a_phs5",
        "clean holdout positive with local miss",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_hld1a_phs5_publish_core_stamp_bias_m003_20260510_005528"),
    ),
    RunSpec(
        "shortgen11_repeat2",
        "clean negative/generalization warning",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"),
    ),
    RunSpec(
        "shortgen11_seg2lagdiag_174924",
        "lag diagnostic, no online projection keeper",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_seg2lagdiag_20260510_174924"),
    ),
    RunSpec(
        "shortgen11_seg2lagproj025_182101",
        "complete opt-in lag projection rerun",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_seg2lagproj025_20260510_182101"),
    ),
    RunSpec(
        "shortgen11_seg2lagproj025_183529",
        "complete opt-in lag projection rerun after action-state fix",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_seg2lagproj025_20260510_183529"),
    ),
]

WINDOWS = [
    ("main_40_180", 40.0, 180.0),
    ("60_100", 60.0, 100.0),
    ("100_120", 100.0, 120.0),
    ("120_140", 120.0, 140.0),
    ("140_160", 140.0, 160.0),
    ("160_180", 160.0, 180.0),
]

POSITIVE_RUNS = [spec.label for spec in RUNS if spec.group == "positive"]
TARGET_RUNS = [spec.label for spec in RUNS if spec.group == "target"]

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
    "publish_projection_active",
    "segment_timing_gate_projection_active",
    "segment_timing_gate_current_active",
    "segment_timing_gate_current_lag_mean_sec",
]

CANDIDATE_FEATURES = [
    "stamp_lag_sec_mean",
    "state_core_gnss_diff_h_m_mean",
    "core_gnss_along_velocity_m_mean",
    "core_gnss_cross_velocity_m_mean",
    "latest_gnss_residual_h_m_mean",
    "latest_gnss_residual_along_velocity_m_mean",
    "latest_gnss_residual_cross_velocity_m_mean",
    "latest_dx_pos_h_m_mean",
    "latest_dx_pos_along_velocity_m_mean",
    "latest_dx_over_residual_h_mean",
    "projection_delta_along_velocity_m_mean",
    "projection_delta_cross_velocity_m_mean",
    "core_speed_mps_mean",
    "gyro_deg_s_mean",
    "turning_now_mean",
    "post_turn_context_mean",
    "armed_cruise_context_mean",
]


def finite(value: object) -> bool:
    return mg.finite(value)


def to_float(value: object, default: float = math.nan) -> float:
    return mg.to_float(value, default)


def fmt(value: object, digits: int = 4) -> str:
    return mg.fmt(value, digits)


def mean(values: Iterable[float]) -> float:
    return mg.mean(values)


def rmse(values: Iterable[float]) -> float:
    return mg.rmse(values)


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
    return mg.markdown_table(headers, rows)


def run_group(label: str) -> str:
    for spec in RUNS:
        if spec.label == label:
            return spec.group
    return ""


def run_role(label: str) -> str:
    for spec in RUNS:
        if spec.label == label:
            return spec.role
    return ""


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def load_all_rows() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for spec in RUNS:
        run_rows = mg.load_rows(mg.RunSpec(spec.label, spec.role, spec.path))
        for row in run_rows:
            row["group"] = spec.group
            row["role"] = spec.role
        rows.extend(run_rows)
    rows.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return rows


def summarize_row_subset(subset: Sequence[dict[str, object]]) -> dict[str, object]:
    raw = rmse(to_float(row.get("raw_iekf_error_m")) for row in subset)
    px4 = rmse(to_float(row.get("px4_iekf_error_m")) for row in subset)
    ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in subset)
    return {
        "rows": len(subset),
        "raw_iekf_rmse_m": raw,
        "px4_sphere_rmse_m": px4,
        "ekf2_rmse_m": ekf2,
        "px4_minus_raw_rmse_m": px4 - raw,
        "raw_minus_ekf2_rmse_m": raw - ekf2,
        "px4_minus_ekf2_rmse_m": px4 - ekf2,
        "projection_benefit_m_mean": mean(to_float(row.get("projection_benefit_m")) for row in subset),
        "stamp_lag_sec_mean": mean(to_float(row.get("stamp_lag_sec")) for row in subset),
        "core_gnss_along_velocity_m_mean": mean(to_float(row.get("core_gnss_along_velocity_m")) for row in subset),
        "latest_gnss_residual_along_velocity_m_mean": mean(
            to_float(row.get("latest_gnss_residual_along_velocity_m")) for row in subset
        ),
        "latest_dx_over_residual_h_mean": mean(to_float(row.get("latest_dx_over_residual_h")) for row in subset),
        "publish_projection_active_frac": mean(
            1.0 if to_float(row.get("publish_projection_active")) > 0.5 else 0.0 for row in subset
        ),
        "segment_gate_projection_active_frac": mean(
            1.0 if to_float(row.get("segment_timing_gate_projection_active")) > 0.5 else 0.0 for row in subset
        ),
        "segment_gate_current_active_frac": mean(
            1.0 if to_float(row.get("segment_timing_gate_current_active")) > 0.5 else 0.0 for row in subset
        ),
        "mission_frac": mean(1.0 if row.get("mavros_mode") == "AUTO.MISSION" else 0.0 for row in subset),
    }


def window_projection_summary(rows: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for spec in RUNS:
        run_rows = [row for row in rows if row.get("run") == spec.label]
        for label, start, end in WINDOWS:
            subset = [row for row in run_rows if in_window(row, start, end)]
            summary = summarize_row_subset(subset)
            summary.update({"run": spec.label, "group": spec.group, "role": spec.role, "window": label})
            out.append(summary)
    return out


def segment_key(row: dict[str, object], segment_sec: float) -> tuple[str, int]:
    t = to_float(row.get("time_since_arm_sec"))
    return str(row.get("run")), int(math.floor((t - 40.0) / segment_sec))


def make_segment(run: str, index: int, segment_sec: float, rows: list[dict[str, object]]) -> dict[str, object]:
    rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
    start = 40.0 + index * segment_sec
    end = start + segment_sec
    raw = rmse(to_float(row.get("raw_iekf_error_m")) for row in rows)
    px4 = rmse(to_float(row.get("px4_iekf_error_m")) for row in rows)
    ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in rows)
    segment: dict[str, object] = {
        "run": run,
        "group": run_group(run),
        "role": run_role(run),
        "segment_sec": segment_sec,
        "segment_index": index,
        "start_sec": start,
        "end_sec": end,
        "rows": len(rows),
        "raw_rmse_m": raw,
        "px4_rmse_m": px4,
        "ekf2_rmse_m": ekf2,
        "px4_minus_raw_rmse_m": px4 - raw,
        "raw_minus_ekf2_rmse_m": raw - ekf2,
        "px4_minus_ekf2_rmse_m": px4 - ekf2,
        "projection_benefit_rmse_m": raw - px4,
        "projection_class": "help" if raw - px4 > 0.02 else "harm" if px4 - raw > 0.02 else "neutral",
    }
    for feature in SEGMENT_FEATURES:
        segment[f"{feature}_mean"] = mean(to_float(row.get(feature)) for row in rows)
    segment["publish_projection_action_frac"] = mean(
        1.0 if row.get("publish_projection_action") == "segment_timing_gate_px4_sphere" else 0.0
        for row in rows
    )
    return segment


def build_segments(rows: Sequence[dict[str, object]], segment_sec: float) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in rows:
        t = to_float(row.get("time_since_arm_sec"))
        if 40.0 <= t < 180.0:
            grouped.setdefault(segment_key(row, segment_sec), []).append(row)
    return [
        make_segment(run, index, segment_sec, segment_rows)
        for (run, index), segment_rows in sorted(grouped.items())
        if len(segment_rows) >= 2
    ]


def segment_in_window(segment: dict[str, object], start: float, end: float) -> bool:
    return to_float(segment.get("start_sec")) < end and to_float(segment.get("end_sec")) > start


def segments_for_window(
    segments: Sequence[dict[str, object]],
    run: str,
    window: str,
) -> list[dict[str, object]]:
    start, end = next((ws, we) for label, ws, we in WINDOWS if label == window)
    return [segment for segment in segments if segment.get("run") == run and segment_in_window(segment, start, end)]


@dataclass(frozen=True)
class Condition:
    feature: str
    op: str
    threshold: float

    def matches(self, segment: dict[str, object]) -> bool:
        value = to_float(segment.get(self.feature))
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
        name = self.feature.replace("_mean", "")
        if self.op == "abs_le":
            return f"abs_{name}_le_{self.threshold:.3f}"
        if self.op == "abs_ge":
            return f"abs_{name}_ge_{self.threshold:.3f}"
        return f"{name}_{self.op}_{self.threshold:.3f}"


@dataclass(frozen=True)
class Candidate:
    name: str
    conditions: tuple[Condition, ...]

    def active(self, segment: dict[str, object]) -> bool:
        return all(condition.matches(segment) for condition in self.conditions)


def condition_groups() -> dict[str, list[Condition]]:
    return {
        "lag": [Condition("stamp_lag_sec_mean", "ge", value) for value in (0.015, 0.020, 0.025, 0.030, 0.033, 0.040, 0.045)],
        "core_diff_le": [Condition("state_core_gnss_diff_h_m_mean", "le", value) for value in (0.5, 0.8, 1.1, 1.4)],
        "core_diff_ge": [Condition("state_core_gnss_diff_h_m_mean", "ge", value) for value in (0.8, 1.1, 1.4)],
        "cg_along_le": [
            Condition("core_gnss_along_velocity_m_mean", "le", value)
            for value in (-1.0, -0.5, 0.0, 0.5, 0.75, 0.80, 0.85, 0.90, 0.95, 1.0)
        ],
        "cg_along_ge": [
            Condition("core_gnss_along_velocity_m_mean", "ge", value)
            for value in (-0.5, 0.0, 0.5, 0.75, 0.90, 1.0)
        ],
        "cg_cross_abs": [Condition("core_gnss_cross_velocity_m_mean", "abs_le", value) for value in (0.5, 1.0, 1.5, 2.0)],
        "res_h_le": [Condition("latest_gnss_residual_h_m_mean", "le", value) for value in (0.5, 1.0, 1.5, 2.0)],
        "res_along_abs": [Condition("latest_gnss_residual_along_velocity_m_mean", "abs_le", value) for value in (0.5, 1.0, 1.5, 2.0)],
        "dx_over": [Condition("latest_dx_over_residual_h_mean", "le", value) for value in (0.03, 0.05, 0.08, 0.12, 0.18)],
        "proj_along_abs": [Condition("projection_delta_along_velocity_m_mean", "abs_ge", value) for value in (0.02, 0.04, 0.06, 0.08)],
        "proj_cross_abs": [Condition("projection_delta_cross_velocity_m_mean", "abs_ge", value) for value in (0.02, 0.04, 0.06, 0.08)],
        "proj_cross_le": [
            Condition("projection_delta_cross_velocity_m_mean", "le", value)
            for value in (-0.12, -0.10, -0.08, -0.06, -0.04, -0.02, 0.0, 0.02, 0.04)
        ],
        "proj_cross_ge": [
            Condition("projection_delta_cross_velocity_m_mean", "ge", value)
            for value in (-0.04, -0.02, 0.0, 0.02, 0.04, 0.06)
        ],
        "speed": [Condition("core_speed_mps_mean", "ge", value) for value in (1.0, 2.0, 3.0)],
        "gyro": [Condition("gyro_deg_s_mean", "le", value) for value in (5.0, 10.0, 20.0)],
        "phase": [
            Condition("turning_now_mean", "ge", 0.25),
            Condition("post_turn_context_mean", "ge", 0.25),
            Condition("armed_cruise_context_mean", "ge", 0.50),
        ],
    }


def candidate_name(conditions: Sequence[Condition]) -> str:
    if not conditions:
        return "always_px4"
    return "__".join(condition.label() for condition in conditions)


def build_candidates() -> list[Candidate]:
    groups = condition_groups()
    candidates: list[Candidate] = [Candidate("always_px4", tuple())]

    all_conditions = [condition for group in groups.values() for condition in group]
    candidates.extend(Candidate(candidate_name((condition,)), (condition,)) for condition in all_conditions)

    non_lag = [condition for name, group in groups.items() if name != "lag" for condition in group]
    for lag in groups["lag"]:
        for condition in non_lag:
            candidates.append(Candidate(candidate_name((lag, condition)), (lag, condition)))

    primary = groups["core_diff_le"] + groups["core_diff_ge"] + groups["cg_along_le"] + groups["cg_along_ge"]
    guards = (
        groups["res_h_le"]
        + groups["res_along_abs"]
        + groups["dx_over"]
        + groups["proj_along_abs"]
        + groups["proj_cross_abs"]
        + groups["proj_cross_le"]
        + groups["proj_cross_ge"]
        + groups["phase"]
    )
    for first in primary:
        for guard in guards:
            if first.feature != guard.feature:
                candidates.append(Candidate(candidate_name((first, guard)), (first, guard)))

    unique: dict[str, Candidate] = {}
    for candidate in candidates:
        unique[candidate.name] = candidate
    return list(unique.values())


def aggregate_candidate(
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
        active = candidate.active(segment)
        chosen = px4 if active else raw
        active_rows += n if active else 0
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


def aggregate_oracle(segments: Sequence[dict[str, object]]) -> dict[str, object]:
    rows = sum(int(to_float(segment.get("rows"), 0.0)) for segment in segments)
    if rows <= 0:
        return {"rows": 0, "oracle_rmse_m": math.nan, "oracle_minus_ekf2_m": math.nan}
    total = 0.0
    ekf2_total = 0.0
    active_rows = 0
    for segment in segments:
        n = int(to_float(segment.get("rows"), 0.0))
        raw = to_float(segment.get("raw_rmse_m"))
        px4 = to_float(segment.get("px4_rmse_m"))
        ekf2 = to_float(segment.get("ekf2_rmse_m"))
        chosen = min(raw, px4)
        active_rows += n if px4 < raw else 0
        total += chosen * chosen * n
        ekf2_total += ekf2 * ekf2 * n
    oracle = math.sqrt(total / rows)
    ekf2 = math.sqrt(ekf2_total / rows)
    return {"rows": rows, "oracle_rmse_m": oracle, "oracle_minus_ekf2_m": oracle - ekf2, "oracle_px4_frac": active_rows / rows}


def candidate_window_metrics(
    segments: Sequence[dict[str, object]],
    candidate: Candidate,
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for spec in RUNS:
        for window, _start, _end in WINDOWS:
            subset = segments_for_window(segments, spec.label, window)
            row = aggregate_candidate(subset, candidate)
            row.update(
                {
                    "candidate": candidate.name,
                    "condition_count": len(candidate.conditions),
                    "run": spec.label,
                    "group": spec.group,
                    "window": window,
                }
            )
            out.append(row)
    return out


def oracle_window_summary(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for spec in RUNS:
        for window, _start, _end in WINDOWS:
            subset = segments_for_window(segments, spec.label, window)
            base = aggregate_candidate(subset, Candidate("always_px4", tuple()))
            oracle = aggregate_oracle(subset)
            row = {
                "run": spec.label,
                "group": spec.group,
                "window": window,
                "rows": base["rows"],
                "raw_rmse_m": base["raw_rmse_m"],
                "px4_rmse_m": base["px4_rmse_m"],
                "ekf2_rmse_m": base["ekf2_rmse_m"],
                "oracle_rmse_m": oracle["oracle_rmse_m"],
                "raw_minus_ekf2_m": to_float(base["raw_rmse_m"]) - to_float(base["ekf2_rmse_m"]),
                "px4_minus_ekf2_m": to_float(base["px4_rmse_m"]) - to_float(base["ekf2_rmse_m"]),
                "oracle_minus_ekf2_m": oracle["oracle_minus_ekf2_m"],
                "oracle_px4_frac": oracle["oracle_px4_frac"],
            }
            out.append(row)
    return out


def summarize_candidate(
    segments: Sequence[dict[str, object]],
    candidate: Candidate,
) -> dict[str, object]:
    scores: dict[tuple[str, str], dict[str, object]] = {}
    for spec in RUNS:
        for window, _start, _end in WINDOWS:
            scores[(spec.label, window)] = aggregate_candidate(segments_for_window(segments, spec.label, window), candidate)
    positive_regs = [to_float(scores[(run, "main_40_180")]["candidate_minus_raw_m"]) for run in POSITIVE_RUNS]
    positive_gaps = [to_float(scores[(run, "main_40_180")]["candidate_minus_ekf2_m"]) for run in POSITIVE_RUNS]
    target_140_gaps = [to_float(scores[(run, "140_160")]["candidate_minus_ekf2_m"]) for run in TARGET_RUNS]
    target_160_gaps = [to_float(scores[(run, "160_180")]["candidate_minus_ekf2_m"]) for run in TARGET_RUNS]
    target_140_improve = [
        to_float(scores[(run, "140_160")]["raw_rmse_m"]) - to_float(scores[(run, "140_160")]["candidate_rmse_m"])
        for run in TARGET_RUNS
    ]
    target_main_gaps = [to_float(scores[(run, "main_40_180")]["candidate_minus_ekf2_m"]) for run in TARGET_RUNS]
    protected = max(positive_regs) <= 0.02 and max(positive_gaps) < 0.02
    return {
        "candidate": candidate.name,
        "condition_count": len(candidate.conditions),
        "positive_max_main_regress_m": max(positive_regs),
        "positive_max_candidate_minus_ekf2_m": max(positive_gaps),
        "positive_all_protected": int(protected),
        "target_worst_main_candidate_minus_ekf2_m": max(target_main_gaps),
        "target_worst_140_160_candidate_minus_ekf2_m": max(target_140_gaps),
        "target_worst_160_180_candidate_minus_ekf2_m": max(target_160_gaps),
        "target_mean_140_160_improve_m": mean(target_140_improve),
        "target_min_140_160_improve_m": min(target_140_improve),
        "all_targets_140_160_2cm": int(protected and max(target_140_gaps) <= 0.02),
        "all_targets_140_180_2cm": int(protected and max(target_140_gaps + target_160_gaps) <= 0.02),
    }


def top_candidates(summary_rows: list[dict[str, object]], protected: bool, limit: int = 12) -> list[dict[str, object]]:
    rows = [
        row
        for row in summary_rows
        if not protected or to_float(row.get("positive_all_protected"), 0.0) > 0.5
    ]
    rows.sort(
        key=lambda row: (
            to_float(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
            to_float(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
            to_float(row.get("positive_max_main_regress_m")),
            -to_float(row.get("target_mean_140_160_improve_m")),
        )
    )
    return rows[:limit]


def feature_contrast(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    target_help = [
        segment
        for segment in segments
        if segment.get("run") in TARGET_RUNS
        and 140.0 <= to_float(segment.get("start_sec")) < 180.0
        and to_float(segment.get("projection_benefit_rmse_m")) > 0.02
    ]
    positive_harm = [
        segment
        for segment in segments
        if segment.get("run") in POSITIVE_RUNS
        and to_float(segment.get("projection_benefit_rmse_m")) < -0.02
    ]
    rows: list[dict[str, object]] = []
    for feature in CANDIDATE_FEATURES:
        help_values = [to_float(segment.get(feature)) for segment in target_help]
        harm_values = [to_float(segment.get(feature)) for segment in positive_harm]
        help_mean = mean(help_values)
        harm_mean = mean(harm_values)
        help_var = mean((value - help_mean) ** 2 for value in help_values if finite(value) and finite(help_mean))
        harm_var = mean((value - harm_mean) ** 2 for value in harm_values if finite(value) and finite(harm_mean))
        pooled = math.sqrt(help_var + harm_var) if finite(help_var) and finite(harm_var) else math.nan
        rows.append(
            {
                "feature": feature,
                "target_help_count": len([value for value in help_values if finite(value)]),
                "positive_harm_count": len([value for value in harm_values if finite(value)]),
                "target_help_mean": help_mean,
                "positive_harm_mean": harm_mean,
                "mean_diff_target_help_minus_positive_harm": help_mean - harm_mean,
                "separation_score": abs(help_mean - harm_mean) / pooled if finite(pooled) and pooled > 1e-9 else math.nan,
            }
        )
    rows.sort(key=lambda row: to_float(row.get("separation_score")), reverse=True)
    return rows


def selected_window_metric_rows(
    metrics: Sequence[dict[str, object]],
    names: set[str],
) -> list[dict[str, object]]:
    keep_windows = {"main_40_180", "140_160", "160_180"}
    return [
        row
        for row in metrics
        if row.get("candidate") in names
        and row.get("window") in keep_windows
        and (row.get("group") == "target" or row.get("window") == "main_40_180")
    ]


def report_table_window(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    focus = [
        row
        for row in rows
        if (row["group"] == "positive" and row["window"] == "main_40_180")
        or (row["group"] == "target" and row["window"] in {"main_40_180", "140_160", "160_180"})
    ]
    return [
        [
            row["run"],
            row["window"],
            fmt(row["raw_iekf_rmse_m"]),
            fmt(row["px4_sphere_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["px4_minus_raw_rmse_m"]),
            fmt(row["stamp_lag_sec_mean"]),
            fmt(row["publish_projection_active_frac"], 3),
        ]
        for row in focus
    ]


def report_table_candidates(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["candidate"],
            row["condition_count"],
            fmt(row["positive_max_main_regress_m"]),
            row["positive_all_protected"],
            fmt(row["target_worst_main_candidate_minus_ekf2_m"]),
            fmt(row["target_worst_140_160_candidate_minus_ekf2_m"]),
            fmt(row["target_worst_160_180_candidate_minus_ekf2_m"]),
            fmt(row["target_mean_140_160_improve_m"]),
            row["all_targets_140_160_2cm"],
        ]
        for row in rows
    ]


def report_table_selected_metrics(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["candidate"],
            row["run"],
            row["window"],
            fmt(row["active_frac"], 3),
            fmt(row["raw_rmse_m"]),
            fmt(row["candidate_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["candidate_minus_raw_m"]),
            fmt(row["candidate_minus_ekf2_m"]),
        ]
        for row in rows
    ]


def report_table_oracle(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    focus = [
        row
        for row in rows
        if (row["group"] == "positive" and row["window"] == "main_40_180")
        or (row["group"] == "target" and row["window"] in {"140_160", "160_180"})
    ]
    return [
        [
            row["run"],
            row["window"],
            fmt(row["raw_rmse_m"]),
            fmt(row["px4_rmse_m"]),
            fmt(row["oracle_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["oracle_px4_frac"], 3),
        ]
        for row in focus
    ]


def write_report(
    out_dir: Path,
    row_windows: list[dict[str, object]],
    oracle_rows: list[dict[str, object]],
    candidate_summaries: list[dict[str, object]],
    selected_metrics: list[dict[str, object]],
    contrast_rows: list[dict[str, object]],
    segment_sec: float,
) -> None:
    protected_top = top_candidates(candidate_summaries, protected=True)
    relaxed_top = top_candidates(candidate_summaries, protected=False)
    strict_all = [row for row in candidate_summaries if to_float(row.get("all_targets_140_160_2cm"), 0.0) > 0.5]
    always = next(row for row in candidate_summaries if row["candidate"] == "always_px4")
    lag025 = next(
        (row for row in candidate_summaries if row["candidate"] == "stamp_lag_sec_ge_0.025"),
        None,
    )
    lag033 = next(
        (row for row in candidate_summaries if row["candidate"] == "stamp_lag_sec_ge_0.033"),
        None,
    )

    contrast_table = [
        [
            row["feature"],
            row["target_help_count"],
            row["positive_harm_count"],
            fmt(row["target_help_mean"]),
            fmt(row["positive_harm_mean"]),
            fmt(row["mean_diff_target_help_minus_positive_harm"]),
            fmt(row["separation_score"]),
        ]
        for row in contrast_rows[:10]
    ]

    key_rows: list[dict[str, object]] = []
    seen_key_candidates: set[str] = set()
    for row in (lag025, lag033, always, protected_top[0] if protected_top else None):
        if row is None or str(row["candidate"]) in seen_key_candidates:
            continue
        seen_key_candidates.add(str(row["candidate"]))
        key_rows.append(row)
    selected_names = {str(row["candidate"]) for row in key_rows}
    selected_display = [row for row in selected_metrics if str(row.get("candidate")) in selected_names]

    lines = [
        "# PHS5 selector replay after lag-only online failure",
        "",
        "Date: 2026-05-10",
        "",
        "Pure offline analysis using existing PHS5 logs and newly generated projection-mode groundtruth tables for the two complete shortgen11 opt-in reruns. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "## Scope",
        "",
        f"- Segment length for selector replay: `{segment_sec:.1f}` s.",
        "- Positive/protection set: shortgen01/02/03 development/control and shortgen04 clean holdout positive.",
        "- Target/generalization-warning set: shortgen11 repeat2, 17:49 lag diagnostic, and the two complete 18:21/18:35 opt-in lag-projection reruns.",
        "- Candidate selectors use timing, phase/context, frame-projection geometry, and measurement/update geometry fields. Groundtruth is used only for scoring and oracle readout.",
        "",
        "## Window Projection Readout",
        "",
        markdown_table(
            ["run", "window", "raw", "PX4 sphere", "EKF2", "PX4-raw", "lag mean", "online proj"],
            report_table_window(row_windows),
        ),
        "",
        "## Oracle Boundary",
        "",
        markdown_table(
            ["run", "window", "raw", "PX4 sphere", "oracle", "EKF2", "oracle PX4 frac"],
            report_table_oracle(oracle_rows),
        ),
        "",
        "## Key Candidate Replay",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "pos max regress",
                "protected",
                "target worst main-EKF2",
                "target worst 140-EKF2",
                "target worst 160-EKF2",
                "target mean 140 improve",
                "all 140 repair",
            ],
            report_table_candidates(key_rows),
        ),
        "",
        "## Top Protected Candidates",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "pos max regress",
                "protected",
                "target worst main-EKF2",
                "target worst 140-EKF2",
                "target worst 160-EKF2",
                "target mean 140 improve",
                "all 140 repair",
            ],
            report_table_candidates(protected_top[:10]),
        ),
        "",
        "## Top Relaxed Candidates",
        "",
        markdown_table(
            [
                "candidate",
                "conds",
                "pos max regress",
                "protected",
                "target worst main-EKF2",
                "target worst 140-EKF2",
                "target worst 160-EKF2",
                "target mean 140 improve",
                "all 140 repair",
            ],
            report_table_candidates(relaxed_top[:10]),
        ),
        "",
        "## Selected Window Metrics",
        "",
        markdown_table(
            [
                "candidate",
                "run",
                "window",
                "active",
                "raw",
                "candidate",
                "EKF2",
                "candidate-raw",
                "candidate-EKF2",
            ],
            report_table_selected_metrics(selected_display),
        ),
        "",
        "## Target Help vs Positive Harm Feature Contrast",
        "",
        markdown_table(
            ["feature", "target help n", "positive harm n", "target help mean", "positive harm mean", "diff", "score"],
            contrast_table,
        ),
        "",
        "## Readout",
        "",
    ]
    if strict_all:
        best = top_candidates(strict_all, protected=False, limit=1)[0]
        lines.append(
            f"- A protected selector repairs every target `140_160` window within the 0.02 m EKF2 tolerance in this grid: `{best['candidate']}`. Treat this as offline diagnostic only until segment-length sensitivity and independent holdouts confirm it."
        )
    else:
        lines.append(
            "- No tested protected selector repairs every shortgen11 target `140_160` window within the 0.02 m EKF2 tolerance. This keeps the mechanism in offline diagnosis, not keeper status."
        )
    lines.extend(
        [
            f"- Global PX4-sphere remains an oracle-relevant action but is not safe as default: positive max main-window regression is `{fmt(always['positive_max_main_regress_m'])}` m.",
            "- Lag-only is not a necessary condition for the shortgen11 failure: the two complete opt-in reruns keep poor 140-160/160-180 behavior while their online projection active fraction and row lag are low in the same windows.",
            "- The next useful offline work is to reduce the candidate grid to a small hand-auditable selector family using the feature-contrast rows, then test that family across segment lengths and route leave-one-out before any new online run.",
            "",
            "Generated files:",
            f"- `{out_dir / 'geometry_row_metrics.csv'}`",
            f"- `{out_dir / 'window_projection_summary.csv'}`",
            f"- `{out_dir / 'segment_projection_summary.csv'}`",
            f"- `{out_dir / 'oracle_window_summary.csv'}`",
            f"- `{out_dir / 'segment_selector_candidate_summary.csv'}`",
            f"- `{out_dir / 'selected_segment_selector_window_metrics.csv'}`",
            f"- `{out_dir / 'target_help_vs_positive_harm_feature_contrast.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    parser.add_argument("--segment-sec", type=float, default=2.0)
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    rows = load_all_rows()
    write_csv(out_dir / "geometry_row_metrics.csv", rows)

    row_windows = window_projection_summary(rows)
    write_csv(out_dir / "window_projection_summary.csv", row_windows)

    segments = build_segments(rows, args.segment_sec)
    write_csv(out_dir / "segment_projection_summary.csv", segments)

    oracle_rows = oracle_window_summary(segments)
    write_csv(out_dir / "oracle_window_summary.csv", oracle_rows)

    candidates = build_candidates()
    candidate_summaries = [summarize_candidate(segments, candidate) for candidate in candidates]
    write_csv(out_dir / "segment_selector_candidate_summary.csv", candidate_summaries)

    selected_names = {
        "always_px4",
        "stamp_lag_sec_ge_0.025",
        "stamp_lag_sec_ge_0.033",
        *(str(row["candidate"]) for row in top_candidates(candidate_summaries, protected=True, limit=3)),
        *(str(row["candidate"]) for row in top_candidates(candidate_summaries, protected=False, limit=3)),
    }
    selected_candidates = [candidate for candidate in candidates if candidate.name in selected_names]
    selected_metrics: list[dict[str, object]] = []
    for candidate in selected_candidates:
        selected_metrics.extend(candidate_window_metrics(segments, candidate))
    selected_metrics = selected_window_metric_rows(selected_metrics, selected_names)
    write_csv(out_dir / "selected_segment_selector_window_metrics.csv", selected_metrics)

    contrast_rows = feature_contrast(segments)
    write_csv(out_dir / "target_help_vs_positive_harm_feature_contrast.csv", contrast_rows)

    write_report(
        out_dir,
        row_windows,
        oracle_rows,
        candidate_summaries,
        selected_metrics,
        contrast_rows,
        args.segment_sec,
    )
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
