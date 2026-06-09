#!/usr/bin/env python3
"""Cross-route PHS5 selector scoring with accbias-z counterexamples.

This is offline-only. Candidate gates use online-visible fields; groundtruth and
projection-mode oracle values are used only for scoring.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_measurement_geometry_selector as mg


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_OUT = BASE / "phs5_cross_route_accbias_selector_2026-05-11"


@dataclass(frozen=True)
class RunSpec:
    label: str
    role: str
    group: str
    path: Path
    mode_subdir: str


RUNS = [
    RunSpec(
        "shortgen01_phs5c",
        "development/control positive",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_phs5c_publish_core_stamp_bias_m003_20260510_003839"),
        "offline_groundtruth_projection_modes_phs_check",
    ),
    RunSpec(
        "shortgen02_phs5b",
        "development/control positive",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_phs5b_publish_core_stamp_bias_m003_20260510_003144"),
        "offline_groundtruth_projection_modes_phs_check",
    ),
    RunSpec(
        "shortgen03_phs5a",
        "development/control positive",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_phs5a_publish_core_stamp_bias_m003_20260510_002551"),
        "offline_groundtruth_projection_modes_phs_check",
    ),
    RunSpec(
        "shortgen04_hld1a_phs5",
        "clean holdout positive with local miss",
        "positive",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_hld1a_phs5_publish_core_stamp_bias_m003_20260510_005528"),
        "offline_groundtruth_projection_modes_phs_check",
    ),
    RunSpec(
        "shortgen11_repeat2",
        "clean negative/generalization warning",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"),
        "offline_groundtruth_projection_modes",
    ),
    RunSpec(
        "shortgen11_seg2lagdiag_174924",
        "lag diagnostic, no online projection keeper",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_seg2lagdiag_20260510_174924"),
        "offline_groundtruth_projection_modes",
    ),
    RunSpec(
        "shortgen11_seg2lagproj025_182101",
        "complete opt-in lag projection rerun",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_seg2lagproj025_20260510_182101"),
        "offline_groundtruth_projection_modes",
    ),
    RunSpec(
        "shortgen11_seg2lagproj025_183529",
        "complete opt-in lag projection rerun after action-state fix",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_seg2lagproj025_20260510_183529"),
        "offline_groundtruth_projection_modes",
    ),
    RunSpec(
        "shortgen11_accbiasz_diag",
        "default-off accbias-z diagnostic futureclamp",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_accbiasz_diag_futureclamp_20260511_111411"),
        "offline_groundtruth_projection_modes",
    ),
    RunSpec(
        "shortgen11_accbiasz_apply",
        "opt-in scalar accbias-z Q action futureclamp",
        "target",
        Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_accbiasz_apply_futureclamp_20260511_112407"),
        "offline_groundtruth_projection_modes",
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

POSITIVE_RUNS = [run.label for run in RUNS if run.group == "positive"]
TARGET_RUNS = [run.label for run in RUNS if run.group == "target"]

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
    "accbias_z_before_mps2",
    "accbias_z_delta_mps2",
    "accbias_z_deep_flag",
    "accbias_z_moderate_flag",
    "accbias_z_recent20_deep_frac",
    "accbias_z_history_deep_frac",
]

CANDIDATE_FEATURES = [f"{feature}_mean" for feature in SEGMENT_FEATURES]


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


def latest_before(
    rows: Sequence[dict[str, object]],
    times: Sequence[float],
    t: float,
    max_age_sec: float,
) -> dict[str, object] | None:
    if not rows or not finite(t):
        return None
    idx = bisect.bisect_right(times, t) - 1
    if idx < 0:
        return None
    age = t - times[idx]
    return rows[idx] if 0.0 <= age <= max_age_sec else None


def load_update_rows(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    rows: list[dict[str, object]] = []
    path = run_dir / "state_update_debug.csv"
    if not path.exists():
        return rows, []
    for raw in read_csv(path):
        if raw.get("event_type") != "gnss_position" or to_float(raw.get("applied"), 0.0) <= 0.5:
            continue
        ba_z = to_float(raw.get("accbias_z_before_mps2"))
        row = {
            "ros_time_sec": to_float(raw.get("ros_time_sec")),
            "armed_time_sec": to_float(raw.get("armed_time_sec")),
            "accbias_z_before_mps2": ba_z,
            "accbias_z_after_mps2": to_float(raw.get("accbias_z_after_mps2")),
            "accbias_z_delta_mps2": to_float(raw.get("accbias_z_after_mps2")) - ba_z,
            "accbias_z_deep_flag": 1.0 if finite(ba_z) and ba_z <= -0.205 else 0.0,
            "accbias_z_moderate_flag": 1.0 if finite(ba_z) and ba_z <= -0.18 else 0.0,
            "cov_ba_z_before_m2ps4": to_float(raw.get("cov_ba_z_before_m2ps4")),
        }
        if finite(row["ros_time_sec"]):
            rows.append(row)
    rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    return rows, [to_float(row.get("ros_time_sec")) for row in rows]


def add_bias_history(rows: list[dict[str, object]]) -> None:
    by_run: dict[str, list[dict[str, object]]] = {}
    for row in rows:
        by_run.setdefault(str(row.get("run")), []).append(row)
    for run_rows in by_run.values():
        run_rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
        for idx, row in enumerate(run_rows):
            t = to_float(row.get("time_since_arm_sec"))
            prior = [
                item
                for item in run_rows[:idx]
                if 40.0 <= to_float(item.get("time_since_arm_sec")) < t
                and finite(item.get("accbias_z_before_mps2"))
            ]
            recent = [
                item
                for item in prior
                if t - to_float(item.get("time_since_arm_sec")) <= 20.0
            ]
            row["accbias_z_history_deep_frac"] = mean(
                1.0 if to_float(item.get("accbias_z_before_mps2")) <= -0.205 else 0.0
                for item in prior
            )
            row["accbias_z_recent20_deep_frac"] = mean(
                1.0 if to_float(item.get("accbias_z_before_mps2")) <= -0.205 else 0.0
                for item in recent
            )


def load_run_rows(spec: RunSpec) -> list[dict[str, object]]:
    rows = mg.load_rows(
        mg.RunSpec(
            label=spec.label,
            role=spec.role,
            kind=spec.group,
            path=spec.path,
            mode_subdir=spec.mode_subdir,
            evaluation_source="local",
        )
    )
    update_rows, update_times = load_update_rows(spec.path)
    for row in rows:
        row["group"] = spec.group
        row["role"] = spec.role
        update = latest_before(update_rows, update_times, to_float(row.get("pair_ros_time_sec")), 0.7)
        if update is None:
            continue
        for key in (
            "accbias_z_before_mps2",
            "accbias_z_after_mps2",
            "accbias_z_delta_mps2",
            "accbias_z_deep_flag",
            "accbias_z_moderate_flag",
            "cov_ba_z_before_m2ps4",
        ):
            row[key] = update.get(key, math.nan)
    return rows


def load_all_rows() -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for spec in RUNS:
        rows.extend(load_run_rows(spec))
    rows.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    add_bias_history(rows)
    return rows


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def segment_key(row: dict[str, object], segment_sec: float) -> tuple[str, int]:
    t = to_float(row.get("time_since_arm_sec"))
    return str(row.get("run")), int(math.floor((t - 40.0) / segment_sec))


def run_group(label: str) -> str:
    return next((spec.group for spec in RUNS if spec.label == label), "")


def run_role(label: str) -> str:
    return next((spec.role for spec in RUNS if spec.label == label), "")


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
        "raw_minus_ekf2_m": raw - ekf2,
        "px4_minus_raw_rmse_m": px4 - raw,
        "px4_minus_ekf2_m": px4 - ekf2,
        "projection_benefit_rmse_m": raw - px4,
        "projection_class": "help" if raw - px4 > 0.02 else "harm" if px4 - raw > 0.02 else "neutral",
    }
    for feature in SEGMENT_FEATURES:
        segment[f"{feature}_mean"] = mean(to_float(row.get(feature)) for row in rows)
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


def segments_for_window(segments: Sequence[dict[str, object]], run: str, window: str) -> list[dict[str, object]]:
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
        base = self.feature.replace("_mean", "")
        if self.op == "abs_le":
            return f"abs_{base}_le_{self.threshold:.3f}"
        if self.op == "abs_ge":
            return f"abs_{base}_ge_{self.threshold:.3f}"
        return f"{base}_{self.op}_{self.threshold:.3f}"


@dataclass(frozen=True)
class Candidate:
    name: str
    conditions: tuple[Condition, ...]

    def active(self, segment: dict[str, object]) -> bool:
        return all(condition.matches(segment) for condition in self.conditions)


def candidate_name(conditions: Sequence[Condition]) -> str:
    if not conditions:
        return "always_px4"
    return "__".join(condition.label() for condition in conditions)


def condition_groups() -> dict[str, list[Condition]]:
    return {
        "lag": [Condition("stamp_lag_sec_mean", "ge", value) for value in (0.015, 0.020, 0.025, 0.033, 0.040)],
        "core_age": [Condition("core_age_sec_mean", "ge", value) for value in (0.02, 0.04, 0.06)],
        "core_diff": [Condition("state_core_gnss_diff_h_m_mean", "le", value) for value in (0.8, 1.1, 1.4)],
        "core_along": [
            Condition("core_gnss_along_velocity_m_mean", "le", value)
            for value in (-0.5, 0.0, 0.5, 0.8, 1.0)
        ],
        "proj_cross": [
            Condition("projection_delta_cross_velocity_m_mean", "le", value)
            for value in (-0.10, -0.06, -0.02, 0.0, 0.02)
        ],
        "resid": [Condition("latest_gnss_residual_h_m_mean", "le", value) for value in (0.18, 0.22, 0.30, 0.50)],
        "dx_gain": [Condition("latest_dx_over_residual_h_mean", "le", value) for value in (0.15, 0.18, 0.22, 0.30)],
        "phase": [
            Condition("turning_now_mean", "le", 0.25),
            Condition("post_turn_context_mean", "ge", 0.25),
            Condition("armed_cruise_context_mean", "ge", 0.40),
        ],
        "accbias": [
            Condition("accbias_z_before_mps2_mean", "le", -0.18),
            Condition("accbias_z_before_mps2_mean", "le", -0.205),
            Condition("accbias_z_moderate_flag_mean", "ge", 0.40),
            Condition("accbias_z_deep_flag_mean", "ge", 0.40),
            Condition("accbias_z_recent20_deep_frac_mean", "ge", 0.40),
            Condition("accbias_z_history_deep_frac_mean", "ge", 0.40),
        ],
    }


def build_candidates() -> list[Candidate]:
    groups = condition_groups()
    candidates = [Candidate("always_px4", tuple())]
    all_conditions = [condition for group in groups.values() for condition in group]
    candidates.extend(Candidate(candidate_name((condition,)), (condition,)) for condition in all_conditions)

    primary = groups["accbias"] + groups["lag"] + groups["core_along"] + groups["proj_cross"]
    guards = groups["phase"] + groups["resid"] + groups["dx_gain"] + groups["core_diff"] + groups["core_age"]
    for first in primary:
        for guard in guards:
            if first.feature != guard.feature:
                candidates.append(Candidate(candidate_name((first, guard)), (first, guard)))

    for accbias in groups["accbias"]:
        for lag in groups["lag"]:
            candidates.append(Candidate(candidate_name((accbias, lag)), (accbias, lag)))
        for geom in groups["core_along"] + groups["proj_cross"]:
            candidates.append(Candidate(candidate_name((accbias, geom)), (accbias, geom)))

    unique: dict[str, Candidate] = {}
    for candidate in candidates:
        unique[candidate.name] = candidate
    return list(unique.values())


def aggregate_candidate(segments: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
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
    totals = {"raw": 0.0, "px4": 0.0, "ekf2": 0.0, "candidate": 0.0}
    active_rows = 0
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
        return {"oracle_rmse_m": math.nan, "oracle_px4_frac": math.nan}
    total = 0.0
    active_rows = 0
    for segment in segments:
        n = int(to_float(segment.get("rows"), 0.0))
        raw = to_float(segment.get("raw_rmse_m"))
        px4 = to_float(segment.get("px4_rmse_m"))
        chosen = min(raw, px4)
        total += chosen * chosen * n
        active_rows += n if px4 < raw else 0
    return {"oracle_rmse_m": math.sqrt(total / rows), "oracle_px4_frac": active_rows / rows}


def window_projection_summary(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for spec in RUNS:
        for window, _start, _end in WINDOWS:
            subset = segments_for_window(segments, spec.label, window)
            base = aggregate_candidate(subset, Candidate("always_px4", tuple()))
            oracle = aggregate_oracle(subset)
            row = {
                "run": spec.label,
                "group": spec.group,
                "role": spec.role,
                "window": window,
                "rows": base["rows"],
                "raw_rmse_m": base["raw_rmse_m"],
                "px4_rmse_m": base["px4_rmse_m"],
                "ekf2_rmse_m": base["ekf2_rmse_m"],
                "raw_minus_ekf2_m": to_float(base["raw_rmse_m"]) - to_float(base["ekf2_rmse_m"]),
                "px4_minus_raw_m": to_float(base["px4_rmse_m"]) - to_float(base["raw_rmse_m"]),
                "px4_minus_ekf2_m": to_float(base["px4_rmse_m"]) - to_float(base["ekf2_rmse_m"]),
                "oracle_rmse_m": oracle["oracle_rmse_m"],
                "oracle_px4_frac": oracle["oracle_px4_frac"],
            }
            for feature in CANDIDATE_FEATURES:
                row[feature] = mean(to_float(segment.get(feature)) for segment in subset)
            out.append(row)
    return out


def summarize_candidate(segments: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
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
    repeat2_140 = scores[("shortgen11_repeat2", "140_160")]
    repeat2_160 = scores[("shortgen11_repeat2", "160_180")]
    protected = max(positive_regs) <= 0.02 and max(positive_gaps) < 0.02
    repeat2_repaired = to_float(repeat2_140["candidate_minus_ekf2_m"]) <= 0.02
    repeat2_160_safe = to_float(repeat2_160["candidate_minus_raw_m"]) <= 0.02
    return {
        "candidate": candidate.name,
        "condition_count": len(candidate.conditions),
        "positive_max_main_regress_m": max(positive_regs),
        "positive_max_candidate_minus_ekf2_m": max(positive_gaps),
        "positive_all_protected": int(protected),
        "repeat2_140_candidate_minus_ekf2_m": repeat2_140["candidate_minus_ekf2_m"],
        "repeat2_140_improve_m": to_float(repeat2_140["raw_rmse_m"]) - to_float(repeat2_140["candidate_rmse_m"]),
        "repeat2_140_active_frac": repeat2_140["active_frac"],
        "repeat2_160_candidate_minus_raw_m": repeat2_160["candidate_minus_raw_m"],
        "repeat2_160_candidate_minus_ekf2_m": repeat2_160["candidate_minus_ekf2_m"],
        "target_worst_140_candidate_minus_ekf2_m": max(target_140_gaps),
        "target_worst_160_candidate_minus_ekf2_m": max(target_160_gaps),
        "target_mean_140_improve_m": mean(target_140_improve),
        "strict_repeat2_pass": int(protected and repeat2_repaired and repeat2_160_safe),
        "strict_all_target_pass": int(protected and max(target_140_gaps) <= 0.02 and max(target_160_gaps) <= 0.02),
    }


def candidate_window_metrics(segments: Sequence[dict[str, object]], candidate: Candidate) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for spec in RUNS:
        for window, _start, _end in WINDOWS:
            row = aggregate_candidate(segments_for_window(segments, spec.label, window), candidate)
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


def top_candidates(summary_rows: list[dict[str, object]], protected: bool, limit: int = 12) -> list[dict[str, object]]:
    rows = [
        row
        for row in summary_rows
        if not protected or to_float(row.get("positive_all_protected"), 0.0) > 0.5
    ]
    rows.sort(
        key=lambda row: (
            to_float(row.get("repeat2_140_candidate_minus_ekf2_m")),
            to_float(row.get("target_worst_140_candidate_minus_ekf2_m")),
            to_float(row.get("repeat2_160_candidate_minus_raw_m")),
            to_float(row.get("positive_max_main_regress_m")),
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


def selected_window_metric_rows(rows: Sequence[dict[str, object]], names: set[str]) -> list[dict[str, object]]:
    return [
        row
        for row in rows
        if row.get("candidate") in names
        and (
            (row.get("group") == "positive" and row.get("window") == "main_40_180")
            or (row.get("group") == "target" and row.get("window") in {"main_40_180", "140_160", "160_180"})
        )
    ]


def report_table_windows(rows: Sequence[dict[str, object]]) -> list[list[object]]:
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
            fmt(row["raw_rmse_m"]),
            fmt(row["px4_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["px4_minus_raw_m"]),
            fmt(row["oracle_rmse_m"]),
            fmt(row["oracle_px4_frac"], 3),
            fmt(row["accbias_z_before_mps2_mean"]),
            fmt(row["stamp_lag_sec_mean"]),
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
            fmt(row["repeat2_140_candidate_minus_ekf2_m"]),
            fmt(row["repeat2_140_improve_m"]),
            fmt(row["repeat2_140_active_frac"], 3),
            fmt(row["repeat2_160_candidate_minus_raw_m"]),
            fmt(row["target_worst_140_candidate_minus_ekf2_m"]),
            row["strict_repeat2_pass"],
            row["strict_all_target_pass"],
        ]
        for row in rows
    ]


def report_table_metrics(rows: Sequence[dict[str, object]]) -> list[list[object]]:
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


def report_table_contrast(rows: Sequence[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["feature"],
            row["target_help_count"],
            row["positive_harm_count"],
            fmt(row["target_help_mean"]),
            fmt(row["positive_harm_mean"]),
            fmt(row["mean_diff_target_help_minus_positive_harm"]),
            fmt(row["separation_score"]),
        ]
        for row in rows[:12]
    ]


def write_report(
    out_dir: Path,
    window_rows: list[dict[str, object]],
    candidate_rows: list[dict[str, object]],
    selected_metrics: list[dict[str, object]],
    contrast_rows: list[dict[str, object]],
    segment_sec: float,
) -> None:
    protected_top = top_candidates(candidate_rows, protected=True)
    relaxed_top = top_candidates(candidate_rows, protected=False)
    strict_repeat2 = [row for row in candidate_rows if to_float(row.get("strict_repeat2_pass"), 0.0) > 0.5]
    strict_all = [row for row in candidate_rows if to_float(row.get("strict_all_target_pass"), 0.0) > 0.5]
    always = next(row for row in candidate_rows if row["candidate"] == "always_px4")
    accbias_only = next(
        (row for row in candidate_rows if row["candidate"] == "accbias_z_before_mps2_le_-0.180"),
        None,
    )

    key_rows: list[dict[str, object]] = []
    seen: set[str] = set()
    for row in (always, accbias_only, protected_top[0] if protected_top else None, relaxed_top[0] if relaxed_top else None):
        if row is not None and str(row["candidate"]) not in seen:
            seen.add(str(row["candidate"]))
            key_rows.append(row)
    selected_names = {str(row["candidate"]) for row in key_rows}
    selected_display = [row for row in selected_metrics if str(row.get("candidate")) in selected_names]

    lines = [
        "# PHS5 cross-route accbias selector scoring",
        "",
        "Date: 2026-05-11",
        "",
        "Offline-only selector scoring. Candidate gates use online-visible timing, motion, measurement geometry, projection geometry, and accbias-z state/history fields. Groundtruth and projection modes are used only as scoring labels.",
        "",
        f"Segment length: `{segment_sec:.1f}` s.",
        "",
        "## Window Projection/Oracle Readout",
        "",
        markdown_table(
            [
                "run",
                "window",
                "raw",
                "PX4 sphere",
                "EKF2",
                "PX4-raw",
                "oracle",
                "oracle PX4",
                "ba_z",
                "lag",
            ],
            report_table_windows(window_rows),
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
                "repeat2 140-EKF2",
                "repeat2 140 improve",
                "repeat2 140 active",
                "repeat2 160 raw delta",
                "target worst 140-EKF2",
                "strict repeat2",
                "strict all targets",
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
                "repeat2 140-EKF2",
                "repeat2 140 improve",
                "repeat2 140 active",
                "repeat2 160 raw delta",
                "target worst 140-EKF2",
                "strict repeat2",
                "strict all targets",
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
                "repeat2 140-EKF2",
                "repeat2 140 improve",
                "repeat2 140 active",
                "repeat2 160 raw delta",
                "target worst 140-EKF2",
                "strict repeat2",
                "strict all targets",
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
            report_table_metrics(selected_display),
        ),
        "",
        "## Target Help vs Positive Harm Feature Contrast",
        "",
        markdown_table(
            ["feature", "target help n", "positive harm n", "target mean", "positive mean", "diff", "score"],
            report_table_contrast(contrast_rows),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict_repeat2:
        best = top_candidates(strict_repeat2, protected=False, limit=1)[0]
        lines.append(
            f"- There are `{len(strict_repeat2)}` protected candidates that repair shortgen11 repeat2 140-160 and do not damage repeat2 160-180 under this offline table. Best: `{best['candidate']}`."
        )
    else:
        lines.append("- No protected candidate repairs shortgen11 repeat2 140-160 while preserving repeat2 160-180 under this grid.")
    if strict_all:
        lines.append(f"- `{len(strict_all)}` candidates also pass the all-target 140/160 gate including the two accbias counterexample runs.")
    else:
        lines.append("- No candidate passes the stricter all-target gate once the two accbias counterexample runs are included.")
    lines.extend(
        [
            f"- Global PX4-sphere remains oracle-relevant but not safe by itself: positive max main-window regression is `{fmt(always['positive_max_main_regress_m'])}` m.",
            "- Treat any passing row here as an offline selector hypothesis only. It still needs segment-length sensitivity and independent holdouts before any default-off online diagnostic/mechanism run.",
            "",
            "Generated files:",
            f"- `{out_dir / 'cross_route_row_metrics.csv'}`",
            f"- `{out_dir / 'cross_route_segment_metrics.csv'}`",
            f"- `{out_dir / 'window_projection_summary.csv'}`",
            f"- `{out_dir / 'candidate_summary.csv'}`",
            f"- `{out_dir / 'selected_window_metrics.csv'}`",
            f"- `{out_dir / 'feature_contrast.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", type=Path, default=DEFAULT_OUT)
    parser.add_argument("--segment-sec", type=float, default=2.0)
    parser.add_argument(
        "--extra-run",
        action="append",
        default=[],
        metavar="LABEL,GROUP,ROLE,PATH,MODE_SUBDIR",
        help="optional additional RunSpec; may be supplied multiple times",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    global POSITIVE_RUNS, TARGET_RUNS
    for item in args.extra_run:
        parts = item.split(",", 4)
        if len(parts) != 5:
            raise ValueError("--extra-run must be LABEL,GROUP,ROLE,PATH,MODE_SUBDIR")
        label, group, role, path, mode_subdir = parts
        RUNS.append(RunSpec(label, role, group, Path(path), mode_subdir))
    POSITIVE_RUNS = [run.label for run in RUNS if run.group == "positive"]
    TARGET_RUNS = [run.label for run in RUNS if run.group == "target"]

    args.out_dir.mkdir(parents=True, exist_ok=True)
    rows = load_all_rows()
    write_csv(args.out_dir / "cross_route_row_metrics.csv", rows)

    segments = build_segments(rows, args.segment_sec)
    write_csv(args.out_dir / "cross_route_segment_metrics.csv", segments)

    window_rows = window_projection_summary(segments)
    write_csv(args.out_dir / "window_projection_summary.csv", window_rows)

    candidates = build_candidates()
    candidate_rows = [summarize_candidate(segments, candidate) for candidate in candidates]
    write_csv(args.out_dir / "candidate_summary.csv", candidate_rows)

    selected_names = {
        "always_px4",
        "accbias_z_before_mps2_le_-0.180",
        *(str(row["candidate"]) for row in top_candidates(candidate_rows, protected=True, limit=4)),
        *(str(row["candidate"]) for row in top_candidates(candidate_rows, protected=False, limit=4)),
    }
    selected_candidates = [candidate for candidate in candidates if candidate.name in selected_names]
    selected_rows: list[dict[str, object]] = []
    for candidate in selected_candidates:
        selected_rows.extend(candidate_window_metrics(segments, candidate))
    selected_rows = selected_window_metric_rows(selected_rows, selected_names)
    write_csv(args.out_dir / "selected_window_metrics.csv", selected_rows)

    contrast_rows = feature_contrast(segments)
    write_csv(args.out_dir / "feature_contrast.csv", contrast_rows)

    write_report(args.out_dir, window_rows, candidate_rows, selected_rows, contrast_rows, args.segment_sec)
    print(f"wrote: {args.out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
