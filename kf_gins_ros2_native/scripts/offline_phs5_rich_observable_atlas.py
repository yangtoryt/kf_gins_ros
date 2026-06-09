#!/usr/bin/env python3
"""Rich PHS5 projection-conflict observable atlas.

This is an offline-only diagnostic. It builds segment observables that were not
included in the first selector grids: projection/velocity/error angle features,
route-frame geometry, and heading-update context. It then tests whether any of
these new observables can separate shortgen11 projection-help segments from
shortgen01/02/03/04 projection-harm segments without running a simulator.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence

import offline_phs5_selector_replay_after_lag_failure as replay


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_INPUT = BASE / "phs5_selector_replay_after_lag_failure_2026-05-10" / "geometry_row_metrics.csv"
DEFAULT_OUT = BASE / "phs5_rich_observable_atlas_2026-05-10"
SEGMENT_LENGTHS = (1.0, 2.0, 5.0, 10.0)

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
RUN_DIRS = {spec.label: spec.path for spec in replay.RUNS}
ORACLE_FAMILIES = {"proj_vs_raw_error", "rawerr_x_context", "rawerr_x_phase"}
ROUTE_DIAG_FAMILIES = {"route_geometry", "route_x_context"}


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


def rmse_sse(sse: float, rows: int) -> float:
    return math.sqrt(sse / rows) if rows > 0 and finite(sse) else math.nan


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


def angle_deg(east: float, north: float) -> float:
    if not finite(east) or not finite(north) or (abs(east) < 1e-9 and abs(north) < 1e-9):
        return math.nan
    return math.degrees(math.atan2(east, north))


def angle_diff_deg(a: float, b: float) -> float:
    if not finite(a) or not finite(b):
        return math.nan
    return (a - b + 180.0) % 360.0 - 180.0


def abs_angle_diff_deg(a: float, b: float) -> float:
    diff = angle_diff_deg(a, b)
    return abs(diff) if finite(diff) else math.nan


def norm(east: float, north: float) -> float:
    return math.hypot(east, north) if finite(east) and finite(north) else math.nan


def unit(east: float, north: float) -> tuple[float, float]:
    length = norm(east, north)
    if not finite(length) or length < 1e-9:
        return math.nan, math.nan
    return east / length, north / length


def cosine_between(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    a_len = norm(a_e, a_n)
    b_len = norm(b_e, b_n)
    if not finite(a_len) or not finite(b_len) or a_len < 1e-9 or b_len < 1e-9:
        return math.nan
    return (a_e * b_e + a_n * b_n) / (a_len * b_len)


def vec_from_along_cross(
    along: float,
    cross: float,
    unit_e: float,
    unit_n: float,
) -> tuple[float, float]:
    if not all(finite(value) for value in (along, cross, unit_e, unit_n)):
        return math.nan, math.nan
    return along * unit_e + cross * unit_n, along * unit_n - cross * unit_e


def load_rows(path: Path) -> list[dict[str, object]]:
    rows: list[dict[str, object]] = []
    for raw in read_csv(path):
        row: dict[str, object] = dict(raw)
        for key in raw:
            if key not in {"run", "role", "group", "mavros_mode", "publish_projection_action"}:
                row[key] = to_float(raw.get(key))
        rows.append(row)
    return rows


def run_arm_stamp_sec(run_dir: Path) -> float:
    pair_path = run_dir / "ekf_iekf_pairs.csv"
    if not pair_path.exists():
        return math.nan
    for row in read_csv(pair_path):
        if str(row.get("mavros_armed", "")).strip().lower() in {"1", "true", "yes"}:
            return to_float(row.get("ekf2_stamp_sec"))
    return math.nan


def load_heading_rows() -> dict[str, list[dict[str, object]]]:
    by_run: dict[str, list[dict[str, object]]] = {}
    for label, run_dir in RUN_DIRS.items():
        path = run_dir / "heading_update_debug.csv"
        if not path.exists() or path.stat().st_size == 0:
            by_run[label] = []
            continue
        arm_stamp = run_arm_stamp_sec(run_dir)
        rows: list[dict[str, object]] = []
        for raw in read_csv(path):
            update_time = to_float(raw.get("update_time_sec"))
            ros_time = to_float(raw.get("ros_time_sec"))
            if finite(update_time) and finite(arm_stamp):
                t_arm = update_time - arm_stamp
            else:
                t_arm = math.nan
            rows.append(
                {
                    "time_since_arm_sec": t_arm,
                    "ros_time_sec": ros_time,
                    "residual_before_abs_deg": abs(to_float(raw.get("residual_before_deg"))),
                    "residual_after_abs_deg": abs(to_float(raw.get("residual_after_deg"))),
                    "yaw_correction_abs_deg": to_float(raw.get("yaw_correction_abs_deg")),
                    "heading_update_underreacted": to_float(raw.get("heading_update_underreacted")),
                    "heading_large_residual_skip_count": to_float(raw.get("heading_large_residual_skip_count")),
                    "post_turn_blocked_count": to_float(raw.get("post_turn_blocked_count")),
                    "armed_cruise_blocked_count": to_float(raw.get("armed_cruise_blocked_count")),
                    "source_yaw_rate_deg_s": to_float(raw.get("source_yaw_rate_deg_s")),
                    "gyro_deg_s": to_float(raw.get("gyro_deg_s")),
                    "used_post_turn_cruise_track": to_float(raw.get("used_post_turn_cruise_track")),
                    "used_armed_cruise_track": to_float(raw.get("used_armed_cruise_track")),
                }
            )
        rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
        by_run[label] = rows
    return by_run


def heading_subset(
    heading_rows: Sequence[dict[str, object]],
    start: float,
    end: float,
) -> list[dict[str, object]]:
    return [
        row for row in heading_rows
        if start <= to_float(row.get("time_since_arm_sec")) < end
    ]


def segment_index(t: float, segment_sec: float) -> int:
    return int(math.floor((t - 40.0) / segment_sec))


def build_segments(
    rows: Sequence[dict[str, object]],
    heading_by_run: dict[str, list[dict[str, object]]],
    segment_sec: float,
) -> list[dict[str, object]]:
    grouped: dict[tuple[str, int], list[dict[str, object]]] = {}
    for row in rows:
        t = to_float(row.get("time_since_arm_sec"))
        if 40.0 <= t < 180.0:
            grouped.setdefault((str(row.get("run")), segment_index(t, segment_sec)), []).append(row)

    out: list[dict[str, object]] = []
    for (run, index), segment_rows in sorted(grouped.items()):
        if len(segment_rows) < 2:
            continue
        segment_rows.sort(key=lambda row: to_float(row.get("time_since_arm_sec")))
        start = 40.0 + index * segment_sec
        end = start + segment_sec
        first = segment_rows[0]
        last = segment_rows[-1]
        group = "positive" if run in POSITIVE_RUNS else "target"

        raw_sse = sum(to_float(row.get("raw_iekf_error_m")) ** 2 for row in segment_rows)
        px4_sse = sum(to_float(row.get("px4_iekf_error_m")) ** 2 for row in segment_rows)
        ekf2_sse = sum(to_float(row.get("ekf2_error_m")) ** 2 for row in segment_rows)
        n = len(segment_rows)

        raw_start_e = to_float(first.get("raw_iekf_x_m"))
        raw_start_n = to_float(first.get("raw_iekf_y_m"))
        raw_end_e = to_float(last.get("raw_iekf_x_m"))
        raw_end_n = to_float(last.get("raw_iekf_y_m"))
        gt_start_e = to_float(first.get("gt_x_m"))
        gt_start_n = to_float(first.get("gt_y_m"))
        gt_end_e = to_float(last.get("gt_x_m"))
        gt_end_n = to_float(last.get("gt_y_m"))

        raw_mid_e = mean(to_float(row.get("raw_iekf_x_m")) for row in segment_rows)
        raw_mid_n = mean(to_float(row.get("raw_iekf_y_m")) for row in segment_rows)
        gt_mid_e = mean(to_float(row.get("gt_x_m")) for row in segment_rows)
        gt_mid_n = mean(to_float(row.get("gt_y_m")) for row in segment_rows)
        proj_e = mean(to_float(row.get("px4_iekf_x_m")) - to_float(row.get("raw_iekf_x_m")) for row in segment_rows)
        proj_n = mean(to_float(row.get("px4_iekf_y_m")) - to_float(row.get("raw_iekf_y_m")) for row in segment_rows)
        raw_err_e = mean(to_float(row.get("raw_iekf_x_m")) - to_float(row.get("gt_x_m")) for row in segment_rows)
        raw_err_n = mean(to_float(row.get("raw_iekf_y_m")) - to_float(row.get("gt_y_m")) for row in segment_rows)
        vel_e = mean(to_float(row.get("core_velocity_e_mps")) for row in segment_rows)
        vel_n = mean(to_float(row.get("core_velocity_n_mps")) for row in segment_rows)
        vel_unit_e, vel_unit_n = unit(vel_e, vel_n)
        residual_vecs = [
            vec_from_along_cross(
                to_float(row.get("latest_gnss_residual_along_velocity_m")),
                to_float(row.get("latest_gnss_residual_cross_velocity_m")),
                *unit(to_float(row.get("core_velocity_e_mps")), to_float(row.get("core_velocity_n_mps"))),
            )
            for row in segment_rows
        ]
        dx_vecs = [
            vec_from_along_cross(
                to_float(row.get("latest_dx_pos_along_velocity_m")),
                to_float(row.get("latest_dx_pos_cross_velocity_m")),
                *unit(to_float(row.get("core_velocity_e_mps")), to_float(row.get("core_velocity_n_mps"))),
            )
            for row in segment_rows
        ]
        residual_e = mean(item[0] for item in residual_vecs)
        residual_n = mean(item[1] for item in residual_vecs)
        dx_e = mean(item[0] for item in dx_vecs)
        dx_n = mean(item[1] for item in dx_vecs)
        core_gnss_e = mean(to_float(row.get("core_minus_gnss_e_m")) for row in segment_rows)
        core_gnss_n = mean(to_float(row.get("core_minus_gnss_n_m")) for row in segment_rows)

        raw_leg_heading = angle_deg(raw_end_e - raw_start_e, raw_end_n - raw_start_n)
        gt_leg_heading = angle_deg(gt_end_e - gt_start_e, gt_end_n - gt_start_n)
        velocity_heading = angle_deg(vel_e, vel_n)
        projection_heading = angle_deg(proj_e, proj_n)
        raw_error_heading = angle_deg(raw_err_e, raw_err_n)
        residual_heading = angle_deg(residual_e, residual_n)
        dx_heading = angle_deg(dx_e, dx_n)
        core_gnss_heading = angle_deg(core_gnss_e, core_gnss_n)

        heading_rows = heading_subset(heading_by_run.get(run, []), start, end)
        segment: dict[str, object] = {
            "run": run,
            "group": group,
            "segment_sec": segment_sec,
            "segment_index": index,
            "start_sec": start,
            "end_sec": end,
            "rows": n,
            "raw_rmse_m": rmse_sse(raw_sse, n),
            "px4_rmse_m": rmse_sse(px4_sse, n),
            "ekf2_rmse_m": rmse_sse(ekf2_sse, n),
            "px4_minus_raw_rmse_m": rmse_sse(px4_sse, n) - rmse_sse(raw_sse, n),
            "px4_minus_ekf2_rmse_m": rmse_sse(px4_sse, n) - rmse_sse(ekf2_sse, n),
            "raw_minus_ekf2_rmse_m": rmse_sse(raw_sse, n) - rmse_sse(ekf2_sse, n),
            "projection_benefit_rmse_m": rmse_sse(raw_sse, n) - rmse_sse(px4_sse, n),
            "projection_class": (
                "help" if rmse_sse(raw_sse, n) - rmse_sse(px4_sse, n) > 0.02
                else "harm" if rmse_sse(px4_sse, n) - rmse_sse(raw_sse, n) > 0.02
                else "neutral"
            ),
            "raw_dist_home_m": norm(raw_mid_e, raw_mid_n),
            "gt_dist_home_m": norm(gt_mid_e, gt_mid_n),
            "raw_bearing_home_deg": angle_deg(raw_mid_e, raw_mid_n),
            "gt_bearing_home_deg": angle_deg(gt_mid_e, gt_mid_n),
            "raw_leg_heading_deg": raw_leg_heading,
            "gt_leg_heading_deg": gt_leg_heading,
            "velocity_heading_deg": velocity_heading,
            "projection_delta_heading_deg": projection_heading,
            "raw_error_heading_deg": raw_error_heading,
            "residual_heading_deg": residual_heading,
            "dx_heading_deg": dx_heading,
            "core_gnss_heading_deg": core_gnss_heading,
            "raw_leg_length_m": norm(raw_end_e - raw_start_e, raw_end_n - raw_start_n),
            "gt_leg_length_m": norm(gt_end_e - gt_start_e, gt_end_n - gt_start_n),
            "projection_delta_h_m": norm(proj_e, proj_n),
            "raw_error_h_m": norm(raw_err_e, raw_err_n),
            "residual_h_from_vec_m": norm(residual_e, residual_n),
            "dx_h_from_vec_m": norm(dx_e, dx_n),
            "core_gnss_h_from_vec_m": norm(core_gnss_e, core_gnss_n),
            "raw_leg_vs_projection_abs_deg": abs_angle_diff_deg(raw_leg_heading, projection_heading),
            "gt_leg_vs_projection_abs_deg": abs_angle_diff_deg(gt_leg_heading, projection_heading),
            "velocity_vs_projection_abs_deg": abs_angle_diff_deg(velocity_heading, projection_heading),
            "raw_error_vs_projection_abs_deg": abs_angle_diff_deg(raw_error_heading, projection_heading),
            "residual_vs_projection_abs_deg": abs_angle_diff_deg(residual_heading, projection_heading),
            "dx_vs_projection_abs_deg": abs_angle_diff_deg(dx_heading, projection_heading),
            "core_gnss_vs_projection_abs_deg": abs_angle_diff_deg(core_gnss_heading, projection_heading),
            "raw_leg_vs_gt_leg_abs_deg": abs_angle_diff_deg(raw_leg_heading, gt_leg_heading),
            "raw_leg_vs_velocity_abs_deg": abs_angle_diff_deg(raw_leg_heading, velocity_heading),
            "projection_dot_raw_error_cos": cosine_between(proj_e, proj_n, raw_err_e, raw_err_n),
            "projection_dot_residual_cos": cosine_between(proj_e, proj_n, residual_e, residual_n),
            "projection_dot_dx_cos": cosine_between(proj_e, proj_n, dx_e, dx_n),
            "projection_dot_core_gnss_cos": cosine_between(proj_e, proj_n, core_gnss_e, core_gnss_n),
            "heading_update_count": len(heading_rows),
            "heading_residual_before_abs_deg_mean": mean(to_float(row.get("residual_before_abs_deg")) for row in heading_rows),
            "heading_residual_after_abs_deg_mean": mean(to_float(row.get("residual_after_abs_deg")) for row in heading_rows),
            "heading_yaw_correction_abs_deg_mean": mean(to_float(row.get("yaw_correction_abs_deg")) for row in heading_rows),
            "heading_underreacted_frac": mean(
                1.0 if to_float(row.get("heading_update_underreacted")) > 0.5 else 0.0
                for row in heading_rows
            ),
            "heading_large_residual_skip_count_mean": mean(to_float(row.get("heading_large_residual_skip_count")) for row in heading_rows),
            "heading_post_turn_blocked_count_mean": mean(to_float(row.get("post_turn_blocked_count")) for row in heading_rows),
            "heading_armed_cruise_blocked_count_mean": mean(to_float(row.get("armed_cruise_blocked_count")) for row in heading_rows),
            "heading_source_yaw_rate_abs_deg_s_mean": mean(abs(to_float(row.get("source_yaw_rate_deg_s"))) for row in heading_rows),
            "heading_used_post_turn_cruise_track_frac": mean(
                1.0 if to_float(row.get("used_post_turn_cruise_track")) > 0.5 else 0.0
                for row in heading_rows
            ),
            "heading_used_armed_cruise_track_frac": mean(
                1.0 if to_float(row.get("used_armed_cruise_track")) > 0.5 else 0.0
                for row in heading_rows
            ),
        }
        for feature in (
            "stamp_lag_sec",
            "state_core_gnss_diff_h_m",
            "core_gnss_along_velocity_m",
            "core_gnss_cross_velocity_m",
            "latest_gnss_residual_h_m",
            "latest_gnss_residual_along_velocity_m",
            "latest_gnss_residual_cross_velocity_m",
            "latest_dx_pos_h_m",
            "latest_dx_over_residual_h",
            "projection_delta_along_velocity_m",
            "projection_delta_cross_velocity_m",
            "core_speed_mps",
            "gyro_deg_s",
            "turning_now",
            "post_turn_context",
            "armed_cruise_context",
        ):
            segment[f"{feature}_mean"] = mean(to_float(row.get(feature)) for row in segment_rows)
        out.append(segment)
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
        if self.op == "abs_ge":
            return abs(value) >= self.threshold
        if self.op == "abs_le":
            return abs(value) <= self.threshold
        raise ValueError(self.op)

    def label(self) -> str:
        return f"{self.field}_{self.op}_{self.threshold:.3f}"


@dataclass(frozen=True)
class Candidate:
    name: str
    family: str
    conditions: tuple[Condition, ...]

    def active(self, row: dict[str, object]) -> bool:
        return all(condition.matches(row) for condition in self.conditions)


def make_candidate(family: str, conditions: Sequence[Condition]) -> Candidate:
    return Candidate("__".join(condition.label() for condition in conditions), family, tuple(conditions))


def build_candidates() -> list[Candidate]:
    angle_ge = lambda field, values: [Condition(field, "ge", value) for value in values]
    angle_le = lambda field, values: [Condition(field, "le", value) for value in values]
    scalar_le = lambda field, values: [Condition(field, "le", value) for value in values]
    scalar_ge = lambda field, values: [Condition(field, "ge", value) for value in values]

    groups: dict[str, list[Condition]] = {
        "proj_vs_raw_error": angle_ge("raw_error_vs_projection_abs_deg", (90.0, 120.0, 150.0)),
        "proj_vs_velocity": angle_le("velocity_vs_projection_abs_deg", (30.0, 60.0, 90.0))
        + angle_ge("velocity_vs_projection_abs_deg", (90.0, 120.0, 150.0)),
        "proj_vs_residual": angle_le("residual_vs_projection_abs_deg", (30.0, 60.0, 90.0))
        + angle_ge("residual_vs_projection_abs_deg", (90.0, 120.0, 150.0)),
        "proj_vs_core_gnss": angle_le("core_gnss_vs_projection_abs_deg", (30.0, 60.0, 90.0))
        + angle_ge("core_gnss_vs_projection_abs_deg", (90.0, 120.0, 150.0)),
        "proj_delta": scalar_ge("projection_delta_h_m", (0.03, 0.06, 0.09, 0.12))
        + scalar_le("projection_delta_h_m", (0.03, 0.06, 0.09, 0.12)),
        "route_geometry": scalar_ge("raw_dist_home_m", (20.0, 40.0, 60.0, 80.0))
        + scalar_le("raw_dist_home_m", (20.0, 40.0, 60.0, 80.0)),
        "heading_context": scalar_ge("heading_update_count", (1.0, 2.0, 4.0))
        + scalar_ge("heading_residual_before_abs_deg_mean", (2.0, 5.0, 10.0))
        + scalar_ge("heading_yaw_correction_abs_deg_mean", (0.5, 1.0, 2.0))
        + scalar_ge("heading_underreacted_frac", (0.25, 0.50)),
        "old_context_guard": scalar_le("core_gnss_along_velocity_m_mean", (0.85, 0.95, 1.05))
        + scalar_le("state_core_gnss_diff_h_m_mean", (0.85, 1.05, 1.25))
        + scalar_le("projection_delta_cross_velocity_m_mean", (-0.06, -0.02, 0.02)),
        "phase_guard": scalar_le("turning_now_mean", (0.25, 0.50))
        + scalar_ge("armed_cruise_context_mean", (0.50, 0.75)),
    }

    candidates: list[Candidate] = []
    for family, conditions in groups.items():
        candidates.extend(make_candidate(family, (condition,)) for condition in conditions)

    pair_families = [
        ("rawerr_x_context", groups["proj_vs_raw_error"], groups["old_context_guard"]),
        ("rawerr_x_phase", groups["proj_vs_raw_error"], groups["phase_guard"]),
        ("velocity_x_context", groups["proj_vs_velocity"], groups["old_context_guard"]),
        ("residual_x_context", groups["proj_vs_residual"], groups["old_context_guard"]),
        ("coregnss_x_context", groups["proj_vs_core_gnss"], groups["old_context_guard"]),
        ("route_x_context", groups["route_geometry"], groups["old_context_guard"]),
        ("heading_x_context", groups["heading_context"], groups["old_context_guard"]),
        ("heading_x_phase", groups["heading_context"], groups["phase_guard"]),
    ]
    for family, first_group, second_group in pair_families:
        for first in first_group:
            for second in second_group:
                candidates.append(make_candidate(family, (first, second)))

    unique: dict[str, Candidate] = {}
    for candidate in candidates:
        unique[f"{candidate.family}:{candidate.name}"] = candidate
    return list(unique.values())


def segment_in_window(segment: dict[str, object], window: str) -> bool:
    start, end = WINDOWS[window]
    return to_float(segment.get("start_sec")) < end and to_float(segment.get("end_sec")) > start


def select_segments(segments: Sequence[dict[str, object]], runs: Sequence[str], window: str) -> list[dict[str, object]]:
    run_set = set(runs)
    return [row for row in segments if row.get("run") in run_set and segment_in_window(row, window)]


def aggregate(segments: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
    rows = sum(int(to_float(row.get("rows"), 0.0)) for row in segments)
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
    for row in segments:
        n = int(to_float(row.get("rows"), 0.0))
        raw = to_float(row.get("raw_rmse_m"))
        px4 = to_float(row.get("px4_rmse_m"))
        ekf2 = to_float(row.get("ekf2_rmse_m"))
        active = candidate.active(row)
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


def score_candidate(segments: Sequence[dict[str, object]], candidate: Candidate) -> dict[str, object]:
    pos = [aggregate(select_segments(segments, [run], "main_40_180"), candidate) for run in POSITIVE_RUNS]
    target_main = [aggregate(select_segments(segments, [run], "main_40_180"), candidate) for run in TARGET_RUNS]
    target_140 = [aggregate(select_segments(segments, [run], "140_160"), candidate) for run in TARGET_RUNS]
    target_160 = [aggregate(select_segments(segments, [run], "160_180"), candidate) for run in TARGET_RUNS]
    pos_reg = [to_float(row.get("candidate_minus_raw_m")) for row in pos]
    pos_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in pos]
    target_140_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in target_140]
    target_160_gap = [to_float(row.get("candidate_minus_ekf2_m")) for row in target_160]
    protected = max(pos_reg) <= 0.02 and max(pos_gap) < 0.02
    return {
        "candidate": candidate.name,
        "family": candidate.family,
        "condition_count": len(candidate.conditions),
        "positive_max_main_regress_m": max(pos_reg),
        "positive_max_candidate_minus_ekf2_m": max(pos_gap),
        "positive_all_protected": int(protected),
        "target_worst_main_candidate_minus_ekf2_m": max(to_float(row.get("candidate_minus_ekf2_m")) for row in target_main),
        "target_worst_140_160_candidate_minus_ekf2_m": max(target_140_gap),
        "target_worst_160_180_candidate_minus_ekf2_m": max(target_160_gap),
        "target_mean_140_160_improve_m": mean(
            to_float(row.get("raw_rmse_m")) - to_float(row.get("candidate_rmse_m")) for row in target_140
        ),
        "target_140_160_pass": int(max(target_140_gap) <= 0.02),
        "target_160_180_pass": int(max(target_160_gap) <= 0.02),
        "strict_pass": int(protected and max(target_140_gap) <= 0.02 and max(target_160_gap) <= 0.02),
    }


def sort_key(row: dict[str, object]) -> tuple[float, float, float, float, float]:
    return (
        0.0 if to_float(row.get("positive_all_protected"), 0.0) > 0.5 else 1.0,
        to_float(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
        to_float(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
        to_float(row.get("positive_max_main_regress_m")),
        -to_float(row.get("target_mean_140_160_improve_m")),
    )


def feature_separation(segments: Sequence[dict[str, object]]) -> list[dict[str, object]]:
    target_help = [
        row for row in segments
        if row.get("run") in TARGET_RUNS
        and 140.0 <= to_float(row.get("start_sec")) < 180.0
        and to_float(row.get("projection_benefit_rmse_m")) > 0.02
    ]
    positive_harm = [
        row for row in segments
        if row.get("run") in POSITIVE_RUNS
        and to_float(row.get("projection_benefit_rmse_m")) < -0.02
    ]
    features = [
        "raw_error_vs_projection_abs_deg",
        "velocity_vs_projection_abs_deg",
        "residual_vs_projection_abs_deg",
        "core_gnss_vs_projection_abs_deg",
        "raw_leg_vs_projection_abs_deg",
        "raw_leg_vs_gt_leg_abs_deg",
        "projection_dot_raw_error_cos",
        "projection_dot_residual_cos",
        "projection_dot_core_gnss_cos",
        "projection_delta_h_m",
        "raw_dist_home_m",
        "raw_bearing_home_deg",
        "heading_update_count",
        "heading_residual_before_abs_deg_mean",
        "heading_yaw_correction_abs_deg_mean",
        "heading_underreacted_frac",
        "heading_source_yaw_rate_abs_deg_s_mean",
        "core_gnss_along_velocity_m_mean",
        "state_core_gnss_diff_h_m_mean",
        "projection_delta_cross_velocity_m_mean",
    ]
    out: list[dict[str, object]] = []
    for feature in features:
        help_values = [to_float(row.get(feature)) for row in target_help]
        harm_values = [to_float(row.get(feature)) for row in positive_harm]
        help_mean = mean(help_values)
        harm_mean = mean(harm_values)
        help_var = mean((value - help_mean) ** 2 for value in help_values if finite(value) and finite(help_mean))
        harm_var = mean((value - harm_mean) ** 2 for value in harm_values if finite(value) and finite(harm_mean))
        pooled = math.sqrt(help_var + harm_var) if finite(help_var) and finite(harm_var) else math.nan
        out.append(
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
    out.sort(key=lambda row: to_float(row.get("separation_score")), reverse=True)
    return out


def table_candidate(rows: Sequence[dict[str, object]]) -> list[list[object]]:
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


def write_report(
    out_dir: Path,
    separation_rows: list[dict[str, object]],
    candidate_rows: list[dict[str, object]],
) -> None:
    strict = [row for row in candidate_rows if to_float(row.get("strict_pass"), 0.0) > 0.5]
    strict.sort(key=sort_key)
    online_safe = [
        row for row in candidate_rows
        if row.get("family") not in ORACLE_FAMILIES and row.get("family") not in ROUTE_DIAG_FAMILIES
    ]
    online_strict = [row for row in online_safe if to_float(row.get("strict_pass"), 0.0) > 0.5]
    online_strict.sort(key=sort_key)
    online_protected_top = [
        row for row in online_safe if to_float(row.get("positive_all_protected"), 0.0) > 0.5
    ]
    online_protected_top.sort(key=sort_key)
    protected_top = [row for row in candidate_rows if to_float(row.get("positive_all_protected"), 0.0) > 0.5]
    protected_top.sort(key=sort_key)
    relaxed_top = sorted(candidate_rows, key=lambda row: (
        to_float(row.get("target_worst_140_160_candidate_minus_ekf2_m")),
        to_float(row.get("target_worst_160_180_candidate_minus_ekf2_m")),
        to_float(row.get("positive_max_main_regress_m")),
    ))
    sep_table = [
        [
            row["feature"],
            row["target_help_count"],
            row["positive_harm_count"],
            fmt(row["target_help_mean"]),
            fmt(row["positive_harm_mean"]),
            fmt(row["mean_diff_target_help_minus_positive_harm"]),
            fmt(row["separation_score"]),
        ]
        for row in separation_rows[:14]
    ]
    lines = [
        "# PHS5 rich observable projection-conflict atlas",
        "",
        "Date: 2026-05-10",
        "",
        "Offline-only analysis using existing row-level PHS5 metrics and heading update debug CSVs. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "## Feature Separation",
        "",
        markdown_table(
            ["feature", "target help n", "positive harm n", "target help mean", "positive harm mean", "diff", "score"],
            sep_table,
        ),
        "",
        "## Top Protected Candidates",
        "",
        "This table includes oracle-diagnostic candidates. Families using `raw_error_vs_projection_*` use groundtruth and are not online deployable.",
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
            table_candidate(protected_top[:12]),
        ),
        "",
        "## Top Online-Safe Protected Candidates",
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
            table_candidate(online_protected_top[:12]),
        ),
        "",
        "## Top Relaxed Candidates",
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
            table_candidate(relaxed_top[:12]),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict:
        best = strict[0]
        lines.append(
            f"- Oracle-diagnostic strict-pass candidates exist; the best is `{best['candidate']}` at `{fmt(best['segment_sec'], 1)}` s. It uses groundtruth raw-error direction, so it is not online deployable."
        )
    else:
        lines.append(
            "- No rich observable candidate in this grid passes both positive protection and all shortgen11 140-180 target repair."
        )
    if online_strict:
        best_online = online_strict[0]
        lines.append(
            f"- An online-safe strict-pass candidate exists: `{best_online['candidate']}` at `{fmt(best_online['segment_sec'], 1)}` s."
        )
    else:
        lines.append(
            "- After removing groundtruth/raw-error and route-overfit families, online-safe strict-pass count is zero."
        )
    lines.extend(
        [
            "- The strongest explanation is geometric: projection helps when the projection delta points opposite the true raw IEKF error. The current online proxies for that direction are still too weak.",
            "- Route-distance/bearing separators are diagnostic only and are likely route-overfit.",
            "- Heading-update features do not separate near the top, so the current projection conflict is primarily frame/error geometry rather than yaw-update state.",
            "",
            "Generated files:",
            f"- `{out_dir / 'rich_segment_observables.csv'}`",
            f"- `{out_dir / 'rich_feature_separation.csv'}`",
            f"- `{out_dir / 'rich_candidate_summary.csv'}`",
            f"- `{out_dir / 'rich_online_safe_candidate_summary.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    rows = load_rows(Path(args.input))
    heading_by_run = load_heading_rows()
    out_dir = Path(args.out_dir)
    candidates = build_candidates()

    all_segments: list[dict[str, object]] = []
    candidate_rows: list[dict[str, object]] = []
    for segment_sec in SEGMENT_LENGTHS:
        segments = build_segments(rows, heading_by_run, segment_sec)
        all_segments.extend(segments)
        for candidate in candidates:
            row = score_candidate(segments, candidate)
            row["segment_sec"] = segment_sec
            candidate_rows.append(row)

    separation_rows = feature_separation([row for row in all_segments if to_float(row.get("segment_sec")) == 2.0])
    online_safe_rows = [
        row for row in candidate_rows
        if row.get("family") not in ORACLE_FAMILIES and row.get("family") not in ROUTE_DIAG_FAMILIES
    ]
    write_csv(out_dir / "rich_segment_observables.csv", all_segments)
    write_csv(out_dir / "rich_feature_separation.csv", separation_rows)
    write_csv(out_dir / "rich_candidate_summary.csv", candidate_rows)
    write_csv(out_dir / "rich_online_safe_candidate_summary.csv", online_safe_rows)
    write_report(out_dir, separation_rows, candidate_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
