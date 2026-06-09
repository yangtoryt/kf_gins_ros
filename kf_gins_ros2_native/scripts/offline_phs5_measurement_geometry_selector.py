#!/usr/bin/env python3
"""Offline PHS5 measurement-geometry selector counterfactual.

This script uses only existing PHS5 artifacts.  It joins groundtruth projection
mode scoring with state publish, GNSS update, and state update debug CSVs, then
tests online-observable segment selectors for when PX4-sphere projection should
be applied or blended.
"""

from __future__ import annotations

import argparse
import bisect
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


DEFAULT_OUT = (
    Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
    / "phs5_measurement_geometry_selector_repeat2_2026-05-10"
)


@dataclass(frozen=True)
class RunSpec:
    label: str
    role: str
    kind: str
    path: Path
    mode_subdir: str | None = None
    evaluation_source: str = "local"


RUNS = [
    RunSpec(
        label="shortgen01_phs5c",
        role="development positive",
        kind="positive",
        path=Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen01_s_bend_sitl_home_headless_compare_core8_pairlogger_phs5c_publish_core_stamp_bias_m003_20260510_003839"),
        mode_subdir="offline_groundtruth_projection_modes_phs_check",
        evaluation_source="local",
    ),
    RunSpec(
        label="shortgen02_phs5b",
        role="development/control positive",
        kind="positive",
        path=Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen02_box_uturn_sitl_home_headless_compare_core8_pairlogger_phs5b_publish_core_stamp_bias_m003_20260510_003144"),
        mode_subdir="offline_groundtruth_projection_modes_phs_check",
        evaluation_source="local",
    ),
    RunSpec(
        label="shortgen03_phs5a",
        role="development/control positive",
        kind="positive",
        path=Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen03_figure8_sitl_home_headless_compare_core8_pairlogger_phs5a_publish_core_stamp_bias_m003_20260510_002551"),
        mode_subdir="offline_groundtruth_projection_modes_phs_check",
        evaluation_source="local",
    ),
    RunSpec(
        label="shortgen04_hld1a_phs5",
        role="clean holdout positive with local miss",
        kind="positive",
        path=Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_hld1a_phs5_publish_core_stamp_bias_m003_20260510_005528"),
        mode_subdir="offline_groundtruth_projection_modes_phs_check",
        evaluation_source="global",
    ),
    RunSpec(
        label="shortgen04_futureclamp_global",
        role="clean holdout protection rerun, HLD1a-comparable source",
        kind="positive",
        path=Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_futureclamp_20260510_231317"),
        mode_subdir="offline_groundtruth_global_projection_modes",
        evaluation_source="global",
    ),
    RunSpec(
        label="shortgen04_futureclamp_local",
        role="same futureclamp run, local-source conflict diagnostic",
        kind="diagnostic",
        path=Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen04_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_futureclamp_20260510_231317"),
        mode_subdir="offline_groundtruth_projection_modes",
        evaluation_source="local",
    ),
    RunSpec(
        label="shortgen11_repeat2",
        role="clean negative/generalization warning",
        kind="negative",
        path=Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_repeat2_cleancheck_20260510_155253"),
        mode_subdir="offline_groundtruth_projection_modes",
        evaluation_source="local",
    ),
    RunSpec(
        label="shortgen11_coretrace",
        role="timestamp-confounded enriched state update trace diagnostic",
        kind="diagnostic",
        path=Path("/home/yang/kf_gins_ws/artifacts/manual/manual_shortgen11_high_clearance_ladder_sitl_home_headless_compare_core8_pairlogger_phs5_coretrace_20260510_223106"),
        mode_subdir="offline_groundtruth_projection_modes",
        evaluation_source="local",
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


def rmse(values: Iterable[float]) -> float:
    vals = [value for value in values if finite(value)]
    return math.sqrt(sum(value * value for value in vals) / len(vals)) if vals else math.nan


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


def nearest(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_dt_sec: float,
) -> dict[str, object] | None:
    if not rows or not finite(t):
        return None
    idx = bisect.bisect_left(times, t)
    candidates: list[tuple[float, dict[str, object]]] = []
    for pos in (idx - 1, idx):
        if 0 <= pos < len(rows):
            candidates.append((abs(times[pos] - t), rows[pos]))
    if not candidates:
        return None
    dt, row = min(candidates, key=lambda item: item[0])
    return row if dt <= max_dt_sec else None


def latest_before(
    rows: list[dict[str, object]],
    times: list[float],
    t: float,
    max_age_sec: float,
) -> tuple[dict[str, object] | None, float]:
    if not rows or not finite(t):
        return None, math.nan
    idx = bisect.bisect_right(times, t) - 1
    if idx < 0:
        return None, math.nan
    age = t - times[idx]
    if 0.0 <= age <= max_age_sec:
        return rows[idx], age
    return None, age


def mode_dir(spec: RunSpec) -> Path:
    if spec.mode_subdir is not None:
        return spec.path / spec.mode_subdir
    direct = spec.path / "offline_groundtruth_projection_modes"
    if direct.exists():
        return direct
    return spec.path / "offline_groundtruth_projection_modes_phs_check"


def dot_along(vec_e: float, vec_n: float, unit_e: float, unit_n: float) -> float:
    if not all(finite(value) for value in (vec_e, vec_n, unit_e, unit_n)):
        return math.nan
    return vec_e * unit_e + vec_n * unit_n


def cross_track(vec_e: float, vec_n: float, unit_e: float, unit_n: float) -> float:
    if not all(finite(value) for value in (vec_e, vec_n, unit_e, unit_n)):
        return math.nan
    return vec_e * unit_n - vec_n * unit_e


def horizontal_norm(vec_e: float, vec_n: float) -> float:
    if not all(finite(value) for value in (vec_e, vec_n)):
        return math.nan
    return math.hypot(vec_e, vec_n)


def latest_gnss_rows(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    rows = [
        row
        for row in read_csv(run_dir / "gnss_update_debug.csv")
        if to_float(row.get("gnss_position_applied"), 0.0) > 0.5
    ]
    rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    return rows, [to_float(row.get("ros_time_sec")) for row in rows]


def latest_state_update_rows(run_dir: Path) -> tuple[list[dict[str, object]], list[float]]:
    rows = [
        row
        for row in read_csv(run_dir / "state_update_debug.csv")
        if row.get("event_type") == "gnss_position" and to_float(row.get("applied"), 0.0) > 0.5
    ]
    rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    return rows, [to_float(row.get("ros_time_sec")) for row in rows]


def load_rows(spec: RunSpec) -> list[dict[str, object]]:
    modes = mode_dir(spec)
    raw_rows = read_csv(modes / "raw_wgs84_enu" / "groundtruth_joined.csv")
    px4_rows: list[dict[str, object]] = read_csv(modes / "px4_sphere_anisotropic" / "groundtruth_joined.csv")
    px4_rows.sort(key=lambda row: to_float(row.get("pair_ros_time_sec")))
    px4_times = [to_float(row.get("pair_ros_time_sec")) for row in px4_rows]

    state_rows: list[dict[str, object]] = read_csv(spec.path / "state_publish_debug.csv")
    state_rows.sort(key=lambda row: to_float(row.get("ros_time_sec")))
    state_times = [to_float(row.get("ros_time_sec")) for row in state_rows]

    gnss_rows, gnss_times = latest_gnss_rows(spec.path)
    update_rows, update_times = latest_state_update_rows(spec.path)

    out: list[dict[str, object]] = []
    for raw in raw_rows:
        t_arm = to_float(raw.get("time_since_arm_sec"))
        if not (40.0 <= t_arm < 180.0):
            continue
        if to_float(raw.get("mavros_armed"), 0.0) < 0.5:
            continue

        pair_t = to_float(raw.get("pair_ros_time_sec"))
        px4 = nearest(px4_rows, px4_times, pair_t, 0.03)
        state = nearest(state_rows, state_times, pair_t, 0.08)
        if px4 is None or state is None:
            continue
        gnss, gnss_age = latest_before(gnss_rows, gnss_times, pair_t, 0.7)
        update, update_age = latest_before(update_rows, update_times, pair_t, 0.7)

        core_offset = to_float(state.get("publish_core_to_ros_offset_sec"), 0.0)
        state_ros = to_float(state.get("ros_time_sec"))
        core_ros = to_float(state.get("last_core_time_sec")) + core_offset
        gnss_source_age = state_ros - to_float(state.get("last_gnss_source_time_sec"))
        stamp_lag = -to_float(state.get("publish_stamp_selected_minus_now_sec"))

        raw_x = to_float(raw.get("iekf_normalized_aligned_x_m"))
        raw_y = to_float(raw.get("iekf_normalized_aligned_y_m"))
        px4_x = to_float(px4.get("iekf_normalized_aligned_x_m"))
        px4_y = to_float(px4.get("iekf_normalized_aligned_y_m"))
        gt_x = to_float(raw.get("gt_x_m"))
        gt_y = to_float(raw.get("gt_y_m"))

        core_vel_e = to_float(state.get("core_velocity_vE_mps"), 0.0)
        core_vel_n = to_float(state.get("core_velocity_vN_mps"), 0.0)
        core_speed = math.hypot(core_vel_e, core_vel_n)
        if core_speed > 0.25:
            unit_e = core_vel_e / core_speed
            unit_n = core_vel_n / core_speed
        else:
            unit_e = math.nan
            unit_n = math.nan

        core_minus_gnss_e = to_float(state.get("core_minus_gnss_e_m"))
        core_minus_gnss_n = to_float(state.get("core_minus_gnss_n_m"))
        residual_e = to_float(gnss.get("gnss_position_residual_e_m")) if gnss else math.nan
        residual_n = to_float(gnss.get("gnss_position_residual_n_m")) if gnss else math.nan
        dx_e = to_float(update.get("dx_pos_e_m")) if update else math.nan
        dx_n = to_float(update.get("dx_pos_n_m")) if update else math.nan

        residual_h = horizontal_norm(residual_e, residual_n)
        dx_h = horizontal_norm(dx_e, dx_n)
        dx_over_residual = dx_h / residual_h if finite(dx_h) and finite(residual_h) and residual_h > 1e-6 else math.nan

        raw_err_e = raw_x - gt_x
        raw_err_n = raw_y - gt_y
        projection_delta_e = px4_x - raw_x
        projection_delta_n = px4_y - raw_y

        out.append(
            {
                "run": spec.label,
                "role": spec.role,
                "kind": spec.kind,
                "evaluation_source": spec.evaluation_source,
                "projection_mode_dir": modes.name,
                "pair_ros_time_sec": pair_t,
                "time_since_arm_sec": t_arm,
                "mavros_mode": raw.get("mavros_mode", ""),
                "turning_now": to_float(raw.get("turning_now"), 0.0),
                "post_turn_context": to_float(raw.get("post_turn_context"), 0.0),
                "armed_cruise_context": to_float(raw.get("armed_cruise_context"), 0.0),
                "speed_mps": to_float(raw.get("horizontal_speed_mps")),
                "core_speed_mps": core_speed,
                "gyro_deg_s": to_float(raw.get("gyro_deg_s")),
                "raw_iekf_x_m": raw_x,
                "raw_iekf_y_m": raw_y,
                "px4_iekf_x_m": px4_x,
                "px4_iekf_y_m": px4_y,
                "gt_x_m": gt_x,
                "gt_y_m": gt_y,
                "raw_iekf_error_m": to_float(raw.get("iekf_error_xy_m")),
                "px4_iekf_error_m": to_float(px4.get("iekf_error_xy_m")),
                "ekf2_error_m": to_float(raw.get("ekf2_error_xy_m")),
                "projection_benefit_m": to_float(raw.get("iekf_error_xy_m")) - to_float(px4.get("iekf_error_xy_m")),
                "stamp_lag_sec": stamp_lag,
                "core_age_sec": state_ros - core_ros,
                "gnss_source_age_sec": gnss_source_age,
                "state_core_gnss_diff_h_m": to_float(state.get("core_gnss_diff_h_m")),
                "core_velocity_e_mps": core_vel_e,
                "core_velocity_n_mps": core_vel_n,
                "core_minus_gnss_e_m": core_minus_gnss_e,
                "core_minus_gnss_n_m": core_minus_gnss_n,
                "core_gnss_along_velocity_m": dot_along(core_minus_gnss_e, core_minus_gnss_n, unit_e, unit_n),
                "core_gnss_cross_velocity_m": cross_track(core_minus_gnss_e, core_minus_gnss_n, unit_e, unit_n),
                "latest_gnss_update_age_sec": gnss_age,
                "latest_gnss_residual_h_m": residual_h,
                "latest_gnss_residual_along_velocity_m": dot_along(residual_e, residual_n, unit_e, unit_n),
                "latest_gnss_residual_cross_velocity_m": cross_track(residual_e, residual_n, unit_e, unit_n),
                "latest_state_update_age_sec": update_age,
                "latest_dx_pos_h_m": dx_h,
                "latest_dx_pos_along_velocity_m": dot_along(dx_e, dx_n, unit_e, unit_n),
                "latest_dx_pos_cross_velocity_m": cross_track(dx_e, dx_n, unit_e, unit_n),
                "latest_dx_over_residual_h": dx_over_residual,
                "raw_error_along_velocity_m": dot_along(raw_err_e, raw_err_n, unit_e, unit_n),
                "raw_error_cross_velocity_m": cross_track(raw_err_e, raw_err_n, unit_e, unit_n),
                "projection_delta_along_velocity_m": dot_along(projection_delta_e, projection_delta_n, unit_e, unit_n),
                "projection_delta_cross_velocity_m": cross_track(projection_delta_e, projection_delta_n, unit_e, unit_n),
                "publish_projection_action": state.get("publish_projection_action", ""),
                "publish_projection_active": to_float(state.get("publish_projection_active")),
                "segment_timing_gate_projection_enabled": to_float(
                    state.get("segment_timing_gate_projection_enabled")
                ),
                "segment_timing_gate_projection_active": to_float(
                    state.get("segment_timing_gate_projection_active")
                ),
                "segment_timing_gate_phase_allowed": to_float(state.get("segment_timing_gate_phase_allowed")),
                "segment_timing_gate_current_active": to_float(state.get("segment_timing_gate_current_active")),
                "segment_timing_gate_current_lag_mean_sec": to_float(
                    state.get("segment_timing_gate_current_lag_mean_sec")
                ),
                "segment_timing_gate_current_lag_latest_sec": to_float(
                    state.get("segment_timing_gate_current_lag_latest_sec")
                ),
                "segment_timing_gate_lag_threshold_sec": to_float(state.get("segment_timing_gate_lag_threshold_sec")),
            }
        )
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
        if self.op == "abs_le":
            return abs(value) <= self.threshold
        raise ValueError(self.op)

    def label(self) -> str:
        op_label = {"ge": "ge", "le": "le", "abs_le": "absle"}[self.op]
        return f"{self.field}_{op_label}_{self.threshold:.3f}"


@dataclass(frozen=True)
class Candidate:
    name: str
    alpha: float
    conditions: tuple[Condition, ...]

    def active(self, row: dict[str, object]) -> bool:
        return all(condition.matches(row) for condition in self.conditions)


def projection_blend_xy(row: dict[str, object], active: bool, alpha: float) -> tuple[float, float]:
    raw_x = to_float(row.get("raw_iekf_x_m"))
    raw_y = to_float(row.get("raw_iekf_y_m"))
    if not active:
        return raw_x, raw_y
    return (
        raw_x + alpha * (to_float(row.get("px4_iekf_x_m")) - raw_x),
        raw_y + alpha * (to_float(row.get("px4_iekf_y_m")) - raw_y),
    )


def candidate_error(row: dict[str, object], candidate: Candidate) -> tuple[float, bool]:
    active = candidate.active(row)
    x, y = projection_blend_xy(row, active, candidate.alpha)
    return math.hypot(x - to_float(row.get("gt_x_m")), y - to_float(row.get("gt_y_m"))), active


def in_window(row: dict[str, object], start: float, end: float) -> bool:
    t = to_float(row.get("time_since_arm_sec"))
    return start <= t < end


def build_window_index(rows: list[dict[str, object]]) -> dict[tuple[str, str], list[dict[str, object]]]:
    indexed: dict[tuple[str, str], list[dict[str, object]]] = {}
    for run in RUNS:
        run_rows = [row for row in rows if row.get("run") == run.label]
        for window_label, start, end in WINDOWS:
            indexed[(run.label, window_label)] = [row for row in run_rows if in_window(row, start, end)]
    return indexed


def runs_by_kind(kind: str) -> list[RunSpec]:
    return [run for run in RUNS if run.kind == kind]


def summarize_rows(
    subset: Sequence[dict[str, object]],
    candidate: Candidate | None,
) -> dict[str, object]:
    if candidate is None:
        cf_errors = [to_float(row.get("raw_iekf_error_m")) for row in subset]
        active_frac = 0.0
    else:
        evaluated = [candidate_error(row, candidate) for row in subset]
        cf_errors = [item[0] for item in evaluated]
        active_frac = mean(1.0 if item[1] else 0.0 for item in evaluated)
    raw_errors = [to_float(row.get("raw_iekf_error_m")) for row in subset]
    px4_errors = [to_float(row.get("px4_iekf_error_m")) for row in subset]
    ekf2_errors = [to_float(row.get("ekf2_error_m")) for row in subset]
    return {
        "rows": len(subset),
        "active_frac": active_frac,
        "raw_iekf_rmse_m": rmse(raw_errors),
        "global_px4_rmse_m": rmse(px4_errors),
        "candidate_rmse_m": rmse(cf_errors),
        "ekf2_rmse_m": rmse(ekf2_errors),
        "candidate_minus_raw_m": rmse(cf_errors) - rmse(raw_errors),
        "candidate_minus_ekf2_m": rmse(cf_errors) - rmse(ekf2_errors),
    }


def summarize_subset(
    indexed: dict[tuple[str, str], list[dict[str, object]]],
    candidate: Candidate | None,
    run_label: str,
    window_label: str,
) -> dict[str, object]:
    row = summarize_rows(indexed[(run_label, window_label)], candidate)
    row["run"] = run_label
    row["window"] = window_label
    return row


def feature_summary(
    indexed: dict[tuple[str, str], list[dict[str, object]]],
) -> list[dict[str, object]]:
    features = [
        "stamp_lag_sec",
        "core_age_sec",
        "gnss_source_age_sec",
        "state_core_gnss_diff_h_m",
        "core_gnss_along_velocity_m",
        "core_gnss_cross_velocity_m",
        "latest_gnss_residual_h_m",
        "latest_gnss_residual_along_velocity_m",
        "latest_dx_pos_h_m",
        "latest_dx_pos_along_velocity_m",
        "latest_dx_over_residual_h",
        "raw_error_along_velocity_m",
        "projection_delta_along_velocity_m",
        "projection_benefit_m",
        "core_speed_mps",
    ]
    out: list[dict[str, object]] = []
    for run in RUNS:
        for window_label, _start, _end in WINDOWS:
            subset = indexed[(run.label, window_label)]
            row: dict[str, object] = {
                "run": run.label,
                "window": window_label,
                "rows": len(subset),
                "raw_iekf_rmse_m": rmse(to_float(item.get("raw_iekf_error_m")) for item in subset),
                "global_px4_rmse_m": rmse(to_float(item.get("px4_iekf_error_m")) for item in subset),
                "ekf2_rmse_m": rmse(to_float(item.get("ekf2_error_m")) for item in subset),
            }
            for feature in features:
                row[f"{feature}_mean"] = mean(to_float(item.get(feature)) for item in subset)
            out.append(row)
    return out


def lag_band(row: dict[str, object]) -> str:
    lag = to_float(row.get("stamp_lag_sec"))
    if not finite(lag):
        return "lag_nan"
    if lag < 0.02:
        return "lag_lt_0.020"
    if lag < 0.035:
        return "lag_0.020_0.035"
    return "lag_ge_0.035"


def lag_band_summary(
    indexed: dict[tuple[str, str], list[dict[str, object]]],
) -> list[dict[str, object]]:
    bands = ["lag_lt_0.020", "lag_0.020_0.035", "lag_ge_0.035"]
    out: list[dict[str, object]] = []
    for run in RUNS:
        for window_label, _start, _end in WINDOWS:
            subset = indexed[(run.label, window_label)]
            for band in bands:
                band_rows = [row for row in subset if lag_band(row) == band]
                raw = rmse(to_float(row.get("raw_iekf_error_m")) for row in band_rows)
                px4 = rmse(to_float(row.get("px4_iekf_error_m")) for row in band_rows)
                ekf2 = rmse(to_float(row.get("ekf2_error_m")) for row in band_rows)
                out.append(
                    {
                        "run": run.label,
                        "window": window_label,
                        "band": band,
                        "rows": len(band_rows),
                        "window_frac": len(band_rows) / len(subset) if subset else math.nan,
                        "raw_iekf_rmse_m": raw,
                        "global_px4_rmse_m": px4,
                        "ekf2_rmse_m": ekf2,
                        "px4_minus_raw_rmse_m": px4 - raw,
                        "px4_minus_ekf2_rmse_m": px4 - ekf2,
                        "projection_benefit_m_mean": mean(to_float(row.get("projection_benefit_m")) for row in band_rows),
                        "stamp_lag_sec_mean": mean(to_float(row.get("stamp_lag_sec")) for row in band_rows),
                        "core_gnss_along_velocity_m_mean": mean(
                            to_float(row.get("core_gnss_along_velocity_m")) for row in band_rows
                        ),
                        "latest_gnss_residual_along_velocity_m_mean": mean(
                            to_float(row.get("latest_gnss_residual_along_velocity_m")) for row in band_rows
                        ),
                        "latest_dx_over_residual_h_mean": mean(
                            to_float(row.get("latest_dx_over_residual_h")) for row in band_rows
                        ),
                    }
                )
    return out


def selector_name(conditions: Sequence[Condition]) -> str:
    if not conditions:
        return "always"
    parts = []
    aliases = {
        "stamp_lag_sec": "lag",
        "core_gnss_along_velocity_m": "cg_along",
        "state_core_gnss_diff_h_m": "cg_h",
        "latest_gnss_residual_along_velocity_m": "res_along",
        "latest_dx_over_residual_h": "dx_over_resid",
        "core_speed_mps": "core_speed",
    }
    for condition in conditions:
        field = aliases.get(condition.field, condition.field)
        if condition.op == "abs_le":
            parts.append(f"abs_{field}_le_{condition.threshold:.3f}")
        else:
            parts.append(f"{field}_{condition.op}_{condition.threshold:.3f}")
    return "_".join(parts)


def make_candidate(alpha: float, conditions: Sequence[Condition]) -> Candidate:
    selector = selector_name(conditions)
    return Candidate(
        name=f"proj_alpha_{alpha:.1f}_{selector}",
        alpha=alpha,
        conditions=tuple(conditions),
    )


def build_condition_groups() -> dict[str, list[Condition]]:
    return {
        "lag": [Condition("stamp_lag_sec", "ge", value) for value in (0.02, 0.03, 0.035, 0.04, 0.045)],
        "cg_along_le": [
            Condition("core_gnss_along_velocity_m", "le", value)
            for value in (-0.5, 0.0, 0.3, 0.5, 0.7, 0.9)
        ],
        "cg_along_ge": [
            Condition("core_gnss_along_velocity_m", "ge", value)
            for value in (0.3, 0.5, 0.7, 0.9)
        ],
        "cg_h_le": [
            Condition("state_core_gnss_diff_h_m", "le", value)
            for value in (0.5, 0.8, 1.1, 1.4)
        ],
        "cg_h_ge": [
            Condition("state_core_gnss_diff_h_m", "ge", value)
            for value in (0.8, 1.1, 1.4)
        ],
        "abs_res_along": [
            Condition("latest_gnss_residual_along_velocity_m", "abs_le", value)
            for value in (0.5, 1.0, 1.5, 2.0)
        ],
        "dx_over": [
            Condition("latest_dx_over_residual_h", "le", value)
            for value in (0.03, 0.05, 0.08, 0.12)
        ],
        "speed": [Condition("core_speed_mps", "ge", value) for value in (1.0, 2.0, 3.0)],
    }


def build_selectors() -> list[tuple[Condition, ...]]:
    groups = build_condition_groups()
    selectors: list[tuple[Condition, ...]] = [tuple()]

    for conditions in groups.values():
        selectors.extend((condition,) for condition in conditions)

    geometry_groups = [
        *groups["cg_along_le"],
        *groups["cg_along_ge"],
        *groups["cg_h_le"],
        *groups["cg_h_ge"],
        *groups["abs_res_along"],
        *groups["dx_over"],
        *groups["speed"],
    ]
    for lag in groups["lag"]:
        for geom in geometry_groups:
            selectors.append((lag, geom))

    primary_geometry = [
        *groups["cg_along_le"],
        *groups["cg_along_ge"],
        *groups["cg_h_le"],
        *groups["cg_h_ge"],
    ]
    guards = [*groups["abs_res_along"], *groups["dx_over"], *groups["speed"]]
    for lag in groups["lag"]:
        for geom in primary_geometry:
            for guard in guards:
                selectors.append((lag, geom, guard))

    unique: dict[str, tuple[Condition, ...]] = {}
    for selector in selectors:
        unique[selector_name(selector)] = selector
    return list(unique.values())


def build_candidates() -> list[Candidate]:
    candidates: list[Candidate] = []
    selectors = build_selectors()
    for alpha in (1.0, 0.8, 0.6, 0.4, 0.3):
        for selector in selectors:
            candidates.append(make_candidate(alpha, selector))
    return candidates


def candidate_summary(
    indexed: dict[tuple[str, str], list[dict[str, object]]],
    candidate: Candidate,
) -> dict[str, object]:
    by_run_window: dict[tuple[str, str], dict[str, object]] = {}
    positives = [run.label for run in runs_by_kind("positive")]
    negatives = [run.label for run in runs_by_kind("negative")]
    needed_windows = [(label, "main_40_180") for label in positives]
    for label in negatives:
        needed_windows.extend(
            [
                (label, "main_40_180"),
                (label, "60_100"),
                (label, "140_160"),
                (label, "160_180"),
            ]
        )
    for run_label, window_label in needed_windows:
        by_run_window[(run_label, window_label)] = summarize_subset(indexed, candidate, run_label, window_label)

    positive_regs = [
        to_float(by_run_window[(label, "main_40_180")]["candidate_minus_raw_m"])
        for label in positives
    ]
    positive_deltas = [
        to_float(by_run_window[(label, "main_40_180")]["candidate_minus_ekf2_m"])
        for label in positives
    ]
    negative_main = [by_run_window[(label, "main_40_180")] for label in negatives]
    negative_140 = [by_run_window[(label, "140_160")] for label in negatives]
    negative_60100 = [by_run_window[(label, "60_100")] for label in negatives]
    negative_160 = [by_run_window[(label, "160_180")] for label in negatives]

    neg_main_gaps = [to_float(row["candidate_minus_ekf2_m"]) for row in negative_main]
    neg_140_gaps = [to_float(row["candidate_minus_ekf2_m"]) for row in negative_140]
    neg_140_improves = [
        to_float(row["raw_iekf_rmse_m"]) - to_float(row["candidate_rmse_m"])
        for row in negative_140
    ]
    neg_main_improves = [
        to_float(row["raw_iekf_rmse_m"]) - to_float(row["candidate_rmse_m"])
        for row in negative_main
    ]

    protected = max(positive_regs) <= 0.02 and max(positive_deltas) < 0.02
    repaired_2cm = (
        protected
        and max(neg_main_gaps) <= 0.02
        and max(neg_140_gaps) <= 0.02
    )
    repaired_beats = (
        protected
        and max(neg_main_gaps) <= 0.0
        and max(neg_140_gaps) <= 0.0
    )
    return {
        "candidate": candidate.name,
        "alpha": candidate.alpha,
        "selector": selector_name(candidate.conditions),
        "condition_count": len(candidate.conditions),
        "negative_main_raw_rmse_max_m": max(to_float(row["raw_iekf_rmse_m"]) for row in negative_main),
        "negative_main_candidate_rmse_max_m": max(to_float(row["candidate_rmse_m"]) for row in negative_main),
        "negative_main_ekf2_rmse_max_m": max(to_float(row["ekf2_rmse_m"]) for row in negative_main),
        "negative_main_improve_mean_m": mean(neg_main_improves),
        "negative_main_candidate_minus_ekf2_max_m": max(neg_main_gaps),
        "negative_main_active_frac_mean": mean(to_float(row["active_frac"]) for row in negative_main),
        "negative_140_160_raw_rmse_max_m": max(to_float(row["raw_iekf_rmse_m"]) for row in negative_140),
        "negative_140_160_candidate_rmse_max_m": max(to_float(row["candidate_rmse_m"]) for row in negative_140),
        "negative_140_160_ekf2_rmse_max_m": max(to_float(row["ekf2_rmse_m"]) for row in negative_140),
        "negative_140_160_improve_mean_m": mean(neg_140_improves),
        "negative_140_160_candidate_minus_ekf2_max_m": max(neg_140_gaps),
        "negative_140_160_active_frac_mean": mean(to_float(row["active_frac"]) for row in negative_140),
        "negative_60_100_candidate_rmse_max_m": max(to_float(row["candidate_rmse_m"]) for row in negative_60100),
        "negative_60_100_ekf2_rmse_max_m": max(to_float(row["ekf2_rmse_m"]) for row in negative_60100),
        "negative_160_180_candidate_rmse_max_m": max(to_float(row["candidate_rmse_m"]) for row in negative_160),
        "negative_160_180_ekf2_rmse_max_m": max(to_float(row["ekf2_rmse_m"]) for row in negative_160),
        "positive_max_main_regress_m": max(positive_regs),
        "positive_mean_main_regress_m": mean(positive_regs),
        "positive_max_candidate_minus_ekf2_m": max(positive_deltas),
        "positive_all_protected": int(protected),
        "strict_repair_2cm": int(repaired_2cm),
        "strict_repair_beats": int(repaired_beats),
    }


def window_metrics(
    indexed: dict[tuple[str, str], list[dict[str, object]]],
    candidate: Candidate,
) -> list[dict[str, object]]:
    out: list[dict[str, object]] = []
    for run in RUNS:
        for window_label, _start, _end in WINDOWS:
            row = summarize_subset(indexed, candidate, run.label, window_label)
            row["candidate"] = candidate.name
            row["alpha"] = candidate.alpha
            row["selector"] = selector_name(candidate.conditions)
            out.append(row)
    return out


def choose_top(summary_rows: list[dict[str, object]], protected: bool, limit: int = 8) -> list[dict[str, object]]:
    rows = [
        row for row in summary_rows
        if (not protected or to_float(row.get("positive_all_protected"), 0.0) > 0.5)
    ]
    rows.sort(
        key=lambda row: (
            to_float(row.get("negative_140_160_improve_mean_m")),
            to_float(row.get("negative_main_improve_mean_m")),
        ),
        reverse=True,
    )
    return rows[:limit]


def choose_by_local_gap(summary_rows: list[dict[str, object]], protected: bool, limit: int = 8) -> list[dict[str, object]]:
    rows = [
        row for row in summary_rows
        if (not protected or to_float(row.get("positive_all_protected"), 0.0) > 0.5)
    ]
    rows.sort(
        key=lambda row: (
            to_float(row.get("negative_140_160_candidate_minus_ekf2_max_m")),
            to_float(row.get("positive_max_main_regress_m")),
            to_float(row.get("negative_main_candidate_minus_ekf2_max_m")),
        )
    )
    return rows[:limit]


def find_candidate(candidates: Sequence[Candidate], name: str) -> Candidate:
    for candidate in candidates:
        if candidate.name == name:
            return candidate
    raise KeyError(name)


def table_summary_rows(rows: list[dict[str, object]]) -> list[list[object]]:
    return [
        [
            row["candidate"],
            fmt(row["alpha"], 1),
            fmt(row["negative_main_candidate_rmse_max_m"]),
            fmt(row["negative_main_candidate_minus_ekf2_max_m"]),
            fmt(row["negative_140_160_candidate_rmse_max_m"]),
            fmt(row["negative_140_160_candidate_minus_ekf2_max_m"]),
            fmt(row["positive_max_main_regress_m"]),
            row["positive_all_protected"],
        ]
        for row in rows
    ]


def write_report(
    out_dir: Path,
    summary_rows: list[dict[str, object]],
    selected_metrics: list[dict[str, object]],
    features: list[dict[str, object]],
    lag_bands: list[dict[str, object]],
) -> None:
    protected_top = choose_top(summary_rows, protected=True)
    protected_gap_top = choose_by_local_gap(summary_rows, protected=True)
    relaxed_gap_top = choose_by_local_gap(summary_rows, protected=False)
    strict_repairs = [
        row for row in summary_rows
        if to_float(row.get("strict_repair_2cm"), 0.0) > 0.5
    ]
    strict_repairs.sort(
        key=lambda row: (
            to_float(row.get("negative_140_160_candidate_minus_ekf2_max_m")),
            to_float(row.get("positive_max_main_regress_m")),
        )
    )
    global_px4 = next(row for row in summary_rows if row["candidate"] == "proj_alpha_1.0_always")
    strict_best = protected_top[0] if protected_top else None
    local_gap_best = protected_gap_top[0] if protected_gap_top else None
    relaxed_best = next(
        (row for row in relaxed_gap_top if row["candidate"] != "proj_alpha_1.0_always"),
        None,
    )

    key_rows: list[dict[str, object]] = []
    seen: set[str] = set()
    for row in (strict_best, local_gap_best, relaxed_best, global_px4):
        if row is None or str(row["candidate"]) in seen:
            continue
        seen.add(str(row["candidate"]))
        key_rows.append(row)

    display_metrics = [
        row for row in selected_metrics
        if (
            (
                row["run"] in {run.label for run in runs_by_kind("negative")}
                and row["window"] in {"main_40_180", "60_100", "140_160", "160_180"}
            )
            or (
                row["run"] not in {run.label for run in runs_by_kind("negative")}
                and row["window"] == "main_40_180"
            )
        )
    ]
    selected_table = [
        [
            row["candidate"],
            row["run"],
            row["window"],
            fmt(row["active_frac"], 3),
            fmt(row["raw_iekf_rmse_m"]),
            fmt(row["candidate_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["candidate_minus_raw_m"]),
            fmt(row["candidate_minus_ekf2_m"]),
        ]
        for row in display_metrics
    ]

    feature_display = [
        row
        for row in features
        if (
            (
                row["run"] in {run.label for run in runs_by_kind("negative")}
                and row["window"] in {"60_100", "140_160", "160_180"}
            )
            or (
                row["run"] not in {run.label for run in runs_by_kind("negative")}
                and row["window"] == "main_40_180"
            )
        )
    ]
    feature_table = [
        [
            row["run"],
            row["window"],
            fmt(row["raw_iekf_rmse_m"]),
            fmt(row["global_px4_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["stamp_lag_sec_mean"]),
            fmt(row["core_gnss_along_velocity_m_mean"]),
            fmt(row["latest_gnss_residual_along_velocity_m_mean"]),
            fmt(row["latest_dx_over_residual_h_mean"]),
            fmt(row["projection_benefit_m_mean"]),
        ]
        for row in feature_display
    ]
    lag_conflict_display = [
        row
        for row in lag_bands
        if (
            (
                row["run"] in {run.label for run in runs_by_kind("negative")}
                and row["window"] in {"60_100", "140_160"}
            )
            or (
                row["run"] not in {run.label for run in runs_by_kind("negative")}
                and row["window"] == "main_40_180"
                and row["band"] in {"lag_0.020_0.035", "lag_ge_0.035"}
            )
        )
    ]
    lag_conflict_table = [
        [
            row["run"],
            row["window"],
            row["band"],
            fmt(row["window_frac"], 3),
            fmt(row["raw_iekf_rmse_m"]),
            fmt(row["global_px4_rmse_m"]),
            fmt(row["ekf2_rmse_m"]),
            fmt(row["px4_minus_raw_rmse_m"]),
            fmt(row["core_gnss_along_velocity_m_mean"]),
            fmt(row["projection_benefit_m_mean"]),
        ]
        for row in lag_conflict_display
    ]

    negative_labels = {run.label for run in runs_by_kind("negative")}
    positive_labels = {run.label for run in runs_by_kind("positive")}
    negative_low_lag = min(
        (
            row
            for row in lag_bands
            if row["run"] in negative_labels and row["window"] == "140_160" and row["band"] == "lag_lt_0.020"
        ),
        key=lambda row: to_float(row.get("px4_minus_raw_rmse_m"), math.inf),
        default={},
    )
    negative_mid_lag = min(
        (
            row
            for row in lag_bands
            if row["run"] in negative_labels and row["window"] == "140_160" and row["band"] == "lag_0.020_0.035"
        ),
        key=lambda row: to_float(row.get("px4_minus_raw_rmse_m"), math.inf),
        default={},
    )
    positive_mid_lag = max(
        (
            row
            for row in lag_bands
            if row["run"] in positive_labels and row["window"] == "main_40_180" and row["band"] == "lag_0.020_0.035"
        ),
        key=lambda row: to_float(row.get("px4_minus_raw_rmse_m"), -math.inf),
        default={},
    )

    lines = [
        "# PHS5 measurement-geometry selector counterfactual on repeat2 and futureclamp",
        "",
        "Date: 2026-05-10",
        "",
        "Pure offline analysis using existing PHS5 logs only. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator code change was run.",
        "",
        "## Scope",
        "",
        "- Protection runs: shortgen01/02/03 PHS5 development/control, shortgen04 HLD1a, and corrected shortgen04 futureclamp global-source replay.",
        "- Negative run: shortgen11 repeat2.",
        "- Diagnostic-only runs: corrected shortgen04 futureclamp local-source replay, and timestamp-confounded shortgen11 coretrace enriched-trace replay. They expose source/timing conflicts but do not define protection or repair acceptance.",
        "- Joined sources: raw WGS84 groundtruth scoring, PX4-sphere projection scoring, state publish debug, accepted GNSS update debug, and accepted state update debug.",
        "- Evaluation source follows each run spec. HLD1a and futureclamp protection use global-source projection outputs where that is the comparable clean-positive evidence; shortgen01/02/03 and shortgen11 remain local-source outputs.",
        "- Selector inputs tested here are online-observable timing/geometry/update fields only. Groundtruth error and offline projection benefit are used only for scoring and diagnostic readout.",
        "- Actions tested: selector-controlled PX4-sphere projection with blend alpha in [1.0, 0.3].",
        "",
        "## Acceptance",
        "",
        "- Protect positive `main_40_180` windows: max regression <= 0.02 m and candidate still not worse than EKF2 by >= 0.02 m.",
        "- Repair the accepted negative `140_160` window while preserving its `main_40_180`; the strict repair column uses a 0.02 m EKF2 tolerance. Timestamp-confounded diagnostic runs are displayed but excluded from acceptance.",
        "",
        "## Key Candidates",
        "",
        markdown_table(
            [
                "candidate",
                "alpha",
                "neg main worst RMSE",
                "neg main worst vs EKF2",
                "neg 140-160 worst RMSE",
                "neg 140-160 worst vs EKF2",
                "positive max regress",
                "protected",
            ],
            table_summary_rows(key_rows),
        ),
        "",
        "## Top Strict By Local Improvement",
        "",
        markdown_table(
            [
                "candidate",
                "alpha",
                "neg main worst RMSE",
                "neg main worst vs EKF2",
                "neg 140-160 worst RMSE",
                "neg 140-160 worst vs EKF2",
                "positive max regress",
                "protected",
            ],
            table_summary_rows(protected_top[:8]),
        ),
        "",
        "## Top Strict By Local EKF2 Gap",
        "",
        markdown_table(
            [
                "candidate",
                "alpha",
                "neg main worst RMSE",
                "neg main worst vs EKF2",
                "neg 140-160 worst RMSE",
                "neg 140-160 worst vs EKF2",
                "positive max regress",
                "protected",
            ],
            table_summary_rows(protected_gap_top[:8]),
        ),
        "",
        "## Window Feature Readout",
        "",
        markdown_table(
            [
                "run",
                "window",
                "raw",
                "PX4 projection",
                "EKF2",
                "lag mean",
                "core-GNSS along mean",
                "residual along mean",
                "dx/resid mean",
                "PX4 benefit mean",
            ],
            feature_table,
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
                "raw IEKF",
                "candidate",
                "EKF2",
                "candidate-raw",
                "candidate-EKF2",
            ],
            selected_table,
        ),
        "",
        "## Lag Band Conflict",
        "",
        markdown_table(
            [
                "run",
                "window",
                "band",
                "frac",
                "raw",
                "PX4 projection",
                "EKF2",
                "PX4-raw",
                "core-GNSS along",
                "benefit mean",
            ],
            lag_conflict_table,
        ),
        "",
        "## Readout",
        "",
    ]
    if strict_repairs:
        best_repair = strict_repairs[0]
        lines.append(
            f"- Found {len(strict_repairs)} strict repair candidates under the 0.02 m EKF2 tolerance. Best worst-gap candidate is `{best_repair['candidate']}` with negative `140_160` worst RMSE `{fmt(best_repair['negative_140_160_candidate_rmse_max_m'])}` m and worst EKF2 gap `{fmt(best_repair['negative_140_160_candidate_minus_ekf2_max_m'])}` m."
        )
    else:
        lines.append(
            "- No tested online-observable measurement-geometry selector simultaneously protects shortgen01/02/03/04 and repairs accepted shortgen11 repeat2 `140_160` to the 0.02 m EKF2 tolerance."
        )
    if strict_best is not None:
        lines.append(
            f"- Best strict local-improvement candidate is `{strict_best['candidate']}`: negative `140_160` worst RMSE improves from `{fmt(strict_best['negative_140_160_raw_rmse_max_m'])}` to `{fmt(strict_best['negative_140_160_candidate_rmse_max_m'])}` m, but the worst EKF2 gap remains `{fmt(strict_best['negative_140_160_candidate_minus_ekf2_max_m'])}` m."
        )
    lines.extend(
        [
            f"- Global PX4-sphere projection remains diagnostically important: across negative `140_160` windows its worst RMSE is `{fmt(global_px4['negative_140_160_candidate_rmse_max_m'])}` m, but its positive-route max main-window regression is `{fmt(global_px4['positive_max_main_regress_m'])}` m.",
            f"- The lag-band conflict is explicit: negative `140_160` can benefit from projection in `lag_lt_0.020` (`{negative_low_lag.get('run', 'nan')}` PX4-raw `{fmt(negative_low_lag.get('px4_minus_raw_rmse_m'))}` m) and `lag_0.020_0.035` (`{negative_mid_lag.get('run', 'nan')}` PX4-raw `{fmt(negative_mid_lag.get('px4_minus_raw_rmse_m'))}` m), while a positive `main_40_180` mid-lag band can be harmed (`{positive_mid_lag.get('run', 'nan')}` PX4-raw `{fmt(positive_mid_lag.get('px4_minus_raw_rmse_m'))}` m).",
            "- The feature readout should therefore be treated as an explanation filter, not yet as a deployable selector: timing/geometry can explain part of the failure, but the present fields still do not isolate the harmful projection cases from the helpful shortgen11 cases.",
            "",
            "Generated files:",
            f"- `{out_dir / 'geometry_row_metrics.csv'}`",
            f"- `{out_dir / 'window_feature_summary.csv'}`",
            f"- `{out_dir / 'lag_band_projection_conflict.csv'}`",
            f"- `{out_dir / 'candidate_summary.csv'}`",
            f"- `{out_dir / 'selected_window_metrics.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    out_dir = Path(args.out_dir)
    rows: list[dict[str, object]] = []
    for spec in RUNS:
        rows.extend(load_rows(spec))
    write_csv(out_dir / "geometry_row_metrics.csv", rows)

    indexed = build_window_index(rows)
    features = feature_summary(indexed)
    write_csv(out_dir / "window_feature_summary.csv", features)
    lag_bands = lag_band_summary(indexed)
    write_csv(out_dir / "lag_band_projection_conflict.csv", lag_bands)

    candidates = build_candidates()
    summary_rows = [candidate_summary(indexed, candidate) for candidate in candidates]
    write_csv(out_dir / "candidate_summary.csv", summary_rows)

    protected_top = choose_top(summary_rows, protected=True)
    protected_gap_top = choose_by_local_gap(summary_rows, protected=True)
    relaxed_gap_top = choose_by_local_gap(summary_rows, protected=False)
    selected_names = {
        "proj_alpha_1.0_always",
        *(str(row["candidate"]) for row in protected_top[:3]),
        *(str(row["candidate"]) for row in protected_gap_top[:3]),
    }
    relaxed_best = next(
        (row for row in relaxed_gap_top if row["candidate"] != "proj_alpha_1.0_always"),
        None,
    )
    if relaxed_best is not None:
        selected_names.add(str(relaxed_best["candidate"]))

    selected_candidates = [candidate for candidate in candidates if candidate.name in selected_names]
    metrics: list[dict[str, object]] = []
    for candidate in selected_candidates:
        metrics.extend(window_metrics(indexed, candidate))
    write_csv(out_dir / "selected_window_metrics.csv", metrics)
    write_report(out_dir, summary_rows, metrics, features, lag_bands)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
