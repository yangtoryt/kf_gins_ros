#!/usr/bin/env python3
"""Offline PHS5 online-safe error-pressure proxy atlas.

The previous rich atlas showed that PX4-sphere projection helps when its delta
points opposite the true raw IEKF error.  True raw error uses groundtruth, so it
cannot be an online selector.  This script tests online-safe proxies for that
error direction: accumulated GNSS residuals, accumulated state update dx,
core-minus-GNSS mean/trend, and simple combinations of those vectors.
"""

from __future__ import annotations

import argparse
import csv
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, Sequence


BASE = Path("/home/yang/kf_gins_ws/artifacts/manual/short_route_generalization_2026-05-07")
DEFAULT_INPUT = BASE / "phs5_selector_replay_after_lag_failure_2026-05-10" / "geometry_row_metrics.csv"
DEFAULT_OUT = BASE / "phs5_error_proxy_atlas_2026-05-10"
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


def norm(east: float, north: float) -> float:
    return math.hypot(east, north) if finite(east) and finite(north) else math.nan


def unit(east: float, north: float) -> tuple[float, float]:
    length = norm(east, north)
    if not finite(length) or length < 1e-9:
        return math.nan, math.nan
    return east / length, north / length


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


def cosine_between(a_e: float, a_n: float, b_e: float, b_n: float) -> float:
    a_len = norm(a_e, a_n)
    b_len = norm(b_e, b_n)
    if not finite(a_len) or not finite(b_len) or a_len < 1e-9 or b_len < 1e-9:
        return math.nan
    return (a_e * b_e + a_n * b_n) / (a_len * b_len)


def vec_from_along_cross(along: float, cross: float, unit_e: float, unit_n: float) -> tuple[float, float]:
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
    rows.sort(key=lambda row: (str(row.get("run")), to_float(row.get("time_since_arm_sec"))))
    return rows


def segment_index(t: float, segment_sec: float) -> int:
    return int(math.floor((t - 40.0) / segment_sec))


def vec_mean(items: Iterable[tuple[float, float]]) -> tuple[float, float]:
    values = [(east, north) for east, north in items if finite(east) and finite(north)]
    if not values:
        return math.nan, math.nan
    return sum(east for east, _north in values) / len(values), sum(north for _east, north in values) / len(values)


def add_vecs(*vectors: tuple[float, float]) -> tuple[float, float]:
    vals = [(east, north) for east, north in vectors if finite(east) and finite(north)]
    if not vals:
        return math.nan, math.nan
    return sum(east for east, _north in vals), sum(north for _east, north in vals)


def proxy_feature_fields(prefix: str) -> list[str]:
    return [
        f"{prefix}_h_m",
        f"{prefix}_vs_projection_abs_deg",
        f"neg_{prefix}_vs_projection_abs_deg",
        f"{prefix}_dot_projection_cos",
        f"neg_{prefix}_dot_projection_cos",
        f"{prefix}_vs_oracle_raw_error_abs_deg",
    ]


def add_proxy_features(
    segment: dict[str, object],
    prefix: str,
    proxy_e: float,
    proxy_n: float,
    proj_e: float,
    proj_n: float,
    raw_err_e: float,
    raw_err_n: float,
) -> None:
    proxy_heading = angle_deg(proxy_e, proxy_n)
    neg_heading = angle_deg(-proxy_e, -proxy_n)
    proj_heading = angle_deg(proj_e, proj_n)
    raw_err_heading = angle_deg(raw_err_e, raw_err_n)
    segment[f"{prefix}_h_m"] = norm(proxy_e, proxy_n)
    segment[f"{prefix}_vs_projection_abs_deg"] = abs_angle_diff_deg(proxy_heading, proj_heading)
    segment[f"neg_{prefix}_vs_projection_abs_deg"] = abs_angle_diff_deg(neg_heading, proj_heading)
    segment[f"{prefix}_dot_projection_cos"] = cosine_between(proxy_e, proxy_n, proj_e, proj_n)
    segment[f"neg_{prefix}_dot_projection_cos"] = cosine_between(-proxy_e, -proxy_n, proj_e, proj_n)
    segment[f"{prefix}_vs_oracle_raw_error_abs_deg"] = abs_angle_diff_deg(proxy_heading, raw_err_heading)


def build_segments(rows: Sequence[dict[str, object]], segment_sec: float) -> list[dict[str, object]]:
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
        n = len(segment_rows)
        raw_sse = sum(to_float(row.get("raw_iekf_error_m")) ** 2 for row in segment_rows)
        px4_sse = sum(to_float(row.get("px4_iekf_error_m")) ** 2 for row in segment_rows)
        ekf2_sse = sum(to_float(row.get("ekf2_error_m")) ** 2 for row in segment_rows)
        raw_rmse = math.sqrt(raw_sse / n)
        px4_rmse = math.sqrt(px4_sse / n)
        ekf2_rmse = math.sqrt(ekf2_sse / n)

        proj_e = mean(to_float(row.get("px4_iekf_x_m")) - to_float(row.get("raw_iekf_x_m")) for row in segment_rows)
        proj_n = mean(to_float(row.get("px4_iekf_y_m")) - to_float(row.get("raw_iekf_y_m")) for row in segment_rows)
        raw_err_e = mean(to_float(row.get("raw_iekf_x_m")) - to_float(row.get("gt_x_m")) for row in segment_rows)
        raw_err_n = mean(to_float(row.get("raw_iekf_y_m")) - to_float(row.get("gt_y_m")) for row in segment_rows)
        core_gnss_e = mean(to_float(row.get("core_minus_gnss_e_m")) for row in segment_rows)
        core_gnss_n = mean(to_float(row.get("core_minus_gnss_n_m")) for row in segment_rows)
        core_trend_e = to_float(segment_rows[-1].get("core_minus_gnss_e_m")) - to_float(segment_rows[0].get("core_minus_gnss_e_m"))
        core_trend_n = to_float(segment_rows[-1].get("core_minus_gnss_n_m")) - to_float(segment_rows[0].get("core_minus_gnss_n_m"))

        residual_vectors: list[tuple[float, float]] = []
        dx_vectors: list[tuple[float, float]] = []
        for row in segment_rows:
            unit_e, unit_n = unit(to_float(row.get("core_velocity_e_mps")), to_float(row.get("core_velocity_n_mps")))
            residual_vectors.append(
                vec_from_along_cross(
                    to_float(row.get("latest_gnss_residual_along_velocity_m")),
                    to_float(row.get("latest_gnss_residual_cross_velocity_m")),
                    unit_e,
                    unit_n,
                )
            )
            dx_vectors.append(
                vec_from_along_cross(
                    to_float(row.get("latest_dx_pos_along_velocity_m")),
                    to_float(row.get("latest_dx_pos_cross_velocity_m")),
                    unit_e,
                    unit_n,
                )
            )
        residual_e, residual_n = vec_mean(residual_vectors)
        dx_e, dx_n = vec_mean(dx_vectors)
        pressure_e, pressure_n = add_vecs((residual_e, residual_n), (dx_e, dx_n))
        correction_pressure_e, correction_pressure_n = add_vecs((dx_e, dx_n), (-core_trend_e, -core_trend_n))
        residual_core_e, residual_core_n = add_vecs((residual_e, residual_n), (core_gnss_e, core_gnss_n))

        segment: dict[str, object] = {
            "run": run,
            "group": "positive" if run in POSITIVE_RUNS else "target",
            "segment_sec": segment_sec,
            "segment_index": index,
            "start_sec": start,
            "end_sec": start + segment_sec,
            "rows": n,
            "raw_rmse_m": raw_rmse,
            "px4_rmse_m": px4_rmse,
            "ekf2_rmse_m": ekf2_rmse,
            "px4_minus_raw_rmse_m": px4_rmse - raw_rmse,
            "px4_minus_ekf2_rmse_m": px4_rmse - ekf2_rmse,
            "raw_minus_ekf2_rmse_m": raw_rmse - ekf2_rmse,
            "projection_benefit_rmse_m": raw_rmse - px4_rmse,
            "projection_class": "help" if raw_rmse - px4_rmse > 0.02 else "harm" if px4_rmse - raw_rmse > 0.02 else "neutral",
            "projection_delta_h_m": norm(proj_e, proj_n),
            "oracle_raw_error_h_m": norm(raw_err_e, raw_err_n),
            "oracle_raw_error_vs_projection_abs_deg": abs_angle_diff_deg(angle_deg(raw_err_e, raw_err_n), angle_deg(proj_e, proj_n)),
            "projection_dot_oracle_raw_error_cos": cosine_between(proj_e, proj_n, raw_err_e, raw_err_n),
            "stamp_lag_sec_mean": mean(to_float(row.get("stamp_lag_sec")) for row in segment_rows),
            "state_core_gnss_diff_h_m_mean": mean(to_float(row.get("state_core_gnss_diff_h_m")) for row in segment_rows),
            "core_gnss_along_velocity_m_mean": mean(to_float(row.get("core_gnss_along_velocity_m")) for row in segment_rows),
            "projection_delta_cross_velocity_m_mean": mean(to_float(row.get("projection_delta_cross_velocity_m")) for row in segment_rows),
            "turning_now_mean": mean(to_float(row.get("turning_now")) for row in segment_rows),
            "armed_cruise_context_mean": mean(to_float(row.get("armed_cruise_context")) for row in segment_rows),
        }
        proxies = {
            "residual_mean": (residual_e, residual_n),
            "dx_mean": (dx_e, dx_n),
            "core_gnss_mean": (core_gnss_e, core_gnss_n),
            "core_gnss_trend": (core_trend_e, core_trend_n),
            "pressure_residual_dx": (pressure_e, pressure_n),
            "pressure_dx_neg_coretrend": (correction_pressure_e, correction_pressure_n),
            "pressure_residual_core": (residual_core_e, residual_core_n),
        }
        for prefix, (proxy_e, proxy_n) in proxies.items():
            add_proxy_features(segment, prefix, proxy_e, proxy_n, proj_e, proj_n, raw_err_e, raw_err_n)
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
    candidates: list[Candidate] = []
    proxy_prefixes = [
        "residual_mean",
        "dx_mean",
        "core_gnss_mean",
        "core_gnss_trend",
        "pressure_residual_dx",
        "pressure_dx_neg_coretrend",
        "pressure_residual_core",
    ]
    angle_thresholds = (30.0, 60.0, 90.0, 120.0, 150.0)
    mag_thresholds = (0.02, 0.05, 0.10, 0.20, 0.40)
    guards = [
        Condition("core_gnss_along_velocity_m_mean", "le", 0.85),
        Condition("core_gnss_along_velocity_m_mean", "le", 0.95),
        Condition("core_gnss_along_velocity_m_mean", "le", 1.05),
        Condition("state_core_gnss_diff_h_m_mean", "le", 0.85),
        Condition("state_core_gnss_diff_h_m_mean", "le", 1.05),
        Condition("state_core_gnss_diff_h_m_mean", "le", 1.25),
        Condition("projection_delta_cross_velocity_m_mean", "le", -0.06),
        Condition("projection_delta_cross_velocity_m_mean", "le", -0.02),
        Condition("projection_delta_cross_velocity_m_mean", "le", 0.02),
        Condition("armed_cruise_context_mean", "ge", 0.50),
        Condition("turning_now_mean", "le", 0.25),
    ]
    for prefix in proxy_prefixes:
        angle_conditions: list[Condition] = []
        for threshold in angle_thresholds:
            angle_conditions.append(Condition(f"{prefix}_vs_projection_abs_deg", "le", threshold))
            angle_conditions.append(Condition(f"{prefix}_vs_projection_abs_deg", "ge", threshold))
            angle_conditions.append(Condition(f"neg_{prefix}_vs_projection_abs_deg", "le", threshold))
            angle_conditions.append(Condition(f"neg_{prefix}_vs_projection_abs_deg", "ge", threshold))
        mag_conditions = [Condition(f"{prefix}_h_m", "ge", value) for value in mag_thresholds]
        for condition in angle_conditions + mag_conditions:
            candidates.append(make_candidate(prefix, (condition,)))
        for angle in angle_conditions:
            for guard in guards:
                candidates.append(make_candidate(f"{prefix}_x_guard", (angle, guard)))
        for angle in angle_conditions:
            for mag in mag_conditions:
                candidates.append(make_candidate(f"{prefix}_x_mag", (angle, mag)))
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
    fields: list[str] = [
        "oracle_raw_error_vs_projection_abs_deg",
        "projection_dot_oracle_raw_error_cos",
        "stamp_lag_sec_mean",
        "state_core_gnss_diff_h_m_mean",
        "core_gnss_along_velocity_m_mean",
        "projection_delta_cross_velocity_m_mean",
    ]
    for prefix in (
        "residual_mean",
        "dx_mean",
        "core_gnss_mean",
        "core_gnss_trend",
        "pressure_residual_dx",
        "pressure_dx_neg_coretrend",
        "pressure_residual_core",
    ):
        fields.extend(proxy_feature_fields(prefix))
    out: list[dict[str, object]] = []
    for field in fields:
        help_values = [to_float(row.get(field)) for row in target_help]
        harm_values = [to_float(row.get(field)) for row in positive_harm]
        help_mean = mean(help_values)
        harm_mean = mean(harm_values)
        help_var = mean((value - help_mean) ** 2 for value in help_values if finite(value) and finite(help_mean))
        harm_var = mean((value - harm_mean) ** 2 for value in harm_values if finite(value) and finite(harm_mean))
        pooled = math.sqrt(help_var + harm_var) if finite(help_var) and finite(harm_var) else math.nan
        out.append(
            {
                "feature": field,
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


def write_report(out_dir: Path, separation_rows: list[dict[str, object]], candidate_rows: list[dict[str, object]]) -> None:
    strict = [row for row in candidate_rows if to_float(row.get("strict_pass"), 0.0) > 0.5]
    strict.sort(key=sort_key)
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
        for row in separation_rows[:16]
    ]
    lines = [
        "# PHS5 online-safe error-pressure proxy atlas",
        "",
        "Date: 2026-05-10",
        "",
        "Offline-only analysis using existing row-level PHS5 metrics. No flight, rebuild, PX4, Gazebo, MAVROS, QGC, RViz2, PlotJuggler, or estimator process was started.",
        "",
        "## Feature Separation",
        "",
        markdown_table(
            ["feature", "target help n", "positive harm n", "target help mean", "positive harm mean", "diff", "score"],
            sep_table,
        ),
        "",
        "## Top Protected Proxy Selectors",
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
            table_candidate(protected_top[:14]),
        ),
        "",
        "## Top Relaxed Proxy Selectors",
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
            table_candidate(relaxed_top[:14]),
        ),
        "",
        "## Readout",
        "",
    ]
    if strict:
        best = strict[0]
        lines.append(
            f"- A strict-pass online-safe proxy exists: `{best['candidate']}` at `{fmt(best['segment_sec'], 1)}` s."
        )
    else:
        lines.append(
            "- No tested online-safe error-pressure proxy passes both positive protection and all shortgen11 140-180 target repair."
        )
    lines.extend(
        [
            "- If the oracle raw-error direction remains much stronger than these proxies, the projection selector needs either a new logged state/error proxy or a different mechanism than projection switching.",
            "",
            "Generated files:",
            f"- `{out_dir / 'error_proxy_segment_observables.csv'}`",
            f"- `{out_dir / 'error_proxy_feature_separation.csv'}`",
            f"- `{out_dir / 'error_proxy_candidate_summary.csv'}`",
        ]
    )
    (out_dir / "report.md").write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", default=str(DEFAULT_INPUT))
    parser.add_argument("--out-dir", default=str(DEFAULT_OUT))
    args = parser.parse_args()

    rows = load_rows(Path(args.input))
    out_dir = Path(args.out_dir)
    candidates = build_candidates()
    all_segments: list[dict[str, object]] = []
    candidate_rows: list[dict[str, object]] = []
    for segment_sec in SEGMENT_LENGTHS:
        segments = build_segments(rows, segment_sec)
        all_segments.extend(segments)
        for candidate in candidates:
            row = score_candidate(segments, candidate)
            row["segment_sec"] = segment_sec
            candidate_rows.append(row)
    separation_rows = feature_separation([row for row in all_segments if to_float(row.get("segment_sec")) == 2.0])
    write_csv(out_dir / "error_proxy_segment_observables.csv", all_segments)
    write_csv(out_dir / "error_proxy_feature_separation.csv", separation_rows)
    write_csv(out_dir / "error_proxy_candidate_summary.csv", candidate_rows)
    write_report(out_dir, separation_rows, candidate_rows)
    print(f"wrote: {out_dir / 'report.md'}")


if __name__ == "__main__":
    main()
